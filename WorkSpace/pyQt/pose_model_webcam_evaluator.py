"""POCO Pose GRU camera performance evaluator.

This standalone PyQt tool deliberately reuses the production Pose pipeline:

    Raspberry Pi Camera Module or USB webcam frame
      -> MediaPipe Pose (create_pose_detector)
      -> build_pose_features
      -> evaluation-only 5 second CalibrationService baseline
      -> PoseGruService (same scaler/TFLite/window/stride)

The production ``saved_model/baseline.pkl`` is never overwritten.  Every run
creates a separate directory under ``WorkSpace/data/pose_evaluation``.

Usage
-----
    cd /home/willtek/POCO/WorkSpace/pyQt
    python3 pose_model_webcam_evaluator.py

Optional camera index:
    python3 pose_model_webcam_evaluator.py --camera 1
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any

# Keep native library logs consistent with the production Pose process.
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ.setdefault("GLOG_minloglevel", "2")

import cv2
import mediapipe as mp
import numpy as np
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


PYQT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = PYQT_DIR.parent
if str(PYQT_DIR) not in sys.path:
    sys.path.insert(0, str(PYQT_DIR))
if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))

import modules.config as config
from processes.pose_process_profile import (
    CONTROL_LANDMARK_MIN_VISIBILITY,
    build_pose_features,
    create_pose_detector,
    pose_control_landmark_quality,
)
from services.calibration_service import CalibrationService
from services.pose_gru_service import PoseGruService


LABEL_INDICES = sorted(config.POSTURE_LABELS)
LABELS = [config.POSTURE_LABELS[index] for index in LABEL_INDICES]
LABEL_TO_INDEX = {label: index for index, label in config.POSTURE_LABELS.items()}

LABEL_DISPLAY = {
    "Optimal": "Optimal (바른 자세)",
    "Asymmetric": "Asymmetric (비대칭)",
    "Forward Head": "Forward Head (거북목)",
    "Chin Propping": "Chin Propping (턱 괴기)",
}


CAMERA_FPS = 30


class OpenCVEvaluationCameraSource:
    """OpenCV USB/V4L2 fallback with the production camera interface."""

    def __init__(self, camera_index=0, width=640, height=480, fps=CAMERA_FPS):
        self.camera_index = int(camera_index)
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.capture = None

    def open(self):
        capture = None
        if sys.platform.startswith("linux"):
            capture = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if capture is None or not capture.isOpened():
            if capture is not None:
                capture.release()
            capture = cv2.VideoCapture(self.camera_index)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                f"OpenCV 카메라를 열 수 없습니다: index={self.camera_index}"
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture = capture

    def start(self):
        return None

    def read(self):
        if self.capture is None:
            return False, None
        success, frame = self.capture.read()
        if not success or frame is None:
            return False, None
        return True, cv2.flip(frame, 1)

    def release(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None


class PiCamera2EvaluationSource:
    """Raspberry Pi Camera Module source matching POCO's production setup."""

    def __init__(self, width=640, height=480, fps=CAMERA_FPS):
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.camera = None

    def open(self):
        from picamera2 import Picamera2
        from libcamera import Transform

        camera = Picamera2()
        # Keep the partially opened object reachable so fallback cleanup can
        # close it even if configuration fails midway.
        self.camera = camera
        camera_config = camera.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)},
            raw=None,
            buffer_count=6,
            transform=Transform(hflip=True),
            controls={"FrameRate": self.fps},
        )
        camera.configure(camera_config)

    def start(self):
        if self.camera is None:
            raise RuntimeError("Picamera2가 구성되지 않았습니다.")
        self.camera.start()

    def read(self):
        if self.camera is None:
            return False, None
        frame = self.camera.capture_array()
        if frame is None:
            return False, None
        return True, frame

    def release(self):
        camera = self.camera
        self.camera = None
        if camera is None:
            return
        try:
            camera.stop()
        except Exception:
            pass
        try:
            camera.close()
        except Exception:
            pass

PREDICTION_FIELDS = [
    "timestamp",
    "clip_id",
    "ground_truth_label",
    "predicted_label",
    "predicted_index",
    "confidence",
    "frame_id",
    "clip_elapsed_sec",
    "latency_ms",
    "landmark_detected",
    "control_landmark_valid",
    "landmark_quality",
]

FEATURE_FIELDS = [f"feature_{index + 1}" for index in range(config.POSE_FEATURE_SIZE)]

FRAME_FIELDS = [
    "timestamp",
    "clip_id",
    "ground_truth_label",
    "frame_id",
    "clip_elapsed_sec",
    "landmark_detected",
    "control_landmark_valid",
    "landmark_quality",
    "feature_valid",
    "processing_latency_ms",
    *FEATURE_FIELDS,
]

CLIP_FIELDS = [
    "clip_id",
    "ground_truth_label",
    "started_at",
    "ended_at",
    "duration_sec",
    "frame_count",
    "control_landmark_valid_count",
    "prediction_count",
    "video_path",
]


def _safe_div(numerator: float, denominator: float) -> float:
    if denominator <= 0:
        return 0.0
    return float(numerator) / float(denominator)


def _round_or_none(value: Any, digits: int = 4):
    if value is None:
        return None
    return round(float(value), digits)


def calculate_metrics(
    prediction_rows: list[dict[str, Any]],
    frame_rows: list[dict[str, Any]],
    clips: list[dict[str, Any]],
) -> dict[str, Any]:
    """Calculate report-ready metrics without requiring scikit-learn."""

    confusion = [[0 for _ in LABELS] for _ in LABELS]
    label_position = {label: position for position, label in enumerate(LABELS)}

    valid_prediction_count = 0
    correct_count = 0
    for row in prediction_rows:
        actual = str(row.get("ground_truth_label", ""))
        predicted = str(row.get("predicted_label", ""))
        if actual not in label_position or predicted not in label_position:
            continue
        actual_position = label_position[actual]
        predicted_position = label_position[predicted]
        confusion[actual_position][predicted_position] += 1
        valid_prediction_count += 1
        if actual == predicted:
            correct_count += 1

    per_label: dict[str, dict[str, Any]] = {}
    f1_values: list[float] = []
    recall_values: list[float] = []
    precision_values: list[float] = []

    for position, label in enumerate(LABELS):
        true_positive = confusion[position][position]
        false_negative = sum(confusion[position]) - true_positive
        false_positive = (
            sum(confusion[row][position] for row in range(len(LABELS)))
            - true_positive
        )
        support = true_positive + false_negative
        predicted_count = true_positive + false_positive
        precision = _safe_div(true_positive, predicted_count)
        recall = _safe_div(true_positive, support)
        f1 = _safe_div(2.0 * precision * recall, precision + recall)

        # Standard macro metrics include all configured classes.  A class with
        # no recorded ground-truth sample therefore remains 0 and is clearly
        # exposed through support=0 instead of silently inflating the score.
        precision_values.append(precision)
        recall_values.append(recall)
        f1_values.append(f1)
        per_label[label] = {
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1": round(f1, 4),
            "support": int(support),
            "predicted_count": int(predicted_count),
        }

    prediction_latencies = [
        float(row["latency_ms"])
        for row in prediction_rows
        if row.get("latency_ms") is not None
        and np.isfinite(float(row["latency_ms"]))
    ]
    processing_latencies = [
        float(row["processing_latency_ms"])
        for row in frame_rows
        if row.get("processing_latency_ms") is not None
        and np.isfinite(float(row["processing_latency_ms"]))
    ]

    total_frames = len(frame_rows)
    detected_frames = sum(bool(row.get("landmark_detected")) for row in frame_rows)
    control_valid_frames = sum(
        bool(row.get("control_landmark_valid")) for row in frame_rows
    )
    feature_valid_frames = sum(bool(row.get("feature_valid")) for row in frame_rows)

    clip_counts = {label: 0 for label in LABELS}
    for clip in clips:
        label = str(clip.get("ground_truth_label", ""))
        if label in clip_counts:
            clip_counts[label] += 1
    missing_ground_truth_labels = [
        label
        for label in LABELS
        if per_label.get(label, {}).get("support", 0) <= 0
    ]

    return {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "labels": list(LABELS),
        "clip_count": len(clips),
        "clip_count_by_label": clip_counts,
        "complete_label_set": not missing_ground_truth_labels,
        "missing_ground_truth_labels": missing_ground_truth_labels,
        "prediction_count": valid_prediction_count,
        "accuracy": round(_safe_div(correct_count, valid_prediction_count), 4),
        "macro_precision": round(float(np.mean(precision_values)), 4),
        "macro_recall": round(float(np.mean(recall_values)), 4),
        "macro_f1": round(float(np.mean(f1_values)), 4),
        "per_label": per_label,
        "confusion_matrix": {
            "row_actual_column_predicted": True,
            "labels": list(LABELS),
            "values": confusion,
        },
        "landmark": {
            "total_recorded_frames": total_frames,
            "detected_frames": detected_frames,
            "control_valid_frames": control_valid_frames,
            "feature_valid_frames": feature_valid_frames,
            "detection_rate": round(_safe_div(detected_frames, total_frames), 4),
            "control_valid_rate": round(
                _safe_div(control_valid_frames, total_frames), 4
            ),
            "feature_valid_rate": round(
                _safe_div(feature_valid_frames, total_frames), 4
            ),
            "control_visibility_threshold": CONTROL_LANDMARK_MIN_VISIBILITY,
        },
        "latency_ms": {
            "definition": (
                "frame timestamp immediately before MediaPipe -> "
                "Pose GRU result completed"
            ),
            "prediction_sample_count": len(prediction_latencies),
            "mean": _round_or_none(
                np.mean(prediction_latencies) if prediction_latencies else None, 3
            ),
            "p50": _round_or_none(
                np.percentile(prediction_latencies, 50)
                if prediction_latencies
                else None,
                3,
            ),
            "p95": _round_or_none(
                np.percentile(prediction_latencies, 95)
                if prediction_latencies
                else None,
                3,
            ),
            "max": _round_or_none(
                np.max(prediction_latencies) if prediction_latencies else None, 3
            ),
            "all_frame_processing_p95": _round_or_none(
                np.percentile(processing_latencies, 95)
                if processing_latencies
                else None,
                3,
            ),
        },
    }


def format_metrics(metrics: dict[str, Any]) -> str:
    latency = metrics.get("latency_ms", {})
    landmark = metrics.get("landmark", {})
    per_label = metrics.get("per_label", {})
    lines = [
        "[전체 성능]",
        f"Prediction 표본: {metrics.get('prediction_count', 0)}",
        f"Accuracy: {metrics.get('accuracy', 0.0):.4f}",
        f"Macro Precision: {metrics.get('macro_precision', 0.0):.4f}",
        f"Macro Recall: {metrics.get('macro_recall', 0.0):.4f}",
        f"Macro F1: {metrics.get('macro_f1', 0.0):.4f}",
    ]
    missing = metrics.get("missing_ground_truth_labels", [])
    if missing:
        lines.append(
            "주의: 아직 Ground Truth 표본이 없는 라벨 — " + ", ".join(missing)
        )
    lines.extend(["", "[자세별 성능]"])
    for label in LABELS:
        values = per_label.get(label, {})
        lines.append(
            f"{label:<14} "
            f"P={values.get('precision', 0.0):.4f} "
            f"R={values.get('recall', 0.0):.4f} "
            f"F1={values.get('f1', 0.0):.4f} "
            f"N={values.get('support', 0)}"
        )

    lines.extend(
        [
            "",
            "[Landmark / 지연시간]",
            (
                "제어 Landmark 유효율: "
                f"{float(landmark.get('control_valid_rate', 0.0)) * 100:.2f}% "
                f"({landmark.get('control_valid_frames', 0)}/"
                f"{landmark.get('total_recorded_frames', 0)})"
            ),
            f"판정 latency 평균: {latency.get('mean')} ms",
            f"판정 latency P95: {latency.get('p95')} ms",
            f"전체 frame 처리 P95: {latency.get('all_frame_processing_p95')} ms",
            "",
            "[Confusion Matrix: 행=실제, 열=예측]",
            " " * 15 + " ".join(f"{label[:4]:>6}" for label in LABELS),
        ]
    )
    values = metrics.get("confusion_matrix", {}).get("values", [])
    for label, row in zip(LABELS, values):
        lines.append(f"{label[:13]:<13}  " + " ".join(f"{value:>6}" for value in row))
    return "\n".join(lines)


def _write_csv_atomic(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _write_json_atomic(path: Path, value: dict[str, Any]):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(path)


class PoseEvaluationWorker(QThread):
    frame_ready = pyqtSignal(QImage)
    status_changed = pyqtSignal(str)
    calibration_progress = pyqtSignal(float, int, int)
    calibration_finished = pyqtSignal(bool, str)
    recording_changed = pyqtSignal(str, bool, int)
    live_state_changed = pyqtSignal(dict)
    metrics_changed = pyqtSignal(dict)
    exported = pyqtSignal(str)
    fatal_error = pyqtSignal(str)

    WAITING = "WAITING"
    CALIBRATING = "CALIBRATING"
    RECORDING = "RECORDING"

    def __init__(self, camera_index: int, session_dir: Path, parent=None):
        super().__init__(parent)
        self.camera_index = int(camera_index)
        self.session_dir = Path(session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.baseline_path = self.session_dir / "evaluation_pose_baseline.pkl"

        self.commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.running = False
        self.state = self.WAITING
        self.calibrated = False
        self.current_label: str | None = None
        self.current_clip_id: int | None = None
        self.clip_started_monotonic: float | None = None
        self.clip_started_at: str | None = None
        self.clip_frame_count = 0
        self.clip_landmark_valid_count = 0
        self.clip_prediction_count = 0
        self.current_video_writer = None
        self.current_video_path: Path | None = None
        self.next_clip_id = 1
        self.frame_id = 0

        self.detector = None
        self.camera = None
        self.calibration_service: CalibrationService | None = None
        self.gru_service: PoseGruService | None = None
        self.calibration_missing_count = 0

        self.prediction_rows: list[dict[str, Any]] = []
        self.frame_rows: list[dict[str, Any]] = []
        self.clips: list[dict[str, Any]] = []
        self.last_live_emit = 0.0
        self.last_status_emit = 0.0
        self.latest_prediction = "-"
        self.latest_confidence = 0.0

    # These methods only enqueue commands.  All MediaPipe/TFLite/camera state
    # remains owned by this QThread.
    def request_calibration(self):
        self.commands.put(("CALIBRATE", None))

    def request_recording_start(self, label: str):
        self.commands.put(("START_RECORDING", str(label)))

    def request_recording_stop(self):
        self.commands.put(("STOP_RECORDING", None))

    def request_reset_results(self):
        self.commands.put(("RESET_RESULTS", None))

    def request_export(self):
        self.commands.put(("EXPORT", None))

    def request_shutdown(self):
        self.commands.put(("SHUTDOWN", None))

    def _open_camera(self):
        errors = []
        if sys.platform.startswith("linux"):
            pi_camera = PiCamera2EvaluationSource(
                width=config.FRAME_WIDTH,
                height=config.FRAME_HEIGHT,
                fps=CAMERA_FPS,
            )
            try:
                pi_camera.open()
                self.status_changed.emit("Picamera2 카메라 모듈을 선택했습니다.")
                return pi_camera
            except Exception as error:
                pi_camera.release()
                errors.append(f"Picamera2 실패: {error}")
                self.status_changed.emit(
                    "Picamera2 연결 실패. OpenCV 카메라를 확인합니다."
                )

        opencv_camera = OpenCVEvaluationCameraSource(
            camera_index=self.camera_index,
            width=config.FRAME_WIDTH,
            height=config.FRAME_HEIGHT,
            fps=CAMERA_FPS,
        )
        try:
            opencv_camera.open()
            self.status_changed.emit(
                f"OpenCV 카메라 index={self.camera_index}를 선택했습니다."
            )
            return opencv_camera
        except Exception as error:
            opencv_camera.release()
            errors.append(f"OpenCV 실패: {error}")

        raise RuntimeError(
            "사용 가능한 카메라를 찾지 못했습니다.\n" + "\n".join(errors)
        )

    def _prepare_pose(self):
        self.detector = create_pose_detector()
        dummy_rgb = np.zeros(
            (config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), dtype=np.uint8
        )
        for _ in range(2):
            self.detector.process(dummy_rgb)

    def _prepare_gru(self):
        if self.gru_service is None:
            service = PoseGruService()
            # Evaluation baseline is isolated from the production baseline.
            service.baseline_path = self.baseline_path
            service.baseline = service.load_baseline(required=True)
            service.load()
            self.gru_service = service
        else:
            self.gru_service.baseline_path = self.baseline_path
            self.gru_service.baseline = self.gru_service.load_baseline(required=True)

    def _handle_commands(self) -> bool:
        should_continue = True
        while True:
            try:
                command, payload = self.commands.get_nowait()
            except queue.Empty:
                break

            if command == "CALIBRATE":
                self._start_calibration()
            elif command == "START_RECORDING":
                self._start_recording(str(payload))
            elif command == "STOP_RECORDING":
                self._stop_recording()
            elif command == "RESET_RESULTS":
                self._reset_results()
            elif command == "EXPORT":
                self._export_results()
            elif command == "SHUTDOWN":
                if self.state == self.RECORDING:
                    self._stop_recording()
                self._export_results()
                should_continue = False
        return should_continue

    def _start_calibration(self):
        if self.state == self.RECORDING:
            self.status_changed.emit("녹화를 먼저 종료해주세요.")
            return
        if self.state == self.CALIBRATING:
            return
        if self.gru_service is not None:
            self.gru_service.stop()
        self.calibration_service = CalibrationService(
            baseline_path=self.baseline_path,
            duration=config.CALIBRATION_TIME,
            expected_fps=30,
            min_sample_ratio=0.6,
        )
        result = self.calibration_service.start()
        self.calibration_missing_count = 0
        self.calibrated = False
        self.state = self.CALIBRATING
        self.status_changed.emit(result.message)
        self.calibration_progress.emit(0.0, 0, 0)

    def _finish_calibration(self, success: bool, message: str):
        if success:
            try:
                self.status_changed.emit("보정 완료. 동일 Pose GRU 모델을 로드합니다...")
                self._prepare_gru()
                self.calibrated = True
                message = (
                    f"{message}\n평가용 baseline: {self.baseline_path}\n"
                    "자세를 먼저 잡은 뒤 해당 라벨의 녹화 시작 버튼을 누르세요."
                )
            except Exception as error:
                success = False
                self.calibrated = False
                message = f"Pose GRU 준비 실패: {error}"
        self.state = self.WAITING
        self.calibration_finished.emit(bool(success), message)
        self.status_changed.emit(message)

    def _start_recording(self, label: str):
        if label not in LABELS:
            self.status_changed.emit(f"지원하지 않는 라벨입니다: {label}")
            return
        if not self.calibrated or self.gru_service is None:
            self.status_changed.emit("먼저 바른 자세 보정을 완료해주세요.")
            return
        if self.state == self.CALIBRATING:
            self.status_changed.emit("보정이 끝날 때까지 기다려주세요.")
            return
        if self.state == self.RECORDING:
            self.status_changed.emit("현재 녹화를 먼저 종료해주세요.")
            return

        self.current_label = label
        self.current_clip_id = self.next_clip_id
        self.next_clip_id += 1
        self.clip_started_monotonic = time.monotonic()
        self.clip_started_at = datetime.now().isoformat(timespec="milliseconds")
        self.clip_frame_count = 0
        self.clip_landmark_valid_count = 0
        self.clip_prediction_count = 0
        self.latest_prediction = "window 준비 중"
        self.latest_confidence = 0.0
        safe_label = label.lower().replace(" ", "_")
        self.current_video_path = (
            self.session_dir
            / f"clip_{self.current_clip_id:02d}_{safe_label}.avi"
        )
        writer = cv2.VideoWriter(
            str(self.current_video_path),
            cv2.VideoWriter_fourcc(*"MJPG"),
            30.0,
            (config.FRAME_WIDTH, config.FRAME_HEIGHT),
        )
        if writer.isOpened():
            self.current_video_writer = writer
        else:
            writer.release()
            self.current_video_writer = None
            self.status_changed.emit(
                "영상 파일을 열지 못했습니다. Feature/예측 평가는 계속합니다."
            )
        self.gru_service.start()
        self.state = self.RECORDING
        self.recording_changed.emit(label, True, self.current_clip_id)
        self.status_changed.emit(
            f"[{LABEL_DISPLAY.get(label, label)}] 녹화 중 — 같은 버튼으로 종료합니다."
        )

    def _stop_recording(self):
        if self.state != self.RECORDING:
            return
        assert self.current_label is not None
        assert self.current_clip_id is not None
        assert self.clip_started_monotonic is not None
        if self.gru_service is not None:
            self.gru_service.stop()
        if self.current_video_writer is not None:
            self.current_video_writer.release()
            self.current_video_writer = None

        ended_at = datetime.now().isoformat(timespec="milliseconds")
        duration = max(0.0, time.monotonic() - self.clip_started_monotonic)
        clip = {
            "clip_id": self.current_clip_id,
            "ground_truth_label": self.current_label,
            "started_at": self.clip_started_at,
            "ended_at": ended_at,
            "duration_sec": round(duration, 3),
            "frame_count": self.clip_frame_count,
            "control_landmark_valid_count": self.clip_landmark_valid_count,
            "prediction_count": self.clip_prediction_count,
            "video_path": (
                str(self.current_video_path)
                if self.current_video_path is not None
                else ""
            ),
        }
        self.clips.append(clip)
        finished_label = self.current_label
        finished_clip_id = self.current_clip_id

        self.state = self.WAITING
        self.current_label = None
        self.current_clip_id = None
        self.clip_started_monotonic = None
        self.clip_started_at = None
        self.current_video_path = None
        self.recording_changed.emit(finished_label, False, finished_clip_id)
        self.status_changed.emit(
            f"[{LABEL_DISPLAY.get(finished_label, finished_label)}] 녹화 종료 — "
            f"frame={clip['frame_count']}, prediction={clip['prediction_count']}"
        )
        if clip["prediction_count"] <= 0:
            self.status_changed.emit(
                f"[{LABEL_DISPLAY.get(finished_label, finished_label)}] 예측 표본이 0개입니다. "
                f"최소 {config.WINDOW_SIZE}개의 유효 feature가 필요하므로 더 길게 다시 녹화하세요."
            )
        self._export_results()

    def _reset_results(self):
        if self.state != self.WAITING:
            self.status_changed.emit("보정/녹화 중에는 결과를 초기화할 수 없습니다.")
            return
        self.prediction_rows.clear()
        self.frame_rows.clear()
        self.clips.clear()
        self.next_clip_id = 1
        metrics = calculate_metrics([], [], [])
        self.metrics_changed.emit(metrics)
        self._export_results()
        self.status_changed.emit("평가 결과를 초기화했습니다. 보정값은 유지됩니다.")

    def _export_results(self):
        try:
            metrics = calculate_metrics(
                self.prediction_rows, self.frame_rows, self.clips
            )
            metrics["session_dir"] = str(self.session_dir)
            metrics["camera_index"] = self.camera_index
            metrics["model_path"] = str(
                self.gru_service.model_path if self.gru_service else config.MODEL_PATH_GRU
            )
            metrics["scaler_path"] = str(
                self.gru_service.scaler_path
                if self.gru_service
                else config.SCALER_PATH_GRU
            )
            metrics["evaluation_baseline_path"] = str(self.baseline_path)
            metrics["window_size"] = config.WINDOW_SIZE
            metrics["stride"] = config.STRIDE
            _write_csv_atomic(
                self.session_dir / "predictions.csv",
                PREDICTION_FIELDS,
                self.prediction_rows,
            )
            _write_csv_atomic(
                self.session_dir / "frame_quality.csv",
                FRAME_FIELDS,
                self.frame_rows,
            )
            _write_csv_atomic(
                self.session_dir / "clips.csv", CLIP_FIELDS, self.clips
            )
            matrix_rows = []
            matrix = metrics["confusion_matrix"]["values"]
            for actual_label, row in zip(LABELS, matrix):
                matrix_rows.append(
                    {
                        "actual_label": actual_label,
                        **{
                            f"predicted_{label}": value
                            for label, value in zip(LABELS, row)
                        },
                    }
                )
            _write_csv_atomic(
                self.session_dir / "confusion_matrix.csv",
                ["actual_label", *[f"predicted_{label}" for label in LABELS]],
                matrix_rows,
            )
            _write_json_atomic(self.session_dir / "summary.json", metrics)
            self.metrics_changed.emit(metrics)
            self.exported.emit(str(self.session_dir))
        except Exception as error:
            self.status_changed.emit(f"평가 결과 저장 실패: {error}")

    def _process_calibration(self, features):
        assert self.calibration_service is not None
        if features is None:
            self.calibration_missing_count += 1
        result = self.calibration_service.update(features)
        elapsed = max(0.0, config.CALIBRATION_TIME - result.remain_time)
        self.calibration_progress.emit(
            elapsed, result.sample_count, self.calibration_missing_count
        )
        if result.is_finished:
            self._finish_calibration(result.success, result.message)

    def _record_frame(
        self,
        timestamp_ns: int,
        features,
        landmark_detected: bool,
        control_landmark_valid: bool,
        landmark_quality: float,
    ):
        assert self.current_label is not None
        assert self.current_clip_id is not None
        assert self.clip_started_monotonic is not None
        assert self.gru_service is not None

        self.clip_frame_count += 1
        if control_landmark_valid:
            self.clip_landmark_valid_count += 1
        elapsed = max(0.0, time.monotonic() - self.clip_started_monotonic)

        gru_result = self.gru_service.update(features)
        processing_latency_ms = (
            time.perf_counter_ns() - timestamp_ns
        ) / 1_000_000.0
        frame_row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "clip_id": self.current_clip_id,
            "ground_truth_label": self.current_label,
            "frame_id": self.frame_id,
            "clip_elapsed_sec": round(elapsed, 4),
            "landmark_detected": int(landmark_detected),
            "control_landmark_valid": int(control_landmark_valid),
            "landmark_quality": round(float(landmark_quality), 6),
            "feature_valid": int(features is not None),
            "processing_latency_ms": round(processing_latency_ms, 4),
        }
        feature_values = None
        if features is not None:
            candidate = np.asarray(features, dtype=np.float32).reshape(-1)
            if (
                candidate.size == config.POSE_FEATURE_SIZE
                and np.all(np.isfinite(candidate))
            ):
                feature_values = candidate
        for index, field_name in enumerate(FEATURE_FIELDS):
            frame_row[field_name] = (
                "" if feature_values is None else round(float(feature_values[index]), 8)
            )
        self.frame_rows.append(frame_row)

        if gru_result is None:
            return

        latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0
        predicted_label = str(gru_result["posture_type"])
        confidence = float(gru_result["confidence"])
        prediction_row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "clip_id": self.current_clip_id,
            "ground_truth_label": self.current_label,
            "predicted_label": predicted_label,
            "predicted_index": int(gru_result["pose_index"]),
            "confidence": round(confidence, 6),
            "frame_id": self.frame_id,
            "clip_elapsed_sec": round(elapsed, 4),
            "latency_ms": round(latency_ms, 4),
            "landmark_detected": int(landmark_detected),
            "control_landmark_valid": int(control_landmark_valid),
            "landmark_quality": round(float(landmark_quality), 6),
        }
        self.prediction_rows.append(prediction_row)
        self.clip_prediction_count += 1
        self.latest_prediction = predicted_label
        self.latest_confidence = confidence

    @staticmethod
    def _to_qimage(frame: np.ndarray) -> QImage:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb.shape
        return QImage(
            rgb.data,
            width,
            height,
            channel * width,
            QImage.Format_RGB888,
        ).copy()

    def _annotate_frame(self, frame, results, control_valid, quality):
        annotated = frame.copy()
        if results is not None and results.pose_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(
                annotated,
                results.pose_landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
            )
        if self.state == self.RECORDING:
            mode_text = f"REC: {self.current_label}"
            color = (0, 0, 255)
        elif self.state == self.CALIBRATING:
            mode_text = "CALIBRATING"
            color = (0, 200, 255)
        else:
            mode_text = "WAITING"
            color = (0, 220, 0)
        cv2.putText(
            annotated,
            mode_text,
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"Landmark {'OK' if control_valid else 'NG'} q={quality:.2f}",
            (8, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 220, 0) if control_valid else (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"Pred: {self.latest_prediction} {self.latest_confidence * 100:.1f}%",
            (8, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return annotated

    def run(self):
        self.running = True
        try:
            self.status_changed.emit("카메라와 MediaPipe Pose를 준비합니다...")
            self.camera = self._open_camera()
            self.camera.start()
            self._prepare_pose()
            self.status_changed.emit(
                "준비 완료. 바른 자세를 잡고 '보정 시작'을 누르세요."
            )

            while self.running:
                if not self._handle_commands():
                    break

                ret, frame = self.camera.read()
                if not ret or frame is None:
                    now = time.monotonic()
                    if now - self.last_status_emit >= 1.0:
                        self.last_status_emit = now
                        self.status_changed.emit("카메라 프레임을 읽지 못했습니다.")
                    self.msleep(5)
                    continue

                if frame.shape[:2] != (config.FRAME_HEIGHT, config.FRAME_WIDTH):
                    frame = cv2.resize(
                        frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT)
                    )

                self.frame_id += 1
                timestamp_ns = time.perf_counter_ns()
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.detector.process(rgb_frame)
                features = build_pose_features(results)
                landmark_detected = bool(
                    results is not None and results.pose_landmarks is not None
                )
                quality, control_valid = pose_control_landmark_quality(results)

                if self.state == self.CALIBRATING:
                    self._process_calibration(features)
                elif self.state == self.RECORDING:
                    self._record_frame(
                        timestamp_ns,
                        features,
                        landmark_detected,
                        control_valid,
                        quality,
                    )
                    if self.current_video_writer is not None:
                        # Raw frame is saved as labeled test evidence.  Writing
                        # happens after the measured inference latency so disk
                        # I/O is not included in that frame's latency value.
                        self.current_video_writer.write(frame)

                annotated = self._annotate_frame(
                    frame, results, control_valid, quality
                )
                self.frame_ready.emit(self._to_qimage(annotated))

                now = time.monotonic()
                if now - self.last_live_emit >= 0.2:
                    self.last_live_emit = now
                    self.live_state_changed.emit(
                        {
                            "mode": self.state,
                            "label": self.current_label,
                            "landmark_detected": landmark_detected,
                            "control_landmark_valid": control_valid,
                            "landmark_quality": float(quality),
                            "prediction": self.latest_prediction,
                            "confidence": self.latest_confidence,
                            "clip_frame_count": self.clip_frame_count,
                            "clip_prediction_count": self.clip_prediction_count,
                        }
                    )

        except Exception as error:
            detail = traceback.format_exc()
            print(detail)
            self.fatal_error.emit(f"{error}\n\n{detail}")
        finally:
            self.running = False
            if self.calibration_service is not None:
                self.calibration_service.cancel()
            if self.gru_service is not None:
                self.gru_service.close()
            if self.current_video_writer is not None:
                self.current_video_writer.release()
                self.current_video_writer = None
            if self.detector is not None:
                self.detector.close()
                self.detector = None
            if self.camera is not None:
                self.camera.release()
                self.camera = None


class PoseEvaluationWindow(QMainWindow):
    def __init__(self, camera_index: int):
        super().__init__()
        session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = (
            WORKSPACE_DIR / "data" / "pose_evaluation" / session_name
        )
        self.worker = PoseEvaluationWorker(camera_index, self.session_dir, self)
        self.recording_label: str | None = None
        self.calibration_ready = False
        self.closing = False
        self.label_buttons: dict[str, QPushButton] = {}

        self.setWindowTitle("POCO Pose GRU 카메라 성능평가")
        self.compact_display = self._fit_to_display()
        self._build_ui()
        self._connect_worker()
        self.worker.start()

    def _fit_to_display(self) -> bool:
        """Fit 800x480/1024x600 Raspberry Pi displays without clipping."""
        screen = QApplication.primaryScreen()
        if screen is None:
            self.resize(1024, 600)
            return True
        available = screen.availableGeometry()
        compact = available.width() <= 1024 or available.height() <= 600
        if compact:
            self.setGeometry(available)
        else:
            self.resize(
                min(1120, available.width()),
                min(720, available.height()),
            )
        return compact

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        margin = 5 if self.compact_display else 10
        outer.setContentsMargins(margin, margin, margin, margin)
        outer.setSpacing(6 if self.compact_display else 10)
        if self.compact_display:
            central.setStyleSheet(
                "QWidget{font-size:11px;}"
                "QGroupBox{font-weight:600;margin-top:7px;}"
                "QGroupBox::title{subcontrol-origin:margin;left:7px;}"
                "QPushButton{min-height:28px;padding:2px 5px;}"
            )

        left_widget = QWidget()
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(5)
        self.camera_label = QLabel("Camera 준비 중")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(240, 180)
        self.camera_label.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding
        )
        self.camera_label.setStyleSheet(
            "background:#111;color:white;border:1px solid #555;"
        )
        left.addWidget(self.camera_label, 1)

        self.live_label = QLabel(
            "Mode WAITING | Landmark -- | Prediction -- | Clip frame 0"
        )
        self.live_label.setWordWrap(True)
        self.live_label.setStyleSheet(
            "background:#f1f5f9;border:1px solid #cbd5e1;"
            f"padding:{4 if self.compact_display else 8}px;font-weight:600;"
        )
        left.addWidget(self.live_label)
        outer.addWidget(left_widget, 3)

        tabs = QTabWidget()
        tabs.setDocumentMode(True)

        controls_scroll = QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        controls_scroll.setFrameShape(QScrollArea.NoFrame)
        controls_widget = QWidget()
        right = QVBoxLayout(controls_widget)
        right.setContentsMargins(5, 5, 5, 5)
        right.setSpacing(5)
        instruction = QLabel(
            "① 바른 자세 5초 보정 → ② 평가 자세를 취한 뒤 라벨 버튼 → "
            "③ 20초 이상 유지 후 같은 버튼으로 종료 → ④ 네 라벨 반복"
        )
        instruction.setWordWrap(True)
        instruction.setStyleSheet(
            f"background:#eff6ff;border:1px solid #93c5fd;"
            f"padding:{5 if self.compact_display else 9}px;"
        )
        right.addWidget(instruction)

        calibration_group = QGroupBox("1. 평가용 바른 자세 보정")
        calibration_layout = QVBoxLayout(calibration_group)
        self.calibration_button = QPushButton("바른 자세 보정 시작 (5초)")
        self.calibration_button.clicked.connect(self._request_calibration)
        self.calibration_progress_bar = QProgressBar()
        self.calibration_progress_bar.setRange(0, config.CALIBRATION_TIME * 10)
        self.calibration_progress_bar.setValue(0)
        self.calibration_detail_label = QLabel("유효 sample 0 / missing 0")
        calibration_layout.addWidget(self.calibration_button)
        calibration_layout.addWidget(self.calibration_progress_bar)
        calibration_layout.addWidget(self.calibration_detail_label)
        right.addWidget(calibration_group)

        recording_group = QGroupBox("2. Ground Truth 라벨별 녹화")
        recording_layout = QGridLayout(recording_group)
        recording_layout.setContentsMargins(6, 10, 6, 6)
        recording_layout.setSpacing(4)
        compact_names = {
            "Optimal": "바른 자세",
            "Asymmetric": "비대칭",
            "Forward Head": "거북목",
            "Chin Propping": "턱 괴기",
        }
        for position, label in enumerate(LABELS):
            display_name = (
                compact_names.get(label, label)
                if self.compact_display
                else LABEL_DISPLAY.get(label, label)
            )
            button = QPushButton(f"{display_name} 녹화 시작")
            button.setProperty("display_name", display_name)
            button.setToolTip(LABEL_DISPLAY.get(label, label))
            button.setEnabled(False)
            button.setMinimumHeight(30 if self.compact_display else 42)
            button.clicked.connect(
                lambda _checked=False, selected=label: self._toggle_recording(
                    selected
                )
            )
            self.label_buttons[label] = button
            recording_layout.addWidget(button, position // 2, position % 2)
        right.addWidget(recording_group)

        action_layout = QHBoxLayout()
        self.reset_button = QPushButton("평가 결과 초기화")
        self.reset_button.clicked.connect(self._reset_results)
        self.export_button = QPushButton("현재 결과 저장")
        self.export_button.clicked.connect(self.worker.request_export)
        action_layout.addWidget(self.reset_button)
        action_layout.addWidget(self.export_button)
        right.addLayout(action_layout)

        self.status_label = QLabel("초기화 중...")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(
            f"background:#fff7ed;border:1px solid #fdba74;"
            f"padding:{4 if self.compact_display else 8}px;font-weight:600;"
        )
        self.output_label = QLabel(f"저장 폴더: {self.session_dir}")
        self.output_label.setWordWrap(True)
        self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        right.addWidget(self.status_label)
        right.addWidget(self.output_label)
        right.addStretch(1)
        controls_scroll.setWidget(controls_widget)
        tabs.addTab(controls_scroll, "시험 조작")

        metrics_widget = QWidget()
        metrics_layout = QVBoxLayout(metrics_widget)
        metrics_layout.setContentsMargins(5, 5, 5, 5)
        self.metrics_text = QPlainTextEdit()
        self.metrics_text.setReadOnly(True)
        self.metrics_text.setPlainText(format_metrics(calculate_metrics([], [], [])))
        metrics_layout.addWidget(self.metrics_text)
        tabs.addTab(metrics_widget, "측정 결과")
        outer.addWidget(tabs, 2)

    def _connect_worker(self):
        self.worker.frame_ready.connect(self._update_frame)
        self.worker.status_changed.connect(self.status_label.setText)
        self.worker.calibration_progress.connect(self._update_calibration_progress)
        self.worker.calibration_finished.connect(self._on_calibration_finished)
        self.worker.recording_changed.connect(self._on_recording_changed)
        self.worker.live_state_changed.connect(self._update_live_state)
        self.worker.metrics_changed.connect(self._update_metrics)
        self.worker.exported.connect(self._on_exported)
        self.worker.fatal_error.connect(self._on_fatal_error)

    def _request_calibration(self):
        if self.recording_label is not None:
            QMessageBox.warning(self, "보정", "현재 녹화를 먼저 종료해주세요.")
            return
        self.calibration_button.setEnabled(False)
        for button in self.label_buttons.values():
            button.setEnabled(False)
        self.calibration_progress_bar.setValue(0)
        self.worker.request_calibration()

    def _toggle_recording(self, label: str):
        if self.recording_label is None:
            self.worker.request_recording_start(label)
            return
        if self.recording_label == label:
            self.worker.request_recording_stop()

    def _reset_results(self):
        if self.recording_label is not None:
            QMessageBox.warning(self, "평가 초기화", "현재 녹화를 먼저 종료해주세요.")
            return
        answer = QMessageBox.question(
            self,
            "평가 결과 초기화",
            "현재 세션의 예측·frame·clip 결과를 모두 초기화할까요?\n"
            "평가용 보정값은 유지됩니다.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.worker.request_reset_results()

    def _update_frame(self, image: QImage):
        if self.closing:
            return
        pixmap = QPixmap.fromImage(image)
        self.camera_label.setPixmap(
            pixmap.scaled(
                self.camera_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def _update_calibration_progress(
        self, elapsed: float, samples: int, missing: int
    ):
        self.calibration_progress_bar.setValue(
            min(config.CALIBRATION_TIME * 10, round(elapsed * 10))
        )
        self.calibration_detail_label.setText(
            f"유효 sample {samples} / missing frame {missing}"
        )

    def _on_calibration_finished(self, success: bool, message: str):
        self.calibration_button.setEnabled(True)
        self.calibration_ready = bool(success)
        if success:
            self.calibration_progress_bar.setValue(config.CALIBRATION_TIME * 10)
            for button in self.label_buttons.values():
                button.setEnabled(True)
            QMessageBox.information(self, "보정 완료", message)
        else:
            for button in self.label_buttons.values():
                button.setEnabled(False)
            QMessageBox.warning(self, "보정 실패", message)

    def _on_recording_changed(self, label: str, active: bool, clip_id: int):
        if active:
            self.recording_label = label
            self.calibration_button.setEnabled(False)
            self.reset_button.setEnabled(False)
            for current_label, button in self.label_buttons.items():
                button.setEnabled(current_label == label)
                if current_label == label:
                    display_name = str(
                        button.property("display_name")
                        or LABEL_DISPLAY.get(label, label)
                    )
                    button.setText(
                        f"● {display_name} 녹화 종료"
                    )
                    button.setStyleSheet(
                        "background:#dc2626;color:white;font-weight:700;"
                    )
        else:
            self.recording_label = None
            self.calibration_button.setEnabled(True)
            self.reset_button.setEnabled(True)
            for current_label, button in self.label_buttons.items():
                button.setEnabled(self.calibration_ready)
                display_name = str(
                    button.property("display_name")
                    or LABEL_DISPLAY.get(current_label, current_label)
                )
                button.setText(
                    f"{display_name} 녹화 시작"
                )
                button.setStyleSheet("")

    def _update_live_state(self, state: dict[str, Any]):
        validity = "OK" if state.get("control_landmark_valid") else "NG"
        self.live_label.setText(
            f"Mode {state.get('mode', '--')} | "
            f"GT {state.get('label') or '-'} | "
            f"Landmark {validity} "
            f"(quality {float(state.get('landmark_quality', 0.0)):.2f}) | "
            f"Prediction {state.get('prediction', '-')} "
            f"{float(state.get('confidence', 0.0)) * 100:.1f}% | "
            f"Clip frame {state.get('clip_frame_count', 0)} / "
            f"prediction {state.get('clip_prediction_count', 0)}"
        )

    def _update_metrics(self, metrics: dict[str, Any]):
        self.metrics_text.setPlainText(format_metrics(metrics))

    def _on_exported(self, directory: str):
        self.output_label.setText(f"저장 완료: {directory}")

    def _on_fatal_error(self, message: str):
        if self.closing:
            return
        QMessageBox.critical(self, "Pose 평가 도구 오류", message)
        self.status_label.setText("치명적 오류로 평가 Worker가 종료되었습니다.")

    def closeEvent(self, event: QCloseEvent):
        self.closing = True
        if self.worker.isRunning():
            self.worker.blockSignals(True)
            self.worker.request_shutdown()
            if not self.worker.wait(8000):
                # Camera read should normally return quickly.  Do not call
                # QThread.terminate(); leave a clear warning instead.
                self.worker.blockSignals(False)
                self.closing = False
                QMessageBox.warning(
                    self,
                    "종료 지연",
                    "카메라 Worker가 아직 종료되지 않았습니다. 잠시 후 다시 닫아주세요.",
                )
                event.ignore()
                return
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(
        description="POCO Pose GRU camera performance evaluator"
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="OpenCV fallback camera index (Picamera2에는 적용되지 않음)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    app = QApplication(sys.argv[:1])
    window = PoseEvaluationWindow(args.camera)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

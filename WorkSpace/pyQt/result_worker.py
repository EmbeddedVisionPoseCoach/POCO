import time
from collections import Counter
from queue import Empty
from types import SimpleNamespace

from PyQt5.QtCore import QThread, pyqtSignal

import modules.config as config

from pathlib import Path
from modules.logger import StudyLogger

ROOT_DIR = Path(__file__).resolve().parent.parent

class VisionResultWorker(QThread):
    status_changed = pyqtSignal(str)
    calibration_finished = pyqtSignal(bool, str)
    result_changed = pyqtSignal(dict)
    ready_changed = pyqtSignal(bool)

    def __init__(self, vision_manager, hardware_controller=None, parent=None):
        super().__init__(parent)

        self.logger = None
        self.log_dir = ROOT_DIR / "data" / "session_log"

        self.vision_manager = vision_manager
        self.hardware_controller = hardware_controller
        self.running = False

        self.pose_ready = False
        self.face_ready = False

        self.pose_calibration_result = None
        self.face_calibration_result = None
        self.pose_calibration_progress = None
        self.face_calibration_progress = None

        self.pose_results = {}
        self.face_results = {}

        self.posture_counter = Counter()
        self.measurement_start_time = None
        self.last_ui_emit_time = 0.0
        self.ui_emit_interval = 0.5

    @property
    def is_ready(self):
        return self.pose_ready and self.face_ready

    def run(self):
        self.running = True

        while self.running:
            handled = False
            handled |= self._drain_pose_queue()
            handled |= self._drain_face_queue()

            if not handled:
                self.msleep(5)

    def _drain_pose_queue(self):
        handled = False

        try:
            while True:
                result = self.vision_manager.pose_result_queue.get_nowait()
                handled = True
                self._handle_pose_result(result)
        except Empty:
            pass

        return handled

    def _drain_face_queue(self):
        handled = False

        try:
            while True:
                result = self.vision_manager.face_result_queue.get_nowait()
                handled = True
                self._handle_face_result(result)
        except Empty:
            pass

        return handled

    def _handle_pose_result(self, result):
        result_type = result.get("type")

        if result_type == "POSE_READY":
            self.pose_ready = True
            print("[ResultWorker] Pose Process 준비 완료")
            self._check_ready()
            return

        if result_type == "POSE_ERROR":
            self.status_changed.emit(f"Pose Process 오류: {result.get('message', '')}")
            return

        if result_type == "POSE_CALIBRATION_PROGRESS":
            self.pose_calibration_progress = result
            self._emit_calibration_progress()
            return

        if result_type == "POSE_CALIBRATION_DONE":
            self.pose_calibration_result = result
            self._check_calibration_done()
            return

        if result_type == "POSE_RESULT":
            frame_id = result["frame_id"]
            self.pose_results[frame_id] = result
            self._try_fuse_result(frame_id)
            self._cleanup_old_results(frame_id)
            return

        if result_type == "POSE_STATS":
            print(
                f"[POSE] FPS={result['fps']:.1f} Drop={result['sequence_drop']} "
                f"Queue={result['queue_latency_ms']:.2f}ms Total={result['total_latency_ms']:.2f}ms"
            )

    def _handle_face_result(self, result):
        result_type = result.get("type")

        if result_type == "FACE_READY":
            self.face_ready = True
            print("[ResultWorker] Face Process 준비 완료")
            self._check_ready()
            return

        if result_type == "FACE_ERROR":
            self.status_changed.emit(f"Face Process 오류: {result.get('message', '')}")
            return

        if result_type == "FACE_CALIBRATION_PROGRESS":
            self.face_calibration_progress = result
            self._emit_calibration_progress()
            return

        if result_type == "FACE_CALIBRATION_DONE":
            self.face_calibration_result = result
            self._check_calibration_done()
            return

        if result_type == "FACE_RESULT":
            frame_id = result["frame_id"]
            self.face_results[frame_id] = result
            self._try_fuse_result(frame_id)
            self._cleanup_old_results(frame_id)
            return

        if result_type == "FACE_STATS":
            print(
                f"[FACE] FPS={result['fps']:.1f} Drop={result['sequence_drop']} "
                f"Queue={result['queue_latency_ms']:.2f}ms Total={result['total_latency_ms']:.2f}ms"
            )

    def _check_ready(self):
        if not self.is_ready:
            return

        self.status_changed.emit("Pose / Face AI Process 준비 완료")
        self.ready_changed.emit(True)

    def reset_calibration(self):
        self.pose_calibration_result = None
        self.face_calibration_result = None
        self.pose_calibration_progress = None
        self.face_calibration_progress = None

    def _emit_calibration_progress(self):
        pose_samples = 0
        face_samples = 0
        pose_remain = None
        face_remain = None

        if self.pose_calibration_progress is not None:
            pose_samples = self.pose_calibration_progress.get("sample_count", 0)
            pose_remain = self.pose_calibration_progress.get("remain_time")

        if self.face_calibration_progress is not None:
            face_samples = self.face_calibration_progress.get("sample_count", 0)
            face_remain = self.face_calibration_progress.get("remain_time")

        remain_values = [value for value in (pose_remain, face_remain) if value is not None]

        if remain_values:
            remain_time = max(remain_values)
            self.status_changed.emit(
                f"초기값 측정 중... {remain_time:.1f}초 / "
                f"Pose {pose_samples}개 / Face {face_samples}개"
            )

    def _check_calibration_done(self):
        if self.pose_calibration_result is None or self.face_calibration_result is None:
            return

        pose = self.pose_calibration_result
        face = self.face_calibration_result

        success = pose.get("success", False) and face.get("success", False)

        message = (
            f"Pose: {pose.get('message', '')}\n"
            f"Pose Sample: {pose.get('sample_count', 0)} / Missing: {pose.get('missing_count', 0)}\n\n"
            f"Face: {face.get('message', '')}\n"
            f"Face Sample: {face.get('sample_count', 0)} / Missing: {face.get('missing_count', 0)}"
        )

        self.vision_manager.stop_analysis()
        self.calibration_finished.emit(success, message)

    def start_measurement_session(self):
        self.posture_counter.clear()
        self.pose_results.clear()
        self.face_results.clear()

        self.measurement_start_time = time.monotonic()
        self.last_ui_emit_time = 0.0

        self.logger = StudyLogger(base_dir=str(self.log_dir))

    def stop_measurement_session(self):
        self.measurement_start_time = None
        self.pose_results.clear()
        self.face_results.clear()
        self.logger = None

    def _try_fuse_result(self, frame_id):
        pose_result = self.pose_results.get(frame_id)
        face_result = self.face_results.get(frame_id)

        if pose_result is None or face_result is None:
            return

        posture_type = pose_result["posture_type"]
        confidence = pose_result["confidence"]
        fatigue_label = face_result["fatigue_label"]
        fatigue_probability = face_result["fatigue_probability"]

        normal_label = config.POSTURE_LABELS.get(0, "Optimal")

        if posture_type != normal_label:
            self.posture_counter[posture_type] += 1

        self._update_hardware(pose_result, face_result)

        now = time.monotonic()

        if now - self.last_ui_emit_time >= self.ui_emit_interval:
            self.last_ui_emit_time = now

            if self.logger is not None:
                self.logger.save({
                    "posture_type": posture_type,
                    "fatigue_label": fatigue_label,
                    "fatigue_probability": float(fatigue_probability)
                })

            self.result_changed.emit({
                "posture_type": posture_type,
                "confidence": confidence,
                "fatigue_label": fatigue_label,
                "fatigue_probability": fatigue_probability,
                "elapsed_sec": self._get_elapsed_sec(),
                "rank_text": self._build_rank_text(),
                "frame_id": frame_id
            })

        del self.pose_results[frame_id]
        del self.face_results[frame_id]

    def _update_hardware(self, pose_result, face_result):
        if self.hardware_controller is None:
            return

        result = SimpleNamespace(
            success=True,
            pose_index=pose_result.get("pose_index"),
            fatigue_index=face_result.get("fatigue_index")
        )

        self.hardware_controller.update_hardware(result)

    def _get_elapsed_sec(self):
        if self.measurement_start_time is None:
            return 0

        return int(time.monotonic() - self.measurement_start_time)

    def _build_rank_text(self):
        total_count = sum(self.posture_counter.values())

        if total_count <= 0:
            return "불안정 자세 TOP 3\n\n1위  -\n2위  -\n3위  -"

        top_3 = self.posture_counter.most_common(3)
        lines = ["불안정 자세 TOP 3", ""]

        for index in range(3):
            if index >= len(top_3):
                lines.append(f"{index + 1}위  -")
                continue

            posture_type, count = top_3[index]
            ratio = count / total_count * 100
            lines.append(f"{index + 1}위  {posture_type}  {count}회  {ratio:.1f}%")

        return "\n".join(lines)

    def _cleanup_old_results(self, current_frame_id):
        min_frame_id = current_frame_id - 30

        for frame_id in list(self.pose_results.keys()):
            if frame_id < min_frame_id:
                print(f"[ResultWorker] Pose 결과 동기화 실패: Frame={frame_id}")
                del self.pose_results[frame_id]

        for frame_id in list(self.face_results.keys()):
            if frame_id < min_frame_id:
                print(f"[ResultWorker] Face 결과 동기화 실패: Frame={frame_id}")
                del self.face_results[frame_id]

    def stop(self):
        self.running = False

        if self.isRunning():
            self.wait()
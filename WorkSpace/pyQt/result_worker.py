import time
from collections import Counter
from queue import Empty
from pathlib import Path

from PyQt5.QtCore import QThread, pyqtSignal

import modules.config as config
from modules.logger import StudyLogger
from ipc.queue_utils import get_latest, drain_ordered


ROOT_DIR = Path(__file__).resolve().parent.parent


class VisionResultWorker(QThread):
    status_changed = pyqtSignal(str)
    calibration_finished = pyqtSignal(bool, str)
    measurement_start_finished = pyqtSignal(bool, str)
    result_changed = pyqtSignal(dict)
    ready_changed = pyqtSignal(bool)

    # 최신 State 전달용
    pose_state_changed = pyqtSignal(dict)
    face_state_changed = pyqtSignal(dict)
    hardware_changed = pyqtSignal(dict)

    # 유실되면 안 되는 Hardware Event 전달용
    hardware_event_changed = pyqtSignal(dict)

    def __init__(self, vision_manager, parent=None):
        super().__init__(parent)

        self.logger = None
        self.log_dir = ROOT_DIR / "data" / "session_log"
        self.vision_manager = vision_manager
        self.running = False

        self.pose_enabled = getattr(self.vision_manager, "enable_pose", True)
        self.face_enabled = getattr(self.vision_manager, "enable_face", False)

        self.pose_ready = not self.pose_enabled
        self.face_ready = not self.face_enabled

        self.pose_calibration_result = None
        self.face_calibration_result = None
        self.pose_calibration_progress = None
        self.face_calibration_progress = None

        self.pose_measurement_start_result = None
        self.face_measurement_start_result = None

        self.latest_pose_state = None
        self.latest_face_state = None
        self.latest_hardware_state = None

        # BOTH 모드에서는 정확히 같은 frame_id를 강제하지 않는다.
        # 각 Process의 가장 최근 GRU 결과를 조합한다.
        self.latest_pose_result = None
        self.latest_face_result = None

        self.posture_counter = Counter()
        self.last_counted_pose_frame_id = None
        self.measurement_start_time = None
        self.last_ui_emit_time = 0.0
        self.ui_emit_interval = 0.5
        self.fusion_max_gap_ns = 1_000_000_000  # 1초

    @property
    def is_ready(self):
        return self.pose_ready and self.face_ready

    def run(self):
        self.running = True

        while self.running:
            handled = False

            if self.pose_enabled:
                handled |= self._drain_pose_result_queue()
                handled |= self._drain_pose_state_queue()

            if self.face_enabled:
                handled |= self._drain_face_result_queue()
                handled |= self._drain_face_state_queue()

            handled |= self._drain_hardware_state_queue()
            handled |= self._drain_hardware_event_queue()

            if not handled:
                self.msleep(5)

    # ---------------------------------------------------------
    # Main result/event queues
    # ---------------------------------------------------------
    def _drain_pose_result_queue(self):
        handled = False
        try:
            while True:
                result = self.vision_manager.pose_result_queue.get_nowait()
                handled = True
                self._handle_pose_result(result)
        except Empty:
            pass
        return handled

    def _drain_face_result_queue(self):
        handled = False
        try:
            while True:
                result = self.vision_manager.face_result_queue.get_nowait()
                handled = True
                self._handle_face_result(result)
        except Empty:
            pass
        return handled

    # ---------------------------------------------------------
    # Latest state queues
    # ---------------------------------------------------------
    def _drain_pose_state_queue(self):
        queue = getattr(self.vision_manager, "pose_state_to_main_queue", None)
        if queue is None:
            return False

        latest = get_latest(queue, None)
        if latest is None:
            return False

        self.latest_pose_state = latest
        self.pose_state_changed.emit(latest)
        return True

    def _drain_face_state_queue(self):
        queue = getattr(self.vision_manager, "face_state_to_main_queue", None)
        if queue is None:
            return False

        latest = get_latest(queue, None)
        if latest is None:
            return False

        self.latest_face_state = latest
        self.face_state_changed.emit(latest)
        return True

    def _drain_hardware_state_queue(self):
        queue = getattr(self.vision_manager, "hw_to_main_state_queue", None)
        if queue is None:
            return False

        latest = get_latest(queue, None)
        if latest is None:
            return False

        self.latest_hardware_state = latest
        self.hardware_changed.emit(latest)
        return True

    def _drain_hardware_event_queue(self):
        queue = getattr(self.vision_manager, "hw_to_main_event_queue", None)
        if queue is None:
            return False

        events = drain_ordered(queue)
        if not events:
            return False

        for event in events:
            self.hardware_event_changed.emit(event)
            event_type = event.get("type", "") if isinstance(event, dict) else str(event)

            if event_type == "HARDWARE_READY":
                self.status_changed.emit(
                    event.get("message", "Hardware Process 준비 완료")
                    if isinstance(event, dict)
                    else "Hardware Process 준비 완료"
                )
            elif event_type == "HARDWARE_STOPPED":
                self.status_changed.emit("Hardware Process 종료")

        return True

    # ---------------------------------------------------------
    # Pose / Face result handling
    # ---------------------------------------------------------
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

        if result_type == "POSE_MEASUREMENT_STARTED":
            self.pose_measurement_start_result = result
            self._check_measurement_start_done()
            return

        if result_type == "POSE_RESULT":
            self.latest_pose_result = result
            self._emit_current_result()
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

        if result_type == "FACE_MEASUREMENT_STARTED":
            self.face_measurement_start_result = result
            self._check_measurement_start_done()
            return

        if result_type == "FACE_RESULT":
            self.latest_face_result = result
            self._emit_current_result()
            return

        if result_type == "FACE_STATS":
            print(
                f"[FACE] FPS={result['fps']:.1f} Drop={result['sequence_drop']} "
                f"Queue={result['queue_latency_ms']:.2f}ms Total={result['total_latency_ms']:.2f}ms"
            )

    # ---------------------------------------------------------
    # Ready
    # ---------------------------------------------------------
    def _check_ready(self):
        if not self.is_ready:
            return

        enabled = []
        if self.pose_enabled:
            enabled.append("Pose")
        if self.face_enabled:
            enabled.append("Face")

        enabled_text = " / ".join(enabled) if enabled else "Vision"
        self.status_changed.emit(f"{enabled_text} AI Process 준비 완료")
        self.ready_changed.emit(True)

    # ---------------------------------------------------------
    # Calibration
    # ---------------------------------------------------------
    def reset_calibration(self):
        self.pose_calibration_result = None if self.pose_enabled else {
            "type": "POSE_CALIBRATION_DONE",
            "success": True,
            "disabled": True,
            "message": "Pose Process 비활성",
            "sample_count": 0,
            "missing_count": 0,
        }
        self.face_calibration_result = None if self.face_enabled else {
            "type": "FACE_CALIBRATION_DONE",
            "success": True,
            "disabled": True,
            "message": "Face Process 비활성",
            "sample_count": 0,
            "missing_count": 0,
        }
        self.pose_calibration_progress = None
        self.face_calibration_progress = None

    def _emit_calibration_progress(self):
        parts = []
        remain_values = []

        if self.pose_enabled and self.pose_calibration_progress is not None:
            samples = self.pose_calibration_progress.get("sample_count", 0)
            remain = self.pose_calibration_progress.get("remain_time")
            parts.append(f"Pose {samples}개")
            if remain is not None:
                remain_values.append(remain)

        if self.face_enabled and self.face_calibration_progress is not None:
            samples = self.face_calibration_progress.get("sample_count", 0)
            remain = self.face_calibration_progress.get("remain_time")
            parts.append(f"Face {samples}개")
            if remain is not None:
                remain_values.append(remain)

        if remain_values:
            self.status_changed.emit(
                f"초기값 측정 중... {max(remain_values):.1f}초 / " + " / ".join(parts)
            )

    def _check_calibration_done(self):
        if self.pose_calibration_result is None or self.face_calibration_result is None:
            return

        pose = self.pose_calibration_result
        face = self.face_calibration_result
        success = pose.get("success", False) and face.get("success", False)

        lines = []
        if self.pose_enabled:
            lines.append(
                f"Pose: {pose.get('message', '')}\n"
                f"Pose Sample: {pose.get('sample_count', 0)} / Missing: {pose.get('missing_count', 0)}"
            )

        if self.face_enabled:
            lines.append(
                f"Face: {face.get('message', '')}\n"
                f"Face Sample: {face.get('sample_count', 0)} / Missing: {face.get('missing_count', 0)}"
            )

        self.vision_manager.stop_analysis()
        self.calibration_finished.emit(success, "\n\n".join(lines))

    # ---------------------------------------------------------
    # Measurement start handshake
    # ---------------------------------------------------------
    def reset_measurement_start(self):
        self.pose_measurement_start_result = None if self.pose_enabled else {
            "type": "POSE_MEASUREMENT_STARTED",
            "success": True,
            "disabled": True,
            "message": "Pose Process 비활성",
        }
        self.face_measurement_start_result = None if self.face_enabled else {
            "type": "FACE_MEASUREMENT_STARTED",
            "success": True,
            "disabled": True,
            "message": "Face Process 비활성",
        }

    def _check_measurement_start_done(self):
        if self.pose_measurement_start_result is None or self.face_measurement_start_result is None:
            return

        pose = self.pose_measurement_start_result
        face = self.face_measurement_start_result
        success = pose.get("success", False) and face.get("success", False)

        messages = []
        if self.pose_enabled:
            messages.append(f"Pose: {pose.get('message', '')}")
        if self.face_enabled:
            messages.append(f"Face: {face.get('message', '')}")

        self.measurement_start_finished.emit(success, " / ".join(messages))

    # ---------------------------------------------------------
    # Measurement session / UI
    # ---------------------------------------------------------
    def start_measurement_session(self):
        self.posture_counter.clear()
        self.last_counted_pose_frame_id = None
        self.latest_pose_result = None
        self.latest_face_result = None
        self.measurement_start_time = time.monotonic()
        self.last_ui_emit_time = 0.0
        self.logger = StudyLogger(base_dir=str(self.log_dir))

    def stop_measurement_session(self):
        self.measurement_start_time = None
        self.latest_pose_result = None
        self.latest_face_result = None
        self.logger = None

    def _emit_current_result(self):
        if self.pose_enabled and self.latest_pose_result is None:
            return
        if self.face_enabled and self.latest_face_result is None:
            return

        pose = self.latest_pose_result or {}
        face = self.latest_face_result or {}

        if self.pose_enabled and self.face_enabled:
            pose_ts = pose.get("timestamp_ns")
            face_ts = face.get("timestamp_ns")
            if pose_ts is not None and face_ts is not None:
                if abs(int(pose_ts) - int(face_ts)) > self.fusion_max_gap_ns:
                    # 한 Process 결과가 너무 오래된 경우 stale 값을 섞지 않는다.
                    return


        # 피로도 기능은 삭제하지 않는다.
        # 현재 POSE_ONLY 모드에서는 Streamlit 스키마와의 호환을 위해
        # Normal / 0.0을 기본값으로 저장한다.
        # Face Process를 다시 켜면 기존 FACE_RESULT 값을 그대로 사용한다.
        fatigue_label = (
            face.get("fatigue_label", "Normal")
            if self.face_enabled
            else "Normal"
        )

        fatigue_probability = (
            face.get("fatigue_probability", 0.0)
            if self.face_enabled
            else 0.0
        )

        pose_frame_id = pose.get("frame_id")
        face_frame_id = face.get("frame_id")
        frame_ids = [value for value in (pose_frame_id, face_frame_id) if value is not None]
        frame_id = max(frame_ids) if frame_ids else -1

        self._emit_ui_result(
            frame_id=frame_id,
            posture_type=pose.get("posture_type", "-"),
            confidence=pose.get("confidence", 0.0),
            fatigue_label=fatigue_label,
            fatigue_probability=fatigue_probability,
            pose_frame_id=pose_frame_id,
        )

    def _emit_ui_result(
        self,
        frame_id,
        posture_type,
        confidence,
        fatigue_label,
        fatigue_probability,
        pose_frame_id=None,
    ):
        normal_label = config.POSTURE_LABELS.get(0, "Optimal")

        # Face 결과가 갱신될 때 동일 Pose 결과를 중복 카운트하지 않는다.
        if (
            pose_frame_id is not None
            and pose_frame_id != self.last_counted_pose_frame_id
        ):
            self.last_counted_pose_frame_id = pose_frame_id
            if posture_type not in ("-", normal_label):
                self.posture_counter[posture_type] += 1

        if self.logger is not None:
            self.logger.save({
                "posture_type": posture_type,
                "fatigue_label": fatigue_label,
                "fatigue_probability": float(fatigue_probability),
            })


        now = time.monotonic()
        if now - self.last_ui_emit_time < self.ui_emit_interval:
            return

        self.last_ui_emit_time = now

        self.result_changed.emit({
            "posture_type": posture_type,
            "confidence": confidence,
            "fatigue_label": fatigue_label,
            "fatigue_probability": fatigue_probability,
            "elapsed_sec": self._get_elapsed_sec(),
            "rank_text": self._build_rank_text(),
            "frame_id": frame_id,
            "pose_frame_id": pose_frame_id,
            "face_frame_id": self.latest_face_result.get("frame_id") if self.latest_face_result else None,
        })

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

    def stop(self):
        self.running = False
        if self.isRunning():
            self.wait()

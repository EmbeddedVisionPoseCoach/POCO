import sys
import time
import traceback
import platform
from datetime import datetime
from enum import Enum, auto
from pathlib import Path

import cv2
import numpy as np

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage


ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR))

import modules.config as config

from managers.vision_process_manager_profile import VisionProcessManager
from result_worker import VisionResultWorker
from performance_metrics import JsonPerformanceLogger


CAMERA_FPS = 30
GUI_FPS = 15
PROFILE_INTERVAL_SEC = 2.0


class ProfileWindow:
    def __init__(self, interval_sec=PROFILE_INTERVAL_SEC):
        self.interval_sec = interval_sec
        self.reset()

    def reset(self):
        self.start_time = time.perf_counter()
        self.cpu_start = time.process_time()
        self.frame_count = 0
        self.gui_count = 0
        self.samples = {
            "capture": [],
            "preprocess": [],
            "shm": [],
            "gui": [],
            "loop": [],
        }

    def add(self, name, elapsed_ns):
        self.samples[name].append(elapsed_ns / 1_000_000.0)

    def frame_done(self):
        self.frame_count += 1

    def gui_done(self):
        self.gui_count += 1

    def ready(self):
        return time.perf_counter() - self.start_time >= self.interval_sec

    @staticmethod
    def _stats(values):
        if not values:
            return 0.0, 0.0, 0.0
        array = np.asarray(values, dtype=np.float64)
        return float(np.mean(array)), float(np.percentile(array, 95)), float(np.max(array))

    def build_snapshot(self, ring_stats=None, temperature=None, mode=None):
        elapsed = max(time.perf_counter() - self.start_time, 1e-9)
        camera_fps = self.frame_count / elapsed
        gui_fps = self.gui_count / elapsed
        main_cpu_percent = max(0.0, (time.process_time() - self.cpu_start) / elapsed * 100.0)

        capture_avg, capture_p95, capture_max = self._stats(self.samples["capture"])
        pre_avg, pre_p95, pre_max = self._stats(self.samples["preprocess"])
        shm_avg, shm_p95, shm_max = self._stats(self.samples["shm"])
        gui_avg, gui_p95, gui_max = self._stats(self.samples["gui"])
        loop_avg, loop_p95, loop_max = self._stats(self.samples["loop"])

        snapshot = {
            "mode": str(mode or "UNKNOWN"),
            "window_sec": elapsed,
            "camera_fps": camera_fps,
            "gui_fps": gui_fps,
            "main_cpu_percent": main_cpu_percent,
            "capture_ms_avg": capture_avg,
            "capture_ms_p95": capture_p95,
            "capture_ms_max": capture_max,
            "preprocess_ms_avg": pre_avg,
            "preprocess_ms_p95": pre_p95,
            "preprocess_ms_max": pre_max,
            "shared_memory_write_ms_avg": shm_avg,
            "shared_memory_write_ms_p95": shm_p95,
            "shared_memory_write_ms_max": shm_max,
            "gui_send_ms_avg": gui_avg,
            "gui_send_ms_p95": gui_p95,
            "gui_send_ms_max": gui_max,
            "camera_loop_ms_avg": loop_avg,
            "camera_loop_ms_p95": loop_p95,
            "camera_loop_ms_max": loop_max,
            "temperature_c": temperature,
        }
        if ring_stats is not None:
            snapshot.update({
                "pose_pending": ring_stats.get("pose_pending", 0),
                "pose_skipped": ring_stats.get("pose_skipped", 0),
                "pose_overrun": ring_stats.get("pose_overrun", 0),
                "pose_written": ring_stats.get("pose_written", 0),
                "pose_read": ring_stats.get("pose_read", 0),
            })
        return snapshot

    def build_report(self, ring_stats=None, temperature=None):
        snapshot = self.build_snapshot(ring_stats=ring_stats, temperature=temperature)
        camera_fps = snapshot["camera_fps"]
        gui_fps = snapshot["gui_fps"]
        capture_avg, capture_p95, capture_max = snapshot["capture_ms_avg"], snapshot["capture_ms_p95"], snapshot["capture_ms_max"]
        pre_avg, pre_p95, pre_max = snapshot["preprocess_ms_avg"], snapshot["preprocess_ms_p95"], snapshot["preprocess_ms_max"]
        shm_avg, shm_p95, shm_max = snapshot["shared_memory_write_ms_avg"], snapshot["shared_memory_write_ms_p95"], snapshot["shared_memory_write_ms_max"]
        gui_avg, gui_p95, gui_max = snapshot["gui_send_ms_avg"], snapshot["gui_send_ms_p95"], snapshot["gui_send_ms_max"]
        loop_avg, loop_p95, loop_max = snapshot["camera_loop_ms_avg"], snapshot["camera_loop_ms_p95"], snapshot["camera_loop_ms_max"]

        lines = [
            "",
            "================ CAMERA PROFILE ================",
            f"Camera FPS : {camera_fps:6.2f}    GUI FPS : {gui_fps:6.2f}    Main CPU : {snapshot['main_cpu_percent']:6.2f}%",
            "------------------------------------------------",
            "Stage          AVG(ms)    P95(ms)    MAX(ms)",
            f"Capture       {capture_avg:7.2f}    {capture_p95:7.2f}    {capture_max:7.2f}",
            f"Preprocess    {pre_avg:7.2f}    {pre_p95:7.2f}    {pre_max:7.2f}",
            f"SharedMemory  {shm_avg:7.2f}    {shm_p95:7.2f}    {shm_max:7.2f}",
            f"GUI(send)     {gui_avg:7.2f}    {gui_p95:7.2f}    {gui_max:7.2f}",
            f"Loop          {loop_avg:7.2f}    {loop_p95:7.2f}    {loop_max:7.2f}",
        ]

        if ring_stats is not None:
            lines.extend([
                "------------------------------------------------",
                f"Pose Pending={ring_stats['pose_pending']} Skip={ring_stats.get('pose_skipped', 0)} "
                f"Overrun={ring_stats['pose_overrun']} Written={ring_stats['pose_written']} Read={ring_stats['pose_read']}",
            ])
            if ring_stats.get("face_enabled", False):
                lines.append(
                    f"Face Pending={ring_stats['face_pending']} Skip={ring_stats.get('face_skipped', 0)} "
                    f"Overrun={ring_stats['face_overrun']} Written={ring_stats['face_written']} Read={ring_stats['face_read']}"
                )

        if temperature is not None:
            lines.append(f"Pi Temperature : {temperature:.1f} C")

        lines.append("================================================")
        return "\n".join(lines)


class OpenCVCameraSource:
    def __init__(self, camera_index=0, width=640, height=480, fps=CAMERA_FPS):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    def open(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("OpenCV 기본 카메라를 열 수 없습니다.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def start(self):
        pass

    def read(self):
        if self.cap is None:
            return False, None
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return False, None
        frame = cv2.flip(frame, 1)
        return True, frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class PiCamera2Source:
    def __init__(self, width=320, height=240, fps=CAMERA_FPS):
        self.width = width
        self.height = height
        self.fps = fps
        self.picam2 = None

    def open(self):
        from picamera2 import Picamera2
        from libcamera import Transform

        self.picam2 = Picamera2()
        camera_config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)},
            raw=None,
            buffer_count=6,
            transform=Transform(hflip=True),
            controls={"FrameRate": self.fps},
        )
        self.picam2.configure(camera_config)

    def start(self):
        if self.picam2 is not None:
            self.picam2.start()

    def read(self):
        if self.picam2 is None:
            return False, None
        frame = self.picam2.capture_array()
        if frame is None:
            return False, None
        return True, frame

    def release(self):
        if self.picam2 is None:
            return

        print("[SHUTDOWN] 1. Picamera2 stop 시작")

        try:
            self.picam2.stop()
            print("[SHUTDOWN] 2. Picamera2 stop 완료")
        except Exception as e:
            print("[SHUTDOWN] Picamera2 stop 오류:", e)

        print("[SHUTDOWN] 3. Picamera2 close 시작")

        try:
            self.picam2.close()
            print("[SHUTDOWN] 4. Picamera2 close 완료")
        except Exception as e:
            print("[SHUTDOWN] Picamera2 close 오류:", e)

        self.picam2 = None

        print("[SHUTDOWN] 5. Picamera2 객체 해제 완료")


class RunMode(Enum):
    PREVIEW = auto()
    PREPARING = auto()
    CALIBRATING = auto()
    MEASURING = auto()


class CameraWorker(QThread):
    frame_changed = pyqtSignal(QImage)
    status_changed = pyqtSignal(str)
    calibration_finished = pyqtSignal(bool, str)
    measurement_started = pyqtSignal(bool, str)
    result_changed = pyqtSignal(dict)
    pose_state_changed = pyqtSignal(dict)
    face_state_changed = pyqtSignal(dict)
    hardware_changed = pyqtSignal(dict)
    hardware_event_changed = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.running = False
        self.mode = RunMode.PREVIEW
        self.camera = None

        self.pending_preview_start = False
        self.pending_preparation_start = False
        self.pending_calibration_start = False
        self.pending_measurement_start = False
        self.hardware_calibration_preparing = False

        self.last_status_emit_time = 0.0
        self.status_emit_interval = 0.4

        frame_shape = (config.FRAME_HEIGHT, config.FRAME_WIDTH, 3)

        # 성능평가 1회 실행 = 앱 실행 1회 기준. 실제 JSON sample은 MEASURING 구간만 저장한다.
        self.performance_session_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.performance_dir = (
            ROOT_DIR / "data" / "performance" / f"multiprocess_{self.performance_session_id}"
        )
        self.performance_logger = JsonPerformanceLogger(
            self.performance_dir / "main_profile.json",
            session_id=self.performance_session_id,
            architecture="MULTIPROCESS",
            component="MAIN",
            metadata={
                "target_camera_fps": CAMERA_FPS,
                "gui_target_fps": GUI_FPS,
                "frame_width": config.FRAME_WIDTH,
                "frame_height": config.FRAME_HEIGHT,
                "profile_interval_sec": PROFILE_INTERVAL_SEC,
                "cpu_percent_basis": "process CPU time / wall time; 100% ~= one logical core",
            },
        )
        self.vision_manager = VisionProcessManager(
            frame_shape=frame_shape,
            performance_dir=self.performance_dir,
            performance_session_id=self.performance_session_id,
        )
        self.performance_logger.update_metadata(
            profile_mode=self.vision_manager.profile_mode,
            ring_slot_count=self.vision_manager.slot_count,
        )
        self.result_worker = VisionResultWorker(
            vision_manager=self.vision_manager,
            parent=self
        )
        self._vision_resources_shutdown = False

        self.result_worker.status_changed.connect(self.status_changed.emit)
        self.result_worker.result_changed.connect(self.result_changed.emit)
        self.result_worker.calibration_finished.connect(self._on_calibration_finished)
        self.result_worker.measurement_start_finished.connect(self._on_measurement_start_finished)
        self.result_worker.ready_changed.connect(self._on_vision_ready)
        self.result_worker.pose_state_changed.connect(self.pose_state_changed.emit)
        self.result_worker.face_state_changed.connect(self.face_state_changed.emit)
        self.result_worker.hardware_changed.connect(self.hardware_changed.emit)
        # Hardware calibration handshake는 CameraWorker가 먼저 처리하고
        # 동일 event를 Main UI에도 그대로 전달한다.
        self.result_worker.hardware_event_changed.connect(self._on_hardware_event)
        self.result_worker.hardware_event_changed.connect(self.hardware_event_changed.emit)

        self.vision_manager.start()
        self.result_worker.start()

    def run(self):
        self.running = True
        error_message = None
        frame_id = 0
        gui_interval = 1.0 / GUI_FPS
        last_gui_time = 0.0
        profiler = ProfileWindow(PROFILE_INTERVAL_SEC)
        last_profile_mode = self.mode

        try:
            self.camera = self.create_camera_source()
            self.camera.start()
            self.status_changed.emit("카메라 프리뷰 준비 완료 [PROFILE MODE]")
            print("[CameraWorker] 카메라 시작 [PROFILE MODE]")
            self.apply_pending_command()

            while self.running:
                # Preview/Calibration/Measurement가 한 profiling window에 섞이지 않게
                # mode 전환 시 측정 창을 새로 시작한다.
                if self.mode != last_profile_mode:
                    profiler.reset()
                    last_profile_mode = self.mode

                loop_start = time.perf_counter_ns()

                capture_start = time.perf_counter_ns()
                ret, frame = self.camera.read()
                capture_end = time.perf_counter_ns()
                profiler.add("capture", capture_end - capture_start)

                if not ret or frame is None:
                    self.emit_status_interval("카메라 프레임을 읽지 못했습니다.")
                    continue

                preprocess_start = time.perf_counter_ns()
                if frame.shape[:2] != (config.FRAME_HEIGHT, config.FRAME_WIDTH):
                    frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
                preprocess_end = time.perf_counter_ns()
                profiler.add("preprocess", preprocess_end - preprocess_start)

                frame_id += 1
                timestamp_ns = time.perf_counter_ns()

                shm_start = time.perf_counter_ns()
                pose_success, face_success = self.vision_manager.write_frame(
                    frame, frame_id, timestamp_ns
                )
                shm_end = time.perf_counter_ns()
                profiler.add("shm", shm_end - shm_start)

                if not pose_success:
                    self.emit_status_interval(f"Pose Ring Overrun 발생: Frame={frame_id}")
                if self.vision_manager.enable_face and not face_success:
                    self.emit_status_interval(f"Face Ring Overrun 발생: Frame={frame_id}")

                current_time = time.perf_counter()
                if current_time - last_gui_time >= gui_interval:
                    last_gui_time = current_time
                    gui_start = time.perf_counter_ns()
                    qimage = self.convert_frame_to_qimage(frame)
                    self.frame_changed.emit(qimage)
                    gui_end = time.perf_counter_ns()
                    profiler.add("gui", gui_end - gui_start)
                    profiler.gui_done()

                loop_end = time.perf_counter_ns()
                profiler.add("loop", loop_end - loop_start)
                profiler.frame_done()

                if profiler.ready():
                    ring_stats = self.vision_manager.get_stats()
                    temperature = self.read_pi_temperature()
                    print(profiler.build_report(ring_stats=ring_stats, temperature=temperature))

                    # 비교 실험에 필요한 Main 지표만 JSON으로 축적한다.
                    # Preview/Calibration 값이 섞이지 않도록 실제 측정 구간만 저장한다.
                    if self.mode == RunMode.MEASURING:
                        snapshot = profiler.build_snapshot(
                            ring_stats=ring_stats,
                            temperature=temperature,
                            mode=self.mode.name,
                        )
                        self.performance_logger.append(snapshot)

                    profiler.reset()

        except Exception as e:
            error_message = traceback.format_exc()
            print(error_message)
            self.status_changed.emit(f"카메라 오류 발생:\n{e}")

        finally:
            # 카메라는 이 QThread에서 생성했으므로 이 QThread에서 닫는다.
            # ResultWorker / multiprocessing 자원은 stop() 호출 스레드(Main)에서 정리한다.
            print("[SHUTDOWN] 0. CameraWorker finally 진입")

            self.release_resources(
                show_message=(error_message is None)
            )

            print("[SHUTDOWN] 6. CameraWorker finally 종료")

    def create_camera_source(self):
        width = config.FRAME_WIDTH
        height = config.FRAME_HEIGHT
        errors = []

        if platform.system().lower() == "linux":
            try:
                camera = PiCamera2Source(width=width, height=height, fps=CAMERA_FPS)
                camera.open()
                self.status_changed.emit(f"Picamera2 카메라를 사용합니다. ({CAMERA_FPS} FPS) [PROFILE]")
                print(f"[CameraWorker] Picamera2 선택 완료 ({CAMERA_FPS} FPS) [PROFILE]")
                return camera
            except Exception as e:
                error_text = f"Picamera2 실패: {e}"
                errors.append(error_text)
                print(error_text)
                self.status_changed.emit("Picamera2 실패. OpenCV 카메라를 시도합니다.")

        try:
            camera = OpenCVCameraSource(camera_index=0, width=width, height=height, fps=CAMERA_FPS)
            camera.open()
            self.status_changed.emit(f"OpenCV 기본 카메라를 사용합니다. ({CAMERA_FPS} FPS) [PROFILE]")
            print(f"[CameraWorker] OpenCV 카메라 선택 완료 ({CAMERA_FPS} FPS) [PROFILE]")
            return camera
        except Exception as e:
            error_text = f"OpenCV 카메라 실패: {e}"
            errors.append(error_text)
            print(error_text)

        raise RuntimeError("사용 가능한 카메라를 찾지 못했습니다.\n" + "\n".join(errors))

    def apply_pending_command(self):
        if self.pending_preparation_start and self.result_worker.is_ready:
            self.pending_preparation_start = False
            self.start_monitor_arm_preparation()
            return
        if self.pending_calibration_start and self.result_worker.is_ready:
            self.pending_calibration_start = False
            self.start_calibration()
            return
        if self.pending_measurement_start and self.result_worker.is_ready:
            self.pending_measurement_start = False
            self.start_measurement()
            return
        if self.pending_preview_start:
            self.pending_preview_start = False
            self.start_preview()

    def _on_vision_ready(self, ready):
        if ready:
            self.apply_pending_command()

    def start_preview(self):
        if not self.running:
            self.pending_preview_start = True
            self.status_changed.emit("카메라 준비 중입니다.")
            return

        if self.hardware_calibration_preparing:
            self.vision_manager.send_hardware_command({"type": "CANCEL_CALIBRATION_PREPARE"})
            self.hardware_calibration_preparing = False

        self.mode = RunMode.PREVIEW
        self.vision_manager.stop_analysis()
        self.vision_manager.send_main_state("PREVIEW")
        self.status_changed.emit("프리뷰 모드입니다. 바른 자세를 준비해주세요.")

    def start_monitor_arm_preparation(self):
        """카메라 프리뷰와 준비용 Pose 눈 랜드마크 처리를 함께 시작한다."""
        if not self.running or not self.result_worker.is_ready:
            self.pending_preparation_start = True
            self.status_changed.emit("카메라/Pose AI 준비 중입니다.")
            return
        self.mode = RunMode.PREPARING
        if not self.vision_manager.start_monitor_arm_preparation():
            self.status_changed.emit("모니터암 준비용 Pose AI를 시작하지 못했습니다.")
            return
        self.status_changed.emit(
            "모니터암 초기 준비 중: MediaPipe 눈 간격과 ToF를 확인하세요."
        )

    def finish_monitor_arm_preparation(self):
        self.pending_preparation_start = False
        self.vision_manager.finish_monitor_arm_preparation()
        self.mode = RunMode.PREVIEW
        self.status_changed.emit(
            "모니터암 초기 준비 완료. 이제 초기값 측정시작을 눌러주세요."
        )

    def start_calibration(self):
        """IMU X/Y 기준값 Calibration 완료 후 Pose/Face baseline Calibration을 시작한다."""
        if not self.running:
            self.pending_calibration_start = True
            self.status_changed.emit("카메라 준비 중입니다. 준비되면 초기 측정을 시작합니다.")
            return

        if not self.result_worker.is_ready:
            self.pending_calibration_start = True
            self.status_changed.emit("AI Process 준비 중입니다.")
            return

        if self.hardware_calibration_preparing or self.mode == RunMode.CALIBRATING:
            return

        # 아직 Pose baseline 수집을 시작하지 않는다.
        # 먼저 Hardware Process가 IMU X/Y 기준값 Calibration을 완료해야 한다.
        self.hardware_calibration_preparing = True
        self.mode = RunMode.PREVIEW
        self.vision_manager.stop_analysis()
        self.vision_manager.send_main_state("CALIBRATION_PRECHECK")

        sent = self.vision_manager.send_hardware_command({
            "type": "PREPARE_CALIBRATION"
        })

        if not sent:
            self.hardware_calibration_preparing = False
            self.vision_manager.send_main_state("PREVIEW")
            message = "Hardware Calibration 준비 명령을 전달하지 못했습니다."
            self.status_changed.emit(message)
            self.calibration_finished.emit(False, message)
            return

        self.status_changed.emit(
            "Calibration 준비: IMU X/Y 기준값을 측정합니다."
        )

    def _begin_vision_calibration(self):
        """Hardware IMU/Motor 준비 완료 ACK 후 실제 MediaPipe baseline 수집 시작."""
        self.result_worker.reset_calibration()
        self.vision_manager.start_calibration()
        self.mode = RunMode.CALIBRATING

        targets = []
        if self.vision_manager.enable_pose:
            targets.append("Pose")
        if self.vision_manager.enable_face:
            targets.append("Face")

        self.status_changed.emit(
            f"IMU/Motor 준비 완료. {' / '.join(targets)} 기준값 측정을 시작합니다. "
            f"{config.CALIBRATION_TIME}초 동안 IMU/Motor3/4 짐벌 제어와 함께 기준 자세를 유지해주세요."
        )

    def _on_hardware_event(self, event):
        if not isinstance(event, dict):
            return

        event_type = str(event.get("type", "")).upper()
        message = event.get("message", "")

        if event_type == "IMU_OFFSET_CALIBRATION_STARTED":
            if self.hardware_calibration_preparing and message:
                self.status_changed.emit(message)
            return

        if event_type == "HARDWARE_CALIBRATION_READY":
            if not self.hardware_calibration_preparing:
                return

            self.hardware_calibration_preparing = False
            success = bool(event.get("success", False))

            if not success:
                self.mode = RunMode.PREVIEW
                self.vision_manager.send_main_state("PREVIEW")
                fail_message = message or "IMU Calibration 준비에 실패했습니다."
                self.status_changed.emit(fail_message)
                self.calibration_finished.emit(False, fail_message)
                return

            self._begin_vision_calibration()
            return

    def _on_calibration_finished(self, success, message):
        self.hardware_calibration_preparing = False
        self.mode = RunMode.PREVIEW
        self.vision_manager.send_main_state("PREVIEW")
        self.calibration_finished.emit(success, message)

    def start_measurement(self):
        if not self.running:
            self.pending_measurement_start = True
            self.status_changed.emit("카메라 준비 중입니다. 준비되면 추론을 시작합니다.")
            return
        if not self.result_worker.is_ready:
            self.pending_measurement_start = True
            self.status_changed.emit("AI Process 준비 중입니다.")
            return

        # Measurement의 짐벌 제어는 현재 실행 세션의 IMU Offset을 그대로 사용한다.
        # 따라서 현재 실행 세션에서 IMU X/Y Calibration을 한 번도 하지 않았다면
        # 저장된 Pose baseline만으로 측정을 시작하지 않는다.
        hardware_state = getattr(self.result_worker, "latest_hardware_state", None)
        imu_state = hardware_state.get("imu", {}) if isinstance(hardware_state, dict) else {}
        motor_state = hardware_state.get("motor", {}) if isinstance(hardware_state, dict) else {}
        monitor_arm_state = (
            hardware_state.get("monitor_arm", {})
            if isinstance(hardware_state, dict)
            else {}
        )
        monitor_arm_calibration = monitor_arm_state.get("calibration", {})

        imu_ready = bool(
            imu_state.get("available", False)
            and imu_state.get("calibrated", False)
            and not imu_state.get("calibrating", False)
        )
        motor_ready = bool(
            motor_state.get("available", False)
            and motor_state.get("enabled", False)
            and motor_state.get("ready", False)
        )

        if not imu_ready or not motor_ready:
            message = (
                "현재 실행 세션의 IMU/Motor3/4 준비가 완료되지 않았습니다. "
                "먼저 초기값 준비 -> 초기값 측정을 진행해 IMU X/Y 기준값을 완료해주세요."
            )
            self.measurement_started.emit(False, message)
            self.status_changed.emit(message)
            return

        if (
            self.vision_manager.enable_pose
            and not monitor_arm_calibration.get("session_ready", False)
        ):
            message = (
                "현재 실행 세션의 ToF/눈 간격 5초 평균값이 없습니다. "
                "먼저 초기값 준비를 완료해주세요."
            )
            self.measurement_started.emit(False, message)
            self.status_changed.emit(message)
            return

        missing = []

        if self.vision_manager.enable_pose:
            pose_baseline = self.resolve_workspace_path(config.BASELINE_PATH)
            if not pose_baseline.exists():
                missing.append("Pose")

        if self.vision_manager.enable_face:
            face_baseline = self.resolve_workspace_path(config.FACE_BASELINE_PATH)
            if not face_baseline.exists():
                missing.append("Face")

        if missing:
            message = f"{', '.join(missing)} Calibration 기준값이 없습니다."
            self.measurement_started.emit(False, message)
            self.status_changed.emit(message)
            return

        # 여기서는 Calibration을 다시 하지 않는다.
        # 각 Process가 저장된 baseline을 실제로 정상 로드했는지 ACK를 받은 뒤
        # 측정 시작 성공을 Main UI에 알린다.
        self.result_worker.reset_measurement_start()
        # self.result_worker.start_measurement_session()
        self.vision_manager.start_measurement()
        self.status_changed.emit("저장된 Calibration 기준값을 불러오는 중입니다.")

    def _on_measurement_start_finished(self, success, detail):
        if not success:
            self.vision_manager.stop_analysis()
            self.result_worker.stop_measurement_session()
            self.mode = RunMode.PREVIEW
            message = f"측정 시작 실패: {detail}"
            self.measurement_started.emit(False, message)
            self.status_changed.emit(message)
            return

        # Pose Process의 모델 / Scaler / Baseline 준비가 끝난 시점부터
        # 실제 측정 시간을 기록한다.
        self.result_worker.start_measurement_session()

        # 모델/Scaler 로딩 ACK를 받은 뒤에만 Ring frame 공급을 재개한다.
        # 측정 시작 중 모델 로딩 때문에 Ring이 4개 가득 차는 overrun을 방지한다.
        self.vision_manager.resume_measurement_frames()
        self.mode = RunMode.MEASURING
        self.vision_manager.send_main_state("MEASURING")

        message = (
            f"GRU 모델 추론을 시작했습니다. {config.WINDOW_SIZE}프레임 수집 후 "
            f"{config.STRIDE}프레임마다 갱신됩니다."
        )
        self.measurement_started.emit(True, message)
        self.status_changed.emit(message)

    def stop_measurement(self):
        self.vision_manager.stop_analysis()
        self.result_worker.stop_measurement_session()
        self.mode = RunMode.PREVIEW
        self.vision_manager.send_main_state("PREVIEW")
        self.status_changed.emit("추론을 종료하고 프리뷰 모드로 돌아갑니다.")

    def convert_frame_to_qimage(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_frame.shape
        bytes_per_line = channel * width
        return QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format_RGB888
        ).copy()

    def resolve_workspace_path(self, path_text):
        path = Path(path_text)
        return path if path.is_absolute() else ROOT_DIR / path

    def emit_status_interval(self, message):
        now = time.monotonic()
        if now - self.last_status_emit_time < self.status_emit_interval:
            return
        self.last_status_emit_time = now
        self.status_changed.emit(message)

    @staticmethod
    def read_pi_temperature():
        path = Path("/sys/class/thermal/thermal_zone0/temp")
        try:
            if path.exists():
                return float(path.read_text().strip()) / 1000.0
        except Exception:
            pass
        return None

    def stop_camera_only(self):
        """Cam Off용. 카메라 QThread만 멈추고 Vision/Hardware Process는 IDLE로 유지한다.

        Cam Off마다 QThread/Queue/SharedMemory/PyQt wrapper를 파괴하지 않기 때문에
        Qt event loop로 복귀하는 시점의 native cleanup 충돌 가능성을 줄인다.
        다음 Cam On/Calibration에서 같은 CameraWorker를 다시 start()할 수 있다.
        """
        self.pending_preview_start = False
        self.pending_calibration_start = False
        self.pending_measurement_start = False

        manager = self.vision_manager
        if self.hardware_calibration_preparing and manager is not None:
            manager.send_hardware_command({"type": "CANCEL_CALIBRATION_PREPARE"})
        self.hardware_calibration_preparing = False

        result_worker = self.result_worker

        if manager is not None:
            manager.stop_analysis()
            manager.send_main_state("CAMERA_OFF")

        if result_worker is not None:
            result_worker.stop_measurement_session()

        self.running = False

        if self.isRunning():
            self.wait()

        self.mode = RunMode.PREVIEW
        print("[CameraWorker] 카메라만 종료 - Vision/Hardware Process는 IDLE 유지")

    def shutdown(self):
        """앱 종료 전용. 카메라를 멈춘 뒤 모든 Vision/Hardware 자원을 정리한다."""
        print("[SHUTDOWN] CameraWorker 전체 종료 시작")
        self.stop_camera_only()
        self.shutdown_vision_resources()
        print("[SHUTDOWN] CameraWorker 전체 종료 완료")

    def stop(self):
        # 기존 호출 호환용: stop()은 앱 전체 종료와 동일하게 처리한다.
        self.shutdown()

    def release_resources(self, show_message=True):
        if self.camera is not None:
            self.camera.release()
            self.camera = None

        print("[SHUTDOWN] Camera resources released")

    def shutdown_vision_resources(self):
        if self._vision_resources_shutdown:
            return

        manager = self.vision_manager
        worker = self.result_worker

        # 앱 종료 중에는 ResultWorker가 Main UI로 새 Qt signal을 보내지 않게 한다.
        if worker is not None:
            worker.blockSignals(True)

        # 생산자 Process를 먼저 멈춰 더 이상 Queue에 쓰지 않게 한다.
        if manager is not None:
            manager.stop()

        # 그 다음 Queue consumer QThread를 완전히 정지한다.
        if worker is not None:
            worker.stop()

        # 마지막에 Queue feeder thread / file descriptor를 명시적으로 정리한다.
        if manager is not None:
            manager.close_queues()

        # 여기서 QThread/QObject Python wrapper를 즉시 파괴하지 않는다.
        # CameraWorker(MainWindow의 child)가 Qt 종료 과정에서 안전하게 소멸하도록 유지한다.
        self._vision_resources_shutdown = True

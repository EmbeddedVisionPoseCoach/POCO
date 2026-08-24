import sys
import time
import traceback
import platform
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
from services.hardware_controller import HardwareController


CAMERA_FPS = 30
GUI_FPS = 15
PROFILE_INTERVAL_SEC = 2.0


class ProfileWindow:
    def __init__(self, interval_sec=PROFILE_INTERVAL_SEC):
        self.interval_sec = interval_sec
        self.reset()

    def reset(self):
        self.start_time = time.perf_counter()
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

    def build_report(self, ring_stats=None, temperature=None):
        elapsed = max(time.perf_counter() - self.start_time, 1e-9)
        camera_fps = self.frame_count / elapsed
        gui_fps = self.gui_count / elapsed

        capture_avg, capture_p95, capture_max = self._stats(self.samples["capture"])
        pre_avg, pre_p95, pre_max = self._stats(self.samples["preprocess"])
        shm_avg, shm_p95, shm_max = self._stats(self.samples["shm"])
        gui_avg, gui_p95, gui_max = self._stats(self.samples["gui"])
        loop_avg, loop_p95, loop_max = self._stats(self.samples["loop"])

        lines = [
            "",
            "================ CAMERA PROFILE ================",
            f"Camera FPS : {camera_fps:6.2f}    GUI FPS : {gui_fps:6.2f}",
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
                f"Pose Pending={ring_stats['pose_pending']} Overrun={ring_stats['pose_overrun']} "
                f"Written={ring_stats['pose_written']} Read={ring_stats['pose_read']}",
                f"Face Pending={ring_stats['face_pending']} Overrun={ring_stats['face_overrun']} "
                f"Written={ring_stats['face_written']} Read={ring_stats['face_read']}",
            ])

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
        try:
            self.picam2.stop()
        except Exception:
            pass
        try:
            self.picam2.close()
        except Exception:
            pass
        self.picam2 = None


class RunMode(Enum):
    PREVIEW = auto()
    HARDWARE = auto()
    CALIBRATING = auto()
    MEASURING = auto()


class CameraWorker(QThread):
    frame_changed = pyqtSignal(QImage)
    status_changed = pyqtSignal(str)
    calibration_finished = pyqtSignal(bool, str)
    measurement_started = pyqtSignal(bool, str)
    result_changed = pyqtSignal(dict)

    def __init__(self, hardware_controller=None, parent=None):
        super().__init__(parent)

        self.running = False
        self.mode = RunMode.PREVIEW
        self.camera = None

        self.pending_preview_start = False
        self.pending_calibration_start = False
        self.pending_measurement_start = False

        self.hardware_init_requested = False
        self.last_status_emit_time = 0.0
        self.status_emit_interval = 0.4

        if hardware_controller is None:
            self.hardware_controller = HardwareController(
                enabled=config.HARDWARE_ENABLED,
                serial_port=config.HARDWARE_SERIAL_PORT,
                baud_rate=config.HARDWARE_BAUD_RATE,
                timeout=config.HARDWARE_TIMEOUT
            )
            self.owns_hardware_controller = True
        else:
            self.hardware_controller = hardware_controller
            self.owns_hardware_controller = False

        frame_shape = (config.FRAME_HEIGHT, config.FRAME_WIDTH, 3)
        self.vision_manager = VisionProcessManager(frame_shape=frame_shape, slot_count=32)
        self.result_worker = VisionResultWorker(
            vision_manager=self.vision_manager,
            hardware_controller=self.hardware_controller
        )

        self.result_worker.status_changed.connect(self.status_changed.emit)
        self.result_worker.result_changed.connect(self.result_changed.emit)
        self.result_worker.calibration_finished.connect(self._on_calibration_finished)
        self.result_worker.ready_changed.connect(self._on_vision_ready)

        self.vision_manager.start()
        self.result_worker.start()

    def run(self):
        self.running = True
        error_message = None
        frame_id = 0
        gui_interval = 1.0 / GUI_FPS
        last_gui_time = 0.0
        profiler = ProfileWindow(PROFILE_INTERVAL_SEC)

        try:
            self.camera = self.create_camera_source()
            self.camera.start()
            self.status_changed.emit("카메라 프리뷰 준비 완료 [PROFILE MODE]")
            print("[CameraWorker] 카메라 시작 [PROFILE MODE]")
            self.apply_pending_command()

            while self.running:
                loop_start = time.perf_counter_ns()

                capture_start = time.perf_counter_ns()
                ret, frame = self.camera.read()
                capture_end = time.perf_counter_ns()
                profiler.add("capture", capture_end - capture_start)

                if not ret or frame is None:
                    self.emit_status_interval("카메라 프레임을 읽지 못했습니다.")
                    continue

                if self.hardware_init_requested:
                    self._start_hardware_then_calibration()

                preprocess_start = time.perf_counter_ns()
                if frame.shape[:2] != (config.FRAME_HEIGHT, config.FRAME_WIDTH):
                    frame = cv2.resize(frame, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
                preprocess_end = time.perf_counter_ns()
                profiler.add("preprocess", preprocess_end - preprocess_start)

                frame_id += 1
                timestamp_ns = time.perf_counter_ns()

                shm_start = time.perf_counter_ns()
                pose_success, face_success = self.vision_manager.write_frame(frame, frame_id, timestamp_ns)
                shm_end = time.perf_counter_ns()
                profiler.add("shm", shm_end - shm_start)

                if not pose_success:
                    self.emit_status_interval(f"Pose Ring Overrun 발생: Frame={frame_id}")
                if not face_success:
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
                    profiler.reset()

        except Exception as e:
            error_message = traceback.format_exc()
            print(error_message)
            self.status_changed.emit(f"카메라 오류 발생:\n{e}")

        finally:
            self.release_resources(show_message=(error_message is None))
            self.shutdown_vision_resources()

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
        self.mode = RunMode.PREVIEW
        self.vision_manager.stop_analysis()
        self.status_changed.emit("프리뷰 모드입니다. 바른 자세를 준비해주세요.")

    def start_calibration(self):
        if not self.running:
            self.pending_calibration_start = True
            self.status_changed.emit("카메라 준비 중입니다. 준비되면 초기 측정을 시작합니다.")
            return
        if not self.result_worker.is_ready:
            self.pending_calibration_start = True
            self.status_changed.emit("AI Process 준비 중입니다.")
            return
        self.mode = RunMode.HARDWARE
        self.hardware_init_requested = True
        self.status_changed.emit("카메라 수평 보정을 시작합니다.")

    def _start_hardware_then_calibration(self):
        if not self.hardware_init_requested:
            return
        self.hardware_init_requested = False
        self.status_changed.emit("카메라 수평 보정 중입니다.")
        hardware_success = self.hardware_controller.start_HardwareSet()
        if not hardware_success:
            print("[CameraWorker] 수평 보정 실패 또는 비활성 상태")
        self.result_worker.reset_calibration()
        self.vision_manager.start_calibration()
        self.mode = RunMode.CALIBRATING
        self.status_changed.emit(
            f"초기 자세/얼굴 기준값 측정을 시작합니다. {config.CALIBRATION_TIME}초 동안 바른 자세를 유지해주세요."
        )

    def _on_calibration_finished(self, success, message):
        self.mode = RunMode.PREVIEW
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

        pose_baseline = self.resolve_workspace_path(config.BASELINE_PATH)
        face_baseline = self.resolve_workspace_path(config.FACE_BASELINE_PATH)
        if not pose_baseline.exists() or not face_baseline.exists():
            message = "Pose / Face Calibration 기준값이 없습니다."
            self.measurement_started.emit(False, message)
            self.status_changed.emit(message)
            return

        self.result_worker.start_measurement_session()
        self.vision_manager.start_measurement()
        self.mode = RunMode.MEASURING
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

    def stop(self):
        self.running = False
        self.pending_preview_start = False
        self.pending_calibration_start = False
        self.pending_measurement_start = False
        if self.isRunning():
            self.wait()

    def release_resources(self, show_message=True):
        if self.camera is not None:
            self.camera.release()
            self.camera = None
        if self.owns_hardware_controller and self.hardware_controller is not None:
            self.hardware_controller.close()
        if show_message:
            self.status_changed.emit("카메라가 종료되었습니다.")

    def shutdown_vision_resources(self):
        if self.result_worker is not None:
            self.result_worker.stop()
            self.result_worker = None
        if self.vision_manager is not None:
            self.vision_manager.stop()
            self.vision_manager = None
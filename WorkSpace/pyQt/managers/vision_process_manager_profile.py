import multiprocessing as mp
import numpy as np

from ipc.queue_utils import put_latest, put_ordered
from ipc.shared_frame_ring import create_ring_resources, SharedFrameWriter


# ============================================================
# PROFILE MODE SWITCH
# "POSE_ONLY" : Pose Process만 실행
# "FACE_ONLY" : Face Process만 실행
# "BOTH"      : Pose + Face 동시 실행
#
# Face 기능을 끄기 위해 관련 코드를 주석처리하지 않는다.
# 실행 여부는 PROFILE_MODE 하나로만 제어한다.
# ============================================================
PROFILE_MODE = "POSE_ONLY"

_VALID_PROFILE_MODES = {"POSE_ONLY", "FACE_ONLY", "BOTH"}


# State : 최신값 1개만 필요 (landmark / feature / sensor / mode 등)
# Event : 순서/유실 방지가 중요 (command / done / error / ack 등)
STATE_QUEUE_SIZE = 1
EVENT_QUEUE_SIZE = 32
VISION_COMMAND_QUEUE_SIZE = 16
VISION_RESULT_QUEUE_SIZE = 64
FRAME_RING_SLOT_COUNT = 4


def pose_process_entry(
    stop_event,
    command_queue,
    result_queue,
    state_to_main_queue,
    ring_resources,
    pose_to_hw_state_queue,
    pose_to_hw_event_queue,
    hw_to_pose_state_queue,
    hw_to_pose_event_queue,
):
    from processes.pose_process_profile import run_pose_process

    run_pose_process(
        stop_event,
        command_queue,
        result_queue,
        state_to_main_queue,
        ring_resources,
        pose_to_hw_state_queue,
        pose_to_hw_event_queue,
        hw_to_pose_state_queue,
        hw_to_pose_event_queue,
    )


def face_process_entry(
    stop_event,
    command_queue,
    result_queue,
    state_to_main_queue,
    ring_resources,
    face_to_hw_state_queue,
    face_to_hw_event_queue,
    hw_to_face_state_queue,
    hw_to_face_event_queue,
):
    from processes.face_process_profile import run_face_process

    run_face_process(
        stop_event,
        command_queue,
        result_queue,
        state_to_main_queue,
        ring_resources,
        face_to_hw_state_queue,
        face_to_hw_event_queue,
        hw_to_face_state_queue,
        hw_to_face_event_queue,
    )


def hardware_process_entry(
    stop_event,
    enable_pose,
    enable_face,
    main_to_hw_state_queue,
    main_to_hw_event_queue,
    hw_to_main_state_queue,
    hw_to_main_event_queue,
    pose_to_hw_state_queue,
    pose_to_hw_event_queue,
    hw_to_pose_state_queue,
    hw_to_pose_event_queue,
    face_to_hw_state_queue,
    face_to_hw_event_queue,
    hw_to_face_state_queue,
    hw_to_face_event_queue,
):
    from processes.hardware_process import run_hardware_process

    run_hardware_process(
        stop_event,
        enable_pose,
        enable_face,
        main_to_hw_state_queue,
        main_to_hw_event_queue,
        hw_to_main_state_queue,
        hw_to_main_event_queue,
        pose_to_hw_state_queue,
        pose_to_hw_event_queue,
        hw_to_pose_state_queue,
        hw_to_pose_event_queue,
        face_to_hw_state_queue,
        face_to_hw_event_queue,
        hw_to_face_state_queue,
        hw_to_face_event_queue,
    )


class VisionProcessManager:
    def __init__(self, frame_shape, slot_count=FRAME_RING_SLOT_COUNT, profile_mode=PROFILE_MODE):
        profile_mode = str(profile_mode).upper()
        if profile_mode not in _VALID_PROFILE_MODES:
            raise ValueError(
                f"지원하지 않는 PROFILE_MODE입니다: {profile_mode}. "
                f"사용 가능: {sorted(_VALID_PROFILE_MODES)}"
            )

        self.profile_mode = profile_mode
        self.enable_pose = profile_mode in ("POSE_ONLY", "BOTH")
        self.enable_face = profile_mode in ("FACE_ONLY", "BOTH")

        self.ctx = mp.get_context("spawn")
        self.frame_shape = tuple(frame_shape)
        self.slot_count = max(2, int(slot_count))
        self.accept_frames = False
        self.stop_event = self.ctx.Event()
        self._stopped = False
        self._queues_closed = False

        # ----------------------------------------------------
        # Main <-> Pose / Face
        # Queue 객체는 항상 만든다.
        # Process 실행 여부만 enable_pose / enable_face로 제어한다.
        # ----------------------------------------------------
        self.pose_command_queue = self.ctx.Queue(maxsize=VISION_COMMAND_QUEUE_SIZE)
        self.face_command_queue = self.ctx.Queue(maxsize=VISION_COMMAND_QUEUE_SIZE)

        self.pose_result_queue = self.ctx.Queue(maxsize=VISION_RESULT_QUEUE_SIZE)
        self.face_result_queue = self.ctx.Queue(maxsize=VISION_RESULT_QUEUE_SIZE)

        self.pose_state_to_main_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.face_state_to_main_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)

        # ----------------------------------------------------
        # Main <-> Hardware
        # ----------------------------------------------------
        self.main_to_hw_state_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.main_to_hw_event_queue = self.ctx.Queue(maxsize=EVENT_QUEUE_SIZE)
        self.hw_to_main_state_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.hw_to_main_event_queue = self.ctx.Queue(maxsize=EVENT_QUEUE_SIZE)

        # ----------------------------------------------------
        # Pose <-> Hardware
        # ----------------------------------------------------
        self.pose_to_hw_state_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.pose_to_hw_event_queue = self.ctx.Queue(maxsize=EVENT_QUEUE_SIZE)
        self.hw_to_pose_state_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.hw_to_pose_event_queue = self.ctx.Queue(maxsize=EVENT_QUEUE_SIZE)

        # ----------------------------------------------------
        # Face <-> Hardware
        # Face가 비활성이어도 IPC 인터페이스 자체는 유지한다.
        # ----------------------------------------------------
        self.face_to_hw_state_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.face_to_hw_event_queue = self.ctx.Queue(maxsize=EVENT_QUEUE_SIZE)
        self.hw_to_face_state_queue = self.ctx.Queue(maxsize=STATE_QUEUE_SIZE)
        self.hw_to_face_event_queue = self.ctx.Queue(maxsize=EVENT_QUEUE_SIZE)

        # ----------------------------------------------------
        # Main(CameraWorker) -> Pose / Face Shared Frame Ring
        # 실제로 활성화된 Vision Process의 Shared Memory만 만든다.
        # ----------------------------------------------------
        self.pose_shm = None
        self.pose_resources = None
        self.pose_writer = None
        if self.enable_pose:
            self.pose_shm, self.pose_resources = create_ring_resources(
                ctx=self.ctx,
                slot_count=self.slot_count,
                frame_shape=self.frame_shape,
                frame_dtype=np.uint8,
            )
            self.pose_writer = SharedFrameWriter(self.pose_resources)

        self.face_shm = None
        self.face_resources = None
        self.face_writer = None
        if self.enable_face:
            self.face_shm, self.face_resources = create_ring_resources(
                ctx=self.ctx,
                slot_count=self.slot_count,
                frame_shape=self.frame_shape,
                frame_dtype=np.uint8,
            )
            self.face_writer = SharedFrameWriter(self.face_resources)

        self.pose_process = None
        self.face_process = None
        self.hardware_process = None

        print(
            f"[VisionProcessManager] Mode={self.profile_mode} "
            f"Pose={'ON' if self.enable_pose else 'OFF'} "
            f"Face={'ON' if self.enable_face else 'OFF'} "
            f"Hardware=IPC_ONLY RingSlots={self.slot_count}"
        )

    # --------------------------------------------------------
    # Process lifecycle
    # --------------------------------------------------------
    def start(self):
        self.hardware_process = self.ctx.Process(
            target=hardware_process_entry,
            args=(
                self.stop_event,
                self.enable_pose,
                self.enable_face,
                self.main_to_hw_state_queue,
                self.main_to_hw_event_queue,
                self.hw_to_main_state_queue,
                self.hw_to_main_event_queue,
                self.pose_to_hw_state_queue,
                self.pose_to_hw_event_queue,
                self.hw_to_pose_state_queue,
                self.hw_to_pose_event_queue,
                self.face_to_hw_state_queue,
                self.face_to_hw_event_queue,
                self.hw_to_face_state_queue,
                self.hw_to_face_event_queue,
            ),
            name="HardwareProcess",
        )
        self.hardware_process.start()

        if self.enable_pose:
            self.pose_process = self.ctx.Process(
                target=pose_process_entry,
                args=(
                    self.stop_event,
                    self.pose_command_queue,
                    self.pose_result_queue,
                    self.pose_state_to_main_queue,
                    self.pose_resources,
                    self.pose_to_hw_state_queue,
                    self.pose_to_hw_event_queue,
                    self.hw_to_pose_state_queue,
                    self.hw_to_pose_event_queue,
                ),
                name="PoseProcess",
            )
            self.pose_process.start()

        if self.enable_face:
            self.face_process = self.ctx.Process(
                target=face_process_entry,
                args=(
                    self.stop_event,
                    self.face_command_queue,
                    self.face_result_queue,
                    self.face_state_to_main_queue,
                    self.face_resources,
                    self.face_to_hw_state_queue,
                    self.face_to_hw_event_queue,
                    self.hw_to_face_state_queue,
                    self.hw_to_face_event_queue,
                ),
                name="FaceProcess",
            )
            self.face_process.start()

    # --------------------------------------------------------
    # Main -> Vision
    # --------------------------------------------------------
    def start_calibration(self):
        """Calibration 버튼에서만 새 baseline 측정을 시작한다."""
        self.accept_frames = True

        if self.enable_pose:
            if not put_ordered(self.pose_command_queue, "START_CALIBRATION"):
                self.pose_result_queue.put({
                    "type": "POSE_CALIBRATION_DONE",
                    "success": False,
                    "message": "Pose command queue가 가득 차 Calibration 명령을 전달하지 못했습니다.",
                    "sample_count": 0,
                    "missing_count": 0,
                })

        if self.enable_face:
            if not put_ordered(self.face_command_queue, "START_CALIBRATION"):
                self.face_result_queue.put({
                    "type": "FACE_CALIBRATION_DONE",
                    "success": False,
                    "message": "Face command queue가 가득 차 Calibration 명령을 전달하지 못했습니다.",
                    "sample_count": 0,
                    "missing_count": 0,
                })

        self.send_main_state("CALIBRATING")

    def start_measurement(self):
        """새 Calibration 없이 저장된 baseline을 로드해 측정을 시작한다.

        모델/Scaler 로딩은 Pose/Face Process 내부에서 동기적으로 일어나므로
        START ACK를 받기 전에는 Shared Frame 기록을 잠시 중단한다.
        카메라/UI 프리뷰는 계속 동작하고 Ring만 채우지 않는다.
        """
        self.accept_frames = False

        if self.enable_pose:
            if not put_ordered(self.pose_command_queue, "START_MEASUREMENT"):
                self.pose_result_queue.put({
                    "type": "POSE_MEASUREMENT_STARTED",
                    "success": False,
                    "message": "Pose command queue가 가득 차 측정 명령을 전달하지 못했습니다.",
                })

        if self.enable_face:
            if not put_ordered(self.face_command_queue, "START_MEASUREMENT"):
                self.face_result_queue.put({
                    "type": "FACE_MEASUREMENT_STARTED",
                    "success": False,
                    "message": "Face command queue가 가득 차 측정 명령을 전달하지 못했습니다.",
                })

        self.send_main_state("MEASUREMENT_STARTING")
        print(f"[VisionProcessManager] Measurement 시작 요청 ({self.profile_mode})")

    def resume_measurement_frames(self):
        """Pose/Face가 모델 로드를 끝내고 START ACK를 보낸 뒤 frame 공급을 재개한다."""
        if not self.stop_event.is_set():
            self.accept_frames = True

    def start_analysis(self):
        self.start_measurement()

    def stop_analysis(self):
        self.accept_frames = False

        if self.enable_pose:
            put_ordered(self.pose_command_queue, "STOP")

        if self.enable_face:
            put_ordered(self.face_command_queue, "STOP")

        self.send_main_state("IDLE")

    # --------------------------------------------------------
    # Main <-> Hardware IPC
    # --------------------------------------------------------
    def send_main_state(self, state, **extra):
        message = {
            "type": "MAIN_STATE",
            "state": state,
            **extra,
        }
        return put_latest(self.main_to_hw_state_queue, message)

    def send_hardware_command(self, message):
        """유실되면 안 되는 Main -> Hardware Command/Event 전송."""
        if isinstance(message, str):
            message = {"type": message}
        return put_ordered(self.main_to_hw_event_queue, message)

    # --------------------------------------------------------
    # Frame
    # --------------------------------------------------------
    def write_frame(self, frame, frame_id, timestamp_ns):
        """Main에서 읽은 동일 frame을 활성 Pose/Face Process에 전달한다.

        반환값은 (pose_success, face_success) 튜플이다.
        비활성 Process는 성공(True)으로 취급한다.
        """
        if not self.accept_frames:
            return True, True

        pose_success = True
        face_success = True

        if self.enable_pose and self.pose_writer is not None:
            pose_success = self.pose_writer.write(frame, frame_id, timestamp_ns)

        if self.enable_face and self.face_writer is not None:
            face_success = self.face_writer.write(frame, frame_id, timestamp_ns)

        return pose_success, face_success

    # --------------------------------------------------------
    # Stats
    # --------------------------------------------------------
    @staticmethod
    def _channel_stats(resources):
        if resources is None:
            return {
                "written": 0,
                "read": 0,
                "pending": 0,
                "skipped": 0,
                "overrun": 0,
            }

        written = resources["written_count"].value
        read = resources["read_count"].value
        return {
            "written": written,
            "read": read,
            "pending": max(0, written - read),
            "skipped": resources["skipped_count"].value,
            "overrun": resources["overrun_count"].value,
        }

    def get_stats(self):
        pose = self._channel_stats(self.pose_resources)
        face = self._channel_stats(self.face_resources)

        return {
            "profile_mode": self.profile_mode,
            "pose_enabled": self.enable_pose,
            "face_enabled": self.enable_face,
            "pose_written": pose["written"],
            "pose_read": pose["read"],
            "pose_pending": pose["pending"],
            "pose_skipped": pose["skipped"],
            "pose_overrun": pose["overrun"],
            "face_written": face["written"],
            "face_read": face["read"],
            "face_pending": face["pending"],
            "face_skipped": face["skipped"],
            "face_overrun": face["overrun"],
        }

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------
    def stop(self):
        """Vision/Hardware child process와 SharedMemory를 먼저 안전하게 종료한다.

        Queue 자체의 close는 ResultWorker가 완전히 멈춘 뒤 close_queues()에서 수행한다.
        """
        if self._stopped:
            return

        self._stopped = True
        self.accept_frames = False
        self.stop_event.set()

        if self.enable_pose:
            try:
                self.pose_command_queue.put_nowait("SHUTDOWN")
            except Exception:
                pass

        if self.enable_face:
            try:
                self.face_command_queue.put_nowait("SHUTDOWN")
            except Exception:
                pass

        self._stop_process(self.pose_process, "PoseProcess")
        self._stop_process(self.face_process, "FaceProcess")
        self._stop_process(self.hardware_process, "HardwareProcess")

        self.pose_process = None
        self.face_process = None
        self.hardware_process = None

        self._close_frame_channel(self.pose_writer, self.pose_shm)
        self._close_frame_channel(self.face_writer, self.face_shm)
        self.pose_writer = None
        self.pose_shm = None
        self.face_writer = None
        self.face_shm = None

        print(f"[VisionProcessManager] 종료 ({self.profile_mode})")

    def close_queues(self):
        """모든 Queue feeder thread/file descriptor를 명시적으로 정리한다."""
        if self._queues_closed:
            return
        self._queues_closed = True

        queue_names = [
            name for name in vars(self)
            if name.endswith("_queue")
        ]

        for name in queue_names:
            queue_obj = getattr(self, name, None)
            if queue_obj is None:
                continue
            try:
                queue_obj.close()
            except Exception:
                pass
            try:
                queue_obj.join_thread()
            except Exception:
                pass

    @staticmethod
    def _close_frame_channel(writer, shm):
        if writer is not None:
            try:
                writer.close()
            except Exception:
                pass

        if shm is not None:
            try:
                shm.close()
            except Exception:
                pass
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
            except Exception:
                pass

    @staticmethod
    def _stop_process(process, process_name="Process"):
        if process is None:
            return

        process.join(timeout=5)

        if process.is_alive():
            print(f"[VisionProcessManager] {process_name} 정상 종료 timeout -> terminate")
            process.terminate()
            process.join(timeout=2)

        exitcode = process.exitcode
        print(f"[VisionProcessManager] {process_name} exitcode={exitcode}")

        try:
            process.close()
        except Exception:
            pass


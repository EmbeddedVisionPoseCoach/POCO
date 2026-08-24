import multiprocessing as mp
import numpy as np

from ipc.shared_frame_ring import create_ring_resources, SharedFrameWriter


# ============================================================
# PROFILE MODE SWITCH - 2026-08-18
# "POSE_ONLY" : Pose Process만 실행
# "FACE_ONLY" : Face Process만 실행
# "BOTH"      : Pose + Face 동시 실행
# ============================================================
PROFILE_MODE = "POSE_ONLY"

_VALID_PROFILE_MODES = {"POSE_ONLY", "FACE_ONLY", "BOTH"}


def pose_process_entry(stop_event, command_queue, result_queue, ring_resources):
    # MediaPipe / TFLite 관련 import를 Main Process가 아니라 Child에서만 수행
    from processes.pose_process_profile import run_pose_process
    run_pose_process(stop_event, command_queue, result_queue, ring_resources)


def face_process_entry(stop_event, command_queue, result_queue, ring_resources):
    # MediaPipe / TFLite 관련 import를 Main Process가 아니라 Child에서만 수행
    from processes.face_process_profile import run_face_process
    run_face_process(stop_event, command_queue, result_queue, ring_resources)


class VisionProcessManager:
    def __init__(self, frame_shape, slot_count=32, profile_mode=PROFILE_MODE):
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
        self.slot_count = slot_count
        self.accept_frames = False
        self.stop_event = self.ctx.Event()

        # ResultWorker API 호환을 위해 Queue는 둘 다 유지한다.
        self.pose_command_queue = self.ctx.Queue()
        self.face_command_queue = self.ctx.Queue()
        self.pose_result_queue = self.ctx.Queue()
        self.face_result_queue = self.ctx.Queue()

        # Camera / ResultWorker / get_stats API를 그대로 유지하기 위해
        # Ring resource도 둘 다 생성한다. 단, 비활성 Ring에는 write하지 않는다.
        self.pose_shm, self.pose_resources = create_ring_resources(
            ctx=self.ctx,
            slot_count=self.slot_count,
            frame_shape=self.frame_shape,
            frame_dtype=np.uint8
        )
        self.face_shm, self.face_resources = create_ring_resources(
            ctx=self.ctx,
            slot_count=self.slot_count,
            frame_shape=self.frame_shape,
            frame_dtype=np.uint8
        )

        self.pose_writer = SharedFrameWriter(self.pose_resources)
        self.face_writer = SharedFrameWriter(self.face_resources)

        self.pose_process = None
        self.face_process = None

        print(
            f"[VisionProcessManager:PROFILE] Mode={self.profile_mode} "
            f"Pose={'ON' if self.enable_pose else 'OFF'} "
            f"Face={'ON' if self.enable_face else 'OFF'}"
        )

    def start(self):
        if self.enable_pose:
            self.pose_process = self.ctx.Process(
                target=pose_process_entry,
                args=(
                    self.stop_event,
                    self.pose_command_queue,
                    self.pose_result_queue,
                    self.pose_resources
                ),
                name="PoseProcessProfile"
            )
            self.pose_process.start()
        else:
            # ResultWorker가 Pose READY를 영원히 기다리지 않도록 처리.
            self.pose_result_queue.put({
                "type": "POSE_READY",
                "success": True,
                "disabled": True,
                "profile_mode": self.profile_mode
            })
            print("[VisionProcessManager:PROFILE] Pose Process 비활성")

        if self.enable_face:
            self.face_process = self.ctx.Process(
                target=face_process_entry,
                args=(
                    self.stop_event,
                    self.face_command_queue,
                    self.face_result_queue,
                    self.face_resources
                ),
                name="FaceProcessProfile"
            )
            self.face_process.start()
        else:
            # ResultWorker가 Face READY를 영원히 기다리지 않도록 처리.
            self.face_result_queue.put({
                "type": "FACE_READY",
                "success": True,
                "disabled": True,
                "profile_mode": self.profile_mode
            })
            print("[VisionProcessManager:PROFILE] Face Process 비활성")

    def start_calibration(self):
        self.accept_frames = True

        if self.enable_pose:
            self.pose_command_queue.put("START_CALIBRATION")

        if self.enable_face:
            self.face_command_queue.put("START_CALIBRATION")

        if self.profile_mode != "BOTH":
            print(
                "[VisionProcessManager:PROFILE] ONLY 모드 Calibration은 "
                "성능 비교용으로 권장하지 않습니다. 기존 baseline으로 MEASURING 하세요."
            )

    def start_measurement(self):
        self.accept_frames = True

        if self.enable_pose:
            self.pose_command_queue.put("START_MEASUREMENT")

        if self.enable_face:
            self.face_command_queue.put("START_MEASUREMENT")

        print(f"[VisionProcessManager:PROFILE] Measurement 시작 ({self.profile_mode})")

    def start_analysis(self):
        self.start_measurement()

    def stop_analysis(self):
        self.accept_frames = False

        if self.enable_pose:
            self.pose_command_queue.put("STOP")

        if self.enable_face:
            self.face_command_queue.put("STOP")

    def write_frame(self, frame, frame_id, timestamp_ns):
        if not self.accept_frames:
            return True, True

        # 비활성 Process Ring에는 절대 write하지 않는다.
        # 그래야 소비자가 없는 Ring이 32칸 차서 CameraWorker를 막지 않는다.
        pose_success = True
        face_success = True

        if self.enable_pose:
            pose_success = self.pose_writer.write(frame, frame_id, timestamp_ns)

        if self.enable_face:
            face_success = self.face_writer.write(frame, frame_id, timestamp_ns)

        return pose_success, face_success

    def get_stats(self):
        pose_written = self.pose_resources["written_count"].value
        pose_read = self.pose_resources["read_count"].value
        face_written = self.face_resources["written_count"].value
        face_read = self.face_resources["read_count"].value

        return {
            "profile_mode": self.profile_mode,
            "pose_enabled": self.enable_pose,
            "face_enabled": self.enable_face,
            "pose_written": pose_written,
            "pose_read": pose_read,
            "pose_pending": pose_written - pose_read,
            "pose_overrun": self.pose_resources["overrun_count"].value,
            "face_written": face_written,
            "face_read": face_read,
            "face_pending": face_written - face_read,
            "face_overrun": self.face_resources["overrun_count"].value
        }

    def stop(self):
        self.accept_frames = False
        self.stop_event.set()

        if self.enable_pose:
            self.pose_command_queue.put("SHUTDOWN")

        if self.enable_face:
            self.face_command_queue.put("SHUTDOWN")

        self._stop_process(self.pose_process)
        self._stop_process(self.face_process)

        self.pose_writer.close()
        self.face_writer.close()

        self.pose_shm.close()
        self.face_shm.close()

        try:
            self.pose_shm.unlink()
        except FileNotFoundError:
            pass

        try:
            self.face_shm.unlink()
        except FileNotFoundError:
            pass

        print(f"[VisionProcessManager:PROFILE] 종료 ({self.profile_mode})")

    @staticmethod
    def _stop_process(process):
        if process is None:
            return

        process.join(timeout=5)

        if process.is_alive():
            process.terminate()
            process.join(timeout=2)
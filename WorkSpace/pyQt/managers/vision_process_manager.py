import multiprocessing as mp
import numpy as np

from ipc.shared_frame_ring import create_ring_resources, SharedFrameWriter
# from processes.face_process import run_face_process
from processes.pose_process import run_pose_process


class VisionProcessManager:
    def __init__(self, frame_shape, slot_count=32):
        self.ctx = mp.get_context("spawn")

        self.frame_shape = tuple(frame_shape)
        self.slot_count = slot_count
        self.accept_frames = False

        self.stop_event = self.ctx.Event()

        self.pose_command_queue = self.ctx.Queue()
        # self.face_command_queue = self.ctx.Queue()

        self.pose_result_queue = self.ctx.Queue()
        # self.face_result_queue = self.ctx.Queue()

        self.pose_shm, self.pose_resources = create_ring_resources(
            ctx=self.ctx,
            slot_count=self.slot_count,
            frame_shape=self.frame_shape,
            frame_dtype=np.uint8
        )

        # self.face_shm, self.face_resources = create_ring_resources(
        #     ctx=self.ctx,
        #     slot_count=self.slot_count,
        #     frame_shape=self.frame_shape,
        #     frame_dtype=np.uint8
        # )

        self.pose_writer = SharedFrameWriter(self.pose_resources)
        # self.face_writer = SharedFrameWriter(self.face_resources)

        self.pose_process = None
        # self.face_process = None

    def start(self):
        self.pose_process = self.ctx.Process(
            target=run_pose_process,
            args=(self.stop_event, self.pose_command_queue, self.pose_result_queue, self.pose_resources),
            name="PoseProcess"
        )

        # self.face_process = self.ctx.Process(
        #     target=run_face_process,
        #     args=(self.stop_event, self.face_command_queue, self.face_result_queue, self.face_resources),
        #     name="FaceProcess"
        # )

        self.pose_process.start()
        # self.face_process.start()

    def start_calibration(self):
        self.accept_frames = True
        self.pose_command_queue.put("START_CALIBRATION")
        # self.face_command_queue.put("START_CALIBRATION")

    def start_measurement(self):
        self.accept_frames = True
        self.pose_command_queue.put("START_MEASUREMENT")
        # self.face_command_queue.put("START_MEASUREMENT")

    # 기존 테스트 코드 호환
    def start_analysis(self):
        self.start_measurement()

    def stop_analysis(self):
        self.accept_frames = False
        self.pose_command_queue.put("STOP")
        # self.face_command_queue.put("STOP")

    def write_frame(self, frame, frame_id, timestamp_ns):
        if not self.accept_frames:
            return True, True

        pose_success = self.pose_writer.write(frame, frame_id, timestamp_ns)
        # face_success = self.face_writer.write(frame, frame_id, timestamp_ns)

        # return pose_success, face_success
        return pose_success

    def get_stats(self):
        pose_written = self.pose_resources["written_count"].value
        pose_read = self.pose_resources["read_count"].value

        # face_written = self.face_resources["written_count"].value
        # face_read = self.face_resources["read_count"].value

        return {
            "pose_written": pose_written,
            "pose_read": pose_read,
            "pose_pending": pose_written - pose_read,
            "pose_overrun": self.pose_resources["overrun_count"].value,

            # "face_written": face_written,
            # "face_read": face_read,
            # "face_pending": face_written - face_read,
            # "face_overrun": self.face_resources["overrun_count"].value
        }

    def stop(self):
        self.accept_frames = False
        self.stop_event.set()

        self.pose_command_queue.put("SHUTDOWN")
        # self.face_command_queue.put("SHUTDOWN")

        self._stop_process(self.pose_process)
        # self._stop_process(self.face_process)

        self.pose_writer.close()
        # self.face_writer.close()

        self.pose_shm.close()
        # self.face_shm.close()

        self.pose_shm.unlink()
        # self.face_shm.unlink()

    @staticmethod
    def _stop_process(process):
        if process is None:
            return

        process.join(timeout=5)

        if process.is_alive():
            process.terminate()
            process.join()
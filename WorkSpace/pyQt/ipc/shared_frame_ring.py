import numpy as np
from multiprocessing import shared_memory


def create_ring_resources(ctx, slot_count, frame_shape, frame_dtype=np.uint8):
    frame_dtype = np.dtype(frame_dtype)
    total_shape = (slot_count, *frame_shape)
    total_size = int(np.prod(total_shape) * frame_dtype.itemsize)

    shm = shared_memory.SharedMemory(create=True, size=total_size)

    resources = {
        "shm_name": shm.name,
        "slot_count": slot_count,
        "frame_shape": tuple(frame_shape),
        "frame_dtype": frame_dtype.name,

        "write_index": ctx.Value("i", 0),
        "read_index": ctx.Value("i", 0),

        "free_slots": ctx.Semaphore(slot_count),
        "filled_slots": ctx.Semaphore(0),

        "frame_ids": ctx.Array("q", slot_count, lock=False),
        "timestamps": ctx.Array("q", slot_count, lock=False),

        "written_count": ctx.Value("q", 0),
        "read_count": ctx.Value("q", 0),
        "skipped_count": ctx.Value("q", 0),
        "overrun_count": ctx.Value("q", 0),
    }

    return shm, resources


class SharedFrameWriter:
    def __init__(self, resources):
        self.resources = resources
        self.slot_count = resources["slot_count"]
        self.frame_shape = tuple(resources["frame_shape"])
        self.frame_dtype = np.dtype(resources["frame_dtype"])

        self.shm = shared_memory.SharedMemory(name=resources["shm_name"])
        self.frames = np.ndarray(
            (self.slot_count, *self.frame_shape),
            dtype=self.frame_dtype,
            buffer=self.shm.buf,
        )

    def write(self, frame, frame_id, timestamp_ns):
        """Camera thread를 막지 않고 frame을 기록한다.

        Vision Process가 밀려 ring이 꽉 찬 경우 현재 frame을 버린다.
        오래된 frame 때문에 카메라/UI가 지연되는 것보다 실시간성을 우선한다.
        """
        if frame.shape != self.frame_shape:
            raise ValueError(f"Frame Shape Error : {frame.shape} != {self.frame_shape}")

        if frame.dtype != self.frame_dtype:
            raise ValueError(f"Frame DType Error : {frame.dtype} != {self.frame_dtype}")

        if not self.resources["free_slots"].acquire(block=False):
            with self.resources["overrun_count"].get_lock():
                self.resources["overrun_count"].value += 1
            return False

        write_index = self.resources["write_index"].value

        np.copyto(self.frames[write_index], frame)
        self.resources["frame_ids"][write_index] = frame_id
        self.resources["timestamps"][write_index] = timestamp_ns
        self.resources["write_index"].value = (write_index + 1) % self.slot_count

        with self.resources["written_count"].get_lock():
            self.resources["written_count"].value += 1

        self.resources["filled_slots"].release()
        return True

    def close(self):
        self.shm.close()


class SharedFrameReader:
    def __init__(self, resources):
        self.resources = resources
        self.slot_count = resources["slot_count"]
        self.frame_shape = tuple(resources["frame_shape"])
        self.frame_dtype = np.dtype(resources["frame_dtype"])

        self.shm = shared_memory.SharedMemory(name=resources["shm_name"])
        self.frames = np.ndarray(
            (self.slot_count, *self.frame_shape),
            dtype=self.frame_dtype,
            buffer=self.shm.buf,
        )

    def read_latest(self, timeout=0.1):
        """대기 중인 frame 중 가장 최신 frame만 반환한다.

        Pose/Face가 카메라보다 느릴 때 과거 frame을 순서대로 처리하지 않고,
        이미 쌓여 있는 오래된 frame은 건너뛰어 실시간 지연 누적을 막는다.
        """
        if not self.resources["filled_slots"].acquire(timeout=timeout):
            return None

        read_index = self.resources["read_index"].value
        consumed = 1

        while self.resources["filled_slots"].acquire(block=False):
            consumed += 1

        latest_index = (read_index + consumed - 1) % self.slot_count
        frame_id = int(self.resources["frame_ids"][latest_index])
        timestamp_ns = int(self.resources["timestamps"][latest_index])

        # free slot을 반환하기 전에 Process 로컬 메모리로 복사한다.
        frame = self.frames[latest_index].copy()

        self.resources["read_index"].value = (read_index + consumed) % self.slot_count

        with self.resources["read_count"].get_lock():
            self.resources["read_count"].value += consumed

        skipped = consumed - 1
        if skipped > 0:
            with self.resources["skipped_count"].get_lock():
                self.resources["skipped_count"].value += skipped

        for _ in range(consumed):
            self.resources["free_slots"].release()

        return frame, frame_id, timestamp_ns

    # 기존 코드 호환용. 새 코드에서는 read_latest() 사용.
    def read(self, timeout=0.1):
        return self.read_latest(timeout=timeout)

    def close(self):
        self.shm.close()

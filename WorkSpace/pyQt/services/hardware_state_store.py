import copy
import threading


class HardwareRuntimeStateStore:
    """Main Process에서 Hardware 최신 상태를 공용으로 꺼내 쓰기 위한 저장소.

    JSON에 기록하는 설정/Calibration과 달리 IR/IMU/Motor 실시간 값은 메모리에만 둔다.
    Hardware Process -> Main 최신 State Queue(maxsize=1)에서 받은 값을 이 객체에 갱신한다.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._hardware_state = {}

    def update(self, state):
        if not isinstance(state, dict):
            return
        with self._lock:
            self._hardware_state = copy.deepcopy(state)

    def clear(self):
        with self._lock:
            self._hardware_state = {}

    def get_hardware_state(self):
        with self._lock:
            return copy.deepcopy(self._hardware_state)

    def get_ir_state(self):
        with self._lock:
            value = self._hardware_state.get("ir", {})
            return copy.deepcopy(value) if isinstance(value, dict) else {}

    def get_imu_state(self):
        with self._lock:
            value = self._hardware_state.get("imu", {})
            return copy.deepcopy(value) if isinstance(value, dict) else {}

    def get_motor_state(self):
        with self._lock:
            value = self._hardware_state.get("motor", {})
            return copy.deepcopy(value) if isinstance(value, dict) else {}

    def get_gimbal_state(self):
        with self._lock:
            value = self._hardware_state.get("gimbal", {})
            return copy.deepcopy(value) if isinstance(value, dict) else {}

    def is_ir_detected(self, require_stable=False):
        ir = self.get_ir_state()
        key = "stable_detected" if require_stable else "detected"
        return bool(ir.get("available", False) and ir.get(key, False))

# Main Process 내부 UI/알림/리포트 모듈들이 동일 최신 상태를 공유하기 위한 singleton.
# multiprocessing child에서는 spawn 시 별도 메모리가 되므로 이 singleton을 IPC 대용으로 쓰지 않는다.
_MAIN_PROCESS_HARDWARE_STATE_STORE = HardwareRuntimeStateStore()


def get_hardware_runtime_state_store():
    return _MAIN_PROCESS_HARDWARE_STATE_STORE

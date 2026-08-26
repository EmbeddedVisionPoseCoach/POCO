"""Hardware IPC facade.

실제 Serial / Sensor / Motor / Alert 로직은 없다.
필요하면 Main Process에서 Hardware Process로 State 또는 Event/Command를 보내기 위한
얇은 IPC 인터페이스로만 사용한다.

현재 active 실행 경로에서는 VisionProcessManager를 직접 사용하므로 필수 클래스는 아니다.
"""

from ipc.queue_utils import get_latest, drain_ordered, put_latest, put_ordered


class HardwareController:
    def __init__(
        self,
        main_to_hw_state_queue=None,
        main_to_hw_event_queue=None,
        hw_to_main_state_queue=None,
        hw_to_main_event_queue=None,
        **_,
    ):
        self.main_to_hw_state_queue = main_to_hw_state_queue
        self.main_to_hw_event_queue = main_to_hw_event_queue
        self.hw_to_main_state_queue = hw_to_main_state_queue
        self.hw_to_main_event_queue = hw_to_main_event_queue

    def bind(
        self,
        main_to_hw_state_queue,
        main_to_hw_event_queue,
        hw_to_main_state_queue,
        hw_to_main_event_queue,
    ):
        self.main_to_hw_state_queue = main_to_hw_state_queue
        self.main_to_hw_event_queue = main_to_hw_event_queue
        self.hw_to_main_state_queue = hw_to_main_state_queue
        self.hw_to_main_event_queue = hw_to_main_event_queue

    def send_state(self, state):
        if self.main_to_hw_state_queue is None:
            return False
        return put_latest(self.main_to_hw_state_queue, state)

    def send_command(self, command):
        if self.main_to_hw_event_queue is None:
            return False
        return put_ordered(self.main_to_hw_event_queue, command)

    def get_latest_state(self, default=None):
        if self.hw_to_main_state_queue is None:
            return default
        return get_latest(self.hw_to_main_state_queue, default)

    def get_events(self):
        if self.hw_to_main_event_queue is None:
            return []
        return drain_ordered(self.hw_to_main_event_queue)

    # Legacy name compatibility. Command는 latest queue가 아니라 ordered event queue로 보낸다.
    def send(self, message):
        return self.send_command(message)

    def close(self):
        pass

import time

from ipc.queue_utils import get_latest, drain_ordered, put_latest, put_ordered


HARDWARE_STATUS_INTERVAL_SEC = 1.0


def _event_type(message):
    if isinstance(message, dict):
        return str(message.get("type", "UNKNOWN"))
    return str(message)


def run_hardware_process(
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
    """Hardware Process IPC 골격.

    현재 실제 IR / IMU / Motor / Serial 로직은 없다.
    Main / Pose / Face와의 양방향 통신 인터페이스만 유지한다.
    비활성 Profile의 런타임 IPC는 완전히 건너뛴다.

    State Queue
      - maxsize=1
      - landmark / feature / sensor / 현재 mode처럼 최신값만 중요한 데이터

    Event Queue
      - 순서대로 모두 소비
      - START / STOP / DONE / ERROR / ACK처럼 유실되면 안 되는 메시지
    """
    print(
        f"[HardwareProcess] 시작 - IPC ONLY "
        f"Pose={'ON' if enable_pose else 'OFF'} "
        f"Face={'ON' if enable_face else 'OFF'}"
    )

    latest_main_state = None
    latest_pose_state = None
    latest_face_state = None
    last_status_time = 0.0

    ready_event = {
        "type": "HARDWARE_READY",
        "ready": True,
        "content_ready": False,
        "message": "Hardware Process IPC 준비 완료",
        "timestamp": time.time(),
    }

    put_ordered(hw_to_main_event_queue, ready_event)
    if enable_pose:
        put_ordered(hw_to_pose_event_queue, ready_event)
    if enable_face:
        put_ordered(hw_to_face_event_queue, ready_event)

    try:
        while not stop_event.is_set():
            # -------------------------------------------------
            # 최신 State 수신
            # -------------------------------------------------
            latest_main_state = get_latest(main_to_hw_state_queue, latest_main_state)

            if enable_pose:
                latest_pose_state = get_latest(pose_to_hw_state_queue, latest_pose_state)

            if enable_face:
                latest_face_state = get_latest(face_to_hw_state_queue, latest_face_state)

            # -------------------------------------------------
            # 순서가 중요한 Event/Command 수신
            # 아직 Hardware content가 없으므로 ACK만 반환한다.
            # -------------------------------------------------
            for event in drain_ordered(main_to_hw_event_queue):
                put_ordered(hw_to_main_event_queue, {
                    "type": "HARDWARE_EVENT_ACK",
                    "source": "MAIN",
                    "received_type": _event_type(event),
                    "timestamp": time.time(),
                })

            if enable_pose:
                for event in drain_ordered(pose_to_hw_event_queue):
                    put_ordered(hw_to_pose_event_queue, {
                        "type": "HARDWARE_EVENT_ACK",
                        "source": "POSE",
                        "received_type": _event_type(event),
                        "timestamp": time.time(),
                    })

            if enable_face:
                for event in drain_ordered(face_to_hw_event_queue):
                    put_ordered(hw_to_face_event_queue, {
                        "type": "HARDWARE_EVENT_ACK",
                        "source": "FACE",
                        "received_type": _event_type(event),
                        "timestamp": time.time(),
                    })

            # -------------------------------------------------
            # Hardware 최신 상태 Broadcast
            # 실제 센서/모터가 추가되면 state에 값을 추가한다.
            # -------------------------------------------------
            now = time.monotonic()
            if now - last_status_time >= HARDWARE_STATUS_INTERVAL_SEC:
                last_status_time = now

                state = {
                    "type": "HARDWARE_STATE",
                    "ready": True,
                    "content_ready": False,
                    "timestamp": time.time(),
                    "main_state_received": latest_main_state is not None,
                    "pose_enabled": bool(enable_pose),
                    "pose_state_received": (latest_pose_state is not None) if enable_pose else False,
                    "face_enabled": bool(enable_face),
                    "face_state_received": (latest_face_state is not None) if enable_face else False,
                }

                put_latest(hw_to_main_state_queue, state)

                if enable_pose:
                    put_latest(hw_to_pose_state_queue, state)

                if enable_face:
                    put_latest(hw_to_face_state_queue, state)

            time.sleep(0.005)

    finally:
        stopped_event = {
            "type": "HARDWARE_STOPPED",
            "ready": False,
            "content_ready": False,
            "timestamp": time.time(),
        }
        put_ordered(hw_to_main_event_queue, stopped_event)
        print("[HardwareProcess] 종료")

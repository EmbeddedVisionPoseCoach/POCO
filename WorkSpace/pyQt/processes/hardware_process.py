import time

from ipc.queue_utils import drain_ordered, get_latest, put_latest, put_ordered
from services.hardware_config_service import HardwareConfigService
from services.imu_service import ADXL345IMUService
from services.motor_service import MotorService
from services.motor12_controller import Motor12Controller
from services.motor34_controller import Motor34Controller


HARDWARE_STATUS_INTERVAL_SEC = 0.05
HARDWARE_LOOP_SLEEP_SEC = 0.002

HW_IDLE = "IDLE"
HW_IMU_OFFSET_CALIBRATING = "IMU_OFFSET_CALIBRATING"
HW_READY_FOR_POSE_CALIBRATION = "READY_FOR_POSE_CALIBRATION"
HW_POSE_CALIBRATING = "POSE_CALIBRATING"
HW_MEASURING = "MEASURING"


def _event_type(message):
    if isinstance(message, dict):
        return str(message.get("type", "UNKNOWN")).upper()
    return str(message).upper()


def _put_hardware_event(queue, event_type, success=None, message="", **extra):
    event = {
        "type": event_type,
        "message": message,
        "timestamp": time.time(),
        **extra,
    }
    if success is not None:
        event["success"] = bool(success)
    put_ordered(queue, event)
    return event


def _extract_pose_landmark_state(pose_state):
    if not isinstance(pose_state, dict):
        return None, None, False

    frame_id = pose_state.get("frame_id")
    valid = bool(pose_state.get("landmark_valid", False))
    landmarks = pose_state.get("landmarks") if valid else None
    return frame_id, landmarks, valid


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
    """Hardware Process.

    핵심 원칙
    ---------
    1. 실제 I2C/Serial은 이 Process만 소유한다.
    2. MotorController/Serial 포트는 MotorService 하나만 생성한다.
    3. Motor1 -> Motor2 -> Motor3 -> Motor4 순서로 Controller update를 호출한다.
    4. 1/2번 로직은 motor12_controller.py만 수정하면 된다.
    5. 3/4번 로직은 motor34_controller.py에 있으며 Motor3=IMU Y, Motor4=IMU X Direct PID로 제어한다.
    6. 추가 센서/상태도 Queue를 새로 만들지 않고 HARDWARE_STATE dict에 추가한다.
    """
    print(
        f"[HardwareProcess] 시작 - IMU + MOTOR1~4 "
        f"Pose={'ON' if enable_pose else 'OFF'} "
        f"Face={'ON' if enable_face else 'OFF'}"
    )

    # ========================================================
    # 1. Config / Hardware 객체 생성
    # ========================================================
    config_service = HardwareConfigService()
    config_data = config_service.load()
    control = config_data["control"]

    # 생성자는 JSON이 아니라 코드 상수 기본값으로 초기화된다.
    imu = ADXL345IMUService()
    motor_service = MotorService()

    # 실제 제어 로직은 별도 Controller로 분리한다.
    motor12 = Motor12Controller(motor_service)
    motor34 = Motor34Controller(motor_service)

    # JSON에는 사용자가 바꿀 튜닝값만 있다.
    imu.apply_control_config(control["imu"], control["pid"])
    motor34.apply_control_config(control["motor"])

    imu_opened = imu.open()
    motor_bus_opened = motor_service.open()
    motor34_opened = bool(
        motor_bus_opened
        and (motor34.motor3_config_enabled or motor34.motor4_config_enabled)
        and motor34.initialize()
    )

    latest_imu_state = dict(imu.latest_state)
    latest_motor12_state = dict(motor12.latest_state)
    latest_motor34_state = dict(motor34.latest_state)

    # ========================================================
    # 2. IPC 최신 상태
    # ========================================================
    latest_main_state = None
    latest_pose_state = None
    latest_face_state = None
    latest_pose_frame_id = None
    latest_pose_landmarks = None
    latest_pose_landmark_valid = False

    # ========================================================
    # 3. Calibration / Runtime 상태
    # ========================================================
    workflow_state = HW_IDLE
    calibration_generation = 0
    calibration_ready_emitted = False

    last_imu_sample_time = 0.0
    last_status_time = 0.0

    def apply_runtime_config(new_config):
        """PyQt UPDATE_CONFIG -> 현재 Runtime 객체에 즉시 반영."""
        nonlocal config_data, control, imu_opened, motor_bus_opened, motor34_opened

        config_data = new_config
        control = config_data["control"]

        imu_ok = imu.apply_control_config(control["imu"], control["pid"])
        if imu_ok and not imu.available:
            imu_ok = imu.open()
        imu_opened = bool(imu.available and imu_ok)

        motor34.apply_control_config(control["motor"])

        if not motor_service.available:
            motor_bus_opened = motor_service.open()
        else:
            motor_bus_opened = True

        if motor_bus_opened and (motor34.motor3_config_enabled or motor34.motor4_config_enabled):
            motor34_opened = motor34.initialize()
        else:
            motor34_opened = False

        return bool(imu_opened and motor34_opened)

    def build_ready_event():
        all_ready = bool(imu_opened and motor34_opened and motor34.ready)
        errors = []
        if not imu_opened:
            errors.append(f"IMU 실패: {imu.last_error}")
        if not motor_bus_opened:
            errors.append(f"Motor Bus 실패: {motor_service.last_error}")
        elif not motor34_opened:
            errors.append(f"Motor3/4 실패: {motor34.last_error}")
        if not motor34.motor3_config_enabled:
            errors.append("Motor3 Disabled")
        if not motor34.motor4_config_enabled:
            errors.append("Motor4 Disabled")

        return {
            "type": "HARDWARE_READY",
            "ready": True,
            "content_ready": all_ready,
            "imu_ready": bool(imu_opened),
            "motor_ready": bool(motor34_opened and motor34.ready),
            "message": (
                "Hardware Process IMU/Motor3/4 준비 완료"
                if all_ready
                else "Hardware 일부 준비 실패 / " + " / ".join(errors)
            ),
            "config_path": str(config_service.path),
            "timestamp": time.time(),
        }

    ready_event = build_ready_event()
    put_ordered(hw_to_main_event_queue, ready_event)
    if enable_pose:
        put_ordered(hw_to_pose_event_queue, ready_event)
    if enable_face:
        put_ordered(hw_to_face_event_queue, ready_event)

    try:
        while not stop_event.is_set():
            # ==================================================
            # A. Process 간 최신 State 받기
            # ==================================================
            latest_main_state = get_latest(main_to_hw_state_queue, latest_main_state)

            if enable_pose:
                latest_pose_state = get_latest(pose_to_hw_state_queue, latest_pose_state)
                (
                    latest_pose_frame_id,
                    latest_pose_landmarks,
                    latest_pose_landmark_valid,
                ) = _extract_pose_landmark_state(latest_pose_state)

            if enable_face:
                latest_face_state = get_latest(face_to_hw_state_queue, latest_face_state)

            main_mode = ""
            if isinstance(latest_main_state, dict):
                main_mode = str(latest_main_state.get("state", "")).upper()

            # ==================================================
            # B. Main -> Hardware 명령 처리
            # ==================================================
            for event in drain_ordered(main_to_hw_event_queue):
                event_type = _event_type(event)

                if event_type == "PREPARE_CALIBRATION":
                    calibration_generation += 1
                    calibration_ready_emitted = False

                    if imu.calibrating:
                        imu.cancel_calibration()

                    missing = []
                    if not imu_opened:
                        missing.append("IMU")
                    if not motor34_opened or not motor34.ready:
                        missing.append("Motor3/4")

                    if missing:
                        workflow_state = HW_IDLE
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CALIBRATION_READY",
                            success=False,
                            message="Calibration 준비 실패: " + ", ".join(missing),
                            generation=calibration_generation,
                            workflow_state=workflow_state,
                        )
                        continue

                    success = imu.start_calibration()
                    latest_imu_state = dict(imu.latest_state)

                    if success:
                        workflow_state = HW_IMU_OFFSET_CALIBRATING
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "IMU_OFFSET_CALIBRATION_STARTED",
                            success=True,
                            message=f"IMU X/Y 기준값 측정 시작 ({imu.calibration_sec:.1f}초)",
                            generation=calibration_generation,
                            workflow_state=workflow_state,
                        )
                        print("[HardwareProcess] IMU X/Y 기준값 Calibration 시작")
                    else:
                        workflow_state = HW_IDLE
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CALIBRATION_READY",
                            success=False,
                            message="IMU X/Y 기준값 Calibration을 시작하지 못했습니다.",
                            generation=calibration_generation,
                            workflow_state=workflow_state,
                        )
                    continue

                if event_type == "CANCEL_CALIBRATION_PREPARE":
                    if imu.calibrating:
                        imu.cancel_calibration()
                    workflow_state = HW_IDLE
                    calibration_ready_emitted = False
                    continue

                if event_type == "IMU_CALIBRATE":
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "IMU_CALIBRATION_REJECTED",
                        success=False,
                        message="PREPARE_CALIBRATION 경로를 사용해주세요.",
                    )
                    continue

                if event_type == "UPDATE_CONFIG":
                    patch = event.get("control", event.get("data", {}))
                    try:
                        updated = config_service.update_control(patch)
                        apply_runtime_config(updated)
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_UPDATED",
                            success=True,
                            message="Hardware 튜닝 설정을 저장하고 즉시 반영했습니다.",
                            config=updated,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_UPDATED",
                            success=False,
                            message=f"Hardware 설정 저장 실패: {error}",
                        )
                    continue

                if event_type == "RELOAD_CONFIG":
                    try:
                        updated = config_service.reload()
                        apply_runtime_config(updated)
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_RELOADED",
                            success=True,
                            message="hardware_control.json을 다시 읽어 반영했습니다.",
                            config=updated,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_RELOADED",
                            success=False,
                            message=f"Hardware 설정 Reload 실패: {error}",
                        )
                    continue

                if event_type == "GET_CONFIG":
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "HARDWARE_CONFIG_STATE",
                        success=True,
                        config=config_service.snapshot(),
                    )
                    continue

                if event_type == "RESET_CONTROL_CONFIG":
                    try:
                        updated = config_service.reset_defaults(preserve_imu_calibration=True)
                        apply_runtime_config(updated)
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_RESET",
                            success=True,
                            message="Hardware 튜닝 설정을 기본값으로 복원했습니다.",
                            config=updated,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_RESET",
                            success=False,
                            message=f"Hardware 설정 초기화 실패: {error}",
                        )
                    continue

                if event_type == "MOTOR_ENABLE":
                    motor34.set_enabled(True)
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR_ENABLE_ACK",
                        success=True,
                        message="Motor3/4 runtime 제어 허용",
                    )
                    continue

                if event_type == "MOTOR_DISABLE":
                    motor34.set_enabled(False)
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR_DISABLE_ACK",
                        success=True,
                        message="Motor3/4 runtime 제어 차단",
                    )
                    continue

                _put_hardware_event(
                    hw_to_main_event_queue,
                    "HARDWARE_EVENT_ACK",
                    source="MAIN",
                    received_type=event_type,
                )

            # 현재 Pose/Face Event는 실제 제어에 쓰지 않고 ACK만 유지한다.
            if enable_pose:
                for event in drain_ordered(pose_to_hw_event_queue):
                    put_ordered(
                        hw_to_pose_event_queue,
                        {
                            "type": "HARDWARE_EVENT_ACK",
                            "source": "POSE",
                            "received_type": _event_type(event),
                            "timestamp": time.time(),
                        },
                    )

            if enable_face:
                for event in drain_ordered(face_to_hw_event_queue):
                    put_ordered(
                        hw_to_face_event_queue,
                        {
                            "type": "HARDWARE_EVENT_ACK",
                            "source": "FACE",
                            "received_type": _event_type(event),
                            "timestamp": time.time(),
                        },
                    )

            # ==================================================
            # C. Sensor sampling
            # ==================================================
            now = time.monotonic()


            if now - last_imu_sample_time >= imu.sample_interval:
                last_imu_sample_time = now
                latest_imu_state = imu.update()

                calibration_result = imu.consume_calibration_result()
                if calibration_result is not None:
                    try:
                        config_data = config_service.update_imu_calibration(
                            calibration_result["x_reference_g"],
                            calibration_result["y_reference_g"],
                            calibration_result["sample_count"],
                            x_reference_raw=calibration_result.get("x_reference_raw", 0.0),
                            y_reference_raw=calibration_result.get("y_reference_raw", 0.0),
                        )
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "IMU_OFFSET_SAVED",
                            success=True,
                            message="IMU X/Y 기준값을 hardware_control.json에 기록했습니다.",
                            calibration=config_data["calibration"]["imu"],
                            config=config_data,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "IMU_OFFSET_SAVED",
                            success=False,
                            message=f"IMU Offset JSON 저장 실패: {error}",
                        )

            # ==================================================
            # D. Calibration State Machine: IMU X/Y Reference
            # ==================================================
            if workflow_state == HW_IMU_OFFSET_CALIBRATING:
                # 실제 IMU Service 세션 상태를 기준으로 READY를 판정한다.
                if imu.calibrated and not imu.calibrating and not calibration_ready_emitted:
                    calibration_ready_emitted = True
                    workflow_state = HW_READY_FOR_POSE_CALIBRATION
                    latest_imu_state = dict(imu.latest_state)
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "HARDWARE_CALIBRATION_READY",
                        success=True,
                        message=(
                            "IMU X/Y 기준값 Calibration 완료. "
                            "Pose Calibration + Motor3/4 짐벌 제어를 시작할 수 있습니다."
                        ),
                        generation=calibration_generation,
                        workflow_state=workflow_state,
                        imu_x_reference_g=latest_imu_state.get("imu_x_reference_g"),
                        imu_y_reference_g=latest_imu_state.get("imu_y_reference_g"),
                        # 기존 UI 호환 진단값
                        pitch_offset_deg=latest_imu_state.get("pitch_offset_deg"),
                        roll_offset_deg=latest_imu_state.get("roll_offset_deg"),
                    )
                    print("[HardwareProcess] IMU X/Y Calibration 완료 -> Pose Calibration READY")

            # ==================================================
            # E. Main mode -> Hardware workflow
            # ==================================================
            if main_mode == "CALIBRATING" and imu.calibrated:
                workflow_state = HW_POSE_CALIBRATING
            elif main_mode == "MEASURING" and imu.calibrated:
                workflow_state = HW_MEASURING
            elif main_mode in ("PREVIEW", "IDLE", "CAMERA_OFF"):
                if workflow_state in (HW_POSE_CALIBRATING, HW_MEASURING):
                    workflow_state = (
                        HW_READY_FOR_POSE_CALIBRATION if imu.calibrated else HW_IDLE
                    )

            # ==================================================
            # F. Motor control context
            # ==================================================
            imu_ready = bool(
                imu.available and imu.calibrated and not imu.calibrating
            )
            motor34_requested = main_mode in ("CALIBRATING", "MEASURING")
            motor34_active = bool(
                motor34_requested
                and imu_ready
                and motor34.ready
            )

            context = {
                "now": now,
                "main_mode": main_mode,
                "workflow_state": workflow_state,
                "imu": latest_imu_state,
                "pose": latest_pose_state,
                "face": latest_face_state,
                "motor34_control_active": motor34_active,
            }

            # 실행 순서가 코드에 그대로 보이도록 유지한다.
            latest_motor12_state = motor12.update(context)   # Motor 1 -> 2
            latest_motor34_state = motor34.update(context)   # Motor 3 -> 4

            # MediaPipe landmark는 현재 수신만 한다.
            if latest_pose_landmark_valid:
                pass

            # 기존 Main 코드와 호환되는 motor3/motor4 key를 유지하면서
            # motor1/2와 공통 bus 정보도 함께 제공한다.
            latest_motor_state = dict(latest_motor34_state)
            latest_motor_state["bus"] = motor_service.get_state()
            latest_motor_state["motor12"] = dict(latest_motor12_state)
            latest_motor_state["motor1"] = dict(latest_motor12_state["motor1"])
            latest_motor_state["motor2"] = dict(latest_motor12_state["motor2"])

            # ==================================================
            # G. Hardware -> Main/Pose/Face 최신 State (20Hz)
            # ==================================================
            if now - last_status_time >= HARDWARE_STATUS_INTERVAL_SEC:
                last_status_time = now

                state = {
                    "type": "HARDWARE_STATE",
                    "ready": True,
                    "content_ready": bool(imu_opened and motor34.ready),
                    "timestamp": time.time(),
                    "workflow_state": workflow_state,
                    "main_mode": main_mode,
                    "main_state_received": latest_main_state is not None,
                    "pose_enabled": bool(enable_pose),
                    "pose_state_received": latest_pose_state is not None if enable_pose else False,
                    "pose_frame_id": latest_pose_frame_id if enable_pose else None,
                    "pose_landmark_valid": latest_pose_landmark_valid if enable_pose else False,
                    "pose_landmark_count": (
                        len(latest_pose_landmarks)
                        if enable_pose and latest_pose_landmarks is not None
                        else 0
                    ),
                    "face_enabled": bool(enable_face),
                    "face_state_received": latest_face_state is not None if enable_face else False,
                    "imu": dict(latest_imu_state),
                    "motor": latest_motor_state,
                    "gimbal": {
                        "requested": motor34_requested,
                        "active": motor34_active,
                        "imu_ready": imu_ready,
                        "motor3_axis": "imu_y",
                        "motor4_axis": "imu_x",
                        "motor3_packet_suppressed_at_zero": True,
                        "motor4_packet_suppressed_at_zero": True,
                    },
                }

                put_latest(hw_to_main_state_queue, state)
                if enable_pose:
                    put_latest(hw_to_pose_state_queue, state)
                if enable_face:
                    put_latest(hw_to_face_state_queue, state)

            time.sleep(HARDWARE_LOOP_SLEEP_SEC)

    finally:
        motor34.close()
        motor_service.close()
        imu.close()

        put_ordered(
            hw_to_main_event_queue,
            {
                "type": "HARDWARE_STOPPED",
                "ready": False,
                "content_ready": False,
                "timestamp": time.time(),
            },
        )
        print("[HardwareProcess] 종료")

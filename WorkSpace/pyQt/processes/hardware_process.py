import time

from ipc.queue_utils import get_latest, drain_ordered, put_latest, put_ordered
from services.hardware_config_service import HardwareConfigService
from services.ir_service import IRSensorService
from services.imu_service import ADXL345IMUService
from services.motor_service import MonitorMotorService


HARDWARE_STATUS_INTERVAL_SEC = 0.05
HARDWARE_LOOP_SLEEP_SEC = 0.002

HW_IDLE = "IDLE"
HW_IR_CHECK = "IR_CHECK"
HW_IMU_OFFSET_CALIBRATING = "IMU_OFFSET_CALIBRATING"
HW_READY_FOR_POSE_CALIBRATION = "READY_FOR_POSE_CALIBRATION"
HW_POSE_CALIBRATING = "POSE_CALIBRATING"
HW_MEASURING = "MEASURING"


def _event_type(message):
    if isinstance(message, dict):
        return str(message.get("type", "UNKNOWN")).upper()
    return str(message).upper()


def _extract_pose_landmark_state(pose_state):
    """Pose -> Hardware 최신 MediaPipe landmark를 저장만 한다."""
    if not isinstance(pose_state, dict):
        return None, None, False

    frame_id = pose_state.get("frame_id")
    landmark_valid = bool(pose_state.get("landmark_valid", False))
    landmarks = pose_state.get("landmarks") if landmark_valid else None
    return frame_id, landmarks, landmark_valid


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


def _create_ir_service():
    """IR Service는 JSON이 아니라 코드 상수 기본값으로 생성한다."""
    return IRSensorService()


def _create_imu_service():
    """IMU Service는 JSON이 아니라 코드 상수 기본값으로 생성한다."""
    return ADXL345IMUService()


def _create_motor_service():
    """Motor Service도 자체 기본값으로 생성 후 Config를 별도 적용한다."""
    return MonitorMotorService()


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
    """Hardware Process 최종 흐름.

    Calibration Start
      IR 안정 감지 -> IMU Offset 3초 -> Offset JSON 기록 -> Pose Calibration
      Pose Calibration/Measurement 동안 IMU Pitch PID -> Motor3 wrist_flex

    Runtime state
      IR/IMU/Motor 값은 JSON에 저장하지 않고 최신 State(maxsize=1)로 Main/Pose/Face에 배포.

    Config
      hardware_control.json은 HardwareConfigService가 관리.
      실행 중 PyQt는 UPDATE_CONFIG IPC를 보내고 Hardware Process가 저장 + 즉시 반영.

    MediaPipe
      Pose landmark는 최신값을 수신/보관만 하고 현재 제어에는 사용하지 않는다.

    Motor4
      인터페이스만 유지하고 pass.
    """
    print(
        f"[HardwareProcess] 시작 - IR + IMU + MOTOR3 "
        f"Pose={'ON' if enable_pose else 'OFF'} "
        f"Face={'ON' if enable_face else 'OFF'}"
    )

    config_service = HardwareConfigService()
    config_data = config_service.load()
    control = config_data["control"]

    latest_main_state = None
    latest_pose_state = None
    latest_face_state = None
    latest_pose_frame_id = None
    latest_pose_landmarks = None
    latest_pose_landmark_valid = False

    # 생성자는 JSON 값을 받지 않는다.
    # Service 내부 코드 상수/기본값으로 먼저 객체를 만든다.
    ir = _create_ir_service()
    imu = _create_imu_service()
    motor = _create_motor_service()

    # JSON은 생성자 입력이 아니라 저장된 Runtime tuning/override다.
    # 객체 생성과 Config 적용을 분리해 이후 PyQt UPDATE_CONFIG도 같은 경로를 사용한다.
    ir.apply_config(control["ir"])
    imu.apply_control_config(control["imu"], control["pid"])
    motor.apply_control_config(control["motor"])

    ir_opened = ir.open()
    latest_ir_state = dict(ir.latest_state)

    imu_opened = imu.open()
    latest_imu_state = dict(imu.latest_state)

    motor_opened = motor.open()
    latest_motor_state = dict(motor.latest_state)

    workflow_state = HW_IDLE
    prepare_started_at = None
    calibration_generation = 0
    calibration_ready_emitted = False

    last_ir_sample_time = 0.0
    last_imu_sample_time = 0.0
    last_status_time = 0.0
    ir_loss_notified_mode = None

    def current_ir_check_timeout():
        return float(config_data["control"]["ir"]["check_timeout_sec"])

    def apply_runtime_config(new_config):
        nonlocal config_data, control, ir_opened, imu_opened, motor_opened
        config_data = new_config
        control = config_data["control"]

        ir_ok = ir.apply_config(control["ir"])
        if ir_ok and not ir.available:
            ir_ok = ir.open()
        ir_opened = bool(ir.available and ir_ok)

        imu_ok = imu.apply_control_config(control["imu"], control["pid"])
        if imu_ok and not imu.available:
            imu_ok = imu.open()
        imu_opened = bool(imu.available and imu_ok)

        motor.apply_control_config(control["motor"])
        if motor.motor3_config_enabled and not motor.available:
            motor_opened = motor.open()
        else:
            motor_opened = bool(motor.available)

        return bool(ir_opened and imu_opened and motor_opened)

    all_ready = bool(
        ir_opened
        and imu_opened
        and motor_opened
        and motor.motor3_config_enabled
    )
    ready_parts = []
    if not ir_opened:
        ready_parts.append(f"IR 실패: {ir.last_error}")
    if not imu_opened:
        ready_parts.append(f"IMU 실패: {imu.last_error}")
    if not motor_opened:
        ready_parts.append(f"Motor3 실패: {motor.last_error}")
    if motor_opened and not motor.motor3_config_enabled:
        ready_parts.append("Motor3가 hardware_control.json에서 disabled")

    ready_event = {
        "type": "HARDWARE_READY",
        "ready": True,
        "content_ready": all_ready,
        "ir_ready": bool(ir_opened),
        "imu_ready": bool(imu_opened),
        "motor_ready": bool(motor_opened and motor.motor3_config_enabled),
        "message": (
            "Hardware Process IR/IMU/Motor3 준비 완료"
            if all_ready
            else "Hardware 일부 준비 실패 / " + " / ".join(ready_parts)
        ),
        "config_path": str(config_service.path),
        "timestamp": time.time(),
    }
    put_ordered(hw_to_main_event_queue, ready_event)
    if enable_pose:
        put_ordered(hw_to_pose_event_queue, ready_event)
    if enable_face:
        put_ordered(hw_to_face_event_queue, ready_event)

    try:
        while not stop_event.is_set():
            # ==================================================
            # Latest State input
            # ==================================================
            latest_main_state = get_latest(main_to_hw_state_queue, latest_main_state)

            if enable_pose:
                latest_pose_state = get_latest(pose_to_hw_state_queue, latest_pose_state)
                (
                    latest_pose_frame_id,
                    latest_pose_landmarks,
                    latest_pose_landmark_valid,
                ) = _extract_pose_landmark_state(latest_pose_state)

                if latest_pose_landmark_valid:
                    # Future: landmark safety condition / posture gate.
                    pass

            if enable_face:
                latest_face_state = get_latest(face_to_hw_state_queue, latest_face_state)

            main_mode = ""
            if isinstance(latest_main_state, dict):
                main_mode = str(latest_main_state.get("state", "")).upper()

            # ==================================================
            # Main -> Hardware Event / Config
            # ==================================================
            for event in drain_ordered(main_to_hw_event_queue):
                event_type = _event_type(event)

                if event_type == "PREPARE_CALIBRATION":
                    calibration_generation += 1
                    calibration_ready_emitted = False
                    ir_loss_notified_mode = None

                    if imu.calibrating:
                        imu.cancel_calibration()

                    missing = []
                    if not ir_opened:
                        missing.append("IR")
                    if not imu_opened:
                        missing.append("IMU")
                    if not motor_opened:
                        missing.append("Motor3")
                    if motor_opened and not motor.motor3_config_enabled:
                        missing.append("Motor3 Disabled")

                    if missing:
                        workflow_state = HW_IDLE
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CALIBRATION_READY",
                            success=False,
                            message=(
                                "Calibration 준비 실패: "
                                + ", ".join(missing)
                                + " Hardware가 준비되지 않았습니다."
                            ),
                            generation=calibration_generation,
                            workflow_state=workflow_state,
                        )
                        continue

                    ir.reset_stability()
                    latest_ir_state = ir.update()
                    workflow_state = HW_IR_CHECK
                    prepare_started_at = time.monotonic()

                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "HARDWARE_CALIBRATION_PRECHECK_STARTED",
                        success=True,
                        message="IR 사용자 감지를 확인합니다.",
                        generation=calibration_generation,
                        workflow_state=workflow_state,
                    )
                    print("[HardwareProcess] Calibration 준비: IR 확인 시작")
                    continue

                if event_type == "CANCEL_CALIBRATION_PREPARE":
                    if imu.calibrating:
                        imu.cancel_calibration()
                    workflow_state = HW_IDLE
                    prepare_started_at = None
                    calibration_ready_emitted = False
                    continue

                if event_type == "IMU_CALIBRATE":
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "IMU_CALIBRATION_REJECTED",
                        success=False,
                        message="앱에서는 PREPARE_CALIBRATION(IR 확인 포함) 경로를 사용해주세요.",
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
                            message="Hardware 제어 설정을 저장하고 즉시 반영했습니다.",
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
                        updated = config_service.reset_defaults(
                            preserve_imu_calibration=True
                        )
                        apply_runtime_config(updated)
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CONFIG_RESET",
                            success=True,
                            message="Hardware 제어 설정을 기본값으로 복원했습니다.",
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
                    motor.set_enabled(True)
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR_ENABLE_ACK",
                        success=True,
                        message="Motor3 runtime 제어 허용",
                    )
                    continue

                if event_type == "MOTOR_DISABLE":
                    motor.set_enabled(False)
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR_DISABLE_ACK",
                        success=True,
                        message="Motor3 runtime 제어 차단",
                    )
                    continue

                _put_hardware_event(
                    hw_to_main_event_queue,
                    "HARDWARE_EVENT_ACK",
                    source="MAIN",
                    received_type=event_type,
                )

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
            # IR / IMU sampling
            # ==================================================
            now = time.monotonic()

            if now - last_ir_sample_time >= ir.sample_interval:
                last_ir_sample_time = now
                latest_ir_state = ir.update()

            if now - last_imu_sample_time >= imu.sample_interval:
                last_imu_sample_time = now
                latest_imu_state = imu.update()

                # IMU Offset이 새로 완성된 순간만 JSON에 기록한다.
                calibration_result = imu.consume_calibration_result()
                if calibration_result is not None:
                    try:
                        config_data = config_service.update_imu_calibration(
                            calibration_result["pitch_offset_deg"],
                            calibration_result["roll_offset_deg"],
                            calibration_result["sample_count"],
                        )
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "IMU_OFFSET_SAVED",
                            success=True,
                            message="IMU Offset을 hardware_control.json에 저장했습니다.",
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
            # Calibration Prepare State Machine: IR -> IMU Offset
            # ==================================================
            if workflow_state == HW_IR_CHECK:
                if latest_ir_state.get("stable_detected", False):
                    success = imu.start_calibration()
                    latest_imu_state = dict(imu.latest_state)

                    if success:
                        workflow_state = HW_IMU_OFFSET_CALIBRATING
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "IR_CHECK_PASSED",
                            success=True,
                            message="IR 사용자 감지 확인 완료. IMU Offset 측정을 시작합니다.",
                            generation=calibration_generation,
                        )
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "IMU_OFFSET_CALIBRATION_STARTED",
                            success=True,
                            message=f"IMU Offset 측정 시작 ({imu.calibration_sec:.1f}초)",
                            generation=calibration_generation,
                        )
                        print("[HardwareProcess] IR 확인 완료 -> IMU Offset 시작")
                    else:
                        workflow_state = HW_IDLE
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "HARDWARE_CALIBRATION_READY",
                            success=False,
                            message="IMU Offset Calibration을 시작하지 못했습니다.",
                            generation=calibration_generation,
                            workflow_state=workflow_state,
                        )

                elif (
                    prepare_started_at is not None
                    and now - prepare_started_at >= current_ir_check_timeout()
                ):
                    workflow_state = HW_IDLE
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "HARDWARE_CALIBRATION_READY",
                        success=False,
                        message=(
                            f"IR 센서에서 사용자를 {current_ir_check_timeout():.1f}초 안에 "
                            "안정적으로 감지하지 못했습니다. 위치를 확인해주세요."
                        ),
                        generation=calibration_generation,
                        workflow_state=workflow_state,
                    )
                    print("[HardwareProcess] IR 확인 실패 -> Calibration 차단")

            elif workflow_state == HW_IMU_OFFSET_CALIBRATING:
                if (
                    latest_ir_state.get("lost_duration_sec", 0.0)
                    >= ir.lost_grace_sec
                ):
                    imu.cancel_calibration()
                    latest_imu_state = dict(imu.latest_state)
                    workflow_state = HW_IDLE
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "HARDWARE_CALIBRATION_READY",
                        success=False,
                        message="IMU Offset 측정 중 IR 사용자 감지가 끊겨 Calibration을 취소했습니다.",
                        generation=calibration_generation,
                        workflow_state=workflow_state,
                    )
                    print("[HardwareProcess] IMU Offset 중 IR LOST -> Calibration 취소")

                elif (
                    imu.calibrated
                    and not imu.calibrating
                    and not calibration_ready_emitted
                ):
                    calibration_ready_emitted = True
                    workflow_state = HW_READY_FOR_POSE_CALIBRATION
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "HARDWARE_CALIBRATION_READY",
                        success=True,
                        message=(
                            "IR 확인 및 IMU Offset 완료. "
                            "Pose Calibration + IMU/Motor3 짐벌 제어를 시작할 수 있습니다."
                        ),
                        generation=calibration_generation,
                        workflow_state=workflow_state,
                        pitch_offset_deg=latest_imu_state.get("pitch_offset_deg"),
                        roll_offset_deg=latest_imu_state.get("roll_offset_deg"),
                    )
                    print("[HardwareProcess] IMU Offset 완료 -> Pose Calibration READY")

            # ==================================================
            # Main mode -> workflow display
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
            # Gimbal: CALIBRATING + MEASURING only
            # ==================================================
            # 제어 허용 여부는 Queue/state snapshot보다 실제 IMU Service의
            # 세션 Calibration 상태를 우선한다. stale state로 Motor3가 너무 일찍
            # 켜지는 것을 막는다.
            imu_ready = bool(
                imu.available
                and imu.calibrated
                and not imu.calibrating
                and latest_imu_state.get("available", False)
            )
            ir_gate = bool(
                latest_ir_state.get("available", False)
                and latest_ir_state.get("detected", False)
                and latest_ir_state.get("lost_duration_sec", 0.0) < ir.lost_grace_sec
            )

            motor_control_requested = main_mode in ("CALIBRATING", "MEASURING")
            motor_control_active = bool(
                motor_control_requested
                and imu_ready
                and ir_gate
                and motor.motor3_config_enabled
            )

            if motor_control_requested and not ir_gate:
                if ir_loss_notified_mode != main_mode:
                    ir_loss_notified_mode = main_mode
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        (
                            "IR_LOST_DURING_CALIBRATION"
                            if main_mode == "CALIBRATING"
                            else "IR_LOST_DURING_MEASUREMENT"
                        ),
                        success=False,
                        message="IR 사용자 감지가 끊겨 Motor3 짐벌 제어를 중지했습니다.",
                        workflow_state=workflow_state,
                    )
            else:
                ir_loss_notified_mode = None

            latest_motor_state = motor.update(
                imu_state=latest_imu_state,
                control_active=motor_control_active,
                pid_limit_deg_s=imu.output_limit_deg_s,
            )

            # MediaPipe landmark: 수신만 하고 현재 아무 동작도 하지 않는다.
            if latest_pose_landmark_valid:
                pass

            # ==================================================
            # Hardware state broadcast 20Hz
            # Main/Pose/Face가 동일 최신 IR state를 꺼내 쓸 수 있다.
            # ==================================================
            if now - last_status_time >= HARDWARE_STATUS_INTERVAL_SEC:
                last_status_time = now

                state = {
                    "type": "HARDWARE_STATE",
                    "ready": True,
                    "content_ready": bool(
                        ir_opened
                        and imu_opened
                        and motor_opened
                        and motor.motor3_config_enabled
                    ),
                    "timestamp": time.time(),
                    "workflow_state": workflow_state,
                    "main_mode": main_mode,
                    "main_state_received": latest_main_state is not None,
                    "pose_enabled": bool(enable_pose),
                    "pose_state_received": (
                        latest_pose_state is not None
                    ) if enable_pose else False,
                    "pose_frame_id": latest_pose_frame_id if enable_pose else None,
                    "pose_landmark_valid": (
                        latest_pose_landmark_valid
                    ) if enable_pose else False,
                    "pose_landmark_count": (
                        len(latest_pose_landmarks)
                        if enable_pose and latest_pose_landmarks is not None
                        else 0
                    ),
                    "face_enabled": bool(enable_face),
                    "face_state_received": (
                        latest_face_state is not None
                    ) if enable_face else False,
                    "ir": dict(latest_ir_state),
                    "imu": dict(latest_imu_state),
                    "motor": dict(latest_motor_state),
                    "gimbal": {
                        "requested": motor_control_requested,
                        "active": motor_control_active,
                        "ir_gate": ir_gate,
                        "imu_ready": imu_ready,
                        "motor3_packet_suppressed_at_zero": True,
                        "motor4_pass": True,
                    },
                }

                put_latest(hw_to_main_state_queue, state)
                if enable_pose:
                    put_latest(hw_to_pose_state_queue, state)
                if enable_face:
                    put_latest(hw_to_face_state_queue, state)

            time.sleep(HARDWARE_LOOP_SLEEP_SEC)

    finally:
        motor.close()
        imu.close()
        ir.close()

        stopped_event = {
            "type": "HARDWARE_STOPPED",
            "ready": False,
            "content_ready": False,
            "timestamp": time.time(),
        }
        put_ordered(hw_to_main_event_queue, stopped_event)
        print("[HardwareProcess] 종료")

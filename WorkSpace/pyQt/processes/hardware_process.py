import json
import time
from pathlib import Path

from modules.config import FRAME_HEIGHT, FRAME_WIDTH
from modules.app_settings import AlarmSettings, SettingsManager

from ipc.queue_utils import drain_ordered, get_latest, put_latest, put_ordered
from services.buzzer_service import BuzzerService
from services.posture_alert_service import PostureAlertService
from services.hardware_config_service import HardwareConfigService
from services.imu_service import ADXL345IMUService
from services.monitor_arm_calibration_service import (
    MonitorArmPreparationCalibrationService,
)
from services.monitor_arm_preparation_controller import (
    MonitorArmPreparationController,
)
from services.monitor_arm_user_x import (
    EyeGapVisionDistanceEstimator,
    ToFUserXSource,
    UserXFusion,
    measure_pose_eye_gap,
)
from services.motor_service import MotorService
from services.motor12_controller import Motor12Controller
from services.motor34_controller import Motor34Controller
from services.tof_service import create_tof_service


HARDWARE_STATUS_INTERVAL_SEC = 0.05
HARDWARE_LOOP_SLEEP_SEC = 0.002

WORKSPACE_DIR = Path(__file__).resolve().parents[2]
MONITOR_ARM_SETTINGS_FILE = WORKSPACE_DIR / "config" / "monitor_arm_settings.json"
ALARM_SETTINGS_FILE = WORKSPACE_DIR / "data" / "settings" / "alarm_settings.json"

HW_IDLE = "IDLE"
HW_IMU_OFFSET_CALIBRATING = "IMU_OFFSET_CALIBRATING"
HW_READY_FOR_POSE_CALIBRATION = "READY_FOR_POSE_CALIBRATION"
HW_MONITOR_ARM_PREPARING = "MONITOR_ARM_PREPARING"
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

def _load_monitor_arm_settings():
    """Motor1/2 모니터암의 고정 프로젝트 설정을 한 번 읽는다.

    hardware_control.json은 PyQt에서 바꿀 수 있는 런타임 튜닝값이고,
    monitor_arm_settings.json은 팀원 모니터암 알고리즘의 고정 기구/안전 설정이므로
    서로 섞지 않고 별도 파일로 유지한다.
    """
    if not MONITOR_ARM_SETTINGS_FILE.exists():
        raise FileNotFoundError(
            f"모니터암 설정 파일이 없습니다: {MONITOR_ARM_SETTINGS_FILE}"
        )

    with MONITOR_ARM_SETTINGS_FILE.open("r", encoding="utf-8") as file:
        settings = json.load(file)

    if not isinstance(settings, dict):
        raise ValueError("모니터암 설정 파일의 최상위 값은 JSON object여야 합니다.")

    return settings


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
    monitor_arm_settings = _load_monitor_arm_settings()

    # --------------------------------------------------------
    # Pose 자세 알림 설정
    # --------------------------------------------------------
    # PyQt가 저장한 alarm_settings.json을 Hardware Process에서도
    # 시작 시 한 번 읽는다.
    #
    # 이렇게 하면 사용자가 앱 실행 후 설정 저장 버튼을 다시 누르지 않아도
    # 이전에 저장해 둔 자세 알림 설정이 바로 적용된다.

    alarm_settings = SettingsManager(ALARM_SETTINGS_FILE).load()

    # --------------------------------------------------------
    # Motor1/2 사용자 위치 입력
    # --------------------------------------------------------
    # 실제 ToF I2C 통신은 tof_service.py가 담당하고,
    # 이 HardwareProcess에서는 거리값을 base-user X로 변환한 뒤
    # 기존 Pose landmark의 눈 간격 기반 Vision 거리와 융합한다.
    tof_cfg = monitor_arm_settings["tof"]
    fusion_cfg = monitor_arm_settings.get("fusion", {},)

    tof_service = create_tof_service(tof_cfg)

    tof_source = ToFUserXSource(
        sensor_service=tof_service,
        sensor_origin_x_m=float(
            tof_cfg.get(
                "sensor_origin_x_m",
                0.0,
            )
        ),
        minimum_user_x_m=float(
            tof_cfg["minimum_user_x_m"]
        ),
        maximum_user_x_m=float(
            tof_cfg["maximum_user_x_m"]
        ),
    )

    vision_estimator = EyeGapVisionDistanceEstimator(
        minimum_eye_gap_px=float(
            fusion_cfg.get(
                "minimum_eye_gap_px",
                5.0,
            )
        ),
        minimum_distance_m=float(
            fusion_cfg.get(
                "minimum_vision_distance_m",
                0.25,
            )
        ),
        maximum_distance_m=float(
            fusion_cfg.get(
                "maximum_vision_distance_m",
                1.2,
            )
        ),
        filter_alpha=float(
            fusion_cfg.get(
                "vision_filter_alpha",
                0.25,
            )
        ),
    )

    user_x_fusion = UserXFusion(
        tof_weight=float(
            fusion_cfg.get(
                "tof_weight",
                0.7,
            )
        ),
        vision_weight=float(
            fusion_cfg.get(
                "vision_weight",
                0.3,
            )
        ),
    )

    # 생성자는 JSON이 아니라 코드 상수 기본값으로 초기화된다.
    imu = ADXL345IMUService()
    motor_service = MotorService()

    # --------------------------------------------------------
    # Pose Alert / Buzzer
    # --------------------------------------------------------
    # PostureAlertService:
    #   Pose 결과를 보고 "언제" 경고할지 판단한다.
    #
    # BuzzerService:
    #   결정된 Alert command를 실제 GPIO18 PWM으로 출력한다.
    posture_alert = PostureAlertService()
    posture_alert.apply_settings(alarm_settings)

    # 일반 자세 Alert가 한 번 발생했을 때
    # 실제 부저를 몇 번 울릴지에 대한 PyQt 설정값.
    posture_alert_count = int(alarm_settings.posture_Hardware_count)

    buzzer = BuzzerService()

    # 실제 제어 로직은 별도 Controller로 분리한다.
    motor12 = Motor12Controller(
        motor_service,
        settings=monitor_arm_settings,
    )
    motor34 = Motor34Controller(motor_service)
    monitor_arm_preparation = MonitorArmPreparationController(
        motor_service,
        motor12,
        motor34,
        settings_path=MONITOR_ARM_SETTINGS_FILE,
    )

    # JSON에는 사용자가 바꿀 튜닝값만 있다.
    imu.apply_control_config(control["imu"], control["pid"])
    motor34.apply_control_config(control["motor"])

    imu_opened = imu.open()

    # Passive Buzzer GPIO18 초기화.
    #
    # 실패하더라도 Hardware Process 전체 준비 실패로 보지 않는다.
    # 자세 추론 / IMU / ToF / Motor는 계속 사용할 수 있다.
    buzzer_opened = buzzer.open()

    # 실제 VL53L0X import/I2C 연결은 ToF Service의 open() 안에서 수행한다.
    # 센서 초기화가 실패해도 HardwareProcess 자체는 종료하지 않고,
    # 아래 user-X 상태를 SAFE_HOLD로 유지한다.
    tof_opened = tof_source.open()

    motor_bus_opened = motor_service.open()

    # Motor1/2와 Motor3/4는 모두 필수 하드웨어다.
    # 하나의 MotorService가 Serial bus를 소유한 상태에서
    # 각 Controller가 자신의 Servo 준비 상태를 검사한다.
    motor12_opened = bool(
        motor_bus_opened
        and motor12.initialize()
    )

    motor34_opened = bool(
        motor_bus_opened
        and (motor34.motor3_config_enabled or motor34.motor4_config_enabled)
        and motor34.initialize()
    )

    latest_imu_state = dict(imu.latest_state)

    # 시작 직후 ToF 상태도 한 번 읽어 Hardware state에 반영한다.
    latest_tof_state = dict(tof_service.update(force=True))

    latest_monitor_arm_input_state = {
        "available": bool(tof_opened),
        "valid": False,
        "tof_user_x_m": None,
        "vision_user_x_m": None,
        "user_x_m": None,
        "fusion_mode": "SAFE_HOLD",
        "eye_gap_px": None,
        "last_error": (
            None
            if tof_opened
            else getattr(
                tof_service,
                "last_error",
                "ToF unavailable",
            )
        ),
        "timestamp": time.time(),
    }

    latest_motor12_state = dict(motor12.latest_state)
    latest_motor34_state = dict(motor34.latest_state)
    latest_buzzer_state = buzzer.get_state()

    # ========================================================
    # 2. IPC 최신 상태
    # ========================================================
    latest_main_state = None
    latest_pose_state = None
    latest_face_state = None
    latest_pose_frame_id = None
    latest_pose_landmarks = None
    latest_pose_landmark_valid = False

    # 자세 Alert가 같은 Pose frame을 중복 처리하지 않도록
    # 마지막으로 Alert 판단에 사용한 Pose frame ID를 기억한다.
    last_alert_pose_frame_id = None

    # 같은 Pose frame을 빠른 Hardware loop에서 반복 처리하면
    # Vision EMA가 한 프레임에 여러 번 적용될 수 있으므로
    # 마지막 처리 frame과 Vision 결과를 별도 상태로 유지한다.
    last_vision_pose_frame_id = None
    latest_vision_user_x_m = None
    latest_eye_gap_px = None

    monitor_arm_calibration = MonitorArmPreparationCalibrationService(
        duration_sec=5.0,
        minimum_tof_samples=max(
            1,
            int(5.0 * float(tof_cfg.get("sample_hz", 20.0)) * 0.6),
        ),
        minimum_eye_samples=30,
    )
    latest_preparation_state = monitor_arm_preparation.snapshot()

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
        nonlocal config_data, control, imu_opened, motor_bus_opened, motor12_opened, motor34_opened

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

        # Motor bus가 복구되었거나 이미 열려 있다면 Motor1/2도
        # 필수 하드웨어 준비 상태를 다시 확보한다.
        if motor_bus_opened:
            motor12_opened = bool(
                motor12.ready or motor12.initialize()
            )
        else:
            motor12_opened = False

        if motor_bus_opened and (motor34.motor3_config_enabled or motor34.motor4_config_enabled):
            motor34_opened = motor34.initialize()
        else:
            motor34_opened = False

        return bool(
            imu_opened
            and motor12_opened
            and motor12.ready
            and motor34_opened
            and motor34.ready
        )

    def build_ready_event():
        motor12_ready = bool(
            motor12_opened and motor12.ready
        )
        motor34_ready = bool(
            motor34_opened and motor34.ready
        )

        # IMU + Motor1/2 + Motor3/4가 모두 준비되어야
        # POCO Hardware 전체가 실제 측정 가능한 상태로 판단된다.
        all_ready = bool(
            imu_opened
            and motor12_ready
            and motor34_ready
        )

        errors = []

        if not imu_opened:
            errors.append(
                f"IMU 실패: {imu.last_error}"
            )

        if not motor_bus_opened:
            errors.append(
                f"Motor Bus 실패: {motor_service.last_error}"
            )
        else:
            if not motor12_ready:
                errors.append(
                    f"Motor1/2 실패: {motor12.last_error}"
                )

            if not motor34_ready:
                errors.append(
                    f"Motor3/4 실패: {motor34.last_error}"
                )

        if not motor34.motor3_config_enabled:
            errors.append("Motor3 Disabled")

        if not motor34.motor4_config_enabled:
            errors.append("Motor4 Disabled")

        return {
            "type": "HARDWARE_READY",
            "ready": True,
            "content_ready": all_ready,
            "imu_ready": bool(imu_opened),
            "motor_ready": bool(
                motor12_ready and motor34_ready
            ),
            "motor12_ready": motor12_ready,
            "motor34_ready": motor34_ready,
            "message": (
                "Hardware Process IMU/Motor1~4 준비 완료"
                if all_ready
                else "Hardware 일부 준비 실패 / "
                + " / ".join(errors)
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

                if event_type == "START_MONITOR_ARM_PREPARATION":
                    monitor_arm_preparation.begin()
                    monitor_arm_calibration.cancel()
                    workflow_state = HW_MONITOR_ARM_PREPARING
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MONITOR_ARM_PREPARATION_STARTED",
                        success=True,
                        message="모니터암 초기 준비 창을 시작했습니다.",
                    )
                    continue

                if event_type == "MONITOR_ARM_CONNECT_ALL":
                    try:
                        preparation_state = monitor_arm_preparation.connect_all()
                        motor_bus_opened = bool(motor_service.available)
                        motor12_opened = bool(motor12.ready)
                        motor34_opened = bool(motor34.ready)
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_CONNECT_ALL_DONE",
                            success=True,
                            message="Servo 1~4 연결/Ping/Calibration 확인 완료",
                            preparation=preparation_state,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_CONNECT_ALL_DONE",
                            success=False,
                            message=f"Servo 1~4 연결 확인 실패: {error}",
                        )
                    continue

                if event_type == "MONITOR_ARM_MOVE_WORKING_START":
                    try:
                        target = monitor_arm_preparation.request_working_start()
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MOVE_WORKING_START_ACK",
                            success=True,
                            message="휴식자세에서 작업 시작 위치로 안전 복구를 시작합니다.",
                            target={
                                "shoulder_lift": target.shoulder_lift_deg,
                                "elbow_flex": target.elbow_flex_deg,
                            },
                        )
                    except Exception as error:
                        monitor_arm_preparation.record_movement_error(error)
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MOVE_WORKING_START_ACK",
                            success=False,
                            message=f"작업 시작 위치 이동 실패: {error}",
                        )
                    continue

                if event_type == "MONITOR_ARM_MOVE_REST":
                    confirmed = bool(event.get("confirmed", False))
                    if not confirmed:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MOVE_REST_ACK",
                            success=False,
                            message="휴식자세 이동에는 confirmed=True 확인이 필요합니다.",
                        )
                        continue
                    try:
                        result = monitor_arm_preparation.request_rest()
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MOVE_REST_ACK",
                            success=True,
                            message="Motor1/2 휴식자세 이동 명령을 전송했습니다.",
                            result=result,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MOVE_REST_ACK",
                            success=False,
                            message=f"휴식자세 이동 실패: {error}",
                        )
                    continue

                if event_type == "MONITOR_ARM_MANUAL_IK_TARGET":
                    try:
                        target = monitor_arm_preparation.command_manual_ik(
                            event["user_x_m"],
                            event["user_monitor_distance_m"],
                            event["monitor_z_m"],
                        )
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MANUAL_IK_TARGET_ACK",
                            success=True,
                            message="Motor1/2 수동 IK 목표를 전송했습니다.",
                            target={
                                "shoulder_lift": target.shoulder_lift_deg,
                                "elbow_flex": target.elbow_flex_deg,
                            },
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_MANUAL_IK_TARGET_ACK",
                            success=False,
                            message=f"수동 IK 명령 실패: {error}",
                        )
                    continue

                if event_type == "MONITOR_ARM_GIMBAL_JOG":
                    try:
                        target = monitor_arm_preparation.jog_gimbal(
                            str(event.get("joint", "")),
                            float(event.get("delta_deg", 0.0)),
                            int(event.get("speed", 100)),
                        )
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_GIMBAL_JOG_ACK",
                            success=True,
                            message=f"{event.get('joint')} 목표 {target:+.2f}°",
                            joint=event.get("joint"),
                            target_deg=target,
                        )
                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MONITOR_ARM_GIMBAL_JOG_ACK",
                            success=False,
                            message=f"Gimbal 조그 실패: {error}",
                            joint=event.get("joint"),
                        )
                    continue

                if event_type == "MONITOR_ARM_GIMBAL_JOG_STOP":
                    monitor_arm_preparation.stop_gimbal_jog(event.get("joint"))
                    continue

                if event_type == "START_MONITOR_ARM_SENSOR_CAPTURE":
                    preparation_state = monitor_arm_preparation.snapshot()
                    if not enable_pose:
                        message = "POSE_ONLY 또는 BOTH 프로필이 아니어서 눈 간격을 측정할 수 없습니다."
                        success = False
                    elif not preparation_state.get("all_motors_ready", False):
                        message = "먼저 Motor1~4 연결 확인을 완료해주세요."
                        success = False
                    elif not preparation_state.get("working_start_completed", False):
                        message = "먼저 휴식자세에서 작업 시작 위치로 이동해주세요."
                        success = False
                    else:
                        monitor_arm_calibration.start()
                        message = "ToF와 MediaPipe 눈 간격의 5초 평균 측정을 시작합니다."
                        success = True
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MONITOR_ARM_SENSOR_CAPTURE_STARTED",
                        success=success,
                        message=message,
                    )
                    continue

                if event_type == "FINISH_MONITOR_ARM_PREPARATION":
                    calibration_state = monitor_arm_calibration.snapshot()
                    preparation_state = monitor_arm_preparation.snapshot()
                    success = bool(
                        calibration_state.get("session_ready", False)
                        and preparation_state.get("all_motors_ready", False)
                        and preparation_state.get("working_start_completed", False)
                    )
                    if success:
                        monitor_arm_preparation.end()
                        workflow_state = HW_IDLE
                        message = "모터 자세와 ToF/눈 간격 초기 준비를 완료했습니다."
                    else:
                        message = (
                            "모터 1~4 연결, 작업 시작 위치 이동, 센서 평균 저장을 "
                            "모두 완료해야 합니다."
                        )
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MONITOR_ARM_PREPARATION_FINISHED",
                        success=success,
                        message=message,
                        calibration=calibration_state,
                    )
                    continue

                if event_type == "CANCEL_MONITOR_ARM_PREPARATION":
                    monitor_arm_calibration.cancel()
                    monitor_arm_preparation.end()
                    workflow_state = HW_IDLE
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MONITOR_ARM_PREPARATION_CANCELLED",
                        success=True,
                        message="모니터암 초기 준비를 취소했습니다.",
                    )
                    continue

                if event_type == "PREPARE_CALIBRATION":
                    calibration_generation += 1
                    calibration_ready_emitted = False

                    if imu.calibrating:
                        imu.cancel_calibration()

                    missing = []

                    if not imu_opened:
                        missing.append("IMU")

                    if not motor12_opened or not motor12.ready:
                        missing.append("Motor1/2")

                    if not motor34_opened or not motor34.ready:
                        missing.append("Motor3/4")

                    if not monitor_arm_calibration.session_ready:
                        missing.append("MonitorArm ToF/눈 간격 초기 준비")

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

                # =============================================
                # PyQt 자세 알림 설정 Runtime 반영
                # =============================================
                if event_type == "UPDATE_ALARM_SETTINGS":
                    try:
                        settings_data = event.get(
                            "settings",
                            event.get(
                                "data",
                                {},
                            ),
                        )

                        if not isinstance(
                            settings_data,
                            dict,
                        ):
                            raise ValueError(
                                "alarm settings는 dict여야 합니다."
                            )

                        # app_settings.py와 동일한 범위 검사를 사용해서
                        # Hardware Process에서도 안전한 설정값만 사용한다.
                        alarm_settings = (
                            AlarmSettings.from_dict(
                                settings_data
                            )
                        )

                        posture_alert.apply_settings(
                            alarm_settings
                        )

                        posture_alert_count = int(
                            alarm_settings
                            .posture_Hardware_count
                        )

                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "ALARM_SETTINGS_UPDATED",
                            success=True,
                            message=(
                                "자세 알림 설정을 "
                                "Hardware Process에 반영했습니다."
                            ),
                            settings=alarm_settings.to_dict(),
                        )

                    except Exception as error:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "ALARM_SETTINGS_UPDATED",
                            success=False,
                            message=(
                                "자세 알림 설정 반영 실패: "
                                f"{error}"
                            ),
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
                    # 전역 Motor 명령이므로 Motor1/2와 Motor3/4를
                    # 동일하게 runtime 제어 가능 상태로 전환한다.
                    motor12.set_enabled(True)
                    motor34.set_enabled(True)

                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR_ENABLE_ACK",
                        success=True,
                        message="Motor1~4 runtime 제어 허용",
                    )
                    continue

                if event_type == "MOTOR_DISABLE":
                    motor12.set_enabled(False)
                    motor34.set_enabled(False)

                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR_DISABLE_ACK",
                        success=True,
                        message="Motor1~4 runtime 제어 차단",
                    )
                    continue

                if event_type == "MOTOR12_REST":
                    # Rest는 Calibration Safe Range 밖의 특수 자세이므로
                    # event에서 명시적인 사용자 확인이 있어야 실행한다.
                    confirmed = bool(
                        event.get(
                            "confirmed",
                            False,
                        )
                        if isinstance(event, dict)
                        else False
                    )

                    if not confirmed:
                        _put_hardware_event(
                            hw_to_main_event_queue,
                            "MOTOR12_REST_CONFIRM_REQUIRED",
                            success=True,
                            message=(
                                "Motor1/2 Rest 자세는 "
                                "Calibration 안전범위 밖의 "
                                "확인된 특수 자세입니다. "
                                "팔/모니터를 지지한 뒤 "
                                "confirmed=True로 다시 요청해주세요."
                            ),
                        )
                        continue

                    result = (motor12.move_to_rest())

                    success = bool(
                        result.get(
                            "accepted",
                            False,
                        )
                    )

                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR12_REST_ACK",
                        success=success,
                        message=(
                            "Motor1/2 Rest 자세 이동 "
                            "명령을 전송했습니다."
                            if success
                            else
                            "Motor1/2 Rest 자세 이동 실패: "
                            + str(
                                result.get(
                                    "error",
                                    "unknown error",
                                )
                            )
                        ),
                        result=result,
                    )
                    continue

                if event_type == "MOTOR12_RESUME":
                    result = (motor12.resume_from_rest())

                    success = bool(
                        result.get(
                            "accepted",
                            False,
                        )
                    )

                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MOTOR12_RESUME_ACK",
                        success=success,
                        message=(
                            "Motor1/2 작업자세 Recovery를 "
                            "시작합니다."
                            if success
                            else
                            "Motor1/2 Recovery 시작 실패: "
                            + str(
                                result.get(
                                    "error",
                                    "unknown error",
                                )
                            )
                        ),
                        result=result,
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

            # ToF Service 내부에서 sample_hz를 기준으로 자체 rate-limit한다.
            # 따라서 Hardware loop에서는 항상 update()를 호출해도
            # 실제 I2C read는 설정된 주기로만 수행된다.
            latest_tof_state = tof_service.update()

            if monitor_arm_calibration.running:
                monitor_arm_calibration.add_tof_state(latest_tof_state)
                monitor_arm_calibration.add_pose_state(latest_pose_state)
                if monitor_arm_calibration.finished_by_time:
                    try:
                        metadata = monitor_arm_preparation.calibration_metadata()
                        calibration_state = monitor_arm_calibration.finish(metadata)
                        success = bool(calibration_state.get("session_ready", False))
                        if success:
                            vision_estimator.calibrate(
                                calibration_state["eye_gap_baseline_px"],
                                calibration_state[
                                    "user_monitor_distance_baseline_m"
                                ],
                            )
                            message = (
                                "초기 준비 센서 평균 저장 완료: "
                                f"ToF {calibration_state['tof_user_x_baseline_m']:.3f}m, "
                                f"Eye {calibration_state['eye_gap_baseline_px']:.2f}px"
                            )
                        else:
                            message = (
                                "초기 준비 센서 평균 실패: "
                                + str(calibration_state.get("last_error"))
                            )
                    except Exception as error:
                        monitor_arm_calibration.cancel()
                        calibration_state = monitor_arm_calibration.snapshot()
                        success = False
                        message = f"초기 준비 센서 평균 저장 실패: {error}"
                    _put_hardware_event(
                        hw_to_main_event_queue,
                        "MONITOR_ARM_SENSOR_CAPTURE_DONE",
                        success=success,
                        message=message,
                        calibration=calibration_state,
                    )


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
            if main_mode == "MONITOR_ARM_PREPARATION":
                workflow_state = HW_MONITOR_ARM_PREPARING
            elif main_mode == "CALIBRATING" and imu.calibrated:
                workflow_state = HW_POSE_CALIBRATING
            elif main_mode == "MEASURING" and imu.calibrated:
                workflow_state = HW_MEASURING
            elif main_mode in ("PREVIEW", "IDLE", "CAMERA_OFF"):
                if workflow_state in (HW_POSE_CALIBRATING, HW_MEASURING):
                    workflow_state = (
                        HW_READY_FOR_POSE_CALIBRATION if imu.calibrated else HW_IDLE
                    )

            # ==================================================
            # F-1. Motor1/2 ToF + Vision 사용자 X 계산
            # ==================================================
            tof_user_x_m = None
            fused_user_x_m = None
            input_error = None
            fusion_mode = "SAFE_HOLD"

            # 팀원 원본 안전정책:
            # - ToF가 없으면 Vision 단독 Motor 제어 금지 → SAFE_HOLD
            # - Vision이 없으면 ToF 단독 사용 가능
            try:
                tof_user_x_m = (
                    tof_source.read_user_x_m()
                )

            except ValueError as error:
                # ToF가 유효하지 않으면 Vision 값을 사용하지 않는다.
                latest_vision_user_x_m = None
                latest_eye_gap_px = None
                input_error = str(error)

            else:
                # Pose Process보다 Hardware loop가 훨씬 빠르므로
                # 새 Pose frame에서만 Vision EMA를 한 번 갱신한다.
                if (
                    latest_pose_landmark_valid
                    and latest_pose_frame_id
                    != last_vision_pose_frame_id
                ):
                    last_vision_pose_frame_id = (
                        latest_pose_frame_id
                    )

                    eye = measure_pose_eye_gap(
                        latest_pose_landmarks,
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                    )

                    latest_eye_gap_px = (
                        None
                        if eye is None
                        else float(eye.gap_px)
                    )

                    latest_vision_user_x_m = None

                    if eye is not None:
                        try:
                            (
                                _current_angles,
                                current_monitor_pose,
                            ) = motor12.read_current_arm_state()

                            # 팀원 원본과 동일하게 첫 Vision 기준거리는
                            # 현재 ToF user X - 현재 Monitor X로 보정한다.
                            if not vision_estimator.calibrated:
                                vision_estimator.calibrate(
                                    eye.gap_px,
                                    tof_user_x_m
                                    - current_monitor_pose.x_m,
                                )

                            vision_distance_m = (
                                vision_estimator
                                .estimate_distance_m(
                                    eye.gap_px
                                )
                            )

                            latest_vision_user_x_m = float(
                                current_monitor_pose.x_m
                                + vision_distance_m
                            )

                        except (
                            RuntimeError,
                            ValueError,
                        ):
                            # 눈 landmark 또는 Motor 현재각이 순간적으로
                            # 유효하지 않아도 ToF 단독 제어는 허용한다.
                            latest_vision_user_x_m = None

                elif not latest_pose_landmark_valid:
                    latest_eye_gap_px = None
                    latest_vision_user_x_m = None

                try:
                    fused_user_x_m = (
                        tof_source.validate_user_x_m(
                            user_x_fusion.fuse(
                                tof_user_x_m,
                                latest_vision_user_x_m,
                            )
                        )
                    )

                    fusion_mode = (
                        "FUSED"
                        if latest_vision_user_x_m
                        is not None
                        else "TOF_ONLY"
                    )

                except ValueError as error:
                    fused_user_x_m = None
                    input_error = str(error)

            latest_monitor_arm_input_state = {
                "available": bool(
                    getattr(
                        tof_service,
                        "available",
                        False,
                    )
                ),
                "valid": (
                    fused_user_x_m is not None
                ),
                "tof_user_x_m": tof_user_x_m,
                "vision_user_x_m": (
                    latest_vision_user_x_m
                ),
                "user_x_m": fused_user_x_m,
                "fusion_mode": fusion_mode,
                "eye_gap_px": latest_eye_gap_px,
                "last_error": input_error,
                "timestamp": time.time(),
            }

            # ==================================================
            # F. Motor control context
            # ==================================================
            imu_ready = bool(imu.available and imu.calibrated and not imu.calibrating)
            latest_preparation_state = monitor_arm_preparation.update(now)
            preparation_active = bool(
                latest_preparation_state.get("active", False)
            )
            preparation_recovery = bool(
                preparation_active
                and latest_preparation_state.get("recovery_active", False)
            )

            motor12_input_state = dict(latest_monitor_arm_input_state)
            if preparation_recovery and not motor12_input_state.get("valid", False):
                motor12_input_state = {
                    "valid": True,
                    "user_x_m": float(
                        monitor_arm_settings.get("manual_cartesian", {}).get(
                            "user_x_min_m", 0.6007655
                        )
                    ),
                    "last_error": None,
                    "source": "PREPARATION_RECOVERY",
                }
            # 준비 창에서는 오직 명시적인 Recovery/수동 IK만 허용한다.
            # 일반 ToF/Vision 자동추종은 실제 자세 측정(MEASURING) 중에만 켜서
            # 카메라/Process 초기화 직후 팔이 예기치 않게 움직이지 않게 한다.
            motor12_requested = bool(
                preparation_recovery
                or (
                    not preparation_active
                    and main_mode == "MEASURING"
                    and latest_monitor_arm_input_state.get("valid", False)
                )
            )

            motor12_active = bool(
                motor12_requested
                and motor12.ready
            )

            motor34_requested = bool(
                not preparation_active
                and main_mode in ("CALIBRATING", "MEASURING")
            )

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

                # Motor12 입력은 한 하위 dict로만 전달해서
                # 기존 context 구조가 여러 flat key로 늘어나지 않도록 한다.
                "motor12": {
                    "control_active": motor12_active,
                    "input": motor12_input_state,
                },

                "motor34_control_active": motor34_active,
            }

            # 실행 순서가 코드에 그대로 보이도록 유지한다.
            latest_motor12_state = motor12.update(context)   # Motor 1 -> 2
            latest_motor34_state = motor34.update(context)   # Motor 3 -> 4
            latest_preparation_state = monitor_arm_preparation.update(now)

            # 기존 Motor3/4 최상위 진단값(command_hz, axis_mapping 등)은
            # 호환성을 위해 유지한다. 대신 available/enabled/ready/control_active는
            # Motor12 + Motor34 전체를 나타내도록 집계하고, 두 Controller 그룹을
            # motor12/motor34로 대칭 제공한다. 개별 motor1~4 key도 계속 유지한다.
            latest_motor_state = dict(latest_motor34_state)

            latest_motor_state["available"] = bool(
                latest_motor12_state.get("available", False)
                and latest_motor34_state.get("available", False)
            )

            latest_motor_state["enabled"] = bool(
                latest_motor12_state.get("enabled", False)
                and latest_motor34_state.get("enabled", False)
            )

            latest_motor_state["ready"] = bool(
                latest_motor12_state.get("ready", False)
                and latest_motor34_state.get("ready", False)
            )

            latest_motor_state["control_active"] = bool(
                latest_motor12_state.get("control_active", False)
                or latest_motor34_state.get("control_active", False)
            )

            latest_motor_state["bus"] = motor_service.get_state()

            latest_motor_state["motor12"] = dict(latest_motor12_state)
            latest_motor_state["motor34"] = dict(latest_motor34_state)

            latest_motor_state["motor1"] = dict(
                latest_motor12_state["motor1"]
            )
            latest_motor_state["motor2"] = dict(
                latest_motor12_state["motor2"]
            )
            latest_motor_state["motor3"] = dict(
                latest_motor34_state["motor3"]
            )
            latest_motor_state["motor4"] = dict(
                latest_motor34_state["motor4"]
            )

            # ==================================================
            # F-2. Pose 자세 알림 -> Passive Buzzer
            # ==================================================
            # 자세 경고는 실제 자세 측정(MEASURING) 중에만 동작한다.
            #
            # PREPARING:
            #   모니터암 위치 / ToF / 눈 간격 준비 단계이므로 경고 금지
            #
            # CALIBRATING:
            #   기준 자세를 측정하는 단계이므로 경고 금지
            #
            # MEASURING:
            #   GRU pose_index를 PostureAlertService에 전달
            #
            # Hardware loop는 약 2ms로 Pose Process보다 훨씬 빠르다.
            # 따라서 같은 POSE_STATE를 반복 처리하지 않고
            # 새 frame_id가 들어왔을 때만 Alert 판단을 수행한다.

            measuring_for_alert = bool(
                enable_pose
                and main_mode == "MEASURING"
            )

            if measuring_for_alert:
                pose_inference = None

                if isinstance(
                    latest_pose_state,
                    dict,
                ):
                    pose_inference = (
                        latest_pose_state.get(
                            "inference"
                        )
                    )

                # Hardware Process는 약 2ms 주기로 반복되지만
                # Pose Process의 GRU 결과는 그보다 느리게 갱신된다.
                #
                # 따라서 같은 Pose frame을 반복해서 Alert 판단에
                # 사용하지 않고 새 frame_id가 들어왔을 때만 처리한다.
                if (
                    isinstance(
                        pose_inference,
                        dict,
                    )
                    and latest_pose_frame_id
                    is not None
                    and latest_pose_frame_id
                    != last_alert_pose_frame_id
                ):
                    last_alert_pose_frame_id = (
                        latest_pose_frame_id
                    )

                    pose_index = (
                        pose_inference.get(
                            "pose_index"
                        )
                    )

                    alert_command = (
                        posture_alert.update(
                            pose_index,
                            now=now,
                        )
                    )

                    # PostureAlertService 판단 결과:
                    #
                    # None
                    #   아직 유지시간 미충족 또는 Cooldown 중
                    #
                    # Optimal
                    #   정상 자세. BuzzerService에서는 소리를 내지 않는다.
                    #
                    # Asymmetric / ForwardHead / ChinPropping
                    #   일반 자세 Alert
                    #
                    # StrongAlert
                    #   강한 자세 Alert
                    if alert_command is not None:
                        buzzer.play_command(
                            alert_command,
                            posture_alert_count,
                        )

            # BuzzerService는 sleep()을 사용하지 않는
            # non-blocking 상태머신이다.
            #
            # MEASURING이 끝나더라도 이미 시작된 Alert는
            # 현재 Pattern까지 정상적으로
            # 마무리할 수 있도록 update()는 계속 호출한다.
            latest_buzzer_state = (
                buzzer.update(
                    now=now
                )
            )

            # ==================================================
            # G. Hardware -> Main/Pose/Face 최신 State (20Hz)
            # ==================================================
            if now - last_status_time >= HARDWARE_STATUS_INTERVAL_SEC:
                last_status_time = now

                state = {
                    "type": "HARDWARE_STATE",
                    "ready": True,
                    "content_ready": bool(
                        imu_opened
                        and motor12_opened
                        and motor12.ready
                        and motor34_opened
                        and motor34.ready
                    ),
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
                    "tof": dict(latest_tof_state),
                    "monitor_arm_input": dict(latest_monitor_arm_input_state),
                    "monitor_arm": {
                        "calibration": monitor_arm_calibration.snapshot(),
                        "preparation": dict(latest_preparation_state),
                        "live_eye_gap_px": (
                            latest_pose_state.get("eye_gap_px")
                            if isinstance(latest_pose_state, dict)
                            else None
                        ),
                        "live_tof_user_x_m": latest_monitor_arm_input_state.get(
                            "tof_user_x_m"
                        ),
                        "fusion_weights": {
                            "tof": user_x_fusion.tof_weight,
                            "vision": user_x_fusion.vision_weight,
                        },
                        "control_prerequisites_ready": bool(
                            monitor_arm_calibration.session_ready
                            and latest_monitor_arm_input_state.get("valid", False)
                        ),
                    },
                    "motor": latest_motor_state,

                    # Passive Buzzer + 자세 Alert Runtime 상태.
                    #
                    # Buzzer가 unavailable이어도 content_ready에는
                    # 영향을 주지 않는다.
                    "buzzer": {
                        **dict(
                            latest_buzzer_state
                        ),

                        # 일반 자세 Alert에서 실제 삐 소리를
                        # 몇 회 반복할지에 대한 PyQt 설정.
                        "posture_count": int(
                            posture_alert_count
                        ),

                        # 자세 유지시간 / StrongAlert / Cooldown
                        # 판단 상태.
                        "posture_alert": (
                            posture_alert.get_state(
                                now=now
                            )
                        ),
                    },

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
        # HardwareProcess가 소유한 I2C/Serial 장치를 모두 정리한다.
        # 각 Controller의 runtime 상태를 먼저 정리한 뒤
        # 공통 MotorService Serial bus를 마지막에 닫는다.
        # GPIO PWM을 먼저 OFF하고 Resource를 반환한다.
        buzzer.close()

        monitor_arm_preparation.end()
        tof_source.close()
        motor12.close()
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

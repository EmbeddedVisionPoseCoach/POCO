import time
from dataclasses import dataclass

from services.monitor_arm_kinematics import JointCommand
from services.monitor_arm_planner import MonitorArmPlanner
from services.monitor_arm_speed import select_speed, validate_speed_profile

MOTOR1_SERVO_ID = 1
MOTOR1_JOINT = "shoulder_lift"
MOTOR2_SERVO_ID = 2
MOTOR2_JOINT = "elbow_flex"

MOTOR12_UPDATE_HZ = 20.0


@dataclass
class _Motor12Runtime:
    """Motor1/2 공통 하드웨어 준비 상태."""

    servo_id: int
    joint: str
    ready: bool = False
    max_speed: int | None = None
    safe_min_deg: float | None = None
    safe_max_deg: float | None = None

    def reset_ready(self):
        self.ready = False
        self.max_speed = None
        self.safe_min_deg = None
        self.safe_max_deg = None


class Motor12Controller:
    """Motor1 / Motor2 모니터암 제어 계층.

    이 Controller는 Motor1/2를 하나의 2축 모니터암으로 다룬다.
    두 Servo는 필수 하드웨어이므로 둘 다 Calibration / Servo ID /
    max_speed / Safe Range / Ping 검사를 통과해야 ready=True가 된다.

    MonitorArmPlanner는 전달받은 고정 settings로 생성한다.
    일반 추종 목표의 속도 선택 / SyncWrite 실행 경로를 내부에 유지하며,
    실제 ToF/융합 user_x 입력과 Planner 자동 호출은 이후 단계에서 연결한다.
    """

    def __init__(self, motor_service, settings=None, update_hz=MOTOR12_UPDATE_HZ):
        self.motor = motor_service

        # settings=None은 기존 self-test 호환을 위해 임시로 허용한다.
        # 실제 HardwareProcess에서는 WorkSpace/config의 고정 JSON을 항상 전달한다.
        if settings is not None and not isinstance(settings, dict):
            raise TypeError("Motor12 settings는 dict 또는 None이어야 합니다.")

        self.settings = settings
        self.planner = MonitorArmPlanner(settings) if settings is not None else None

        # 팀원 standalone Motor1/2 코드의 Pose 추종 속도 정책을 그대로 사용한다.
        # settings=None은 기존 self-test 호환용이며 실제 HardwareProcess에서는
        # WorkSpace/config/monitor_arm_settings.json이 항상 전달된다.
        self.pose_max_speed = None
        self.pose_acc = None
        self.pose_speed_mode = None
        self.pose_min_speed = None
        self.pose_full_speed_error_deg = None

        # Motor12 Controller 자체 update는 20Hz로 수행하지만,
        # 실제 Servo1/2 명령은 monitor_arm_settings.json의 command_hz로 제한한다.
        self.command_hz = 5.0

        if settings is not None:
            control = settings.get("control", {})

            self.command_hz = max(
                1.0,
                float(
                    control.get(
                        "command_hz",
                        self.command_hz,
                    )
                ),
            )

            self.pose_max_speed = int(
                control.get(
                    "pose_speed",
                    control.get("vertical_ik_speed", control.get("speed", 1)),
                )
            )
            self.pose_acc = int(
                control.get(
                    "pose_acc",
                    control.get("vertical_ik_acc", control.get("acc", 10)),
                )
            )
            self.pose_speed_mode = str(
                control.get(
                    "pose_speed_mode",
                    control.get("vertical_ik_speed_mode", "fixed"),
                )
            )
            self.pose_min_speed = int(
                control.get(
                    "pose_variable_min_speed",
                    control.get("vertical_ik_variable_min_speed", 1),
                )
            )
            self.pose_full_speed_error_deg = float(
                control.get(
                    "pose_variable_full_speed_error_deg",
                    min(
                        float(
                            control.get(
                                "vertical_ik_variable_full_speed_error_deg",
                                30.0,
                            )
                        ),
                        float(settings["safety"]["max_joint_step_deg"]),
                    ),
                )
            )

            validate_speed_profile(
                self.pose_speed_mode,
                self.pose_max_speed,
                self.pose_min_speed,
                self.pose_full_speed_error_deg,
            )

            if not 0 <= self.pose_acc <= 30:
                raise ValueError("Motor1/2 pose_acc 허용범위는 0~30입니다.")

        self.command_interval = 1.0 / self.command_hz
        self.last_command_time = 0.0

        # 자동추종 진단 상태.
        # HARDWARE_STATE에서 마지막 목표와 실제 명령 결과를 확인할 수 있도록 유지한다.
        self.last_target = None
        self.last_command_speed = 0
        self.last_largest_delta_deg = 0.0
        self.last_success = None
        self.last_user_x_m = None
        self.hold_reason = "IDLE"
    
        self.update_hz = max(1.0, float(update_hz))
        self.update_interval = 1.0 / self.update_hz
        self.last_update_time = 0.0
        self.enabled = True
        self.available = False
        self.last_error = None

        self.motor1 = _Motor12Runtime(
            servo_id=MOTOR1_SERVO_ID,
            joint=MOTOR1_JOINT,
        )
        self.motor2 = _Motor12Runtime(
            servo_id=MOTOR2_SERVO_ID,
            joint=MOTOR2_JOINT,
        )

        self.latest_state = self._build_state(False)

    @property
    def ready(self):
        if not self.available or not self.motor.available:
            return False

        return bool(
            self.motor1.ready
            and self.motor2.ready
        )

    def initialize(self):
        """Motor1/2 Calibration + Servo Ping + 안전 설정을 확인한다."""
        self.available = False
        self.last_error = None
        self._reset_ready_state()
        self._reset_tracking_runtime(reset_reference=True)

        if not self.motor.available:
            self.last_error = "MotorService가 준비되지 않았습니다."
            self.latest_state = self._build_state(False)
            return False

        try:
            checked = []

            for axis in (self.motor1, self.motor2):
                self._initialize_axis(axis)

                checked.append(
                    f"Servo{axis.servo_id}={axis.joint}(PING=OK) "
                    f"safe={axis.safe_min_deg:+.2f}~{axis.safe_max_deg:+.2f}deg "
                    f"max_speed={axis.max_speed}"
                )

            self.available = True
            self.last_error = None
            self.latest_state = self._build_state(False)

            print(
                "[MOTOR12] 모니터암 준비 완료 "
                + " / ".join(checked)
            )

            return True

        except Exception as error:
            self.available = False
            self.last_error = str(error)
            self._reset_ready_state()
            self.latest_state = self._build_state(False)

            print(f"[MOTOR12] 초기화/안전검사 실패: {error}")

            return False

    def _initialize_axis(self, axis):
        servo = self.motor.get_joint_metadata(axis.joint)

        if not servo:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                "Calibration 정보가 없습니다."
            )

        actual_servo_id = int(servo.get("servo_id", -1))

        if actual_servo_id != axis.servo_id:
            raise RuntimeError(
                f"{axis.joint} Servo ID 불일치: "
                f"expected={axis.servo_id}, "
                f"actual={actual_servo_id}"
            )

        max_speed = self.motor.get_max_speed(axis.joint)

        if max_speed is None or max_speed <= 0:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                "max_speed가 설정되지 않았습니다."
            )

        safe_range = self.motor.get_safe_angle_range(axis.joint)

        if safe_range is None:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                "Safe Range를 읽지 못했습니다."
            )

        safe_min_deg = float(safe_range[0])
        safe_max_deg = float(safe_range[1])

        if safe_min_deg >= safe_max_deg:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                "Safe Range가 유효하지 않습니다: "
                f"{safe_min_deg}~{safe_max_deg}"
            )

        ping_result = self.motor.ping_joint(axis.joint)

        if not ping_result.get("success", False):
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                f"Ping 실패: {ping_result}"
            )

        axis.max_speed = int(max_speed)

        if (
            self.pose_max_speed is not None
            and self.pose_max_speed > axis.max_speed
        ):
            raise RuntimeError(
                f"{axis.joint} pose_speed={self.pose_max_speed}가 "
                f"Calibration max_speed={axis.max_speed}보다 큽니다."
            )
    
        axis.safe_min_deg = safe_min_deg
        axis.safe_max_deg = safe_max_deg
        axis.ready = True

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)

        if not self.enabled:
            self._reset_tracking_runtime(reset_reference=True)

        self.latest_state = self._build_state(False)
        return self.enabled

    def close(self):
        self.available = False
        self._reset_ready_state()
        self._reset_tracking_runtime(reset_reference=True)
        self.latest_state = self._build_state(False)

    def _reset_ready_state(self):
        self.motor1.reset_ready()
        self.motor2.reset_ready()

    def _reset_tracking_runtime(self, reset_reference=False):
        """일반 자동추종의 Runtime 상태를 초기화한다."""
        self.last_command_time = 0.0
        self.last_target = None
        self.last_command_speed = 0
        self.last_largest_delta_deg = 0.0
        self.last_success = None
        self.last_user_x_m = None
        self.hold_reason = "IDLE"

        # Motor 제어를 다시 시작할 때는 현재 실제 자세의 Z를
        # 새 기준으로 잡도록 기존 Planner reference를 제거한다.
        if reset_reference and self.planner is not None:
            self.planner.reference_z_m = None
            self.planner.recovery_active = False

    def _calibration_ranges(self):
        """Motor1/2의 현재 Calibration 안전각 범위를 반환한다."""
        if (
            self.motor1.safe_min_deg is None
            or self.motor1.safe_max_deg is None
            or self.motor2.safe_min_deg is None
            or self.motor2.safe_max_deg is None
        ):
            raise RuntimeError("Motor1/2 Safe Range가 준비되지 않았습니다.")

        return {
            MOTOR1_JOINT: (
                float(self.motor1.safe_min_deg),
                float(self.motor1.safe_max_deg),
            ),
            MOTOR2_JOINT: (
                float(self.motor2.safe_min_deg),
                float(self.motor2.safe_max_deg),
            ),
        }

    def _read_current_angles(self):
        """Motor1/2 현재 TEAM 기준 각도를 함께 읽는다."""
        shoulder = self.motor.get_joint_angle(MOTOR1_JOINT)
        elbow = self.motor.get_joint_angle(MOTOR2_JOINT)

        if shoulder is None or elbow is None:
            raise RuntimeError("Servo 1·2 현재 각도를 읽지 못했습니다.")

        return JointCommand(
            shoulder_lift_deg=float(shoulder),
            elbow_flex_deg=float(elbow),
        )

    def read_current_arm_state(self):
        """현재 Motor1/2 관절각과 그 자세의 모니터 위치를 함께 읽는다.

        HardwareProcess의 ToF + Vision 사용자 위치 계산에서는
        Vision 카메라 거리값을 base-user X로 바꾸기 위해 현재 모니터 X가 필요하다.

        Servo 현재각 읽기와 Forward Kinematics는 Motor12/Planner 영역에
        유지하고, HardwareProcess에는 계산된 현재 상태만 전달한다.
        """
        if self.planner is None:
            raise RuntimeError("MonitorArmPlanner가 준비되지 않았습니다.")

        current = self._read_current_angles()
        monitor_pose = self.planner.kinematics.forward(current)

        return current, monitor_pose

    def _normal_tracking_block_reason(self, current):
        """일반 추종으로 움직이면 안 되는 현재 자세인지 검사한다.

        Rest/Recovery처럼 Calibration 또는 정상 작업 Z 범위를 벗어난
        자세는 일반 move_joints() 경로에서 처리하지 않는다.
        """
        if self.planner is None:
            return "PLANNER_NOT_READY"

        calibration_ranges = self._calibration_ranges()

        current_angles = {
            MOTOR1_JOINT: float(current.shoulder_lift_deg),
            MOTOR2_JOINT: float(current.elbow_flex_deg),
        }

        for joint, angle in current_angles.items():
            minimum, maximum = calibration_ranges[joint]

            if not minimum <= angle <= maximum:
                return "RECOVERY_REQUIRED"

        current_pose = self.planner.kinematics.forward(current)

        if not (
            self.planner.working_z_min_m
            <= current_pose.z_m
            <= self.planner.working_z_max_m
        ):
            return "RECOVERY_REQUIRED"

        if self.planner.recovery_active:
            return "RECOVERY_REQUIRED"

        return None

    def _select_tracking_speed(self, current, target):
        """두 Joint 중 더 큰 목표각 오차를 기준으로 공통 Speed를 결정한다."""
        if self.pose_max_speed is None:
            raise RuntimeError("Motor12 모니터암 속도 설정이 준비되지 않았습니다.")

        largest_delta = max(
            abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(target.elbow_flex_deg - current.elbow_flex_deg),
        )

        speed = select_speed(
            self.pose_speed_mode,
            self.pose_max_speed,
            self.pose_min_speed,
            self.pose_full_speed_error_deg,
            largest_delta,
        )

        return speed, largest_delta

    def _move_normal_target(self, target):
        """일반 자동추종 목표를 Motor1/2 SyncWrite로 전송한다.

        Rest/Recovery처럼 Calibration 범위를 벗어나는 특수 자세는
        이 경로로 보내지 않는다. 해당 기능은 별도 예외 경로로 유지한다.
        """
        target_angles = {
            MOTOR1_JOINT: float(target.shoulder_lift_deg),
            MOTOR2_JOINT: float(target.elbow_flex_deg),
        }

        calibration_ranges = self._calibration_ranges()

        # 일반 자동추종에서는 Calibration 안전범위 밖의 목표를 명시적으로 차단한다.
        # Rest/Recovery의 예외 이동과 일반 제어를 섞지 않기 위한 방어선이다.
        for joint, angle in target_angles.items():
            minimum, maximum = calibration_ranges[joint]

            if not minimum <= angle <= maximum:
                raise RuntimeError(
                    f"일반 추종 목표 {joint}={angle:+.2f}°가 "
                    f"Calibration 안전범위 "
                    f"{minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                )

        # 팀원 원본과 동일하게 실제 명령 직전에 현재각을 다시 읽고
        # 두 축 중 큰 오차를 기준으로 하나의 공통 Speed를 선택한다.
        current = self._read_current_angles()
        speed, largest_delta = self._select_tracking_speed(current, target)

        success = self.motor.move_joints(
            target_angles,
            speed=speed,
            acc=self.pose_acc,
            wait=False,
        )

        if not success:
            raise RuntimeError(
                self.motor.last_error
                or "Servo 1·2 동기 이동 명령이 거부되었습니다."
            )

        return {
            "accepted": True,
            "speed": speed,
            "largest_delta_deg": largest_delta,
        }

    def update(self, context):
        """융합 user X를 이용해 Motor1/2 일반 자동추종을 수행한다.

        Controller 상태 계산은 20Hz로 수행하지만 실제 Servo 명령은
        control.command_hz 설정에 따라 제한한다.

        ToF/Fusion 입력이 유효하지 않으면 SAFE_HOLD하고,
        Rest/Recovery가 필요한 자세는 일반 추종 경로에서 움직이지 않는다.
        """
        now = float(
            context.get(
                "now",
                time.monotonic(),
            )
        )

        # Motor12 상태 계산 자체는 20Hz.
        if (
            now - self.last_update_time
            < self.update_interval
        ):
            return self.latest_state

        self.last_update_time = now

        motor12_context = context.get("motor12", {}, )

        if not isinstance(
            motor12_context,
            dict,
        ):
            motor12_context = {}

        input_state = motor12_context.get("input", {}, )

        if not isinstance(
            input_state,
            dict,
        ):
            input_state = {}

        control_requested = bool(
            motor12_context.get(
                "control_active",
                False,
            )
        )

        input_valid = bool(
            input_state.get(
                "valid",
                False,
            )
        )

        user_x_m = input_state.get("user_x_m")

        can_control = bool(
            self.available
            and self.enabled
            and control_requested
            and input_valid
            and user_x_m is not None
            and self.motor.available
            and self.ready
            and self.planner is not None
        )

        # -----------------------------------------------------
        # SAFE HOLD / Disabled / Hardware not ready
        # -----------------------------------------------------
        if not can_control:
            # 다시 유효한 입력이 들어오면 즉시 한 번 명령할 수 있도록
            # command gate를 초기화한다.
            self.last_command_time = 0.0

            if (
                not input_valid
                or user_x_m is None
            ):
                self.hold_reason = "SAFE_HOLD"
                self.last_error = (
                    input_state.get("last_error")
                    or "Motor1/2 사용자 위치 입력이 유효하지 않습니다."
                )

            elif not self.enabled:
                self.hold_reason = "DISABLED"
                self.last_error = None

            else:
                self.hold_reason = "NOT_READY"

            self.latest_state = self._build_state(False)
            return self.latest_state

        try:
            self.last_user_x_m = float(user_x_m)

        except (TypeError, ValueError) as error:
            self.last_success = False
            self.last_error = (f"Motor1/2 user_x 변환 실패: {error}")
            self.hold_reason = "ERROR"
            self.latest_state = self._build_state(False)
            return self.latest_state

        self.hold_reason = None

        # -----------------------------------------------------
        # 실제 Servo 명령은 설정값 command_hz로 제한한다.
        # 현재 설정: 5Hz = 0.2초마다 최대 1회.
        # -----------------------------------------------------
        if (
            now - self.last_command_time
            < self.command_interval
        ):
            self.latest_state = self._build_state(True)
            return self.latest_state

        self.last_command_time = now

        try:
            # Planner 호출 직전에 현재 실제각을 다시 읽는다.
            current = self._read_current_angles()

            # Rest / Recovery가 필요한 위치에서는 일반 추종 이동 금지.
            block_reason = (self._normal_tracking_block_reason(current))

            if block_reason is not None:
                self.last_success = False
                self.hold_reason = block_reason
                self.last_error = (
                    "Motor1/2 현재 자세가 일반 자동추종 범위를 "
                    "벗어나 Recovery가 필요합니다."
                )

                self.latest_state = self._build_state(False)
                return self.latest_state

            target = self.planner.plan(
                current=current,
                user_x_m=self.last_user_x_m,
                calibration_ranges=(self._calibration_ranges()),
            )

            # 사용자와 모니터 사이 X 오차가 deadband 안이면
            # Planner가 None을 반환한다. 정상적인 '이동 불필요' 상태다.
            if target is None:
                self.last_target = None
                self.last_command_speed = 0
                self.last_largest_delta_deg = 0.0
                self.last_success = True
                self.last_error = None
                self.hold_reason = "DEADBAND"

                self.latest_state = self._build_state(True)
                return self.latest_state

            result = self._move_normal_target(target)

            self.last_target = target
            self.last_command_speed = int(result["speed"])
            self.last_largest_delta_deg = float(result["largest_delta_deg"])
            self.last_success = True
            self.last_error = None
            self.hold_reason = None

            self.latest_state = self._build_state(True)
            return self.latest_state

        except Exception as error:
            self.last_success = False
            self.last_error = str(error)
            self.hold_reason = "ERROR"

            self.latest_state = self._build_state(False)
            return self.latest_state

    @staticmethod
    def _axis_state(axis):
        return {
            "servo_id": axis.servo_id,
            "joint": axis.joint,
            "implemented": True,
            "ready": bool(axis.ready),
            "max_speed": axis.max_speed,
            "safe_min_deg": axis.safe_min_deg,
            "safe_max_deg": axis.safe_max_deg,
        }

    def _build_state(self, control_active):
        return {
            "available": bool(self.available),
            "enabled": bool(self.enabled),
            "ready": bool(self.ready),
            "control_active": bool(
                control_active
                and self.available
                and self.enabled
                and self.ready
            ),
            "update_hz": self.update_hz,
            "command_hz": self.command_hz,
            "pose_speed_mode": self.pose_speed_mode,
            "pose_max_speed": self.pose_max_speed,
            "pose_acc": self.pose_acc,
            "user_x_m": self.last_user_x_m,
            "target": (
                None
                if self.last_target is None
                else {
                    MOTOR1_JOINT: float(self.last_target.shoulder_lift_deg),
                    MOTOR2_JOINT: float(self.last_target.elbow_flex_deg),
                }
            ),
            "command_speed": self.last_command_speed,
            "largest_delta_deg": (self.last_largest_delta_deg),
            "last_success": self.last_success,
            "hold_reason": self.hold_reason,
            "last_error": self.last_error,
            "motor1": self._axis_state(self.motor1),
            "motor2": self._axis_state(self.motor2),
            "timestamp": time.time(),
        }
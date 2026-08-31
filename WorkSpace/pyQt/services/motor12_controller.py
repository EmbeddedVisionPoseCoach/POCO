import time
from dataclasses import dataclass

from services.monitor_arm_kinematics import (
    JointCommand,
    MotionSafetyError,
    monitor_target_from_user,
)
from services.monitor_arm_planner import (
    MonitorArmPlanner,
    RecoveryTimeoutError,
)
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

        # Rest는 일반 Calibration Safe Range 밖의 확인된 특수 자세다.
        # 실제 값은 settings 검증 후 아래에서 채운다.
        self.rest_command = None
        self.rest_speed_cap = None
        self.rest_acc_cap = None

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

            # --------------------------------------------------
            # Rest 특수 자세
            # --------------------------------------------------
            rest = (
                settings.get("postures", {})
                .get("rest")
            )

            if not isinstance(rest, dict):
                raise ValueError("Motor1/2 Rest 자세 설정이 없습니다.")

            if (
                "shoulder_lift_deg" not in rest
                or "elbow_flex_deg" not in rest
            ):
                raise ValueError("Motor1/2 Rest Joint 각도가 없습니다.")

            self.rest_command = JointCommand(
                shoulder_lift_deg=float(rest["shoulder_lift_deg"]),
                elbow_flex_deg=float(rest["elbow_flex_deg"]),
            )

            self.rest_speed_cap = int(rest.get("speed_cap", 200))
            self.rest_acc_cap = int(rest.get("acc_cap", 10))

            if self.rest_speed_cap <= 0:
                raise ValueError(
                    "Motor1/2 rest speed_cap은 "
                    "1 이상이어야 합니다."
                )

            if not 0 <= self.rest_acc_cap <= 30:
                raise ValueError(
                    "Motor1/2 rest acc_cap "
                    "허용범위는 0~30입니다."
                )

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

        # Rest에서는 자동추종을 완전히 정지한다.
        # Resume 요청 후에만 Planner의 inward Recovery를 허용한다.
        self.rest_mode = False
        self.last_recovery_special = False
    
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
        self.rest_mode = False

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
        self.rest_mode = False
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
        self.last_recovery_special = False

        # Motor 제어를 다시 시작할 때는 현재 실제 자세의 Z를
        # 새 기준으로 잡도록 기존 Planner reference를 제거한다.
        if reset_reference and self.planner is not None:
            self.planner.reference_z_m = None
            self.planner.cancel_working_pose_recovery()

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

    @staticmethod
    def _outside_distance(
        value,
        minimum,
        maximum,
    ):
        """값이 안전범위 밖에 얼마나 벗어났는지 반환한다."""
        value = float(value)

        if value < minimum:
            return minimum - value

        if value > maximum:
            return value - maximum

        return 0.0

    def _rest_speed_acc(self):
        """Rest 이동의 보수적인 Speed/Acc를 결정한다."""
        if (
            self.rest_command is None
            or self.rest_speed_cap is None
            or self.rest_acc_cap is None
        ):
            raise RuntimeError(
                "Motor1/2 Rest 설정이 준비되지 않았습니다."
            )

        if (
            self.motor1.max_speed is None
            or self.motor2.max_speed is None
        ):
            raise RuntimeError(
                "Motor1/2 max_speed가 준비되지 않았습니다."
            )

        speed_candidates = [
            int(self.rest_speed_cap),
            int(self.motor1.max_speed),
            int(self.motor2.max_speed),
        ]

        if self.pose_max_speed is not None:
            speed_candidates.append(
                int(self.pose_max_speed)
            )

        speed = max(
            1,
            min(speed_candidates),
        )

        pose_acc = (
            int(self.rest_acc_cap)
            if self.pose_acc is None
            else int(self.pose_acc)
        )

        acc = min(
            int(self.rest_acc_cap),
            pose_acc,
        )

        return speed, acc

    def _move_recovery_target(
        self,
        current,
        target,
    ):
        """Recovery 목표를 안전범위 방향으로만 이동시킨다.

        목표가 아직 Calibration 밖이면 12-A에서 만든 특수 SyncWrite를
        사용한다. 목표가 다시 Calibration 안으로 들어온 순간부터는
        일반 move_joints() 경로로 자동 복귀한다.
        """
        target_angles = {
            MOTOR1_JOINT: float(
                target.shoulder_lift_deg
            ),
            MOTOR2_JOINT: float(
                target.elbow_flex_deg
            ),
        }

        current_angles = {
            MOTOR1_JOINT: float(
                current.shoulder_lift_deg
            ),
            MOTOR2_JOINT: float(
                current.elbow_flex_deg
            ),
        }

        calibration_ranges = (self._calibration_ranges())

        needs_special = False

        for joint, target_angle in (target_angles.items()):
            minimum, maximum = (calibration_ranges[joint])

            current_outside = (
                self._outside_distance(
                    current_angles[joint],
                    minimum,
                    maximum,
                )
            )

            target_outside = (
                self._outside_distance(
                    target_angle,
                    minimum,
                    maximum,
                )
            )

            if target_outside > 1e-6:
                needs_special = True

                # Calibration 밖의 목표는 반드시 현재보다
                # 안전범위 쪽으로 가까워지는 경우만 허용한다.
                if (
                    current_outside <= 1e-6
                    or target_outside
                    >= current_outside - 1e-6
                ):
                    raise RuntimeError(
                        f"{joint} Recovery 목표가 "
                        "Calibration 안전범위 쪽으로 "
                        "가까워지지 않습니다."
                    )

        speed, largest_delta = (
            self._select_tracking_speed(
                current,
                target,
            )
        )

        if needs_special:
            success = (
                self.motor.move_joints_special(
                    target_angles,
                    speed=speed,
                    acc=self.pose_acc,
                    wait=False,
                )
            )
        else:
            success = self.motor.move_joints(
                target_angles,
                speed=speed,
                acc=self.pose_acc,
                wait=False,
            )

        if not success:
            raise RuntimeError(
                self.motor.last_error
                or "Servo 1·2 Recovery "
                "SyncWrite가 거부되었습니다."
            )

        return {
            "accepted": True,
            "speed": speed,
            "largest_delta_deg": (
                largest_delta
            ),
            "special": needs_special,
        }

    def move_manual_user_target(
        self,
        user_x_m,
        user_monitor_distance_m,
        monitor_z_m,
    ):
        """초기 준비 UI의 사용자 X/Z 입력을 한 번의 안전한 IK 명령으로 보낸다."""
        if not (
            self.available
            and self.enabled
            and self.motor.available
            and self.ready
            and self.planner is not None
        ):
            return {
                "accepted": False,
                "error": "Motor1/2 수동 IK를 실행할 준비가 되지 않았습니다.",
            }

        try:
            current = self._read_current_angles()
            block_reason = self._normal_tracking_block_reason(current)
            if block_reason is not None:
                raise RuntimeError(
                    "현재 자세는 수동 IK 안전범위 밖입니다. "
                    "먼저 '휴식 → 작업 시작 위치 이동'을 완료해주세요."
                )

            monitor_z_m = float(monitor_z_m)
            requested_pose = monitor_target_from_user(
                user_x_m=float(user_x_m),
                user_monitor_distance_m=float(user_monitor_distance_m),
                monitor_z_m=monitor_z_m,
            )
            target = self.planner.kinematics.inverse(
                requested_pose.x_m,
                requested_pose.z_m,
            )
            self.planner.kinematics.validate_motion(
                current=current,
                target=target,
                reference_z_m=monitor_z_m,
                limits=self.planner.limits,
                calibration_ranges=self._calibration_ranges(),
                enforce_step_limit=False,
            )
            result = self._move_normal_target(target)
            self.rest_mode = False
            self.planner.reference_z_m = monitor_z_m
            self.planner.cancel_working_pose_recovery()
            self.last_target = target
            self.last_command_speed = int(result["speed"])
            self.last_largest_delta_deg = float(result["largest_delta_deg"])
            self.last_recovery_special = False
            self.last_success = True
            self.last_error = None
            self.hold_reason = "MANUAL_PREPARATION"
            return {
                "accepted": True,
                "target": {
                    MOTOR1_JOINT: float(target.shoulder_lift_deg),
                    MOTOR2_JOINT: float(target.elbow_flex_deg),
                },
                "monitor_pose": {
                    "x_m": float(requested_pose.x_m),
                    "z_m": float(requested_pose.z_m),
                },
                **result,
            }
        except Exception as error:
            self.last_success = False
            self.last_error = str(error)
            self.hold_reason = "MANUAL_PREPARATION_ERROR"
            return {"accepted": False, "error": str(error)}

    def move_to_rest(self):
        """확인된 Rest 특수 자세로 Motor1/2를 동시에 이동한다.

        Rest 진입 후에는 일반 ToF/Vision 자동추종을 완전히 정지한다.
        Recovery는 resume_from_rest()가 명시적으로 호출된 뒤에만 시작한다.
        """
        if (
            self.rest_command is None
            or self.planner is None
        ):
            return {
                "accepted": False,
                "error": (
                    "Motor1/2 Rest 설정이 "
                    "준비되지 않았습니다."
                ),
            }

        if not (
            self.available
            and self.enabled
            and self.motor.available
            and self.ready
        ):
            return {
                "accepted": False,
                "error": (
                    "Motor1/2가 Rest 이동 가능한 "
                    "상태가 아닙니다."
                ),
            }

        # 같은 Rest 요청이 반복되어도 새 명령을 중복 전송하지 않는다.
        if (
            self.rest_mode
            and self.last_success is True
        ):
            return {
                "accepted": True,
                "already_rest": True,
            }

        # Rest 명령 실패 시에도 일반 자동추종이 즉시 재개되지 않도록
        # 먼저 Rest latch를 켠다.
        self.rest_mode = True
        self._reset_tracking_runtime(reset_reference=False)
        self.hold_reason = "REST"
        self.last_error = None

        try:
            speed, acc = (self._rest_speed_acc())

            targets = {
                MOTOR1_JOINT: float(
                    self.rest_command
                    .shoulder_lift_deg
                ),
                MOTOR2_JOINT: float(
                    self.rest_command
                    .elbow_flex_deg
                ),
            }

            success = (
                self.motor.move_joints_special(
                    targets,
                    speed=speed,
                    acc=acc,
                    wait=False,
                )
            )

            if not success:
                raise RuntimeError(
                    self.motor.last_error
                    or "Servo 1·2 Rest "
                    "SyncWrite가 거부되었습니다."
                )

            # Rest 자세에서는 자동추종을 멈춰두지만,
            # Resume 시 작업자세로 복구할 수 있도록 Planner를 latch한다.
            self.planner.request_working_pose_recovery()

            self.last_target = (self.rest_command)
            self.last_command_speed = speed
            self.last_largest_delta_deg = 0.0
            self.last_recovery_special = True
            self.last_success = True
            self.last_error = None
            self.hold_reason = "REST"

            self.latest_state = (self._build_state(False))

            return {
                "accepted": True,
                "speed": speed,
                "acc": acc,
            }

        except Exception as error:
            self.last_success = False
            self.last_error = str(error)
            self.hold_reason = "REST_ERROR"

            # 실패해도 Rest latch는 유지한다.
            # 사용자가 상태를 확인한 뒤 명시적으로 Resume해야 한다.
            self.latest_state = (self._build_state(False))

            return {
                "accepted": False,
                "error": str(error),
            }

    def resume_from_rest(self):
        """Rest 또는 Recovery-required 자세에서 작업자세 복구를 시작한다.

        이 함수 자체는 Servo 이동 명령을 보내지 않는다.
        다음 update()부터 Planner의 inward-only Recovery가 5Hz로 실행된다.
        """
        if self.planner is None:
            return {
                "accepted": False,
                "error": (
                    "MonitorArmPlanner가 "
                    "준비되지 않았습니다."
                ),
            }

        if not (
            self.available
            and self.enabled
            and self.motor.available
            and self.ready
        ):
            return {
                "accepted": False,
                "error": (
                    "Motor1/2가 Recovery를 시작할 "
                    "수 있는 상태가 아닙니다."
                ),
            }

        try:
            current = (self._read_current_angles())

        except Exception as error:
            self.last_error = str(error)
            self.hold_reason = ("RECOVERY_ERROR")

            self.latest_state = (self._build_state(False))

            return {
                "accepted": False,
                "error": str(error),
            }

        self.rest_mode = False

        self._reset_tracking_runtime(reset_reference=False)

        # 현재 위치가 Rest 또는 Calibration 밖이어도
        # Planner가 5°씩 안전범위 안쪽으로 복구하도록 명시적으로 요청한다.
        self.planner.request_working_pose_recovery()

        self.hold_reason = "RECOVERY"

        self.latest_state = (self._build_state(False))

        return {
            "accepted": True,
            "current": {
                MOTOR1_JOINT: float(
                    current.shoulder_lift_deg
                ),
                MOTOR2_JOINT: float(
                    current.elbow_flex_deg
                ),
            },
        }

    def update(self, context):
        """융합 user X를 이용해 Motor1/2 자동추종/Recovery를 수행한다.

        상태 계산은 20Hz, 실제 Servo 명령은 command_hz(현재 5Hz)로 제한한다.

        제어 우선순위:
        1. Rest mode -> 완전 HOLD
        2. ToF/Fusion invalid -> SAFE_HOLD
        3. 명시적으로 요청된 Recovery -> inward-only 복구
        4. 정상 자세 -> 일반 ToF/Vision X 추종
        """
        now = float(
            context.get(
                "now",
                time.monotonic(),
            )
        )

        if (
            now - self.last_update_time
            < self.update_interval
        ):
            return self.latest_state

        self.last_update_time = now

        motor12_context = context.get(
            "motor12",
            {},
        )

        if not isinstance(
            motor12_context,
            dict,
        ):
            motor12_context = {}

        input_state = motor12_context.get(
            "input",
            {},
        )

        if not isinstance(
            input_state,
            dict,
        ):
            input_state = {}

        # -----------------------------------------------------
        # Rest mode
        # -----------------------------------------------------
        if self.rest_mode:
            self.last_command_time = 0.0

            if self.hold_reason != "REST_ERROR":
                self.hold_reason = "REST"

            self.latest_state = (
                self._build_state(False)
            )
            return self.latest_state

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

        user_x_m = input_state.get(
            "user_x_m"
        )

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
            self.last_command_time = 0.0

            if (
                not input_valid
                or user_x_m is None
            ):
                self.hold_reason = "SAFE_HOLD"
                self.last_error = (
                    input_state.get("last_error")
                    or "Motor1/2 사용자 위치 입력이 "
                    "유효하지 않습니다."
                )

            elif not self.enabled:
                self.hold_reason = "DISABLED"
                self.last_error = None

            else:
                self.hold_reason = "NOT_READY"

            self.latest_state = (
                self._build_state(False)
            )
            return self.latest_state

        try:
            self.last_user_x_m = float(
                user_x_m
            )

        except (TypeError, ValueError) as error:
            self.last_success = False
            self.last_error = (
                f"Motor1/2 user_x 변환 실패: "
                f"{error}"
            )
            self.hold_reason = "ERROR"

            self.latest_state = (
                self._build_state(False)
            )
            return self.latest_state

        # 실제 Servo 명령은 5Hz gate.
        if (
            now - self.last_command_time
            < self.command_interval
        ):
            self.latest_state = (
                self._build_state(True)
            )
            return self.latest_state

        self.last_command_time = now
        recovery_was_active = bool(
            self.planner is not None and self.planner.recovery_active
        )

        try:
            current = (
                self._read_current_angles()
            )

            # -------------------------------------------------
            # Explicit Recovery
            # -------------------------------------------------
            if self.planner.recovery_active:
                target = self.planner.plan(
                    current=current,
                    user_x_m=self.last_user_x_m,
                    calibration_ranges=(
                        self._calibration_ranges()
                    ),
                )

                # 작업자세에 충분히 가까워지면 Planner가 Recovery latch를
                # 해제하고 None을 반환한다.
                if target is None:
                    if self.planner.recovery_active:
                        self.last_target = None
                        self.last_command_speed = 0
                        self.last_largest_delta_deg = float(
                            self.planner.recovery_largest_error_deg or 0.0
                        )
                        self.last_recovery_special = False
                        self.last_success = True
                        self.last_error = None
                        self.hold_reason = "RECOVERY_STABILIZING"
                        self.latest_state = self._build_state(True)
                        return self.latest_state

                    self.last_target = None
                    self.last_command_speed = 0
                    self.last_largest_delta_deg = 0.0
                    self.last_recovery_special = False
                    self.last_success = True
                    self.last_error = None
                    self.hold_reason = (
                        "RECOVERY_COMPLETE"
                    )

                    self.latest_state = (
                        self._build_state(True)
                    )
                    return self.latest_state

                result = (
                    self._move_recovery_target(
                        current,
                        target,
                    )
                )

                self.last_target = target
                self.last_command_speed = int(
                    result["speed"]
                )
                self.last_largest_delta_deg = float(
                    result[
                        "largest_delta_deg"
                    ]
                )
                self.last_recovery_special = bool(
                    result["special"]
                )
                self.last_success = True
                self.last_error = None
                self.hold_reason = "RECOVERY"

                self.latest_state = (
                    self._build_state(True)
                )
                return self.latest_state

            # -------------------------------------------------
            # Normal tracking
            # -------------------------------------------------
            block_reason = (
                self._normal_tracking_block_reason(
                    current
                )
            )

            if block_reason is not None:
                self.last_success = False
                self.last_recovery_special = False
                self.hold_reason = block_reason
                self.last_error = (
                    "Motor1/2 현재 자세가 일반 "
                    "자동추종 범위를 벗어나 "
                    "Recovery가 필요합니다."
                )

                self.latest_state = (
                    self._build_state(False)
                )
                return self.latest_state

            target = self.planner.plan(
                current=current,
                user_x_m=self.last_user_x_m,
                calibration_ranges=(
                    self._calibration_ranges()
                ),
            )

            if target is None:
                self.last_target = None
                self.last_command_speed = 0
                self.last_largest_delta_deg = 0.0
                self.last_recovery_special = False
                self.last_success = True
                self.last_error = None
                self.hold_reason = "DEADBAND"

                self.latest_state = (
                    self._build_state(True)
                )
                return self.latest_state

            result = self._move_normal_target(
                target
            )

            self.last_target = target
            self.last_command_speed = int(
                result["speed"]
            )
            self.last_largest_delta_deg = float(
                result["largest_delta_deg"]
            )
            self.last_recovery_special = False
            self.last_success = True
            self.last_error = None
            self.hold_reason = None

            self.latest_state = (
                self._build_state(True)
            )
            return self.latest_state

        except Exception as error:
            self.last_success = False
            self.last_error = str(error)

            if recovery_was_active:
                self.last_largest_delta_deg = (
                    self.planner.recovery_largest_error_deg or 0.0
                )
                self.planner.cancel_working_pose_recovery()
                if isinstance(error, RecoveryTimeoutError):
                    self.hold_reason = "RECOVERY_TIMEOUT"
                elif isinstance(error, MotionSafetyError):
                    self.hold_reason = "RECOVERY_SAFETY_ERROR"
                else:
                    self.hold_reason = "RECOVERY_COMMAND_ERROR"
            else:
                self.hold_reason = "ERROR"

            self.latest_state = (
                self._build_state(False)
            )
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
            "rest_mode": bool(self.rest_mode),
            "recovery_active": bool(
                self.planner is not None
                and self.planner.recovery_active
            ),
            "recovery_goal_error_deg": (
                None
                if self.planner is None
                else self.planner.recovery_largest_error_deg
            ),
            "recovery_stable_samples": (
                0
                if self.planner is None
                else self.planner.recovery_stable_sample_count
            ),
            "recovery_required_stable_samples": (
                None
                if self.planner is None
                else self.planner.working_start_stable_samples
            ),
            "recovery_arrival_tolerance_deg": (
                None
                if self.planner is None
                else self.planner.working_start_arrival_tolerance_deg
            ),
            "recovery_timeout_sec": (
                None
                if self.planner is None
                else self.planner.working_start_timeout_sec
            ),
            "recovery_special_move": bool(self.last_recovery_special),
            "rest_target": (
                None
                if self.rest_command is None
                else {
                    MOTOR1_JOINT: float(
                        self.rest_command
                        .shoulder_lift_deg
                    ),
                    MOTOR2_JOINT: float(
                        self.rest_command
                        .elbow_flex_deg
                    ),
                }
            ),
            "rest_speed_cap": (self.rest_speed_cap),
            "rest_acc_cap": (self.rest_acc_cap),
            "last_error": self.last_error,
            "motor1": self._axis_state(self.motor1),
            "motor2": self._axis_state(self.motor2),
            "timestamp": time.time(),
        }

import time
from dataclasses import dataclass


MOTOR3_SERVO_ID = 3
MOTOR3_JOINT = "wrist_flex"
MOTOR3_AXIS = "roll"      # 실제 장착 기준: 좌/우

MOTOR4_SERVO_ID = 4
MOTOR4_JOINT = "wrist_roll"
MOTOR4_AXIS = "pitch"     # 실제 장착 기준: 위/아래

MOTOR_COMMAND_HZ = 10.0
MOTOR_PID_SPEED_DEADBAND_DEG_S = 0.25
MOTOR3_DIRECTION_SIGN = +1.0
MOTOR4_DIRECTION_SIGN = +1.0


@dataclass
class _AxisMotorRuntime:
    servo_id: int
    joint: str
    axis: str
    config_enabled: bool = True
    direction_sign: float = 1.0
    ready: bool = False
    max_speed: int | None = None
    safe_min_deg: float | None = None
    safe_max_deg: float | None = None
    target_angle_deg: float | None = None
    last_command_angle_deg: float | None = None
    last_command_speed: int = 0
    last_success: bool | None = None
    last_control_time: float | None = None
    debug_last_print_time: float = 0.0

    def reset_runtime(self):
        self.target_angle_deg = None
        self.last_command_angle_deg = None
        self.last_command_speed = 0
        self.last_success = None
        self.last_control_time = None
        self.debug_last_print_time = 0.0


class Motor34Controller:
    """Motor 3 / 4 짐벌 제어 로직.

    실제 장착 기준
    -------------
    - Motor3 / Servo3 / wrist_flex : 좌/우 -> IMU Roll PID 사용
    - Motor4 / Servo4 / wrist_roll : 위/아래 -> IMU Pitch PID 사용

    주의
    ----
    - joint 이름은 Servo Calibration 파일의 기존 이름을 그대로 사용한다.
    - 실제 Serial/MotorController 소유권은 MotorService 하나만 가진다.
    - PID 출력이 deadband 안이면 현재각 Read와 Move packet을 모두 보내지 않는다.
    """

    def __init__(self, motor_service):
        self.motor = motor_service
        self.enabled = True
        self.available = False

        self.command_hz = MOTOR_COMMAND_HZ
        self.command_interval = 1.0 / self.command_hz
        self.pid_speed_deadband_deg_s = MOTOR_PID_SPEED_DEADBAND_DEG_S
        self.last_command_time = 0.0
        self.last_error = None
        self._debug_last_control_active = False

        self.motor3 = _AxisMotorRuntime(
            servo_id=MOTOR3_SERVO_ID,
            joint=MOTOR3_JOINT,
            axis=MOTOR3_AXIS,
            config_enabled=True,
            direction_sign=MOTOR3_DIRECTION_SIGN,
        )
        self.motor4 = _AxisMotorRuntime(
            servo_id=MOTOR4_SERVO_ID,
            joint=MOTOR4_JOINT,
            axis=MOTOR4_AXIS,
            config_enabled=True,
            direction_sign=MOTOR4_DIRECTION_SIGN,
        )

        self.latest_state = self._build_state(False)

    # 기존 코드/외부 참조 호환용 property
    @property
    def motor3_config_enabled(self):
        return self.motor3.config_enabled

    @property
    def motor4_config_enabled(self):
        return self.motor4.config_enabled

    @property
    def motor3_direction_sign(self):
        return self.motor3.direction_sign

    @property
    def motor4_direction_sign(self):
        return self.motor4.direction_sign

    @property
    def ready(self):
        if not self.available or not self.motor.available:
            return False

        enabled_axes = [
            axis for axis in (self.motor3, self.motor4)
            if axis.config_enabled
        ]
        if not enabled_axes:
            return False

        return all(axis.ready for axis in enabled_axes)

    def initialize(self):
        """활성화된 Motor3/4의 Calibration + 실제 Servo Ping을 확인한다."""
        self.available = False
        self.last_error = None
        self._reset_runtime_targets()

        if not self.motor.available:
            self.last_error = "MotorService가 준비되지 않았습니다."
            return False

        try:
            checked = []
            for axis in (self.motor3, self.motor4):
                axis.ready = False
                axis.max_speed = None
                axis.safe_min_deg = None
                axis.safe_max_deg = None

                if not axis.config_enabled:
                    continue

                self._initialize_axis(axis)
                checked.append(
                    f"Servo{axis.servo_id}={axis.joint}({axis.axis.upper()}, PING=OK) "
                    f"safe={axis.safe_min_deg:+.2f}~{axis.safe_max_deg:+.2f}deg "
                    f"max_speed={axis.max_speed}"
                )

            if not checked:
                raise RuntimeError("Motor3/4가 모두 Disabled 상태입니다.")

            self.available = True
            self.last_error = None
            print("[MOTOR34] 준비 완료 " + " / ".join(checked))
            return True

        except Exception as error:
            self.available = False
            self.last_error = str(error)
            self._reset_runtime_targets()
            print(f"[MOTOR34] 초기화/안전검사 실패: {error}")
            return False

    def _initialize_axis(self, axis):
        servo = self.motor.get_joint_metadata(axis.joint)
        if not servo:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) Calibration 정보가 없습니다."
            )

        actual_servo_id = int(servo.get("servo_id", -1))
        if actual_servo_id != axis.servo_id:
            raise RuntimeError(
                f"{axis.joint} Servo ID 불일치: expected={axis.servo_id}, "
                f"actual={actual_servo_id}"
            )

        max_speed = self.motor.get_max_speed(axis.joint)
        if max_speed is None or max_speed <= 0:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) max_speed가 설정되지 않았습니다."
            )

        safe_range = self.motor.get_safe_angle_range(axis.joint)
        if safe_range is None:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) Safe Range를 읽지 못했습니다."
            )

        ping_result = self.motor.ping_joint(axis.joint)
        if not ping_result.get("success", False):
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) Ping 실패: {ping_result}"
            )

        axis.max_speed = int(max_speed)
        axis.safe_min_deg = float(safe_range[0])
        axis.safe_max_deg = float(safe_range[1])
        axis.ready = True

    def apply_control_config(self, motor_config):
        motor_config = motor_config if isinstance(motor_config, dict) else {}

        self.command_hz = max(1.0, float(motor_config.get("command_hz", self.command_hz)))
        self.command_interval = 1.0 / self.command_hz
        self.pid_speed_deadband_deg_s = max(
            0.0,
            float(
                motor_config.get(
                    "pid_speed_deadband_deg_s",
                    self.pid_speed_deadband_deg_s,
                )
            ),
        )

        self._apply_axis_config(self.motor3, motor_config.get("motor3", {}))
        self._apply_axis_config(self.motor4, motor_config.get("motor4", {}))

        self._reset_runtime_targets()
        return True

    @staticmethod
    def _apply_axis_config(axis, config):
        config = config if isinstance(config, dict) else {}
        axis.config_enabled = bool(config.get("enabled", axis.config_enabled))
        sign = float(config.get("direction_sign", axis.direction_sign))
        axis.direction_sign = 1.0 if sign >= 0.0 else -1.0

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not self.enabled:
            self._reset_runtime_targets()
        return self.enabled

    def close(self):
        self.available = False
        self.motor3.ready = False
        self.motor4.ready = False
        self._reset_runtime_targets()

    def _reset_runtime_targets(self):
        self.motor3.reset_runtime()
        self.motor4.reset_runtime()
        self.last_command_time = 0.0

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def _pid_deg_s_to_servo_speed(self, correction_speed_deg_s, pid_limit_deg_s, max_speed):
        magnitude = abs(float(correction_speed_deg_s))
        limit = max(abs(float(pid_limit_deg_s)), 1e-6)

        if magnitude <= self.pid_speed_deadband_deg_s:
            return 0

        ratio = self._clamp(magnitude / limit, 0.0, 1.0)
        speed = int(round(ratio * int(max_speed)))
        return max(1, min(speed, int(max_speed)))

    def _ensure_target(self, axis):
        if axis.target_angle_deg is not None:
            return True

        current_angle = self.motor.get_joint_angle(axis.joint)
        if current_angle is None:
            self.last_error = (
                f"Servo {axis.servo_id}({axis.joint}) 현재 각도를 읽지 못했습니다."
            )
            return False

        axis.target_angle_deg = float(current_angle)
        axis.last_command_angle_deg = float(current_angle)
        axis.last_control_time = None
        print(
            f"[MOTOR{axis.servo_id}] 시작 기준각={axis.target_angle_deg:+.2f}deg "
            f"axis={axis.axis.upper()}"
        )
        return True

    def _control_axis(self, axis, correction_speed_deg_s, pid_limit_deg_s, now):
        if not axis.config_enabled:
            return True
        if not axis.ready:
            axis.last_success = False
            self.last_error = f"Motor{axis.servo_id}가 준비되지 않았습니다."
            return False

        correction_speed = float(correction_speed_deg_s) * axis.direction_sign
        command_speed = self._pid_deg_s_to_servo_speed(
            correction_speed,
            pid_limit_deg_s,
            axis.max_speed,
        )

        # 0이면 현재각 Read까지 포함해 Servo packet을 전혀 보내지 않는다.
        if command_speed <= 0:
            axis.last_command_speed = 0
            axis.last_success = True
            if now - axis.debug_last_print_time >= 1.0:
                axis.debug_last_print_time = now
                print(
                    f"[MOTOR{axis.servo_id}] HOLD(no packet) "
                    f"axis={axis.axis.upper()} "
                    f"pid_speed={correction_speed:+.3f}deg/s "
                    f"deadband={self.pid_speed_deadband_deg_s:.3f}deg/s"
                )
            return True

        if not self._ensure_target(axis):
            axis.last_success = False
            return False

        if axis.last_control_time is None:
            dt = self.command_interval
        else:
            dt = max(0.0, now - axis.last_control_time)
        dt = min(dt, self.command_interval * 2.0)
        axis.last_control_time = now

        next_target = axis.target_angle_deg + correction_speed * dt
        next_target = self._clamp(
            next_target,
            axis.safe_min_deg,
            axis.safe_max_deg,
        )

        success = self.motor.move_joint(
            axis.joint,
            angle=next_target,
            speed=command_speed,
            wait=False,
        )

        if now - axis.debug_last_print_time >= 0.5:
            axis.debug_last_print_time = now
            print(
                f"[MOTOR{axis.servo_id}] CMD "
                f"axis={axis.axis.upper()} "
                f"pid_speed={correction_speed:+.3f}deg/s "
                f"target={next_target:+.2f}deg "
                f"servo_speed={command_speed} success={bool(success)}"
            )

        axis.last_success = bool(success)
        axis.last_command_speed = int(command_speed)

        if success:
            axis.target_angle_deg = float(next_target)
            axis.last_command_angle_deg = float(next_target)
            self.last_error = None
        else:
            self.last_error = (
                f"Servo {axis.servo_id} 이동 명령 실패 "
                f"target={next_target:.2f} speed={command_speed}"
            )

        return bool(success)

    def _control_motor3(self, correction_roll_speed_deg_s, pid_limit_deg_s, now):
        # Motor3 = 좌/우 = Roll PID
        return self._control_axis(
            self.motor3,
            correction_roll_speed_deg_s,
            pid_limit_deg_s,
            now,
        )

    def _control_motor4(self, correction_pitch_speed_deg_s, pid_limit_deg_s, now):
        # Motor4 = 위/아래 = Pitch PID
        return self._control_axis(
            self.motor4,
            correction_pitch_speed_deg_s,
            pid_limit_deg_s,
            now,
        )

    def update(self, context):
        now = float(context.get("now", time.monotonic()))
        imu_state = context.get("imu", {})
        control_active = bool(context.get("motor34_control_active", False))
        pid_limit_deg_s = float(context.get("pid_limit_deg_s", 30.0))

        imu_ready = bool(
            imu_state.get("available", False)
            and imu_state.get("calibrated", False)
            and not imu_state.get("calibrating", False)
        )

        can_control = bool(
            self.available
            and self.enabled
            and control_active
            and imu_ready
            and self.motor.available
            and self.ready
        )

        if not can_control:
            if self._debug_last_control_active:
                print(
                    "[MOTOR34] GIMBAL OFF "
                    f"available={self.available} enabled={self.enabled} "
                    f"requested={control_active} imu_ready={imu_ready} "
                    f"motor3_ready={self.motor3.ready} motor4_ready={self.motor4.ready}"
                )
            self._debug_last_control_active = False
            if not control_active:
                self._reset_runtime_targets()
            self.latest_state = self._build_state(False)
            return self.latest_state

        if not self._debug_last_control_active:
            print(
                "[MOTOR34] GIMBAL ON "
                f"M3(ROLL)={float(imu_state.get('roll_deg', 0.0)):+.2f}deg/"
                f"{float(imu_state.get('correction_roll_speed_deg_s', 0.0)):+.3f}deg/s "
                f"M4(PITCH)={float(imu_state.get('pitch_deg', 0.0)):+.2f}deg/"
                f"{float(imu_state.get('correction_pitch_speed_deg_s', 0.0)):+.3f}deg/s"
            )
        self._debug_last_control_active = True

        if now - self.last_command_time >= self.command_interval:
            self.last_command_time = now

            # 실제 장착 방향 기준 순서
            # Motor3: 좌/우 <- Roll PID
            # Motor4: 위/아래 <- Pitch PID
            self._control_motor3(
                imu_state.get("correction_roll_speed_deg_s", 0.0),
                pid_limit_deg_s,
                now,
            )
            self._control_motor4(
                imu_state.get("correction_pitch_speed_deg_s", 0.0),
                pid_limit_deg_s,
                now,
            )

        self.latest_state = self._build_state(True)
        return self.latest_state

    def _axis_state(self, axis):
        return {
            "servo_id": axis.servo_id,
            "joint": axis.joint,
            "axis": axis.axis,
            "implemented": True,
            "ready": bool(axis.ready),
            "config_enabled": bool(axis.config_enabled),
            "direction_sign": axis.direction_sign,
            "max_speed": axis.max_speed,
            "safe_min_deg": axis.safe_min_deg,
            "safe_max_deg": axis.safe_max_deg,
            "target_angle_deg": axis.target_angle_deg,
            "command_speed": axis.last_command_speed,
            "last_success": axis.last_success,
        }

    def _build_state(self, control_active):
        return {
            "available": bool(self.available),
            "enabled": bool(self.enabled),
            "ready": bool(self.ready),
            "motor3_config_enabled": bool(self.motor3.config_enabled),
            "motor4_config_enabled": bool(self.motor4.config_enabled),
            "control_active": bool(
                control_active and self.available and self.enabled and self.ready
            ),
            "axis_mapping": {
                "motor3": "roll",
                "motor4": "pitch",
            },
            "last_error": self.last_error,
            "motor3": self._axis_state(self.motor3),
            "motor4": self._axis_state(self.motor4),
            "timestamp": time.time(),
        }

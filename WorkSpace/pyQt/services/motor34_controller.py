import time
from dataclasses import dataclass

from services.hardware_constants import (
    MOTOR34_AUTO_ACC,
    MOTOR34_AUTO_SPEED,
    MOTOR34_COMMAND_HZ,
    MOTOR3_IMU_Y_DIRECTION_SIGN,
    MOTOR4_IMU_X_DIRECTION_SIGN,
)


MOTOR3_SERVO_ID = 3
MOTOR3_JOINT = "wrist_flex"
MOTOR3_AXIS = "imu_y"

MOTOR4_SERVO_ID = 4
MOTOR4_JOINT = "wrist_roll"
MOTOR4_AXIS = "imu_x"


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
    last_command_acc: int = 0
    last_pid_output_deg_s: float = 0.0
    last_success: bool | None = None
    last_control_time: float | None = None
    debug_last_print_time: float = 0.0

    def reset_runtime(self):
        self.target_angle_deg = None
        self.last_command_angle_deg = None
        self.last_command_speed = 0
        self.last_command_acc = 0
        self.last_pid_output_deg_s = 0.0
        self.last_success = None
        self.last_control_time = None
        self.debug_last_print_time = 0.0


class Motor34Controller:
    """Motor3 / Motor4 Direct IMU 짐벌 제어.

    검증된 imu_xy_direct_pid_tuner V1.9 제어를 POCO 구조에 이식한다.

    매핑
    ----
    - Motor3 / wrist_flex <- IMU Y PID output [deg/s]
    - Motor4 / wrist_roll <- IMU X PID output [deg/s]

    핵심
    ----
    1. PID output 자체가 target velocity [deg/s]다.
    2. TEAM target += direction_sign * pid_output * dt
       (POCO MotorController 내부에서 TEAM -> URDF 방향변환 -1이 추가 적용됨)
    3. Servo Speed를 PID 비율로 다시 줄이지 않는다.
    4. STS3215 Speed/Acc는 충분히 높은 고정값을 사용한다.
    5. 최종 target은 servo_calibration_result.json의 safe angle 범위로 clamp한다.
    6. 실제 Serial 소유권은 MotorService 하나만 가진다.
    """

    def __init__(self, motor_service):
        self.motor = motor_service
        self.enabled = True
        self.available = False

        self.command_hz = MOTOR34_COMMAND_HZ
        self.command_interval = 1.0 / self.command_hz
        self.auto_speed = MOTOR34_AUTO_SPEED
        self.auto_acc = MOTOR34_AUTO_ACC
        self.last_command_time = 0.0
        self.last_error = None
        self._debug_last_control_active = False

        self.motor3 = _AxisMotorRuntime(
            servo_id=MOTOR3_SERVO_ID,
            joint=MOTOR3_JOINT,
            axis=MOTOR3_AXIS,
            config_enabled=True,
            direction_sign=MOTOR3_IMU_Y_DIRECTION_SIGN,
        )
        self.motor4 = _AxisMotorRuntime(
            servo_id=MOTOR4_SERVO_ID,
            joint=MOTOR4_JOINT,
            axis=MOTOR4_AXIS,
            config_enabled=True,
            direction_sign=MOTOR4_IMU_X_DIRECTION_SIGN,
        )

        self.latest_state = self._build_state(False)

    # 기존 외부 참조 호환 property
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
            axis
            for axis in (self.motor3, self.motor4)
            if axis.config_enabled
        ]
        if not enabled_axes:
            return False

        return all(axis.ready for axis in enabled_axes)

    def initialize(self):
        """활성화된 Motor3/4 Calibration + Servo Ping 확인."""
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
            print("[MOTOR34] Direct IMU 준비 완료 " + " / ".join(checked))
            print(
                "[MOTOR34 CONTROL] "
                f"command={self.command_hz:.0f}Hz "
                f"speed={self.auto_speed} acc={self.auto_acc} "
                f"M3<-Y TEAMsign={self.motor3.direction_sign:+.0f} "
                f"M4<-X TEAMsign={self.motor4.direction_sign:+.0f}"
            )
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
        """Direct IMU Motor3/4 runtime 설정 반영."""
        motor_config = motor_config if isinstance(motor_config, dict) else {}

        self.command_hz = max(
            1.0,
            float(motor_config.get("command_hz", self.command_hz)),
        )
        self.command_interval = 1.0 / self.command_hz
        self.auto_speed = max(
            1,
            int(motor_config.get("auto_speed", self.auto_speed)),
        )
        self.auto_acc = max(
            1,
            int(motor_config.get("auto_acc", self.auto_acc)),
        )

        self._apply_axis_config(self.motor3, motor_config.get("motor3", {}))
        self._apply_axis_config(self.motor4, motor_config.get("motor4", {}))

        # 방향/주기 변경 후에는 현재 실제각을 다시 기준 target으로 잡는다.
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
        return max(minimum, min(float(value), maximum))

    def _ensure_target(self, axis):
        """Closed loop 진입 시 현재 실제각을 target 시작점으로 1회 읽는다."""
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
            f"[MOTOR{axis.servo_id}] Closed Loop 시작각="
            f"{axis.target_angle_deg:+.2f}deg axis={axis.axis.upper()}"
        )
        return True

    def _fixed_servo_speed(self, axis):
        if axis.max_speed is None:
            return int(self.auto_speed)
        return max(1, min(int(self.auto_speed), int(axis.max_speed)))

    def move_to_neutral(self, motor_angles_deg):
        """Move both gimbal axes to the sensor-calibrated neutral angles."""
        if not self.ready:
            return {"accepted": False, "error": "Motor3/4가 준비되지 않았습니다."}
        values = dict(motor_angles_deg or {})
        targets = {}
        for axis in (self.motor3, self.motor4):
            value = values.get(axis.joint)
            if value is None:
                return {"accepted": False, "error": f"중립각이 없습니다: {axis.joint}"}
            target = float(value)
            if not float(axis.safe_min_deg) <= target <= float(axis.safe_max_deg):
                return {"accepted": False, "error": f"{axis.joint} 중립각이 안전범위 밖입니다."}
            targets[axis.joint] = target
        speed = min(self._fixed_servo_speed(self.motor3), self._fixed_servo_speed(self.motor4))
        success = self.motor.move_joints(
            targets, speed=speed, acc=int(self.auto_acc), wait=False
        )
        if success:
            for axis in (self.motor3, self.motor4):
                axis.target_angle_deg = targets[axis.joint]
                axis.last_command_angle_deg = targets[axis.joint]
            self.last_error = None
        else:
            self.last_error = self.motor.last_error or "Motor3/4 중립 이동 실패"
        return {"accepted": bool(success), "targets": targets, "error": self.last_error}

    def _control_axis(self, axis, pid_output_deg_s, now):
        if not axis.config_enabled:
            return True

        if not axis.ready:
            axis.last_success = False
            self.last_error = f"Motor{axis.servo_id}가 준비되지 않았습니다."
            return False

        if not self._ensure_target(axis):
            axis.last_success = False
            return False

        if axis.last_control_time is None:
            dt = self.command_interval
        else:
            dt = max(0.0, now - axis.last_control_time)

        # 긴 stall 뒤 target이 한 번에 크게 뛰지 않도록 최대 2주기까지만 인정.
        dt = min(dt, self.command_interval * 2.0)
        axis.last_control_time = now

        pid_output = float(pid_output_deg_s)
        target_velocity = pid_output * axis.direction_sign

        next_target = axis.target_angle_deg + target_velocity * dt
        next_target = self._clamp(
            next_target,
            axis.safe_min_deg,
            axis.safe_max_deg,
        )

        # V1.9 핵심: PID output으로 Servo Speed를 다시 줄이지 않는다.
        command_speed = self._fixed_servo_speed(axis)
        command_acc = int(self.auto_acc)

        success = self.motor.move_joint(
            axis.joint,
            angle=next_target,
            speed=command_speed,
            acc=command_acc,
            wait=False,
        )

        axis.last_success = bool(success)
        axis.last_command_speed = command_speed
        axis.last_command_acc = command_acc
        axis.last_pid_output_deg_s = pid_output

        if success:
            axis.target_angle_deg = float(next_target)
            axis.last_command_angle_deg = float(next_target)
            self.last_error = None
        else:
            self.last_error = (
                f"Servo {axis.servo_id} 이동 명령 실패 "
                f"target={next_target:.2f} speed={command_speed} acc={command_acc}"
            )

        if now - axis.debug_last_print_time >= 0.5:
            axis.debug_last_print_time = now
            print(
                f"[MOTOR{axis.servo_id}] DIRECT CMD "
                f"axis={axis.axis.upper()} "
                f"pid={pid_output:+.3f}deg/s "
                f"sign={axis.direction_sign:+.0f} "
                f"target={next_target:+.2f}deg "
                f"speed={command_speed} acc={command_acc} "
                f"success={bool(success)}"
            )

        return bool(success)

    def _control_motor3(self, imu_y_pid_output_deg_s, now):
        return self._control_axis(
            self.motor3,
            imu_y_pid_output_deg_s,
            now,
        )

    def _control_motor4(self, imu_x_pid_output_deg_s, now):
        return self._control_axis(
            self.motor4,
            imu_x_pid_output_deg_s,
            now,
        )

    def update(self, context):
        now = float(context.get("now", time.monotonic()))
        imu_state = context.get("imu", {})
        control_active = bool(context.get("motor34_control_active", False))

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
                    "[MOTOR34] DIRECT GIMBAL OFF "
                    f"available={self.available} enabled={self.enabled} "
                    f"requested={control_active} imu_ready={imu_ready} "
                    f"motor3_ready={self.motor3.ready} motor4_ready={self.motor4.ready}"
                )
            self._debug_last_control_active = False

            # 다음 Closed Loop ON 때 현재 실제각을 다시 시작 target으로 사용한다.
            if not control_active:
                self._reset_runtime_targets()

            self.latest_state = self._build_state(False)
            return self.latest_state

        if not self._debug_last_control_active:
            print(
                "[MOTOR34] DIRECT GIMBAL ON "
                f"M3<-Y error={float(imu_state.get('imu_y_error_g', 0.0)):+.5f}g/"
                f"{float(imu_state.get('motor3_correction_speed_deg_s', 0.0)):+.3f}deg/s "
                f"M4<-X error={float(imu_state.get('imu_x_error_g', 0.0)):+.5f}g/"
                f"{float(imu_state.get('motor4_correction_speed_deg_s', 0.0)):+.3f}deg/s"
            )
        self._debug_last_control_active = True

        # Hardware Process는 2ms 정도로 돌지만 실제 M3/M4 command는 100Hz gate.
        if now - self.last_command_time >= self.command_interval:
            self.last_command_time = now

            motor3_ok = self._control_motor3(
                imu_state.get("motor3_correction_speed_deg_s", 0.0),
                now,
            )
            motor4_ok = self._control_motor4(
                imu_state.get("motor4_correction_speed_deg_s", 0.0),
                now,
            )

            if not motor3_ok or not motor4_ok:
                self.last_error = self.last_error or "Motor3/4 command 실패"

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
            "pid_output_deg_s": axis.last_pid_output_deg_s,
            "command_speed": axis.last_command_speed,
            "command_acc": axis.last_command_acc,
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
            "command_hz": self.command_hz,
            "auto_speed": self.auto_speed,
            "auto_acc": self.auto_acc,
            "axis_mapping": {
                "motor3": "imu_y",
                "motor4": "imu_x",
            },
            "last_error": self.last_error,
            "motor3": self._axis_state(self.motor3),
            "motor4": self._axis_state(self.motor4),
            "timestamp": time.time(),
        }

import json
import time
from pathlib import Path


# pyQt/services/motor_service.py -> WorkSpace
WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_CALIBRATION_FILE = WORKSPACE_DIR / "servo_calibration_result.json"

MOTOR3_SERVO_ID = 3
MOTOR3_JOINT = "wrist_flex"

MOTOR4_SERVO_ID = 4
MOTOR4_JOINT = "wrist_roll"

# STServo Serial 명령은 IMU 50Hz보다 느리게 보낸다.
MOTOR_COMMAND_HZ = 10.0

# PID 출력이 이 값보다 작으면 새 모터 명령을 보내지 않는다.
# IMU 쪽 Dead Band와 별도로 Serial 명령 떨림을 줄이는 목적이다.
MOTOR_PID_SPEED_DEADBAND_DEG_S = 0.25

# IMU Pitch 보정 +방향과 wrist_flex 팀원용 +방향(위)의 관계.
# 실제 ADXL345 장착 방향 테스트에서 반대로 움직이면 -1.0으로 바꾼다.
MOTOR3_DIRECTION_SIGN = +1.0


class MonitorMotorService:
    """모니터 짐벌 Servo 3/4용 상위 제어 계층.

    현재 연결
      - Servo 3 / wrist_flex : IMU Pitch PID 출력으로 실제 제어
      - Servo 4 / wrist_roll : 인터페이스만 유지하고 pass

    중요한 설계
      - MediaPipe landmark는 이 클래스에서 사용하지 않는다.
      - IMU Service가 계산한 PID 보정속도(deg/s)를 받아 목표 관절각을 적분한다.
      - 실제 STServo speed 값은 calibration JSON의 max_speed를 기준으로
        PID 출력 비율에 맞춰 변환한다.
      - motor_control의 Safe Range / Calibration 검사를 그대로 사용한다.
    """

    def __init__(
        self,
        calibration_file=DEFAULT_CALIBRATION_FILE,
        command_hz=MOTOR_COMMAND_HZ,
        controller_factory=None,
    ):
        self.calibration_file = Path(calibration_file)
        self.command_hz = max(float(command_hz), 1.0)
        self.command_interval = 1.0 / self.command_hz
        self.controller_factory = controller_factory

        self.arm = None
        self.available = False
        self.enabled = True
        self.motor3_config_enabled = True
        self.motor4_config_enabled = False
        self.pid_speed_deadband_deg_s = MOTOR_PID_SPEED_DEADBAND_DEG_S
        self.motor3_direction_sign = MOTOR3_DIRECTION_SIGN
        self.last_error = None

        self.motor3_max_speed = None
        self.motor3_safe_min_deg = None
        self.motor3_safe_max_deg = None
        self.motor3_target_angle_deg = None
        self.motor3_last_command_angle_deg = None
        self.motor3_last_command_speed = 0
        self.motor3_last_success = None

        self.motor4_last_success = None

        self.last_control_time = None
        self.last_command_time = 0.0

        self.latest_state = self._empty_state()

    def _empty_state(self):
        return {
            "available": False,
            "enabled": bool(self.enabled),
            "motor3_config_enabled": bool(self.motor3_config_enabled),
            "control_active": False,
            "last_error": None,
            "motor3": {
                "servo_id": MOTOR3_SERVO_ID,
                "joint": MOTOR3_JOINT,
                "implemented": True,
                "ready": False,
                "config_enabled": bool(self.motor3_config_enabled),
                "direction_sign": self.motor3_direction_sign,
                "max_speed": None,
                "safe_min_deg": None,
                "safe_max_deg": None,
                "target_angle_deg": None,
                "command_speed": 0,
                "last_success": None,
            },
            "motor4": {
                "servo_id": MOTOR4_SERVO_ID,
                "joint": MOTOR4_JOINT,
                "implemented": False,
                "config_enabled": bool(self.motor4_config_enabled),
                "pass": True,
                "last_success": None,
            },
            "timestamp": time.time(),
        }

    def _load_motor3_calibration_metadata(self):
        if not self.calibration_file.exists():
            raise RuntimeError(
                f"Servo Calibration 파일이 없습니다: {self.calibration_file}"
            )

        with self.calibration_file.open("r", encoding="utf-8") as file:
            data = json.load(file)

        servo = data.get("servos", {}).get(str(MOTOR3_SERVO_ID))
        if not isinstance(servo, dict):
            raise RuntimeError("Servo 3(wrist_flex) Calibration 정보가 없습니다.")

        if servo.get("joint") != MOTOR3_JOINT:
            raise RuntimeError(
                f"Servo 3 Joint 불일치: expected={MOTOR3_JOINT}, "
                f"actual={servo.get('joint')}"
            )

        max_speed = servo.get("max_speed")
        if max_speed is None:
            raise RuntimeError(
                "Servo 3(wrist_flex) max_speed가 null입니다. "
                "motor_control 안전 규칙상 실제 이동 명령을 보낼 수 없습니다."
            )

        self.motor3_max_speed = int(max_speed)
        if self.motor3_max_speed <= 0:
            raise RuntimeError(
                f"Servo 3 max_speed 값이 올바르지 않습니다: {self.motor3_max_speed}"
            )

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

        motor3 = motor_config.get("motor3", {})
        self.motor3_config_enabled = bool(
            motor3.get("enabled", self.motor3_config_enabled)
        )
        sign = float(motor3.get("direction_sign", self.motor3_direction_sign))
        self.motor3_direction_sign = 1.0 if sign >= 0.0 else -1.0

        motor4 = motor_config.get("motor4", {})
        self.motor4_config_enabled = bool(
            motor4.get("enabled", self.motor4_config_enabled)
        )
        # Motor4는 설정값과 무관하게 현재 구현은 pass 상태를 유지한다.

        self._reset_runtime_target()
        return True

    def open(self):
        try:
            # README 규칙대로 max_speed가 정해지지 않았으면
            # 저수준 Driver를 우회하지 않고 제어 자체를 막는다.
            self._load_motor3_calibration_metadata()

            if self.controller_factory is None:
                from motor_control import MotorController
                factory = MotorController
            else:
                factory = self.controller_factory

            self.arm = factory(calibration_file=str(self.calibration_file))

            safe_min, safe_max = self.arm.calibration.get_safe_angle_range(MOTOR3_JOINT)
            self.motor3_safe_min_deg = float(safe_min)
            self.motor3_safe_max_deg = float(safe_max)

            self.available = True
            self.last_error = None
            self._reset_runtime_target()

            print(
                "[MOTOR] 준비 완료 "
                f"Servo3={MOTOR3_JOINT} "
                f"safe={self.motor3_safe_min_deg:+.2f}~{self.motor3_safe_max_deg:+.2f}deg "
                f"max_speed={self.motor3_max_speed} / "
                f"Servo4={MOTOR4_JOINT}=PASS"
            )
            return True

        except Exception as error:
            self.available = False
            self.last_error = str(error)
            self.close()
            print(f"[MOTOR] 초기화/안전검사 실패: {error}")
            return False

    def close(self):
        arm = self.arm
        self.arm = None
        self.available = False
        self._reset_runtime_target()

        if arm is not None:
            try:
                arm.close()
            except Exception:
                pass

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not self.enabled:
            self._reset_runtime_target()
        return self.enabled

    def _reset_runtime_target(self):
        self.motor3_target_angle_deg = None
        self.motor3_last_command_angle_deg = None
        self.motor3_last_command_speed = 0
        self.motor3_last_success = None
        self.last_control_time = None
        self.last_command_time = 0.0

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def _pid_deg_s_to_servo_speed(self, correction_speed_deg_s, pid_limit_deg_s):
        """PID deg/s를 STServo Speed 값으로 비율 변환한다.

        STServo의 speed 단위와 deg/s는 동일하지 않으므로 직접 동일시하지 않는다.
        대신 IMU PID 출력 한계 대비 비율을 calibration max_speed에 적용한다.
        """
        magnitude = abs(float(correction_speed_deg_s))
        limit = max(abs(float(pid_limit_deg_s)), 1e-6)

        if magnitude <= self.pid_speed_deadband_deg_s:
            return 0

        ratio = self._clamp(magnitude / limit, 0.0, 1.0)
        speed = int(round(ratio * self.motor3_max_speed))
        return max(1, min(speed, self.motor3_max_speed))

    def _ensure_motor3_target(self):
        if self.motor3_target_angle_deg is not None:
            return True

        current_angle = self.arm.get_joint_angle(MOTOR3_JOINT)
        if current_angle is None:
            self.last_error = "Servo 3 현재 각도를 읽지 못했습니다."
            return False

        self.motor3_target_angle_deg = float(current_angle)
        self.motor3_last_command_angle_deg = float(current_angle)
        # 첫 실제 보정 명령은 command_interval만큼 적분되도록 None을 유지한다.
        self.last_control_time = None

        print(f"[MOTOR3] 시작 기준각={self.motor3_target_angle_deg:+.2f}deg")
        return True

    def _control_motor3_wrist_flex(
        self,
        correction_pitch_speed_deg_s,
        pid_limit_deg_s,
        now,
    ):
        """Servo 3(wrist_flex) 실제 제어.

        PID 보정속도를 dt만큼 적분하여 새로운 절대 목표각을 만든다.
        이렇게 해야 IMU 오차가 0이 된 뒤에도 보정된 모터 위치가 유지된다.
        """
        correction_speed = (
            float(correction_pitch_speed_deg_s)
            * self.motor3_direction_sign
        )
        command_speed = self._pid_deg_s_to_servo_speed(
            correction_speed,
            pid_limit_deg_s,
        )

        # 각도/PID 보정값이 Dead Band 안이면 Servo 버스에 아무 패킷도 보내지 않는다.
        # 현재각 조회(_ensure_motor3_target)조차 하지 않고 즉시 반환한다.
        if command_speed <= 0:
            self.motor3_last_command_speed = 0
            self.motor3_last_success = True
            return True

        if not self._ensure_motor3_target():
            self.motor3_last_success = False
            return False

        if self.last_control_time is None:
            dt = self.command_interval
        else:
            dt = max(0.0, now - self.last_control_time)

        # 프로세스 스케줄 지연 때문에 한 번에 큰 각도가 적분되는 것을 방지한다.
        dt = min(dt, self.command_interval * 2.0)
        self.last_control_time = now

        next_target = self.motor3_target_angle_deg + correction_speed * dt
        next_target = self._clamp(
            next_target,
            self.motor3_safe_min_deg,
            self.motor3_safe_max_deg,
        )

        success = self.arm.move_joint(
            MOTOR3_JOINT,
            angle=next_target,
            speed=command_speed,
            wait=False,
        )

        self.motor3_last_success = bool(success)
        self.motor3_last_command_speed = int(command_speed)

        if success:
            self.motor3_target_angle_deg = float(next_target)
            self.motor3_last_command_angle_deg = float(next_target)
            self.last_error = None
        else:
            self.last_error = (
                f"Servo 3 이동 명령 실패 target={next_target:.2f} "
                f"speed={command_speed}"
            )

        return bool(success)

    def _control_motor4_wrist_roll(
        self,
        correction_roll_speed_deg_s,
        pid_limit_deg_s,
        now,
    ):
        """Servo 4(wrist_roll) 예약 자리.

        IMU Roll -> wrist_roll 연결 예정.
        사용자 요청에 따라 현재는 아무 명령도 보내지 않는다.
        """
        pass

    def update(
        self,
        imu_state,
        control_active,
        pid_limit_deg_s,
    ):
        now = time.monotonic()

        imu_state = imu_state if isinstance(imu_state, dict) else {}
        imu_ready = bool(
            imu_state.get("available", False)
            and imu_state.get("calibrated", False)
            and not imu_state.get("calibrating", False)
        )

        can_control = bool(
            self.available
            and self.enabled
            and self.motor3_config_enabled
            and control_active
            and imu_ready
            and self.arm is not None
        )

        if not can_control:
            # 측정이 꺼졌다가 다시 켜질 때 현재 실제 Servo 위치를 새 기준으로 잡는다.
            if not control_active:
                self._reset_runtime_target()

            self.latest_state = self._build_state(control_active=False)
            return self.latest_state

        if now - self.last_command_time >= self.command_interval:
            self.last_command_time = now

            self._control_motor3_wrist_flex(
                correction_pitch_speed_deg_s=imu_state.get(
                    "correction_pitch_speed_deg_s", 0.0
                ),
                pid_limit_deg_s=pid_limit_deg_s,
                now=now,
            )

            # Servo 4는 호출 경로만 마련하고 실제 동작은 pass.
            self._control_motor4_wrist_roll(
                correction_roll_speed_deg_s=imu_state.get(
                    "correction_roll_speed_deg_s", 0.0
                ),
                pid_limit_deg_s=pid_limit_deg_s,
                now=now,
            )

        self.latest_state = self._build_state(control_active=True)
        return self.latest_state

    def _build_state(self, control_active):
        return {
            "available": bool(self.available),
            "enabled": bool(self.enabled),
            "motor3_config_enabled": bool(self.motor3_config_enabled),
            "control_active": bool(control_active and self.available and self.enabled),
            "last_error": self.last_error,
            "motor3": {
                "servo_id": MOTOR3_SERVO_ID,
                "joint": MOTOR3_JOINT,
                "implemented": True,
                "ready": bool(self.available and self.motor3_max_speed is not None),
                "config_enabled": bool(self.motor3_config_enabled),
                "direction_sign": self.motor3_direction_sign,
                "max_speed": self.motor3_max_speed,
                "safe_min_deg": self.motor3_safe_min_deg,
                "safe_max_deg": self.motor3_safe_max_deg,
                "target_angle_deg": self.motor3_target_angle_deg,
                "command_speed": self.motor3_last_command_speed,
                "last_success": self.motor3_last_success,
            },
            "motor4": {
                "servo_id": MOTOR4_SERVO_ID,
                "joint": MOTOR4_JOINT,
                "implemented": False,
                "config_enabled": bool(self.motor4_config_enabled),
                "pass": True,
                "last_success": self.motor4_last_success,
            },
            "timestamp": time.time(),
        }

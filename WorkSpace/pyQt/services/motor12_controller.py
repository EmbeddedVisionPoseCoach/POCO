import time
from dataclasses import dataclass

from services.monitor_arm_planner import MonitorArmPlanner

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
    실제 ToF 입력 / 속도 선택 / SyncWrite 제어는 이후 단계에서 연결한다.
    """

    def __init__(self, motor_service, settings=None, update_hz=MOTOR12_UPDATE_HZ):
        self.motor = motor_service

        # settings=None은 기존 self-test 호환을 위해 임시로 허용한다.
        # 실제 HardwareProcess에서는 WorkSpace/config의 고정 JSON을 항상 전달한다.
        if settings is not None and not isinstance(settings, dict):
            raise TypeError("Motor12 settings는 dict 또는 None이어야 합니다.")

        self.settings = settings
        self.planner = MonitorArmPlanner(settings) if settings is not None else None
        
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

            print(
                f"[MOTOR12] 초기화/안전검사 실패: {error}"
            )

            return False

    def _initialize_axis(self, axis):
        servo = self.motor.get_joint_metadata(axis.joint)

        if not servo:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                "Calibration 정보가 없습니다."
            )

        actual_servo_id = int(
            servo.get("servo_id", -1)
        )

        if actual_servo_id != axis.servo_id:
            raise RuntimeError(
                f"{axis.joint} Servo ID 불일치: "
                f"expected={axis.servo_id}, "
                f"actual={actual_servo_id}"
            )

        max_speed = self.motor.get_max_speed(
            axis.joint
        )

        if max_speed is None or max_speed <= 0:
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                "max_speed가 설정되지 않았습니다."
            )

        safe_range = self.motor.get_safe_angle_range(
            axis.joint
        )

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

        ping_result = self.motor.ping_joint(
            axis.joint
        )

        if not ping_result.get("success", False):
            raise RuntimeError(
                f"Servo {axis.servo_id}({axis.joint}) "
                f"Ping 실패: {ping_result}"
            )

        axis.max_speed = int(max_speed)
        axis.safe_min_deg = safe_min_deg
        axis.safe_max_deg = safe_max_deg
        axis.ready = True

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        self.latest_state = self._build_state(False)
        return self.enabled

    def close(self):
        self.available = False
        self._reset_ready_state()
        self.latest_state = self._build_state(False)

    def _reset_ready_state(self):
        self.motor1.reset_ready()
        self.motor2.reset_ready()

    def update(self, context):
        """현재 단계에서는 상태 갱신만 수행한다.

        실제 Motor1/2 제어는 Planner / ToF /
        다축 SyncWrite 연결 단계에서 추가한다.
        """
        now = float(
            context.get("now", time.monotonic())
        )

        if (
            now - self.last_update_time
            < self.update_interval
        ):
            return self.latest_state

        self.last_update_time = now

        self.latest_state = self._build_state(False)

        return self.latest_state

    @staticmethod
    def _axis_state(axis):
        return {
            "servo_id": axis.servo_id,
            "joint": axis.joint,
            "implemented": False,
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
            "last_error": self.last_error,
            "motor1": self._axis_state(
                self.motor1
            ),
            "motor2": self._axis_state(
                self.motor2
            ),
            "timestamp": time.time(),
        }
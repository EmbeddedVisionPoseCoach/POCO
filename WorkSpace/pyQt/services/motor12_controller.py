import time


MOTOR1_SERVO_ID = 1
MOTOR1_JOINT = "shoulder_lift"
MOTOR2_SERVO_ID = 2
MOTOR2_JOINT = "elbow_flex"

MOTOR12_UPDATE_HZ = 20.0


class Motor12Controller:
    """Motor 1 / 2 전용 로직 자리.

    다른 팀원은 원칙적으로 이 파일만 수정하면 된다.
    Hardware Process / Queue / STServo SDK / Motor3 PID는 건드리지 않는다.

    실제 모터 명령 예시:
        self.motor.move_joint(MOTOR1_JOINT, angle=10.0, speed=30, wait=False)
        self.motor.move_joint(MOTOR2_JOINT, angle=-5.0, speed=30, wait=False)

    현재는 1/2번 로직이 아직 없으므로 두 함수 모두 pass다.
    """

    def __init__(self, motor_service, update_hz=MOTOR12_UPDATE_HZ):
        self.motor = motor_service
        self.update_hz = max(1.0, float(update_hz))
        self.update_interval = 1.0 / self.update_hz
        self.last_update_time = 0.0
        self.enabled = True
        self.latest_state = self._build_state()

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)

    def update(self, context):
        now = float(context.get("now", time.monotonic()))

        if not self.enabled or now - self.last_update_time < self.update_interval:
            return self.latest_state

        self.last_update_time = now

        # =====================================================
        # 실행 순서: Motor 1 -> Motor 2
        # 다른 팀원은 아래 두 함수 내부만 구현하면 된다.
        # =====================================================
        self._control_motor1(context)
        self._control_motor2(context)

        self.latest_state = self._build_state()
        return self.latest_state

    def _control_motor1(self, context):
        """Servo 1 / shoulder_lift 로직 구현 위치."""
        pass

    def _control_motor2(self, context):
        """Servo 2 / elbow_flex 로직 구현 위치."""
        pass

    def _build_state(self):
        return {
            "enabled": bool(self.enabled),
            "implemented": False,
            "update_hz": self.update_hz,
            "motor1": {
                "servo_id": MOTOR1_SERVO_ID,
                "joint": MOTOR1_JOINT,
                "implemented": False,
                "pass": True,
            },
            "motor2": {
                "servo_id": MOTOR2_SERVO_ID,
                "joint": MOTOR2_JOINT,
                "implemented": False,
                "pass": True,
            },
            "timestamp": time.time(),
        }

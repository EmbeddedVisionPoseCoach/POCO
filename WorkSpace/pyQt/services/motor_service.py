import time
from pathlib import Path


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
HARDWARE_DIR = WORKSPACE_DIR / "hardware"
DEFAULT_CALIBRATION_FILE = HARDWARE_DIR / "servo_calibration_result.json"


class MotorService:
    """1~4번 Servo가 공통으로 사용하는 실제 모터 통신 계층.

    이 클래스는 '어떻게 움직일지' 판단하지 않는다.
    역할은 아래 5개뿐이다.

    - MotorController / Serial 포트 1회 생성
    - Joint ping
    - 현재 각도 읽기
    - 절대/상대 각도 이동
    - Calibration 정보 조회

    Motor1/2 로직과 Motor3/4 로직은 각각 Controller 클래스에서 담당한다.
    모든 Controller가 같은 MotorService 인스턴스를 공유하므로 /dev/ttyACM0도 한 곳에서만 소유한다.
    """

    def __init__(self, calibration_file=DEFAULT_CALIBRATION_FILE, controller_factory=None):
        self.calibration_file = Path(calibration_file)
        self.controller_factory = controller_factory
        self.arm = None
        self.available = False
        self.last_error = None
        self.latest_state = self._build_state()

    def _build_state(self):
        device = None
        baudrate = None
        if self.arm is not None:
            calibration = getattr(self.arm, "calibration", None)
            device = getattr(calibration, "device", None)
            baudrate = getattr(calibration, "baudrate", None)

        return {
            "available": bool(self.available),
            "device": device,
            "baudrate": baudrate,
            "calibration_file": str(self.calibration_file),
            "last_error": self.last_error,
            "timestamp": time.time(),
        }

    def open(self):
        try:
            if not self.calibration_file.exists():
                raise RuntimeError(
                    f"Servo Calibration 파일이 없습니다: {self.calibration_file}"
                )

            if self.controller_factory is None:
                from hardware.motor_control import MotorController
                factory = MotorController
            else:
                factory = self.controller_factory

            self.arm = factory(calibration_file=str(self.calibration_file))
            self.available = True
            self.last_error = None
            self.latest_state = self._build_state()
            print(
                "[MOTOR BUS] 준비 완료 "
                f"device={self.latest_state.get('device')} "
                f"baudrate={self.latest_state.get('baudrate')}"
            )
            return True

        except Exception as error:
            self.last_error = str(error)
            self.available = False
            self.close()
            self.latest_state = self._build_state()
            print(f"[MOTOR BUS] 초기화 실패: {error}")
            return False

    def close(self):
        arm = self.arm
        self.arm = None
        self.available = False
        if arm is not None:
            try:
                arm.close()
            except Exception:
                pass
        self.latest_state = self._build_state()

    def get_joint_metadata(self, joint_name):
        if not self.available or self.arm is None:
            return None
        try:
            servo = self.arm.calibration.get_joint(joint_name)
            return dict(servo) if isinstance(servo, dict) else None
        except Exception as error:
            self.last_error = str(error)
            return None

    def get_safe_angle_range(self, joint_name):
        if not self.available or self.arm is None:
            return None
        try:
            safe_min, safe_max = self.arm.calibration.get_safe_angle_range(joint_name)
            return float(safe_min), float(safe_max)
        except Exception as error:
            self.last_error = str(error)
            return None

    def get_max_speed(self, joint_name):
        servo = self.get_joint_metadata(joint_name)
        if not servo:
            return None
        value = servo.get("max_speed")
        return None if value is None else int(value)

    def ping_joint(self, joint_name):
        if not self.available or self.arm is None:
            return {"success": False, "error": "motor bus unavailable"}

        servo = self.get_joint_metadata(joint_name)
        if not servo:
            return {"success": False, "error": f"joint not found: {joint_name}"}

        driver = getattr(self.arm, "driver", None)
        if driver is None or not hasattr(driver, "ping"):
            # FakeController/self-test 호환. 실기 MotorController에는 ping이 있어야 한다.
            if self.controller_factory is not None:
                return {"success": True, "mock": True}
            return {"success": False, "error": "Servo ping API not found"}

        try:
            result = driver.ping(int(servo["servo_id"]))
            if not isinstance(result, dict):
                return {"success": False, "error": f"invalid ping result: {result}"}
            return result
        except Exception as error:
            self.last_error = str(error)
            return {"success": False, "error": str(error)}

    def get_joint_angle(self, joint_name):
        if not self.available or self.arm is None:
            return None
        try:
            return self.arm.get_joint_angle(joint_name)
        except Exception as error:
            self.last_error = str(error)
            return None

    def move_joint(self, joint_name, angle, speed, acc=None, wait=False):
        """절대각도 이동.

        acc=None이면 기존 MotorController 기본 Acc를 그대로 사용한다.
        Motor3/4 Direct IMU 제어는 검증값 Acc=12를 명시해서 호출한다.
        기존 Motor1/2 호출은 acc를 넘기지 않아도 그대로 동작한다.
        """
        if not self.available or self.arm is None:
            self.last_error = "Motor bus unavailable"
            return False
        try:
            kwargs = {
                "angle": float(angle),
                "speed": int(speed),
                "wait": bool(wait),
            }
            if acc is not None:
                kwargs["acc"] = int(acc)

            return bool(
                self.arm.move_joint(
                    joint_name,
                    **kwargs,
                )
            )
        except Exception as error:
            self.last_error = str(error)
            return False

    def move_joint_relative(self, joint_name, delta_angle, speed, wait=False):
        if not self.available or self.arm is None:
            self.last_error = "Motor bus unavailable"
            return False
        try:
            return bool(
                self.arm.move_joint_relative(
                    joint_name,
                    delta_angle=float(delta_angle),
                    speed=int(speed),
                    wait=bool(wait),
                )
            )
        except Exception as error:
            self.last_error = str(error)
            return False

    def get_state(self):
        self.latest_state = self._build_state()
        return dict(self.latest_state)


# 이전 이름을 import하던 코드가 있어도 깨지지 않도록 한동안 alias를 유지한다.
MonitorMotorService = MotorService

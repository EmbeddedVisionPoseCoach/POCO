import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(ROOT_DIR))


class HardwareController:
    """
    PyQt와 하드웨어 코드를 연결하는 얇은 중간 계층.

    목적:
    - CameraWorker가 serial, Arduino 명령, 하드웨어 내부 함수를 직접 알지 않게 한다.
    - 하드웨어 팀은 arduino_control.py에 함수만 제공하면 된다.
    - 하드웨어가 연결되지 않아도 PyQt 측정 기능은 죽지 않게 한다.
    """

    def __init__(
        self,
        enabled=True,
        serial_port="/dev/ttyACM0",
        baud_rate=115200,
        timeout=1,
        connect_delay=2.0,
    ):
        self.enabled = enabled
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.connect_delay = connect_delay

        self.arduino = None
        self.is_connected = False

        # 하드웨어 팀 모듈 함수들
        self.serial_module = None

    def connect(self):
        """
        아두이노와 시리얼 연결을 연다.

        연결 실패해도 예외를 밖으로 던지지 않고 False를 반환한다.
        그래야 하드웨어 없이도 PyQt 테스트가 가능하다.
        """

        if not self.enabled:
            print("[Hardware] 비활성화 상태입니다.")
            return False

        try:
            import serial

            # 하드웨어 팀 코드 모듈.
            # 파일 위치 예시:
            # WorkSpace/hardware/arduino_control.py
            from hardware import arduino_control

            self.serial_module = arduino_control

            self.arduino = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.timeout
            )

            # 아두이노 리셋 안정화 대기
            time.sleep(self.connect_delay)

            self.is_connected = True
            print("[Hardware] 아두이노 연결 완료")
            return True

        except Exception as e:
            self.is_connected = False
            self.arduino = None
            print(f"[Hardware] 연결 실패: {e}")
            return False

    def start_HardwareSet(self):
        """
        캘리브레이션 전에 1회 실행할 카메라 수평 보정.

        내부적으로 하드웨어 팀의 start_leveling(arduino)을 호출한다.
        """

        if not self.enabled:
            print("[Hardware] 비활성화 상태라 leveling을 건너뜁니다.")
            return True

        if not self.ensure_connected():
            print("[Hardware] 연결되지 않아 leveling을 건너뜁니다.")
            return False

        try:
            print("[Hardware] 카메라 수평 보정 시작")
            self.serial_module.start_leveling(self.arduino)
            print("[Hardware] 카메라 수평 보정 완료")
            return True

        except Exception as e:
            print(f"[Hardware] 카메라 수평 보정 실패: {e}")
            return False

    def update_hardware(self, result):
        """
        추론 결과를 받아 하드웨어 제어 함수로 넘긴다.

        Parameters
        ----------
        result:
            mlp_inference_service.InferenceResult 또는
            gru_inference_service.InferenceResult

        요구 필드:
            result.pose_index
            result.fatigue_index
        """

        if not self.enabled:
            return

        if result is None:
            return

        if not getattr(result, "success", False):
            return

        if not self.ensure_connected():
            return

        pose_index = getattr(result, "pose_index", None)
        fatigue_index = getattr(result, "fatigue_index", None)

        if pose_index is None:
            return

        try:
            # 현재 하드웨어 팀 함수는 pose class index 기반으로 command를 생성한다.
            command = self.serial_module.get_posture_result_from_ai(pose_index)

            if command is not None:
                self.serial_module.send_command(self.arduino, command)

            # 아두이노 응답이 있으면 읽어둔다.
            self.serial_module.read_response(self.arduino)

            # fatigue_index는 현재 하드웨어 팀 함수에 아직 반영되어 있지 않다.
            # 나중에 졸림 상태에 따라 별도 명령이 필요하면 여기서 추가하면 된다.
            # 예:
            # if fatigue_index == 1:
            #     self.serial_module.send_command(self.arduino, "Drowsy")

        except Exception as e:
            print(f"[Hardware] result 처리 실패: {e}")

    def ensure_connected(self):
        """
        연결되어 있으면 True.
        연결 안 되어 있으면 한 번 연결 시도.
        """

        if self.is_connected and self.arduino is not None:
            return True

        return self.connect()

    def close(self):
        """
        시리얼 연결 종료.
        """

        try:
            if self.arduino is not None:
                self.arduino.close()
                print("[Hardware] 시리얼 연결 종료")
        except Exception:
            pass

        self.arduino = None
        self.is_connected = False
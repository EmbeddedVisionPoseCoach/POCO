import sys
import time
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(ROOT_DIR))

from modules.app_settings import SettingsManager, AlarmSettings

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

        # 알림 기능 사용 여부
        # True  : 자세/졸림 알림을 아두이노로 전송
        # False : 추론은 계속 하지만 아두이노 알림은 보내지 않음
        self.alert_enabled = True

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

    def set_hardware_Values(self, settings: AlarmSettings):
        print(f"[Hardware] 새로운 설정값 적용: {settings.alarm_enabled}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.posture_Hardware_count}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.fatigue_Hardware_count}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.bad_posture_duration_sec}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.fatigue_duration_sec}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.posture_Strong_limit}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.fatigue_Strong_limit}")
        print(f"[Hardware] 새로운 설정값 적용: {settings.strong_alert_cooldown_min}")

        self.set_alert_enabled(settings.alarm_enabled)

        self.set_alert_counts(
            settings.posture_Hardware_count,
            settings.fatigue_Hardware_count
        )

        self.set_hold_seconds(
            settings.bad_posture_duration_sec,
            settings.fatigue_duration_sec
        )

        self.set_strong_alert_settings(
            settings.posture_Strong_limit,
            settings.fatigue_Strong_limit,
            settings.strong_alert_cooldown_min
        )



    def set_alert_enabled(self, enabled):
        """
        PyQt UI에서 알림 ON/OFF 값이 바뀌었을 때 호출된다.

        enabled:
            True  -> 알림 켜기
            False -> 알림 끄기
        """

        self.alert_enabled = enabled

        if enabled:
            print("[Hardware] 알림 기능 ON")
        else:
            print("[Hardware] 알림 기능 OFF")

    def set_alert_counts(self, posture_count, drowsy_count):
        """
        PyQt에서 설정한 자세/졸음 알림 횟수를 아두이노로 전달한다.

        Parameters
        ----------
        posture_count:
            자세 불량 알림 횟수
            Asymmetric, ForwardHead, ChinPropping에 적용된다.

        drowsy_count:
            졸음 알림 횟수
            Drowsy에 적용된다.
        """

        if not self.enabled:
            print("[Hardware] 비활성화 상태라 알림 횟수 설정을 건너뜁니다.")
            return False

        if not self.ensure_connected():
            print("[Hardware] 연결되지 않아 알림 횟수 설정을 건너뜁니다.")
            return False

        try:
            # 자세 알림 횟수 설정 명령 전송
            self.serial_module.set_posture_alert_count(
                self.arduino,
                posture_count
            )

            # 졸음 알림 횟수 설정 명령 전송
            self.serial_module.set_drowsy_alert_count(
                self.arduino,
                drowsy_count
            )

            # 아두이노가 설정 완료 메시지를 보내면 읽어둔다.
            self.serial_module.read_response(self.arduino)
            self.serial_module.read_response(self.arduino)

            print(
                f"[Hardware] 알림 횟수 설정 완료 "
                f"(posture={posture_count}, drowsy={drowsy_count})"
            )

            return True

        except Exception as e:
            print(f"[Hardware] 알림 횟수 설정 실패: {e}")
            return False


    def set_hold_seconds(
        self,
        posture_seconds,
        drowsy_seconds
    ):
        """
        PyQt에서 설정한 유지시간을 저장한다.

        Parameters
        ----------
        posture_seconds:
            자세 유지시간

        drowsy_seconds:
            졸음 유지시간
        """

        # 설정 적용 전 연결 확인
        if not self.ensure_connected():
            print(
                "[Hardware] 연결되지 않아 유지시간 설정을 적용하지 못했습니다."
            )
            return False

        try:

            # 자세 유지시간 설정
            self.serial_module.set_posture_hold_seconds(
                posture_seconds
            )


            # 졸음 유지시간 설정
            self.serial_module.set_drowsy_hold_seconds(
                drowsy_seconds
            )


            print(

                f"[Hardware] 유지시간 설정 완료 "

                f"(posture={posture_seconds}s, "

                f"drowsy={drowsy_seconds}s)"

            )

            return True


        except Exception as e:

            print(

                f"[Hardware] 유지시간 설정 실패: {e}"

            )

            return False
        

    def set_strong_alert_settings(
        self,
        posture_limit,
        drowsy_limit,
        cooldown_minutes
    ):
        """
        PyQt에서 설정한 StrongAlert 기준값을 arduino_control.py에 전달한다.

        posture_limit:
            자세 일반 알람이 몇 회 연속 반복되면 StrongAlert로 바꿀지

        drowsy_limit:
            졸음 일반 알람이 몇 회 연속 반복되면 StrongAlert로 바꿀지

        cooldown_minutes:
            StrongAlert 이후 같은 알람을 몇 분 동안 중단할지
        """

        # 설정 적용 전 연결 확인
        if not self.ensure_connected():
            print(
                "[Hardware] 연결되지 않아 StrongAlert 설정을 적용하지 못했습니다."
            )
            return False

        try:
            self.serial_module.set_strong_alert_settings(
                posture_limit,
                drowsy_limit,
                cooldown_minutes
            )

            print(
                f"[Hardware] StrongAlert 설정 완료 "
                f"(posture_limit={posture_limit}, "
                f"drowsy_limit={drowsy_limit}, "
                f"cooldown={cooldown_minutes}분)"
            )

            return True

        except Exception as e:
            print(f"[Hardware] StrongAlert 설정 실패: {e}")
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
        
        # 알림 OFF 상태이면 아두이노로 자세/졸림 알림을 보내지 않는다.
        # 단, 추론 자체는 계속 진행되고 UI 표시도 정상 동작한다.
        if not self.alert_enabled:
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
            # 현재 AI 추론 결과(pose + fatigue)를 기반으로
            # 아두이노에 보낼 최종 command를 생성한다.
            #
            # 우선순위:
            # Drowsy > ChinPropping > ForwardHead > Asymmetric > Optimal
            #
            # 예:
            # fatigue_index = 1 (Drowsy)
            # pose_index = 2 (ForwardHead)
            #
            # → 졸림 상태가 더 중요하므로 "Drowsy" 전송
            #
            # 예:
            # fatigue_index = 0 (Normal)
            # pose_index = 2 (ForwardHead)
            #
            # → "ForwardHead" 전송

            command = self.serial_module.get_alert_result_from_ai(
                pose_index,
                fatigue_index
            )

            # 생성된 command가 있으면 아두이노로 전송
            # None이면 현재 전송할 상태가 없다는 의미
            if command is not None:
                self.serial_module.send_command(
                    self.arduino,
                    command
                )

            # 아두이노가 보낸 응답 메시지가 있으면 읽어둔다.
            # 예:
            # "LEVELING_DONE"
            # "ALERT_DONE"
            self.serial_module.read_response(
                self.arduino
            )

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
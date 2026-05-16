# 쉘에 설치
# python3 -m pip show picamera2
# python3 -m pip install pyserial

import serial  # 라즈베리파이와 아두이노 간 시리얼 통신을 하기 위한 라이브러리
import time    # 대기 시간 sleep()을 사용하기 위한 라이브러리


# =========================
# 시리얼 설정
# =========================

# 아두이노가 연결된 포트 이름
# 라즈베리파이에서는 보통 Arduino Uno가 /dev/ttyACM0 또는 /dev/ttyUSB0로 잡힘
SERIAL_PORT = "/dev/ttyACM0"

# 만약 위 포트로 연결이 안 되면 아래처럼 바꿔서 테스트할 수 있음
# SERIAL_PORT = "/dev/ttyUSB0"

# 통신 속도
# 아두이노 코드의 Serial.begin(115200); 과 반드시 같아야 함
BAUD_RATE = 115200

# 시리얼 데이터를 읽을 때 최대 몇 초까지 기다릴지 설정
# 1초 동안 데이터가 없으면 읽기를 포기하고 다음 코드로 넘어감
TIMEOUT = 1

# 이전에 보낸 자세 상태
last_sent_idx = None

# 나쁜 자세가 시작된 시간
bad_start_time = None

# 현재 감지 중인 나쁜 자세 번호
current_bad_idx = None


# =========================
# 아두이노로 명령 보내는 함수
# =========================
def send_command(arduino, command):
    # command 문자열 뒤에 "\n"을 붙여서 아두이노로 전송
    # 아두이노에서는 보통 readStringUntil('\n') 같은 방식으로 한 줄씩 읽기 때문에
    # 줄바꿈 문자를 붙여주는 것이 중요함
    arduino.write((command + "\n").encode("utf-8"))

    # 라즈베리파이에서 어떤 명령을 보냈는지 터미널에 출력
    print(f"[RasPi → Arduino] {command}")


# =========================
# 아두이노 응답 읽는 함수
# =========================
def read_response(arduino):
    # arduino.in_waiting은 현재 시리얼 버퍼에 도착해 있는 데이터의 바이트 수
    # 즉, 아두이노가 보낸 데이터가 있는지 확인하는 조건
    if arduino.in_waiting > 0:

        # readline()으로 한 줄 읽기
        # decode("utf-8")로 바이트 데이터를 문자열로 변환
        # errors="ignore"는 깨진 문자가 있어도 오류를 내지 않고 무시하게 함
        # strip()은 앞뒤 공백과 줄바꿈 문자를 제거
        line = arduino.readline().decode("utf-8", errors="ignore").strip()

        # 읽은 내용이 빈 문자열이 아니면 출력하고 반환
        if line:
            print(f"[Arduino → RasPi] {line}")
            return line

    # 읽을 데이터가 없거나 빈 줄이면 None 반환
    return None


# =========================
# 처음 1회 카메라 수평 보정
# =========================
def start_leveling(arduino):
    # 수평 보정을 시작한다는 안내 메시지 출력
    print("카메라 수평 보정 시작 요청...")

    # 아두이노에게 START_LEVELING 명령 전송
    # 아두이노는 이 명령을 받으면 MPU6050 값으로 기울기를 측정하고
    # 서보모터를 움직여 카메라 수평을 맞추는 함수 실행
    send_command(arduino, "START_LEVELING")

    # 수평 보정이 끝날 때까지 계속 아두이노 응답 확인
    while True:
        # 아두이노가 보낸 문자열 읽기
        response = read_response(arduino)

        # 아두이노가 LEVELING_DONE을 보내면 수평 보정 완료로 판단
        if response == "LEVELING_DONE":
            print("카메라 수평 보정 완료!")
            break

        # 너무 빠르게 반복하면 CPU를 불필요하게 많이 쓰므로 0.1초 대기
        time.sleep(0.1)


# =========================
# 임시 자세 판단 함수
# 나중에 실제 자세 판단 결과와 연결할 부분
# =========================

def convert_class_idx_to_command(class_idx):
    """
    AI class_idx를 아두이노로 보낼 문자열로 변환
    AI class_idx:
        0: Optimal
        1: ForwardHead
        2: ChinPropping
        3: Asymmetric

        Face는 문자열로 넘어옴
        Normal
        Drowsy
        

    기존 임시 입력:
        1: Good
        2: Asymmetry
        3: ForwardHead
        4: ChinRest
    """

    commands = {
        0: "Good",
        1: "Asymmetry",
        2: "ForwardHead",
        3: "ChinRest"
    }

    return commands.get(class_idx)


def get_posture_result_from_ai(class_idx):
    global last_sent_idx
    global bad_start_time
    global current_bad_idx

    now = time.time()

    # =========================
    # 0번: Good 상태
    # 정상 자세는 바로 전송
    # =========================
    if class_idx == 0:
        bad_start_time = None
        current_bad_idx = None

        # 이전에 보낸 값과 다를 때만 전송
        if last_sent_idx != class_idx:
            last_sent_idx = class_idx
            return "Good"

        return None

    # =========================
    # 1~3번: 나쁜 자세 상태
    # 3초 이상 지속될 때만 전송
    # =========================
    else:
        # 새 나쁜 자세가 감지되면 시간 측정 시작
        if current_bad_idx != class_idx:
            current_bad_idx = class_idx
            bad_start_time = now
            return None

        # 같은 나쁜 자세가 계속 유지되는 중
        elapsed_time = now - bad_start_time

        # 3초 이상 지속되고, 아직 같은 값을 보내지 않았다면 전송
        if elapsed_time >= 3 and last_sent_idx != class_idx:
            last_sent_idx = class_idx
            return convert_class_idx_to_command(class_idx)

        return None

# =========================
# 메인 실행부
# =========================
def main():
    try:
        # 라즈베리파이에서 아두이노와 시리얼 연결 시작
        # SERIAL_PORT: 아두이노 포트
        # BAUD_RATE: 통신 속도
        # timeout: 읽기 대기 시간
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

        # 아두이노는 시리얼 연결이 열리면 자동으로 리셋되는 경우가 많음
        # 그래서 바로 명령을 보내면 아두이노가 준비되기 전에 데이터가 날아갈 수 있음
        # 안정화를 위해 2초 정도 대기
        time.sleep(2)

        print("아두이노 연결 완료!")

        # 1. 프로그램 시작 시 카메라 수평 보정 1회 실행
        # 라즈베리파이가 START_LEVELING 명령을 보내고
        # 아두이노가 LEVELING_DONE을 보낼 때까지 기다림
        start_leveling(arduino)

        # 2. 수평 보정이 끝난 뒤에는 자세 알림 전송 모드로 진입
        print("\n자세 알림 전송 모드 시작!")

        while True:
            # 현재는 사용자가 직접 입력한 자세 상태를 가져옴
            command = get_posture_result_from_ai(class_idx)

            # # 사용자가 q를 입력하면 반복문 종료
            # if posture == "QUIT":
            #     print("프로그램 종료")
            #     break

            # # 올바른 자세 상태가 입력된 경우에만 아두이노로 전송
            # if posture is not None:
            #     send_command(arduino, posture)

            if command is not None:
                arduino.write((command + "\n").encode())
                print("아두이노로 전송:", command)

            # 아두이노가 명령 처리 후 응답할 시간을 조금 줌
            time.sleep(0.2)

            # 아두이노가 출력한 확인 메시지 읽기
            read_response(arduino)

            
    # 시리얼 연결 자체가 실패했을 때 실행됨
    # 예: 포트 이름이 틀렸거나, 아두이노가 연결되어 있지 않거나,
    # 권한 문제가 있을 때 발생 가능
    except serial.SerialException:
        print("아두이노 시리얼 연결 실패!")
        print("포트가 /dev/ttyACM0인지 /dev/ttyUSB0인지 확인해줘.")

    # 사용자가 Ctrl + C로 강제 종료했을 때 실행됨
    except KeyboardInterrupt:
        print("\n사용자 종료")

    finally:
        # 프로그램이 정상 종료되든 오류로 종료되든
        # 마지막에는 시리얼 연결을 닫아주는 것이 좋음
        try:
            arduino.close()
            print("시리얼 연결 종료")

        # arduino 객체가 생성되기 전에 오류가 났을 수도 있으므로
        # close()에서 오류가 나도 그냥 넘어가게 처리
        except:
            pass


# 이 파일을 직접 실행했을 때만 main() 함수 실행
# 다른 파일에서 import할 경우에는 자동 실행되지 않음
if __name__ == "__main__":
    main()
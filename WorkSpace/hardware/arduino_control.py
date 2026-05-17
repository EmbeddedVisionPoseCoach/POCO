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
# Drowsy(졸림) 상태 관리용 변수
# =========================

# 졸림 상태가 처음 감지된 시간
drowsy_start_time = None

# Drowsy 알림 상태 기록용 변수
# 현재 구조에서는 Drowsy가 해제되었는지 판단할 때 사용
last_sent_drowsy = False


# ==================================================
# 사용자 설정 유지시간
#
# 사용자가 설정 가능:
# - 자세 유지시간
# - 졸음 유지시간
#
# 사용자가 설정 불가:
# - LED/부저 ON-OFF 시간
# ==================================================

# 자세 유지시간(초)
# Asymmetric
# ForwardHead
# ChinPropping
posture_hold_seconds = 3


# 졸음 유지시간(초)
# Drowsy
drowsy_hold_seconds = 5


# ==================================================
# 강한 알람 / 쿨타임 설정값
# ==================================================

# 자세 일반 알람이 몇 회 반복되면 강한 알람으로 바꿀지
posture_strong_alert_limit = 3

# 졸음 일반 알람이 몇 회 반복되면 강한 알람으로 바꿀지
drowsy_strong_alert_limit = 2

# 강한 알람 후 같은 알람을 몇 분 동안 중단할지
alert_cooldown_minutes = 5


# ==================================================
# 알람 반복 횟수 / 쿨타임 상태 변수
# ==================================================

## 아래 주석 부분 수정 
# # 현재 같은 알람이 몇 번 울렸는지 저장
# current_alert_repeat_count = 0

# # 현재 반복 카운트 중인 알람 종류
# # 예: "ForwardHead", "Drowsy"
# current_alert_command = None
## 여기까지

# 마지막으로 연속 카운트 중인 알람 command
last_alert_command = None

# 알람 종류별 연속 반복 횟수 저장
# 예: {"ForwardHead": 2, "Drowsy": 1}
continuous_alert_counts = {}

# 쿨타임이 끝나는 시간
# 예: {"ForwardHead": 1710000000.0}
cooldown_until = {}


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


# ==================================================
# 자세 알림 횟수 설정
# --------------------------------------------------
# 라즈베리 → 아두이노
#
# 예:
# SET_POSTURE_COUNT:5
#
# 자세 알림:
# Asymmetric
# ForwardHead
# ChinPropping
#
# 횟수 변경
# ==================================================
def set_posture_alert_count(
    arduino,
    count
):

    command = (
        f"SET_POSTURE_COUNT:{count}"
    )

    send_command(
        arduino,
        command
    )



# ==================================================
# 졸음 알림 횟수 설정
# --------------------------------------------------
# 라즈베리 → 아두이노
#
# 예:
# SET_DROWSY_COUNT:3
#
# Drowsy 알림 횟수 변경
# ==================================================
def set_drowsy_alert_count(
    arduino,
    count
):

    command = (
        f"SET_DROWSY_COUNT:{count}"
    )

    send_command(
        arduino,
        command
    )


# ==================================================
# 자세 유지시간 설정
# --------------------------------------------------
# PyQt에서 사용자가 설정한 자세 유지시간을 저장한다.
#
# 예:
# 자세 유지시간 3초
# 자세 유지시간 5초
#
# 적용 대상:
# - Asymmetric
# - ForwardHead
# - ChinPropping
# ==================================================
def set_posture_hold_seconds(seconds):
    global posture_hold_seconds

    # 너무 작거나 큰 값이 들어오는 것을 방지
    posture_hold_seconds = max(1, min(int(seconds), 60))

    print(
        f"[Hardware Setting] 자세 유지시간: {posture_hold_seconds}초"
    )


# ==================================================
# 졸음 유지시간 설정
# --------------------------------------------------
# PyQt에서 사용자가 설정한 졸음 유지시간을 저장한다.
#
# 예:
# 졸음 유지시간 5초
# 졸음 유지시간 10초
#
# 적용 대상:
# - Drowsy
# ==================================================
def set_drowsy_hold_seconds(seconds):
    global drowsy_hold_seconds

    # 너무 작거나 큰 값이 들어오는 것을 방지
    drowsy_hold_seconds = max(1, min(int(seconds), 60))

    print(
        f"[Hardware Setting] 졸음 유지시간: {drowsy_hold_seconds}초"
    )


# ==================================================
# StrongAlert / 쿨타임 설정
# --------------------------------------------------
# PyQt에서 사용자가 설정한 강한 알람 기준값을 저장한다.
#
# posture_limit:
#   자세 일반 알람이 몇 회 연속 반복되면 StrongAlert로 바꿀지
#
# drowsy_limit:
#   졸음 일반 알람이 몇 회 연속 반복되면 StrongAlert로 바꿀지
#
# cooldown_minutes:
#   StrongAlert 이후 같은 알람을 몇 분 동안 중단할지
#
# 주의:
#   이 값들은 아두이노로 보내는 값이 아니다.
#   StrongAlert 판단은 라즈베리파이 Python에서 처리한다.
# ==================================================
def set_strong_alert_settings(
    posture_limit,
    drowsy_limit,
    cooldown_minutes
):
    global posture_strong_alert_limit
    global drowsy_strong_alert_limit
    global alert_cooldown_minutes

    # 너무 작거나 큰 값이 들어오는 것을 방지
    posture_strong_alert_limit = max(1, min(int(posture_limit), 20))
    drowsy_strong_alert_limit = max(1, min(int(drowsy_limit), 20))
    alert_cooldown_minutes = max(1, min(int(cooldown_minutes), 60))

    print(
        "[Hardware Setting] StrongAlert 설정 완료 "
        f"(posture_limit={posture_strong_alert_limit}, "
        f"drowsy_limit={drowsy_strong_alert_limit}, "
        f"cooldown={alert_cooldown_minutes}분)"
    )


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
# 자세 판단 함수
# 실제 자세 판단 결과와 연결할 부분
# =========================

def convert_class_idx_to_command(class_idx):
    """
    AI class_idx를 아두이노로 보낼 문자열로 변환
    AI class_idx:
        0: "Optimal",
        1: "Asymmetric",
        2: "ForwardHead",
        3: "ChinPropping"

        Face는 문자열로 넘어옴
        Normal
        Drowsy
        

    """

    commands = {
        0: "Optimal",
        1: "Asymmetric",
        2: "ForwardHead",
        3: "ChinPropping"
    }

    return commands.get(class_idx)


# ==================================================
# 일반 알람 / 강한 알람 / 쿨타임 판단 함수
# ==================================================
def process_alert_with_cooldown(command, strong_limit):
    """
    같은 command가 '연속으로' 반복될 때만 StrongAlert를 발생시킨다.

    예:
        ForwardHead → ForwardHead → ForwardHead
        → StrongAlert 가능

        ForwardHead → ChinPropping → ForwardHead
        → 연속이 아니므로 StrongAlert 아님

    StrongAlert 이후에는 해당 command만 쿨타임에 들어간다.
    """

    global last_alert_command
    global continuous_alert_counts
    global cooldown_until

    now = time.time()

    # =========================
    # 1. 쿨타임 확인
    # =========================
    # StrongAlert가 이미 울린 command라면
    # cooldown_until 딕셔너리에 종료 시간이 저장되어 있다.
    #
    # 예:
    # cooldown_until = {
    #     "ForwardHead": 1710000000.0
    # }
    #
    # 현재 시간이 종료 시간보다 작으면 아직 쿨타임 중이므로
    # 같은 command 알림은 보내지 않는다.
    if command in cooldown_until and now < cooldown_until[command]:
        return None

    # =========================
    # 2. 쿨타임 종료 처리
    # =========================
    # 현재 시간이 쿨타임 종료 시간보다 크거나 같으면
    # 해당 command는 다시 알림을 보낼 수 있는 상태가 된다.
    if command in cooldown_until and now >= cooldown_until[command]:

        # 쿨타임 목록에서 제거
        del cooldown_until[command]

        # 해당 command의 연속 카운트 초기화
        continuous_alert_counts[command] = 0

        # 마지막 알람이 현재 command였다면
        # 연속 판단도 다시 시작할 수 있도록 초기화
        if last_alert_command == command:
            last_alert_command = None

    # =========================
    # 3. 연속 알람 카운트 처리
    # =========================
    # 직전에 처리한 알람과 현재 알람이 같으면
    # 같은 알람이 연속으로 들어온 것이므로 카운트를 증가시킨다.
    if last_alert_command == command:
        continuous_alert_counts[command] = (
            continuous_alert_counts.get(command, 0) + 1
        )

    # 직전 알람과 현재 알람이 다르면
    # 연속 흐름이 끊긴 것이다.
    #
    # 예:
    # ForwardHead → ChinPropping
    #
    # 이 경우 ChinPropping은 1회차부터 새로 시작한다.
    else:
        continuous_alert_counts[command] = 1
        last_alert_command = command

    # 현재 command의 연속 알림 횟수
    current_count = continuous_alert_counts[command]

    # =========================
    # 4. StrongAlert 판단
    # =========================
    # 같은 command가 strong_limit 횟수만큼 연속되면
    # StrongAlert를 반환한다.
    #
    # 예:
    # strong_limit = 3
    #
    # ForwardHead 1회 → 일반 알람
    # ForwardHead 2회 → 일반 알람
    # ForwardHead 3회 → StrongAlert
    if current_count >= strong_limit:

        # 해당 command만 쿨타임 시작
        cooldown_until[command] = now + (alert_cooldown_minutes * 60)

        # StrongAlert 이후 해당 command 연속 카운트 초기화
        continuous_alert_counts[command] = 0

        # 마지막 알람도 초기화해서 다음 알람은 새로 시작
        last_alert_command = None

        return "StrongAlert"

    # 아직 strong_limit에 도달하지 않았다면 일반 알람 반환
    return command



def get_posture_result_from_ai(class_idx):
    global last_sent_idx
    global bad_start_time
    global current_bad_idx

    now = time.time()

    # =========================
    # 0번: Optimal 상태
    # 정상 자세는 바로 전송
    # =========================
    if class_idx == 0:
        bad_start_time = None
        current_bad_idx = None

        # 이전에 보낸 값과 다를 때만 전송
        if last_sent_idx != class_idx:
            last_sent_idx = class_idx
            return "Optimal"

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

        # 설정된 유지시간 이상 지속되고,
        # 아직 같은 값을 보내지 않았다면 전송
        if elapsed_time >= posture_hold_seconds:
            last_sent_idx = class_idx

            # 알람을 한 번 보낸 뒤에도 같은 자세가 계속 유지되면
            # 다시 posture_hold_seconds 만큼 시간을 재서 다음 알람을 보낸다.
            # 그래야 같은 자세 알람이 연속으로 카운트되어 StrongAlert까지 갈 수 있다.
            bad_start_time = now

            command = convert_class_idx_to_command(class_idx)

            return process_alert_with_cooldown(
                command,
                posture_strong_alert_limit
            )

        return None
    

# ==================================================
# Pose + Face 결과를 합쳐 최종 알림 우선순위를 결정
# ==================================================
def get_alert_result_from_ai(pose_index, fatigue_index):
    """
    AI 추론 결과(pose + fatigue)를 받아
    우선순위 기준으로 최종 알림 command를 생성한다.

    Parameters
    ----------
    pose_index:
        자세 모델 결과

        0: Optimal
        1: Asymmetric
        2: ForwardHead
        3: ChinPropping


    fatigue_index:
        피로도 모델 결과

        0: Normal
        1: Drowsy


    우선순위:
        Drowsy > ChinPropping > ForwardHead > Asymmetric > Optimal

    동작 기준:
        - Drowsy는 5초 이상 지속될 때 알림
        - 자세 불량은 기존처럼 3초 유지 후 알림
        - Optimal은 바로 전송
    """

    global drowsy_start_time
    global last_sent_drowsy
    global bad_start_time
    global current_bad_idx

    now = time.time()

    # =========================
    # 1순위: Drowsy
    # =========================
    if fatigue_index == 1:

        # Drowsy가 감지되는 동안에는
        # 자세 판단보다 Drowsy 판단을 우선한다.
        #
        # 따라서 이전에 진행 중이던 자세 불량 유지시간은 초기화한다.
        # 예:
        # ForwardHead 2초 감지 중 Drowsy가 들어오면
        # ForwardHead 타이머는 다시 0초부터 보도록 한다.
        bad_start_time = None
        current_bad_idx = None

        # 처음 Drowsy가 감지된 순간 시간 기록
        if drowsy_start_time is None:
            drowsy_start_time = now
            return None

        elapsed_time = now - drowsy_start_time


        # 설정된 유지시간 이상 유지되고,
        # 아직 Drowsy를 보내지 않았다면
        if elapsed_time >= drowsy_hold_seconds:
            last_sent_drowsy = True

            # 알람을 한 번 보낸 뒤에도 Drowsy가 계속 유지되면
            # 다시 drowsy_hold_seconds 만큼 시간을 재서 다음 알람을 보낸다.
            # 그래야 Drowsy 알람도 연속 카운트되어 StrongAlert까지 갈 수 있다.
            drowsy_start_time = now

            return process_alert_with_cooldown(
                "Drowsy",
                drowsy_strong_alert_limit
            )

        return None


    # =========================
    # Drowsy가 아닌 경우
    # =========================
    # Drowsy 유지시간 초기화
    drowsy_start_time = None
    last_sent_drowsy = False


    # Drowsy가 아니면 기존 자세 알림 로직으로 처리
    # =========================
    # 기존 자세 판단 사용
    # =========================
    return get_posture_result_from_ai(
        pose_index
    )



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
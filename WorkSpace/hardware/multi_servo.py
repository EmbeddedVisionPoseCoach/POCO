#!/usr/bin/env python
import sys
import os
import time


BASE_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

SDK_PATH = os.path.join(
    BASE_DIR,
    "stservo_env",
    "scservo_sdk"
)

if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

try:
    from port_handler import PortHandler
    from sms_sts import sms_sts
    from scservo_def import COMM_SUCCESS
except ModuleNotFoundError:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
    from scservo_sdk.scservo_def import COMM_SUCCESS


DEVICENAME = "/dev/ttyACM0"
BAUDRATE = 1000000

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    raise SystemExit("포트 오픈 실패!")

try:
    if not portHandler.setBaudRate(BAUDRATE):
        raise SystemExit("보레이트 설정 실패!")

    print("✅ 다중 모터 제어 테스트 시작 (1번 ~ 4번)")

    servo_ids = [1, 2, 3, 4]
    target_position = 2048
    speed = 2000
    acc = 50

    for servo_id in servo_ids:
        print(f"👉 [ID: {servo_id}] 모터를 위치 {target_position}으로 이동 중...")

        result, error = packetHandler.WritePosEx(
            servo_id,
            target_position,
            speed,
            acc,
        )

        if result != COMM_SUCCESS:
            print(
                f"ID {servo_id} 통신 에러: "
                f"{packetHandler.getTxRxResult(result)}"
            )
        elif error != 0:
            print(
                f"ID {servo_id} 모터 에러 발생: "
                f"{packetHandler.getRxPacketError(error)}"
            )
        else:
            print(f"ID {servo_id} 명령 전송 완료!")

        time.sleep(1.0)

    print("\n모든 모터 순차 제어 완료!")
finally:
    portHandler.closePort()

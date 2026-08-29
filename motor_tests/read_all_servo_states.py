#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
read_all_servo_states.py

[이 코드가 하는 역할]
STS3215 Servo ID 1~4의 현재 상태값만 읽어서 터미널에 출력하는
"읽기 전용" 진단 프로그램이다.

이 파일은 Calibration / Zero / 방향 / Safe Range / TEAM 각도를 사용하지 않는다.
Servo의 현재 RAW 상태를 확인하기 위한 용도이다.

------------------------------------------------------------
[읽는 값]

- Servo ID
- Model Number (Ping)
- Position
- Speed
- Load
- Load Percent
- Voltage
- Temperature
- Current Raw
- Moving

------------------------------------------------------------
[중요]

이 프로그램은 모터에 이동 명령을 보내지 않는다.

사용하지 않는 기능:
- WritePosEx()
- SyncWritePosEx()
- Torque ON/OFF Write
- Goal Position Write

즉 실행해도 Servo 목표 위치나 Torque 상태를 변경하지 않는다.

------------------------------------------------------------
[기본 통신 설정]

Device   : /dev/ttyACM0
Baudrate : 1,000,000

Servo:
ID 1 = shoulder_lift
ID 2 = elbow_flex
ID 3 = wrist_flex
ID 4 = wrist_roll

------------------------------------------------------------
[실행]

예:
    cd ~/VisionPoseCoach
    python motor_tests/read_all_servo_states.py

메뉴:
    1 = ID 1~4 상태 한 번 읽기
    2 = ID 1~4 상태 계속 읽기 (0.5초 간격)
    q = 종료

연속 읽기는 Ctrl+C로 종료한다.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path


# ============================================================
# 1. Servo / Serial 설정
# ============================================================

DEVICENAME = "/dev/ttyACM0"
BAUDRATE = 1_000_000

SERVO_MAP = {
    1: "shoulder_lift",
    2: "elbow_flex",
    3: "wrist_flex",
    4: "wrist_roll",
}

READ_INTERVAL_SEC = 0.5


# ============================================================
# 2. STServo 상태 레지스터
# ============================================================
#
# 현재 VisionPoseCoach motor_control/servo_driver.py에서
# 상태 읽기에 사용하는 주소와 동일한 값이다.
#
# 주의:
# 여기서는 READ만 수행한다.

ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69


# ============================================================
# 3. STServo SDK Import
# ============================================================
#
# 우선 pip로 설치된 scservo_sdk를 사용하고,
# 찾지 못하면 현재 VisionPoseCoach 내부의 로컬 SDK 경로도 확인한다.

PROJECT_ROOT = Path(__file__).resolve().parent.parent

LOCAL_SDK_CANDIDATES = [
    PROJECT_ROOT
    / "WorkSpace"
    / "hardware"
    / "STServo_Python"
    / "stservo-env"
    / "scservo_sdk",

    PROJECT_ROOT
    / "STServo_Python"
    / "stservo-env"
    / "scservo_sdk",
]


try:
    # from WorkSpace.hardware.stservo_env.scservo_sdk.port_handler import PortHandler
    # from WorkSpace.hardware.stservo_env.scservo_sdk.sms_sts import sms_sts
    # from WorkSpace.hardware.stservo_env.scservo_sdk.scservo_def import COMM_SUCCESS
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
    from scservo_sdk.scservo_def import COMM_SUCCESS

except ModuleNotFoundError:

    sdk_found = False

    for sdk_path in LOCAL_SDK_CANDIDATES:
        if sdk_path.exists():

            sdk_parent = str(
                sdk_path.parent
            )

            if sdk_parent not in sys.path:
                sys.path.insert(
                    0,
                    sdk_parent,
                )

            sdk_found = True
            break

    if not sdk_found:
        print()
        print("============================================================")
        print("[FAIL] STServo SDK를 찾을 수 없습니다.")
        print("============================================================")
        print(
            "pip 설치 또는 프로젝트의 STServo_Python 경로를 확인하세요."
        )
        raise SystemExit(1)

    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
    from scservo_sdk.scservo_def import COMM_SUCCESS


# ============================================================
# 4. 통신 객체 생성
# ============================================================

port_handler = PortHandler(
    DEVICENAME
)

packet_handler = sms_sts(
    port_handler
)


# ============================================================
# 5. 통신 에러 표시
# ============================================================

def communication_error_text(
    result,
    error,
):
    """
    SDK 통신 결과를 사람이 보기 쉬운 문자열로 변환한다.
    """

    texts = []

    if result != COMM_SUCCESS:
        try:
            texts.append(
                packet_handler.getTxRxResult(
                    result
                )
            )
        except Exception:
            texts.append(
                f"COMM result={result}"
            )

    if error != 0:
        try:
            texts.append(
                packet_handler.getRxPacketError(
                    error
                )
            )
        except Exception:
            texts.append(
                f"Servo error={error}"
            )

    return " / ".join(
        texts
    )


# ============================================================
# 6. Servo Ping
# ============================================================

def ping_servo(
    servo_id: int,
):
    """
    Servo가 통신 가능한지 확인하고 Model Number를 읽는다.
    """

    model_number, result, error = (
        packet_handler.ping(
            servo_id
        )
    )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        return {
            "success": False,
            "model_number": None,
            "error": communication_error_text(
                result,
                error,
            ),
        }

    return {
        "success": True,
        "model_number": int(
            model_number
        ),
        "error": None,
    }


# ============================================================
# 7. Servo 상태 읽기
# ============================================================

def read_servo_state(
    servo_id: int,
):
    """
    한 Servo의 현재 상태를 읽는다.

    이 함수에서는 모든 동작이 READ 명령이다.
    Servo Position/Goal/Torque를 변경하지 않는다.
    """

    # --------------------------------------------------------
    # Position
    # --------------------------------------------------------

    position, result, error = (
        packet_handler.ReadPos(
            servo_id
        )
    )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        return {
            "success": False,
            "servo_id": servo_id,
            "error": (
                "Position read failed: "
                + communication_error_text(
                    result,
                    error,
                )
            ),
        }

    position = int(
        position
    )

    # --------------------------------------------------------
    # Speed
    # --------------------------------------------------------

    speed, result, error = (
        packet_handler.ReadSpeed(
            servo_id
        )
    )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        speed = None
    else:
        speed = int(
            speed
        )

    # --------------------------------------------------------
    # Load
    # --------------------------------------------------------

    load_raw, result, error = (
        packet_handler.read2ByteTxRx(
            servo_id,
            ADDR_PRESENT_LOAD,
        )
    )

    if (
        result == COMM_SUCCESS
        and error == 0
    ):

        # STS Load:
        # 하위 10bit = 크기
        # bit10      = 방향
        load_value = (
            load_raw
            & 0x03FF
        )

        if (
            load_raw
            & 0x0400
        ):
            load_value = (
                -load_value
            )

        load_percent = (
            abs(
                load_value
            )
            / 1000.0
            * 100.0
        )

    else:
        load_value = None
        load_percent = None

    # --------------------------------------------------------
    # Voltage
    # --------------------------------------------------------

    voltage_raw, result, error = (
        packet_handler.read1ByteTxRx(
            servo_id,
            ADDR_PRESENT_VOLTAGE,
        )
    )

    if (
        result == COMM_SUCCESS
        and error == 0
    ):
        voltage = (
            float(
                voltage_raw
            )
            * 0.1
        )
    else:
        voltage = None

    # --------------------------------------------------------
    # Temperature
    # --------------------------------------------------------

    temperature, result, error = (
        packet_handler.read1ByteTxRx(
            servo_id,
            ADDR_PRESENT_TEMPERATURE,
        )
    )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        temperature = None
    else:
        temperature = int(
            temperature
        )

    # --------------------------------------------------------
    # Current
    # --------------------------------------------------------
    #
    # 현재 프로젝트에서도 mA 변환계수를 최종 확정하지 않았기 때문에
    # Current는 raw 값 그대로 표시한다.

    current_raw, result, error = (
        packet_handler.read2ByteTxRx(
            servo_id,
            ADDR_PRESENT_CURRENT,
        )
    )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        current_raw = None
    else:
        current_raw = int(
            current_raw
        )

    # --------------------------------------------------------
    # Moving
    # --------------------------------------------------------

    moving, result, error = (
        packet_handler.read1ByteTxRx(
            servo_id,
            ADDR_MOVING,
        )
    )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        moving = None
    else:
        moving = int(
            moving
        )

    return {
        "success": True,
        "servo_id": servo_id,
        "joint": SERVO_MAP.get(
            servo_id,
            "unknown",
        ),
        "position": position,
        "speed": speed,
        "load": load_value,
        "load_percent": load_percent,
        "voltage": voltage,
        "temperature": temperature,
        "current_raw": current_raw,
        "moving": moving,
        "error": None,
    }


# ============================================================
# 8. 값 표시 Helper
# ============================================================

def value_or_dash(
    value,
    format_spec=None,
):
    """
    읽기 실패(None) 값은 '-'로 표시한다.
    """

    if value is None:
        return "-"

    if format_spec:
        return format(
            value,
            format_spec,
        )

    return str(
        value
    )


def print_single_state(
    state,
    model_number=None,
):
    """
    Servo 하나의 상태를 사람이 보기 쉽게 출력한다.
    """

    servo_id = state.get(
        "servo_id"
    )

    joint = state.get(
        "joint",
        SERVO_MAP.get(
            servo_id,
            "unknown",
        ),
    )

    print()
    print(
        "------------------------------------------------------------"
    )
    print(
        f"ID {servo_id} | {joint}"
    )
    print(
        "------------------------------------------------------------"
    )

    if not state.get(
        "success"
    ):
        print(
            f"[READ FAIL] {state.get('error')}"
        )
        return

    print(
        f"Model Number : "
        f"{value_or_dash(model_number)}"
    )

    print(
        f"Position     : "
        f"{value_or_dash(state['position'])}"
    )

    print(
        f"Speed        : "
        f"{value_or_dash(state['speed'])}"
    )

    print(
        f"Load         : "
        f"{value_or_dash(state['load'])}"
    )

    print(
        f"Load Percent : "
        f"{value_or_dash(state['load_percent'], '.1f')} %"
    )

    print(
        f"Voltage      : "
        f"{value_or_dash(state['voltage'], '.1f')} V"
    )

    print(
        f"Temperature  : "
        f"{value_or_dash(state['temperature'])} °C"
    )

    print(
        f"Current Raw  : "
        f"{value_or_dash(state['current_raw'])}"
    )

    moving = state.get(
        "moving"
    )

    if moving == 1:
        moving_text = "1 (MOVING)"
    elif moving == 0:
        moving_text = "0 (STOPPED)"
    else:
        moving_text = "-"

    print(
        f"Moving       : "
        f"{moving_text}"
    )


# ============================================================
# 9. ID 1~4 한 번 읽기
# ============================================================

def read_all_once():
    """
    ID 1~4를 순서대로 Ping하고 상태를 한 번씩 읽는다.
    """

    print()
    print(
        "============================================================"
    )
    print(
        " STS3215 ID 1~4 CURRENT STATE"
    )
    print(
        "============================================================"
    )

    for servo_id in SERVO_MAP:

        ping = ping_servo(
            servo_id
        )

        if not ping[
            "success"
        ]:

            print()
            print(
                "------------------------------------------------------------"
            )
            print(
                f"ID {servo_id} | "
                f"{SERVO_MAP[servo_id]}"
            )
            print(
                "------------------------------------------------------------"
            )
            print(
                f"[PING FAIL] "
                f"{ping['error']}"
            )

            continue

        state = read_servo_state(
            servo_id
        )

        print_single_state(
            state,
            model_number=ping[
                "model_number"
            ],
        )

    print()


# ============================================================
# 10. ID 1~4 연속 읽기
# ============================================================

def read_all_continuously():
    """
    ID 1~4의 상태를 0.5초 간격으로 반복 출력한다.

    모터를 직접 움직여보거나 손으로 부하를 줬을 때
    Position / Load / Current / Moving 등이 어떻게 변하는지
    확인할 때 사용할 수 있다.

    Ctrl+C로 종료한다.
    """

    print()
    print(
        "============================================================"
    )
    print(
        " CONTINUOUS READ MODE"
    )
    print(
        "============================================================"
    )
    print(
        f"Update Interval : "
        f"{READ_INTERVAL_SEC:.1f} sec"
    )
    print(
        "종료: Ctrl+C"
    )
    print()

    try:
        while True:

            timestamp = time.strftime(
                "%H:%M:%S"
            )

            print(
                "============================================================"
            )
            print(
                f"[{timestamp}] ID 1~4 상태"
            )
            print(
                "============================================================"
            )

            # 연속 모드에서는 화면을 너무 길게 쓰지 않도록
            # 한 Servo를 한 줄로 요약한다.
            print(
                "ID | Joint           | Pos  | Speed | Load% | Volt | Temp | Current | Moving"
            )
            print(
                "---+-----------------+------+-------+-------+------+------+---------+-------"
            )

            for servo_id, joint in (
                SERVO_MAP.items()
            ):

                state = read_servo_state(
                    servo_id
                )

                if not state.get(
                    "success"
                ):
                    print(
                        f"{servo_id:>2} | "
                        f"{joint:<15} | "
                        f"READ FAIL: "
                        f"{state.get('error')}"
                    )
                    continue

                moving = state[
                    "moving"
                ]

                if moving == 1:
                    moving_text = "YES"
                elif moving == 0:
                    moving_text = "NO"
                else:
                    moving_text = "-"

                print(
                    f"{servo_id:>2} | "
                    f"{joint:<15} | "
                    f"{value_or_dash(state['position']):>4} | "
                    f"{value_or_dash(state['speed']):>5} | "
                    f"{value_or_dash(state['load_percent'], '.1f'):>5} | "
                    f"{value_or_dash(state['voltage'], '.1f'):>4} | "
                    f"{value_or_dash(state['temperature']):>4} | "
                    f"{value_or_dash(state['current_raw']):>7} | "
                    f"{moving_text:>6}"
                )

            print()

            time.sleep(
                READ_INTERVAL_SEC
            )

    except KeyboardInterrupt:
        print()
        print(
            "[INFO] 연속 상태 읽기를 종료합니다."
        )


# ============================================================
# 11. Main
# ============================================================

def main():

    print()
    print(
        "============================================================"
    )
    print(
        " STS3215 READ-ONLY STATUS CHECK"
    )
    print(
        "============================================================"
    )
    print(
        f"Device   : {DEVICENAME}"
    )
    print(
        f"Baudrate : {BAUDRATE}"
    )
    print()
    print(
        "[READ ONLY]"
    )
    print(
        "이 프로그램은 Servo 이동/Torque/Goal Position을 변경하지 않습니다."
    )

    if not os.path.exists(
        DEVICENAME
    ):
        print()
        print(
            f"[FAIL] Serial Device 없음: "
            f"{DEVICENAME}"
        )
        raise SystemExit(1)

    if not port_handler.openPort():
        print()
        print(
            f"[FAIL] Port Open 실패: "
            f"{DEVICENAME}"
        )
        raise SystemExit(1)

    if not port_handler.setBaudRate(
        BAUDRATE
    ):
        port_handler.closePort()

        print()
        print(
            f"[FAIL] Baudrate 설정 실패: "
            f"{BAUDRATE}"
        )

        raise SystemExit(1)

    print()
    print(
        "[OK] Serial Port 연결 성공"
    )

    try:
        while True:

            print()
            print(
                "1 = ID 1~4 상태 한 번 읽기"
            )
            print(
                "2 = ID 1~4 상태 계속 읽기"
            )
            print(
                "q = 종료"
            )

            command = input(
                "선택: "
            ).strip().lower()

            if command == "1":
                read_all_once()

            elif command == "2":
                read_all_continuously()

            elif command == "q":
                break

            else:
                print(
                    "[INFO] 1, 2, q 중 하나를 입력하세요."
                )

    finally:
        port_handler.closePort()

        print()
        print(
            "[OK] Port 종료"
        )


if __name__ == "__main__":
    main()

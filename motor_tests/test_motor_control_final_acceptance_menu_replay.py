#!/usr/bin/env python3
"""
test_motor_control_final_acceptance.py

[목적]
POCO 프로젝트에서 최종 확정된 motor_control 패키지를
"팀원이 실제로 사용하는 방식"으로 검증하는 최종 하드웨어 인수 테스트.

중요 원칙
---------
1. 실제 모터 동작 명령은 반드시 공개 API인 MotorController 메서드로만 수행한다.
2. ServoDriver의 Write 함수나 Calibration 내부 변환 함수를 이용해
   테스트를 우회하지 않는다.
3. Goal Position / Torque Enable처럼 공개 API만으로 확인할 수 없는 값은
   같은 MotorController가 이미 소유한 연결을 통해 "읽기 전용 진단"만 수행한다.
   진단 함수에는 Write가 없다.
4. 실제 물리 방향과 자세 유지 여부는 프로그램이 판단할 수 없으므로
   사용자가 로봇을 직접 보고 y/N으로 확인한다.
5. 기존 emergency_stop()은 Torque OFF이므로 모든 일반 테스트가 끝난 뒤
   가장 마지막에만 실행한다.

[검증 대상 공개 API]
- MotorController()
- move_joint()
- move_joint_relative()
- move_joints()
- move_to_zero()
- move_all_to_zero()
- get_joint_angle()
- get_joint_state()
- get_all_states()
- is_moving()
- user_stop()
- is_user_stopped()
- resume_user_stop()
- emergency_stop()
- is_emergency_stopped()
- close()
- context manager (__enter__ / __exit__)

[실행 위치 권장]
파일 위치:
    ~/POCO/motor_tests/test_motor_control_final_acceptance.py

실행:
    cd ~/POCO
    python motor_tests/test_motor_control_final_acceptance.py

이 파일은 자신의 위치를 기준으로 ~/POCO/WorkSpace를 sys.path에 추가하므로
현재 터미널 위치(CWD)에 의존하지 않는다.

[주의]
- 로봇팔 주변 장애물을 모두 치운 뒤 실행한다.
- 처음에는 사람이 즉시 전원을 차단할 수 있는 위치에서 테스트한다.
- 마지막 emergency_stop() 테스트에서는 Torque가 OFF되므로
  반드시 로봇팔을 손으로 지지한 상태에서 실행한다.
"""

import json
import math
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path


# ============================================================
# 0. 프로젝트 경로 설정
# ============================================================
#
# 이 테스트 파일을:
#   POCO/motor_tests/test_motor_control_final_acceptance.py
# 에 둔다는 전제다.
#
# 따라서 현재 파일의 상위 상위 폴더가 POCO 프로젝트 루트이다.

TEST_FILE = Path(__file__).resolve()
PROJECT_ROOT = TEST_FILE.parent.parent
WORKSPACE_DIR = PROJECT_ROOT / "WorkSpace"

if not WORKSPACE_DIR.exists():
    raise RuntimeError(
        f"WorkSpace 폴더를 찾을 수 없습니다: {WORKSPACE_DIR}\n"
        "이 파일을 POCO/motor_tests/ 아래에 두었는지 확인하세요."
    )

if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))


# 실제 팀원과 동일한 공개 import 방식
from hardware.motor_control import MotorController


# ============================================================
# 1. 테스트 기본 설정
# ============================================================

JOINTS = (
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

SERVO_ID_BY_JOINT = {
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_roll": 4,
}

# 팀원 기준 +방향 명령을 보냈을 때 예상되는 RAW Position 변화.
# 우리가 실제 하드웨어 방향 테스트로 확정한 기준이다.
EXPECTED_RAW_SIGN_POSITIVE = {
    "shoulder_lift": -1,  # TEAM + = 위 = RAW 감소
    "elbow_flex": -1,     # TEAM + = 위 = RAW 감소
    "wrist_flex": -1,     # TEAM + = 위 = RAW 감소
    "wrist_roll": +1,     # TEAM + = CW = RAW 증가
}

PHYSICAL_POSITIVE_TEXT = {
    "shoulder_lift": "팔 끝단이 '위' 방향으로 움직였습니까?",
    "elbow_flex": "팔꿈치 이후 링크/끝단이 '위' 방향으로 움직였습니까?",
    "wrist_flex": "손목 끝단이 '위' 방향으로 움직였습니까?",
    "wrist_roll": (
        "모니터가 위치한 정면에서 로봇팔을 바라봤을 때 "
        "'CW(시계 방향)'로 회전했습니까?"
    ),
}

PHYSICAL_NEGATIVE_TEXT = {
    "shoulder_lift": "팔 끝단이 '아래' 방향으로 움직였습니까?",
    "elbow_flex": "팔꿈치 이후 링크/끝단이 '아래' 방향으로 움직였습니까?",
    "wrist_flex": "손목 끝단이 '아래' 방향으로 움직였습니까?",
    "wrist_roll": (
        "모니터가 위치한 정면에서 로봇팔을 바라봤을 때 "
        "'CCW(반시계 방향)'로 회전했습니까?"
    ),
}

TEST_SPEED = 50
FAST_SPEED = 80
SLOW_SPEED = 30
USER_STOP_MOTION_SPEED = 20
TEST_ACC = 10

# 각도 검증 허용 오차.
# STS Position tolerance(5 step)보다 여유를 둔 하드웨어 인수 테스트 기준.
ANGLE_TOLERANCE_DEG = 1.5

# user_stop 후 Hold 목표와 실제 위치 사이 허용 RAW 차이.
USER_STOP_HOLD_TOLERANCE_RAW = 25

# 읽기 전용 진단용 STS 레지스터
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_MOVING = 66


# ============================================================
# 2. 결과 기록
# ============================================================

RESULTS = []
TEST_STARTED_AT = datetime.now()


def add_result(name, status, detail=""):
    status = str(status).upper()
    RESULTS.append(
        {
            "name": name,
            "status": status,
            "detail": str(detail),
            "time": datetime.now().isoformat(timespec="seconds"),
        }
    )

    marker = {
        "PASS": "[PASS]",
        "FAIL": "[FAIL]",
        "SKIP": "[SKIP]",
        "INFO": "[INFO]",
        "WARN": "[WARN]",
    }.get(status, f"[{status}]")

    print(f"{marker} {name}")
    if detail:
        print(f"       {detail}")


def section(title):
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def ask_yes_no(message):
    while True:
        answer = input(f"{message} [y/N]: ").strip().lower()

        if answer in ("y", "yes"):
            return True

        if answer in ("", "n", "no"):
            return False

        print("y 또는 n으로 입력하세요.")



def ask_visual_confirm(message, replay_callback=None):
    """
    실제 물리 동작을 사람이 눈으로 확인하는 항목용.

    y : 정상으로 확인
    n : 잘못된 동작으로 확인
    r : 동작을 제대로 못 봤으므로 동일 동작을 다시 실행

    replay_callback이 None이면 r은 사용할 수 없다.
    """
    while True:
        prompt = f"{message} [y/N"
        if replay_callback is not None:
            prompt += "/r=다시보기"
        prompt += "]: "

        answer = input(prompt).strip().lower()

        if answer in ("y", "yes"):
            return True

        if answer in ("", "n", "no"):
            return False

        if answer in ("r", "replay", "retry"):
            if replay_callback is None:
                print("[INFO] 이 항목은 다시보기를 지원하지 않습니다.")
                continue

            print()
            print("[REPLAY] 같은 동작을 다시 실행합니다.")

            try:
                replay_ok = replay_callback()
            except Exception as error:
                print(
                    f"[ERROR] 다시보기 실행 중 오류: "
                    f"{type(error).__name__}: {error}"
                )
                return False

            if replay_ok is False:
                print("[ERROR] 다시보기 동작 실행에 실패했습니다.")
                return False

            print("[REPLAY] 동작을 다시 확인한 뒤 y/n/r로 응답하세요.")
            continue

        print("y / n / r 중 하나를 입력하세요.")


def require_user_continue(message):
    print()
    print(message)
    input("준비되면 Enter를 누르세요: ")


# ============================================================
# 3. 읽기 전용 하드웨어 진단
# ============================================================
#
# 팀원이 사용할 모터 동작은 위의 공개 API만 사용한다.
# 아래 함수들은 테스트 검증을 위해 Goal/Torque/RAW Position을 읽기만 한다.
# WritePosEx / SyncWrite / Torque Write는 절대 호출하지 않는다.


def diagnostic_read_1byte(arm, servo_id, address):
    try:
        with arm.driver._io_lock:
            value, result, error = arm.driver.packet_handler.read1ByteTxRx(
                int(servo_id),
                int(address),
            )
    except Exception:
        return None

    if result != 0 or error != 0:
        return None

    return int(value)


def diagnostic_read_2byte(arm, servo_id, address):
    try:
        with arm.driver._io_lock:
            value, result, error = arm.driver.packet_handler.read2ByteTxRx(
                int(servo_id),
                int(address),
            )
    except Exception:
        return None

    if result != 0 or error != 0:
        return None

    return int(value)


def diagnostic_read_raw_position(arm, joint_name):
    servo_id = SERVO_ID_BY_JOINT[joint_name]
    # driver.read_position()은 내부 Driver 메서드지만 READ ONLY이다.
    return arm.driver.read_position(servo_id)


def diagnostic_read_goal(arm, joint_name):
    return diagnostic_read_2byte(
        arm,
        SERVO_ID_BY_JOINT[joint_name],
        ADDR_GOAL_POSITION,
    )


def diagnostic_read_torque(arm, joint_name):
    return diagnostic_read_1byte(
        arm,
        SERVO_ID_BY_JOINT[joint_name],
        ADDR_TORQUE_ENABLE,
    )


def diagnostic_goal_map(arm):
    return {
        joint: diagnostic_read_goal(arm, joint)
        for joint in JOINTS
    }


def diagnostic_torque_map(arm):
    return {
        joint: diagnostic_read_torque(arm, joint)
        for joint in JOINTS
    }


def diagnostic_position_map(arm):
    return {
        joint: diagnostic_read_raw_position(arm, joint)
        for joint in JOINTS
    }


# ============================================================
# 4. 공통 검증 Helper
# ============================================================


def angle_close(actual, expected, tolerance=ANGLE_TOLERANCE_DEG):
    if actual is None:
        return False

    return abs(float(actual) - float(expected)) <= float(tolerance)


def wait_until_angle_near(
    arm,
    joint_name,
    target_angle,
    timeout=5.0,
    tolerance=ANGLE_TOLERANCE_DEG,
):
    start = time.monotonic()

    while time.monotonic() - start < timeout:
        angle = arm.get_joint_angle(joint_name)

        if angle_close(angle, target_angle, tolerance):
            moving = arm.is_moving(joint_name)

            if moving is False or moving is None:
                return True

        time.sleep(0.05)

    return False


def wait_until_angle_at_least(
    arm,
    joint_name,
    threshold,
    timeout=3.0,
):
    start = time.monotonic()

    while time.monotonic() - start < timeout:
        angle = arm.get_joint_angle(joint_name)

        if angle is not None and float(angle) >= float(threshold):
            return True

        time.sleep(0.02)

    return False


def get_safe_test_angle(arm, joint_name, sign, preferred=10.0):
    """
    Safe Range 안쪽의 작은 테스트 각도를 자동 선택한다.

    테스트가 Safe Limit 가까이 가지 않도록
    각 방향 허용범위의 절반 이내에서만 테스트한다.
    """
    safe_min, safe_max = arm.calibration.get_safe_angle_range(joint_name)

    if not (safe_min < 0.0 < safe_max):
        raise RuntimeError(
            f"{joint_name} Safe Range가 Zero를 포함하지 않습니다: "
            f"{safe_min:.2f} ~ {safe_max:.2f}"
        )

    if sign > 0:
        available = float(safe_max)
        angle = min(float(preferred), available * 0.40)

        if angle < 2.0:
            raise RuntimeError(
                f"{joint_name} +방향 테스트 공간이 너무 작습니다: {safe_max:.2f}°"
            )

        return float(angle)

    available = abs(float(safe_min))
    angle = min(float(preferred), available * 0.40)

    if angle < 2.0:
        raise RuntimeError(
            f"{joint_name} -방향 테스트 공간이 너무 작습니다: {safe_min:.2f}°"
        )

    return -float(angle)


def check_no_goal_change(before, after):
    if before.keys() != after.keys():
        return False

    return all(before[key] == after[key] for key in before)


def print_state_table(arm):
    print()
    print(
        f"{'Joint':<18}"
        f"{'Angle':>10}"
        f"{'Raw':>8}"
        f"{'Moving':>10}"
        f"{'Torque':>9}"
        f"{'Goal':>8}"
    )
    print("-" * 63)

    for joint in JOINTS:
        angle = arm.get_joint_angle(joint)
        raw = diagnostic_read_raw_position(arm, joint)
        moving = arm.is_moving(joint)
        torque = diagnostic_read_torque(arm, joint)
        goal = diagnostic_read_goal(arm, joint)

        angle_text = "-" if angle is None else f"{angle:+.2f}"

        print(
            f"{joint:<18}"
            f"{angle_text:>10}"
            f"{str(raw):>8}"
            f"{str(moving):>10}"
            f"{str(torque):>9}"
            f"{str(goal):>8}"
        )


# ============================================================
# 5. Context Manager / 생성 / Close 기본 테스트
# ============================================================


def test_context_manager():
    section("TEST 0. MotorController 생성 + Context Manager + 자동 close")

    temp_arm = None

    try:
        with MotorController() as temp_arm:
            angle = temp_arm.get_joint_angle("wrist_flex")

            if angle is None:
                add_result(
                    "Context Manager 내부 상태 읽기",
                    "FAIL",
                    "wrist_flex angle을 읽지 못했습니다.",
                )
                return False

            add_result(
                "MotorController() + __enter__()",
                "PASS",
                f"wrist_flex={angle:+.2f}°",
            )

        # __exit__()가 close()를 호출했는지 진단
        if temp_arm.driver.is_open:
            add_result(
                "__exit__() -> close()",
                "FAIL",
                "Context Manager 종료 후 Serial Port가 아직 open 상태입니다.",
            )
            return False

        add_result(
            "__exit__() -> close()",
            "PASS",
            "Context Manager 종료 후 Serial Port가 정상적으로 닫혔습니다.",
        )
        return True

    except Exception as error:
        add_result(
            "Context Manager",
            "FAIL",
            f"{type(error).__name__}: {error}",
        )
        return False


# ============================================================
# 6. Preflight / Calibration / 상태 읽기
# ============================================================


def test_preflight(arm):
    section("TEST 1. Preflight + Calibration + 상태 읽기 공개 API")

    print(f"Project Root : {PROJECT_ROOT}")
    print(f"WorkSpace    : {WORKSPACE_DIR}")
    print(f"Device       : {arm.calibration.device}")
    print(f"Baudrate     : {arm.calibration.baudrate}")

    if arm.is_emergency_stopped():
        add_result(
            "초기 Emergency latch",
            "FAIL",
            "새 MotorController 생성 직후 Emergency latch가 ON입니다.",
        )
        return False

    if arm.is_user_stopped():
        add_result(
            "초기 User Stop latch",
            "FAIL",
            "새 MotorController 생성 직후 User Stop latch가 ON입니다.",
        )
        return False

    add_result(
        "초기 Stop latch",
        "PASS",
        "Emergency=False / UserStop=False",
    )

    # Calibration
    calibration_ok = True

    print()
    print("Calibration / Safe Range")
    print("-" * 78)

    for joint in JOINTS:
        try:
            servo = arm.calibration.get_joint(joint)
            safe_min, safe_max = arm.calibration.get_safe_angle_range(joint)

            print(
                f"{joint:<18} "
                f"ID={servo['servo_id']} "
                f"Zero={servo['zero_position']} "
                f"direction={servo['direction']:+d} "
                f"Safe={safe_min:+.2f}° ~ {safe_max:+.2f}° "
                f"max_speed={servo['max_speed']}"
            )

            if (
                servo.get("zero_position") is None
                or servo.get("safe_position_at_min_angle") is None
                or servo.get("safe_position_at_max_angle") is None
                or servo.get("max_speed") is None
            ):
                calibration_ok = False

        except Exception as error:
            calibration_ok = False
            print(f"{joint}: ERROR {error}")

    add_result(
        "4축 Calibration 필수값",
        "PASS" if calibration_ok else "FAIL",
        "Zero / Safe MIN/MAX / max_speed 확인",
    )

    if not calibration_ok:
        return False

    # get_joint_angle()
    angle_ok = True

    for joint in JOINTS:
        if arm.get_joint_angle(joint) is None:
            angle_ok = False

    add_result(
        "get_joint_angle() 4축",
        "PASS" if angle_ok else "FAIL",
    )

    # get_joint_state()
    required_state_keys = {
        "joint",
        "angle",
        "speed",
        "load",
        "load_percent",
        "voltage",
        "temperature",
        "current_raw",
        "moving",
    }

    state_ok = True

    for joint in JOINTS:
        state = arm.get_joint_state(joint)

        if state is None:
            state_ok = False
            continue

        if not required_state_keys.issubset(state.keys()):
            state_ok = False

    add_result(
        "get_joint_state() 4축",
        "PASS" if state_ok else "FAIL",
        "Angle/Speed/Load/Voltage/Temperature/Current/Moving 반환 확인",
    )

    # get_all_states()
    all_states = arm.get_all_states()
    all_states_ok = (
        isinstance(all_states, dict)
        and set(all_states.keys()) == set(JOINTS)
        and all(value is not None for value in all_states.values())
    )

    add_result(
        "get_all_states()",
        "PASS" if all_states_ok else "FAIL",
        "4축 상태 dictionary 확인",
    )

    # is_moving()
    moving_ok = True

    for joint in JOINTS:
        value = arm.is_moving(joint)

        if value not in (True, False, None):
            moving_ok = False

    add_result(
        "is_moving() 4축",
        "PASS" if moving_ok else "FAIL",
    )

    # Torque가 이미 OFF인 상태에서는 이후 이동 테스트를 하면 안 된다.
    torque_map = diagnostic_torque_map(arm)

    torque_ok = all(value == 1 for value in torque_map.values())

    add_result(
        "초기 Torque Enable",
        "PASS" if torque_ok else "FAIL",
        str(torque_map),
    )

    print_state_table(arm)

    return (
        angle_ok
        and state_ok
        and all_states_ok
        and moving_ok
        and torque_ok
    )


# ============================================================
# 7. Zero 이동
# ============================================================


def test_zero_functions(arm):
    section("TEST 2. move_all_to_zero() + move_to_zero()")

    require_user_continue(
        "이제 실제 모터를 움직입니다.\n"
        "로봇팔 주변에 사람/물체가 없는지 확인하세요."
    )

    result = arm.move_all_to_zero(
        speed=TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    all_zero_ok = bool(result)

    for joint in JOINTS:
        angle = arm.get_joint_angle(joint)
        all_zero_ok = all_zero_ok and angle_close(angle, 0.0)

    add_result(
        "move_all_to_zero()",
        "PASS" if all_zero_ok else "FAIL",
        "4축 Zero 복귀",
    )

    if not all_zero_ok:
        return False

    # move_to_zero()는 wrist_flex를 작은 위치로 보낸 뒤 테스트한다.
    test_angle = get_safe_test_angle(
        arm,
        "wrist_flex",
        +1,
        preferred=8.0,
    )

    move_ok = arm.move_joint(
        "wrist_flex",
        test_angle,
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    zero_result = arm.move_to_zero(
        "wrist_flex",
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    final_angle = arm.get_joint_angle("wrist_flex")

    single_zero_ok = (
        bool(move_ok)
        and bool(zero_result)
        and angle_close(final_angle, 0.0)
    )

    add_result(
        "move_to_zero()",
        "PASS" if single_zero_ok else "FAIL",
        f"final={final_angle}",
    )

    return single_zero_ok


# ============================================================
# 8. 4축 TEAM +/- 방향 + move_joint()
# ============================================================


def _test_absolute_direction_all_joints_legacy(arm):
    section("TEST 3. move_joint() + TEAM +/- 실제 방향 4축 확인")

    for joint in JOINTS:
        print()
        print("-" * 78)
        print(f"[{joint}] TEAM + / TEAM - 방향 확인")
        print("-" * 78)

        # ---------------- Positive ----------------
        if not arm.move_to_zero(joint, TEST_SPEED, wait=True):
            add_result(f"{joint} Zero 준비", "FAIL")
            return False

        raw_zero = diagnostic_read_raw_position(arm, joint)
        positive_angle = get_safe_test_angle(
            arm,
            joint,
            +1,
            preferred=8.0,
        )

        # acc/wait를 생략하여 팀원이 가장 흔하게 사용하는 기본 인자도 검증한다.
        result = arm.move_joint(
            joint,
            positive_angle,
            TEST_SPEED,
        )

        raw_positive = diagnostic_read_raw_position(arm, joint)
        actual_angle = arm.get_joint_angle(joint)

        expected_sign = EXPECTED_RAW_SIGN_POSITIVE[joint]
        raw_delta = (
            None
            if raw_zero is None or raw_positive is None
            else raw_positive - raw_zero
        )

        raw_sign_ok = (
            raw_delta is not None
            and raw_delta != 0
            and (1 if raw_delta > 0 else -1) == expected_sign
        )

        angle_ok = angle_close(actual_angle, positive_angle)

        physical_ok = ask_yes_no(
            f"{joint} TEAM +: {PHYSICAL_POSITIVE_TEXT[joint]}"
        )

        positive_ok = bool(result) and raw_sign_ok and angle_ok and physical_ok

        add_result(
            f"{joint} move_joint(TEAM +)",
            "PASS" if positive_ok else "FAIL",
            (
                f"target={positive_angle:+.2f}°, actual={actual_angle}, "
                f"RAW {raw_zero}->{raw_positive} delta={raw_delta}"
            ),
        )

        # 잘못된 물리 방향이면 추가 모션 테스트를 진행하면 안 된다.
        arm.move_to_zero(joint, TEST_SPEED, wait=True)

        if not positive_ok:
            return False

        # ---------------- Negative ----------------
        raw_zero = diagnostic_read_raw_position(arm, joint)
        negative_angle = get_safe_test_angle(
            arm,
            joint,
            -1,
            preferred=8.0,
        )

        result = arm.move_joint(
            joint,
            negative_angle,
            TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        )

        raw_negative = diagnostic_read_raw_position(arm, joint)
        actual_angle = arm.get_joint_angle(joint)

        raw_delta = (
            None
            if raw_zero is None or raw_negative is None
            else raw_negative - raw_zero
        )

        expected_negative_sign = -EXPECTED_RAW_SIGN_POSITIVE[joint]

        raw_sign_ok = (
            raw_delta is not None
            and raw_delta != 0
            and (1 if raw_delta > 0 else -1) == expected_negative_sign
        )

        angle_ok = angle_close(actual_angle, negative_angle)

        physical_ok = ask_yes_no(
            f"{joint} TEAM -: {PHYSICAL_NEGATIVE_TEXT[joint]}"
        )

        negative_ok = bool(result) and raw_sign_ok and angle_ok and physical_ok

        add_result(
            f"{joint} move_joint(TEAM -)",
            "PASS" if negative_ok else "FAIL",
            (
                f"target={negative_angle:+.2f}°, actual={actual_angle}, "
                f"RAW {raw_zero}->{raw_negative} delta={raw_delta}"
            ),
        )

        arm.move_to_zero(joint, TEST_SPEED, wait=True)

        if not negative_ok:
            return False

    return True



# ============================================================
# 8-A. 단일 Joint TEAM +/- 방향 테스트
# ============================================================

def test_absolute_direction_one_joint(arm, joint):
    section(f"TEST. move_joint() TEAM +/- 실제 방향 - {joint}")

    if joint not in JOINTS:
        add_result(
            f"{joint} 방향 테스트",
            "FAIL",
            "알 수 없는 Joint",
        )
        return False

    print()
    print("-" * 78)
    print(f"[{joint}] TEAM + / TEAM - 방향 확인")
    print("-" * 78)

    # ========================================================
    # TEAM + 다시보기까지 포함한 실행 함수
    # ========================================================
    positive_angle = get_safe_test_angle(
        arm,
        joint,
        +1,
        preferred=8.0,
    )

    positive_observation = {}

    def run_positive_motion():
        if not arm.move_to_zero(
            joint,
            TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        ):
            return False

        raw_zero = diagnostic_read_raw_position(
            arm,
            joint,
        )

        result = arm.move_joint(
            joint,
            positive_angle,
            TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        )

        raw_after = diagnostic_read_raw_position(
            arm,
            joint,
        )
        actual_angle = arm.get_joint_angle(
            joint
        )

        raw_delta = (
            None
            if raw_zero is None or raw_after is None
            else raw_after - raw_zero
        )

        positive_observation.clear()
        positive_observation.update(
            {
                "result": result,
                "raw_zero": raw_zero,
                "raw_after": raw_after,
                "raw_delta": raw_delta,
                "actual_angle": actual_angle,
            }
        )

        return bool(result)

    if not run_positive_motion():
        add_result(
            f"{joint} move_joint(TEAM +)",
            "FAIL",
            "TEAM + 이동 명령 실패",
        )
        return False

    physical_ok = ask_visual_confirm(
        f"{joint} TEAM +: {PHYSICAL_POSITIVE_TEXT[joint]}",
        replay_callback=run_positive_motion,
    )

    expected_sign = EXPECTED_RAW_SIGN_POSITIVE[
        joint
    ]

    raw_delta = positive_observation[
        "raw_delta"
    ]

    raw_sign_ok = (
        raw_delta is not None
        and raw_delta != 0
        and (
            1 if raw_delta > 0 else -1
        ) == expected_sign
    )

    angle_ok = angle_close(
        positive_observation[
            "actual_angle"
        ],
        positive_angle,
    )

    positive_ok = (
        bool(
            positive_observation[
                "result"
            ]
        )
        and raw_sign_ok
        and angle_ok
        and physical_ok
    )

    add_result(
        f"{joint} move_joint(TEAM +)",
        "PASS" if positive_ok else "FAIL",
        (
            f"target={positive_angle:+.2f}°, "
            f"actual={positive_observation['actual_angle']}, "
            f"RAW {positive_observation['raw_zero']}"
            f"->{positive_observation['raw_after']} "
            f"delta={positive_observation['raw_delta']}"
        ),
    )

    # 다음 방향 전에 Zero 복귀
    arm.move_to_zero(
        joint,
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    if not positive_ok:
        return False

    # ========================================================
    # TEAM - 다시보기까지 포함한 실행 함수
    # ========================================================
    negative_angle = get_safe_test_angle(
        arm,
        joint,
        -1,
        preferred=8.0,
    )

    negative_observation = {}

    def run_negative_motion():
        if not arm.move_to_zero(
            joint,
            TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        ):
            return False

        raw_zero = diagnostic_read_raw_position(
            arm,
            joint,
        )

        result = arm.move_joint(
            joint,
            negative_angle,
            TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        )

        raw_after = diagnostic_read_raw_position(
            arm,
            joint,
        )
        actual_angle = arm.get_joint_angle(
            joint
        )

        raw_delta = (
            None
            if raw_zero is None or raw_after is None
            else raw_after - raw_zero
        )

        negative_observation.clear()
        negative_observation.update(
            {
                "result": result,
                "raw_zero": raw_zero,
                "raw_after": raw_after,
                "raw_delta": raw_delta,
                "actual_angle": actual_angle,
            }
        )

        return bool(result)

    if not run_negative_motion():
        add_result(
            f"{joint} move_joint(TEAM -)",
            "FAIL",
            "TEAM - 이동 명령 실패",
        )
        return False

    physical_ok = ask_visual_confirm(
        f"{joint} TEAM -: {PHYSICAL_NEGATIVE_TEXT[joint]}",
        replay_callback=run_negative_motion,
    )

    raw_delta = negative_observation[
        "raw_delta"
    ]

    expected_negative_sign = (
        -EXPECTED_RAW_SIGN_POSITIVE[
            joint
        ]
    )

    raw_sign_ok = (
        raw_delta is not None
        and raw_delta != 0
        and (
            1 if raw_delta > 0 else -1
        ) == expected_negative_sign
    )

    angle_ok = angle_close(
        negative_observation[
            "actual_angle"
        ],
        negative_angle,
    )

    negative_ok = (
        bool(
            negative_observation[
                "result"
            ]
        )
        and raw_sign_ok
        and angle_ok
        and physical_ok
    )

    add_result(
        f"{joint} move_joint(TEAM -)",
        "PASS" if negative_ok else "FAIL",
        (
            f"target={negative_angle:+.2f}°, "
            f"actual={negative_observation['actual_angle']}, "
            f"RAW {negative_observation['raw_zero']}"
            f"->{negative_observation['raw_after']} "
            f"delta={negative_observation['raw_delta']}"
        ),
    )

    arm.move_to_zero(
        joint,
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    return negative_ok


def test_absolute_direction_all_joints(arm):
    section("TEST. move_joint() + TEAM +/- 실제 방향 4축 확인")

    for joint in JOINTS:
        if not test_absolute_direction_one_joint(arm, joint):
            return False

    return True


# ============================================================
# 8-B. 단일 Joint Zero 테스트
# ============================================================

def test_single_joint_zero_interactive(arm, joint):
    section(f"TEST. move_to_zero() - {joint}")

    if joint not in JOINTS:
        add_result(
            f"{joint} move_to_zero()",
            "FAIL",
            "알 수 없는 Joint",
        )
        return False

    before = arm.get_joint_angle(joint)

    result = arm.move_to_zero(
        joint,
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    after = arm.get_joint_angle(joint)

    ok = bool(result) and angle_close(after, 0.0)

    add_result(
        f"{joint} move_to_zero()",
        "PASS" if ok else "FAIL",
        f"before={before}, after={after}",
    )

    return ok


# ============================================================
# 9. 상대이동
# ============================================================


def test_relative_move(arm):
    section("TEST 4. move_joint_relative() - 현재 실제 위치 기준")

    joint = "wrist_flex"

    if not arm.move_to_zero(joint, TEST_SPEED, wait=True):
        add_result("Relative 준비 Zero", "FAIL")
        return False

    before = arm.get_joint_angle(joint)
    delta = 6.0

    result1 = arm.move_joint_relative(
        joint,
        delta,
        TEST_SPEED,
    )

    after1 = arm.get_joint_angle(joint)
    expected1 = float(before) + delta

    ok1 = bool(result1) and angle_close(after1, expected1)

    add_result(
        "move_joint_relative(+6°)",
        "PASS" if ok1 else "FAIL",
        f"before={before:+.2f}°, expected={expected1:+.2f}°, actual={after1}",
    )

    if not ok1:
        arm.move_to_zero(joint, TEST_SPEED, wait=True)
        return False

    before2 = arm.get_joint_angle(joint)

    result2 = arm.move_joint_relative(
        joint,
        -delta,
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    after2 = arm.get_joint_angle(joint)
    expected2 = float(before2) - delta

    ok2 = bool(result2) and angle_close(after2, expected2)

    add_result(
        "move_joint_relative(-6°)",
        "PASS" if ok2 else "FAIL",
        f"before={before2:+.2f}°, expected={expected2:+.2f}°, actual={after2}",
    )

    zero_ok = arm.move_to_zero(joint, TEST_SPEED, wait=True)

    return ok1 and ok2 and bool(zero_ok)


# ============================================================
# 10. 다축 SyncWrite 공개 API
# ============================================================


def test_multi_joint(arm):
    section("TEST 5. move_joints() - 4축 실제 팀 사용 형태")

    targets = {
        joint: get_safe_test_angle(
            arm,
            joint,
            +1,
            preferred=7.0,
        )
        for joint in JOINTS
    }

    observation = {}

    def run_multi_motion():
        # 다시보기일 때도 항상 같은 출발점에서 봐야 비교 가능하다.
        if not arm.move_all_to_zero(
            TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        ):
            return False

        # 사람이 시작 순간을 보기 쉽도록 잠깐 여유를 둔다.
        time.sleep(0.5)

        print()
        print("[MOVE_JOINTS] 4축 명령을 지금 전송합니다.")

        result = arm.move_joints(
            targets,
            speed=TEST_SPEED,
            acc=TEST_ACC,
            wait=True,
        )

        angles = {
            joint: arm.get_joint_angle(
                joint
            )
            for joint in JOINTS
        }

        observation.clear()
        observation.update(
            {
                "result": result,
                "angles": angles,
            }
        )

        return bool(result)

    if not run_multi_motion():
        add_result(
            "move_joints() 4축",
            "FAIL",
            "4축 move_joints 명령 또는 도착 확인 실패",
        )
        return False

    physical_ok = ask_visual_confirm(
        (
            "4축이 따로따로 순차 출발한 것이 아니라 "
            "거의 동시에 움직이기 시작했습니까?"
        ),
        replay_callback=run_multi_motion,
    )

    angles = observation[
        "angles"
    ]

    auto_ok = (
        bool(
            observation[
                "result"
            ]
        )
        and all(
            angle_close(
                angles[joint],
                targets[joint],
            )
            for joint in JOINTS
        )
    )

    ok = auto_ok and physical_ok

    add_result(
        "move_joints() 4축",
        "PASS" if ok else "FAIL",
        f"targets={targets}, actual={angles}",
    )

    zero_ok = arm.move_all_to_zero(
        TEST_SPEED,
        acc=TEST_ACC,
        wait=True,
    )

    return ok and bool(zero_ok)


# ============================================================
# 11. 잘못된 명령 차단
# ============================================================


def test_validation_blocks(arm):
    section("TEST 6. Invalid Joint / Speed / Acc / Safe Range / Multi Prevalidation")

    if not arm.move_all_to_zero(TEST_SPEED, wait=True):
        return False

    all_ok = True

    # --------------------------------------------------------
    # Unknown joint
    # --------------------------------------------------------
    before = diagnostic_goal_map(arm)

    result = arm.move_joint(
        "__invalid_joint__",
        0,
        TEST_SPEED,
        wait=False,
    )

    after = diagnostic_goal_map(arm)

    ok = (result is False) and check_no_goal_change(before, after)
    all_ok &= ok

    add_result(
        "알 수 없는 Joint 차단",
        "PASS" if ok else "FAIL",
        f"Goal unchanged={check_no_goal_change(before, after)}",
    )

    # --------------------------------------------------------
    # Speed > max_speed
    # --------------------------------------------------------
    joint = "wrist_flex"
    servo = arm.calibration.get_joint(joint)
    invalid_speed = int(servo["max_speed"]) + 1

    before = diagnostic_goal_map(arm)

    result = arm.move_joint(
        joint,
        0,
        invalid_speed,
        wait=False,
    )

    after = diagnostic_goal_map(arm)

    ok = (result is False) and check_no_goal_change(before, after)
    all_ok &= ok

    add_result(
        "Max Speed 초과 차단",
        "PASS" if ok else "FAIL",
        f"requested={invalid_speed}, max={servo['max_speed']}",
    )

    # --------------------------------------------------------
    # Acc > package max(254)
    # --------------------------------------------------------
    before = diagnostic_goal_map(arm)

    result = arm.move_joint(
        joint,
        0,
        TEST_SPEED,
        acc=255,
        wait=False,
    )

    after = diagnostic_goal_map(arm)

    ok = (result is False) and check_no_goal_change(before, after)
    all_ok &= ok

    add_result(
        "Acc 범위 초과 차단",
        "PASS" if ok else "FAIL",
        "acc=255 / Goal write 없음 확인",
    )

    # --------------------------------------------------------
    # Safe Range 초과
    # --------------------------------------------------------
    safe_min, safe_max = arm.calibration.get_safe_angle_range(joint)
    invalid_angle = float(safe_max) + 5.0

    before = diagnostic_goal_map(arm)

    result = arm.move_joint(
        joint,
        invalid_angle,
        TEST_SPEED,
        wait=False,
    )

    after = diagnostic_goal_map(arm)

    ok = (result is False) and check_no_goal_change(before, after)
    all_ok &= ok

    add_result(
        "Safe Range 초과 차단",
        "PASS" if ok else "FAIL",
        f"requested={invalid_angle:+.2f}°, safe_max={safe_max:+.2f}°",
    )

    # --------------------------------------------------------
    # Multi prevalidation:
    # 하나가 invalid이면 다른 정상 Joint까지 전송되면 안 된다.
    # --------------------------------------------------------
    shoulder_angle = get_safe_test_angle(
        arm,
        "shoulder_lift",
        +1,
        preferred=5.0,
    )

    before = diagnostic_goal_map(arm)

    result = arm.move_joints(
        {
            "shoulder_lift": shoulder_angle,
            "wrist_flex": invalid_angle,
        },
        speed=TEST_SPEED,
        wait=False,
    )

    after = diagnostic_goal_map(arm)

    ok = (result is False) and check_no_goal_change(before, after)
    all_ok &= ok

    add_result(
        "move_joints() 전체 사전검증",
        "PASS" if ok else "FAIL",
        "한 축 invalid일 때 정상 축 포함 전체 Goal write 없음",
    )

    return all_ok


# ============================================================
# 12. 실제 motor_service 형태의 wait=False 10Hz 연속 명령
# ============================================================


def test_service_style_10hz(arm):
    section("TEST 7. 실제 MotorService 형태 - wait=False 약 10Hz 연속 Goal 갱신")

    joint = "wrist_flex"

    if not arm.move_to_zero(joint, TEST_SPEED, wait=True):
        return False

    targets = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0]
    command_results = []
    goal_checks = []

    print("0.1초 간격으로 wrist_flex 목표를 연속 갱신합니다.")

    for target in targets:
        result = arm.move_joint(
            joint,
            target,
            FAST_SPEED,
            acc=TEST_ACC,
            wait=False,
        )

        command_results.append(bool(result))

        # 목표 레지스터가 최신 명령으로 즉시 바뀌었는지 읽기 전용 진단.
        expected_goal = arm.calibration.command_angle_to_position(
            joint,
            target,
        )
        actual_goal = diagnostic_read_goal(arm, joint)

        goal_checks.append(actual_goal == expected_goal)

        print(
            f"target={target:+5.1f}° | "
            f"expected Goal={expected_goal} | actual Goal={actual_goal}"
        )

        time.sleep(0.10)

    final_reached = wait_until_angle_near(
        arm,
        joint,
        targets[-1],
        timeout=5.0,
    )

    final_angle = arm.get_joint_angle(joint)

    ok = (
        all(command_results)
        and all(goal_checks)
        and final_reached
    )

    add_result(
        "10Hz wait=False 반복 재호출",
        "PASS" if ok else "FAIL",
        f"final={final_angle}, target={targets[-1]:+.2f}°",
    )

    arm.move_to_zero(joint, TEST_SPEED, wait=True)

    return ok


# ============================================================
# 13. 이동 중 Goal 두 번 재호출
# ============================================================


def test_recall_twice_while_moving(arm):
    section("TEST 8. 이동 중 move_joint() 두 번 재호출")

    joint = "wrist_flex"

    if not arm.move_to_zero(joint, TEST_SPEED, wait=True):
        return False

    target1 = 25.0
    target2 = 15.0
    target3 = 10.0

    # Safe Range 검사
    for target in (target1, target2, target3):
        arm.calibration.command_angle_to_position(joint, target)

    ok1 = arm.move_joint(
        joint,
        target1,
        SLOW_SPEED,
        wait=False,
    )

    reached_trigger1 = wait_until_angle_at_least(
        arm,
        joint,
        threshold=3.0,
        timeout=3.0,
    )

    ok2 = arm.move_joint(
        joint,
        target2,
        SLOW_SPEED,
        wait=False,
    )

    expected_goal2 = arm.calibration.command_angle_to_position(joint, target2)
    actual_goal2 = diagnostic_read_goal(arm, joint)

    reached_trigger2 = wait_until_angle_at_least(
        arm,
        joint,
        threshold=6.0,
        timeout=3.0,
    )

    ok3 = arm.move_joint(
        joint,
        target3,
        SLOW_SPEED,
        wait=False,
    )

    expected_goal3 = arm.calibration.command_angle_to_position(joint, target3)
    actual_goal3 = diagnostic_read_goal(arm, joint)

    final_reached = wait_until_angle_near(
        arm,
        joint,
        target3,
        timeout=5.0,
    )

    final_angle = arm.get_joint_angle(joint)

    ok = (
        bool(ok1)
        and reached_trigger1
        and bool(ok2)
        and actual_goal2 == expected_goal2
        and reached_trigger2
        and bool(ok3)
        and actual_goal3 == expected_goal3
        and final_reached
    )

    add_result(
        "이동 중 +25° -> +15° -> +10° 재호출",
        "PASS" if ok else "FAIL",
        (
            f"Goal2={actual_goal2}/{expected_goal2}, "
            f"Goal3={actual_goal3}/{expected_goal3}, "
            f"final={final_angle}"
        ),
    )

    arm.move_to_zero(joint, TEST_SPEED, wait=True)

    return ok


# ============================================================
# 14. wait=True + 다른 Thread의 최신 명령으로 대체
# ============================================================


def test_wait_true_superseded(arm):
    section("TEST 9. wait=True 중 다른 Thread 재호출")

    joint = "wrist_flex"

    if not arm.move_to_zero(joint, TEST_SPEED, wait=True):
        return False

    thread_result = {
        "return": None,
        "start": None,
        "end": None,
    }

    def old_command():
        thread_result["start"] = time.monotonic()
        thread_result["return"] = arm.move_joint(
            joint,
            25.0,
            SLOW_SPEED,
            acc=TEST_ACC,
            wait=True,
            timeout=5.0,
        )
        thread_result["end"] = time.monotonic()

    worker = threading.Thread(
        target=old_command,
        name="OldWaitCommand",
    )

    worker.start()

    trigger_ok = wait_until_angle_at_least(
        arm,
        joint,
        threshold=3.0,
        timeout=3.0,
    )

    recall_time = time.monotonic()

    new_result = arm.move_joint(
        joint,
        10.0,
        SLOW_SPEED,
        acc=TEST_ACC,
        wait=False,
    )

    worker.join(timeout=2.0)

    thread_finished = not worker.is_alive()

    if thread_result["end"] is not None:
        cancel_delay = thread_result["end"] - recall_time
    else:
        cancel_delay = None

    final_reached = wait_until_angle_near(
        arm,
        joint,
        10.0,
        timeout=5.0,
    )

    # 기존 wait=True 명령은 새 generation을 감지해 False로 끝나야 한다.
    ok = (
        trigger_ok
        and bool(new_result)
        and thread_finished
        and thread_result["return"] is False
        and cancel_delay is not None
        and cancel_delay < 1.0
        and final_reached
    )

    add_result(
        "wait=True 이전 명령 superseded 처리",
        "PASS" if ok else "FAIL",
        (
            f"old_return={thread_result['return']}, "
            f"cancel_delay={cancel_delay}, "
            f"thread_finished={thread_finished}"
        ),
    )

    arm.move_to_zero(joint, TEST_SPEED, wait=True)

    return ok


# ============================================================
# 15. 4축 User Stop + 차단 + Resume
# ============================================================


def test_user_stop(arm):
    section("TEST 10. 4축 user_stop() + 모든 이동 차단 + resume_user_stop()")

    if not arm.move_all_to_zero(TEST_SPEED, wait=True):
        return False

    # 각 축이 충분히 이동 중일 수 있도록 작은 안전 범위 안에서
    # 일반 방향보다 조금 큰 +목표를 사용하되 Safe Range의 40%를 넘지 않는다.
    moving_targets = {
        joint: get_safe_test_angle(
            arm,
            joint,
            +1,
            preferred=15.0,
        )
        for joint in JOINTS
    }

    target_goal_raw = {
        joint: arm.calibration.command_angle_to_position(
            joint,
            angle,
        )
        for joint, angle in moving_targets.items()
    }

    print()
    print("4축 이동을 시작합니다.")
    print("모터가 움직이는 것을 확인한 뒤 가능한 빨리 Enter를 누르세요.")
    print("Enter 입력 직후 user_stop()이 호출됩니다.")
    print()

    started = arm.move_joints(
        moving_targets,
        speed=USER_STOP_MOTION_SPEED,
        acc=TEST_ACC,
        wait=False,
    )

    if not started:
        add_result("user_stop 준비 4축 이동", "FAIL")
        return False

    input(">>> 4축이 움직이는 중에 Enter를 누르세요: ")

    stop_call_start = time.monotonic()
    stop_result = arm.user_stop()
    stop_call_end = time.monotonic()

    hold_goals = diagnostic_goal_map(arm)
    torque_after_stop = diagnostic_torque_map(arm)
    position_after_stop = diagnostic_position_map(arm)

    # Hold Goal이 원래 이동 목표와 다른지 확인한다.
    # 다르면 최종 목적지에 도달하기 전에 현재 위치 Goal로 덮어쓴 것이다.
    interrupted_before_target = {
        joint: (
            hold_goals[joint] is not None
            and target_goal_raw[joint] is not None
            and abs(hold_goals[joint] - target_goal_raw[joint]) > 5
        )
        for joint in JOINTS
    }

    # Hold Goal과 stop 직후 실제 위치가 충분히 가까운지 확인.
    hold_near_position = {
        joint: (
            hold_goals[joint] is not None
            and position_after_stop[joint] is not None
            and abs(hold_goals[joint] - position_after_stop[joint])
            <= USER_STOP_HOLD_TOLERANCE_RAW
        )
        for joint in JOINTS
    }

    torque_ok = all(
        value == 1
        for value in torque_after_stop.values()
    )

    latch_ok = arm.is_user_stopped() is True

    # Hold 후 약 1초 관찰.
    max_drift = {joint: 0 for joint in JOINTS}
    observe_start = time.monotonic()

    while time.monotonic() - observe_start < 1.0:
        positions = diagnostic_position_map(arm)

        for joint in JOINTS:
            if (
                positions[joint] is not None
                and hold_goals[joint] is not None
            ):
                drift = abs(
                    positions[joint] - hold_goals[joint]
                )
                max_drift[joint] = max(
                    max_drift[joint],
                    drift,
                )

        time.sleep(0.05)

    moving_after = {
        joint: arm.is_moving(joint)
        for joint in JOINTS
    }

    physical_hold_ok = ask_yes_no(
        "user_stop() 후 Torque가 풀리지 않고 로봇팔이 현재 자세를 힘 있게 유지합니까?"
    )

    stop_core_ok = (
        bool(stop_result)
        and latch_ok
        and torque_ok
        and all(hold_near_position.values())
        and physical_hold_ok
    )

    # "실제로 4축 모두 이동 도중 정지했는가"는 별도로 매우 엄격하게 체크.
    all_four_interrupted = all(
        interrupted_before_target.values()
    )

    add_result(
        "user_stop() Torque ON 4축 Hold",
        "PASS" if stop_core_ok else "FAIL",
        (
            f"call_time={stop_call_end - stop_call_start:.4f}s, "
            f"Torque={torque_after_stop}, "
            f"HoldNearPosition={hold_near_position}, "
            f"max_drift_raw={max_drift}"
        ),
    )

    add_result(
        "user_stop() 4축 실제 이동 도중 개입",
        "PASS" if all_four_interrupted else "FAIL",
        (
            f"original_target_goal={target_goal_raw}, "
            f"hold_goal={hold_goals}, "
            f"interrupted={interrupted_before_target}"
        ),
    )

    if not stop_core_ok:
        return False

    # --------------------------------------------------------
    # User Stop 상태에서 공개 이동 API 전부 차단
    # --------------------------------------------------------

    block_tests = []

    safe_plus = get_safe_test_angle(
        arm,
        "wrist_flex",
        +1,
        preferred=5.0,
    )

    # 1) move_joint
    before = diagnostic_goal_map(arm)
    result = arm.move_joint(
        "wrist_flex",
        safe_plus,
        TEST_SPEED,
        wait=False,
    )
    after = diagnostic_goal_map(arm)
    block_tests.append(
        (
            "User Stop 중 move_joint()",
            result is False and check_no_goal_change(before, after),
        )
    )

    # 2) move_joint_relative
    before = diagnostic_goal_map(arm)
    result = arm.move_joint_relative(
        "wrist_flex",
        3.0,
        TEST_SPEED,
        wait=False,
    )
    after = diagnostic_goal_map(arm)
    block_tests.append(
        (
            "User Stop 중 move_joint_relative()",
            result is False and check_no_goal_change(before, after),
        )
    )

    # 3) move_joints
    before = diagnostic_goal_map(arm)
    result = arm.move_joints(
        {
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
        },
        TEST_SPEED,
        wait=False,
    )
    after = diagnostic_goal_map(arm)
    block_tests.append(
        (
            "User Stop 중 move_joints()",
            result is False and check_no_goal_change(before, after),
        )
    )

    # 4) move_to_zero
    before = diagnostic_goal_map(arm)
    result = arm.move_to_zero(
        "wrist_flex",
        TEST_SPEED,
        wait=False,
    )
    after = diagnostic_goal_map(arm)
    block_tests.append(
        (
            "User Stop 중 move_to_zero()",
            result is False and check_no_goal_change(before, after),
        )
    )

    # 5) move_all_to_zero
    before = diagnostic_goal_map(arm)
    result = arm.move_all_to_zero(
        TEST_SPEED,
        wait=False,
    )
    after = diagnostic_goal_map(arm)
    block_tests.append(
        (
            "User Stop 중 move_all_to_zero()",
            result is False and check_no_goal_change(before, after),
        )
    )

    blocks_ok = True

    for name, ok in block_tests:
        blocks_ok &= ok
        add_result(
            name,
            "PASS" if ok else "FAIL",
            "실제 Goal register 변화 없음 확인",
        )

    # --------------------------------------------------------
    # Stop 상태에서도 상태 읽기 허용
    # --------------------------------------------------------

    read_angle = arm.get_joint_angle("wrist_flex")
    read_state = arm.get_joint_state("wrist_flex")
    read_all = arm.get_all_states()
    read_moving = arm.is_moving("wrist_flex")

    reads_ok = (
        read_angle is not None
        and read_state is not None
        and isinstance(read_all, dict)
        and set(read_all.keys()) == set(JOINTS)
        and read_moving in (True, False, None)
    )

    add_result(
        "User Stop 중 상태 읽기",
        "PASS" if reads_ok else "FAIL",
        "get_joint_angle/state/all_states/is_moving",
    )

    # --------------------------------------------------------
    # Resume는 latch만 해제하고 아무 동작도 하지 않아야 한다.
    # --------------------------------------------------------

    goal_before_resume = diagnostic_goal_map(arm)
    position_before_resume = diagnostic_position_map(arm)
    torque_before_resume = diagnostic_torque_map(arm)

    resume_result = arm.resume_user_stop()

    time.sleep(0.5)

    goal_after_resume = diagnostic_goal_map(arm)
    position_after_resume = diagnostic_position_map(arm)
    torque_after_resume = diagnostic_torque_map(arm)

    resume_no_goal_write = check_no_goal_change(
        goal_before_resume,
        goal_after_resume,
    )

    resume_torque_same = (
        torque_before_resume == torque_after_resume
        and all(
            value == 1
            for value in torque_after_resume.values()
        )
    )

    resume_latch_off = arm.is_user_stopped() is False

    resume_drift = {}

    for joint in JOINTS:
        before_pos = position_before_resume[joint]
        after_pos = position_after_resume[joint]

        if before_pos is None or after_pos is None:
            resume_drift[joint] = None
        else:
            resume_drift[joint] = abs(after_pos - before_pos)

    physical_resume_ok = ask_yes_no(
        "resume_user_stop() 호출 자체 때문에 로봇팔이 새로운 위치로 움직이지 않았습니까?"
    )

    resume_ok = (
        bool(resume_result)
        and resume_no_goal_write
        and resume_torque_same
        and resume_latch_off
        and physical_resume_ok
    )

    add_result(
        "resume_user_stop() 자체 무동작",
        "PASS" if resume_ok else "FAIL",
        (
            f"Goal unchanged={resume_no_goal_write}, "
            f"Torque={torque_after_resume}, "
            f"position_drift_raw={resume_drift}"
        ),
    )

    # Resume 이후 새 명령에서만 다시 이동하는지 확인.
    post_target = get_safe_test_angle(
        arm,
        "wrist_flex",
        +1,
        preferred=5.0,
    )

    post_result = arm.move_joint(
        "wrist_flex",
        post_target,
        TEST_SPEED,
        wait=True,
    )

    post_angle = arm.get_joint_angle("wrist_flex")

    post_ok = (
        bool(post_result)
        and angle_close(post_angle, post_target)
    )

    add_result(
        "Resume 후 새 이동 명령",
        "PASS" if post_ok else "FAIL",
        f"target={post_target:+.2f}°, actual={post_angle}",
    )

    zero_ok = arm.move_all_to_zero(
        TEST_SPEED,
        wait=True,
    )

    return (
        stop_core_ok
        and all_four_interrupted
        and blocks_ok
        and reads_ok
        and resume_ok
        and post_ok
        and bool(zero_ok)
    )


# ============================================================
# 16. 기존 emergency_stop() - 반드시 마지막
# ============================================================


def test_emergency_stop_last(arm):
    section("TEST 11. 기존 emergency_stop() - 마지막 Torque OFF 테스트")

    print()
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("이 테스트는 모든 Servo의 Torque를 OFF합니다.")
    print("로봇팔이 중력으로 떨어질 수 있습니다.")
    print("반드시 팔을 손으로 지지한 상태에서 실행하세요.")
    print("이 테스트 이후 같은 MotorController 세션에서는 정상 이동을 계속하지 않습니다.")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print()

    token = input(
        "실제로 emergency_stop()까지 테스트하려면 EMERGENCY 를 정확히 입력하세요.\n"
        "건너뛰려면 Enter: "
    ).strip()

    if token != "EMERGENCY":
        add_result(
            "emergency_stop()",
            "SKIP",
            "사용자가 Torque OFF 최종 테스트를 건너뜀. 전체 인수 테스트는 미완료 상태.",
        )
        return None

    # 가능한 안전한 시작 자세로 먼저 복귀.
    zero_result = arm.move_all_to_zero(
        TEST_SPEED,
        wait=True,
    )

    if not zero_result:
        add_result(
            "Emergency 준비 Zero",
            "FAIL",
        )
        return False

    require_user_continue(
        "이제 팔을 손으로 확실하게 지지하세요.\n"
        "다음 단계에서 Torque가 OFF됩니다."
    )

    goal_before = diagnostic_goal_map(arm)

    result = arm.emergency_stop()

    torque_after = diagnostic_torque_map(arm)
    latch_on = arm.is_emergency_stopped() is True

    torque_off_ok = all(
        value == 0
        for value in torque_after.values()
    )

    emergency_ok = bool(result) and latch_on and torque_off_ok

    add_result(
        "emergency_stop() 4축 Torque OFF",
        "PASS" if emergency_ok else "FAIL",
        f"Torque={torque_after}, latch={latch_on}",
    )

    if not emergency_ok:
        return False

    # Emergency 상태에서 이동 명령이 실제 Goal을 바꾸지 않아야 한다.
    result_move = arm.move_joint(
        "wrist_flex",
        5.0,
        TEST_SPEED,
        wait=False,
    )

    goal_after = diagnostic_goal_map(arm)

    block_ok = (
        result_move is False
        and check_no_goal_change(
            goal_before,
            goal_after,
        )
    )

    add_result(
        "Emergency 상태 이동 차단",
        "PASS" if block_ok else "FAIL",
        "move_joint() False + Goal register 변화 없음",
    )

    # 상태 읽기는 Emergency 상태에서도 허용되어야 한다.
    state = arm.get_joint_state("wrist_flex")
    read_ok = state is not None

    add_result(
        "Emergency 상태 상태 읽기",
        "PASS" if read_ok else "FAIL",
    )

    # Torque OFF 상태에서 user_stop()은 Hold를 시도하면 안 된다.
    user_stop_result = arm.user_stop()

    user_stop_blocked = (
        user_stop_result is False
        and arm.is_emergency_stopped() is True
    )

    add_result(
        "Emergency 상태 user_stop() 차단",
        "PASS" if user_stop_blocked else "FAIL",
    )

    return (
        emergency_ok
        and block_ok
        and read_ok
        and user_stop_blocked
    )


# ============================================================
# 17. 결과 저장
# ============================================================


def save_report():
    TEST_FILE.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    timestamp = TEST_STARTED_AT.strftime(
        "%Y%m%d_%H%M%S"
    )

    report_path = (
        TEST_FILE.parent
        / f"motor_control_final_acceptance_{timestamp}.json"
    )

    pass_count = sum(
        1
        for item in RESULTS
        if item["status"] == "PASS"
    )

    fail_count = sum(
        1
        for item in RESULTS
        if item["status"] == "FAIL"
    )

    skip_count = sum(
        1
        for item in RESULTS
        if item["status"] == "SKIP"
    )

    report = {
        "started_at": TEST_STARTED_AT.isoformat(),
        "finished_at": datetime.now().isoformat(),
        "project_root": str(PROJECT_ROOT),
        "test_file": str(TEST_FILE),
        "summary": {
            "pass": pass_count,
            "fail": fail_count,
            "skip": skip_count,
            "complete": (
                fail_count == 0
                and skip_count == 0
            ),
        },
        "results": RESULTS,
    }

    with report_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            report,
            file,
            ensure_ascii=False,
            indent=2,
        )

    return report_path, report["summary"]


# ============================================================
# 18. Menu 기반 Main
# ============================================================

MENU_TEXT = """
==============================================================================
POCO motor_control FINAL ACCEPTANCE TEST - MENU MODE
==============================================================================

[기본 / 상태]
  0  : Preflight + Calibration + 상태 읽기
  s  : 현재 4축 상태표 출력

[Zero]
  z  : 4축 전체 Zero Position으로 이동
  zj : 선택한 1축만 Zero Position으로 이동

[이동 API]
  1  : move_joint() TEAM +/- 방향 - 선택한 1축
  2  : move_joint() TEAM +/- 방향 - 4축 전체
  3  : move_joint_relative()
  4  : move_joints() 4축 SyncWrite

[안전 / 실제 팀 사용 패턴]
  5  : Invalid Joint / Speed / Acc / Safe Range 차단
  6  : 실제 motor_service 형태 10Hz wait=False 반복 명령
  7  : 이동 중 move_joint() 두 번 재호출
  8  : wait=True 중 다른 Thread의 새 명령으로 대체

[비상정지]
  9  : user_stop() + 이동 차단 + resume_user_stop()
  e  : 기존 emergency_stop() 최종 Torque OFF 테스트
       ※ 실행 후에는 이 세션에서 더 이상 이동 테스트를 하지 않는다.

[기타]
  c  : Context Manager / 자동 close 별도 테스트
  a  : 일반 전체 테스트 순차 실행 (emergency_stop 제외)
  r  : 현재 세션 PASS/FAIL 요약
  q  : 종료

※ 실패해도 프로그램 전체를 종료하지 않는다.
※ 필요한 항목만 다시 실행할 수 있다.
※ 시각 확인 질문에서 r을 입력하면 같은 동작을 다시 볼 수 있다.
※ z 명령으로 언제든 4축 Zero 복귀를 시도할 수 있다.
"""


def choose_joint():
    print()
    print("1 : shoulder_lift")
    print("2 : elbow_flex")
    print("3 : wrist_flex")
    print("4 : wrist_roll")

    mapping = {
        "1": "shoulder_lift",
        "2": "elbow_flex",
        "3": "wrist_flex",
        "4": "wrist_roll",
    }

    while True:
        value = input("Joint 선택 [1-4 / q 취소]: ").strip().lower()

        if value == "q":
            return None

        if value in mapping:
            return mapping[value]

        print("1~4 중 하나를 입력하세요.")


def print_session_summary():
    section("CURRENT SESSION SUMMARY")

    if not RESULTS:
        print("아직 실행한 테스트가 없습니다.")
        return

    for item in RESULTS:
        print(
            f"{item['status']:<5} | "
            f"{item['name']}"
        )

    pass_count = sum(
        item["status"] == "PASS"
        for item in RESULTS
    )
    fail_count = sum(
        item["status"] == "FAIL"
        for item in RESULTS
    )
    skip_count = sum(
        item["status"] == "SKIP"
        for item in RESULTS
    )

    print()
    print(
        f"PASS={pass_count} "
        f"FAIL={fail_count} "
        f"SKIP={skip_count}"
    )


def save_report_snapshot():
    report_path, summary = save_report()
    print(
        f"[REPORT] 현재 결과 저장: {report_path} "
        f"(PASS={summary['pass']} FAIL={summary['fail']} SKIP={summary['skip']})"
    )


def run_test_safely(name, func):
    """
    메뉴 테스트 한 항목에서 예외가 나도 전체 메뉴는 종료하지 않는다.
    Ctrl+C는 현재 항목만 중단하고 메뉴로 돌아간다.
    """
    print()

    try:
        result = func()

        if result is True:
            print(f"\n[TEST RESULT] {name}: PASS")
        elif result is False:
            print(f"\n[TEST RESULT] {name}: FAIL")
        else:
            print(f"\n[TEST RESULT] {name}: 완료/사용자 SKIP")

        return result

    except KeyboardInterrupt:
        print()
        print(f"[INTERRUPT] {name} 테스트만 중단했습니다.")
        print("자동 emergency_stop()은 호출하지 않습니다.")

        add_result(
            f"{name} 사용자 중단",
            "WARN",
            "Ctrl+C",
        )

        return None

    except Exception as error:
        print()
        print(f"[TEST ERROR] {name}")
        print(f"{type(error).__name__}: {error}")
        traceback.print_exc()

        add_result(
            f"{name} 예외",
            "FAIL",
            f"{type(error).__name__}: {error}",
        )

        return False

    finally:
        # 한 항목이 끝날 때마다 현재 결과를 저장한다.
        # 실패해서 프로그램을 다시 시작하더라도 터미널 기록 외에 JSON이 남는다.
        try:
            save_report_snapshot()
        except Exception as error:
            print(f"[WARNING] 결과 JSON 저장 실패: {error}")


def run_normal_suite(arm):
    """
    emergency_stop()을 제외한 일반 전체 인수 테스트.
    이전 버전처럼 하나 실패했다고 프로세스를 종료하지 않는다.
    다만 방향 테스트 실패처럼 물리적으로 위험할 수 있는 항목은
    해당 함수 자체가 즉시 False를 반환한다.
    """

    suite = [
        ("Preflight", lambda: test_preflight(arm)),
        ("Zero 기능", lambda: test_zero_functions(arm)),
        ("4축 TEAM 방향", lambda: test_absolute_direction_all_joints(arm)),
        ("상대이동", lambda: test_relative_move(arm)),
        ("다축 move_joints", lambda: test_multi_joint(arm)),
        ("안전 차단", lambda: test_validation_blocks(arm)),
        ("10Hz 반복 명령", lambda: test_service_style_10hz(arm)),
        ("이동 중 2회 재호출", lambda: test_recall_twice_while_moving(arm)),
        ("wait=True superseded", lambda: test_wait_true_superseded(arm)),
        ("user_stop", lambda: test_user_stop(arm)),
    ]

    print()
    print("[ALL TEST] emergency_stop()을 제외한 일반 테스트를 순서대로 실행합니다.")
    print("실패 항목이 생겨도 가능한 경우 다음 항목으로 넘어갑니다.")
    print("단, 물리 상태가 불안하면 Ctrl+C로 현재 항목을 중단하고 메뉴에서 z를 실행하세요.")

    for name, func in suite:
        run_test_safely(name, func)

        if arm.is_emergency_stopped():
            print()
            print("[STOP] Emergency latch가 ON이라 일반 전체 테스트를 중단합니다.")
            break

        if arm.is_user_stopped():
            print()
            print(
                "[STOP] User Stop latch가 ON입니다. "
                "안전을 위해 일반 전체 테스트를 중단합니다."
            )
            print(
                "user_stop 테스트 실패 상황이라면 상태를 확인한 후 "
                "프로그램을 재시작하거나 명시적으로 resume 정책을 검토하세요."
            )
            break


def main():
    section("POCO motor_control FINAL ACCEPTANCE TEST - MENU MODE")

    print(
        "이 버전은 실패할 때마다 처음부터 다시 실행하지 않고 "
        "필요한 항목만 선택해서 반복 검증합니다."
    )

    print()
    print("테스트 파일 위치 :", TEST_FILE)
    print("프로젝트 루트     :", PROJECT_ROOT)
    print("WorkSpace         :", WORKSPACE_DIR)

    print()
    print("[최종 방향 기준]")
    print("shoulder_lift TEAM + = 위")
    print("elbow_flex    TEAM + = 위")
    print("wrist_flex    TEAM + = 위")
    print("wrist_roll    TEAM + = CW")
    print("wrist_roll 관찰 기준 = 모니터가 위치한 정면에서 로봇팔을 바라보는 기준")

    require_user_continue(
        "전원 12V, /dev/ttyACM0 연결, 로봇 주변 안전 상태를 확인하세요."
    )

    arm = None
    emergency_executed = False

    try:
        arm = MotorController()

        print(MENU_TEXT)

        while True:
            if arm.is_emergency_stopped():
                print()
                print(
                    "[EMERGENCY LATCH ON] 기존 emergency_stop()이 실행된 상태입니다."
                )
                print(
                    "Torque가 OFF이므로 추가 이동 테스트는 실행하지 마세요."
                )
                print("r=결과보기 / s=상태읽기 / q=종료만 권장합니다.")

            command = input("\n테스트 선택 > ").strip().lower()

            if not command:
                continue

            # ------------------------------------------------
            # 도움말 / 메뉴 재출력
            # ------------------------------------------------
            if command in ("h", "help", "menu"):
                print(MENU_TEXT)
                continue

            # ------------------------------------------------
            # 종료
            # ------------------------------------------------
            if command in ("q", "quit", "exit"):
                break

            # ------------------------------------------------
            # 결과 요약
            # ------------------------------------------------
            if command == "r":
                print_session_summary()
                continue

            # ------------------------------------------------
            # 상태 읽기
            # ------------------------------------------------
            if command == "s":
                run_test_safely(
                    "현재 4축 상태 읽기",
                    lambda: (print_state_table(arm) or True),
                )
                continue

            # Emergency 후에는 상태 읽기/요약/종료 외 이동 메뉴를 차단.
            if arm.is_emergency_stopped():
                print(
                    "[BLOCK] Emergency latch가 ON입니다. "
                    "이동 테스트를 실행할 수 없습니다."
                )
                continue

            # ------------------------------------------------
            # Preflight
            # ------------------------------------------------
            if command == "0":
                run_test_safely(
                    "Preflight",
                    lambda: test_preflight(arm),
                )
                continue

            # ------------------------------------------------
            # 4축 Zero
            # ------------------------------------------------
            if command == "z":
                run_test_safely(
                    "4축 Zero 이동",
                    lambda: (
                        add_result(
                            "메뉴 4축 move_all_to_zero()",
                            "PASS"
                            if arm.move_all_to_zero(
                                TEST_SPEED,
                                acc=TEST_ACC,
                                wait=True,
                            )
                            else "FAIL",
                            "메뉴 수동 Zero 복귀",
                        )
                        or RESULTS[-1]["status"] == "PASS"
                    ),
                )
                continue

            # ------------------------------------------------
            # 단일 Joint Zero
            # ------------------------------------------------
            if command == "zj":
                joint = choose_joint()

                if joint is not None:
                    run_test_safely(
                        f"{joint} Zero",
                        lambda j=joint: test_single_joint_zero_interactive(
                            arm,
                            j,
                        ),
                    )
                continue

            # ------------------------------------------------
            # 단일 Joint TEAM +/-
            # ------------------------------------------------
            if command == "1":
                joint = choose_joint()

                if joint is not None:
                    run_test_safely(
                        f"{joint} TEAM +/-",
                        lambda j=joint: test_absolute_direction_one_joint(
                            arm,
                            j,
                        ),
                    )
                continue

            # ------------------------------------------------
            # 4축 TEAM +/-
            # ------------------------------------------------
            if command == "2":
                run_test_safely(
                    "4축 TEAM +/-",
                    lambda: test_absolute_direction_all_joints(arm),
                )
                continue

            # ------------------------------------------------
            # Relative
            # ------------------------------------------------
            if command == "3":
                run_test_safely(
                    "move_joint_relative",
                    lambda: test_relative_move(arm),
                )
                continue

            # ------------------------------------------------
            # Multi
            # ------------------------------------------------
            if command == "4":
                run_test_safely(
                    "move_joints 4축",
                    lambda: test_multi_joint(arm),
                )
                continue

            # ------------------------------------------------
            # Validation
            # ------------------------------------------------
            if command == "5":
                run_test_safely(
                    "안전 차단",
                    lambda: test_validation_blocks(arm),
                )
                continue

            # ------------------------------------------------
            # 10Hz
            # ------------------------------------------------
            if command == "6":
                run_test_safely(
                    "10Hz wait=False",
                    lambda: test_service_style_10hz(arm),
                )
                continue

            # ------------------------------------------------
            # Recall twice
            # ------------------------------------------------
            if command == "7":
                run_test_safely(
                    "이동 중 2회 재호출",
                    lambda: test_recall_twice_while_moving(arm),
                )
                continue

            # ------------------------------------------------
            # wait=True superseded
            # ------------------------------------------------
            if command == "8":
                run_test_safely(
                    "wait=True superseded",
                    lambda: test_wait_true_superseded(arm),
                )
                continue

            # ------------------------------------------------
            # user_stop
            # ------------------------------------------------
            if command == "9":
                run_test_safely(
                    "user_stop",
                    lambda: test_user_stop(arm),
                )
                continue

            # ------------------------------------------------
            # Context Manager는 현재 arm과 별개의 임시 Controller를
            # 열었다 닫으므로, 실제 통합 앱이 실행 중이지 않은
            # 이 테스트 환경에서만 사용한다.
            # ------------------------------------------------
            if command == "c":
                run_test_safely(
                    "Context Manager",
                    test_context_manager,
                )
                continue

            # ------------------------------------------------
            # 일반 전체 테스트
            # ------------------------------------------------
            if command == "a":
                run_normal_suite(arm)
                continue

            # ------------------------------------------------
            # Emergency - 마지막 기능
            # ------------------------------------------------
            if command == "e":
                emergency_result = run_test_safely(
                    "emergency_stop",
                    lambda: test_emergency_stop_last(arm),
                )

                if emergency_result is True:
                    emergency_executed = True
                    print()
                    print(
                        "[IMPORTANT] emergency_stop()이 정상 실행되어 "
                        "Torque가 OFF되었습니다."
                    )
                    print(
                        "이 세션에서는 추가 모션 테스트를 하지 말고 "
                        "상태 확인 후 q로 종료하세요."
                    )

                continue

            print("알 수 없는 메뉴입니다. h를 입력하면 메뉴를 다시 볼 수 있습니다.")

        return 0

    except KeyboardInterrupt:
        print()
        print("[EXIT] 메뉴에서 Ctrl+C 감지")
        add_result(
            "테스트 프로그램 사용자 종료",
            "WARN",
            "Ctrl+C",
        )
        return 130

    except Exception as error:
        print()
        print("[UNEXPECTED ERROR]")
        print(f"{type(error).__name__}: {error}")
        traceback.print_exc()

        add_result(
            "예상치 못한 예외",
            "FAIL",
            f"{type(error).__name__}: {error}",
        )
        return 1

    finally:
        if arm is not None:
            try:
                arm.close()

                if not arm.driver.is_open:
                    add_result(
                        "close()",
                        "PASS",
                        "Serial Port closed",
                    )
                else:
                    add_result(
                        "close()",
                        "FAIL",
                        "Serial Port가 닫히지 않았습니다.",
                    )

            except Exception as error:
                add_result(
                    "close()",
                    "FAIL",
                    str(error),
                )

        try:
            report_path, summary = save_report()

            section("FINAL SESSION SUMMARY")
            print_session_summary()
            print()
            print(f"결과 JSON: {report_path}")

            if emergency_executed:
                print(
                    "[INFO] 이번 세션에서는 emergency_stop() Torque OFF까지 검증했습니다."
                )

        except Exception as error:
            print(f"[WARNING] 최종 결과 저장 실패: {error}")


if __name__ == "__main__":
    sys.exit(main())

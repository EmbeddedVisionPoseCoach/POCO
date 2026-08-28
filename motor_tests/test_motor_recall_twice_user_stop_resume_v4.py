#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_motor_recall_and_user_stop_candidate.py

[이 코드가 전체 프로젝트에서 하는 역할]
현재 VisionPoseCoach의 motor_control 패키지를 수정하지 않고,
실제 하드웨어에서 아래 두 가지를 독립적으로 검증하기 위한 테스트 파일이다.

TEST 1. 이동 중 같은 함수 2회 재호출
    wrist_flex를 TEAM 0° -> +50°로 이동시키는 도중,
    약 +10°에 도달했을 때 +30°로 1차 재호출하고,
    약 +20°에 도달했을 때 +25°로 2차 재호출한다.

    확인할 것:
    - 첫 번째 move_joint(..., wait=False)가 즉시 반환하는가
    - 이동 중 두 번째 move_joint(..., wait=False)가 정상 호출되는가
    - Goal Position이 +50° -> +30° -> +25°로 두 번 갱신되는가
    - 최종 위치가 마지막 목표인 +25° 부근에서 끝나는가

TEST 2. 사용자 수동 STOP 입력 기반 "Torque 유지 정지" 후보 검증
    wrist_flex가 TEAM 0° -> +50°로 이동하는 도중,
    사용자가 터미널에서 1을 입력하면 그 순간 현재 raw Position을 읽고
    그 값을 새로운 Goal Position으로 다시 쓴다.

    확인할 것:
    - 기존 이동 목표가 현재 위치 Goal로 덮어써지는가
    - Moving이 0으로 바뀌는가
    - 정지 후 Position이 현재 위치 부근에 유지되는가
    - Torque Enable(주소 40)이 계속 1인가
    - 실제 로봇팔이 힘이 빠지지 않고 자세를 유지하는가

중요:
이 파일은 "사용자용 Emergency Stop 기능을 패키지에 추가하는 코드"가 아니다.
현재 위치 Hold 방식이 실제 STS3215에서 사용자용 정지 방식으로 적합한지
하드웨어에서 검증하기 위한 실험 파일이다.

------------------------------------------------------------
[실제 프로젝트 구조]

사용 위치:
    ~/VisionPoseCoach/motor_tests/
        test_motor_recall_and_user_stop_standalone.py

패키지:
    ~/VisionPoseCoach/WorkSpace/hardware/motor_control/

Calibration:
    ~/VisionPoseCoach/WorkSpace/hardware/servo_calibration_result.json

실제 앱과 동일한 import:
    from hardware.motor_control import MotorController

------------------------------------------------------------
[테스트 Joint]

현재 실제 프로젝트에서 motor_service.py가 반복 명령을 보내는
Servo ID 3 / wrist_flex를 사용한다.

TEAM 방향:
    wrist_flex + = 위
    wrist_flex - = 아래

------------------------------------------------------------
[안전]

- Calibration Safe Range 안에서만 테스트한다.
- max_speed보다 낮은 속도를 자동 선택한다.
- 첫 목표 +50°, 재호출 목표 +30°는 실행 전에 Safe Range를 검증한다.
- 실제 이동 전 매번 사용자가 Enter를 눌러야 한다.
- Ctrl+C 또는 예외 발생 시 Torque OFF를 자동 실행하지 않는다.
  대신 현재 Position을 Goal로 다시 쓰는 Best-Effort Hold를 먼저 시도한다.
- TEST 2는 아직 검증되지 않은 "사용자용 정지 후보" 실험이다.
- 물리적인 비상 상황에서는 소프트웨어 테스트에 의존하지 말고
  실제 전원/비상정지 수단을 사용할 수 있도록 준비한다.

------------------------------------------------------------
[패키지 수정 여부]

이 파일은:
- pyQt/services 등 팀원 파일을 import하거나 수정하지 않음
- WorkSpace 내부 Python 파일을 생성/수정하지 않음
- motor_control 코드를 수정하지 않음
- Calibration JSON을 수정하지 않음
- Torque Enable 값을 쓰지 않음
- 기존 emergency_stop()을 호출하지 않음

Torque Enable 주소 40은 TEST 2에서 "읽기"만 한다.
"""

from __future__ import annotations

import json
import sys
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import Any


# ============================================================
# 1. 실제 VisionPoseCoach 경로
# ============================================================

TEST_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TEST_DIR.parent

WORKSPACE_DIR = (
    PROJECT_ROOT
    / "WorkSpace"
)

HARDWARE_DIR = (
    WORKSPACE_DIR
    / "hardware"
)

CALIBRATION_FILE = (
    HARDWARE_DIR
    / "servo_calibration_result.json"
)

if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(WORKSPACE_DIR),
    )


# ============================================================
# 2. 실제 프로젝트 패키지 Import
# ============================================================

try:
    from hardware.motor_control import MotorController
    from hardware.motor_control.servo_driver import COMM_SUCCESS

except Exception as error:
    print()
    print("============================================================")
    print("[FATAL] hardware.motor_control Import 실패")
    print("============================================================")
    print(error)
    print()
    print(
        "이 파일을 ~/VisionPoseCoach/motor_tests/에 두었는지 확인하세요."
    )
    raise SystemExit(1)


# ============================================================
# 3. 테스트 설정
# ============================================================

TEST_JOINT = "wrist_flex"
TEST_SERVO_ID = 3

FIRST_TARGET_DEG = 50.0

# 1차 재호출:
#   +50°로 이동 중 +10° 부근에서 목표를 +30°로 변경
RECALL1_TRIGGER_DEG = 10.0
RECALL1_TARGET_DEG = 30.0

# 2차 재호출:
#   +30°로 계속 이동 중 +20° 부근에서 목표를 +25°로 다시 변경
RECALL2_TRIGGER_DEG = 20.0
RECALL2_TARGET_DEG = 25.0

# 사용자용 정지 후보 테스트도 이 지점에서 정지 명령을 보낸다.
STOP_TRIGGER_DEG = 10.0

DEFAULT_TEST_SPEED = 80
TEST_ACC = 10

COMMAND_TIMEOUT_SEC = 12.0
MONITOR_INTERVAL_SEC = 0.02

# 최종 각도 판정 허용 오차
FINAL_ANGLE_TOLERANCE_DEG = 1.5

# 재호출 이후에도 예전 +50° 목표까지 계속 간다면 실패로 보기 위한 기준.
# 정상이라면 +30° 부근에서 멈추므로 +40° 이상 갈 이유가 없다.
OLD_TARGET_PERSIST_THRESHOLD_DEG = 40.0

# Hold 명령 이후 실제로 더 움직인 각도 측정 시간
STOP_OBSERVE_SEC = 2.0

# STS3215 SRAM
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_MOVING = 66


# ============================================================
# 4. 결과 기록
# ============================================================

RESULTS: list[dict[str, Any]] = []


def record(
    name: str,
    passed: bool,
    detail: Any,
):
    result = {
        "test": name,
        "passed": bool(passed),
        "detail": detail,
        "time": datetime.now().isoformat(),
    }

    RESULTS.append(result)

    print()
    print(
        f"[{'PASS' if passed else 'FAIL'}] "
        f"{name}"
    )

    if detail is not None:
        print(
            json.dumps(
                detail,
                ensure_ascii=False,
                indent=2,
                default=str,
            )
        )


def save_results():
    path = (
        TEST_DIR
        / (
            "motor_recall_stop_test_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".json"
        )
    )

    with path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            {
                "created_at":
                    datetime.now().isoformat(),

                "joint":
                    TEST_JOINT,

                "servo_id":
                    TEST_SERVO_ID,

                "results":
                    RESULTS,
            },
            file,
            ensure_ascii=False,
            indent=4,
        )

    print()
    print(
        f"[RESULT FILE] {path}"
    )


# ============================================================
# 5. 출력 / 사용자 확인
# ============================================================

def section(title: str):
    print()
    print(
        "============================================================"
    )
    print(
        f" {title}"
    )
    print(
        "============================================================"
    )


def wait_enter(message: str):
    print()
    print(message)
    input(
        "준비되면 Enter를 누르세요."
    )


def ask_yes_no(message: str) -> bool:
    while True:
        answer = input(
            f"{message} [y/N]: "
        ).strip().lower()

        if not answer:
            return False

        if answer in (
            "y",
            "yes",
        ):
            return True

        if answer in (
            "n",
            "no",
        ):
            return False

        print(
            "[INFO] y 또는 n으로 입력하세요."
        )


# ============================================================
# 6. 테스트용 Low-Level Read Helper
# ============================================================
#
# 실제 제어는 가능한 한 MotorController 공개 API를 사용한다.
#
# 다만 이번 검증에서는:
# - Goal Position이 실제로 덮어써졌는지
# - Torque Enable이 계속 ON인지
#
# 를 확인해야 하므로 읽기 전용으로 STS 레지스터를 직접 확인한다.


def _read_1byte(
    arm: MotorController,
    servo_id: int,
    address: int,
):
    with arm.driver._io_lock:
        value, result, error = (
            arm.driver.packet_handler.read1ByteTxRx(
                int(servo_id),
                int(address),
            )
        )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        return None

    return int(value)


def _read_2byte(
    arm: MotorController,
    servo_id: int,
    address: int,
):
    with arm.driver._io_lock:
        value, result, error = (
            arm.driver.packet_handler.read2ByteTxRx(
                int(servo_id),
                int(address),
            )
        )

    if (
        result != COMM_SUCCESS
        or error != 0
    ):
        return None

    return int(value)


def read_goal_position(
    arm: MotorController,
):
    return _read_2byte(
        arm,
        TEST_SERVO_ID,
        ADDR_GOAL_POSITION,
    )


def read_torque_enable(
    arm: MotorController,
):
    return _read_1byte(
        arm,
        TEST_SERVO_ID,
        ADDR_TORQUE_ENABLE,
    )


def read_moving(
    arm: MotorController,
):
    return _read_1byte(
        arm,
        TEST_SERVO_ID,
        ADDR_MOVING,
    )


def read_position_and_angle(
    arm: MotorController,
):
    raw = arm.driver.read_position(
        TEST_SERVO_ID
    )

    if raw is None:
        return None, None

    angle = (
        arm.calibration.position_to_command_angle(
            TEST_JOINT,
            raw,
        )
    )

    return int(raw), float(angle)


def print_live_status(
    arm: MotorController,
    stage: str,
    start_time: float,
):
    """
    테스트 중 사용자가 터미널에서 실제 상태를 계속 확인할 수 있도록
    현재 Position / TEAM Angle / Goal Position / Moving / Torque를
    한 줄로 갱신해서 표시한다.
    """

    raw, angle = read_position_and_angle(
        arm
    )

    goal = read_goal_position(
        arm
    )

    moving = read_moving(
        arm
    )

    torque = read_torque_enable(
        arm
    )

    elapsed = (
        time.monotonic()
        - start_time
    )

    if raw is None or angle is None:
        text = (
            f"[{stage}] "
            f"t={elapsed:6.2f}s | "
            f"Position READ FAIL"
        )
    else:
        text = (
            f"[{stage:<10}] "
            f"t={elapsed:6.2f}s | "
            f"Angle={angle:+7.2f}° | "
            f"Raw={raw:4d} | "
            f"Goal={str(goal):>4} | "
            f"Moving={moving} | "
            f"Torque={torque}"
        )

    print(
        "\r" + text + " " * 8,
        end="",
        flush=True,
    )

    return {
        "t": elapsed,
        "raw": raw,
        "angle": angle,
        "goal": goal,
        "moving": moving,
        "torque": torque,
        "stage": stage,
    }


# ============================================================
# 7. 현재 위치 Goal 덮어쓰기
# ============================================================
#
# 사용자용 정지 후보의 핵심 실험.
#
# Torque를 끄지 않고:
#   Present Position 읽기
#       ↓
#   그 raw Position을 즉시 Goal Position으로 WritePosEx
#
# 한다.
#
# 아직 motor_control 공개 함수로 추가하지 않는다.
# 이 테스트의 결과를 보고 추가 여부를 결정한다.


def write_current_position_as_goal(
    arm: MotorController,
    speed: int,
):
    with arm._command_lock:

        current_raw = (
            arm.driver.read_position(
                TEST_SERVO_ID
            )
        )

        if current_raw is None:
            return {
                "success": False,
                "reason":
                    "현재 Position 읽기 실패",
            }

        current_angle = (
            arm.calibration.position_to_command_angle(
                TEST_JOINT,
                current_raw,
            )
        )

        success = (
            arm.driver.write_position(
                servo_id=TEST_SERVO_ID,
                position=current_raw,
                speed=speed,
                acc=TEST_ACC,
            )
        )

    return {
        "success": bool(success),
        "hold_raw": int(current_raw),
        "hold_angle": float(current_angle),
    }


# ============================================================
# 8. 예외 발생 시 Best-Effort Hold
# ============================================================
#
# 자동 Torque OFF를 하지 않는다.
# 테스트 도중 Ctrl+C가 들어오면 가능한 한 현재 위치 Goal을 다시 쓴다.


def best_effort_hold(
    arm: MotorController | None,
    speed: int,
):
    """
    예외/Ctrl+C 발생 시 안전 보조 처리.

    중요:
    - Servo가 실제로 Moving=1일 때만 현재 Position Hold를 쓴다.
    - PRECHECK 오류처럼 아직 움직이지 않은 상태(Moving=0)에서는
      Servo에 어떠한 Write 명령도 보내지 않는다.
    """

    if arm is None:
        return

    try:
        moving = read_moving(
            arm
        )

        if moving != 1:
            print()
            print(
                "[SAFETY] Servo가 이동 중이 아니므로 "
                "추가 Hold Write를 보내지 않습니다."
            )
            print(
                f"[SAFETY] Moving={moving}"
            )
            return

        result = write_current_position_as_goal(
            arm,
            speed,
        )

        print()
        print(
            "[SAFETY] 이동 중 예외 감지 -> "
            "Best-Effort Current Position Hold"
        )
        print(
            result
        )

    except Exception as error:
        print()
        print(
            "[WARNING] Current Position Hold 확인/실행 실패:"
        )
        print(error)


# ============================================================
# 9. Calibration / Safe Range 사전검사
# ============================================================

def precheck(
    arm: MotorController,
):
    section(
        "PRECHECK"
    )

    servo = (
        arm.calibration.get_joint(
            TEST_JOINT
        )
    )

    servo_id = int(
        servo["servo_id"]
    )

    if servo_id != TEST_SERVO_ID:
        raise RuntimeError(
            f"{TEST_JOINT} Servo ID 불일치: "
            f"{servo_id}"
        )

    max_speed = int(
        servo["max_speed"]
    )

    test_speed = min(
        DEFAULT_TEST_SPEED,
        max_speed,
    )

    safe_min, safe_max = (
        arm.calibration.get_safe_angle_range(
            TEST_JOINT
        )
    )

    print(
        f"Joint       : {TEST_JOINT}"
    )
    print(
        f"Servo ID    : {servo_id}"
    )
    print(
        f"Safe Angle  : "
        f"{safe_min:+.2f}° ~ {safe_max:+.2f}°"
    )
    print(
        f"max_speed   : {max_speed}"
    )
    print(
        f"test_speed  : {test_speed}"
    )
    print(
        f"Torque      : {read_torque_enable(arm)}"
    )

    required_angles = (
        0.0,
        FIRST_TARGET_DEG,
        RECALL1_TARGET_DEG,
        RECALL2_TARGET_DEG,
        RECALL1_TRIGGER_DEG,
        RECALL2_TRIGGER_DEG,
        STOP_TRIGGER_DEG,
    )

    for angle in required_angles:
        if not (
            safe_min
            <= angle
            <= safe_max
        ):
            raise RuntimeError(
                f"테스트 각도 {angle:+.1f}°가 "
                f"Safe Range 밖입니다."
            )

    torque = read_torque_enable(
        arm
    )

    if torque != 1:
        raise RuntimeError(
            "Servo3 Torque Enable이 1이 아닙니다. "
            f"현재 값={torque}. "
            "이 스크립트는 Torque를 자동으로 켜지 않습니다."
        )

    print()
    print(
        "[OK] 사전검사 통과"
    )

    return test_speed


# ============================================================
# 10. 특정 TEAM Angle까지 도달할 때까지 관찰
# ============================================================

def wait_until_angle_reaches(
    arm: MotorController,
    target_angle: float,
    timeout: float,
    stage: str,
):
    """
    지정한 TEAM Angle 이상에 도달할 때까지 실시간 상태를 터미널에 출력한다.
    """

    start = time.monotonic()
    samples = []

    while (
        time.monotonic()
        - start
        < timeout
    ):
        sample = print_live_status(
            arm,
            stage,
            start,
        )

        if sample["raw"] is None:
            print()
            raise RuntimeError(
                "Position 읽기 실패"
            )

        samples.append(
            sample
        )

        if sample["angle"] >= target_angle:
            print()
            return samples

        time.sleep(
            MONITOR_INTERVAL_SEC
        )

    print()

    raise RuntimeError(
        f"{target_angle:+.1f}° 도달 대기 Timeout"
    )


# ============================================================
# 11. 정지/목표 도달 모니터
# ============================================================

def monitor_until_stopped(
    arm: MotorController,
    expected_angle: float,
    timeout: float,
    stage: str,
):
    """
    최종 목표 도착 여부를 확인하면서
    현재 Angle / Raw / Goal / Moving / Torque를 계속 표시한다.
    """

    start = time.monotonic()
    samples = []
    stable_stop_count = 0

    while (
        time.monotonic()
        - start
        < timeout
    ):
        sample = print_live_status(
            arm,
            stage,
            start,
        )

        if sample["raw"] is None:
            print()
            raise RuntimeError(
                "Position 읽기 실패"
            )

        samples.append(
            sample
        )

        if (
            sample["moving"] == 0
            and
            abs(
                sample["angle"]
                - expected_angle
            )
            <= FINAL_ANGLE_TOLERANCE_DEG
        ):
            stable_stop_count += 1
        else:
            stable_stop_count = 0

        if stable_stop_count >= 3:
            print()
            return samples

        time.sleep(
            MONITOR_INTERVAL_SEC
        )

    print()
    return samples


# ============================================================
# 12. TEST 1 - move_joint() 이동 중 재호출
# ============================================================

def test_move_joint_recall(
    arm: MotorController,
    speed: int,
):
    """
    move_joint()을 이동 중 두 번 재호출한다.

    0° -> +50°
    +10° 부근에서 +30°로 1차 재호출
    +20° 부근에서 +25°로 2차 재호출

    최종적으로 마지막 명령인 +25° 부근에 정지해야 한다.
    """

    section(
        "TEST 1 - move_joint() 이동 중 2회 재호출"
    )

    print(
        "검증 순서:"
    )
    print(
        "  ① 0° -> +50°   : 최초 호출"
    )
    print(
        "  ② 약 +10°      : +30°로 1차 재호출"
    )
    print(
        "  ③ 약 +20°      : +25°로 2차 재호출"
    )
    print(
        "  ④ 최종 위치    : +25° 부근에서 정지"
    )
    print()
    print(
        "터미널 표시:"
    )
    print(
        "  Stage / 경과시간 / TEAM Angle / Raw / Goal / Moving / Torque"
    )

    wait_enter(
        "먼저 wrist_flex를 Zero로 이동합니다."
    )

    if not arm.move_to_zero(
        TEST_JOINT,
        speed=speed,
        acc=TEST_ACC,
        wait=True,
        timeout=COMMAND_TIMEOUT_SEC,
    ):
        raise RuntimeError(
            "Zero 이동 실패"
        )

    expected_raw_50 = (
        arm.calibration.command_angle_to_position(
            TEST_JOINT,
            FIRST_TARGET_DEG,
        )
    )

    expected_raw_30 = (
        arm.calibration.command_angle_to_position(
            TEST_JOINT,
            RECALL1_TARGET_DEG,
        )
    )

    expected_raw_25 = (
        arm.calibration.command_angle_to_position(
            TEST_JOINT,
            RECALL2_TARGET_DEG,
        )
    )

    print()
    print(
        "[EXPECTED GOAL RAW]"
    )
    print(
        f"최초 +50° = {expected_raw_50}"
    )
    print(
        f"1차  +30° = {expected_raw_30}"
    )
    print(
        f"2차  +25° = {expected_raw_25}"
    )

    wait_enter(
        "TEST 1을 시작합니다. "
        "wrist_flex TEAM + 방향은 '위'입니다."
    )

    # ========================================================
    # CALL 1 : +50°
    # ========================================================

    call1_start = time.monotonic()

    call1_result = arm.move_joint(
        TEST_JOINT,
        angle=FIRST_TARGET_DEG,
        speed=speed,
        acc=TEST_ACC,
        wait=False,
    )

    call1_elapsed = (
        time.monotonic()
        - call1_start
    )

    goal_after_call1 = (
        read_goal_position(
            arm
        )
    )

    print()
    print(
        f"[CALL 1] +50° | "
        f"return={call1_result} | "
        f"elapsed={call1_elapsed:.4f}s | "
        f"Goal={goal_after_call1}"
    )

    if not call1_result:
        raise RuntimeError(
            "첫 번째 move_joint() 실패"
        )

    # ========================================================
    # +10° 부근까지 실시간 관찰
    # ========================================================

    samples_before_recall1 = (
        wait_until_angle_reaches(
            arm,
            RECALL1_TRIGGER_DEG,
            timeout=COMMAND_TIMEOUT_SEC,
            stage="TO RECALL1",
        )
    )

    recall1_raw, recall1_angle = (
        read_position_and_angle(
            arm
        )
    )

    print()
    print(
        f"[RECALL 1 POINT] "
        f"Angle={recall1_angle:+.2f}° | "
        f"Raw={recall1_raw}"
    )

    # ========================================================
    # CALL 2 : +30°로 1차 재호출
    # ========================================================

    call2_start = time.monotonic()

    call2_result = arm.move_joint(
        TEST_JOINT,
        angle=RECALL1_TARGET_DEG,
        speed=speed,
        acc=TEST_ACC,
        wait=False,
    )

    call2_elapsed = (
        time.monotonic()
        - call2_start
    )

    goal_after_call2 = (
        read_goal_position(
            arm
        )
    )

    print()
    print(
        f"[CALL 2] +30° | "
        f"return={call2_result} | "
        f"elapsed={call2_elapsed:.4f}s | "
        f"Goal={goal_after_call2}"
    )

    # ========================================================
    # +20° 부근까지 계속 실시간 관찰
    # ========================================================

    samples_before_recall2 = (
        wait_until_angle_reaches(
            arm,
            RECALL2_TRIGGER_DEG,
            timeout=COMMAND_TIMEOUT_SEC,
            stage="TO RECALL2",
        )
    )

    recall2_raw, recall2_angle = (
        read_position_and_angle(
            arm
        )
    )

    print()
    print(
        f"[RECALL 2 POINT] "
        f"Angle={recall2_angle:+.2f}° | "
        f"Raw={recall2_raw}"
    )

    # ========================================================
    # CALL 3 : +25°로 2차 재호출
    # ========================================================

    call3_start = time.monotonic()

    call3_result = arm.move_joint(
        TEST_JOINT,
        angle=RECALL2_TARGET_DEG,
        speed=speed,
        acc=TEST_ACC,
        wait=False,
    )

    call3_elapsed = (
        time.monotonic()
        - call3_start
    )

    goal_after_call3 = (
        read_goal_position(
            arm
        )
    )

    print()
    print(
        f"[CALL 3] +25° | "
        f"return={call3_result} | "
        f"elapsed={call3_elapsed:.4f}s | "
        f"Goal={goal_after_call3}"
    )

    # ========================================================
    # 마지막 목표 +25°에서 정지하는지 확인
    # ========================================================

    after_recall2_samples = (
        monitor_until_stopped(
            arm,
            expected_angle=RECALL2_TARGET_DEG,
            timeout=COMMAND_TIMEOUT_SEC,
            stage="FINAL",
        )
    )

    final_raw, final_angle = (
        read_position_and_angle(
            arm
        )
    )

    final_goal = (
        read_goal_position(
            arm
        )
    )

    final_moving = (
        read_moving(
            arm
        )
    )

    final_torque = (
        read_torque_enable(
            arm
        )
    )

    max_angle_after_recall2 = max(
        sample["angle"]
        for sample in after_recall2_samples
    )

    goal1_ok = (
        goal_after_call1
        == expected_raw_50
    )

    goal2_ok = (
        goal_after_call2
        == expected_raw_30
    )

    goal3_ok = (
        goal_after_call3
        == expected_raw_25
    )

    final_goal_ok = (
        final_goal
        == expected_raw_25
    )

    final_ok = (
        final_angle is not None
        and
        abs(
            final_angle
            - RECALL2_TARGET_DEG
        )
        <= FINAL_ANGLE_TOLERANCE_DEG
    )

    final_moving_ok = (
        final_moving == 0
    )

    torque_ok = (
        final_torque == 1
    )

    passed = (
        call1_result is True
        and
        call2_result is True
        and
        call3_result is True
        and
        goal1_ok
        and
        goal2_ok
        and
        goal3_ok
        and
        final_goal_ok
        and
        final_ok
        and
        final_moving_ok
        and
        torque_ok
    )

    print()
    print(
        "------------------------------------------------------------"
    )
    print(
        "TEST 1 FINAL STATE"
    )
    print(
        "------------------------------------------------------------"
    )
    print(
        f"최종 TEAM Angle : {final_angle:+.2f}°"
    )
    print(
        f"최종 Raw        : {final_raw}"
    )
    print(
        f"최종 Goal       : {final_goal}"
    )
    print(
        f"최종 Moving     : {final_moving}"
    )
    print(
        f"최종 Torque     : {final_torque}"
    )
    print()
    print(
        f"Goal 50° 기록   : {'PASS' if goal1_ok else 'FAIL'}"
    )
    print(
        f"Goal 30° 갱신   : {'PASS' if goal2_ok else 'FAIL'}"
    )
    print(
        f"Goal 25° 갱신   : {'PASS' if goal3_ok else 'FAIL'}"
    )
    print(
        f"최종 25° 정지   : {'PASS' if final_ok else 'FAIL'}"
    )

    detail = {
        "call1": {
            "target_deg":
                FIRST_TARGET_DEG,
            "return":
                call1_result,
            "elapsed_sec":
                call1_elapsed,
            "expected_goal_raw":
                expected_raw_50,
            "actual_goal_raw":
                goal_after_call1,
        },

        "recall1": {
            "trigger_deg":
                RECALL1_TRIGGER_DEG,
            "actual_trigger_angle":
                recall1_angle,
            "actual_trigger_raw":
                recall1_raw,
            "new_target_deg":
                RECALL1_TARGET_DEG,
            "return":
                call2_result,
            "elapsed_sec":
                call2_elapsed,
            "expected_goal_raw":
                expected_raw_30,
            "actual_goal_raw":
                goal_after_call2,
        },

        "recall2": {
            "trigger_deg":
                RECALL2_TRIGGER_DEG,
            "actual_trigger_angle":
                recall2_angle,
            "actual_trigger_raw":
                recall2_raw,
            "new_target_deg":
                RECALL2_TARGET_DEG,
            "return":
                call3_result,
            "elapsed_sec":
                call3_elapsed,
            "expected_goal_raw":
                expected_raw_25,
            "actual_goal_raw":
                goal_after_call3,
        },

        "final": {
            "angle":
                final_angle,
            "raw":
                final_raw,
            "goal_raw":
                final_goal,
            "moving":
                final_moving,
            "torque":
                final_torque,
            "max_angle_after_recall2":
                max_angle_after_recall2,
        },

        "checks": {
            "goal1_ok":
                goal1_ok,
            "goal2_ok":
                goal2_ok,
            "goal3_ok":
                goal3_ok,
            "final_goal_ok":
                final_goal_ok,
            "final_angle_ok":
                final_ok,
            "final_moving_ok":
                final_moving_ok,
            "torque_ok":
                torque_ok,
        },
    }

    record(
        "move_joint() running re-call x2",
        passed,
        detail,
    )

    wait_enter(
        "TEST 1이 끝났습니다. "
        "wrist_flex를 Zero로 복귀합니다."
    )

    arm.move_to_zero(
        TEST_JOINT,
        speed=speed,
        acc=TEST_ACC,
        wait=True,
        timeout=COMMAND_TIMEOUT_SEC,
    )

    return passed


# ============================================================
# 13. TEST 2 - Torque 유지 정지 후보
# ============================================================

def test_user_stop_candidate(
    arm: MotorController,
    speed: int,
):
    """
    사용자용 비상정지 후보의 전체 흐름을 독립 테스트한다.

    중요:
    현재 motor_control 패키지에는 아직 사용자용
    stop/resume 공개 API를 추가하지 않았다.

    따라서 이 테스트 파일 내부에서만 별도의 software latch를 만들어
    최종 패키지에 넣을 동작 정책을 검증한다.

    흐름:
        1) wrist_flex 0° -> +50° 이동
        2) 사용자가 '1' 입력
        3) software latch ON
        4) 현재 Position을 Goal로 덮어써 Torque ON 상태로 정지
        5) latch 상태에서 +30° 이동을 일부러 재호출
           -> Servo에 명령을 보내지 않고 BLOCK되어야 함
        6) 사용자가 '2' 입력
        7) latch OFF
           -> 이 순간에는 모터 명령을 보내지 않음
        8) +30° 이동을 다시 호출
           -> 이번에는 정상 실행되어야 함

    즉 최종 사용자용 API에서 필요한:
        STOP
        -> 이동 차단
        -> RESUME
        -> 다시 이동 허용
    전체 정책을 확인한다.
    """

    section(
        "TEST 2 - 사용자 비상정지 + 재호출 차단 + 해제"
    )

    print(
        "이 테스트는 기존 arm.emergency_stop()을 사용하지 않습니다."
    )
    print(
        "기존 emergency_stop()은 Torque OFF용 개발/점검 기능입니다."
    )
    print()
    print(
        "이번 테스트:"
    )
    print(
        "  ① 0° -> +50° 이동"
    )
    print(
        "  ② 이동 중 '1' 입력 = 사용자 비상정지 후보"
    )
    print(
        "  ③ Torque ON 상태에서 현재 자세 Hold"
    )
    print(
        "  ④ 정지 latch 중 +30° 이동 재호출 -> BLOCK 확인"
    )
    print(
        "  ⑤ '2' 입력 = 정지 latch 해제"
    )
    print(
        "  ⑥ 해제 자체로는 움직이지 않는지 확인"
    )
    print(
        "  ⑦ +30° 이동 재호출 -> 정상 실행 확인"
    )

    # 이 Event는 테스트 전용.
    # 최종 패키지에서는 Controller 내부 상태로 구현할 후보이다.
    user_stop_latched = threading.Event()

    def guarded_move_joint(
        angle: float,
        *,
        wait: bool,
    ):
        """
        최종 사용자용 stop latch 정책을 테스트 파일에서 모사한다.

        latch ON이면 실제 arm.move_joint()을 절대 호출하지 않는다.
        따라서 Servo Goal Position도 변경되어서는 안 된다.
        """

        if user_stop_latched.is_set():
            print()
            print(
                "[BLOCK] USER STOP latch가 ON이므로 "
                f"move_joint({angle:+.1f}°)을 Servo에 보내지 않습니다."
            )
            return False

        return arm.move_joint(
            TEST_JOINT,
            angle=angle,
            speed=speed,
            acc=TEST_ACC,
            wait=wait,
            timeout=COMMAND_TIMEOUT_SEC,
        )

    wait_enter(
        "먼저 wrist_flex를 Zero로 이동합니다."
    )

    if not arm.move_to_zero(
        TEST_JOINT,
        speed=speed,
        acc=TEST_ACC,
        wait=True,
        timeout=COMMAND_TIMEOUT_SEC,
    ):
        raise RuntimeError(
            "Zero 이동 실패"
        )

    torque_before = (
        read_torque_enable(
            arm
        )
    )

    if torque_before != 1:
        raise RuntimeError(
            f"테스트 전 Torque Enable이 1이 아닙니다: "
            f"{torque_before}"
        )

    wait_enter(
        "TEST 2를 시작합니다. "
        "모터가 움직이면 원하는 순간 '1'을 입력하고 Enter를 누르세요."
    )

    stop_event = threading.Event()
    input_time_holder = {
        "time": None,
    }

    def stop_input_worker():
        while True:
            try:
                command = input(
                    "\n[USER STOP] 이동 중 1 입력 후 Enter: "
                ).strip()
            except EOFError:
                return

            if command == "1":
                input_time_holder["time"] = time.monotonic()
                stop_event.set()
                return

            print()
            print(
                "[INFO] 사용자 비상정지는 정확히 1을 입력하세요."
            )

    input_thread = threading.Thread(
        target=stop_input_worker,
        daemon=True,
    )

    input_thread.start()

    # ========================================================
    # A. +50° 이동 시작
    # ========================================================

    move_start_time = time.monotonic()

    move_result = guarded_move_joint(
        FIRST_TARGET_DEG,
        wait=False,
    )

    if not move_result:
        raise RuntimeError(
            "+50° 이동 시작 실패"
        )

    goal_before_stop = (
        read_goal_position(
            arm
        )
    )

    print()
    print(
        f"[MOVE START] target=+{FIRST_TARGET_DEG:.1f}° "
        f"| Goal={goal_before_stop}"
    )

    samples_before_stop = []

    while not stop_event.is_set():

        sample = print_live_status(
            arm,
            "WAIT 1",
            move_start_time,
        )

        samples_before_stop.append(
            sample
        )

        if sample["raw"] is None:
            print()
            raise RuntimeError(
                "Position 읽기 실패"
            )

        if (
            sample["moving"] == 0
            and
            abs(
                sample["angle"]
                - FIRST_TARGET_DEG
            )
            <= FINAL_ANGLE_TOLERANCE_DEG
        ):
            print()
            raise RuntimeError(
                "1 입력 전에 +50° 목표에 도착했습니다. "
                "다시 실행해서 이동 중에 1을 입력하세요."
            )

        time.sleep(
            MONITOR_INTERVAL_SEC
        )

    print()

    # ========================================================
    # B. 1 입력 즉시 latch ON
    # ========================================================
    #
    # 최종 기능에서도 먼저 latch를 걸고,
    # 그 다음 물리 정지 동작을 수행하는 방식이 안전하다.
    # 이렇게 해야 정지 처리 중 다른 이동 명령이 끼어들지 않는다.

    user_stop_latched.set()

    print(
        "[USER STOP] latch = ON"
    )

    input_detected_time = (
        input_time_holder["time"]
        if input_time_holder["time"] is not None
        else time.monotonic()
    )

    before_stop_raw, before_stop_angle = (
        read_position_and_angle(
            arm
        )
    )

    stop_command_start = time.monotonic()

    stop_result = (
        write_current_position_as_goal(
            arm,
            speed=speed,
        )
    )

    stop_command_end = time.monotonic()

    if not stop_result.get(
        "success"
    ):
        raise RuntimeError(
            f"Current Position Hold 명령 실패: "
            f"{stop_result}"
        )

    hold_raw = int(
        stop_result[
            "hold_raw"
        ]
    )

    hold_angle = float(
        stop_result[
            "hold_angle"
        ]
    )

    goal_after_stop = (
        read_goal_position(
            arm
        )
    )

    torque_immediately_after = (
        read_torque_enable(
            arm
        )
    )

    input_to_command_delay = (
        stop_command_start
        - input_detected_time
    )

    command_elapsed = (
        stop_command_end
        - stop_command_start
    )

    print()
    print(
        "------------------------------------------------------------"
    )
    print(
        "USER STOP RESULT"
    )
    print(
        "------------------------------------------------------------"
    )
    print(
        f"정지 입력 직전 Angle : {before_stop_angle:+.2f}°"
    )
    print(
        f"Hold Goal Angle      : {hold_angle:+.2f}°"
    )
    print(
        f"Hold Goal Raw        : {hold_raw}"
    )
    print(
        f"Goal Register        : {goal_after_stop}"
    )
    print(
        f"Torque Enable        : {torque_immediately_after}"
    )
    print(
        f"USER STOP latch      : {user_stop_latched.is_set()}"
    )
    print(
        f"입력→정지처리 지연    : {input_to_command_delay:.4f}s"
    )
    print(
        f"정지 명령 처리시간    : {command_elapsed:.4f}s"
    )

    # ========================================================
    # C. 정지 후 2초 상태 관찰
    # ========================================================

    observe_start = time.monotonic()
    samples_after_stop = []

    while (
        time.monotonic()
        - observe_start
        < STOP_OBSERVE_SEC
    ):

        sample = print_live_status(
            arm,
            "STOPPED",
            observe_start,
        )

        sample["hold_angle"] = (
            hold_angle
        )

        samples_after_stop.append(
            sample
        )

        time.sleep(
            MONITOR_INTERVAL_SEC
        )

    print()

    stopped_raw, stopped_angle = (
        read_position_and_angle(
            arm
        )
    )

    stopped_goal = (
        read_goal_position(
            arm
        )
    )

    stopped_moving = (
        read_moving(
            arm
        )
    )

    torque_stopped = (
        read_torque_enable(
            arm
        )
    )

    max_extra_travel = max(
        abs(
            sample["angle"]
            - hold_angle
        )
        for sample in samples_after_stop
        if sample["angle"] is not None
    )

    physical_hold = ask_yes_no(
        "실제로 모터 힘이 유지되고 현재 자세를 잡고 있나요?"
    )

    # ========================================================
    # D. latch 상태에서 이동 함수 재호출 -> 반드시 BLOCK
    # ========================================================

    section(
        "TEST 2-A - USER STOP 중 함수 재호출 BLOCK"
    )

    print(
        "현재 USER STOP latch = ON 상태입니다."
    )
    print(
        "이제 +30° move_joint()을 일부러 호출합니다."
    )
    print(
        "정상이라면 Servo에는 아무 명령도 가지 않아야 합니다."
    )

    goal_before_block_test = (
        read_goal_position(
            arm
        )
    )

    blocked_call_result = (
        guarded_move_joint(
            RECALL1_TARGET_DEG,
            wait=False,
        )
    )

    time.sleep(
        0.3
    )

    goal_after_block_test = (
        read_goal_position(
            arm
        )
    )

    blocked_raw, blocked_angle = (
        read_position_and_angle(
            arm
        )
    )

    blocked_moving = (
        read_moving(
            arm
        )
    )

    blocked_torque = (
        read_torque_enable(
            arm
        )
    )

    block_ok = (
        blocked_call_result is False
        and
        goal_after_block_test
        == goal_before_block_test
    )

    print()
    print(
        f"재호출 return       : {blocked_call_result}"
    )
    print(
        f"호출 전 Goal        : {goal_before_block_test}"
    )
    print(
        f"호출 후 Goal        : {goal_after_block_test}"
    )
    print(
        f"현재 Angle          : {blocked_angle:+.2f}°"
    )
    print(
        f"현재 Moving         : {blocked_moving}"
    )
    print(
        f"현재 Torque         : {blocked_torque}"
    )
    print(
        f"BLOCK 검증          : {'PASS' if block_ok else 'FAIL'}"
    )

    record(
        "USER STOP latch blocks move_joint()",
        block_ok,
        {
            "call_return":
                blocked_call_result,
            "goal_before":
                goal_before_block_test,
            "goal_after":
                goal_after_block_test,
            "angle":
                blocked_angle,
            "moving":
                blocked_moving,
            "torque":
                blocked_torque,
        },
    )

    if not block_ok:
        raise RuntimeError(
            "USER STOP latch 중 이동 차단 실패"
        )

    # ========================================================
    # E. 사용자 입력 2 -> latch 해제
    # ========================================================

    section(
        "TEST 2-B - USER STOP 해제"
    )

    print(
        "이제 사용자용 비상정지 상태를 해제합니다."
    )
    print(
        "중요: 해제 자체는 Servo에 이동 명령을 보내지 않습니다."
    )
    print(
        "따라서 2를 입력한 순간 로봇팔이 움직이면 안 됩니다."
    )

    while True:
        command = input(
            "해제하려면 2 입력 후 Enter: "
        ).strip()

        if command == "2":
            break

        print(
            "[INFO] 해제는 정확히 2를 입력하세요."
        )

    raw_before_release, angle_before_release = (
        read_position_and_angle(
            arm
        )
    )

    goal_before_release = (
        read_goal_position(
            arm
        )
    )

    user_stop_latched.clear()

    print()
    print(
        "[USER STOP] latch = OFF"
    )

    # 해제 자체로 움직이지 않는지 짧게 관찰
    release_observe_start = time.monotonic()
    release_samples = []

    while (
        time.monotonic()
        - release_observe_start
        < 0.8
    ):
        sample = print_live_status(
            arm,
            "RELEASED",
            release_observe_start,
        )
        release_samples.append(
            sample
        )
        time.sleep(
            MONITOR_INTERVAL_SEC
        )

    print()

    raw_after_release, angle_after_release = (
        read_position_and_angle(
            arm
        )
    )

    goal_after_release = (
        read_goal_position(
            arm
        )
    )

    torque_after_release = (
        read_torque_enable(
            arm
        )
    )

    release_did_not_command_motion = (
        goal_after_release
        == goal_before_release
    )

    release_position_shift = abs(
        angle_after_release
        - angle_before_release
    )

    release_ok = (
        not user_stop_latched.is_set()
        and
        release_did_not_command_motion
        and
        torque_after_release == 1
        and
        release_position_shift
        <= FINAL_ANGLE_TOLERANCE_DEG
    )

    print()
    print(
        f"해제 전 Angle       : {angle_before_release:+.2f}°"
    )
    print(
        f"해제 후 Angle       : {angle_after_release:+.2f}°"
    )
    print(
        f"해제 전 Goal        : {goal_before_release}"
    )
    print(
        f"해제 후 Goal        : {goal_after_release}"
    )
    print(
        f"Torque              : {torque_after_release}"
    )
    print(
        f"해제 자체 무동작     : {'PASS' if release_ok else 'FAIL'}"
    )

    record(
        "USER STOP release does not move servo",
        release_ok,
        {
            "angle_before":
                angle_before_release,
            "angle_after":
                angle_after_release,
            "position_shift_deg":
                release_position_shift,
            "goal_before":
                goal_before_release,
            "goal_after":
                goal_after_release,
            "torque":
                torque_after_release,
        },
    )

    if not release_ok:
        raise RuntimeError(
            "USER STOP 해제 동작 검증 실패"
        )

    # ========================================================
    # F. 해제 후 같은 +30° 명령 재호출 -> 이번에는 정상 동작
    # ========================================================

    section(
        "TEST 2-C - 해제 후 함수 재호출"
    )

    wait_enter(
        "이제 USER STOP이 해제되었습니다. "
        "동일하게 +30° move_joint()을 호출합니다."
    )

    expected_resume_goal = (
        arm.calibration.command_angle_to_position(
            TEST_JOINT,
            RECALL1_TARGET_DEG,
        )
    )

    resume_call_result = (
        guarded_move_joint(
            RECALL1_TARGET_DEG,
            wait=False,
        )
    )

    resume_goal = (
        read_goal_position(
            arm
        )
    )

    print()
    print(
        f"[RESUME CALL] +30° | "
        f"return={resume_call_result} | "
        f"Goal={resume_goal} | "
        f"Expected={expected_resume_goal}"
    )

    resume_samples = (
        monitor_until_stopped(
            arm,
            expected_angle=RECALL1_TARGET_DEG,
            timeout=COMMAND_TIMEOUT_SEC,
            stage="RESUMED",
        )
    )

    final_raw, final_angle = (
        read_position_and_angle(
            arm
        )
    )

    final_goal = (
        read_goal_position(
            arm
        )
    )

    final_moving = (
        read_moving(
            arm
        )
    )

    final_torque = (
        read_torque_enable(
            arm
        )
    )

    resume_ok = (
        resume_call_result is True
        and
        resume_goal
        == expected_resume_goal
        and
        final_goal
        == expected_resume_goal
        and
        abs(
            final_angle
            - RECALL1_TARGET_DEG
        )
        <= FINAL_ANGLE_TOLERANCE_DEG
        and
        final_moving == 0
        and
        final_torque == 1
    )

    record(
        "move_joint() works after USER STOP release",
        resume_ok,
        {
            "call_return":
                resume_call_result,
            "expected_goal":
                expected_resume_goal,
            "goal_after_call":
                resume_goal,
            "final_goal":
                final_goal,
            "final_angle":
                final_angle,
            "final_raw":
                final_raw,
            "final_moving":
                final_moving,
            "final_torque":
                final_torque,
        },
    )

    # ========================================================
    # G. TEST 2 전체 판정
    # ========================================================

    goal_overwritten = (
        goal_after_stop
        == hold_raw
        and
        stopped_goal
        == hold_raw
    )

    torque_kept = (
        torque_before == 1
        and
        torque_immediately_after == 1
        and
        torque_stopped == 1
        and
        blocked_torque == 1
        and
        torque_after_release == 1
        and
        final_torque == 1
    )

    stopped_near_hold = (
        abs(
            stopped_angle
            - hold_angle
        )
        <= FINAL_ANGLE_TOLERANCE_DEG
    )

    moving_stopped = (
        stopped_moving == 0
    )

    passed = (
        goal_overwritten
        and
        torque_kept
        and
        stopped_near_hold
        and
        moving_stopped
        and
        physical_hold
        and
        block_ok
        and
        release_ok
        and
        resume_ok
    )

    print()
    print(
        "============================================================"
    )
    print(
        "TEST 2 FINAL SUMMARY"
    )
    print(
        "============================================================"
    )
    print(
        f"Torque 유지 정지       : "
        f"{'PASS' if goal_overwritten and stopped_near_hold else 'FAIL'}"
    )
    print(
        f"Torque=1 유지          : "
        f"{'PASS' if torque_kept else 'FAIL'}"
    )
    print(
        f"정지 후 Moving=0       : "
        f"{'PASS' if moving_stopped else 'FAIL'}"
    )
    print(
        f"실제 자세 유지         : "
        f"{'PASS' if physical_hold else 'FAIL'}"
    )
    print(
        f"정지 중 이동 재호출 차단: "
        f"{'PASS' if block_ok else 'FAIL'}"
    )
    print(
        f"2 입력 해제 자체 무동작 : "
        f"{'PASS' if release_ok else 'FAIL'}"
    )
    print(
        f"해제 후 재호출 가능     : "
        f"{'PASS' if resume_ok else 'FAIL'}"
    )
    print(
        f"정지 후 최대 추가이동   : "
        f"{max_extra_travel:.2f}°"
    )

    record(
        "USER STOP + latch + release full candidate",
        passed,
        {
            "hold_angle":
                hold_angle,
            "stopped_angle":
                stopped_angle,
            "max_extra_travel_deg":
                max_extra_travel,
            "goal_overwritten":
                goal_overwritten,
            "torque_kept":
                torque_kept,
            "moving_stopped":
                moving_stopped,
            "physical_hold":
                physical_hold,
            "block_ok":
                block_ok,
            "release_ok":
                release_ok,
            "resume_ok":
                resume_ok,
        },
    )

    if passed:
        wait_enter(
            "TEST 2 전체 검증이 끝났습니다. "
            "wrist_flex를 Zero로 복귀합니다."
        )

        zero_result = arm.move_to_zero(
            TEST_JOINT,
            speed=speed,
            acc=TEST_ACC,
            wait=True,
            timeout=COMMAND_TIMEOUT_SEC,
        )

        record(
            "move_to_zero() after USER STOP full test",
            zero_result is True,
            {
                "return":
                    zero_result,
            },
        )

    else:
        print()
        print(
            "[FAIL] 사용자용 비상정지 후보 전체 조건을 "
            "만족하지 않았으므로 자동 Zero 복귀하지 않습니다."
        )

    return passed


# ============================================================
# 14. Main
# ============================================================

def main():
    section(
        "MOTOR RE-CALL x2 + USER STOP / RELEASE TEST (v4)"
    )

    print(
        f"Project root expected : "
        f"{PROJECT_ROOT}"
    )
    print(
        f"Hardware dir          : "
        f"{HARDWARE_DIR}"
    )
    print(
        f"Calibration           : "
        f"{CALIBRATION_FILE}"
    )

    if not CALIBRATION_FILE.exists():
        raise SystemExit(
            "[FAIL] servo_calibration_result.json을 찾을 수 없습니다."
        )

    print()
    print(
        "메뉴:"
    )
    print(
        "1 = TEST 1 재호출만"
    )
    print(
        "2 = TEST 2 Torque 유지 정지 후보만"
    )
    print(
        "3 = 두 테스트 순서대로 실행 (권장)"
    )

    choice = input(
        "선택 [1/2/3]: "
    ).strip()

    if choice not in (
        "1",
        "2",
        "3",
    ):
        raise SystemExit(
            "[INFO] 테스트 취소"
        )

    arm = None
    test_speed = DEFAULT_TEST_SPEED

    try:
        arm = MotorController(
            calibration_file=str(
                CALIBRATION_FILE
            )
        )

        test_speed = precheck(
            arm
        )

        print()
        print(
            "[IMPORTANT]"
        )
        print(
            "실제 Servo3 wrist_flex가 움직입니다."
        )
        print(
            "주변 간섭이 없는지 확인하세요."
        )

        if not ask_yes_no(
            "실제 하드웨어 테스트를 진행할까요?"
        ):
            print(
                "[INFO] 사용자가 테스트 취소"
            )
            return

        if choice in (
            "1",
            "3",
        ):
            test_move_joint_recall(
                arm,
                test_speed,
            )

        if choice in (
            "2",
            "3",
        ):
            test_user_stop_candidate(
                arm,
                test_speed,
            )

    except KeyboardInterrupt:
        print()
        print(
            "[WARNING] Ctrl+C 감지"
        )

        best_effort_hold(
            arm,
            test_speed,
        )

        record(
            "KeyboardInterrupt",
            False,
            "사용자 Ctrl+C",
        )

    except Exception as error:
        print()
        print(
            "============================================================"
        )
        print(
            "[TEST ERROR]"
        )
        print(
            error
        )
        print(
            "============================================================"
        )

        best_effort_hold(
            arm,
            test_speed,
        )

        record(
            "Unhandled error",
            False,
            str(error),
        )

    finally:
        if arm is not None:
            try:
                arm.close()
            except Exception as error:
                print(
                    f"[WARNING] close() 실패: {error}"
                )

        if RESULTS:
            try:
                save_results()
            except Exception as error:
                print(
                    f"[WARNING] 결과 저장 실패: {error}"
                )


if __name__ == "__main__":
    main()

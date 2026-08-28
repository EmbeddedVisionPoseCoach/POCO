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
    if arm is None:
        return

    try:
        result = write_current_position_as_goal(
            arm,
            speed,
        )

        print()
        print(
            "[SAFETY] Best-Effort Current Position Hold"
        )
        print(
            result
        )

    except Exception as error:
        print()
        print(
            "[WARNING] Current Position Hold 실패:"
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
        RECALL_TRIGGER_DEG,
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
    실제 사용자 비상정지 입력과 비슷하게 테스트한다.

    wrist_flex를 0° -> +50°로 이동시키고,
    사용자가 원하는 순간 터미널에서 STOP + Enter를 입력한다.

    STOP 입력이 감지되면:
        Present Position 읽기
        -> 그 Position을 새 Goal Position으로 즉시 덮어쓰기
        -> Torque는 계속 ON

    확인:
        - 기존 목표가 현재 위치 Goal로 바뀌는가
        - Moving이 0으로 되는가
        - Torque Enable이 계속 1인가
        - 실제 팔이 힘을 유지하는가
        - STOP 입력 후 얼마나 더 이동했는가
    """

    section(
        "TEST 2 - 수동 입력 사용자 비상정지 후보"
    )

    print(
        "이 테스트는 기존 arm.emergency_stop()을 사용하지 않습니다."
    )
    print()
    print(
        "테스트 순서:"
    )
    print(
        "  ① wrist_flex 0° -> +50° 이동 시작"
    )
    print(
        "  ② 이동 중 원하는 순간 터미널에 1 입력 후 Enter"
    )
    print(
        "  ③ 그 순간 Present Position을 새 Goal로 덮어씀"
    )
    print(
        "  ④ Torque=1 유지 여부와 실제 자세 유지 확인"
    )
    print()
    print(
        "중요: +50° 도달 전에 1을 입력하세요."
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
                ).strip().upper()
            except EOFError:
                return

            if command == "1":
                input_time_holder["time"] = time.monotonic()
                stop_event.set()
                return

            print(
                "\n[INFO] 비상정지는 정확히 1을 입력해야 합니다."
            )

    # 사용자 입력은 별도 스레드에서 기다리고,
    # 메인 스레드는 Servo 상태를 계속 출력한다.
    input_thread = threading.Thread(
        target=stop_input_worker,
        daemon=True,
    )

    input_thread.start()

    # --------------------------------------------------------
    # +50° 이동 시작
    # --------------------------------------------------------

    move_start_time = time.monotonic()

    move_result = arm.move_joint(
        TEST_JOINT,
        angle=FIRST_TARGET_DEG,
        speed=speed,
        acc=TEST_ACC,
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

    # --------------------------------------------------------
    # 1 입력 전까지 실시간 상태 출력
    # --------------------------------------------------------

    while not stop_event.is_set():

        sample = print_live_status(
            arm,
            "WAIT STOP",
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

        # 이미 목표에 도착했는데 STOP이 안 들어온 경우
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

    # --------------------------------------------------------
    # 1 입력 직후 현재 위치를 Goal로 덮어씀
    # --------------------------------------------------------

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
        "USER STOP COMMAND RESULT"
    )
    print(
        "------------------------------------------------------------"
    )
    print(
        f"정지 입력 당시 Angle : {before_stop_angle:+.2f}°"
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
        f"입력→명령 지연       : {input_to_command_delay:.4f}s"
    )
    print(
        f"정지 명령 처리시간   : {command_elapsed:.4f}s"
    )

    # --------------------------------------------------------
    # 정지 후 2초 동안 상태 계속 출력
    # --------------------------------------------------------

    observe_start = time.monotonic()
    samples_after_stop = []

    while (
        time.monotonic()
        - observe_start
        < STOP_OBSERVE_SEC
    ):

        sample = print_live_status(
            arm,
            "AFTER STOP",
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

    torque_after = (
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

    goal_overwritten = (
        goal_after_stop
        == hold_raw
        and
        final_goal
        == hold_raw
    )

    torque_kept = (
        torque_before == 1
        and
        torque_immediately_after == 1
        and
        torque_after == 1
    )

    stopped_near_hold = (
        final_angle is not None
        and
        abs(
            final_angle
            - hold_angle
        )
        <= FINAL_ANGLE_TOLERANCE_DEG
    )

    moving_stopped = (
        final_moving == 0
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
    )

    print()
    print(
        "------------------------------------------------------------"
    )
    print(
        "TEST 2 FINAL STATE"
    )
    print(
        "------------------------------------------------------------"
    )
    print(
        f"정지 당시 Angle      : {hold_angle:+.2f}°"
    )
    print(
        f"최종 Angle           : {final_angle:+.2f}°"
    )
    print(
        f"정지 후 최대 추가이동: {max_extra_travel:.2f}°"
    )
    print(
        f"최종 Goal            : {final_goal}"
    )
    print(
        f"최종 Moving          : {final_moving}"
    )
    print(
        f"최종 Torque          : {torque_after}"
    )
    print(
        f"실제 자세 유지       : {physical_hold}"
    )

    detail = {
        "move_start_return":
            move_result,

        "goal_before_stop":
            goal_before_stop,

        "stop_input_angle":
            before_stop_angle,

        "stop_input_raw":
            before_stop_raw,

        "hold_angle":
            hold_angle,

        "hold_raw":
            hold_raw,

        "goal_after_stop":
            goal_after_stop,

        "final_goal":
            final_goal,

        "input_to_command_delay_sec":
            input_to_command_delay,

        "stop_command_elapsed_sec":
            command_elapsed,

        "max_extra_travel_deg":
            max_extra_travel,

        "final_angle":
            final_angle,

        "final_raw":
            final_raw,

        "final_moving":
            final_moving,

        "torque_before":
            torque_before,

        "torque_immediately_after":
            torque_immediately_after,

        "torque_after":
            torque_after,

        "physical_hold":
            physical_hold,

        "checks": {
            "goal_overwritten":
                goal_overwritten,
            "torque_kept":
                torque_kept,
            "stopped_near_hold":
                stopped_near_hold,
            "moving_stopped":
                moving_stopped,
        },
    }

    record(
        "Manual-1-trigger Torque-ON stop candidate",
        passed,
        detail,
    )

    if passed:
        print()
        print(
            "[PASS]"
        )
        print(
            "수동 1 입력 시 현재 Goal로 변경되고 "
            "Torque를 유지한 상태에서 정지했습니다."
        )

        wait_enter(
            "검증이 끝났습니다. "
            "정상 move_to_zero()로 복귀합니다."
        )

        zero_result = arm.move_to_zero(
            TEST_JOINT,
            speed=speed,
            acc=TEST_ACC,
            wait=True,
            timeout=COMMAND_TIMEOUT_SEC,
        )

        record(
            "move_to_zero() after manual stop",
            zero_result is True,
            {
                "return":
                    zero_result,
            },
        )

    else:
        print()
        print(
            "[FAIL]"
        )
        print(
            "현재 방식은 사용자용 비상정지 요구조건을 "
            "모두 만족하지 않았습니다."
        )
        print(
            "실패 상태에서는 추가 이동을 자동 실행하지 않습니다."
        )

    return passed


# ============================================================
# 14. Main
# ============================================================

def main():
    section(
        "MOTOR RE-CALL + USER STOP CANDIDATE TEST"
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

"""
motor_control/config.py

[역할]
motor_control 패키지 전체에서 공통으로 사용하는 설정값과
팀원용 모터 제어 방향 기준을 정의한다.

[최종 팀원용 방향]
- shoulder_lift : + = 위,   - = 아래
- elbow_flex    : + = 위,   - = 아래
- wrist_flex    : + = 위,   - = 아래
- wrist_roll    : + = CW,   - = CCW

주의:
servo_calibration_result.json의 direction은 "URDF 각도 ↔ raw Position" 관계이고,
COMMAND_TO_URDF_DIRECTION은 "팀원용 각도 ↔ URDF 각도" 관계이다.
둘은 서로 다른 값이다.
"""

import os


# ============================================================
# 1. 프로젝트 경로
# ============================================================

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PACKAGE_DIR)

CALIBRATION_FILE = os.path.join(
    PROJECT_ROOT,
    "servo_calibration_result.json",
)

SDK_PATH = os.path.join(
    PROJECT_ROOT,
    "STServo_Python",
    "stservo-env",
    "scservo_sdk",
)


# ============================================================
# 2. Serial 기본 설정
# ============================================================
# Calibration JSON에 device / baudrate가 있으면 그 값을 우선 사용한다.

DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1_000_000


# ============================================================
# 3. STS Position 기준
# ============================================================

STS_POSITION_MIN = 0
STS_POSITION_MAX = 4095
STS_POSITION_RESOLUTION = 4096

POSITION_PER_DEGREE = STS_POSITION_RESOLUTION / 360.0
DEGREE_PER_POSITION = 360.0 / STS_POSITION_RESOLUTION


# ============================================================
# 4. 팀원 입력 각도 -> URDF 각도 방향
# ============================================================
# +1 : 팀원 +각도 = URDF +각도
# -1 : 팀원 +각도 = URDF -각도
#
# wrist_roll은 실물 재확인 결과:
#   관찰 기준 = 모니터가 위치한 정면에서 로봇팔을 바라보는 기준
#   raw 증가 = CW
#   raw 감소 = CCW
#   calibration direction = -1
# 따라서 URDF + -> raw 감소 -> CCW이고,
# 팀원 +는 CW이므로 URDF -와 같은 방향이다.
# 따라서 wrist_roll command direction은 -1이다.

COMMAND_TO_URDF_DIRECTION = {
    "shoulder_lift": -1,
    "elbow_flex": -1,
    "wrist_flex": -1,
    "wrist_roll": -1,
}


COMMAND_DIRECTION_DESCRIPTION = {
    "shoulder_lift": {"positive": "위", "negative": "아래"},
    "elbow_flex": {"positive": "위", "negative": "아래"},
    "wrist_flex": {"positive": "위", "negative": "아래"},
    "wrist_roll": {"positive": "CW", "negative": "CCW"},
}


# ============================================================
# 5. 팀원 +각도 명령 시 예상 raw Position 변화 방향
# ============================================================
# +1 : raw Position 증가
# -1 : raw Position 감소
#
# shoulder_lift / elbow_flex / wrist_flex의 TEAM +는 raw 감소,
# wrist_roll의 TEAM +는 raw 증가가 되어야 한다.

EXPECTED_RAW_SIGN_FOR_POSITIVE_COMMAND = {
    "shoulder_lift": -1,
    "elbow_flex": -1,
    "wrist_flex": -1,
    "wrist_roll": +1,
}


# ============================================================
# 6. 제어 기본값
# ============================================================

# acc는 선택 인자이며 생략하면 10을 사용한다.
DEFAULT_ACC = 10
MIN_ACC = 0
MAX_ACC = 254

DEFAULT_WAIT = True
DEFAULT_TIMEOUT_SEC = 5.0

# wait=True에서 목표 도착으로 인정할 Position 오차 범위
POSITION_TOLERANCE = 20
POLL_INTERVAL_SEC = 0.05


# ============================================================
# 7. Stop / Torque 설정
# ============================================================
# STS3215 계열 Torque Enable SRAM 주소.
# 0 = Torque OFF
# 1 = Torque ON
#
# emergency_stop()
#   -> 기존 개발/점검용 전체 Torque OFF 기능. 기존 API/동작 유지.
#
# user_stop()
#   -> Torque를 건드리지 않고 현재 Position을 Goal로 다시 써서 자세를 Hold한다.
#      아래 Hold Speed/Acc는 ID3 wrist_flex 단축 하드웨어 후보 테스트에서
#      speed=80, acc=10으로 정상 정지/유지를 확인한 값을 기본으로 사용한다.
#      최종 사용자 안전 기능 승인 전에는 4축 동시/부하 조건에서 별도 실물 검증이 필요하다.

ADDR_TORQUE_ENABLE = 40
TORQUE_OFF = 0
TORQUE_ON = 1

USER_STOP_HOLD_SPEED = 80
USER_STOP_HOLD_ACC = DEFAULT_ACC

import json
from pathlib import Path
from typing import Any, Dict


# ------------------------------------------------------------
# 기준 스키마 로드
# ------------------------------------------------------------

SCHEMA_PATH = Path(__file__).resolve().parent / "log_schema.json"


def get_default_log_schema() -> Dict[str, Any]:
    """
    log_schema.json을 읽지 못했을 때 사용하는 기본 스키마다.
    실제 프로젝트 기준은 log_schema.json을 우선으로 사용한다.
    """
    return {
        "required_columns": [
            "timestamp",
            "elapsed_sec",
            "posture_type",
            "fatigue_label",
            "fatigue_probability"
        ],
        "numeric_columns": [
            "elapsed_sec",
            "fatigue_probability"
        ],
        "bool_columns": [],
        "allowed_posture_types": [
            "Optimal",
            "Forward Head",
            "Chin Propping",
            "Asymmetric"
        ],
        "good_posture_label": "Optimal",
        "bad_posture_labels": [
            "Forward Head",
            "Chin Propping",
            "Asymmetric"
        ],
        "allowed_fatigue_labels": [
            "Normal",
            "Drowsy"
        ],
        "normal_fatigue_label": "Normal",
        "drowsy_label": "Drowsy",
        "posture_display_names": {
            "Optimal": "정자세",
            "Forward Head": "거북목",
            "Chin Propping": "턱괴기",
            "Asymmetric": "비대칭"
        },
        "fatigue_display_names": {
            "Normal": "정상",
            "Drowsy": "피로 경고"
        },
        "posture_colors": {
            "Optimal": "#19b87a",
            "Forward Head": "#ff7a17",
            "Chin Propping": "#f5a000",
            "Asymmetric": "#f44444"
        },
        "fatigue_colors": {
            "Normal": "#3d7ee8",
            "Drowsy": "#8b5cf6"
        },
        "posture_tips": {
            "Forward Head": "모니터와 얼굴 사이 거리를 유지하고 턱을 살짝 당겨보세요.",
            "Chin Propping": "손으로 얼굴을 받치지 말고 손을 책상 위에 내려두세요.",
            "Asymmetric": "양쪽 엉덩이에 체중을 고르게 싣고 어깨 높이를 맞춰보세요."
        },
        "default_posture_tip": "자세를 다시 바로잡고 잠깐 몸을 정렬해보세요."
    }


def load_log_schema() -> Dict[str, Any]:
    """
    프로젝트 전체에서 공유할 로그 스키마를 읽는다.
    """
    default_schema = get_default_log_schema()

    if not SCHEMA_PATH.exists():
        return default_schema

    with SCHEMA_PATH.open("r", encoding="utf-8") as f:
        loaded_schema = json.load(f)

    # 누락된 키가 있어도 기본값으로 보완한다.
    merged_schema = default_schema.copy()
    merged_schema.update(loaded_schema)

    return merged_schema


LOG_SCHEMA = load_log_schema()


# ------------------------------------------------------------
# 컬럼 기준
# ------------------------------------------------------------

REQUIRED_COLUMNS = LOG_SCHEMA["required_columns"]
NUMERIC_COLUMNS = LOG_SCHEMA["numeric_columns"]
BOOL_COLUMNS = LOG_SCHEMA["bool_columns"]


# ------------------------------------------------------------
# 자세 라벨 기준
# ------------------------------------------------------------

POSTURE_LABELS = LOG_SCHEMA["allowed_posture_types"]
GOOD_POSTURE_LABEL = LOG_SCHEMA["good_posture_label"]
BAD_POSTURE_LABELS = LOG_SCHEMA["bad_posture_labels"]

POSTURE_OPTIMAL = "Optimal"
POSTURE_FORWARD_HEAD = "Forward Head"
POSTURE_CHIN_PROPPING = "Chin Propping"
POSTURE_ASYMMETRIC = "Asymmetric"


# ------------------------------------------------------------
# 피로도 라벨 기준
# ------------------------------------------------------------

FATIGUE_LABELS = LOG_SCHEMA["allowed_fatigue_labels"]
NORMAL_FATIGUE_LABEL = LOG_SCHEMA["normal_fatigue_label"]
DROWSY_LABEL = LOG_SCHEMA["drowsy_label"]

FATIGUE_NORMAL = "Normal"
FATIGUE_DROWSY = "Drowsy"


# ------------------------------------------------------------
# 화면 표시 기준
# ------------------------------------------------------------

POSTURE_DISPLAY_NAME = LOG_SCHEMA["posture_display_names"]
FATIGUE_DISPLAY_NAME = LOG_SCHEMA["fatigue_display_names"]

POSTURE_COLOR = LOG_SCHEMA["posture_colors"]
FATIGUE_COLOR = LOG_SCHEMA["fatigue_colors"]

POSTURE_TIP = LOG_SCHEMA["posture_tips"]
DEFAULT_POSTURE_TIP = LOG_SCHEMA["default_posture_tip"]


# ------------------------------------------------------------
# 공통 헬퍼
# ------------------------------------------------------------

def get_posture_name(label: str) -> str:
    return POSTURE_DISPLAY_NAME.get(label, label)


def get_fatigue_name(label: str) -> str:
    return FATIGUE_DISPLAY_NAME.get(label, label)


def get_posture_color(label: str) -> str:
    return POSTURE_COLOR.get(label, "#9aa3af")


def get_fatigue_color(label: str) -> str:
    return FATIGUE_COLOR.get(label, "#9aa3af")


def get_posture_tip(label: str) -> str:
    return POSTURE_TIP.get(label, DEFAULT_POSTURE_TIP)

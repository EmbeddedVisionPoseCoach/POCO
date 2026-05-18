import json
from pathlib import Path
from datetime import datetime
from typing import Any, Dict, List, Optional


# ------------------------------------------------------------
# 경로 설정
# ------------------------------------------------------------

# 현재 파일:
# WorkSpace/preprocess/baseline_manager.py
#
# parents[1] = WorkSpace
BASE_DIR = Path(__file__).resolve().parents[1]

CONFIG_DIR = BASE_DIR / "data" / "config"
BASELINE_PATH = CONFIG_DIR / "baseline_config.json"


# ------------------------------------------------------------
# config_manager import
# ------------------------------------------------------------
# app.py에서 import할 때:
# from preprocess.baseline_manager import ...
#
# baseline_manager.py를 직접 실행할 때:
# python preprocess/baseline_manager.py
#
# 두 경우 모두 동작하도록 예외 처리를 둔다.
try:
    from preprocess.config_manager import mark_baseline_completed, reset_baseline_status
except ModuleNotFoundError:
    from config_manager import mark_baseline_completed, reset_baseline_status


# ------------------------------------------------------------
# 기준 자세로 저장할 feature 목록
# ------------------------------------------------------------

POSTURE_FEATURE_KEYS = [
    "forward_head_ratio",
    "chin_rest_score",
    "asymmetry_angle",
    "shoulder_angle",
    "body_tilt_angle"
]

FATIGUE_FEATURE_KEYS = [
    "eye_closed_ratio",
    "mouth_open_ratio"
]


# ------------------------------------------------------------
# 기본 유틸 함수
# ------------------------------------------------------------

def get_now_string() -> str:
    """
    현재 시간을 JSON에 저장하기 좋은 문자열 형태로 반환한다.
    """
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def ensure_config_dir() -> None:
    """
    data/config 폴더가 없으면 자동 생성한다.
    """
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> Dict[str, Any]:
    """
    JSON 파일을 읽어서 dict로 반환한다.
    """
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """
    dict 데이터를 JSON 파일로 저장한다.
    """
    ensure_config_dir()

    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)


def safe_float(value: Any) -> Optional[float]:
    """
    값을 float으로 변환한다.

    변환할 수 없는 값이면 None을 반환한다.

    예:
    "0.31" -> 0.31
    0.31   -> 0.31
    ""     -> None
    None   -> None
    """
    if value is None:
        return None

    if value == "":
        return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def calculate_average(rows: List[Dict[str, Any]], key: str) -> Optional[float]:
    """
    여러 프레임 또는 여러 로그 row에서 특정 key의 평균값을 계산한다.

    rows 예시:
    [
        {"forward_head_ratio": 0.18},
        {"forward_head_ratio": 0.20},
        {"forward_head_ratio": 0.19}
    ]

    결과:
    0.19
    """
    values = []

    for row in rows:
        value = safe_float(row.get(key))

        if value is not None:
            values.append(value)

    if len(values) == 0:
        return None

    return round(sum(values) / len(values), 4)


# ------------------------------------------------------------
# baseline 기본 구조
# ------------------------------------------------------------

def get_default_baseline_config() -> Dict[str, Any]:
    """
    baseline_config.json이 없을 때 사용할 기본 구조를 반환한다.
    """
    return {
        "user_id": "default_user",
        "baseline_created_at": None,
        "baseline_duration_sec": 5,
        "sample_count": 0,

        "posture_baseline": {
            "forward_head_ratio": None,
            "chin_rest_score": None,
            "asymmetry_angle": None,
            "shoulder_angle": None,
            "body_tilt_angle": None
        },

        "fatigue_baseline": {
            "eye_closed_ratio": None,
            "mouth_open_ratio": None
        },

        "metadata": {
            "description": "사용자의 정상 자세 기준값입니다.",
            "camera_position": "front"
        }
    }


def create_baseline_if_missing() -> Dict[str, Any]:
    """
    baseline_config.json이 없으면 기본값으로 생성한다.
    이미 있으면 기존 파일을 읽어서 반환한다.
    """
    ensure_config_dir()

    if not BASELINE_PATH.exists():
        baseline = get_default_baseline_config()
        save_json(BASELINE_PATH, baseline)
        return baseline

    return load_json(BASELINE_PATH)


def load_baseline_config() -> Dict[str, Any]:
    """
    baseline_config.json을 읽는다.
    없으면 자동 생성 후 반환한다.
    """
    return create_baseline_if_missing()


def save_baseline_config(baseline: Dict[str, Any]) -> None:
    """
    baseline_config.json을 저장한다.
    """
    save_json(BASELINE_PATH, baseline)


# ------------------------------------------------------------
# 기준 자세 생성 로직
# ------------------------------------------------------------

def build_baseline_from_rows(
    rows: List[Dict[str, Any]],
    user_id: str = "default_user",
    baseline_duration_sec: int = 5,
    camera_position: str = "front"
) -> Dict[str, Any]:
    """
    기준 자세 측정 데이터 rows를 받아서 baseline_config 구조로 변환한다.

    rows는 카메라/모델에서 일정 시간 동안 들어온 데이터라고 생각하면 된다.

    예:
    [
        {
            "forward_head_ratio": 0.18,
            "chin_rest_score": 0.05,
            "asymmetry_angle": 1.2,
            "eye_closed_ratio": 0.10,
            "mouth_open_ratio": 0.22
        },
        ...
    ]
    """

    posture_baseline = {}
    fatigue_baseline = {}

    for key in POSTURE_FEATURE_KEYS:
        posture_baseline[key] = calculate_average(rows, key)

    for key in FATIGUE_FEATURE_KEYS:
        fatigue_baseline[key] = calculate_average(rows, key)

    baseline = {
        "user_id": user_id,
        "baseline_created_at": get_now_string(),
        "baseline_duration_sec": baseline_duration_sec,
        "sample_count": len(rows),

        "posture_baseline": posture_baseline,

        "fatigue_baseline": fatigue_baseline,

        "metadata": {
            "description": "사용자의 정상 자세 기준값입니다.",
            "camera_position": camera_position
        }
    }

    return baseline


def save_baseline_from_rows(
    rows: List[Dict[str, Any]],
    user_id: str = "default_user",
    baseline_duration_sec: int = 5,
    camera_position: str = "front"
) -> Dict[str, Any]:
    """
    기준 자세 측정 rows를 받아서 평균값을 계산하고,
    baseline_config.json에 저장한다.

    저장이 끝나면 user_profile.json의 is_baseline_completed도 True로 바꾼다.
    """
    if len(rows) == 0:
        raise ValueError("기준 자세를 만들 수 없습니다. rows 데이터가 비어 있습니다.")

    baseline = build_baseline_from_rows(
        rows=rows,
        user_id=user_id,
        baseline_duration_sec=baseline_duration_sec,
        camera_position=camera_position
    )

    save_baseline_config(baseline)

    # user_profile.json 업데이트
    mark_baseline_completed()

    return baseline


def reset_baseline_config() -> Dict[str, Any]:
    """
    기준 자세를 초기화한다.

    baseline_config.json은 기본값으로 되돌리고,
    user_profile.json의 is_baseline_completed는 False로 변경한다.
    """
    baseline = get_default_baseline_config()
    save_baseline_config(baseline)

    reset_baseline_status()

    return baseline


# ------------------------------------------------------------
# 기준값 조회 함수
# ------------------------------------------------------------

def get_posture_baseline_value(key: str) -> Optional[float]:
    """
    posture_baseline 안에서 특정 기준값을 가져온다.

    예:
    get_posture_baseline_value("forward_head_ratio")
    """
    baseline = load_baseline_config()
    return baseline.get("posture_baseline", {}).get(key)


def get_fatigue_baseline_value(key: str) -> Optional[float]:
    """
    fatigue_baseline 안에서 특정 기준값을 가져온다.

    예:
    get_fatigue_baseline_value("eye_closed_ratio")
    """
    baseline = load_baseline_config()
    return baseline.get("fatigue_baseline", {}).get(key)


def is_baseline_valid() -> bool:
    """
    기준 자세값이 실제로 저장되어 있는지 확인한다.

    최소한 posture_baseline의 주요 값 중 하나라도 None이 아니면
    기준값이 있다고 판단한다.
    """
    baseline = load_baseline_config()
    posture_baseline = baseline.get("posture_baseline", {})

    for key in POSTURE_FEATURE_KEYS:
        if posture_baseline.get(key) is not None:
            return True

    return False


# ------------------------------------------------------------
# 테스트 실행
# ------------------------------------------------------------

if __name__ == "__main__":
    # 테스트용 기준 자세 데이터
    # 실제로는 카메라/모델에서 5초 동안 들어온 값들이 여기에 들어온다고 보면 된다.
    sample_rows = [
        {
            "forward_head_ratio": 0.18,
            "chin_rest_score": 0.05,
            "asymmetry_angle": 1.2,
            "shoulder_angle": 1.0,
            "body_tilt_angle": 0.8,
            "eye_closed_ratio": 0.10,
            "mouth_open_ratio": 0.22
        },
        {
            "forward_head_ratio": 0.20,
            "chin_rest_score": 0.04,
            "asymmetry_angle": 1.5,
            "shoulder_angle": 1.1,
            "body_tilt_angle": 0.7,
            "eye_closed_ratio": 0.12,
            "mouth_open_ratio": 0.24
        },
        {
            "forward_head_ratio": 0.19,
            "chin_rest_score": 0.06,
            "asymmetry_angle": 1.3,
            "shoulder_angle": 0.9,
            "body_tilt_angle": 0.9,
            "eye_closed_ratio": 0.11,
            "mouth_open_ratio": 0.23
        }
    ]

    baseline = save_baseline_from_rows(
        rows=sample_rows,
        user_id="default_user",
        baseline_duration_sec=5,
        camera_position="front"
    )

    print("기준 자세 저장 완료")
    print(json.dumps(baseline, ensure_ascii=False, indent=4))

    print("기준값 유효 여부:", is_baseline_valid())



# // ------------------------------------------------------------
# from preprocess.config_manager import is_baseline_required
# from preprocess.baseline_manager import save_baseline_from_rows


# if is_baseline_required():
#     st.warning("초기 기준 자세 설정이 필요합니다.")

#     if st.button("기준 자세 설정 시작"):
#         # 실제로는 여기서 카메라 값을 5초 동안 수집해야 함
#         baseline_rows = []

#         # 예시 데이터
#         baseline_rows.append({
#             "forward_head_ratio": 0.18,
#             "chin_rest_score": 0.05,
#             "asymmetry_angle": 1.2,
#             "shoulder_angle": 1.0,
#             "body_tilt_angle": 0.8,
#             "eye_closed_ratio": 0.10,
#             "mouth_open_ratio": 0.22
#         })

#         save_baseline_from_rows(baseline_rows)

#         st.success("기준 자세 설정이 완료되었습니다.")
# else:
#     st.success("기준 자세 설정이 완료되어 있습니다.")
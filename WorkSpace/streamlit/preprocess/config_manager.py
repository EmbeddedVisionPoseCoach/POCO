import json
from pathlib import Path
from datetime import datetime


# 현재 파일 위치 기준:
# WorkSpace/preprocess/config_manager.py
# parents[1] = WorkSpace

## 현재 실행 중인 파일의 절대 경로를 기준으로 
# 2단계 상위 디렉토리(상위-상위 폴더)를 가리키는 코드

BASE_DIR = Path(__file__).resolve().parents[1]

CONFIG_DIR = BASE_DIR / "data" / "config"
USER_PROFILE_PATH = CONFIG_DIR / "user_profile.json"

STREAMLIT_APP_PATH = "streamlit/app.py"
STREAMLIT_PORT = 8501


def get_now_string() -> str:
    """
    현재 시간을 문자열로 반환한다.
    JSON에 저장하기 좋은 형태로 사용한다.
    """
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def get_default_user_profile() -> dict:
    """
    user_profile.json이 없을 때 사용할 기본 사용자 설정값을 반환한다.
    """
    return {
        "user_id": "default_user",
        "user_name": "사용자",
        "created_at": get_now_string(),
        "height_cm": None,
        "camera_position": "front",
        "is_baseline_completed": False,
        "baseline_file": "baseline_config.json"
    }


def ensure_config_dir() -> None:
    """
    data/config 폴더가 없으면 자동으로 생성한다.
    """
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> dict:
    """
    JSON 파일을 읽어서 dict로 반환한다.
    파일이 없으면 FileNotFoundError가 발생한다.
    """
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data: dict) -> None:
    """
    dict 데이터를 JSON 파일로 저장한다.
    한글이 깨지지 않도록 ensure_ascii=False를 사용한다.
    """
    ensure_config_dir()

    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)


def create_user_profile_if_missing() -> dict:
    """
    user_profile.json이 없으면 기본값으로 생성한다.
    이미 있으면 기존 파일을 읽어서 반환한다.
    """
    ensure_config_dir()

    if not USER_PROFILE_PATH.exists():
        default_profile = get_default_user_profile()
        save_json(USER_PROFILE_PATH, default_profile)
        return default_profile

    return load_json(USER_PROFILE_PATH)


def load_user_profile() -> dict:
    """
    user_profile.json을 읽는다.
    파일이 없으면 자동으로 생성 후 반환한다.
    """
    return create_user_profile_if_missing()


def save_user_profile(profile: dict) -> None:
    """
    user_profile 데이터를 저장한다.
    """
    save_json(USER_PROFILE_PATH, profile)


def is_baseline_required() -> bool:
    """
    기준 자세 설정이 필요한지 확인한다.

    반환값:
    True  -> 기준 자세 설정 필요
    False -> 기준 자세 설정 완료됨
    """
    profile = load_user_profile()
    return not profile.get("is_baseline_completed", False)


def mark_baseline_completed() -> None:
    """
    기준 자세 설정이 완료되었을 때 호출한다.
    user_profile.json의 is_baseline_completed 값을 True로 변경한다.
    """
    profile = load_user_profile()
    profile["is_baseline_completed"] = True
    profile["baseline_completed_at"] = get_now_string()
    save_user_profile(profile)


def reset_baseline_status() -> None:
    """
    기준 자세를 다시 설정해야 할 때 사용한다.
    예를 들어 사용자가 '초기 기준 다시 설정' 버튼을 눌렀을 때 호출한다.
    """
    profile = load_user_profile()
    profile["is_baseline_completed"] = False
    profile["baseline_completed_at"] = None
    save_user_profile(profile)


if __name__ == "__main__":
    profile = load_user_profile()

    print("현재 사용자 설정:")
    print(profile)

    if is_baseline_required():
        print("기준 자세 설정이 필요합니다.")
    else:
        print("기준 자세 설정이 완료되어 있습니다.")
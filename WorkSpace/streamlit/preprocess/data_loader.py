from pathlib import Path
from typing import Optional

import pandas as pd


# ------------------------------------------------------------
# 경로 설정
# ------------------------------------------------------------

# 현재 파일:
# WorkSpace/streamlit/preprocess/data_loader.py
#
# parents[2] = WorkSpace
BASE_DIR = Path(__file__).resolve().parents[2]

LOG_DIR = BASE_DIR / "data" / "session_log"
# 구버전 단일 로그 파일 호환용 fallback 경로
DEFAULT_LOG_PATH = LOG_DIR / "posture_log.csv"


# ------------------------------------------------------------
# validator / schema import
# ------------------------------------------------------------

try:
    from .data_validator import raise_if_invalid
    from .domain_schema import NUMERIC_COLUMNS, BOOL_COLUMNS
except ImportError:
    from data_validator import raise_if_invalid
    from domain_schema import NUMERIC_COLUMNS, BOOL_COLUMNS


def parse_bool(value) -> bool:
    """
    CSV에서 읽은 값을 bool로 변환한다.

    예:
    "True"  -> True
    "False" -> False
    1       -> True
    0       -> False
    """
    if value in [True, "True", "true", "TRUE", "1", 1]:
        return True

    if value in [False, "False", "false", "FALSE", "0", 0]:
        return False

    raise ValueError(f"bool로 변환할 수 없는 값입니다: {value}")


def clean_log_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    """
    CSV 원본 DataFrame에서 공백/중복 데이터를 정리한다.

    - 컬럼명 앞뒤 공백 제거
    - posture_type / fatigue_label 값 앞뒤 공백 제거
    - 완전히 같은 row 제거
    """
    result = df.copy()

    result.columns = result.columns.str.strip()

    if "posture_type" in result.columns:
        result["posture_type"] = result["posture_type"].astype(str).str.strip()

    if "fatigue_label" in result.columns:
        result["fatigue_label"] = result["fatigue_label"].astype(str).str.strip()

    result = result.drop_duplicates().reset_index(drop=True)

    return result


def remove_duplicate_rows(df: pd.DataFrame) -> pd.DataFrame:
    """
    중복으로 쌓인 로그를 제거한다.

    주의:
    날짜별 CSV 안에 여러 측정 세션이 들어갈 수 있으므로
    elapsed_sec 기준으로 중복 제거하면 안 된다.

    예:
    10:59:47, 0, Optimal
    11:40:15, 0, Optimal

    위 두 row는 서로 다른 세션의 시작값이므로 둘 다 살아 있어야 한다.

    처리 기준:
    - 완전히 같은 row는 clean_log_dataframe()에서 이미 제거
    - 같은 timestamp에 여러 row가 찍힌 경우만 마지막 값을 사용
    - 전체 정렬은 timestamp 기준
    """
    result = df.copy()

    if "timestamp" in result.columns:
        result = result.sort_values("timestamp")
        result = result.drop_duplicates(subset=["timestamp"], keep="last")

    return result.reset_index(drop=True)


def load_posture_log(csv_path: Optional[str | Path] = None) -> pd.DataFrame:
    """
    posture_log.csv를 읽고 전처리된 DataFrame으로 반환한다.

    처리 순서:
    1. CSV 읽기
    2. 컬럼명/라벨값 앞뒤 공백 제거
    3. 완전 중복 row 제거
    4. 데이터 검증
    5. timestamp datetime 변환
    6. 숫자 컬럼 numeric 변환
    7. bool 컬럼 변환
    8. timestamp 기준 중복 제거
    9. timestamp 기준 정렬
    """

    if csv_path is None:
        # 날짜별 로그가 있으면 가장 최근 파일을 기본값으로 사용한다.
        # Streamlit의 날짜 선택 로직에서는 항상 선택된 csv_path를 직접 넘긴다.
        dated_logs = sorted(LOG_DIR.glob("posture_log_*.csv"), reverse=True)
        csv_path = dated_logs[0] if dated_logs else DEFAULT_LOG_PATH

    csv_path = Path(csv_path)

    if not csv_path.exists():
        raise FileNotFoundError(f"로그 파일을 찾을 수 없습니다: {csv_path}")

    df = pd.read_csv(csv_path)
    df = clean_log_dataframe(df)

    # 정리된 상태에서 검증
    raise_if_invalid(df)

    # timestamp 변환
    df["timestamp"] = pd.to_datetime(df["timestamp"])

    # 숫자 컬럼 변환
    for column in NUMERIC_COLUMNS:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column])

    # bool 컬럼 변환
    for column in BOOL_COLUMNS:
        if column in df.columns:
            df[column] = df[column].apply(parse_bool)

    # 기존 스펙 호환: yawn_detected가 남아 있을 때만 변환
    if "yawn_detected" in df.columns:
        df["yawn_detected"] = df["yawn_detected"].apply(parse_bool)

    # timestamp 기준 중복 제거
    df = remove_duplicate_rows(df)

    # timestamp 기준 정렬
    df = df.sort_values("timestamp").reset_index(drop=True)

    return df


if __name__ == "__main__":
    log_df = load_posture_log()

    print("로그 로드 성공")
    print("--------------------")
    print(log_df.head())
    print("--------------------")
    print(log_df.dtypes)
    print("--------------------")
    print(f"총 row 수: {len(log_df)}")

from datetime import datetime
from typing import Any, Dict, List, Optional

import pandas as pd


# ------------------------------------------------------------
# data_loader import
# ------------------------------------------------------------

try:
    from workspace.preprocess.data_loader import load_posture_log
except ModuleNotFoundError:
    try:
        from .data_loader import load_posture_log
    except ImportError:
        from data_loader import load_posture_log

try:
    from .domain_schema import (
        GOOD_POSTURE_LABEL,
        DROWSY_LABEL,
        NORMAL_FATIGUE_LABEL,
    )
except ImportError:
    from domain_schema import (
        GOOD_POSTURE_LABEL,
        DROWSY_LABEL,
        NORMAL_FATIGUE_LABEL,
    )


# ------------------------------------------------------------
# 기본 설정
# ------------------------------------------------------------

# 현재 로그는 1초에 1row가 들어온다고 가정
LOG_INTERVAL_SEC = 1
SESSION_GAP_THRESHOLD_SEC = 5

# 현재는 실제 점수 컬럼이 없으므로 임시 리터럴 값 사용
# 나중에 CSV에 posture_score, fatigue_score가 들어오면 자동으로 실제 평균값 사용
PLACEHOLDER_POSTURE_SCORE = 82
PLACEHOLDER_FATIGUE_SCORE = 76


# ------------------------------------------------------------
# 공통 유틸
# ------------------------------------------------------------

def get_now_string() -> str:
    """
    현재 시간을 문자열로 반환한다.
    report_summary 생성 시간을 기록할 때 사용한다.
    """
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def safe_round(value: Any, digit: int = 4) -> Optional[float]:
    """
    숫자를 안전하게 반올림한다.
    NaN이면 None을 반환한다.
    """
    if value is None:
        return None

    if pd.isna(value):
        return None

    return round(float(value), digit)


def safe_ratio(part: float, total: float, digit: int = 4) -> float:
    """
    비율을 안전하게 계산한다.
    total이 0이면 0을 반환한다.
    """
    if total == 0:
        return 0.0

    return round(part / total, digit)


def seconds_to_text(seconds: int) -> str:
    """
    초 단위를 화면 표시용 문자열로 변환한다.

    예:
    65 -> "1분 5초"
    3600 -> "1시간 0분 0초"
    """
    seconds = int(seconds)

    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    remain_seconds = seconds % 60

    if hours > 0:
        return f"{hours}시간 {minutes}분 {remain_seconds}초"

    if minutes > 0:
        return f"{minutes}분 {remain_seconds}초"

    return f"{remain_seconds}초"


def get_numeric_series(df: pd.DataFrame, column: str) -> pd.Series:
    """
    특정 컬럼을 숫자 Series로 변환한다.
    변환 불가능한 값은 NaN으로 처리하고 제거한다.
    """
    if column not in df.columns:
        return pd.Series(dtype="float64")

    return pd.to_numeric(df[column], errors="coerce").dropna()


def get_most_common_issue(
    seconds_dict: Dict[str, int],
    exclude_labels: Optional[List[str]] = None
) -> Optional[str]:
    """
    특정 라벨을 제외하고 가장 많이 등장한 라벨을 반환한다.

    예:
    posture_type_seconds에서 좋은 자세 라벨을 제외하고
    가장 많이 나온 자세 문제를 찾을 때 사용한다.
    """
    if exclude_labels is None:
        exclude_labels = []

    filtered = {
        label: seconds
        for label, seconds in seconds_dict.items()
        if label not in exclude_labels and seconds > 0
    }

    if not filtered:
        return None

    return max(filtered, key=filtered.get)


def build_top_items(
    seconds_dict: Dict[str, int],
    total_sec: int,
    exclude_labels: Optional[List[str]] = None,
    top_n: int = 3
) -> List[Dict[str, Any]]:
    """
    누적 시간이 많은 라벨 TOP N을 만든다.
    """
    if exclude_labels is None:
        exclude_labels = []

    items = []

    for label, seconds in seconds_dict.items():
        if label in exclude_labels:
            continue

        if seconds <= 0:
            continue

        items.append({
            "label": label,
            "seconds": int(seconds),
            "time_text": seconds_to_text(int(seconds)),
            "ratio": safe_ratio(seconds, total_sec)
        })

    items.sort(key=lambda item: item["seconds"], reverse=True)

    return items[:top_n]


def safe_mean_column(df: pd.DataFrame, column: str, default: float = 0.0, digit: int = 4) -> float:
    if column not in df.columns:
        return default

    series = pd.to_numeric(df[column], errors="coerce").dropna()

    if series.empty:
        return default

    return round(float(series.mean()), digit)


def safe_max_column(df: pd.DataFrame, column: str, default: float = 0.0, digit: int = 4) -> float:
    if column not in df.columns:
        return default

    series = pd.to_numeric(df[column], errors="coerce").dropna()

    if series.empty:
        return default

    return round(float(series.max()), digit)


def safe_sum_column(df: pd.DataFrame, column: str, default: int = 0) -> int:
    if column not in df.columns:
        return default

    series = pd.to_numeric(df[column], errors="coerce").fillna(0)

    return int(series.sum())

# ------------------------------------------------------------
# 점수 요약
# ------------------------------------------------------------

def build_score_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """
    점수 상세 탭에서 사용할 자세 점수 / 피로도 점수 요약을 만든다.

    현재는 posture_score, fatigue_score 컬럼이 없으므로 placeholder 값을 사용한다.
    나중에 CSV에 posture_score, fatigue_score 컬럼이 추가되면 자동으로 실제 값을 사용한다.
    """

    posture_scores = get_numeric_series(df, "posture_score")

    if len(posture_scores) > 0:
        posture_score = safe_round(posture_scores.mean(), digit=1)
        max_posture_score = safe_round(posture_scores.max(), digit=1)
        min_posture_score = safe_round(posture_scores.min(), digit=1)
        posture_score_source = "log"
    else:
        posture_score = PLACEHOLDER_POSTURE_SCORE
        max_posture_score = PLACEHOLDER_POSTURE_SCORE
        min_posture_score = PLACEHOLDER_POSTURE_SCORE
        posture_score_source = "placeholder"

    fatigue_scores = get_numeric_series(df, "fatigue_score")

    if len(fatigue_scores) > 0:
        fatigue_score = safe_round(fatigue_scores.mean(), digit=1)
        max_fatigue_score = safe_round(fatigue_scores.max(), digit=1)
        min_fatigue_score = safe_round(fatigue_scores.min(), digit=1)
        fatigue_score_source = "log"
    else:
        fatigue_score = PLACEHOLDER_FATIGUE_SCORE
        max_fatigue_score = PLACEHOLDER_FATIGUE_SCORE
        min_fatigue_score = PLACEHOLDER_FATIGUE_SCORE
        fatigue_score_source = "placeholder"

    return {
        "posture_score": posture_score,
        "max_posture_score": max_posture_score,
        "min_posture_score": min_posture_score,
        "posture_score_source": posture_score_source,

        "fatigue_score": fatigue_score,
        "max_fatigue_score": max_fatigue_score,
        "min_fatigue_score": min_fatigue_score,
        "fatigue_score_source": fatigue_score_source
    }


# ------------------------------------------------------------
# 자세 요약
# ------------------------------------------------------------

def build_posture_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """
    자세 관련 요약 데이터를 만든다.

    생성 항목:
    - 좋은 자세 시간
    - 나쁜 자세 시간
    - 좋은 자세 비율
    - 나쁜 자세 비율
    - 자세별 누적 시간
    - 가장 많이 나온 문제 자세
    - TOP 3 자세 문제
    - 자세 feature 평균/최대값
    """

    total_logged_sec = len(df) * LOG_INTERVAL_SEC

    posture_counts = df["posture_type"].value_counts().to_dict()

    posture_type_seconds = {
        posture_type: int(count * LOG_INTERVAL_SEC)
        for posture_type, count in posture_counts.items()
    }

    good_sec = posture_type_seconds.get(GOOD_POSTURE_LABEL, 0)
    bad_sec = max(total_logged_sec - good_sec, 0)

    most_common_posture = get_most_common_issue(
        seconds_dict=posture_type_seconds,
        exclude_labels=[GOOD_POSTURE_LABEL]
    )

    top_posture_issues = build_top_items(
        seconds_dict=posture_type_seconds,
        total_sec=total_logged_sec,
        exclude_labels=[GOOD_POSTURE_LABEL],
        top_n=3
    )

    return {
        "good_sec": int(good_sec),
        "good_time_text": seconds_to_text(good_sec),

        "bad_sec": int(bad_sec),
        "bad_time_text": seconds_to_text(bad_sec),

        "good_ratio": safe_ratio(good_sec, total_logged_sec),
        "bad_ratio": safe_ratio(bad_sec, total_logged_sec),

        "most_common_posture": most_common_posture,

        "posture_type_seconds": posture_type_seconds,
        "top_posture_issues": top_posture_issues,

        "metric_summary": {}
    }


# ------------------------------------------------------------
# 피로도 요약
# ------------------------------------------------------------

def build_fatigue_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """
    피로도 관련 요약 데이터를 만든다.

    새 스펙 기준:
    - fatigue_label
    - fatigue_probability
    """

    total_logged_sec = len(df) * LOG_INTERVAL_SEC

    fatigue_counts = df["fatigue_label"].value_counts().to_dict()

    fatigue_label_seconds = {
        fatigue_label: int(count * LOG_INTERVAL_SEC)
        for fatigue_label, count in fatigue_counts.items()
    }

    normal_sec = fatigue_label_seconds.get(NORMAL_FATIGUE_LABEL, 0)
    drowsy_sec = fatigue_label_seconds.get(DROWSY_LABEL, 0)

    avg_fatigue_probability = safe_mean_column(df, "fatigue_probability", default=0.0)
    max_fatigue_probability = safe_max_column(df, "fatigue_probability", default=0.0)

    return {
        "normal_sec": int(normal_sec),
        "normal_time_text": seconds_to_text(normal_sec),

        "drowsy_sec": int(drowsy_sec),
        "drowsy_time_text": seconds_to_text(drowsy_sec),

        "drowsy_ratio": safe_ratio(drowsy_sec, total_logged_sec),

        "fatigue_label_seconds": fatigue_label_seconds,

        # 새 스펙에서는 하품/눈감김/입벌림 데이터가 없으므로 기본값
        "total_yawn_count": 0,
        "yawn_detected_sec": 0,
        "yawn_detected_time_text": "0초",

        "max_eye_closed_duration": 0.0,
        "avg_eye_closed_ratio": 0.0,
        "avg_mouth_open_ratio": 0.0,

        "avg_fatigue_probability": avg_fatigue_probability,
        "max_fatigue_probability": max_fatigue_probability,

        "metric_summary": {
            "avg_fatigue_probability": avg_fatigue_probability,
            "max_fatigue_probability": max_fatigue_probability
        }
    }


# ------------------------------------------------------------
# 분 단위 요약
# ------------------------------------------------------------

def build_minute_summary(df: pd.DataFrame) -> List[Dict[str, Any]]:
    """
    1분 단위 요약 데이터를 만든다.

    timestamp 기준으로 1분 단위 요약을 만든다.
    날짜별 CSV 안에 여러 측정 구간이 있어도 elapsed_sec 리셋 문제를 피할 수 있다.
    """

    temp_df = df.copy()

    temp_df["timestamp"] = pd.to_datetime(temp_df["timestamp"], errors="coerce")
    temp_df = temp_df.dropna(subset=["timestamp"])
    temp_df = temp_df.sort_values("timestamp").reset_index(drop=True)

    if temp_df.empty:
        return []

    # 이전 로그와의 시간 차이 계산
    temp_df["time_diff_sec"] = temp_df["timestamp"].diff().dt.total_seconds().fillna(0)

    # 일정 시간 이상 비면 새로운 측정 구간으로 판단
    temp_df["segment_index"] = (
        temp_df["time_diff_sec"] > SESSION_GAP_THRESHOLD_SEC
    ).cumsum().astype(int)

    # 전체 날짜 기준 minute_index
    session_start_time = temp_df["timestamp"].min()
    temp_df["minute_index"] = (
        (temp_df["timestamp"] - session_start_time)
        .dt.total_seconds()
        // 60
    ).astype(int)

    minute_summaries = []

    grouped = temp_df.groupby(["segment_index", "minute_index"])

    for (segment_index, minute_index), group in grouped:
        logged_sec = len(group) * LOG_INTERVAL_SEC

        good_sec = int((group["posture_type"] == GOOD_POSTURE_LABEL).sum() * LOG_INTERVAL_SEC)
        bad_sec = max(logged_sec - good_sec, 0)

        drowsy_sec = int((group["fatigue_label"] == DROWSY_LABEL).sum() * LOG_INTERVAL_SEC)

        posture_scores = get_numeric_series(group, "posture_score")
        fatigue_scores = get_numeric_series(group, "fatigue_score")

        if len(posture_scores) > 0:
            avg_posture_score = safe_round(posture_scores.mean(), digit=1)
        else:
            avg_posture_score = PLACEHOLDER_POSTURE_SCORE

        if len(fatigue_scores) > 0:
            avg_fatigue_score = safe_round(fatigue_scores.mean(), digit=1)
        else:
            avg_fatigue_score = PLACEHOLDER_FATIGUE_SCORE

        minute_summary = {
            "segment_index": int(segment_index),
            "minute_index": int(minute_index),
            "segment_start_time": str(group["timestamp"].min()),
            "segment_end_time": str(group["timestamp"].max()),

            "logged_sec": int(logged_sec),

            "good_sec": good_sec,
            "bad_sec": bad_sec,
            "good_ratio": safe_ratio(good_sec, logged_sec),
            "bad_ratio": safe_ratio(bad_sec, logged_sec),

            "drowsy_sec": drowsy_sec,
            "drowsy_ratio": safe_ratio(drowsy_sec, logged_sec),

            "avg_posture_score": avg_posture_score,
            "avg_fatigue_score": avg_fatigue_score,

            "avg_fatigue_probability": safe_mean_column(
                group,
                "fatigue_probability",
                default=0.0
            )
        }

        minute_summaries.append(minute_summary)

    return minute_summaries


# ------------------------------------------------------------
# 전체 리포트 요약 생성
# ------------------------------------------------------------

def build_report_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """
    전체 report_summary를 생성한다.

    이 함수 결과가 report_summary.json으로 저장되고,
    Streamlit에서 탭별 화면 표시용으로 사용된다.
    """

    if df.empty:
        raise ValueError("요약을 생성할 수 없습니다. 로그 데이터가 비어 있습니다.")

    start_time = df["timestamp"].min()
    end_time = df["timestamp"].max()

    # 시작 timestamp부터 마지막 timestamp까지의 실제 시각 차이
    total_span_sec = int((end_time - start_time).total_seconds())

    # 실제 측정 로그가 찍힌 누적 시간
    # 현재는 1초에 1row라고 가정한다.
    total_logged_sec = int(len(df) * LOG_INTERVAL_SEC)

    # 기존 키 호환용:
    # 하루 CSV에 여러 세션이 들어갈 수 있으므로 elapsed_sec.max()를 쓰지 않는다.
    total_elapsed_sec = total_logged_sec

    score_summary = build_score_summary(df)
    posture_summary = build_posture_summary(df)
    fatigue_summary = build_fatigue_summary(df)
    minute_summary = build_minute_summary(df)

    report_summary = {
        "generated_at": get_now_string(),

        "session": {
            "start_time": str(start_time),
            "end_time": str(end_time),

            "total_elapsed_sec": total_elapsed_sec,
            "total_elapsed_time_text": seconds_to_text(total_elapsed_sec),

            "total_logged_sec": total_logged_sec,
            "total_logged_time_text": seconds_to_text(total_logged_sec),

            "total_span_sec": total_span_sec,
            "total_span_time_text": seconds_to_text(total_span_sec),

            "log_interval_sec": LOG_INTERVAL_SEC,
            "row_count": int(len(df))
        },

        "score_summary": score_summary,

        "posture_summary": posture_summary,

        "fatigue_summary": fatigue_summary,

        "minute_summary": minute_summary
    }

    return report_summary


# ------------------------------------------------------------
# 테스트 실행
# ------------------------------------------------------------

if __name__ == "__main__":
    import json

    df = load_posture_log()
    summary = build_report_summary(df)

    print("리포트 요약 생성 성공")
    print("--------------------")
    print(json.dumps(summary, ensure_ascii=False, indent=4))
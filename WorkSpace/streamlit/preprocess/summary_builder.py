from datetime import datetime
from typing import Any, Dict, List, Optional

import pandas as pd


# ------------------------------------------------------------
# data_loader import
# ------------------------------------------------------------

try:
    from preprocess.data_loader import load_posture_log
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
        POSTURE_FORWARD_HEAD,
        POSTURE_CHIN_PROPPING,
        POSTURE_ASYMMETRIC,
    )
except ImportError:
    try:
        from domain_schema import (
            GOOD_POSTURE_LABEL,
            DROWSY_LABEL,
            NORMAL_FATIGUE_LABEL,
            POSTURE_FORWARD_HEAD,
            POSTURE_CHIN_PROPPING,
            POSTURE_ASYMMETRIC,
        )
    except ImportError:
        # domain_schema import가 실패해도 테스트가 가능하도록 최소 fallback
        GOOD_POSTURE_LABEL = "Optimal"
        DROWSY_LABEL = "Drowsy"
        NORMAL_FATIGUE_LABEL = "Normal"
        POSTURE_FORWARD_HEAD = "ForwardHead"
        POSTURE_CHIN_PROPPING = "ChinPropping"
        POSTURE_ASYMMETRIC = "Asymmetric"


# ------------------------------------------------------------
# 기본 설정
# ------------------------------------------------------------

# 현재 로그는 1초에 1row가 들어온다고 가정
LOG_INTERVAL_SEC = 1
SESSION_GAP_THRESHOLD_SEC = 5

# 정자세 라벨 호환 처리
# 프로젝트 중간에 Good / Optimal이 섞여도 같은 정자세로 처리한다.
GOOD_POSTURE_ALIASES = {
    str(GOOD_POSTURE_LABEL),
    "Good",
    "Optimal",
}

# 계산에서 제외할 비정상 라벨
INVALID_LABELS = {
    "",
    "-",
    "None",
    "nan",
    "NaN",
    "Unknown",
}

# 자세 감점 가중치
# 점수 계산식:
# posture_score = 100 - sum(나쁜 자세 비율 * 자세별 가중치 * 100)
POSTURE_PENALTY_WEIGHTS = {
    str(POSTURE_FORWARD_HEAD): 0.8,
    "ForwardHead": 0.8,
    "Forward Head": 0.8,

    str(POSTURE_CHIN_PROPPING): 1.0,
    "ChinPropping": 1.0,
    "ChinRest": 1.0,
    "Chin Propping": 1.0,
    "Chin Rest": 1.0,

    str(POSTURE_ASYMMETRIC): 0.9,
    "Asymmetric": 0.9,
    "Asymmetry": 0.9,
    "ShoulderImbalance": 0.9,
    "Shoulder Imbalance": 0.9,
}

# 혹시 새로운 나쁜 자세 라벨이 추가됐는데
# 가중치를 아직 등록하지 않았을 때 사용할 기본 감점
DEFAULT_BAD_POSTURE_WEIGHT = 0.8


# ------------------------------------------------------------
# 공통 유틸
# ------------------------------------------------------------

def get_now_string() -> str:
    """
    현재 시간을 문자열로 반환한다.
    report_summary 생성 시간을 기록할 때 사용한다.
    """
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def clamp_score(score: float) -> float:
    """
    점수를 0~100 사이로 제한한다.
    """
    if score is None:
        return 0.0

    return max(0.0, min(100.0, float(score)))


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
    if total <= 0:
        return 0.0

    return round(float(part) / float(total), digit)


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


def normalize_label(label: Any) -> str:
    """
    라벨 값을 안전한 문자열로 변환한다.
    """
    if label is None:
        return ""

    return str(label).strip()


def is_invalid_label(label: Any) -> bool:
    return normalize_label(label) in INVALID_LABELS


def is_good_posture_label(label: Any) -> bool:
    return normalize_label(label) in GOOD_POSTURE_ALIASES


def is_drowsy_label(label: Any) -> bool:
    return normalize_label(label) in {str(DROWSY_LABEL), "Drowsy"}


def count_seconds_by_label(df: pd.DataFrame, column: str) -> Dict[str, int]:
    """
    특정 라벨 컬럼의 등장 횟수를 초 단위로 변환한다.
    현재는 1row = 1초 기준이다.
    """
    if column not in df.columns:
        return {}

    counts = df[column].apply(normalize_label).value_counts().to_dict()

    return {
        str(label): int(count * LOG_INTERVAL_SEC)
        for label, count in counts.items()
        if not is_invalid_label(label)
    }


def sum_seconds_by_labels(seconds_dict: Dict[str, int], labels: set[str]) -> int:
    total = 0

    for label, seconds in seconds_dict.items():
        if normalize_label(label) in labels:
            total += int(seconds)

    return total


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

    exclude_set = {normalize_label(label) for label in exclude_labels}

    filtered = {
        label: seconds
        for label, seconds in seconds_dict.items()
        if normalize_label(label) not in exclude_set
        and not is_good_posture_label(label)
        and not is_invalid_label(label)
        and seconds > 0
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

    exclude_set = {normalize_label(label) for label in exclude_labels}

    items = []

    for label, seconds in seconds_dict.items():
        label_text = normalize_label(label)

        if label_text in exclude_set:
            continue

        if is_good_posture_label(label_text):
            continue

        if is_invalid_label(label_text):
            continue

        if seconds <= 0:
            continue

        items.append({
            "label": label_text,
            "seconds": int(seconds),
            "time_text": seconds_to_text(int(seconds)),
            "ratio": safe_ratio(seconds, total_sec)
        })

    items.sort(key=lambda item: item["seconds"], reverse=True)

    return items[:top_n]


# ------------------------------------------------------------
# 점수 계산
# ------------------------------------------------------------

def calculate_posture_score(posture_type_seconds: Dict[str, int], total_logged_sec: int) -> float:
    """
    자세 점수 계산.

    기본 철학:
    - 정자세는 감점 없음
    - 나쁜 자세는 누적 비율에 따라 감점
    - 자세 유형별로 감점 가중치를 다르게 적용

    예:
    ForwardHead 20%, weight 0.8 -> 16점 감점
    ChinPropping 10%, weight 1.0 -> 10점 감점
    """

    if total_logged_sec <= 0:
        return 0.0

    penalty = 0.0

    for label, seconds in posture_type_seconds.items():
        label_text = normalize_label(label)

        if is_invalid_label(label_text):
            continue

        if is_good_posture_label(label_text):
            continue

        weight = POSTURE_PENALTY_WEIGHTS.get(label_text, DEFAULT_BAD_POSTURE_WEIGHT)
        ratio = int(seconds) / total_logged_sec

        penalty += ratio * weight * 100

    score = 100 - penalty

    return round(clamp_score(score), 1)


def calculate_fatigue_score(fatigue_label_seconds: Dict[str, int], total_logged_sec: int) -> float:
    """
    피로도 점수 계산.

    현재 피로도 라벨은 Normal / Drowsy 두 단계이므로
    Drowsy 누적 비율만큼 감점한다.

    fatigue_score = 100 - drowsy_ratio * 100
    """

    if total_logged_sec <= 0:
        return 0.0

    drowsy_sec = 0

    for label, seconds in fatigue_label_seconds.items():
        if is_drowsy_label(label):
            drowsy_sec += int(seconds)

    drowsy_ratio = drowsy_sec / total_logged_sec
    score = 100 - (drowsy_ratio * 100)

    return round(clamp_score(score), 1)


def get_posture_grade(score: float) -> str:
    if score >= 90:
        return "매우 좋음"

    if score >= 80:
        return "좋음"

    if score >= 70:
        return "주의"

    if score >= 60:
        return "나쁨"

    return "매우 나쁨"


def get_fatigue_grade(score: float) -> str:
    if score >= 90:
        return "매우 안정"

    if score >= 80:
        return "양호"

    if score >= 70:
        return "주의"

    if score >= 60:
        return "피로 누적"

    return "휴식 필요"


# ------------------------------------------------------------
# 점수 요약
# ------------------------------------------------------------

def build_score_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """
    점수 상세 탭에서 사용할 자세 점수 / 피로도 점수 요약을 만든다.

    CSV에 posture_score / fatigue_score 컬럼이 있으면 실제 평균값을 사용하고,
    없으면 posture_type / fatigue_label 누적 비율 기반으로 계산한다.
    """

    total_logged_sec = len(df) * LOG_INTERVAL_SEC

    posture_type_seconds = count_seconds_by_label(df, "posture_type")
    fatigue_label_seconds = count_seconds_by_label(df, "fatigue_label")

    posture_scores = get_numeric_series(df, "posture_score")

    if len(posture_scores) > 0:
        posture_score = safe_round(posture_scores.mean(), digit=1)
        max_posture_score = safe_round(posture_scores.max(), digit=1)
        min_posture_score = safe_round(posture_scores.min(), digit=1)
        posture_score_source = "log"
    else:
        posture_score = calculate_posture_score(posture_type_seconds, total_logged_sec)
        max_posture_score = posture_score
        min_posture_score = posture_score
        posture_score_source = "calculated"

    fatigue_scores = get_numeric_series(df, "fatigue_score")

    if len(fatigue_scores) > 0:
        fatigue_score = safe_round(fatigue_scores.mean(), digit=1)
        max_fatigue_score = safe_round(fatigue_scores.max(), digit=1)
        min_fatigue_score = safe_round(fatigue_scores.min(), digit=1)
        fatigue_score_source = "log"
    else:
        fatigue_score = calculate_fatigue_score(fatigue_label_seconds, total_logged_sec)
        max_fatigue_score = fatigue_score
        min_fatigue_score = fatigue_score
        fatigue_score_source = "calculated"

    return {
        "posture_score": posture_score,
        "max_posture_score": max_posture_score,
        "min_posture_score": min_posture_score,
        "posture_score_source": posture_score_source,
        "posture_grade": get_posture_grade(posture_score),

        "fatigue_score": fatigue_score,
        "max_fatigue_score": max_fatigue_score,
        "min_fatigue_score": min_fatigue_score,
        "fatigue_score_source": fatigue_score_source,
        "fatigue_grade": get_fatigue_grade(fatigue_score),
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
    - 자세 점수
    """

    total_logged_sec = len(df) * LOG_INTERVAL_SEC

    posture_type_seconds = count_seconds_by_label(df, "posture_type")

    good_sec = sum(
        seconds
        for label, seconds in posture_type_seconds.items()
        if is_good_posture_label(label)
    )

    bad_sec = sum(
        seconds
        for label, seconds in posture_type_seconds.items()
        if not is_good_posture_label(label)
        and not is_invalid_label(label)
    )

    # 혹시 bad_sec 계산이 전체 로그보다 커지는 이상 상황 방어
    bad_sec = min(int(bad_sec), int(total_logged_sec))
    good_sec = min(int(good_sec), int(total_logged_sec))

    most_common_posture = get_most_common_issue(
        seconds_dict=posture_type_seconds,
        exclude_labels=list(GOOD_POSTURE_ALIASES)
    )

    top_posture_issues = build_top_items(
        seconds_dict=posture_type_seconds,
        total_sec=total_logged_sec,
        exclude_labels=list(GOOD_POSTURE_ALIASES),
        top_n=3
    )

    posture_score = calculate_posture_score(posture_type_seconds, total_logged_sec)

    forward_head_sec = sum_seconds_by_labels(
        posture_type_seconds,
        {str(POSTURE_FORWARD_HEAD), "ForwardHead", "Forward Head"}
    )
    chin_propping_sec = sum_seconds_by_labels(
        posture_type_seconds,
        {str(POSTURE_CHIN_PROPPING), "ChinPropping", "ChinRest", "Chin Propping", "Chin Rest"}
    )
    asymmetric_sec = sum_seconds_by_labels(
        posture_type_seconds,
        {str(POSTURE_ASYMMETRIC), "Asymmetric", "Asymmetry", "ShoulderImbalance", "Shoulder Imbalance"}
    )

    return {
        "good_sec": int(good_sec),
        "good_time_text": seconds_to_text(good_sec),

        "bad_sec": int(bad_sec),
        "bad_time_text": seconds_to_text(bad_sec),

        "good_ratio": safe_ratio(good_sec, total_logged_sec),
        "bad_ratio": safe_ratio(bad_sec, total_logged_sec),

        "posture_score": posture_score,
        "posture_grade": get_posture_grade(posture_score),

        "most_common_posture": most_common_posture,

        "posture_type_seconds": posture_type_seconds,
        "top_posture_issues": top_posture_issues,

        "metric_summary": {
            "forward_head_sec": int(forward_head_sec),
            "forward_head_time_text": seconds_to_text(forward_head_sec),
            "forward_head_ratio": safe_ratio(forward_head_sec, total_logged_sec),

            "chin_propping_sec": int(chin_propping_sec),
            "chin_propping_time_text": seconds_to_text(chin_propping_sec),
            "chin_propping_ratio": safe_ratio(chin_propping_sec, total_logged_sec),

            "asymmetric_sec": int(asymmetric_sec),
            "asymmetric_time_text": seconds_to_text(asymmetric_sec),
            "asymmetric_ratio": safe_ratio(asymmetric_sec, total_logged_sec),
        }
    }


# ------------------------------------------------------------
# 피로도 요약
# ------------------------------------------------------------

def build_fatigue_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """
    피로도 관련 요약 데이터를 만든다.

    현재 스펙:
    - fatigue_label: Normal / Drowsy
    - fatigue_probability
    """

    total_logged_sec = len(df) * LOG_INTERVAL_SEC

    fatigue_label_seconds = count_seconds_by_label(df, "fatigue_label")

    normal_sec = 0
    drowsy_sec = 0

    for label, seconds in fatigue_label_seconds.items():
        if is_drowsy_label(label):
            drowsy_sec += int(seconds)
        elif normalize_label(label) == str(NORMAL_FATIGUE_LABEL):
            normal_sec += int(seconds)

    # Normal이 누락되어 있고 Drowsy만 있는 경우를 대비
    if normal_sec + drowsy_sec < total_logged_sec:
        normal_sec += max(total_logged_sec - normal_sec - drowsy_sec, 0)

    avg_fatigue_probability = safe_mean_column(df, "fatigue_probability", default=0.0)
    max_fatigue_probability = safe_max_column(df, "fatigue_probability", default=0.0)

    fatigue_score = calculate_fatigue_score(fatigue_label_seconds, total_logged_sec)

    return {
        "normal_sec": int(normal_sec),
        "normal_time_text": seconds_to_text(normal_sec),
        "normal_ratio": safe_ratio(normal_sec, total_logged_sec),

        "drowsy_sec": int(drowsy_sec),
        "drowsy_time_text": seconds_to_text(drowsy_sec),
        "drowsy_ratio": safe_ratio(drowsy_sec, total_logged_sec),

        "fatigue_score": fatigue_score,
        "fatigue_grade": get_fatigue_grade(fatigue_score),

        "fatigue_label_seconds": fatigue_label_seconds,

        # 현재 스펙에서는 하품/눈감김/입벌림 데이터가 없으므로 기본값
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

        posture_type_seconds = count_seconds_by_label(group, "posture_type")
        fatigue_label_seconds = count_seconds_by_label(group, "fatigue_label")

        good_sec = sum(
            seconds
            for label, seconds in posture_type_seconds.items()
            if is_good_posture_label(label)
        )
        bad_sec = sum(
            seconds
            for label, seconds in posture_type_seconds.items()
            if not is_good_posture_label(label)
            and not is_invalid_label(label)
        )

        drowsy_sec = sum(
            seconds
            for label, seconds in fatigue_label_seconds.items()
            if is_drowsy_label(label)
        )

        posture_scores = get_numeric_series(group, "posture_score")
        fatigue_scores = get_numeric_series(group, "fatigue_score")

        if len(posture_scores) > 0:
            avg_posture_score = safe_round(posture_scores.mean(), digit=1)
        else:
            avg_posture_score = calculate_posture_score(posture_type_seconds, logged_sec)

        if len(fatigue_scores) > 0:
            avg_fatigue_score = safe_round(fatigue_scores.mean(), digit=1)
        else:
            avg_fatigue_score = calculate_fatigue_score(fatigue_label_seconds, logged_sec)

        minute_summary = {
            "segment_index": int(segment_index),
            "minute_index": int(minute_index),
            "segment_start_time": str(group["timestamp"].min()),
            "segment_end_time": str(group["timestamp"].max()),

            "logged_sec": int(logged_sec),

            "good_sec": int(good_sec),
            "bad_sec": int(bad_sec),
            "good_ratio": safe_ratio(good_sec, logged_sec),
            "bad_ratio": safe_ratio(bad_sec, logged_sec),

            "drowsy_sec": int(drowsy_sec),
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

    df = df.copy()

    if "timestamp" not in df.columns:
        raise ValueError("요약을 생성할 수 없습니다. timestamp 컬럼이 없습니다.")

    df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["timestamp"])
    df = df.sort_values("timestamp").reset_index(drop=True)

    if df.empty:
        raise ValueError("요약을 생성할 수 없습니다. 유효한 timestamp 데이터가 없습니다.")

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

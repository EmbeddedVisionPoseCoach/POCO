import pandas as pd
from typing import List, Tuple




# ------------------------------------------------------------
# 우리가 최종 확정한 로그 컬럼
# ------------------------------------------------------------
# 라벨/컬럼 기준은 log_schema.json -> domain_schema.py를 통해 한 곳에서 관리한다.

try:
    from .domain_schema import (
        REQUIRED_COLUMNS,
        NUMERIC_COLUMNS,
        BOOL_COLUMNS,
        POSTURE_LABELS,
        FATIGUE_LABELS,
    )
except ImportError:
    from domain_schema import (
        REQUIRED_COLUMNS,
        NUMERIC_COLUMNS,
        BOOL_COLUMNS,
        POSTURE_LABELS,
        FATIGUE_LABELS,
    )


ALLOWED_POSTURE_TYPES = POSTURE_LABELS
ALLOWED_FATIGUE_LABELS = FATIGUE_LABELS


ALLOWED_BOOL_VALUES = [
    True,
    False,
    "True",
    "False",
    "true",
    "false",
    "TRUE",
    "FALSE",
    "1",
    "0",
    1,
    0
]


def validate_required_columns(df: pd.DataFrame) -> List[str]:
    """
    필수 컬럼이 모두 존재하는지 검사한다.
    """
    errors = []

    missing_columns = []

    for column in REQUIRED_COLUMNS:
        if column not in df.columns:
            missing_columns.append(column)

    if missing_columns:
        errors.append(f"필수 컬럼이 없습니다: {missing_columns}")

    return errors


def validate_not_empty(df: pd.DataFrame) -> List[str]:
    """
    CSV에 데이터 row가 존재하는지 검사한다.
    """
    errors = []

    if df.empty:
        errors.append("로그 데이터가 비어 있습니다.")

    return errors


def validate_timestamp_column(df: pd.DataFrame) -> List[str]:
    """
    timestamp 컬럼이 datetime으로 변환 가능한지 검사한다.
    """
    errors = []

    if "timestamp" not in df.columns:
        return errors

    converted = pd.to_datetime(df["timestamp"], errors="coerce")

    invalid_mask = converted.isna()

    if invalid_mask.any():
        invalid_rows = df.index[invalid_mask].tolist()
        errors.append(f"timestamp 컬럼에 날짜로 변환할 수 없는 값이 있습니다. row index: {invalid_rows}")

    return errors


def validate_numeric_columns(df: pd.DataFrame) -> List[str]:
    """
    숫자 컬럼들이 숫자로 변환 가능한지 검사한다.
    """
    errors = []

    for column in NUMERIC_COLUMNS:
        if column not in df.columns:
            continue

        converted = pd.to_numeric(df[column], errors="coerce")

        # 빈 문자열이나 None도 숫자로 쓸 수 없으므로 invalid 처리
        invalid_mask = converted.isna()

        if invalid_mask.any():
            invalid_rows = df.index[invalid_mask].tolist()
            errors.append(f"{column} 컬럼에 숫자로 변환할 수 없는 값이 있습니다. row index: {invalid_rows}")

    return errors


def validate_bool_columns(df: pd.DataFrame) -> List[str]:
    """
    bool 컬럼이 True/False 형태로 들어왔는지 검사한다.
    """
    errors = []

    for column in BOOL_COLUMNS:
        if column not in df.columns:
            continue

        invalid_values = []

        for value in df[column].unique():
            if value not in ALLOWED_BOOL_VALUES:
                invalid_values.append(value)

        if invalid_values:
            errors.append(f"{column} 컬럼에 bool로 변환하기 어려운 값이 있습니다: {invalid_values}")

    return errors


def validate_posture_labels(df: pd.DataFrame) -> List[str]:
    """
    posture_type 값이 허용된 라벨인지 검사한다.
    """
    errors = []

    if "posture_type" not in df.columns:
        return errors

    invalid_values = []

    for value in df["posture_type"].unique():
        if value not in ALLOWED_POSTURE_TYPES:
            invalid_values.append(value)

    if invalid_values:
        errors.append(f"허용되지 않은 posture_type 값이 있습니다: {invalid_values}")

    return errors


def validate_fatigue_labels(df: pd.DataFrame) -> List[str]:
    """
    fatigue_label 값이 허용된 라벨인지 검사한다.
    """
    errors = []

    if "fatigue_label" not in df.columns:
        return errors

    invalid_values = []

    for value in df["fatigue_label"].unique():
        if value not in ALLOWED_FATIGUE_LABELS:
            invalid_values.append(value)

    if invalid_values:
        errors.append(f"허용되지 않은 fatigue_label 값이 있습니다: {invalid_values}")

    return errors


def validate_elapsed_sec(df: pd.DataFrame) -> List[str]:
    """
    elapsed_sec가 1 이상인지 검사한다.
    """
    errors = []

    if "elapsed_sec" not in df.columns:
        return errors

    converted = pd.to_numeric(df["elapsed_sec"], errors="coerce")

    invalid_mask = converted < 0

    if invalid_mask.any():
        invalid_rows = df.index[invalid_mask].tolist()
        errors.append(f"elapsed_sec가 0보다 작은 row가 있습니다. row index: {invalid_rows}")

    return errors


def validate_log_dataframe(df: pd.DataFrame) -> Tuple[bool, List[str]]:
    """
    로그 DataFrame 전체를 검증한다.

    반환:
    - is_valid: 검증 성공 여부
    - errors: 에러 메시지 리스트
    """
    errors = []

    errors.extend(validate_not_empty(df))
    errors.extend(validate_required_columns(df))

    # 필수 컬럼이 없으면 이후 검사는 의미가 없으므로 여기서 종료
    if errors:
        return False, errors

    errors.extend(validate_timestamp_column(df))
    errors.extend(validate_numeric_columns(df))
    errors.extend(validate_bool_columns(df))
    errors.extend(validate_posture_labels(df))
    errors.extend(validate_fatigue_labels(df))
    errors.extend(validate_elapsed_sec(df))

    is_valid = len(errors) == 0

    return is_valid, errors


def raise_if_invalid(df: pd.DataFrame) -> None:
    """
    로그 데이터가 유효하지 않으면 ValueError를 발생시킨다.
    """
    is_valid, errors = validate_log_dataframe(df)

    if not is_valid:
        error_message = "\n".join(errors)
        raise ValueError(f"로그 데이터 검증 실패:\n{error_message}")
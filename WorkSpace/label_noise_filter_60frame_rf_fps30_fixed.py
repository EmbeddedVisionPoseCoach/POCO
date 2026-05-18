r"""
label_noise_filter_60frame_rf.py

60프레임 GRU 입력 단위 라벨 노이즈 필터링 스크립트.

목적
----
- 60프레임 window 하나를 샘플 1개로 보고 라벨 노이즈를 검사한다.
- 입력이 이미 GRU용 flattened CSV이면 그대로 사용한다.
  예: f0_eye_blink_left ... f59_jaw_open, label
- 입력이 프레임 단위 CSV이면 video_name별로 60프레임 window를 만든 뒤 검사한다.
  예: video_name, frame_idx, eye_blink_left, eye_blink_right, eye_closed_score, jaw_open, label

필터링 방식
-----------
- RandomForestClassifier + GroupKFold OOF 예측
- 같은 video_name에서 나온 window가 train/valid에 동시에 들어가지 않도록 한다.
- 원래 label과 모델이 본 패턴이 크게 다르면 suspect로 분리한다.

기본 판단 기준
--------------
- label == normal(0)인데 drowsy_probability >= 0.75  -> suspect
- label == drowsy(1)인데 drowsy_probability <= 0.25 -> suspect
- drowsy_probability가 0.40~0.60이면 ambiguous
- 나머지는 clean

실행 예시
---------
python label_noise_filter_60frame_rf.py

경로를 직접 지정하고 싶을 때:
python label_noise_filter_60frame_rf.py ^
  --input "C:\Users\KCCISTC\Desktop\csv\GRU_cal_csv\gru_face_dataset_calibrated.csv" ^
  --output "C:\Users\KCCISTC\Desktop\csv\GRU_LabelNoise_60Frame_RF"

필요 패키지
-----------
pip install pandas numpy scikit-learn
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import GroupKFold, StratifiedKFold
from sklearn.metrics import classification_report, confusion_matrix, roc_auc_score


# ============================================================
# 기본 설정
# ============================================================

DEFAULT_INPUT_CSV = Path(
    r"C:\Users\KCCISTC\Desktop\csv\GRU_FPS30_Filtered\gru_face_dataset_calibrated_fps30_balanced.csv"
)

DEFAULT_OUTPUT_DIR = Path(
    r"C:\Users\KCCISTC\Desktop\csv\GRU_clean_csv"
)

WINDOW_SIZE = 60
STRIDE = 5

BASE_FEATURES = [
    "eye_blink_left",
    "eye_blink_right",
    "eye_closed_score",
    "jaw_open",
]

RANDOM_STATE = 42
N_SPLITS = 3

# 라벨 반대 패턴 의심 기준
NORMAL_SUSPECT_DROWSY_PROBA = 0.75
DROWSY_SUSPECT_DROWSY_PROBA = 0.25

# 애매한 확률 구간
AMBIGUOUS_LOW = 0.40
AMBIGUOUS_HIGH = 0.60


# ============================================================
# 유틸 함수
# ============================================================

def normalize_label(value) -> Optional[int]:
    """
    다양한 라벨 표현을 binary label로 변환한다.

    허용:
        0, "0", "normal", "Normal" -> 0
        1, "1", 10, "10", "drowsy", "Drowsy" -> 1

    제외:
        5, "5", "ambiguous", 알 수 없는 값 -> None
    """
    if pd.isna(value):
        return None

    text = str(value).strip().lower()

    if text in {"0", "0.0", "normal", "n"}:
        return 0

    if text in {"1", "1.0", "10", "10.0", "drowsy", "d", "sleepy"}:
        return 1

    # 5 라벨은 경계/애매한 라벨로 보고 학습 및 필터링에서 제외
    if text in {"5", "5.0", "ambiguous", "unknown", "borderline"}:
        return None

    return None


def ensure_output_dir(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)


def find_label_column(df: pd.DataFrame) -> str:
    candidates = ["label", "Label", "target", "class", "y"]
    for col in candidates:
        if col in df.columns:
            return col
    raise ValueError(
        "라벨 컬럼을 찾지 못했습니다. CSV에 label 컬럼이 필요합니다."
    )


def find_group_column(df: pd.DataFrame) -> Optional[str]:
    candidates = [
        "video_name",
        "filename",
        "file_name",
        "source_file",
        "video",
        "video_id",
        "person_id",
        "subject_id",
        "group",
    ]

    for col in candidates:
        if col in df.columns:
            return col

    return None


def flattened_feature_columns() -> List[str]:
    """
    현재 dataCollectFace.py 스타일 컬럼명:
        f0_eye_blink_left
        f0_eye_blink_right
        ...
        f59_jaw_open
    """
    cols = []
    for frame_idx in range(WINDOW_SIZE):
        for feat in BASE_FEATURES:
            cols.append(f"f{frame_idx}_{feat}")
    return cols


def detect_flattened_columns(df: pd.DataFrame) -> List[str]:
    """
    60프레임 flattened 입력인지 확인한다.

    1순위:
        f0_eye_blink_left ~ f59_jaw_open 형태

    2순위:
        정규식으로 f숫자_피처 컬럼을 찾아서 정렬
    """
    expected = flattened_feature_columns()

    if all(col in df.columns for col in expected):
        return expected

    # 혹시 f00_eye_blink_left, f01_eye_blink_left처럼 0 padding이 들어간 경우까지 지원
    pattern = re.compile(
        r"^f(?P<frame>\d+)_(?P<feature>eye_blink_left|eye_blink_right|eye_closed_score|jaw_open)$"
    )

    found: Dict[Tuple[int, str], str] = {}

    for col in df.columns:
        match = pattern.match(str(col))
        if match:
            frame_idx = int(match.group("frame"))
            feature_name = match.group("feature")

            if 0 <= frame_idx < WINDOW_SIZE:
                found[(frame_idx, feature_name)] = col

    ordered_cols = []

    for frame_idx in range(WINDOW_SIZE):
        for feat in BASE_FEATURES:
            key = (frame_idx, feat)
            if key not in found:
                return []
            ordered_cols.append(found[key])

    return ordered_cols


def is_frame_level_csv(df: pd.DataFrame) -> bool:
    return all(col in df.columns for col in BASE_FEATURES)


def get_sort_columns(df: pd.DataFrame) -> List[str]:
    candidates = [
        "frame_idx",
        "frame_index",
        "frame",
        "timestamp",
        "time",
        "time_sec",
    ]
    return [col for col in candidates if col in df.columns]


# ============================================================
# 입력 CSV 처리
# ============================================================

def prepare_flattened_dataset(
    df: pd.DataFrame,
    feature_cols: Sequence[str],
    label_col: str,
) -> Tuple[pd.DataFrame, List[str], str]:
    """
    이미 60프레임 window 단위로 펼쳐진 CSV를 필터링용 형태로 정리한다.
    """
    work_df = df.copy()

    work_df["label_original"] = work_df[label_col]
    work_df["label_binary"] = work_df[label_col].apply(normalize_label)

    before = len(work_df)
    work_df = work_df[work_df["label_binary"].isin([0, 1])].copy()
    dropped = before - len(work_df)

    if dropped > 0:
        print(f"⚠️ 라벨 0/1로 변환할 수 없는 행 {dropped}개를 제외했습니다.")

    for col in feature_cols:
        work_df[col] = pd.to_numeric(work_df[col], errors="coerce")

    work_df = work_df.dropna(subset=list(feature_cols) + ["label_binary"]).copy()
    work_df["label_binary"] = work_df["label_binary"].astype(int)

    group_col = find_group_column(work_df)

    if group_col is None:
        # video_name이 없는 경우 row 단위 group을 사용한다.
        # 이 경우 영상 단위 누수 방지는 불가능하므로 경고를 출력한다.
        group_col = "__group_id__"
        work_df[group_col] = np.arange(len(work_df)).astype(str)
        print(
            "⚠️ video_name/group 컬럼이 없습니다. "
            "영상 단위 GroupKFold 대신 각 행을 별도 group으로 처리합니다."
        )

    return work_df, list(feature_cols), group_col


def build_windows_from_frame_level_csv(
    df: pd.DataFrame,
    label_col: str,
) -> Tuple[pd.DataFrame, List[str], str]:
    """
    프레임 단위 CSV에서 video_name별 60프레임 window를 만들어 flattened dataset으로 변환한다.
    """
    group_col = find_group_column(df)

    if group_col is None:
        raise ValueError(
            "프레임 단위 CSV에서 60프레임 window를 만들려면 "
            "video_name 또는 filename 같은 그룹 컬럼이 필요합니다."
        )

    work_df = df.copy()
    work_df["label_binary"] = work_df[label_col].apply(normalize_label)
    work_df = work_df[work_df["label_binary"].isin([0, 1])].copy()
    work_df["label_binary"] = work_df["label_binary"].astype(int)

    for col in BASE_FEATURES:
        work_df[col] = pd.to_numeric(work_df[col], errors="coerce")

    work_df = work_df.dropna(subset=BASE_FEATURES + ["label_binary", group_col]).copy()

    sort_cols = get_sort_columns(work_df)

    rows = []
    feature_cols = flattened_feature_columns()

    for video_name, video_df in work_df.groupby(group_col):
        if sort_cols:
            video_df = video_df.sort_values(sort_cols)
        else:
            video_df = video_df.sort_index()

        labels = video_df["label_binary"].unique()

        if len(labels) != 1:
            print(
                f"⚠️ {video_name}: 한 영상 안에 여러 라벨이 섞여 있어 제외합니다. labels={labels}"
            )
            continue

        label = int(labels[0])
        values = video_df[BASE_FEATURES].to_numpy(dtype=np.float32)

        if len(values) < WINDOW_SIZE:
            print(f"⚠️ {video_name}: 프레임 수 {len(values)} < {WINDOW_SIZE}, 제외")
            continue

        window_id = 0

        for start_idx in range(0, len(values) - WINDOW_SIZE + 1, STRIDE):
            window = values[start_idx:start_idx + WINDOW_SIZE]
            flat = window.reshape(-1)

            row = {
                "video_name": video_name,
                "window_id": window_id,
                "start_frame_offset": start_idx,
                "end_frame_offset": start_idx + WINDOW_SIZE - 1,
                "label": label,
                "label_original": label,
                "label_binary": label,
            }

            for col_name, value in zip(feature_cols, flat):
                row[col_name] = float(value)

            rows.append(row)
            window_id += 1

    if not rows:
        raise ValueError("생성된 60프레임 window가 없습니다.")

    result_df = pd.DataFrame(rows)

    return result_df, feature_cols, "video_name"


def load_and_prepare_dataset(input_csv: Path) -> Tuple[pd.DataFrame, List[str], str]:
    if not input_csv.exists():
        raise FileNotFoundError(f"입력 CSV를 찾을 수 없습니다: {input_csv}")

    print(f"✅ 입력 CSV 로드: {input_csv}")
    df = pd.read_csv(input_csv)
    print(f"원본 shape: {df.shape}")

    label_col = find_label_column(df)
    feature_cols = detect_flattened_columns(df)

    if feature_cols:
        print("✅ 입력 형식 감지: 이미 60프레임 flattened GRU CSV입니다.")
        return prepare_flattened_dataset(df, feature_cols, label_col)

    if is_frame_level_csv(df):
        print("✅ 입력 형식 감지: 프레임 단위 CSV입니다. 60프레임 window를 생성합니다.")
        return build_windows_from_frame_level_csv(df, label_col)

    raise ValueError(
        "CSV 형식을 인식하지 못했습니다.\n"
        "필요 형식 1: f0_eye_blink_left ~ f59_jaw_open + label\n"
        "필요 형식 2: eye_blink_left, eye_blink_right, eye_closed_score, jaw_open + label + video_name"
    )


# ============================================================
# RandomForest OOF 예측
# ============================================================

def make_random_forest() -> RandomForestClassifier:
    return RandomForestClassifier(
        n_estimators=120,          # 트리 개수: 기존 500개보다 훨씬 가볍게
        max_depth=17,              # 트리 깊이 제한: 과적합 방지 + 속도 개선
        min_samples_leaf=10,       # leaf 노드 최소 샘플 수
        min_samples_split=20,      # split 최소 샘플 수
        max_features="sqrt",       # 각 split에서 일부 feature만 사용
        class_weight="balanced_subsample",
        bootstrap=True,
        max_samples=0.45,          # 각 트리가 전체 데이터의 45%만 샘플링해서 학습
        random_state=RANDOM_STATE,
        n_jobs=-1,                 # CPU 코어 최대 사용
    )


def get_oof_probabilities(
    df: pd.DataFrame,
    feature_cols: Sequence[str],
    group_col: str,
) -> Tuple[np.ndarray, np.ndarray]:
    X = df[list(feature_cols)].to_numpy(dtype=np.float32)
    y = df["label_binary"].to_numpy(dtype=int)
    groups = df[group_col].astype(str).to_numpy()

    unique_groups = np.unique(groups)
    unique_labels = np.unique(y)

    if len(unique_labels) < 2:
        raise ValueError(
            "라벨이 한 종류만 있습니다. normal/drowsy 두 라벨이 모두 있어야 필터링할 수 있습니다."
        )

    oof_proba = np.full(len(df), np.nan, dtype=np.float32)
    fold_ids = np.full(len(df), -1, dtype=int)

    if len(unique_groups) >= 2:
        n_splits = min(N_SPLITS, len(unique_groups))
        cv = GroupKFold(n_splits=n_splits)
        split_iter = cv.split(X, y, groups)
        print(f"✅ GroupKFold 사용: n_splits={n_splits}, groups={len(unique_groups)}")
    else:
        # group이 한 개뿐이면 영상 단위 분리는 불가능하므로 StratifiedKFold 사용
        n_splits = min(N_SPLITS, np.bincount(y).min())
        if n_splits < 2:
            raise ValueError(
                "데이터가 너무 적어서 교차검증을 만들 수 없습니다. "
                "각 라벨별 샘플이 최소 2개 이상 필요합니다."
            )
        cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=RANDOM_STATE)
        split_iter = cv.split(X, y)
        print(f"⚠️ StratifiedKFold 사용: n_splits={n_splits}")

    for fold, (train_idx, valid_idx) in enumerate(split_iter, start=1):
        y_train = y[train_idx]

        if len(np.unique(y_train)) < 2:
            raise ValueError(
                f"Fold {fold}: train set에 라벨이 한 종류만 있습니다. "
                "group 구성 또는 데이터 수를 확인하세요."
            )

        model = make_random_forest()
        model.fit(X[train_idx], y_train)

        proba = model.predict_proba(X[valid_idx])

        # classes_ 순서가 [0,1]이 아닐 수도 있으므로 drowsy class index를 찾는다.
        if 1 not in model.classes_:
            raise ValueError(f"Fold {fold}: drowsy class가 train set에 없습니다.")

        drowsy_index = list(model.classes_).index(1)
        oof_proba[valid_idx] = proba[:, drowsy_index].astype(np.float32)
        fold_ids[valid_idx] = fold

        print(
            f"Fold {fold} 완료 | train={len(train_idx)} | valid={len(valid_idx)}"
        )

    if np.isnan(oof_proba).any():
        missing = int(np.isnan(oof_proba).sum())
        raise RuntimeError(f"OOF 예측이 비어 있는 샘플이 있습니다: {missing}개")

    return oof_proba, fold_ids


# ============================================================
# 라벨 노이즈 판정
# ============================================================

def classify_noise_status(row: pd.Series) -> Tuple[str, str]:
    label = int(row["label_binary"])
    p = float(row["rf_drowsy_probability"])

    if label == 0 and p >= NORMAL_SUSPECT_DROWSY_PROBA:
        return "suspect", "normal_label_but_drowsy_pattern"

    if label == 1 and p <= DROWSY_SUSPECT_DROWSY_PROBA:
        return "suspect", "drowsy_label_but_normal_pattern"

    if AMBIGUOUS_LOW <= p <= AMBIGUOUS_HIGH:
        return "ambiguous", "low_confidence_boundary"

    return "clean", "label_consistent"


def add_filter_columns(df: pd.DataFrame, oof_proba: np.ndarray, fold_ids: np.ndarray) -> pd.DataFrame:
    result = df.copy()

    result["rf_drowsy_probability"] = oof_proba
    result["rf_pred_label"] = (result["rf_drowsy_probability"] >= 0.5).astype(int)
    result["rf_fold"] = fold_ids

    statuses = result.apply(classify_noise_status, axis=1)
    result["filter_status"] = [status for status, _ in statuses]
    result["filter_reason"] = [reason for _, reason in statuses]

    return result


# ============================================================
# 저장
# ============================================================

def save_outputs(
    result_df: pd.DataFrame,
    feature_cols: Sequence[str],
    group_col: str,
    output_dir: Path,
) -> None:
    ensure_output_dir(output_dir)

    clean_df = result_df[result_df["filter_status"] == "clean"].copy()
    suspect_df = result_df[result_df["filter_status"] == "suspect"].copy()
    ambiguous_df = result_df[result_df["filter_status"] == "ambiguous"].copy()

    all_path = output_dir / "all_scored_60frame_rf.csv"
    clean_scored_path = output_dir / "clean_with_scores_60frame_rf.csv"
    suspect_path = output_dir / "suspect_60frame_rf.csv"
    ambiguous_path = output_dir / "ambiguous_60frame_rf.csv"
    train_path = output_dir / "clean_for_gru_training_60frame_rf.csv"
    report_path = output_dir / "label_noise_report_60frame_rf.csv"
    video_summary_path = output_dir / "video_summary_60frame_rf.csv"
    config_path = output_dir / "filter_config_60frame_rf.json"

    result_df.to_csv(all_path, index=False, encoding="utf-8-sig")
    clean_df.to_csv(clean_scored_path, index=False, encoding="utf-8-sig")
    suspect_df.to_csv(suspect_path, index=False, encoding="utf-8-sig")
    ambiguous_df.to_csv(ambiguous_path, index=False, encoding="utf-8-sig")

    # 학습용 clean CSV:
    # - 가능한 경우 video_name/window_id는 유지
    # - feature 240개와 binary label만 넣어서 학습 코드에서 바로 쓰기 쉽게 구성
    meta_cols = []
    for col in [
        group_col,
        "window_id",
        "start_frame_offset",
        "end_frame_offset",
        "start_frame_idx",
        "end_frame_idx",
        "start_sec",
        "end_sec",
    ]:
        if col in clean_df.columns and col not in meta_cols and not col.startswith("__"):
            meta_cols.append(col)

    train_df = clean_df[meta_cols + list(feature_cols)].copy()
    train_df["label"] = clean_df["label_binary"].astype(int).values
    train_df.to_csv(train_path, index=False, encoding="utf-8-sig")

    # 전체 요약
    summary_rows = []

    for status, sub_df in result_df.groupby("filter_status"):
        summary_rows.append({
            "filter_status": status,
            "count": len(sub_df),
            "ratio": len(sub_df) / len(result_df) if len(result_df) > 0 else 0,
            "normal_count": int((sub_df["label_binary"] == 0).sum()),
            "drowsy_count": int((sub_df["label_binary"] == 1).sum()),
            "mean_drowsy_probability": float(sub_df["rf_drowsy_probability"].mean()) if len(sub_df) else np.nan,
        })

    report_df = pd.DataFrame(summary_rows)
    report_df.to_csv(report_path, index=False, encoding="utf-8-sig")

    # 영상별 요약
    if group_col in result_df.columns:
        video_summary = (
            result_df
            .groupby(group_col)
            .agg(
                total_windows=("filter_status", "size"),
                clean_windows=("filter_status", lambda s: int((s == "clean").sum())),
                suspect_windows=("filter_status", lambda s: int((s == "suspect").sum())),
                ambiguous_windows=("filter_status", lambda s: int((s == "ambiguous").sum())),
                mean_drowsy_probability=("rf_drowsy_probability", "mean"),
                label_binary=("label_binary", lambda s: int(s.mode().iloc[0]) if len(s.mode()) else int(s.iloc[0])),
            )
            .reset_index()
        )
        video_summary["suspect_ratio"] = video_summary["suspect_windows"] / video_summary["total_windows"]
        video_summary["ambiguous_ratio"] = video_summary["ambiguous_windows"] / video_summary["total_windows"]
        video_summary.to_csv(video_summary_path, index=False, encoding="utf-8-sig")

    config = {
        "window_size": WINDOW_SIZE,
        "stride": STRIDE,
        "base_features": BASE_FEATURES,
        "model": "RandomForestClassifier",
        "n_estimators": 500,
        "min_samples_leaf": 3,
        "normal_suspect_drowsy_probability": NORMAL_SUSPECT_DROWSY_PROBA,
        "drowsy_suspect_drowsy_probability": DROWSY_SUSPECT_DROWSY_PROBA,
        "ambiguous_low": AMBIGUOUS_LOW,
        "ambiguous_high": AMBIGUOUS_HIGH,
        "random_state": RANDOM_STATE,
    }

    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=4)

    print("\n" + "=" * 70)
    print("✅ 저장 완료")
    print(f"전체 점수 포함 CSV: {all_path}")
    print(f"Clean 점수 포함 CSV: {clean_scored_path}")
    print(f"Suspect CSV: {suspect_path}")
    print(f"Ambiguous CSV: {ambiguous_path}")
    print(f"GRU 학습용 Clean CSV: {train_path}")
    print(f"요약 리포트: {report_path}")
    print(f"영상별 요약: {video_summary_path}")
    print(f"필터 설정 JSON: {config_path}")
    print("=" * 70)


# ============================================================
# 평가 출력
# ============================================================

def print_diagnostics(result_df: pd.DataFrame) -> None:
    y_true = result_df["label_binary"].to_numpy(dtype=int)
    y_pred = result_df["rf_pred_label"].to_numpy(dtype=int)
    y_proba = result_df["rf_drowsy_probability"].to_numpy(dtype=np.float32)

    print("\n" + "=" * 70)
    print("OOF 예측 진단")
    print("=" * 70)

    print("\n[Confusion Matrix] rows=true, cols=pred")
    print(confusion_matrix(y_true, y_pred))

    print("\n[Classification Report]")
    print(
        classification_report(
            y_true,
            y_pred,
            target_names=["normal", "drowsy"],
            digits=4,
            zero_division=0,
        )
    )

    try:
        auc = roc_auc_score(y_true, y_proba)
        print(f"ROC-AUC: {auc:.4f}")
    except Exception as e:
        print(f"ROC-AUC 계산 불가: {e}")

    print("\n[Filter Status Counts]")
    print(result_df["filter_status"].value_counts(dropna=False))

    print("\n[Filter Reason Counts]")
    print(result_df["filter_reason"].value_counts(dropna=False))

    print("=" * 70)


# ============================================================
# main
# ============================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="60프레임 GRU 입력 단위 RandomForest 라벨 노이즈 필터링"
    )

    parser.add_argument(
        "--input",
        type=str,
        default=str(DEFAULT_INPUT_CSV),
        help="입력 CSV 경로",
    )

    parser.add_argument(
        "--output",
        type=str,
        default=str(DEFAULT_OUTPUT_DIR),
        help="결과 저장 폴더",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    input_csv = Path(args.input)
    output_dir = Path(args.output)

    df, feature_cols, group_col = load_and_prepare_dataset(input_csv)

    print("\n" + "=" * 70)
    print("필터링 대상 데이터")
    print("=" * 70)
    print(f"샘플 수: {len(df)}")
    print(f"feature 수: {len(feature_cols)}")
    print(f"group 컬럼: {group_col}")
    print(f"라벨 분포:\n{df['label_binary'].value_counts().sort_index()}")
    print("=" * 70)

    oof_proba, fold_ids = get_oof_probabilities(df, feature_cols, group_col)

    result_df = add_filter_columns(df, oof_proba, fold_ids)

    print_diagnostics(result_df)

    save_outputs(result_df, feature_cols, group_col, output_dir)


if __name__ == "__main__":
    main()

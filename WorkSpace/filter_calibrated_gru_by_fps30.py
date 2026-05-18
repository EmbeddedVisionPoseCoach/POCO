import pandas as pd
from pathlib import Path
import re

# ============================================================
# 1. 경로 설정
# ============================================================

FPS30_VIDEO_CSV = Path(
    r"C:\Users\KCCISTC\Desktop\video_fps_analysis\by_fps\fps_30_videos.csv"
)

FULL_CALIBRATED_CSV = Path(
    r"C:\Users\KCCISTC\Desktop\csv\GRU_cal_csv\gru_face_dataset_calibrated.csv"
)

OUTPUT_DIR = Path(
    r"C:\Users\KCCISTC\Desktop\csv\GRU_FPS30_Filtered"
)

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

FINAL_OUTPUT_CSV = OUTPUT_DIR / "gru_face_dataset_calibrated_fps30_balanced.csv"

# 확인용 파일들
MATCHED_BEFORE_BALANCE_CSV = OUTPUT_DIR / "matched_fps30_before_person_balance.csv"
PERSON_SUMMARY_CSV = OUTPUT_DIR / "person_label_summary.csv"
EXCLUDED_PERSONS_CSV = OUTPUT_DIR / "excluded_persons_only_one_label.csv"
UNMATCHED_FPS30_CSV = OUTPUT_DIR / "unmatched_fps30_videos_not_in_calibrated_csv.csv"


# ============================================================
# 2. 이름 정규화 함수
# ============================================================

def normalize_video_key(value):
    """
    영상명 / CSV명 / 경로를 비교용 key로 통일한다.

    예:
        raw_frame_normal_0__19.csv  -> 0__19
        raw_frame_drowsy_10__19.csv -> 10__19
        normal_0__19.csv            -> 0__19
        drowsy_10__19.csv           -> 10__19
        0__19.mp4                   -> 0__19
        10__19                      -> 10__19
    """

    if pd.isna(value):
        return ""

    value = str(value).replace("\\", "/")
    name = Path(value).name.lower()
    stem = Path(name).stem

    # 앞쪽 접두어 제거
    stem = re.sub(r"^raw_frame_", "", stem)
    stem = re.sub(r"^(normal|drowsy)_", "", stem)

    # 혹시 파일명 안에 0__19, 10__19 패턴이 있으면 그것만 추출
    match = re.search(r"(10|0)__([0-9]+)", stem)
    if match:
        label_code = match.group(1)
        person_id = match.group(2)
        return f"{label_code}__{person_id}"

    return stem


def extract_person_id(video_key):
    """
    같은 사람 판별용 ID 추출.

    예:
        0__19  -> 19
        10__19 -> 19
    """

    video_key = str(video_key)

    if "__" in video_key:
        return video_key.split("__", 1)[1]

    return video_key


def infer_label_from_key(video_key):
    """
    video_key 기준 라벨 추정.

    0__xx  -> 0, normal
    10__xx -> 1, drowsy
    """

    video_key = str(video_key).lower()

    if video_key.startswith("0__"):
        return 0

    if video_key.startswith("10__"):
        return 1

    if "normal" in video_key:
        return 0

    if "drowsy" in video_key:
        return 1

    return None


def normalize_label(value, video_key):
    """
    label 컬럼 값을 0/1로 통일한다.
    label 컬럼이 이상하면 video_key에서 추정한다.
    """

    if pd.notna(value):
        text = str(value).strip().lower()

        if text in ["0", "0.0", "normal"]:
            return 0

        if text in ["1", "1.0", "drowsy"]:
            return 1

    return infer_label_from_key(video_key)


# ============================================================
# 3. fps_30_videos.csv에서 30fps 영상 key 수집
# ============================================================

def build_fps30_key_set(fps30_df):
    """
    fps_30_videos.csv 안에 어떤 컬럼명이 있든 최대한 대응되도록 key를 만든다.

    새 코드 기준:
        standard_name, video_key

    예전 코드 기준:
        video_name, video_stem, video_path,
        expected_csv_name_1, expected_csv_name_2, expected_csv_name_3
    """

    candidate_cols = [
        "standard_name",
        "video_key",
        "video_name",
        "video_stem",
        "video_path",
        "original_video_name",
        "original_video_stem",
        "original_video_path",
        "expected_csv_name_1",
        "expected_csv_name_2",
        "expected_csv_name_3",
    ]

    keys = set()

    for col in candidate_cols:
        if col not in fps30_df.columns:
            continue

        for value in fps30_df[col].dropna():
            key = normalize_video_key(value)
            if key:
                keys.add(key)

    return keys


# ============================================================
# 4. 메인 처리
# ============================================================

def main():
    print("=" * 80)
    print("30fps 영상 기준 보정 GRU CSV 필터링 시작")
    print("=" * 80)

    if not FPS30_VIDEO_CSV.exists():
        raise FileNotFoundError(f"fps_30_videos.csv를 찾을 수 없습니다:\n{FPS30_VIDEO_CSV}")

    if not FULL_CALIBRATED_CSV.exists():
        raise FileNotFoundError(f"보정된 전체 CSV를 찾을 수 없습니다:\n{FULL_CALIBRATED_CSV}")

    fps30_df = pd.read_csv(FPS30_VIDEO_CSV)
    full_df = pd.read_csv(FULL_CALIBRATED_CSV)

    print(f"FPS 30 영상 목록 shape: {fps30_df.shape}")
    print(f"전체 보정 CSV shape: {full_df.shape}")

    if "video_name" not in full_df.columns:
        raise ValueError("보정된 전체 CSV에 video_name 컬럼이 없습니다.")

    # ------------------------------------------------------------
    # 1) fps_30_videos.csv에서 30fps 영상 key 생성
    # ------------------------------------------------------------

    fps30_keys = build_fps30_key_set(fps30_df)

    print(f"\n30fps 영상 key 수: {len(fps30_keys)}")

    if len(fps30_keys) == 0:
        raise RuntimeError("fps_30_videos.csv에서 사용할 영상 key를 만들지 못했습니다.")

    # ------------------------------------------------------------
    # 2) 보정 CSV에도 비교용 key / person_id / label 추가
    # ------------------------------------------------------------

    full_df["__video_key"] = full_df["video_name"].apply(normalize_video_key)
    full_df["__person_id"] = full_df["__video_key"].apply(extract_person_id)

    if "label" in full_df.columns:
        full_df["__label_int"] = full_df.apply(
            lambda row: normalize_label(row["label"], row["__video_key"]),
            axis=1
        )
    else:
        full_df["__label_int"] = full_df["__video_key"].apply(infer_label_from_key)

    # ------------------------------------------------------------
    # 3) 30fps 영상에 해당하는 row만 추출
    #    보정 CSV에 해당 파일이 없으면 자동으로 제외됨
    # ------------------------------------------------------------

    matched_df = full_df[full_df["__video_key"].isin(fps30_keys)].copy()

    print("\n[1차 필터] 30fps 영상만 추출")
    print(f"매칭된 row 수: {len(matched_df)}")
    print(f"매칭된 영상 수: {matched_df['__video_key'].nunique()}")

    matched_df.to_csv(
        MATCHED_BEFORE_BALANCE_CSV,
        index=False,
        encoding="utf-8-sig"
    )

    # fps30 목록에는 있는데 보정 CSV에는 없는 영상 확인용 저장
    matched_keys = set(matched_df["__video_key"].dropna().unique())
    unmatched_keys = sorted(list(fps30_keys - matched_keys))

    unmatched_df = pd.DataFrame({
        "unmatched_video_key": unmatched_keys
    })

    unmatched_df.to_csv(
        UNMATCHED_FPS30_CSV,
        index=False,
        encoding="utf-8-sig"
    )

    print(f"보정 CSV에 없어서 넘어간 30fps 영상 수: {len(unmatched_df)}")

    if len(matched_df) == 0:
        raise RuntimeError("30fps 영상과 보정 CSV가 하나도 매칭되지 않았습니다.")

    # ------------------------------------------------------------
    # 4) 영상 단위로 요약
    # ------------------------------------------------------------

    video_summary = (
        matched_df
        .groupby("__video_key")
        .agg(
            person_id=("__person_id", "first"),
            label_int=("__label_int", "first"),
            row_count=("__video_key", "count")
        )
        .reset_index()
    )

    video_summary["label_name"] = video_summary["label_int"].map({
        0: "normal",
        1: "drowsy"
    })

    # ------------------------------------------------------------
    # 5) 같은 사람 기준 normal / drowsy 둘 다 있는지 검사
    # ------------------------------------------------------------

    person_summary = (
        video_summary
        .groupby("person_id")
        .agg(
            video_count=("__video_key", "count"),
            normal_video_count=("label_int", lambda x: (x == 0).sum()),
            drowsy_video_count=("label_int", lambda x: (x == 1).sum()),
            row_count=("row_count", "sum"),
            video_keys=("__video_key", lambda x: ",".join(sorted(x))),
        )
        .reset_index()
    )

    person_summary["use_for_training"] = (
        (person_summary["normal_video_count"] > 0) &
        (person_summary["drowsy_video_count"] > 0)
    )

    person_summary.to_csv(
        PERSON_SUMMARY_CSV,
        index=False,
        encoding="utf-8-sig"
    )

    excluded_persons = person_summary[
        ~person_summary["use_for_training"]
    ].copy()

    excluded_persons.to_csv(
        EXCLUDED_PERSONS_CSV,
        index=False,
        encoding="utf-8-sig"
    )

    keep_person_ids = set(
        person_summary.loc[
            person_summary["use_for_training"],
            "person_id"
        ]
    )

    # ------------------------------------------------------------
    # 6) 한 라벨만 있는 사람 제거
    # ------------------------------------------------------------

    final_df = matched_df[
        matched_df["__person_id"].isin(keep_person_ids)
    ].copy()

    print("\n[2차 필터] 같은 사람이 normal/drowsy 둘 다 있는 경우만 유지")
    print(f"유지된 사람 수: {len(keep_person_ids)}")
    print(f"제외된 사람 수: {len(excluded_persons)}")
    print(f"최종 row 수: {len(final_df)}")
    print(f"최종 영상 수: {final_df['__video_key'].nunique()}")
    print(f"최종 사람 수: {final_df['__person_id'].nunique()}")

    print("\n최종 row 라벨 분포:")
    print(final_df["__label_int"].value_counts().sort_index())

    print("\n최종 영상 라벨 분포:")
    print(
        final_df
        .drop_duplicates("__video_key")["__label_int"]
        .value_counts()
        .sort_index()
    )

    # ------------------------------------------------------------
    # 7) 내부 확인용 컬럼 제거 후 최종 저장
    # ------------------------------------------------------------

    internal_cols = [
        "__video_key",
        "__person_id",
        "__label_int",
    ]

    export_df = final_df.drop(
        columns=[col for col in internal_cols if col in final_df.columns]
    )

    export_df.to_csv(
        FINAL_OUTPUT_CSV,
        index=False,
        encoding="utf-8-sig"
    )

    print("\n" + "=" * 80)
    print("저장 완료")
    print("=" * 80)
    print(f"최종 30fps 보정 CSV:")
    print(f"  {FINAL_OUTPUT_CSV}")
    print()
    print(f"30fps 매칭 후 사람 필터 전 CSV:")
    print(f"  {MATCHED_BEFORE_BALANCE_CSV}")
    print()
    print(f"사람별 normal/drowsy 보유 여부:")
    print(f"  {PERSON_SUMMARY_CSV}")
    print()
    print(f"한 라벨만 있어서 제외된 사람 목록:")
    print(f"  {EXCLUDED_PERSONS_CSV}")
    print()
    print(f"30fps 목록에는 있지만 보정 CSV에 없어서 넘어간 영상:")
    print(f"  {UNMATCHED_FPS30_CSV}")


if __name__ == "__main__":
    main()
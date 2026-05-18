import cv2
import pandas as pd
from pathlib import Path
import re

# ============================================================
# 1. 경로 설정
# ============================================================

VIDEO_ROOT = Path(r"E:\video")

OUTPUT_DIR = Path(r"E:\video_fps_analysis")

VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".webm"}

# 사용할 라벨만 지정
USE_LABELS = {"0", "10"}

# 제외할 라벨
EXCLUDE_LABELS = {"5"}


# ============================================================
# 2. 영상 파일명에서 라벨 추출
# ============================================================

def extract_label_from_filename(video_path: Path):
    """
    파일명 앞부분에서 0, 5, 10 라벨을 추출한다.

    예:
        0.mp4        -> 0
        0_01.mp4     -> 0
        10.mp4       -> 10
        10_test.mp4  -> 10
        5.mp4        -> 5
    """

    stem = video_path.stem.strip()

    # 10을 0보다 먼저 검사해야 함
    match = re.match(r"^(10|5|0)(?=\D|$)", stem)

    if match:
        return match.group(1)

    return None


# ============================================================
# 3. 영상 정보 읽기
# ============================================================

def get_video_info(video_path: Path):
    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        return {
            "open_ok": False,
            "fps_raw": None,
            "fps_group": None,
            "total_frames": None,
            "duration_sec": None,
            "sec_per_60_frames": None,
            "width": None,
            "height": None,
        }

    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    cap.release()

    fps = float(fps) if fps else 0.0
    total_frames = int(total_frames) if total_frames else 0

    if fps > 0:
        fps_group = int(round(fps))   # 29.97 -> 30
        duration_sec = total_frames / fps
        sec_per_60_frames = 60 / fps
    else:
        fps_group = None
        duration_sec = None
        sec_per_60_frames = None

    return {
        "open_ok": True,
        "fps_raw": fps,
        "fps_group": fps_group,
        "total_frames": total_frames,
        "duration_sec": duration_sec,
        "sec_per_60_frames": sec_per_60_frames,
        "width": int(width) if width else None,
        "height": int(height) if height else None,
    }


# ============================================================
# 4. 표준 저장 이름 생성
# ============================================================

def make_standard_name(label_code: str, person_folder_name: str):
    """
    E:\\video\\19 안의 0 라벨 영상 -> 0__19
    E:\\video\\19 안의 10 라벨 영상 -> 10__19

    폴더명이 01이면 0__01 형태로 저장된다.
    만약 0__1처럼 앞 0을 제거하고 싶으면 아래 주석 참고.
    """

    person_id = person_folder_name

    # 앞자리 0 제거를 원하면 이 줄 사용:
    # person_id = str(int(person_folder_name))

    return f"{label_code}__{person_id}"


def make_expected_csv_names(standard_name: str):
    """
    나중에 GRU CSV나 raw_frame CSV와 매칭할 때 쓸 후보 이름.
    """

    label_code = standard_name.split("__")[0]

    if label_code == "0":
        label_text = "normal"
    elif label_code == "10":
        label_text = "drowsy"
    else:
        label_text = "unknown"

    return [
        f"raw_frame_{label_text}_{standard_name}.csv",
        f"raw_frame_{standard_name}.csv",
        f"{standard_name}.csv",
    ]


# ============================================================
# 5. 메인 분석
# ============================================================

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    by_fps_dir = OUTPUT_DIR / "by_fps"
    by_fps_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    skipped_rows = []

    person_dirs = [
        p for p in VIDEO_ROOT.iterdir()
        if p.is_dir()
    ]

    person_dirs = sorted(person_dirs, key=lambda x: x.name)

    print(f"사람 폴더 수: {len(person_dirs)}")

    for person_dir in person_dirs:
        person_id = person_dir.name

        video_files = [
            p for p in person_dir.iterdir()
            if p.is_file() and p.suffix.lower() in VIDEO_EXTS
        ]

        for video_path in video_files:
            label_code = extract_label_from_filename(video_path)

            if label_code is None:
                skipped_rows.append({
                    "reason": "label_not_found",
                    "person_folder": person_id,
                    "video_name": video_path.name,
                    "video_path": str(video_path),
                })
                continue

            if label_code not in USE_LABELS:
                skipped_rows.append({
                    "reason": f"excluded_label_{label_code}",
                    "person_folder": person_id,
                    "video_name": video_path.name,
                    "video_path": str(video_path),
                })
                continue

            standard_name = make_standard_name(label_code, person_id)

            if label_code == "0":
                label_name = "normal"
                label_int = 0
            elif label_code == "10":
                label_name = "drowsy"
                label_int = 1
            else:
                label_name = "unknown"
                label_int = None

            expected_csv_names = make_expected_csv_names(standard_name)

            info = get_video_info(video_path)

            row = {
                "person_folder": person_id,
                "person_id": person_id,
                "label_code": label_code,
                "label_name": label_name,
                "label_int": label_int,

                # 핵심: 나중에 매칭할 표준 이름
                "standard_name": standard_name,

                # 예: 0__19, 10__19
                "video_key": standard_name,

                "original_video_name": video_path.name,
                "original_video_stem": video_path.stem,
                "original_video_path": str(video_path),

                "expected_csv_name_1": expected_csv_names[0],
                "expected_csv_name_2": expected_csv_names[1],
                "expected_csv_name_3": expected_csv_names[2],
            }

            row.update(info)
            rows.append(row)

    if len(rows) == 0:
        print("분석할 0/10 라벨 영상이 없습니다.")
        return

    df = pd.DataFrame(rows)

    # ========================================================
    # 6. 전체 영상별 FPS 보고서 저장
    # ========================================================

    video_report_path = OUTPUT_DIR / "video_fps_report_0_10_only.csv"
    df.to_csv(video_report_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 7. FPS별 분포 저장
    # ========================================================

    fps_distribution = (
        df.groupby("fps_group", dropna=False)
        .agg(
            video_count=("standard_name", "count"),
            normal_count=("label_name", lambda x: (x == "normal").sum()),
            drowsy_count=("label_name", lambda x: (x == "drowsy").sum()),
            person_count=("person_id", "nunique"),
            avg_fps_raw=("fps_raw", "mean"),
            avg_total_frames=("total_frames", "mean"),
            avg_duration_sec=("duration_sec", "mean"),
            avg_sec_per_60_frames=("sec_per_60_frames", "mean"),
        )
        .reset_index()
        .sort_values("video_count", ascending=False)
    )

    fps_distribution_path = OUTPUT_DIR / "fps_distribution_0_10_only.csv"
    fps_distribution.to_csv(fps_distribution_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 8. FPS별 영상명 목록 저장
    # ========================================================

    fps_video_names = df[
        [
            "fps_group",
            "person_id",
            "label_code",
            "label_name",
            "label_int",
            "standard_name",
            "video_key",
            "original_video_name",
            "original_video_path",
            "fps_raw",
            "total_frames",
            "duration_sec",
            "sec_per_60_frames",
            "expected_csv_name_1",
            "expected_csv_name_2",
            "expected_csv_name_3",
        ]
    ].sort_values(["fps_group", "person_id", "label_code"])

    fps_video_names_path = OUTPUT_DIR / "fps_group_video_names_0_10_only.csv"
    fps_video_names.to_csv(fps_video_names_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 9. FPS 그룹별 개별 CSV 저장
    # ========================================================

    for fps_group in sorted(df["fps_group"].dropna().unique()):
        fps_group_int = int(fps_group)

        group_df = df[df["fps_group"] == fps_group].copy()
        group_df = group_df.sort_values(["person_id", "label_code"])

        save_path = by_fps_dir / f"fps_{fps_group_int}_videos.csv"
        group_df.to_csv(save_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 10. 사람별 normal/drowsy 보유 여부 저장
    # ========================================================

    person_label_summary = (
        df.groupby("person_id")
        .agg(
            video_count=("standard_name", "count"),
            normal_count=("label_name", lambda x: (x == "normal").sum()),
            drowsy_count=("label_name", lambda x: (x == "drowsy").sum()),
            fps_groups=("fps_group", lambda x: ",".join(map(str, sorted(set(x.dropna()))))),
            video_keys=("standard_name", lambda x: ",".join(sorted(x))),
        )
        .reset_index()
    )

    person_label_summary["has_both_0_and_10"] = (
        (person_label_summary["normal_count"] > 0) &
        (person_label_summary["drowsy_count"] > 0)
    )

    person_summary_path = OUTPUT_DIR / "person_label_summary_0_10_only.csv"
    person_label_summary.to_csv(person_summary_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 11. 5 라벨 / 라벨 추출 실패 영상 저장
    # ========================================================

    skipped_df = pd.DataFrame(skipped_rows)
    skipped_path = OUTPUT_DIR / "skipped_videos.csv"
    skipped_df.to_csv(skipped_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 12. 선택용 템플릿 저장
    # ========================================================

    select_template = df.copy()
    select_template["use_for_training"] = False

    select_template_path = OUTPUT_DIR / "select_video_template_0_10_only.csv"
    select_template.to_csv(select_template_path, index=False, encoding="utf-8-sig")

    # ========================================================
    # 13. 출력
    # ========================================================

    print("\n분석 완료")
    print(f"전체 0/10 영상 수: {len(df)}")
    print(f"사람 수: {df['person_id'].nunique()}")

    print("\nFPS별 분포")
    print(fps_distribution)

    print("\n저장 경로")
    print(f"1. 영상별 FPS 보고서:")
    print(f"   {video_report_path}")

    print(f"2. FPS별 분포:")
    print(f"   {fps_distribution_path}")

    print(f"3. FPS별 영상명 목록:")
    print(f"   {fps_video_names_path}")

    print(f"4. FPS별 개별 파일 폴더:")
    print(f"   {by_fps_dir}")

    print(f"5. 사람별 0/10 보유 여부:")
    print(f"   {person_summary_path}")

    print(f"6. 제외된 5 라벨 / 라벨 인식 실패 목록:")
    print(f"   {skipped_path}")

    print(f"7. 선택용 템플릿:")
    print(f"   {select_template_path}")


if __name__ == "__main__":
    main()
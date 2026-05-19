import os
import csv
import time
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


# ============================================================
# 1. 경로 설정
# ============================================================

VIDEO_DIR = r"C:\Users\KCCISTC\Desktop\피곤"

MODEL_PATH = r"C:\Users\KCCISTC\Desktop\VisionPoseCoach\WorkSpace\tasks\face_landmarker.task"

RAW_OUTPUT_CSV = os.path.join(VIDEO_DIR, "피곤_raw_blendshape_label1.csv")
GRU_OUTPUT_CSV = os.path.join(VIDEO_DIR, "피곤_GRU_window60_stride5_label1.csv")

LABEL = 1

WINDOW_SIZE = 60
STRIDE = 5


# ============================================================
# 2. 전체 Blendshape 컬럼
# ============================================================

BLENDSHAPE_COLUMNS = [
    "_neutral",
    "browDownLeft",
    "browDownRight",
    "browInnerUp",
    "browOuterUpLeft",
    "browOuterUpRight",
    "cheekPuff",
    "cheekSquintLeft",
    "cheekSquintRight",
    "eyeBlinkLeft",
    "eyeBlinkRight",
    "eyeLookDownLeft",
    "eyeLookDownRight",
    "eyeLookInLeft",
    "eyeLookInRight",
    "eyeLookOutLeft",
    "eyeLookOutRight",
    "eyeLookUpLeft",
    "eyeLookUpRight",
    "eyeSquintLeft",
    "eyeSquintRight",
    "eyeWideLeft",
    "eyeWideRight",
    "jawForward",
    "jawLeft",
    "jawOpen",
    "jawRight",
    "mouthClose",
    "mouthDimpleLeft",
    "mouthDimpleRight",
    "mouthFrownLeft",
    "mouthFrownRight",
    "mouthFunnel",
    "mouthLeft",
    "mouthLowerDownLeft",
    "mouthLowerDownRight",
    "mouthPressLeft",
    "mouthPressRight",
    "mouthPucker",
    "mouthRight",
    "mouthRollLower",
    "mouthRollUpper",
    "mouthShrugLower",
    "mouthShrugUpper",
    "mouthSmileLeft",
    "mouthSmileRight",
    "mouthStretchLeft",
    "mouthStretchRight",
    "mouthUpperUpLeft",
    "mouthUpperUpRight",
    "noseSneerLeft",
    "noseSneerRight",
]


# ============================================================
# 3. GRU에서 사용할 feature 4개
# ============================================================

GRU_FEATURE_COLUMNS = [
    "eyeBlinkLeft",
    "eyeBlinkRight",
    "eyeClosed",
    "jawOpen",
]


# ============================================================
# 4. 유틸 함수
# ============================================================

def format_time(seconds: float) -> str:
    seconds = max(0, int(seconds))

    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60

    if h > 0:
        return f"{h:02d}:{m:02d}:{s:02d}"

    return f"{m:02d}:{s:02d}"


def get_video_files(video_dir: str):
    video_exts = [".mp4", ".avi", ".mov", ".mkv", ".wmv", ".m4v"]

    video_files = []

    for path in Path(video_dir).rglob("*"):
        if path.suffix.lower() in video_exts:
            video_files.append(str(path))

    return sorted(video_files)


def create_face_landmarker():
    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)

    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        output_face_blendshapes=True,
        num_faces=1,
    )

    return vision.FaceLandmarker.create_from_options(options)


def make_raw_header():
    header = ["video_name", "frame_idx", "timestamp_sec"]
    header.extend(BLENDSHAPE_COLUMNS)
    header.append("label")
    return header


def make_gru_header():
    header = [
        "video_name",
        "window_id",
        "start_frame_idx",
        "end_frame_idx",
        "start_timestamp_sec",
        "end_timestamp_sec",
    ]

    for t in range(WINDOW_SIZE):
        for feature_name in GRU_FEATURE_COLUMNS:
            header.append(f"t{t:03d}_{feature_name}")

    header.append("label")

    return header


# ============================================================
# 5. 영상 하나 처리
# ============================================================

def process_video(
    video_path,
    raw_writer,
    gru_writer,
    file_index,
    total_files,
):
    video_name = os.path.basename(video_path)

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"\n[스킵] 영상을 열 수 없음: {video_path}")
        return 0, 0, 0

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    if fps <= 0:
        fps = 30.0

    print(f"\n[{file_index}/{total_files}] 처리 시작: {video_name}")
    print(f"총 프레임: {total_frames}, FPS: {fps:.2f}")

    frame_idx = 0
    saved_frame_count = 0
    skipped_frame_count = 0

    # GRU window 생성을 위해 얼굴 인식 성공한 프레임만 저장
    detected_gru_frames = []

    start_time = time.time()

    # 중요:
    # MediaPipe VIDEO 모드는 timestamp가 증가해야 하므로
    # 영상마다 landmarker를 새로 생성하는 방식이 안전함
    with create_face_landmarker() as landmarker:
        while True:
            ret, frame = cap.read()

            if not ret:
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_frame,
            )

            timestamp_ms = int((frame_idx / fps) * 1000)
            timestamp_sec = timestamp_ms / 1000.0

            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            if result.face_blendshapes and len(result.face_blendshapes) > 0:
                blendshape_dict = {
                    item.category_name: item.score
                    for item in result.face_blendshapes[0]
                }

                # --------------------------------------------
                # 1) raw frame CSV 저장
                # --------------------------------------------
                raw_row = [
                    video_name,
                    frame_idx,
                    timestamp_sec,
                ]

                for col in BLENDSHAPE_COLUMNS:
                    raw_row.append(blendshape_dict.get(col, 0.0))

                raw_row.append(LABEL)
                raw_writer.writerow(raw_row)

                saved_frame_count += 1

                # --------------------------------------------
                # 2) GRU용 4개 feature 저장
                # --------------------------------------------
                eye_blink_left = blendshape_dict.get("eyeBlinkLeft", 0.0)
                eye_blink_right = blendshape_dict.get("eyeBlinkRight", 0.0)
                jaw_open = blendshape_dict.get("jawOpen", 0.0)

                eye_closed = (eye_blink_left + eye_blink_right) / 2.0

                detected_gru_frames.append({
                    "frame_idx": frame_idx,
                    "timestamp_sec": timestamp_sec,
                    "eyeBlinkLeft": eye_blink_left,
                    "eyeBlinkRight": eye_blink_right,
                    "eyeClosed": eye_closed,
                    "jawOpen": jaw_open,
                })

            else:
                skipped_frame_count += 1

            frame_idx += 1

            # --------------------------------------------
            # 진행도 출력
            # --------------------------------------------
            if total_frames > 0 and (frame_idx % 30 == 0 or frame_idx == total_frames):
                elapsed = time.time() - start_time
                progress = frame_idx / total_frames
                remain = elapsed / progress - elapsed if progress > 0 else 0

                print(
                    f"\r진행률: {progress * 100:6.2f}% "
                    f"({frame_idx}/{total_frames}) | "
                    f"저장 프레임: {saved_frame_count} | "
                    f"스킵: {skipped_frame_count} | "
                    f"남은 시간: {format_time(remain)}",
                    end="",
                )

    cap.release()

    print()

    # ========================================================
    # 6. GRU window 생성
    # ========================================================

    gru_window_count = 0

    detected_count = len(detected_gru_frames)

    if detected_count < WINDOW_SIZE:
        print(f"GRU window 생성 불가: 얼굴 인식 성공 프레임 {detected_count}개")
    else:
        window_id = 0

        for start in range(0, detected_count - WINDOW_SIZE + 1, STRIDE):
            end = start + WINDOW_SIZE
            window = detected_gru_frames[start:end]

            start_frame_idx = window[0]["frame_idx"]
            end_frame_idx = window[-1]["frame_idx"]
            start_timestamp_sec = window[0]["timestamp_sec"]
            end_timestamp_sec = window[-1]["timestamp_sec"]

            gru_row = [
                video_name,
                window_id,
                start_frame_idx,
                end_frame_idx,
                start_timestamp_sec,
                end_timestamp_sec,
            ]

            for item in window:
                gru_row.append(item["eyeBlinkLeft"])
                gru_row.append(item["eyeBlinkRight"])
                gru_row.append(item["eyeClosed"])
                gru_row.append(item["jawOpen"])

            gru_row.append(LABEL)
            gru_writer.writerow(gru_row)

            window_id += 1
            gru_window_count += 1

    print(f"완료: {video_name}")
    print(f"저장된 raw 프레임: {saved_frame_count}")
    print(f"얼굴 미인식 스킵 프레임: {skipped_frame_count}")
    print(f"생성된 GRU window: {gru_window_count}")

    return saved_frame_count, skipped_frame_count, gru_window_count


# ============================================================
# 7. 메인 실행
# ============================================================

def main():
    video_files = get_video_files(VIDEO_DIR)

    if len(video_files) == 0:
        print("처리할 영상 파일이 없습니다.")
        return

    print(f"찾은 영상 개수: {len(video_files)}")
    print(f"Raw CSV 저장 위치: {RAW_OUTPUT_CSV}")
    print(f"GRU CSV 저장 위치: {GRU_OUTPUT_CSV}")
    print(f"GRU 설정: WINDOW_SIZE={WINDOW_SIZE}, STRIDE={STRIDE}, FEATURE=4")

    raw_header = make_raw_header()
    gru_header = make_gru_header()

    total_saved_frames = 0
    total_skipped_frames = 0
    total_gru_windows = 0

    whole_start = time.time()

    with open(RAW_OUTPUT_CSV, mode="w", newline="", encoding="utf-8-sig") as raw_f, \
         open(GRU_OUTPUT_CSV, mode="w", newline="", encoding="utf-8-sig") as gru_f:

        raw_writer = csv.writer(raw_f)
        gru_writer = csv.writer(gru_f)

        raw_writer.writerow(raw_header)
        gru_writer.writerow(gru_header)

        for idx, video_path in enumerate(video_files, start=1):
            saved, skipped, gru_windows = process_video(
                video_path=video_path,
                raw_writer=raw_writer,
                gru_writer=gru_writer,
                file_index=idx,
                total_files=len(video_files),
            )

            total_saved_frames += saved
            total_skipped_frames += skipped
            total_gru_windows += gru_windows

            elapsed = time.time() - whole_start
            avg_per_file = elapsed / idx
            remain_files = len(video_files) - idx
            remain_time = avg_per_file * remain_files

            print(
                f"전체 진행: {idx}/{len(video_files)} | "
                f"전체 raw 프레임: {total_saved_frames} | "
                f"전체 GRU window: {total_gru_windows} | "
                f"전체 남은 예상 시간: {format_time(remain_time)}"
            )

    print("\n전체 작업 완료")
    print(f"Raw CSV 저장 완료: {RAW_OUTPUT_CSV}")
    print(f"GRU CSV 저장 완료: {GRU_OUTPUT_CSV}")
    print(f"총 저장 raw 프레임: {total_saved_frames}")
    print(f"총 얼굴 미인식 스킵 프레임: {total_skipped_frames}")
    print(f"총 GRU window 개수: {total_gru_windows}")


if __name__ == "__main__":
    main()
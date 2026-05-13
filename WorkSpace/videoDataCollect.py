import cv2
import os
import csv
import mediapipe as mp
import numpy as np
from datetime import datetime
from tqdm import tqdm
from features import calculate_features # 함수명 확인

def main():
    # 1. 설정 및 경로
    base_path = "Pose"  # 최상위 폴더
    save_folder = "collected_data"
    os.makedirs(save_folder, exist_ok=True)
    
    # 라벨 매핑 설정
    label_mapping = {
        "정자세": 0,
        "비대칭": 1,
        "거북목": 2,
        "턱굄": 3
    }

    # 2. MediaPipe 초기화
    mp_pose = mp.solutions.pose
    pose_detector = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=2, # 라즈베리파이 5라면 1 또는 2 권장
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    all_data = []
    total_processed_files = 0

    print("🚀 비디오 배치 피처 추출을 시작합니다...")

    # 3. 폴더 순회 및 데이터 추출
    for folder_name, label_id in label_mapping.items():
        folder_path = os.path.join(base_path, folder_name)
        
        if not os.path.exists(folder_path):
            print(f"⚠️ 폴더 없음: {folder_path} (건너뜀)")
            continue

        video_files = [f for f in os.listdir(folder_path) if f.lower().endswith(('.mp4', '.avi', '.mov'))]
        print(f"📂 [{folder_name}] 처리 중... 파일 {len(video_files)}개 발견")

        for video_file in video_files:
            video_path = os.path.join(folder_path, video_file)
            cap = cv2.VideoCapture(video_path)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            
            # tqdm으로 진행률 표시
            with tqdm(total=total_frames, desc=f"  📹 {video_file[:15]}...") as pbar:
                while cap.isOpened():
                    ret, frame = cap.read()
                    if not ret: break

                    # RGB 변환 및 추론
                    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = pose_detector.process(img_rgb)

                    if results.pose_landmarks:
                        # 12개 피처 추출 (인자 구조 주의: 리스트로 감싸서 전달)
                        landmark_list = [results.pose_landmarks.landmark]
                        features = calculate_features(landmark_list)
                        
                        if features and any(features):
                            # 피처 리스트 끝에 라벨 추가
                            data_row = features + [label_id]
                            all_data.append(data_row)

                            # [추가] 실시간 샘플 카운트를 tqdm 옆에 표시
                            pbar.set_postfix(collected=len(all_data), label=folder_name)
                    
                    pbar.update(1)
            
            cap.release()
            total_processed_files += 1

    # 4. CSV 파일 저장 (파일명에 시간 및 샘플 수 포함)
    if all_data:
        total_samples = len(all_data)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 파일명 구성: 일시_총샘플수_데이터.csv
        filename = f"batch_collected_{timestamp}_{total_samples}samples.csv"
        filepath = os.path.join(save_folder, filename)
        
        # 헤더 정의 (12개 피처 + Label)
        header = ["F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12", "Label"]
        
        with open(filepath, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(all_data)
            
        print("\n" + "="*50)
        print(f"✅ 데이터 수집 완료!")
        print(f"📅 생성 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"🎥 처리된 비디오: {total_processed_files}개")
        print(f"📊 총 추출 샘플 수: {total_samples}개")
        print(f"💾 파일 저장 경로: {filepath}")
        print("="*50)
    else:
        print("❌ 추출된 데이터가 없습니다. 영상 상태를 확인하세요.")

    pose_detector.close()

if __name__ == "__main__":
    main()
import pandas as pd
import numpy as np
import os
import glob
from collections import deque

# --- 설정 상수 ---
WINDOW_SIZE = 60
STRIDE = 5
FEATURE_COLUMNS = ['eye_blink_left', 'eye_blink_right', 'eye_closed_score', 'jaw_open']
INPUT_DIR = 'face_csv'
OUTPUT_FILE = 'gru_face_dataset.csv'

def process_face_csv_files():
    all_sequences = []
    
    # 1. 대상 파일 리스트 확보
    file_list = glob.glob(os.path.join(INPUT_DIR, "*.csv"))
    print(f"총 {len(file_list)}개의 파일을 찾았습니다.")

    for file_path in file_list:
        file_name = os.path.basename(file_path)
        
        # 2. 파일명 기반 라벨 결정 (normal=0, drowsy=1)
        if 'normal' in file_name.lower():
            label = 0
        elif 'drowsy' in file_name.lower():
            label = 1
        else:
            continue # 정의되지 않은 라벨 파일은 건너뜀

        # 3. CSV 로드
        try:
            df = pd.read_csv(file_path)
            # 필요한 피처 열만 추출
            if not all(col in df.columns for col in FEATURE_COLUMNS):
                print(f"⚠️ {file_name}: 필수 컬럼이 부족하여 건너뜁니다.")
                continue
                
            features_data = df[FEATURE_COLUMNS].values # (프레임수, 4)
            
            # 4. 슬라이딩 윈도우 적용 (Deque 활용 구조)
            # mainGRU.py의 로직과 유사하게 큐를 사용하여 스트라이드만큼 이동하며 추출
            num_frames = len(features_data)
            if num_frames < WINDOW_SIZE:
                print(f"⚠️ {file_name}: 프레임 수({num_frames})가 윈도우 크기보다 작습니다.")
                continue

            # 0부터 (전체-윈도우)까지 STRIDE 간격으로 시작점 이동
            for start_idx in range(0, num_frames - WINDOW_SIZE + 1, STRIDE):
                # 60프레임 묶음 추출
                window = features_data[start_idx : start_idx + WINDOW_SIZE]
                
                # 2차원(60, 4) 데이터를 1차원(240,)으로 펼치기
                flattened_window = window.flatten()
                
                # 마지막에 라벨 추가
                sample = np.append(flattened_window, label)
                all_sequences.append(sample)
                
            print(f"✅ {file_name} 처리 완료 (생성된 샘플 수: {(num_frames - WINDOW_SIZE) // STRIDE + 1})")

        except Exception as e:
            print(f"❌ {file_name} 처리 중 오류 발생: {e}")

    # 5. 최종 데이터셋 저장
    if all_sequences:
        # 컬럼 이름 생성 (feat_0_0, feat_0_1 ... feat_59_3, label)
        col_names = []
        for i in range(WINDOW_SIZE):
            for feat in FEATURE_COLUMNS:
                col_names.append(f"f{i}_{feat}")
        col_names.append("label")

        result_df = pd.DataFrame(all_sequences, columns=col_names)
        result_df.to_csv(OUTPUT_FILE, index=False)
        print("-" * 30)
        print(f"🎉 모든 작업 완료!")
        print(f"최종 데이터셋 저장 경로: {OUTPUT_FILE}")
        print(f"총 샘플 개수: {len(result_df)}")
    else:
        print("추출된 데이터가 없습니다.")

if __name__ == "__main__":
    # face_csv 폴더가 있는지 확인 후 실행
    if not os.path.exists(INPUT_DIR):
        print(f"폴더를 찾을 수 없습니다: {INPUT_DIR}")
    else:
        process_face_csv_files()
import pandas as pd
import numpy as np
import joblib
import os
from tqdm import tqdm

def generate_final_face_dataset_with_stats(input_csv="face_frame_features_only.csv", output_csv="gru_face_final_caliFilter.csv"):
    # 1. 데이터 로드
    if not os.path.exists(input_csv):
        print(f"❌ 파일이 없습니다: {input_csv}")
        return

    df = pd.read_csv(input_csv)
    feature_cols = ['ear_l', 'ear_r', 'ear_avg', 'jaw_open']
    
    # 2. Baseline 보정
    normal_data = df[df['label'] == 0][feature_cols]
    if normal_data.empty:
        baseline = np.zeros(len(feature_cols))
    else:
        baseline = normal_data.mean().values
        print(baseline)
        joblib.dump(baseline, 'global_face_baseline.pkl')

    df_calibrated = df.copy()
    df_calibrated[feature_cols] = df[feature_cols] - baseline

    # 3. 설정
    WINDOW_SIZE = 60
    STRIDE_NORMAL = 5
    STRIDE_DROWSY = 2 
    EAR_DROP_THRESHOLD = -0.05 # 이 값보다 더 낮게(눈을 더 감게) 내려가야 함
    
    final_gru_data = []
    num_frames = len(df_calibrated)
    
    # --- 통계용 변수 초기화 ---
    stats = {
        "total_attempted_drowsy": 0,
        "discarded_drowsy": 0,
        "kept_drowsy": 0,
        "kept_normal": 0
    }

    print(f"🔄 보정 및 필터링 시작 (임계값: {EAR_DROP_THRESHOLD})...")

    idx = 0
    while idx <= num_frames - WINDOW_SIZE:
        window_df = df_calibrated.iloc[idx : idx + WINDOW_SIZE]
        current_label = int(window_df.iloc[-1]['label'])
        window_features = window_df[feature_cols].values
        
        is_valid = True
        
        if current_label == 1: # 졸음 데이터인 경우 필터링 검사
            stats["total_attempted_drowsy"] += 1
            max_drop = np.min(window_features[:, 2]) # ear_avg의 최솟값
            
            if max_drop > EAR_DROP_THRESHOLD: # 눈을 충분히 감지 않음
                is_valid = False
                stats["discarded_drowsy"] += 1
            else:
                stats["kept_drowsy"] += 1
        else:
            stats["kept_normal"] += 1
        
        if is_valid:
            flattened = window_features.flatten()
            final_gru_data.append(np.append(flattened, current_label))
            idx += STRIDE_DROWSY if current_label == 1 else STRIDE_NORMAL
        else:
            # 필터링된 졸음 구간은 다음 유효 구간을 빨리 찾기 위해 1프레임씩 이동
            idx += 1 

    # 4. 결과 저장
    cols = [f"{feat}_t{t}" for t in range(1, WINDOW_SIZE+1) for feat in feature_cols] + ["label"]
    result_df = pd.DataFrame(final_gru_data, columns=cols)
    result_df.to_csv(output_csv, index=False, encoding='utf-8-sig')

    # 5. [중요] 필터링 통계 출력
    print("\n" + "="*60)
    print("📊 데이터 정제 결과 보고서")
    print("-"*60)
    print(f"✅ 정상 데이터(Normal) 생성 수: {stats['kept_normal']} set")
    print(f"🚀 졸음 데이터(Drowsy) 분석 결과:")
    print(f"   - 총 시도된 졸음 윈도우: {stats['total_attempted_drowsy']} 회")
    print(f"   - 필터링되어 버려진 윈도우: {stats['discarded_drowsy']} 회 (눈 안 감음)")
    print(f"   - 최종 승인된 졸음 윈도우: {stats['kept_drowsy']} 회")
    
    if stats['total_attempted_drowsy'] > 0:
        discard_rate = (stats['discarded_drowsy'] / stats['total_attempted_drowsy']) * 100
        print(f"   - 졸음 데이터 유효 추출률: {100 - discard_rate:.1f}%")
        print(f"   - 데이터 탈락률: {discard_rate:.1f}%")
    
    print("-"*60)
    print(f"📁 최종 저장 파일: {output_csv}")
    print(f"💾 보정값 저장: global_face_baseline.pkl")
    print("="*60)

if __name__ == "__main__":
    generate_final_face_dataset_with_stats()
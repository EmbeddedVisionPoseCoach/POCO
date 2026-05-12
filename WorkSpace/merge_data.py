import pandas as pd
import glob
import os
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime

def process_and_merge_data(source_folder, output_folder):
    # 1. 출력 폴더 생성
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"📂 새 폴더 생성 완료: {output_folder}")

    # 2. 모든 CSV 파일 로드 및 병합
    file_list = glob.glob(os.path.join(source_folder, "*.csv"))
    if not file_list:
        print(f"❌ '{source_folder}'에 데이터가 없습니다.")
        return

    df_list = [pd.read_csv(f) for f in file_list]
    df = pd.concat(df_list, ignore_index=True)
    
    # 3. 클래스 밸런스 확인 (시각화 및 출력)
    print("\n" + "="*30)
    print("📊 클래스별 데이터 분포")
    print("-" * 30)
    
    # 라벨 매핑 (가독성용)
    label_names = {0: "Optimal", 1: "Forward", 2: "Asymmetric", 3: "Propping"}
    counts = df['Label'].value_counts().sort_index()
    
    for idx, count in counts.items():
        name = label_names.get(idx, f"Label {idx}")
        print(f"• {name:12}: {count} samples")
    
    # 그래프 시각화 (선택 사항)
    plt.figure(figsize=(10, 6))
    sns.barplot(x=[label_names.get(i, i) for i in counts.index], y=counts.values, palette='viridis')
    plt.title('Class Distribution of Posture Data')
    plt.xlabel('Posture Labels')
    plt.ylabel('Number of Samples')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # 그래프 저장
    plot_path = os.path.join(output_folder, 'class_distribution.png')
    plt.savefig(plot_path)
    print(f"\n📈 분포 그래프 저장 완료: {plot_path}")

    # 4. 랜덤 셔플링 (Shuffling)
    # frac=1은 전체 데이터를 무작위로 추출함을 의미하며, random_state는 결과 재현성을 위해 설정합니다.
    print("\n🔄 데이터 무작위 셔플링 중...")
    df_shuffled = df.sample(frac=1, random_state=42).reset_index(drop=True)

    # 5. 결과 저장
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    total_samples = len(df_shuffled)
    output_filename = f"shuffled_dataset_{timestamp}_{total_samples}samples.csv"
    output_path = os.path.join(output_folder, output_filename)
    
    df_shuffled.to_csv(output_path, index=False, encoding='utf-8-sig')
    
    print("="*50)
    print(f"✅ 전처리 완료!")
    print(f"📂 최종 데이터: {output_path}")
    print(f"📊 총 데이터: {total_samples}개")
    print("="*50)

if __name__ == "__main__":
    SOURCE = "collected_data"
    OUTPUT = "processed_data"
    
    process_and_merge_data(SOURCE, OUTPUT)
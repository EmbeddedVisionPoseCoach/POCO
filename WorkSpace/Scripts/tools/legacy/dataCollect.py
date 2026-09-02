import cv2
import time
import csv
import os
from datetime import datetime

# 기존에 작성한 모듈들을 임포트합니다.
from modules.camera import CameraStream
from detector import LandmarkerDetector
from modules.visualizer import Visualizer
from modules.features import calculate_features
import mediapipe as mp

def main():
    # 1. 사용자로부터 라벨 정보를 입력받습니다.
    print("="*50)
    print(" [Posture Data Collector] ")
    print("="*50)
    print("0: Optimal(정자세)")
    print("1: Forward(거북목)")
    print("2: Asymmetric(비대칭)")
    print("3: Propping(턱괴기)")
    print("-"*50)
    
    try:
        label = int(input("수집할 라벨 번호를 입력하고 Enter를 누르세요: "))
    except ValueError:
        print("❌ 에러: 숫자만 입력 가능합니다.")
        return

    label_map = {
        0: "Optimal(정자세)",
        1: "Forward(거북목)",
        2: "Asymmetric(비대칭)",
        3: "Propping(턱괴기)"
    }
    
    if label not in label_map:
        print("❌ 에러: 0, 1, 2, 3 중 하나를 입력해야 합니다.")
        return

    label_name = label_map[label]
    data_list = [] # 프레임별 피처 데이터가 담길 리스트

    # 2. 데이터 저장 폴더 준비
    save_folder = "collected_data"
    os.makedirs(save_folder, exist_ok=True)

    # 3. 분석 모듈 초기화
    detector = LandmarkerDetector()
    visualizer = Visualizer()
    stream = CameraStream(src=0).start()
    
    # 윈도우 생성 및 위치 설정
    win_name = "Data Collection Mode"
    cv2.namedWindow(win_name)
    cv2.moveWindow(win_name, 100, 100)
    
    print(f"\n🎬 [{label_name}] 데이터 수집을 시작합니다.")
    print("👉 종료하고 저장하려면 웹캠 창에서 'q'를 누르세요.")
    
    start_time = time.time()
    prev_time = 0

    try:
        while True:
            frame = stream.read()
            if frame is None:
                continue

            # 전처리: 좌우 반전 및 MediaPipe용 변환
            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # 랜드마크 탐지 및 8대 핵심 피처 계산
            face_res, pose_res = detector.detect(mp_image)
            current_features = calculate_features(pose_res.pose_landmarks)
            
            # 포즈가 정상적으로 탐지된 프레임만 데이터 리스트에 추가
            if pose_res.pose_landmarks:
                # [F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, Label] 형태의 한 행 구성
                row = current_features + [label]
                data_list.append(row)

            # 실시간 FPS 및 시각화 처리
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time
            
            # 영상 위에 랜드마크 및 안내 텍스트 출력
            display_frame = visualizer.draw_webcam(frame, face_res, pose_res, fps)
            
            # 수집 정보 UI 상단 표시
            status_text = f"REC: {label_name}"
            count_text = f"Samples: {len(data_list)}"
            cv2.putText(display_frame, status_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(display_frame, count_text, (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            
            cv2.imshow(win_name, display_frame)

            # 'q' 키를 누르면 수집 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"⚠️ 실행 중 오류 발생: {e}")

    finally:
        # 종료 시 통계 계산
        end_time = time.time()
        duration = int(end_time - start_time)
        data_count = len(data_list)
        
        # 4. CSV 파일 저장 실행
        if data_count > 0:
            # 파일명 형식: 일시_라벨번호_라벨명_수집시간_샘플수.csv
            timestamp = datetime.now().strftime("%Y%m%d_%H%M")
            filename = f"{timestamp}_{label}_{label_name}_{duration}s_{data_count}samples.csv"
            filepath = os.path.join(save_folder, filename)
            
            # CSV 파일 작성 (한글 깨짐 방지를 위해 utf-8-sig 사용)
            header = ["F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "Label"]
            with open(filepath, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data_list)
            
            print("\n" + "="*50)
            print(f"✅ 데이터 저장 완료!")
            print(f"📂 경로: {filepath}")
            print(f"📊 수집 시간: {duration}초 / 샘플 수: {data_count}개")
            print("="*50)
        else:
            print("\nℹ️ 수집된 데이터가 없어 파일을 생성하지 않았습니다.")

        # 자원 해제
        stream.stop()
        detector.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
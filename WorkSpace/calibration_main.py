import cv2
import numpy as np
import time
import joblib
import os
import mediapipe as mp

# 병현님의 기존 모듈들
from camera import CameraStream
from features import calculate_features
from visualizer import Visualizer
import config

def main():
    # 1. 초기 설정
    cam = CameraStream(src=0).start()
    viz = Visualizer()
    
    # MediaPipe 설정
    mp_pose = mp.solutions.pose
    pose_detector = mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7)

    # 데이터 수집용 버퍼 및 시간 설정
    feature_buffer = []
    calibration_duration = 5  # 5초 동안 데이터 수집 (config.CALIBRATION_TIME 사용 권장)
    start_time = None
    is_calibrating = False

    print("="*50)
    print(" [Calibration Mode] ")
    print(" 'S' 키를 누르면 5초간 바른 자세 측정을 시작합니다.")
    print(" ESC 키를 누르면 종료합니다.")
    print("="*50)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue

            frame = cv2.flip(frame, 1)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose_detector.process(img_rgb)

            h, w, _ = frame.shape

            # 랜드마크가 검출되었을 때
            if results.pose_landmarks:
                viz.draw_landmarks(frame, results.pose_landmarks)
                
                # 측정 시작 상태일 때 데이터 수집
                if is_calibrating:
                    elapsed = time.time() - start_time
                    
                    # 12개 피처 추출
                    landmark_list = [results.pose_landmarks.landmark]
                    current_features = calculate_features(landmark_list)
                    
                    if any(current_features):
                        feature_buffer.append(current_features)
                    
                    # 진행률 표시 (Visualizer에 해당 기능이 있다면 활용, 없으면 여기서 직접 그림)
                    progress = int((elapsed / calibration_duration) * 100)
                    cv2.rectangle(frame, (100, h-50), (w-100, h-30), (50, 50, 50), -1)
                    cv2.rectangle(frame, (100, h-50), (100 + int((w-200)*(progress/100)), h-30), (0, 255, 0), -1)
                    cv2.putText(frame, f"Calibrating... {progress}%", (w//2 - 80, h-60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # ---------------------------------------------------------
                    # [추후 웹캠 모터 각도 조절 로직이 들어갈 자리]
                    # 예: baseline 측정 전에 수평을 맞추는 동작 등
                    # ---------------------------------------------------------

                    if elapsed >= calibration_duration:
                        is_calibrating = False
                        break # 루프 종료 및 저장 단계로 이동

            # 화면 안내 메시지
            if not is_calibrating:
                cv2.putText(frame, "Press 'S' to Start Calibration", (w//2 - 180, h//2), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow('Calibration - Set Baseline', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and not is_calibrating:
                print("🚀 측정을 시작합니다. 바른 자세를 유지하세요!")
                feature_buffer = []
                start_time = time.time()
                is_calibrating = True
            elif key == 27: # ESC
                break

        # 2. 평균값 계산 및 저장
        if feature_buffer:
            # 12개 피처의 평균 계산
            baseline_avg = np.mean(feature_buffer, axis=0)
            
            # 폴더 생성 및 저장
            os.makedirs('saved_model', exist_ok=True)
            save_path = 'saved_model/baseline.pkl'
            joblib.dump(baseline_avg, save_path)
            
            print("\n" + "="*50)
            print(f"✅ Calibration 완료!")
            print(f"📍 저장 경로: {save_path}")
            print(f"📊 수집된 샘플 수: {len(feature_buffer)}")
            print(f"📝 평균 피처 값: \n{baseline_avg}")
            print("="*50)
        else:
            print("\n❌ 수집된 데이터가 없습니다. 다시 시도해주세요.")

    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
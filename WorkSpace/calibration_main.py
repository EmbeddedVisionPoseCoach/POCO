import cv2
import numpy as np
import time
import joblib
import os
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# 병현님의 기존 모듈들
from modules.camera import CameraStream
from modules.features import calculate_features , calculate_face_features_for_window
from modules.visualizer import Visualizer
import modules.config as config

def main():
    # 1. 초기 설정
    cam = CameraStream(src=0).start()
    viz = Visualizer()
    
    # MediaPipe 설정
    

    # MediaPipe FaceLandmarker task 파일 경로
    # MediaPipe FaceLandmarker task 파일 경로
    face_task_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "tasks", "face_landmarker.task")
    )

    print("FaceLandmarker task path:", face_task_path)

    if not os.path.exists(face_task_path):
        raise FileNotFoundError(f"face_landmarker.task 파일을 찾을 수 없습니다: {face_task_path}")

    # 핵심 수정:
    # Windows 절대경로를 model_asset_path로 넘기면 MediaPipe가 경로를 잘못 해석할 수 있음.
    # 따라서 파일을 직접 읽어서 model_asset_buffer로 전달.
    with open(face_task_path, "rb") as f:
        face_task_buffer = f.read()

    base_options = python.BaseOptions(
        model_asset_buffer=face_task_buffer
    )

    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        output_face_blendshapes=True,
        num_faces=1
    )

    face_detector = vision.FaceLandmarker.create_from_options(options)
    
    mp_pose = mp.solutions.pose
    pose_detector = mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7)

    face_detector = vision.FaceLandmarker.create_from_options(options)
    
    # 데이터 수집용 버퍼 및 시간 설정
    pose_buffer = []
    face_buffer = []
    calibration_duration = config.CALIBRATION_TIME  # 5초 동안 데이터 수집 (config.CALIBRATION_TIME 사용 권장)
    start_time = None
    is_calibrating = False

    print("="*50)
    print(" [Calibration Mode] ")
    print(f" 'S' 키를 누르면 {calibration_duration}초간 바른 자세 측정을 시작합니다.")
    print(" ESC 키를 누르면 종료합니다.")
    print("="*50)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)

            results_pose = pose_detector.process(img_rgb)
            results_face = face_detector.detect(mp_image)

            

            # 랜드마크가 검출되었을 때
            if results_pose.pose_landmarks:
                viz.draw_landmarks(frame, results_pose.pose_landmarks)
                
                # 측정 시작 상태일 때 데이터 수집
                if is_calibrating:
                    elapsed = time.time() - start_time
                    
                    # 10개 피처 추출
                    landmark_list = [results_pose.pose_landmarks.landmark]
                    current_pose_features = calculate_features(landmark_list)

                    # 얼굴 blendshape 피처 추출
                    current_face_features = None

                    if results_face.face_blendshapes:
                        current_face_features = calculate_face_features_for_window(results_face.face_blendshapes)

                    if current_face_features is not None and len(current_face_features) == config.FACE_FEATURE_SIZE:
                        face_buffer.append(current_face_features)
                    
                    if any(current_pose_features):
                        pose_buffer.append(current_pose_features)
                    
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
                pose_buffer = []
                face_buffer = []
                start_time = time.time()
                is_calibrating = True
            elif key == 27: # ESC
                break

        # 2. 평균값 계산 및 저장
        if pose_buffer:
            # 12개 피처의 평균 계산
            baseline_avg = np.mean(pose_buffer, axis=0)
            
            # 폴더 생성 및 저장
            os.makedirs('saved_model', exist_ok=True)
            save_path = 'saved_model/baseline.pkl'
            joblib.dump(baseline_avg, save_path)
            
            print("\n" + "="*50)
            print(f"✅ Calibration 완료!")
            print(f"📍 저장 경로: {save_path}")
            print(f"📊 수집된 샘플 수: {len(pose_buffer)}")
            print(f"📝 평균 피처 값: \n{baseline_avg}")
            print("="*50)
        else:
            print("\n❌ 수집된 데이터가 없습니다. 다시 시도해주세요.")

        if face_buffer:
            face_baseline_avg = np.mean(face_buffer, axis=0)
            face_save_path = 'saved_model/face_baseline.pkl'
            joblib.dump(face_baseline_avg, face_save_path)
            print(f"📍 얼굴 피처 저장 경로: {face_save_path}")
            print(f"📝 얼굴 평균 피처 값: \n{face_baseline_avg}")

    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
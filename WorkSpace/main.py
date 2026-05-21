import cv2
import numpy as np
import mediapipe as mp
from collections import deque
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import time

# 모듈들 임포트
from modules.camera import CameraStream
from modules.features import calculate_face_feature, calculate_features  # 11개 피처 버전
from modules.visualizer import Visualizer
import modules.config as config 
from modules.logger import StudyLogger  # 로그 기록 모듈 추가
from modules.TFLiteEngine import TFLiteEngine  # TFLite 추론 엔진 모듈 추가



def main():

    # 라벨링 출력용 창 설정
    
    label_dashboard = np.zeros((config.LABEL_DASHBOARD_HEIGHT, config.LABEL_DASHBOARD_WIDTH, 3), dtype=np.uint8)
    label_dashboard.fill(20)  # 배경색: 짙은 회색
    margin_x = config.LABEL_MARGIN_X

    # Blendshape 출력용 옵션
    base_options = python.BaseOptions(model_asset_path=config.FACE_MODEL_PATH) 
    options = vision.FaceLandmarkerOptions(
    base_options=base_options,
    output_face_blendshapes=True,  
    num_faces=1
    )
    face_detector = vision.FaceLandmarker.create_from_options(options)
    
    # 얼굴용 데이터 구조========================
   
    window_start_time = time.time()
    prev_time = window_start_time
    face_prediction = 0 
    # 30초 동안 face_res.face_blendshapes[0]을 저장하는 리스트
    blendshape_window = []

    # 최근 예측 결과 저장
    latest_result = None

    # 최근 30초 feature 저장
    latest_features = None
    # =======================================

    # 1. 인스턴스 초기화
    cam = CameraStream(src=0).start() 
    viz = Visualizer()
    # config.py에 정의된 경로 사용
    engine = TFLiteEngine(config.MODEL_PATH, config.MODEL_FACE_PATH, config.SCALER_PATH, config.SCALER_FACE_PATH, config.BASELINE_PATH)
    
    # 로그 기록을 위한 StudyLogger 인스턴스 생성
    logger = StudyLogger()

    # 2. MediaPipe 탐지기 설정
    mp_pose = mp.solutions.pose
    mp_face_mesh = mp.solutions.face_mesh
    
    pose_detector = mp_pose.Pose(
        min_detection_confidence=0.5, 
        min_tracking_confidence=0.5
    )
   

    # 3. 라벨 및 색상 정의 (학습 시 인덱스 순서와 일치해야 함)
    # 0: 정자세, 1: 비대칭, 2: 거북목, 3: 굄 (예시 순서)
    labels = config.POSTURE_LABELS
    colors = config.POSTURE_COLORS

    print("🚀 실시간 자세 분석 시스템 가동 중... (추론 모드)")

    prediction_queue = deque(maxlen=config.LABEL_FRAME)  # 최근 LABEL_FRAME개 예측 저장
                                                        #

    try:
        while True:
            # 프레임 읽기
            frame = cam.read()
            if frame is None:
                continue
            # 매 프레임마다 대시보드 도화지를 깨끗하게 지웁니다 ---
            label_dashboard.fill(20)

            # 30초 blenshape 수집용
            curr_time = time.time()

            dt = curr_time - prev_time
            
            if dt > 0:
                fps = 1.0 / dt
            else:
                fps = 0.00

            prev_time = curr_time

            # 전처리 (좌우 반전 및 RGB 변환)
            frame = cv2.flip(frame, 1)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 랜드마크 추론
            results_pose = pose_detector.process(img_rgb)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
            results_face = face_detector.detect(mp_image)

            # 기본 빈 대시보드 생성 (데이터 없을 때 표시용)
            dashboard = np.zeros((config.DASHBOARD_HEIGHT, config.DASHBOARD_WIDTH, 3), dtype=np.uint8)
            cv2.putText(dashboard, "Waiting for Pose Detection...", (50, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 1)
            
            if results_face.face_blendshapes and len(results_face.face_blendshapes) > 0:
                blendshape_window.append(results_face.face_blendshapes[0])
                
            elapsed_sec = curr_time - window_start_time

            if elapsed_sec >= config.WINDOW_SEC :
                face_features = calculate_face_feature(blendshape_window)  

                if face_features is None:
                    print("[WARN] 유효한 blendshape가 부족해서 예측을 건너뜁니다.")
                    latest_features = None

                else : 
                    latest_features = face_features

                    scaled_face_features = engine.face_scaler.transform(np.array(face_features).reshape(1, -1)).astype(np.float32)
                    face_prediction = engine.predict_face(scaled_face_features)

                # 30초가 지나면 blendshape 윈도우 초기화 및 타이머 리셋
                blendshape_window.clear()
                window_start_time = time.time()

            if results_pose.pose_landmarks :
                viz.draw_landmarks(frame, results_pose.pose_landmarks, results_face.face_landmarks[0])
                # [Features] 11개 피처 추출
                landmark_list = [results_pose.pose_landmarks.landmark]
                raw_features = calculate_features(landmark_list)
                
                hand_visible = raw_features[-1] 

            
                
            
            
            
            
                if any(raw_features):
                    # [AI 추론] 모든 클래스의 확률값 획득
                    probs = engine.predict(raw_features)
                    
                    if hand_visible == 0:
                        probs[3] = 0
                    
                    
                    
                    # 가장 높은 확률 정보 추출 (메인 화면 표시용)
                    if face_prediction is not None:
                        class_face_idx = 1 if face_prediction >= 0.5 else 0 
                        confidence_face = face_prediction
                        status_face_text = "Drowsy" if class_face_idx == 1 else "Normal"
                    class_idx = np.argmax(probs)

                    
                    confidence = probs[class_idx]
                    

                    status_text = labels[class_idx]
                    
                    
                    

                    prediction_queue.append(class_idx)
                    final_label= max(set(prediction_queue), key=prediction_queue.count)
                    final_status_text = labels[final_label]
                    status_color = colors[final_label]

                    # --- [로그 저장 로직 추가] ---
                    # data detail.png의 포즈 관련 데이터 매핑
                    UI_log = {
                        "posture_type": labels[final_label],
                        "fatigue_label": status_face_text,
                        "fatigue_probability": confidence_face
                    }
                    logger.save(UI_log)
                    # --------------------------
                    
                    
                    # [Visualizer] 모든 라벨의 확률 대시보드 생성 (별도 창)
                    dashboard = viz.draw_confidence_dashboard(probs, labels, colors)

                    # 메인 뷰 상단 상태 표시
                    cv2.rectangle(frame, (0, 0), (120, 60), (0, 0, 0), -1)
                    # cv2.putText(frame, f"{final_status_text}", (10, 40), 
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
                    
                    # 라벨링 전용 출력창
                    # A. 실시간 포즈 라벨 (상단)
                    cv2.putText(label_dashboard, "POSTURE STATUS", (margin_x, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
                    cv2.putText(label_dashboard, f"{final_status_text}", (margin_x, 110), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.4, status_color, 3)

                    # B. 얼굴 분석 라벨 (중단 - 30초 주기 업데이트)
                    face_ui_color = (0, 0, 255) if class_face_idx == 1 else (0, 255, 0)
                    cv2.putText(label_dashboard, "FACE STATUS", (margin_x, 180), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
                    cv2.putText(label_dashboard, f"{status_face_text}", (margin_x, 240), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.4, face_ui_color, 3)
                    
                    # C. 다음 업데이트 남은 시간 (하단)
                    curr_elapsed = time.time() - window_start_time
                    rem_time = max(0, config.WINDOW_SEC - curr_elapsed)
                    progress = min(curr_elapsed / config.WINDOW_SEC, 1.0)

                    # 타이머 텍스트
                    timer_text = f"Next Face Update: {rem_time:.1f}s"
                    cv2.putText(label_dashboard, timer_text, (margin_x, 310), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

                    # 심플 프로그레스 라인 (남은 시간을 시각적으로만 보조)
                    line_y = 330
                    cv2.line(label_dashboard, (margin_x, line_y), (config.LABEL_DASHBOARD_WIDTH - margin_x, line_y), (50, 50, 50), 2)
                    cv2.line(label_dashboard, (margin_x, line_y), (margin_x + int((config.LABEL_DASHBOARD_WIDTH - 2*margin_x) * progress), line_y), (255, 165, 0), 2)



            # 두 개의 결과 창 출력
            cv2.imshow('Landmark View', frame)
            cv2.imshow('Confidence Analysis', dashboard)
            cv2.imshow('Status Dashboard', label_dashboard)

            # ESC 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
    finally:
        # 자원 해제
        print("🔌 시스템을 종료합니다.")
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
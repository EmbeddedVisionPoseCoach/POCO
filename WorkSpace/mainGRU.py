import cv2
import numpy as np
import mediapipe as mp
from collections import deque
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import time

# 모듈들 임포트
from modules.camera import CameraStream
from modules.features import calculate_face_features_for_window, calculate_features  # 10개 피처 버전
from modules.visualizer import Visualizer
import modules.config as config 
from modules.logger import StudyLogger  # 로그 기록 모듈 추가
from modules.TFLiteEngine import TFLiteEngine  # TFLite 추론 엔진 모듈 추가


def main():
    # 인스턴스 초기화
    cam = CameraStream(src=0).start()
    viz = Visualizer()
    
    # config.py에 정의된 경로 사용
    if config.USE_POSE:
        engine = TFLiteEngine(
            config.MODEL_PATH,
            config.MODEL_FACE_PATH,
            config.SCALER_PATH,
            config.SCALER_FACE_PATH,
            config.BASELINE_PATH,
            config.FACE_BASELINE_PATH,
            config.THRESHOLD_PATH
        )
    else:
        engine = TFLiteEngine(
            None,
            config.MODEL_FACE_PATH,
            None,
            config.SCALER_FACE_PATH,
            None,
            config.FACE_BASELINE_PATH,
            config.THRESHOLD_PATH
        )
    
    # 로그 기록을 위한 StudyLogger 인스턴스 생성
    logger = StudyLogger()

    # 2. MediaPipe 탐지기 설정
    base_options = python.BaseOptions(model_asset_path=config.FACE_MODEL_PATH) 
    
    options = vision.FaceLandmarkerOptions(
    base_options=base_options,
    output_face_blendshapes=True,  
    num_faces=1
    )

    pose_detector = None

    if config.USE_POSE:
        mp_pose = mp.solutions.pose

        pose_detector = mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    face_detector = vision.FaceLandmarker.create_from_options(options)

    # 큐 초기화
    pose_feature_window = deque(maxlen=config.WINDOW_SIZE)  # GRU 모델이 요구하는 시퀀스 길이로 설정
    face_feature_window = deque(maxlen=config.WINDOW_SIZE)  # 얼굴 피처용 큐 추가
    
    # 상태 관리 변수
    frame_count = 0
    
    current_color = (255, 255, 255) # 기본 흰색 문자

    # 3. 디스플레이를 위한 윈도우 창 분리 생성 및 크기 설정
    cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Stream", config.FRAME_WIDTH, config.FRAME_HEIGHT)
    
    cv2.namedWindow("Label Status", cv2.WINDOW_NORMAL)

    label_dashboard = np.zeros(
        (config.LABEL_DASHBOARD_HEIGHT, config.LABEL_DASHBOARD_WIDTH, 3),
        dtype=np.uint8
    )

    cv2.putText(
        label_dashboard,
        "Collecting face frames...",
        (50, 180),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (200, 200, 200),
        1
    )

    cv2.resizeWindow("Label Status", config.LABEL_DASHBOARD_WIDTH, config.LABEL_DASHBOARD_HEIGHT)

    print(f"대기 중... 얼굴 피처 개수({config.FACE_FEATURE_SIZE}개) 기반 최초 {config.WINDOW_SIZE}프레임 수집 후 추론이 시작됩니다.")

    while True:
        frame = cam.read()

        if frame is None:
            continue

        frame_count += 1


        

        

        # 전처리 (좌우 반전 및 RGB 변환)
        frame = cv2.flip(frame, 1)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 랜드마크 추론
        results_pose = None

        if config.USE_POSE:
            results_pose = pose_detector.process(img_rgb)

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        results_face = face_detector.detect(mp_image)

        # [포즈 피처 추출] 매 프레임마다 포즈 피처 추출
        # 현재는 USE_POSE=False라 실행되지 않음

        if config.USE_POSE and results_pose is not None and results_pose.pose_landmarks is not None:
            landmark_list = [results_pose.pose_landmarks.landmark]
            pose_features = calculate_features(landmark_list)

            if pose_features is not None and len(pose_features) == config.POSE_FEATURE_SIZE:
                pose_features = np.array(pose_features, dtype=np.float32)
                pose_features = pose_features - engine.baseline
            else:
                pose_features = np.zeros(config.POSE_FEATURE_SIZE, dtype=np.float32)

            pose_feature_window.append(pose_features)
        

        # [얼굴 피쳐 추출] 매 프레임마다 얼굴 피처 추출
        # [얼굴 피처 추출] FaceLandmarker blendshape에서 4개 feature 추출
        face_features = None
        face_features_raw = None

        if results_face.face_blendshapes:
            face_features = calculate_face_features_for_window(results_face.face_blendshapes)

        if face_features is not None and len(face_features) == config.FACE_FEATURE_SIZE:
            face_features_raw = np.array(face_features, dtype=np.float32)
            face_features = face_features_raw - engine.face_baseline
        else:
            continue  # 얼굴 피처가 유효하지 않으면 다음 프레임으로 넘어감

        face_feature_window.append(face_features)


        if len(face_feature_window) == config.WINDOW_SIZE and (frame_count % config.STRIDE == 0):
            # 모델 입력 형태로 변환
            # Face 모델 입력 형태로 변환
            model_input_face = np.array(face_feature_window, dtype=np.float32)  # [60, 4]

            # 학습 때 사용한 scaler 적용
            if engine.face_scaler is not None:
                model_input_face = engine.face_scaler.transform(model_input_face)

            # GRU 입력 형태: [1, 60, 4]
            input_tensor_face = np.expand_dims(model_input_face, axis=0)

            # Face GRU 추론
            face_label_idx, drowsy_probability = engine.predict_face(input_tensor_face)
            face_label = config.FACE_LABELS[face_label_idx]

            # ============================================================
            # [터미널 디버그 출력]
            # 졸음 확률과 최근 프레임의 얼굴 피처값 출력
            # ============================================================

            latest_face_features = face_feature_window[-1]

            feature_names = [
                "eye_blink_left",
                "eye_blink_right",
                "eye_closed_score",
                "jaw_open"
            ]

            print("\n" + "=" * 70)
            print(f"[FACE RESULT] Label: {face_label}")
            print(f"[FACE RESULT] Drowsy Probability: {drowsy_probability:.4f}")
            print(f"[FACE RESULT] Threshold: {engine.face_threshold:.4f}")
            print("-" * 70)
            print("[RAW FEATURE] baseline 보정 전")

            for name, value in zip(feature_names, face_features_raw):
                print(f"{name}: {float(value):.6f}")

            print("-" * 70)
            print("[CORRECTED FEATURE] baseline 보정 후")

            for name, value in zip(feature_names, latest_face_features):
                print(f"{name}: {float(value):.6f}")

            print("=" * 70)

            # Pose는 현재 비활성화
            pose_label = "Disabled"
            pose_confidence = 0.0
            current_color = (180, 180, 180)

            # 로그 저장 로직 -------------
            
            UI_log = {
                "posture_type": pose_label,
                "fatigue_label": face_label,
                "fatigue_probability": drowsy_probability
            }
            logger.save(UI_log)
            # --------------------------

            # [Visualizer]
            # 라벨과 색상 업데이트
            current_label = f"Pose: {pose_label} ({pose_confidence:.2f}), Face: {face_label} ({drowsy_probability:.2f})"
            current_color = (0, 255, 0) if pose_label == "Optimal" else (0, 0, 255)
            
            # 기본 빈 대시보드 생성
            label_dashboard = np.zeros((config.LABEL_DASHBOARD_HEIGHT, config.DASHBOARD_WIDTH, 3), dtype=np.uint8)
            cv2.putText(label_dashboard, "Waiting for Pose Detection...", (50, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 1)
            # 메인 뷰 상단 상태 표시
            cv2.rectangle(frame, (0, 0), (120, 60), (0, 0, 0), -1)
            
            # 라벨링 전용 출력창
            # A. 실시간 포즈 라벨 (상단)
            margin_x = config.LABEL_MARGIN_X
            cv2.putText(label_dashboard, "POSTURE STATUS", (margin_x, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
            cv2.putText(label_dashboard, f"{pose_label}", (margin_x, 110), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, current_color, 3)

            # B. 얼굴 분석 라벨 (중단 - 30초 주기 업데이트)
            face_ui_color = (0, 0, 255) if face_label_idx == 1 else (0, 255, 0)
            cv2.putText(label_dashboard, "FACE STATUS", (margin_x, 180), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
            cv2.putText(label_dashboard, f"{face_label}", (margin_x, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, face_ui_color, 3)

        # 결과 창 출력
        cv2.imshow("Camera Stream", frame)
        cv2.imshow("Label Status", label_dashboard)

        # ESC 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == 27:
            break

    print("🔌 시스템을 종료합니다.")
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
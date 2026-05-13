import cv2
import numpy as np
import joblib
import os
import mediapipe as mp
from collections import deque
from modules.logger import StudyLogger  # 로그 기록 모듈 추가


# TFLite 런타임 (라즈베리 파이 최적화용)
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

# 병현님의 모듈들 임포트
from modules.camera import CameraStream
from modules.features import calculate_features  # 11개 피처 버전
from modules.visualizer import Visualizer
import modules.config as config 

class TFLiteEngine:
    """TFLite 모델 로드 및 추론 엔진"""
    def __init__(self, model_path, scaler_path, baseline_path):
        # 스케일러 로드
        self.scaler = joblib.load(scaler_path)
        
        # TFLite 인터프리터 설정
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        # 입출력 텐서 인덱스 확보
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # 초기 자세 기준값(Baseline) 로드
        self.baseline = self.load_baseline(baseline_path)

    def load_baseline(self, path):
        if os.path.exists(path):
            print(f"✅ 기준값 로드 완료: {path}")
            return joblib.load(path)
        else:
            print("⚠️ 기준값 파일이 없어 모든 피처를 0으로 초기화합니다.")
            return np.zeros(11) # 피처가 11개인 경우

    def predict(self, raw_features):
        """11개 피처를 입력받아 각 클래스별 확률 배열을 반환"""
        # 피처를 numpy 배열로 변환 (1, 11) 및 스케일링
        relative_features = np.array(raw_features) - self.baseline
        
        # 스케일링 적용
        features_np = relative_features.reshape(1, -1)
        scaled_data = self.scaler.transform(features_np).astype(np.float32)
        
        # TFLite 추론 실행
        self.interpreter.set_tensor(self.input_details[0]['index'], scaled_data)
        self.interpreter.invoke()
        
        return self.interpreter.get_tensor(self.output_details[0]['index'])[0]

def main():
    # 1. 인스턴스 초기화
    cam = CameraStream(src=0).start() 
    viz = Visualizer()
    # config.py에 정의된 경로 사용
    engine = TFLiteEngine(config.MODEL_PATH, config.SCALER_PATH, config.BASELINE_PATH)
    
    # 로그 기록을 위한 StudyLogger 인스턴스 생성
    logger = StudyLogger()

    # 2. MediaPipe 탐지기 설정
    mp_pose = mp.solutions.pose
    mp_face_mesh = mp.solutions.face_mesh
    
    pose_detector = mp_pose.Pose(
        min_detection_confidence=0.5, 
        min_tracking_confidence=0.5
    )
    face_detector = mp_face_mesh.FaceMesh(
        max_num_faces=1, 
        refine_landmarks=True
    )

    # 3. 라벨 및 색상 정의 (학습 시 인덱스 순서와 일치해야 함)
    # 0: 정자세, 1: 비대칭, 2: 거북목, 3: 턱굄 (예시 순서)
    labels = {0: "Optimal", 1: "Asymmetric", 2: "Forward Head", 3: "Chin Propping"}
    colors = {
        0: (0, 255, 0),      # Green
        1: (255, 191, 0),    # Blue-Green
        2: (0, 165, 255),    # Orange
        3: (0, 0, 255)       # Red
    }

    print("🚀 실시간 자세 분석 시스템 가동 중... (추론 모드)")

    prediction_queue = deque(maxlen=20)

    try:
        while True:
            # 프레임 읽기
            frame = cam.read()
            if frame is None:
                continue

            # 전처리 (좌우 반전 및 RGB 변환)
            frame = cv2.flip(frame, 1)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 랜드마크 추론
            results_pose = pose_detector.process(img_rgb)
            results_face = face_detector.process(img_rgb)

            # 기본 빈 대시보드 생성 (데이터 없을 때 표시용)
            dashboard = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(dashboard, "Waiting for Pose Detection...", (50, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 1)
            
            if results_pose.pose_landmarks:
                # [Visualizer] 뼈대 및 얼굴 그물망 그리기
                face_lms = results_face.multi_face_landmarks[0] if results_face.multi_face_landmarks else None
                viz.draw_landmarks(frame, results_pose.pose_landmarks, face_lms)
                
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
                    class_idx = np.argmax(probs)
                    
                    confidence = probs[class_idx]
                    status_text = labels[class_idx]
                    status_color = colors[class_idx]

                    prediction_queue.append(class_idx)
                    final_label= max(set(prediction_queue), key=prediction_queue.count)
                    final_status_text = labels[final_label]

                    # --- [로그 저장 로직 추가] ---
                    # data detail.png의 포즈 관련 데이터 매핑
                    pose_log = {
                        "posture_type": labels[final_label],
                        "forward_head_ratio": float(probs[0]), # 예시: 피처 배열의 인덱스에 맞게 수정
                        "chin_rest_score": float(probs[3]),           # 턱괸 확률을 점수로 활용
                        "asymmetry_angle": float(raw_features[1])      # 예시: 피처 배열의 인덱스에 맞게 수정
                    }
                    logger.save(pose_log)
                    # --------------------------
                    
                    
                    # [Visualizer] 모든 라벨의 확률 대시보드 생성 (별도 창)
                    dashboard = viz.draw_confidence_dashboard(probs, labels, colors)

                    # 메인 뷰 상단 상태 표시
                    cv2.rectangle(frame, (0, 0), (120, 60), (0, 0, 0), -1)
                    cv2.putText(frame, f"{final_status_text}", (10, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

            # 두 개의 결과 창 출력
            cv2.imshow('Landmark View', frame)
            cv2.imshow('Confidence Analysis', dashboard)

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
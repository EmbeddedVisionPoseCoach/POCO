import cv2
import numpy as np
import joblib
import os
import mediapipe as mp

# TFLite 런타임 (라즈베리 파이 최적화용)
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

# 병현님의 실제 파일들에서 함수와 클래스 임포트
from camera import CameraStream
from features import calculate_8_features  # 클래스가 아닌 함수 임포트
from visualizer import Visualizer
import config 

class TFLiteEngine:
    def __init__(self, model_path, scaler_path):
        self.scaler = joblib.load(scaler_path)
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
    def predict(self, raw_features):
        # 8개 피처를 numpy 배열로 변환 후 스케일링
        features_np = np.array(raw_features).reshape(1, -1)
        scaled_data = self.scaler.transform(features_np).astype(np.float32)
        
        self.interpreter.set_tensor(self.input_details[0]['index'], scaled_data)
        self.interpreter.invoke()
        return self.interpreter.get_tensor(self.output_details[0]['index'])[0]

def main():
    # 1. 파일 경로 설정
    MODEL_PATH = 'saved_model/posture_model.tflite'
    SCALER_PATH = 'saved_model/posture_scaler.pkl'
    
    # 2. 인스턴스 초기화
    # 병현님의 CameraStream 클래스 활용
    cam = CameraStream(src=0).start() 
    viz = Visualizer()
    engine = TFLiteEngine(MODEL_PATH, SCALER_PATH)

    # 3. MediaPipe Pose 초기화 (함수가 Pose 랜드마크만 쓰므로 Pose만 로드)
    mp_pose = mp.solutions.pose
    pose_detector = mp_pose.Pose(
        min_detection_confidence=0.5, 
        min_tracking_confidence=0.5
    )

    labels = {0: "Optimal", 1: "Forward Head", 2: "Asymmetric", 3: "Chin Propping"}
    colors = {0: (0, 255, 0), 1: (0, 165, 255), 2: (255, 191, 0), 3: (0, 0, 255)}

    print("🚀 실시간 자세 판별 엔진 가동 중... (종료: ESC)")

    try:
        while True:
            # 병현님의 CameraStream 스레드에서 프레임 읽기
            frame = cam.read()
            if frame is None:
                continue

            # 전처리 (좌우 반전 및 RGB 변환)
            frame = cv2.flip(frame, 1)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 랜드마크 추출
            results = pose_detector.process(img_rgb)

            # 대시보드(상태창) 배경 생성
            dashboard = np.zeros((300, 500, 3), dtype=np.uint8)
            
            if results.pose_landmarks:
                # [Visualizer] 랜드마크 시각화
                viz.draw_landmarks(frame, results.pose_landmarks)
                
                # [Features] 병현님의 함수 호출
                # 함수의 'lm = pose_landmarks[0]' 구조를 맞추기 위해 리스트로 감싸서 전달
                landmark_list = [results.pose_landmarks.landmark]
                raw_features = calculate_8_features(landmark_list)
                
                # AI 추론 (반환된 8개 피처값이 유효할 때만 실행)
                if any(raw_features):
                    probs = engine.predict(raw_features)
                    class_idx = np.argmax(probs)
                    confidence = probs[class_idx]
                    
                    status_text = labels[class_idx]
                    status_color = colors[class_idx]

                    # 대시보드 UI 업데이트
                    cv2.putText(dashboard, "AI POSTURE DASHBOARD", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                    cv2.rectangle(dashboard, (20, 60), (480, 180), status_color, -1)
                    cv2.putText(dashboard, status_text, (40, 140), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                    cv2.putText(dashboard, f"Confidence: {confidence*100:.1f}%", (20, 240), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)

                    # 메인 뷰 하단 표시
                    cv2.putText(frame, f"STATUS: {status_text}", (10, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, status_color, 2)

            # 결과 출력
            cv2.imshow('Landmark View', frame)
            cv2.imshow('Status Dashboard', dashboard)

            if cv2.waitKey(1) & 0xFF == 27: # ESC 키
                break
    finally:
        # 종료 처리
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
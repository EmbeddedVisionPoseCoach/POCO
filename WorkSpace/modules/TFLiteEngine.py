import numpy as np
import joblib
import os

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

class TFLiteEngine:
    """TFLite 모델 로드 및 추론 엔진"""
    def __init__(self, model_path, face_model_path, scaler_path, face_scaler_path, baseline_path):
        # 스케일러 로드
        self.scaler = joblib.load(scaler_path)
        self.face_scaler = joblib.load(face_scaler_path)
        
        # TFLite 인터프리터 설정
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.face_interpreter = tflite.Interpreter(model_path=face_model_path)
        self.face_interpreter.allocate_tensors()

        
        
        # 입출력 텐서 인덱스 확보
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.face_input_details = self.face_interpreter.get_input_details()
        self.face_output_details = self.face_interpreter.get_output_details()


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
    
    def predict_face(self, raw_features):
        """얼굴용 피처를 입력받아 확률을 반환"""
        features_np = np.array(raw_features).reshape(1, -1)
        scaled_data = self.face_scaler.transform(features_np).astype(np.float32)

        self.face_interpreter.set_tensor(self.face_input_details[0]['index'], scaled_data)
        self.face_interpreter.invoke()

        prediction = self.face_interpreter.get_tensor(self.face_output_details[0]['index'])[0]

        return float(prediction[0]) if len(prediction.shape) > 0 else float(prediction)
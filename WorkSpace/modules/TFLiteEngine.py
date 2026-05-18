import numpy as np
import joblib
import os
import json

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite


class TFLiteEngine:
    """TFLite 모델 로드 및 추론 엔진"""

    def __init__(
        self,
        model_path=None,
        face_model_path=None,
        scaler_path=None,
        face_scaler_path=None,
        baseline_path=None,
        face_baseline_path=None,
        threshold_path="saved_model/threshold.json"
    ):
        # =====================================================
        # 1. Pose 모델 관련 초기화
        # =====================================================
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.scaler = None
        self.baseline = None

        if model_path is not None and scaler_path is not None:
            self.scaler = joblib.load(scaler_path)

            self.interpreter = tflite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()

            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()

            self.baseline = self.load_baseline(baseline_path)
            print("✅ Pose 모델 로드 완료")
        else:
            print("⚠️ Pose 모델 비활성화 상태입니다.")

        # =====================================================
        # 2. Face 모델 관련 로드
        # =====================================================
        if face_model_path is None:
            raise ValueError("face_model_path가 필요합니다.")

        if face_scaler_path is None:
            raise ValueError("face_scaler_path가 필요합니다.")

        self.face_scaler = joblib.load(face_scaler_path)

        self.face_interpreter = tflite.Interpreter(model_path=face_model_path)
        self.face_interpreter.allocate_tensors()

        self.face_input_details = self.face_interpreter.get_input_details()
        self.face_output_details = self.face_interpreter.get_output_details()

        self.face_baseline = self.load_face_baseline(face_baseline_path)

        # threshold 로드
        self.face_threshold = self.load_threshold(threshold_path)
        print(f"✅ 얼굴 threshold 로드 완료: {self.face_threshold}")

    def load_baseline(self, path):
        if path is not None and os.path.exists(path):
            print(f"✅ 기준값 로드 완료: {path}")
            return joblib.load(path)
        else:
            print("⚠️ 기준값 파일이 없어 모든 포즈 피처를 0으로 초기화합니다.")
            return np.zeros(10, dtype=np.float32)

    def load_face_baseline(self, path):
        if path is not None and os.path.exists(path):
            print(f"✅ 얼굴 기준값 로드 완료: {path}")
            return joblib.load(path)
        else:
            print("⚠️ 얼굴 기준값 파일이 없어 모든 얼굴 피처를 0으로 초기화합니다.")
            return np.zeros(4, dtype=np.float32)

    def load_threshold(self, path):
        if path is not None and os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return float(data.get("threshold", 0.77))

        return 0.77

    def predict(self, raw_features):
        """
        Pose 모델 예측.
        현재 USE_POSE=False일 때는 호출하지 않음.
        """
        if self.interpreter is None:
            return 0, 0.0

        features = np.array(raw_features, dtype=np.float32)

        self.interpreter.set_tensor(
            self.input_details[0]["index"],
            features
        )
        self.interpreter.invoke()

        output = self.interpreter.get_tensor(
            self.output_details[0]["index"]
        )[0]

        label_idx = int(np.argmax(output))
        confidence = float(output[label_idx])

        return label_idx, confidence

    def predict_face(self, input_tensor_face):
        """
        Face GRU 모델 예측.

        input_tensor_face shape:
            [1, 60, 4]

        반환:
            face_label_idx:
                0 = Normal
                1 = Drowsy

            drowsy_probability:
                모델이 출력한 실제 졸음 확률
        """
        input_tensor_face = np.array(input_tensor_face, dtype=np.float32)

        self.face_interpreter.set_tensor(
            self.face_input_details[0]["index"],
            input_tensor_face
        )
        self.face_interpreter.invoke()

        prediction = self.face_interpreter.get_tensor(
            self.face_output_details[0]["index"]
        )

        # 모델 sigmoid 출력값
        drowsy_probability = float(np.squeeze(prediction))

        if drowsy_probability >= self.face_threshold:
            face_label_idx = 1
        else:
            face_label_idx = 0

        return face_label_idx, drowsy_probability
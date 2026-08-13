import sys
from collections import deque
from pathlib import Path

import joblib
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
sys.path.append(str(ROOT_DIR))

import modules.config as config


class FaceGruService:
    def __init__(self):
        self.model_path = self.resolve_path(config.MODEL_FACE_PATH_GRU)
        self.scaler_path = self.resolve_path(config.SCALER_FACE_PATH_GRU)
        self.baseline_path = self.resolve_path(config.FACE_BASELINE_PATH)

        self.labels = config.FACE_LABELS
        self.window = deque(maxlen=config.WINDOW_SIZE)

        self.scaler = None
        self.interpreter = None
        self.input_details = None
        self.output_details = None

        self.baseline = self.load_baseline()
        self.frame_count = 0
        self.is_running = False

    def resolve_path(self, path):
        path = Path(path)
        return path if path.is_absolute() else ROOT_DIR / path

    def load_baseline(self):
        if not self.baseline_path.exists():
            print("[FaceGRU] Baseline 없음. 0으로 초기화합니다.")
            return np.zeros(config.FACE_FEATURE_SIZE, dtype=np.float32)

        baseline = joblib.load(self.baseline_path)
        baseline = np.asarray(baseline, dtype=np.float32)

        if baseline.size != config.FACE_FEATURE_SIZE:
            print(
                f"[FaceGRU] Baseline 크기 오류 : "
                f"{baseline.size} != {config.FACE_FEATURE_SIZE}"
            )
            return np.zeros(config.FACE_FEATURE_SIZE, dtype=np.float32)

        print(f"[FaceGRU] Baseline 로드 완료 : {self.baseline_path}")
        return baseline

    def create_interpreter(self):
        try:
            import tflite_runtime.interpreter as tflite
        except ImportError:
            import tensorflow.lite as tflite

        return tflite.Interpreter(model_path=str(self.model_path))

    def load(self):
        if not self.model_path.exists():
            raise FileNotFoundError(f"Face GRU 모델 없음 : {self.model_path}")

        if not self.scaler_path.exists():
            raise FileNotFoundError(f"Face Scaler 없음 : {self.scaler_path}")

        self.scaler = joblib.load(self.scaler_path)

        self.interpreter = self.create_interpreter()
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        print(f"[FaceGRU] 모델 로드 완료 : {self.model_path}")

    def start(self):
        if self.interpreter is None:
            self.load()

        self.window.clear()
        self.frame_count = 0
        self.is_running = True

        print("[FaceGRU] 추론 시작")

    def stop(self):
        self.is_running = False
        self.window.clear()
        self.frame_count = 0

    def update(self, features):
        if not self.is_running:
            return None

        self.frame_count += 1

        safe_features = self.build_features(features)
        corrected_features = safe_features - self.baseline

        # 눈깜빡임 포함 모든 30FPS Feature를 Window에 저장
        self.window.append(corrected_features)

        if len(self.window) < config.WINDOW_SIZE:
            return None

        if self.frame_count % config.STRIDE != 0:
            return None

        fatigue_label, probability, fatigue_index = self.predict()

        return {
            "fatigue_label": fatigue_label,
            "fatigue_probability": probability,
            "fatigue_index": fatigue_index
        }

    def build_features(self, features):
        if features is None:
            return np.zeros(config.FACE_FEATURE_SIZE, dtype=np.float32)

        features = np.asarray(features, dtype=np.float32)

        if features.size != config.FACE_FEATURE_SIZE:
            return np.zeros(config.FACE_FEATURE_SIZE, dtype=np.float32)

        return features

    def predict(self):
        model_input = np.asarray(self.window, dtype=np.float32)

        model_input = model_input.reshape(1, -1)
        model_input = self.scaler.transform(model_input).astype(np.float32)

        input_tensor = model_input.reshape(
            1,
            config.WINDOW_SIZE,
            config.FACE_FEATURE_SIZE
        )

        self.interpreter.set_tensor(self.input_details[0]["index"], input_tensor)
        self.interpreter.invoke()

        output = self.interpreter.get_tensor(self.output_details[0]["index"])

        label_index, probability = self.parse_output(output)
        label = self.labels.get(label_index, f"Unknown({label_index})")

        return label, probability, label_index

    @staticmethod
    def parse_output(output):
        probs = np.squeeze(np.asarray(output))

        if probs.ndim == 0:
            probability = float(probs)
            label_index = 1 if probability >= 0.5 else 0
            confidence = probability if label_index == 1 else 1.0 - probability
            return label_index, confidence

        if probs.ndim == 1 and probs.shape[0] == 1:
            probability = float(probs[0])
            label_index = 1 if probability >= 0.5 else 0
            confidence = probability if label_index == 1 else 1.0 - probability
            return label_index, confidence

        label_index = int(np.argmax(probs))
        confidence = float(probs[label_index])

        return label_index, confidence

    def close(self):
        self.stop()

        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.scaler = None
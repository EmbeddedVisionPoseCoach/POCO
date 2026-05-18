import os
import numpy as np
import tensorflow as tf

# ============================================================
# 경로 설정
# ============================================================

KERAS_MODEL_PATH = r"C:\Users\HP\Downloads\face_gru_model_v001\face_model.keras"
TFLITE_SAVE_PATH = r"saved_model/face_model.tflite"

# ============================================================
# 1. TensorFlow 버전 확인
# ============================================================

print("=" * 60)
print("TensorFlow version:", tf.__version__)
print("=" * 60)

# ============================================================
# 2. Keras 모델 로드
# ============================================================

if not os.path.exists(KERAS_MODEL_PATH):
    raise FileNotFoundError(f"Keras 모델 파일을 찾을 수 없습니다: {KERAS_MODEL_PATH}")

print(f"✅ Keras 모델 로드 중: {KERAS_MODEL_PATH}")

model = tf.keras.models.load_model(KERAS_MODEL_PATH, compile=False)

print("✅ Keras 모델 로드 완료")
print("모델 입력 shape:", model.input_shape)
print("모델 출력 shape:", model.output_shape)

# ============================================================
# 3. TFLite 변환
# ============================================================

print("✅ TFLite 변환 시작")

converter = tf.lite.TFLiteConverter.from_keras_model(model)

# 현재 PC TensorFlow 2.15 런타임 호환을 우선으로 변환
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS
]

# float32 모델로 유지
converter.optimizations = []

tflite_model = converter.convert()

# ============================================================
# 4. 저장
# ============================================================

os.makedirs(os.path.dirname(TFLITE_SAVE_PATH), exist_ok=True)

with open(TFLITE_SAVE_PATH, "wb") as f:
    f.write(tflite_model)

print(f"✅ TFLite 저장 완료: {TFLITE_SAVE_PATH}")
print(f"파일 크기: {os.path.getsize(TFLITE_SAVE_PATH) / 1024:.2f} KB")

# ============================================================
# 5. 변환된 TFLite 모델 로드 테스트
# ============================================================

print("✅ TFLite 로드 테스트 시작")

interpreter = tf.lite.Interpreter(model_path=TFLITE_SAVE_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("TFLite input details:")
for item in input_details:
    print(item)

print("TFLite output details:")
for item in output_details:
    print(item)

# ============================================================
# 6. 더미 입력으로 추론 테스트
# ============================================================

input_shape = input_details[0]["shape"]
input_dtype = input_details[0]["dtype"]

dummy_input = np.zeros(input_shape, dtype=input_dtype)

interpreter.set_tensor(input_details[0]["index"], dummy_input)
interpreter.invoke()

output = interpreter.get_tensor(output_details[0]["index"])

print("✅ 더미 추론 성공")
print("출력 shape:", output.shape)
print("출력값:", output)

print("=" * 60)
print("변환 및 로드 테스트 완료")
print("=" * 60)
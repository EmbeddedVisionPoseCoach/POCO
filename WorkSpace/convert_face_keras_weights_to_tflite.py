import os
import zipfile
import tempfile
import numpy as np
import tensorflow as tf

# ============================================================
# 경로 설정
# ============================================================

KERAS_MODEL_PATH = r"C:\Users\HP\Downloads\face_gru_model_v001\face_model.keras"
TFLITE_SAVE_PATH = r"saved_model\face_model.tflite"

# ============================================================
# 1. TensorFlow 버전 확인
# ============================================================

print("=" * 60)
print("TensorFlow version:", tf.__version__)
print("=" * 60)

if not os.path.exists(KERAS_MODEL_PATH):
    raise FileNotFoundError(f"Keras 모델 파일을 찾을 수 없습니다: {KERAS_MODEL_PATH}")

# ============================================================
# 2. TensorFlow 2.15에서 동일한 모델 구조 직접 생성
# ============================================================

print("✅ TensorFlow 2.15용 GRU 모델 구조 생성")

inputs = tf.keras.Input(
    shape=(60, 4),
    batch_size=1,
    name="face_sequence_input"
)

x = tf.keras.layers.GRU(
    32,
    return_sequences=True,
    unroll=True,
    name="gru_1"
)(inputs)

x = tf.keras.layers.GRU(
    16,
    return_sequences=False,
    unroll=True,
    name="gru_2"
)(x)

x = tf.keras.layers.Dense(
    16,
    activation="relu",
    kernel_regularizer=tf.keras.regularizers.l2(0.0001),
    name="dense_1"
)(x)

outputs = tf.keras.layers.Dense(
    1,
    activation="sigmoid",
    name="drowsy_probability"
)(x)

model = tf.keras.Model(
    inputs=inputs,
    outputs=outputs,
    name="face_gru_model"
)

model.summary()

# ============================================================
# 3. .keras 파일에서 weights 파일 추출
# ============================================================

print("✅ .keras 파일에서 weights 추출")

temp_dir = tempfile.mkdtemp()

with zipfile.ZipFile(KERAS_MODEL_PATH, "r") as z:
    z.extractall(temp_dir)

weights_path = os.path.join(temp_dir, "model.weights.h5")

if not os.path.exists(weights_path):
    print("압축 내부 파일 목록:")
    for name in os.listdir(temp_dir):
        print(" -", name)
    raise FileNotFoundError("model.weights.h5 파일을 찾을 수 없습니다.")

print(f"✅ weights 경로: {weights_path}")

# ============================================================
# 4. weights 로드
# ============================================================

print("✅ weights 로드 시작")

model.load_weights(weights_path)

print("✅ weights 로드 완료")

# ============================================================
# 5. 더미 입력으로 Keras 모델 추론 테스트
# ============================================================

dummy_input = np.zeros((1, 60, 4), dtype=np.float32)
dummy_output = model.predict(dummy_input, verbose=0)

print("✅ Keras 더미 추론 성공")
print("출력 shape:", dummy_output.shape)
print("출력값:", dummy_output)

# ============================================================
# 6. TensorFlow 2.15 기준 TFLite 변환
# ============================================================

print("✅ TFLite 변환 시작")

converter = tf.lite.TFLiteConverter.from_keras_model(model)

converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS
]

converter.optimizations = []

tflite_model = converter.convert()

# ============================================================
# 7. TFLite 저장
# ============================================================

os.makedirs(os.path.dirname(TFLITE_SAVE_PATH), exist_ok=True)

with open(TFLITE_SAVE_PATH, "wb") as f:
    f.write(tflite_model)

print(f"✅ TFLite 저장 완료: {TFLITE_SAVE_PATH}")
print(f"파일 크기: {os.path.getsize(TFLITE_SAVE_PATH) / 1024:.2f} KB")

# ============================================================
# 8. 변환된 TFLite 로드 테스트
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

input_shape = input_details[0]["shape"]
input_dtype = input_details[0]["dtype"]

dummy_tflite_input = np.zeros(input_shape, dtype=input_dtype)

interpreter.set_tensor(input_details[0]["index"], dummy_tflite_input)
interpreter.invoke()

tflite_output = interpreter.get_tensor(output_details[0]["index"])

print("✅ TFLite 더미 추론 성공")
print("출력 shape:", tflite_output.shape)
print("출력값:", tflite_output)

print("=" * 60)
print("변환 완료")
print("=" * 60)
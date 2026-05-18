import os
import zipfile
import tempfile
import numpy as np
import tensorflow as tf
import h5py

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
    reset_after=True,
    name="gru_1"
)(inputs)

x = tf.keras.layers.GRU(
    16,
    return_sequences=False,
    unroll=True,
    reset_after=True,
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
# 3. .keras 파일에서 model.weights.h5 추출
# ============================================================

print("✅ .keras 파일에서 weights 추출")

temp_dir = tempfile.mkdtemp()

with zipfile.ZipFile(KERAS_MODEL_PATH, "r") as z:
    z.extractall(temp_dir)

weights_path = os.path.join(temp_dir, "model.weights.h5")

if not os.path.exists(weights_path):
    print("압축 내부 파일 목록:")
    for root, dirs, files in os.walk(temp_dir):
        for file in files:
            print(os.path.join(root, file))
    raise FileNotFoundError("model.weights.h5 파일을 찾을 수 없습니다.")

print(f"✅ weights 경로: {weights_path}")

# ============================================================
# 4. h5 내부에서 weight 배열 직접 찾기
# ============================================================

def collect_var_sets(h5_file):
    """
    Keras 3 .keras 내부의 model.weights.h5에서
    숫자 key를 가진 vars 그룹들을 전부 수집한다.

    예:
        layers/gru/cell/vars/0
        layers/gru/cell/vars/1
        layers/gru/cell/vars/2
    """
    var_sets = []

    def visitor(name, obj):
        if isinstance(obj, h5py.Group):
            numeric_keys = [
                key for key in obj.keys()
                if key.isdigit() and isinstance(obj[key], h5py.Dataset)
            ]

            if numeric_keys:
                numeric_keys = sorted(numeric_keys, key=lambda x: int(x))
                arrays = [np.array(obj[key]) for key in numeric_keys]
                shapes = [tuple(arr.shape) for arr in arrays]
                var_sets.append({
                    "path": name,
                    "shapes": shapes,
                    "arrays": arrays
                })

    h5_file.visititems(visitor)
    return var_sets


def find_by_shapes(var_sets, expected_shapes, layer_name):
    """
    shape 조합으로 원하는 레이어 weight를 찾는다.
    """
    for item in var_sets:
        if item["shapes"] == expected_shapes:
            print(f"✅ {layer_name} weights 찾음: {item['path']}")
            print(f"   shapes: {item['shapes']}")
            return item["arrays"]

    print("\n❌ 원하는 weight shape을 찾지 못했습니다.")
    print(f"대상 레이어: {layer_name}")
    print(f"필요한 shape: {expected_shapes}")
    print("\n현재 h5 안에서 발견된 vars 목록:")

    for item in var_sets:
        print(f" - {item['path']}: {item['shapes']}")

    raise ValueError(f"{layer_name} weights를 찾지 못했습니다.")


with h5py.File(weights_path, "r") as h5:
    var_sets = collect_var_sets(h5)

print("\n✅ 발견된 weight 그룹 목록")
for item in var_sets:
    print(f" - {item['path']}: {item['shapes']}")

# ============================================================
# 5. shape 기준으로 각 레이어 weight 매칭
# ============================================================
# GRU 32:
#   kernel:           (4, 96)
#   recurrent_kernel: (32, 96)
#   bias:             (2, 96)
#
# GRU 16:
#   kernel:           (32, 48)
#   recurrent_kernel: (16, 48)
#   bias:             (2, 48)
#
# Dense 16:
#   kernel:           (16, 16)
#   bias:             (16,)
#
# Output Dense:
#   kernel:           (16, 1)
#   bias:             (1,)
# ============================================================

with h5py.File(weights_path, "r") as h5:
    var_sets = collect_var_sets(h5)

    gru_1_weights = find_by_shapes(
        var_sets,
        [(4, 96), (32, 96), (2, 96)],
        "gru_1"
    )

    gru_2_weights = find_by_shapes(
        var_sets,
        [(32, 48), (16, 48), (2, 48)],
        "gru_2"
    )

    dense_1_weights = find_by_shapes(
        var_sets,
        [(16, 16), (16,)],
        "dense_1"
    )

    output_weights = find_by_shapes(
        var_sets,
        [(16, 1), (1,)],
        "drowsy_probability"
    )

# ============================================================
# 6. 모델 레이어에 weights 직접 주입
# ============================================================

print("\n✅ weights 직접 주입 시작")

model.get_layer("gru_1").set_weights(gru_1_weights)
model.get_layer("gru_2").set_weights(gru_2_weights)
model.get_layer("dense_1").set_weights(dense_1_weights)
model.get_layer("drowsy_probability").set_weights(output_weights)

print("✅ weights 직접 주입 완료")

# ============================================================
# 7. Keras 모델 더미 추론 테스트
# ============================================================

dummy_input = np.zeros((1, 60, 4), dtype=np.float32)
dummy_output = model.predict(dummy_input, verbose=0)

print("\n✅ Keras 더미 추론 성공")
print("출력 shape:", dummy_output.shape)
print("출력값:", dummy_output)

# ============================================================
# 8. TFLite 변환
# ============================================================

print("\n✅ TFLite 변환 시작")

converter = tf.lite.TFLiteConverter.from_keras_model(model)

converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS
]

converter.optimizations = []

tflite_model = converter.convert()

# ============================================================
# 9. TFLite 저장
# ============================================================

os.makedirs(os.path.dirname(TFLITE_SAVE_PATH), exist_ok=True)

with open(TFLITE_SAVE_PATH, "wb") as f:
    f.write(tflite_model)

print(f"\n✅ TFLite 저장 완료: {TFLITE_SAVE_PATH}")
print(f"파일 크기: {os.path.getsize(TFLITE_SAVE_PATH) / 1024:.2f} KB")

# ============================================================
# 10. 변환된 TFLite 로드 테스트
# ============================================================

print("\n✅ TFLite 로드 테스트 시작")

interpreter = tf.lite.Interpreter(model_path=TFLITE_SAVE_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("\nTFLite input details:")
for item in input_details:
    print(item)

print("\nTFLite output details:")
for item in output_details:
    print(item)

input_shape = input_details[0]["shape"]
input_dtype = input_details[0]["dtype"]

dummy_tflite_input = np.zeros(input_shape, dtype=input_dtype)

interpreter.set_tensor(input_details[0]["index"], dummy_tflite_input)
interpreter.invoke()

tflite_output = interpreter.get_tensor(output_details[0]["index"])

print("\n✅ TFLite 더미 추론 성공")
print("출력 shape:", tflite_output.shape)
print("출력값:", tflite_output)

print("=" * 60)
print("변환 완료")
print("=" * 60)
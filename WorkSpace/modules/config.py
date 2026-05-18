import mediapipe as mp

# 해상도 고정
FRAME_WIDTH = 320
FRAME_HEIGHT = 240


WINDOW_SEC = 30.0 # 30초 동안의 데이터 수집

# 모델 경로
FACE_MODEL_PATH = "tasks/face_landmarker.task"
POSE_MODEL_PATH = "tasks/pose_landmarker_heavy.task"

# 피쳐 개수
FACE_FEATURE_SIZE = 4
POSE_FEATURE_SIZE = 10



# 학습시킨 모델 경로
MODEL_PATH = 'saved_model/posture_model.tflite'
SCALER_PATH = 'saved_model/posture_scaler.pkl' # StandardScaler 객체 저장 경로

MODEL_FACE_PATH = 'saved_model/face_model.tflite'
SCALER_FACE_PATH = 'saved_model/face_scaler.pkl'

MODEL_PATH_GRU = 'saved_model/posture_model_GRU.tflite'
SCALER_PATH_GRU = 'saved_model/posture_scaler_GRU.pkl' # StandardScaler 객체 저장 경로

MODEL_FACE_PATH_GRU = 'saved_model/face_model_GRU.tflite'
SCALER_FACE_PATH_GRU = 'saved_model/face_scaler_GRU.pkl'



BASELINE_PATH = 'saved_model/baseline.pkl'

# 라벨링 및 색상 정보
POSTURE_LABELS = {0: "Optimal", 1: "Asymmetric", 2: "Forward Head", 3: "Chin Propping"}
POSTURE_COLORS = {
    0: (0, 255, 0),      # Green
    1: (255, 191, 0),    # Blue-Green
    2: (0, 165, 255),    # Orange
    3: (0, 0, 255)       # Red
}
FACE_LABELS = {0 : "Normal", 1 : "Drowsy"}

# 라벨링 측정 프레임 수
LABEL_FRAME = 20

# 탐지 옵션
MIN_CONFIDENCE = 0.5

POSE_CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16), # 팔
    (11, 23), (12, 24), (23, 24),                    # 몸통
    (23, 25), (25, 27), (24, 26), (26, 28),          # 다리
    # 손 부분 상세 연결
    (15, 17), (17, 19), (19, 15), (15, 21),          # 왼쪽 손
    (16, 18), (18, 20), (20, 16), (16, 22)           # 오른쪽 손
]

# 초기값 설정
CALIBRATION_TIME = 5 # 초 단위


# UI 설정
LABEL_DASHBOARD_WIDTH = 500
LABEL_DASHBOARD_HEIGHT = 350
LABEL_MARGIN_X = 40

DASHBOARD_WIDTH = 600
DASHBOARD_HEIGHT = 400


# GRU 용 상수
WINDOW_SIZE = 60       # 모델이 요구하는 시퀀스 길이
STRIDE = 5             # 추론 주기 (프레임 단위)

# DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

MODEL_VERSION_MLP = "mlp"
MODEL_VERSION_GRU = "gru"

MODEL_VERSION = MODEL_VERSION_MLP
# MODEL_VERSION = MODEL_VERSION_GRU



# ------------------------------------------------------------
# Hardware 설정
# ------------------------------------------------------------

# Windows 개발 환경에서는 False 추천
HARDWARE_ENABLED = False

# 아두이노가 연결된 포트 이름
# 라즈베리파이에서는 보통 Arduino Uno가 /dev/ttyACM0 또는 /dev/ttyUSB0로 잡힘
HARDWARE_SERIAL_PORT = "/dev/ttyACM0"

# 만약 위 포트로 연결이 안 되면 아래처럼 바꿔서 테스트할 수 있음
# HARDWARE_SERIAL_PORT = "/dev/ttyUSB0"

# 통신 속도
# 아두이노 코드의 Serial.begin(115200); 과 반드시 같아야 함
HARDWARE_BAUD_RATE = 115200

# 시리얼 데이터를 읽을 때 최대 몇 초까지 기다릴지 설정
# 1초 동안 데이터가 없으면 읽기를 포기하고 다음 코드로 넘어감
HARDWARE_TIMEOUT = 1
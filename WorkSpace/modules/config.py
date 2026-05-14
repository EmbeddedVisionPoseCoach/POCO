import mediapipe as mp

# 해상도 고정
FRAME_WIDTH = 320
FRAME_HEIGHT = 240


WINDOW_SEC = 30.0 # 30초 동안의 데이터 수집

# 모델 경로
FACE_MODEL_PATH = "tasks/face_landmarker.task"
POSE_MODEL_PATH = "tasks/pose_landmarker_heavy.task"

# 학습시킨 모델 경로
MODEL_PATH = 'saved_model/posture_model.tflite'
SCALER_PATH = 'saved_model/posture_scaler.pkl' # StandardScaler 객체 저장 경로

MODEL_FACE_PATH = 'saved_model/face_model.tflite'
SCALER_FACE_PATH = 'saved_model/face_scaler.pkl'

BASELINE_PATH = 'saved_model/baseline.pkl'

# 라벨링 및 색상 정보
POSTURE_LABELS = {0: "Optimal", 1: "Asymmetric", 2: "Forward Head", 3: "Chin Propping"}
POSTURE_COLORS = {
    0: (0, 255, 0),      # Green
    1: (255, 191, 0),    # Blue-Green
    2: (0, 165, 255),    # Orange
    3: (0, 0, 255)       # Red
}

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
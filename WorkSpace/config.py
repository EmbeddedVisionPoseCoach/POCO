import mediapipe as mp

# 해상도 고정
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# 모델 경로
FACE_MODEL_PATH = "face_landmarker.task"
POSE_MODEL_PATH = "pose_landmarker_heavy.task"

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
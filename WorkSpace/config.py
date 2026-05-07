import mediapipe as mp

# 해상도 고정
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# 모델 경로
FACE_MODEL_PATH = "face_landmarker.task"
POSE_MODEL_PATH = "pose_landmarker.task"

# 탐지 옵션
MIN_CONFIDENCE = 0.5
POSE_CONNECTIONS = mp.solutions.pose.POSE_CONNECTIONS
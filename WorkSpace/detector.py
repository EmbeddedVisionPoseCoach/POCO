from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import config

class LandmarkerDetector:
    def __init__(self):
        # Face Detector
        f_base = python.BaseOptions(model_asset_path=config.FACE_MODEL_PATH)
        f_opt = vision.FaceLandmarkerOptions(
            base_options=f_base, running_mode=vision.RunningMode.IMAGE,
            output_face_blendshapes=True, min_face_detection_confidence=config.MIN_CONFIDENCE
        )
        self.face_detector = vision.FaceLandmarker.create_from_options(f_opt)

        # Pose Detector
        p_base = python.BaseOptions(model_asset_path=config.POSE_MODEL_PATH)
        p_opt = vision.PoseLandmarkerOptions(
            base_options=p_base, running_mode=vision.RunningMode.IMAGE,
            min_pose_detection_confidence=config.MIN_CONFIDENCE
        )
        self.pose_detector = vision.PoseLandmarker.create_from_options(p_opt)

    def detect(self, mp_image):
        return self.face_detector.detect(mp_image), self.pose_detector.detect(mp_image)

    def close(self):
        self.face_detector.close()
        self.pose_detector.close()
import sys
import time
import traceback
from enum import Enum, auto
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage

from mediapipe.tasks import python
from mediapipe.tasks.python import vision


ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR))


import modules.config as config
from modules.features import calculate_features
from modules.visualizer import Visualizer

# services 폴더는 pyQt/services 안에 있으므로 이렇게 import
from services.calibration_service import CalibrationService
from services.mlp_inference_service import FrameInferenceService
from services.gru_inference_service import GruInferenceService

class RunMode(Enum):
    """
    CameraWorker의 현재 동작 상태.

    PREVIEW:
        카메라와 랜드마크만 보여주는 상태.

    CALIBRATING:
        baseline.pkl 생성을 위해 feature를 수집하는 상태.

    MEASURING:
        실시간 자세 / 피로도 추론 중인 상태.
    """

    PREVIEW = auto()
    CALIBRATING = auto()
    MEASURING = auto()


class CameraWorker(QThread):
    """
    PyQt 카메라 처리 전용 Worker.

    main.py의 핵심 흐름을 PyQt 구조로 가져온 버전이다.

    담당:
    1. cv2.VideoCapture로 카메라 직접 실행
    2. MediaPipe Pose / FaceLandmarker 실행
    3. 프레임에 랜드마크 그리기
    4. calculate_features()로 pose feature 추출
    5. 현재 mode에 따라 CalibrationService 또는 InferenceService 호출
    6. QImage와 결과 dict를 PyQt UI로 emit

    주의:
    - PyQt에서는 modules.camera.CameraStream을 사용하지 않는다.
    - CameraStream은 main.py 테스트용으로만 유지한다.
    """

    frame_changed = pyqtSignal(QImage)
    status_changed = pyqtSignal(str)
    calibration_finished = pyqtSignal(bool, str)
    measurement_started = pyqtSignal(bool, str)
    result_changed = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.running = False
        self.mode = RunMode.PREVIEW

        # Camera / Vision
        self.cap = None
        self.viz = None
        self.pose_detector = None
        self.face_detector = None

        # Service
        self.calibration_service = CalibrationService(
            baseline_path=self.resolve_workspace_path(config.BASELINE_PATH),
            duration=config.CALIBRATION_TIME,
        )

        self.inference_service = None

        # 버튼을 누른 시점에 worker가 아직 시작 전일 수 있으므로 pending 처리
        self.pending_preview_start = False
        self.pending_calibration_start = False
        self.pending_measurement_start = False

        # 상태 메시지 너무 자주 emit하지 않기 위한 변수
        self.last_status_emit_time = 0.0
        self.status_emit_interval = 0.4

    # ---------------------------------------------------------
    # Main Loop
    # ---------------------------------------------------------
    def run(self):
        """
        QThread 시작 시 실행되는 메인 루프.

        카메라를 계속 읽고,
        현재 mode에 따라 캘리브레이션 또는 추론을 수행한다.
        """

        self.running = True
        error_message = None

        try:
            self.initialize_camera_system()
            self.status_changed.emit("카메라 프리뷰 준비 완료")

            # worker 시작 전에 버튼이 눌렸던 경우 처리
            self.apply_pending_command()

            while self.running:
                ret, frame = self.cap.read()

                if not ret or frame is None:
                    self.emit_status_interval("카메라 프레임을 읽지 못했습니다.")
                    continue

                # main.py에서 가져온 기본 전처리
                frame = cv2.flip(frame, 1)
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # MediaPipe Pose
                results_pose = self.pose_detector.process(img_rgb)

                # MediaPipe FaceLandmarker
                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=img_rgb
                )

                results_face = self.face_detector.detect(mp_image)

                raw_features = None

                if results_pose.pose_landmarks:
                    face_landmarks = self.extract_face_landmarks(results_face)

                    # 화면 표시용 랜드마크 그리기
                    self.viz.draw_landmarks(
                        frame,
                        results_pose.pose_landmarks,
                        face_landmarks
                    )

                    landmark_list = [results_pose.pose_landmarks.landmark]
                    raw_features = calculate_features(landmark_list)

                    if self.is_valid_feature(raw_features):
                        self.process_by_mode(raw_features, results_face)
                    else:
                        if self.mode == RunMode.CALIBRATING:
                            self.emit_status_interval("feature 추출이 아직 불안정합니다.")

                else:
                    if self.mode == RunMode.CALIBRATING:
                        self.emit_status_interval("자세가 인식되지 않습니다. 카메라 정면에 앉아주세요.")

                # PyQt QLabel 표시용으로 프레임 전달
                q_image = self.convert_frame_to_qimage(frame)
                self.frame_changed.emit(q_image)

        except Exception as e:
            error_message = traceback.format_exc()
            print(error_message)
            self.status_changed.emit(f"카메라 오류 발생:\n{e}")

        finally:
            self.release_resources(show_message=(error_message is None))

    # ---------------------------------------------------------
    # Initialize
    # ---------------------------------------------------------
    def initialize_camera_system(self):
        """
        카메라와 MediaPipe 시스템을 초기화한다.

        PyQt용 CameraWorker에서는 CameraStream을 쓰지 않고
        cv2.VideoCapture를 직접 사용한다.
        """

        face_task_path = self.resolve_workspace_path(config.FACE_MODEL_PATH)

        if not face_task_path.exists():
            raise FileNotFoundError(
                f"FaceLandmarker 모델 파일을 찾을 수 없습니다:\n{face_task_path}"
            )

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            raise RuntimeError("카메라를 열 수 없습니다.")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

        self.viz = Visualizer()

        mp_pose = mp.solutions.pose

        self.pose_detector = mp_pose.Pose(
            min_detection_confidence=config.MIN_CONFIDENCE,
            min_tracking_confidence=config.MIN_CONFIDENCE,
        )

        with open(face_task_path, "rb") as f:
            face_model_buffer = f.read()

        base_options = python.BaseOptions(
            model_asset_buffer=face_model_buffer
        )

        options = vision.FaceLandmarkerOptions(
            base_options=base_options,
            output_face_blendshapes=True,
            num_faces=1,
        )

        self.face_detector = vision.FaceLandmarker.create_from_options(options)

    # ---------------------------------------------------------
    # Pending Command
    # ---------------------------------------------------------
    def apply_pending_command(self):
        """
        Worker가 완전히 시작되기 전에 버튼 명령이 들어온 경우 처리한다.
        """

        if self.pending_calibration_start:
            self.pending_calibration_start = False
            self.start_calibration()
            return

        if self.pending_measurement_start:
            self.pending_measurement_start = False
            self.start_measurement()
            return

        if self.pending_preview_start:
            self.pending_preview_start = False
            self.start_preview()
            return

    # ---------------------------------------------------------
    # Button Control
    # ---------------------------------------------------------
    def start_preview(self):
        """
        Calibration 버튼을 눌렀을 때 호출.

        카메라는 계속 켜두고,
        프리뷰와 랜드마크만 보여주는 상태로 둔다.
        """

        if not self.running:
            self.pending_preview_start = True
            self.status_changed.emit("카메라 준비 중입니다. 준비되면 프리뷰를 시작합니다.")
            return

        self.mode = RunMode.PREVIEW
        self.status_changed.emit("프리뷰 모드입니다. 바른 자세를 준비해주세요.")

    def start_calibration(self):
        """
        Calibration Start 버튼을 눌렀을 때 호출.

        실제 baseline feature 수집을 시작한다.
        """

        if not self.running:
            self.pending_calibration_start = True
            self.status_changed.emit("카메라 준비 중입니다. 준비되면 초기 측정을 시작합니다.")
            return

        self.mode = RunMode.CALIBRATING

        result = self.calibration_service.start()
        self.status_changed.emit(result.message)

    def start_measurement(self):
        """
        추론 시작 버튼을 눌렀을 때 호출.

        config.MODEL_VERSION을 보고 사용할 추론 서비스를 선택한다.
        현재는 mlp 모델을 우선 연결한다.
        """

        if not self.running:
            self.pending_measurement_start = True
            self.status_changed.emit("카메라 준비 중입니다. 준비되면 추론을 시작합니다.")
            return

        try:
            # 기존 추론 서비스가 있으면 먼저 정리
            if self.inference_service is not None:
                self.inference_service.stop()
                self.inference_service = None

            model_version = getattr(config, "MODEL_VERSION", "mlp")

            if model_version == "mlp":
                self.inference_service = FrameInferenceService(
                    model_path=self.resolve_workspace_path(config.MODEL_PATH),
                    face_model_path=self.resolve_workspace_path(config.MODEL_FACE_PATH),
                    scaler_path=self.resolve_workspace_path(config.SCALER_PATH),
                    face_scaler_path=self.resolve_workspace_path(config.SCALER_FACE_PATH),
                    baseline_path=self.resolve_workspace_path(config.BASELINE_PATH),
                    labels=config.POSTURE_LABELS,
                    smoothing_frame=config.LABEL_FRAME,
                    ui_emit_interval=0.5,
                    fatigue_threshold=0.5,
                )
            elif model_version == "gru":
                self.inference_service = GruInferenceService(
                    model_path=self.resolve_workspace_path(config.MODEL_PATH),
                    face_model_path=self.resolve_workspace_path(config.MODEL_FACE_PATH),
                    scaler_path=self.resolve_workspace_path(config.SCALER_PATH),
                    face_scaler_path=self.resolve_workspace_path(config.SCALER_FACE_PATH),
                    labels=config.POSTURE_LABELS,
                    ui_emit_interval=0.5,
                )
            else:
                self.measurement_started.emit(
                    False,
                    f"지원하지 않는 MODEL_VERSION입니다: {model_version}"
                )
                return

            start_result = self.inference_service.start()

            if not start_result.success:
                self.mode = RunMode.PREVIEW
                self.measurement_started.emit(False, start_result.message)
                self.status_changed.emit(start_result.message)
                return

            self.mode = RunMode.MEASURING

            self.measurement_started.emit(True, start_result.message)
            self.status_changed.emit(start_result.message)

        except Exception as e:
            msg = f"추론 시작 실패:\n{e}"
            self.mode = RunMode.PREVIEW
            self.measurement_started.emit(False, msg)
            self.status_changed.emit(msg)

    def stop_measurement(self):
        """
        추론만 종료하고 카메라는 유지하고 싶을 때 사용한다.

        추론 종료 버튼을 따로 만들면 이 함수를 연결하면 된다.
        """

        if self.inference_service is not None:
            self.inference_service.stop()
            self.inference_service = None

        self.mode = RunMode.PREVIEW
        self.status_changed.emit("추론을 종료하고 프리뷰 모드로 돌아갑니다.")

    # ---------------------------------------------------------
    # Mode Processing
    # ---------------------------------------------------------
    def process_by_mode(self, raw_features, results_face):
        """
        현재 mode에 따라 feature를 처리한다.
        """

        if self.mode == RunMode.CALIBRATING:
            self.process_calibration(raw_features)

        elif self.mode == RunMode.MEASURING:
            self.process_measurement(raw_features, results_face)

    def process_calibration(self, raw_features):
        """
        CalibrationService에 feature를 넘겨 baseline을 수집한다.
        """

        result = self.calibration_service.update(raw_features)

        self.emit_status_interval(result.message)

        if not result.is_finished:
            return

        # 캘리브레이션 완료 후 자동으로 프리뷰 상태로 복귀
        self.mode = RunMode.PREVIEW

        final_message = f"{result.message}\n저장 경로: {result.baseline_path}"

        self.status_changed.emit(final_message)
        self.calibration_finished.emit(result.success, final_message)

    def process_measurement(self, raw_features, results_face):
        """
        추론 서비스에 feature와 face 결과를 넘기고 UI 결과를 emit한다.
        """

        if self.inference_service is None:
            return

        result = self.inference_service.update(raw_features, results_face)

        if not result.success:
            self.emit_status_interval(result.message)
            return

        if not result.should_emit_ui:
            return

        self.result_changed.emit({
            "posture_type": result.posture_type,
            "confidence": result.confidence,
            "fatigue_label": result.fatigue_label,
            "fatigue_probability": result.fatigue_probability,
            "elapsed_sec": result.elapsed_sec,
            "rank_text": result.rank_text,
        })

        self.status_changed.emit(result.message)

    # ---------------------------------------------------------
    # Helper
    # ---------------------------------------------------------
    def is_valid_feature(self, raw_features):
        """
        pose feature가 추론/캘리브레이션에 사용할 수 있는 상태인지 검사한다.
        """

        if raw_features is None:
            return False

        feature_array = np.asarray(raw_features, dtype=np.float32)

        if feature_array.size != config.POSE_FEATURE_SIZE:
            self.emit_status_interval(
                f"pose feature 개수 불일치: 현재={feature_array.size}, 필요={config.POSE_FEATURE_SIZE}"
            )
            return False

        if not np.any(feature_array):
            return False

        return True

    def extract_face_landmarks(self, results_face):
        """
        Visualizer에 넘길 face landmark를 안전하게 꺼낸다.
        """

        if results_face is None:
            return None

        if not hasattr(results_face, "face_landmarks"):
            return None

        if not results_face.face_landmarks:
            return None

        if len(results_face.face_landmarks) <= 0:
            return None

        return results_face.face_landmarks[0]

    def resolve_workspace_path(self, path_text):
        """
        config 경로를 프로젝트 ROOT 기준 절대 경로로 변환한다.
        """

        path = Path(path_text)

        if path.is_absolute():
            return path

        return ROOT_DIR / path

    def emit_status_interval(self, message):
        """
        상태 메시지가 너무 자주 바뀌면 UI가 지저분해지므로
        일정 간격마다만 emit한다.
        """

        now = time.time()

        if now - self.last_status_emit_time < self.status_emit_interval:
            return

        self.last_status_emit_time = now
        self.status_changed.emit(message)

    def convert_frame_to_qimage(self, frame):
        """
        OpenCV BGR frame을 PyQt QLabel에 표시 가능한 QImage로 변환한다.
        """

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w

        return QImage(
            rgb_frame.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()

    # ---------------------------------------------------------
    # Stop / Release
    # ---------------------------------------------------------
    def stop(self):
        """
        Camera Off 또는 창 종료 시 호출한다.

        카메라 루프 종료, 서비스 정리, 리소스 해제를 수행한다.
        """

        self.running = False

        self.pending_preview_start = False
        self.pending_calibration_start = False
        self.pending_measurement_start = False

        self.calibration_service.cancel()

        if self.inference_service is not None:
            self.inference_service.stop()
            self.inference_service = None

        self.wait()

    def release_resources(self, show_message=True):
        """
        카메라와 MediaPipe 리소스를 정리한다.
        """

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if self.pose_detector is not None:
            self.pose_detector.close()
            self.pose_detector = None

        if self.face_detector is not None:
            self.face_detector.close()
            self.face_detector = None

        if show_message:
            self.status_changed.emit("카메라가 종료되었습니다.")
import sys
import time
from dataclasses import dataclass
from collections import deque, Counter
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(ROOT_DIR))

import modules.config as config
from modules.TFLiteEngine import TFLiteEngine
from modules.features import calculate_face_feature
from modules.logger import StudyLogger


@dataclass
class InferenceStartResult:
    """
    추론 시작 결과.

    CameraWorker가 start_measurement()를 호출했을 때
    성공/실패와 메시지를 UI에 넘기기 위한 용도.
    """
    success: bool
    message: str


@dataclass
class InferenceResult:
    """
    PyQt UI에 넘길 실시간 추론 결과.
    """
    success: bool
    message: str
    should_emit_ui: bool = False

    posture_type: str = "-"
    confidence: float = 0.0

    fatigue_label: str = "Normal"
    fatigue_probability: float = 0.0

    elapsed_sec: int = 0
    rank_text: str = ""


class FrameInferenceService:
    """
    이전 모델 V1 전용 추론 서비스.

    main.py에 있던 기능 중 추론 관련 로직만 가져온 클래스.

    담당:
    1. TFLiteEngine 로드
    2. 자세 모델 추론
    3. 피로도 blendshape 누적
    4. WINDOW_SEC마다 피로도 모델 추론
    5. 라벨 안정화
    6. 불안정 자세 TOP3 계산
    7. 로그 저장
    8. PyQt UI에 넘길 결과 생성

    이 클래스는 카메라, PyQt, QImage를 모른다.
    오직 pose feature와 FaceLandmarker 결과만 받는다.
    """

    def __init__(
        self,
        model_path,
        face_model_path,
        scaler_path,
        face_scaler_path,
        baseline_path,
        labels=None,
        smoothing_frame=None,
        ui_emit_interval=0.5,
        fatigue_threshold=0.5,
        log_dir="../data/session_log",
    ):
        self.model_path = str(model_path)
        self.face_model_path = str(face_model_path)
        self.scaler_path = str(scaler_path)
        self.face_scaler_path = str(face_scaler_path)
        self.baseline_path = str(baseline_path)

        self.labels = labels if labels is not None else config.POSTURE_LABELS
        self.face_labels = config.FACE_LABELS

        self.smoothing_frame = (
            smoothing_frame if smoothing_frame is not None else config.LABEL_FRAME
        )

        self.ui_emit_interval = ui_emit_interval
        self.fatigue_threshold = fatigue_threshold
        self.log_dir = log_dir

        self.engine = None

        # 자세 라벨 안정화용 큐
        self.prediction_queue = deque(maxlen=self.smoothing_frame)

        # 불안정 자세 TOP3 계산용 카운터
        self.posture_counter = Counter()

        # 피로도 계산용 blendshape window
        self.blendshape_window = []
        self.face_window_start_time = None

        # 최근 피로도 결과
        # 피로도는 WINDOW_SEC마다 갱신되므로 최신 값을 계속 유지한다.
        self.latest_fatigue_label = "Normal"
        self.latest_fatigue_probability = 0.0

        # 시간 / UI emit 제어
        self.session_start_time = None
        self.last_ui_emit_time = 0.0
        self.is_running = False

        # 로그
        self.logger = None
        self.enable_logging = True

    # ---------------------------------------------------------
    # Life Cycle
    # ---------------------------------------------------------
    def start(self):
        """
        추론 시작 버튼을 눌렀을 때 호출한다.
        """

        try:
            self.engine = TFLiteEngine(
                self.model_path,
                self.face_model_path,
                self.scaler_path,
                self.face_scaler_path,
                self.baseline_path,
            )

        except Exception as e:
            return InferenceStartResult(
                success=False,
                message=f"모델 로드 실패:\n{e}"
            )

        self.prediction_queue.clear()
        self.posture_counter.clear()
        self.blendshape_window.clear()

        self.face_window_start_time = time.time()
        self.session_start_time = time.time()
        self.last_ui_emit_time = 0.0

        self.latest_fatigue_label = "Normal"
        self.latest_fatigue_probability = 0.0

        self.is_running = True

        if self.enable_logging:
            self.logger = StudyLogger(base_dir=self.log_dir)

        return InferenceStartResult(
            success=True,
            message="MLP 모델 방식으로 실시간 추론을 시작했습니다."
        )

    def stop(self):
        """
        추론 종료 또는 Camera Off 시 호출한다.
        """

        self.is_running = False

        self.prediction_queue.clear()
        self.posture_counter.clear()
        self.blendshape_window.clear()

        self.face_window_start_time = None
        self.session_start_time = None
        self.last_ui_emit_time = 0.0

        self.engine = None
        self.logger = None

    # ---------------------------------------------------------
    # Update
    # ---------------------------------------------------------
    def update(self, pose_features, results_face=None):
        """
        카메라 프레임마다 호출한다.

        Parameters
        ----------
        pose_features:
            calculate_features()에서 반환한 자세 feature.

        results_face:
            FaceLandmarker.detect() 결과.
        """

        if not self.is_running:
            return InferenceResult(
                success=False,
                message="추론이 실행 중이 아닙니다."
            )

        if self.engine is None:
            return InferenceResult(
                success=False,
                message="모델이 로드되지 않았습니다."
            )

        if pose_features is None:
            return InferenceResult(
                success=False,
                message="pose feature가 없습니다."
            )

        pose_features = np.asarray(pose_features, dtype=np.float32)

        if pose_features.size != config.POSE_FEATURE_SIZE:
            return InferenceResult(
                success=False,
                message=(
                    "pose feature 개수가 맞지 않습니다. "
                    f"현재={pose_features.size}, 필요={config.POSE_FEATURE_SIZE}"
                )
            )

        if not np.any(pose_features):
            return InferenceResult(
                success=False,
                message="유효한 pose feature가 아닙니다."
            )

        # 1. 피로도 업데이트
        self.update_fatigue(results_face)

        # 2. 자세 모델 추론
        try:
            probs = self.engine.predict(pose_features)

        except Exception as e:
            return InferenceResult(
                success=False,
                message=f"자세 추론 오류: {e}"
            )

        # 3. 손 미검출 시 Chin Propping 확률 제거
        # 현재 feature 마지막 값은 Hand Visible Flag로 사용 중
        hand_visible = pose_features[-1]

        if len(probs) > 3 and hand_visible == 0:
            probs[3] = 0

        current_label = int(np.argmax(probs))
        self.prediction_queue.append(current_label)

        # 4. 라벨 안정화
        final_label = max(
            set(self.prediction_queue),
            key=self.prediction_queue.count
        )

        posture_type = self.labels.get(final_label, f"Unknown({final_label})")
        confidence = float(probs[final_label])

        # 5. 불안정 자세 TOP3 누적
        normal_label = self.labels.get(0, "Optimal")

        if posture_type != normal_label:
            self.posture_counter[posture_type] += 1

        elapsed_sec = self.get_elapsed_sec()
        should_emit_ui = self.should_emit_ui()

        if should_emit_ui:
            self.save_log(
                posture_type=posture_type,
                fatigue_label=self.latest_fatigue_label,
                fatigue_probability=self.latest_fatigue_probability,
            )

        return InferenceResult(
            success=True,
            message=f"측정 중",
            should_emit_ui=should_emit_ui,
            posture_type=posture_type,
            confidence=confidence,
            fatigue_label=self.latest_fatigue_label,
            fatigue_probability=self.latest_fatigue_probability,
            elapsed_sec=elapsed_sec,
            rank_text=self.build_rank_text(),
        )

    # ---------------------------------------------------------
    # Fatigue
    # ---------------------------------------------------------
    def update_fatigue(self, results_face):
        """
        V1 피로도 방식.

        FaceLandmarker 결과에서 face_blendshapes[0]를 WINDOW_SEC 동안 모은 뒤,
        calculate_face_feature()로 16개 통계 feature를 만들고 predict_face()를 호출한다.
        """

        self.collect_blendshape(results_face)

        if self.face_window_start_time is None:
            self.face_window_start_time = time.time()

        elapsed = time.time() - self.face_window_start_time

        if elapsed < config.WINDOW_SEC:
            return

        face_features = calculate_face_feature(self.blendshape_window)

        # 다음 window를 위해 초기화
        self.blendshape_window.clear()
        self.face_window_start_time = time.time()

        if face_features is None:
            return

        try:
            probability = float(self.engine.predict_face(face_features))

        except Exception:
            return

        label_index = 1 if probability >= self.fatigue_threshold else 0

        self.latest_fatigue_label = self.face_labels.get(
            label_index,
            "Drowsy" if label_index == 1 else "Normal"
        )

        self.latest_fatigue_probability = probability

    def collect_blendshape(self, results_face):
        """
        FaceLandmarker 결과에서 face_blendshapes[0]만 수집한다.
        """

        if results_face is None:
            return

        if not hasattr(results_face, "face_blendshapes"):
            return

        if not results_face.face_blendshapes:
            return

        if len(results_face.face_blendshapes) <= 0:
            return

        self.blendshape_window.append(results_face.face_blendshapes[0])

    # ---------------------------------------------------------
    # Helper
    # ---------------------------------------------------------
    def get_elapsed_sec(self):
        if self.session_start_time is None:
            return 0

        return int(time.time() - self.session_start_time)

    def should_emit_ui(self):
        now = time.time()

        if now - self.last_ui_emit_time < self.ui_emit_interval:
            return False

        self.last_ui_emit_time = now
        return True

    def build_rank_text(self):
        total_count = sum(self.posture_counter.values())

        if total_count <= 0:
            return (
                "불안정 자세 TOP 3\n\n"
                "1위  -\n"
                "2위  -\n"
                "3위  -"
            )

        top_3 = self.posture_counter.most_common(3)

        lines = ["불안정 자세 TOP 3", ""]

        for index in range(3):
            if index < len(top_3):
                posture_type, count = top_3[index]
                ratio = count / total_count * 100
                lines.append(
                    f"{index + 1}위  {posture_type}  {count}회  {ratio:.1f}%"
                )
            else:
                lines.append(f"{index + 1}위  -")

        return "\n".join(lines)

    def save_log(self, posture_type, fatigue_label, fatigue_probability):
        if not self.enable_logging:
            return

        if self.logger is None:
            return

        self.logger.save({
            "posture_type": posture_type,
            "fatigue_label": fatigue_label,
            "fatigue_probability": float(fatigue_probability),
        })
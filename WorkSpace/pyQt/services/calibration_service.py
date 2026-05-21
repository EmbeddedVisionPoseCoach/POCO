import time
from dataclasses import dataclass
from pathlib import Path

import joblib
import numpy as np


@dataclass
class CalibrationResult:
    is_finished: bool
    success: bool
    message: str
    remain_time: float = 0.0
    sample_count: int = 0
    baseline_path: str = ""


class CalibrationService:
    """
    Calibration 전용 서비스.

    역할:
    1. 캘리브레이션 시작 시간 기록
    2. feature_buffer에 feature 누적
    3. 지정 시간 이후 평균값 계산
    4. baseline.pkl 저장

    이 클래스는 PyQt, OpenCV, MediaPipe를 모른다.
    오직 feature 데이터만 처리한다.
    """

    def __init__(self, baseline_path, duration=5):
        self.baseline_path = Path(baseline_path)
        self.duration = duration

        self.feature_buffer = []
        self.start_time = None
        self.is_running = False

    def start(self):
        """
        Calibration Start 버튼을 눌렀을 때 호출된다.
        """

        self.feature_buffer.clear()
        self.start_time = time.time()
        self.is_running = True

        return CalibrationResult(
            is_finished=False,
            success=True,
            message=f"초기 자세 측정을 시작합니다. {self.duration}초 동안 바른 자세를 유지해주세요.",
            remain_time=float(self.duration),
            sample_count=0,
            baseline_path=str(self.baseline_path)
        )

    def update(self, raw_features):
        """
        카메라 프레임마다 호출된다.

        CameraWorker가 calculate_features()로 뽑은 raw_features를
        이 함수에 넘겨주면 된다.
        """

        if not self.is_running:
            return CalibrationResult(
                is_finished=False,
                success=False,
                message="초기값 측정 중 이 아닙니다."
            )

        if raw_features is None:
            return self._build_running_result("아직 준비되지 않았습니다.")

        feature_array = np.asarray(raw_features, dtype=np.float32)

        if feature_array.size == 0 or not np.any(feature_array):
            return self._build_running_result("자세를 다시 잡아주세요.")

        self.feature_buffer.append(feature_array)

        elapsed = time.time() - self.start_time
        remain_time = max(0.0, self.duration - elapsed)

        # 측정 시간 지나면 Finsh 호출해서 baseline으로 저장하는 부분입니다.
        if elapsed >= self.duration:
            return self.finish()

        return CalibrationResult(
            is_finished=False,
            success=True,
            # message=f"초기 자세 측정 중... 남은 시간 {remain_time:.1f}초 / 수집 샘플 {len(self.feature_buffer)}개",
            message=f"초기 자세 측정 중... 남은 시간 {remain_time:.1f}초",
            remain_time=remain_time,
            sample_count=len(self.feature_buffer),
            baseline_path=str(self.baseline_path)
        )

    def finish(self):
        """
        feature 평균값을 계산하고 baseline.pkl로 저장한다.
        """

        self.is_running = False

        if len(self.feature_buffer) == 0:
            return CalibrationResult(
                is_finished=True,
                success=False,
                message="수집된 자세 데이터가 없습니다. 다시 시도해주세요.",
                sample_count=0,
                baseline_path=str(self.baseline_path)
            )

        baseline_avg = np.mean(self.feature_buffer, axis=0)

        self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
        joblib.dump(baseline_avg, self.baseline_path)

        return CalibrationResult(
            is_finished=True,
            success=True,
            # message=(
            #     "Calibration 완료! "
            #     f"샘플 {len(self.feature_buffer)}개를 기준으로 baseline.pkl을 저장했습니다."
            # ),
            message=(
                "초기값 측정 완료!"
                # f"샘플 {len(self.feature_buffer)}개를 기준으로 baseline.pkl을 저장했습니다."
            ),
            remain_time=0.0,
            sample_count=len(self.feature_buffer),
            baseline_path=str(self.baseline_path)
        )

    def cancel(self):
        """
        카메라를 끄거나 중간에 취소할 때 사용한다.
        """

        self.is_running = False
        self.feature_buffer.clear()
        self.start_time = None

    def _build_running_result(self, message):
        elapsed = 0.0

        if self.start_time is not None:
            elapsed = time.time() - self.start_time

        remain_time = max(0.0, self.duration - elapsed)

        return CalibrationResult(
            is_finished=False,
            success=False,
            message=message,
            remain_time=remain_time,
            sample_count=len(self.feature_buffer),
            baseline_path=str(self.baseline_path)
        )
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
    def __init__(self, baseline_path, duration=5, expected_fps=30, min_sample_ratio=0.6, wait_timeout=10):
        self.baseline_path = Path(baseline_path)
        self.duration = duration
        self.expected_fps = expected_fps
        self.min_sample_ratio = min_sample_ratio
        self.wait_timeout = wait_timeout

        self.target_sample_count = int(duration * expected_fps)
        self.min_sample_count = max(1, int(self.target_sample_count * min_sample_ratio))

        self.feature_buffer = []
        self.request_time = None
        self.start_time = None
        self.is_running = False

    def start(self):
        self.feature_buffer.clear()
        self.request_time = time.monotonic()
        self.start_time = None
        self.is_running = True

        return CalibrationResult(
            is_finished=False,
            success=True,
            message=f"랜드마크 인식 후 {self.duration}초 동안 초기값을 측정합니다.",
            remain_time=float(self.duration),
            sample_count=0,
            baseline_path=str(self.baseline_path)
        )

    def update(self, raw_features):
        if not self.is_running:
            return CalibrationResult(
                is_finished=False,
                success=False,
                message="초기값 측정 중이 아닙니다."
            )

        # 아직 유효 Feature가 한 번도 들어오지 않은 상태
        if raw_features is None:
            return self._handle_invalid_feature("랜드마크 인식을 기다리고 있습니다.")

        feature_array = np.asarray(raw_features, dtype=np.float32).reshape(-1)

        if feature_array.size == 0:
            return self._handle_invalid_feature("Feature가 비어 있습니다.")

        # 0 값 자체는 정상 Feature일 수 있으므로 np.any() 검사하지 않는다.
        if not np.all(np.isfinite(feature_array)):
            return self._handle_invalid_feature("Feature에 NaN 또는 Inf가 포함되어 있습니다.")

        # 첫 번째 정상 Feature가 들어온 순간부터 진짜 Calibration 5초 시작
        if self.start_time is None:
            self.start_time = time.monotonic()

        self.feature_buffer.append(feature_array)

        elapsed = time.monotonic() - self.start_time
        remain_time = max(0.0, self.duration - elapsed)

        if elapsed >= self.duration:
            return self.finish()

        return CalibrationResult(
            is_finished=False,
            success=True,
            message=f"초기값 측정 중... 남은 시간 {remain_time:.1f}초",
            remain_time=remain_time,
            sample_count=len(self.feature_buffer),
            baseline_path=str(self.baseline_path)
        )

    def _handle_invalid_feature(self, message):
        # 첫 Feature를 너무 오래 못 찾으면 무한 대기하지 않고 실패
        if self.start_time is None:
            wait_elapsed = time.monotonic() - self.request_time

            if wait_elapsed >= self.wait_timeout:
                self.is_running = False

                return CalibrationResult(
                    is_finished=True,
                    success=False,
                    message=f"{self.wait_timeout}초 동안 유효한 Feature를 찾지 못했습니다.",
                    remain_time=float(self.duration),
                    sample_count=0,
                    baseline_path=str(self.baseline_path)
                )

            return CalibrationResult(
                is_finished=False,
                success=False,
                message=message,
                remain_time=float(self.duration),
                sample_count=0,
                baseline_path=str(self.baseline_path)
            )

        elapsed = time.monotonic() - self.start_time

        if elapsed >= self.duration:
            return self.finish()

        return CalibrationResult(
            is_finished=False,
            success=False,
            message=message,
            remain_time=max(0.0, self.duration - elapsed),
            sample_count=len(self.feature_buffer),
            baseline_path=str(self.baseline_path)
        )

    def finish(self):
        self.is_running = False
        sample_count = len(self.feature_buffer)

        # 유효 샘플 부족 시 기존 baseline을 덮어쓰지 않는다.
        if sample_count < self.min_sample_count:
            return CalibrationResult(
                is_finished=True,
                success=False,
                message=(
                    f"유효 데이터가 부족합니다. "
                    f"{sample_count}/{self.min_sample_count}개 "
                    f"(목표 {self.target_sample_count}개)"
                ),
                remain_time=0.0,
                sample_count=sample_count,
                baseline_path=str(self.baseline_path)
            )

        baseline_avg = np.mean(self.feature_buffer, axis=0)

        self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
        joblib.dump(baseline_avg, self.baseline_path)

        return CalibrationResult(
            is_finished=True,
            success=True,
            message=f"초기값 측정 완료! 샘플 {sample_count}개를 저장했습니다.",
            remain_time=0.0,
            sample_count=sample_count,
            baseline_path=str(self.baseline_path)
        )

    def cancel(self):
        self.is_running = False
        self.feature_buffer.clear()
        self.request_time = None
        self.start_time = None
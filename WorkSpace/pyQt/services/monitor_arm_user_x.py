"""Motor1/2 모니터암의 사용자 X 계산 계층.

실제 ToF 거리 센서 서비스는 ``tof_service.py``가 담당한다. 이 모듈은
센서 거리값을 POCO base 좌표계의 user X로 변환하고, 기존 PoseProcess가
전달하는 직렬 landmark에서 눈 간격 기반 거리 추정값을 만든 뒤 ToF와
Vision 결과를 융합한다.

팀원 standalone의 ToFUserXSource / EyeGapVisionDistanceEstimator /
UserXFusion 계산식을 그대로 유지하되, Pose 입력 경계만 POCO의
``[x, y, z, visibility]`` 직렬 landmark 형식에 맞춘다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


LEFT_EYE_INDEX = 2
RIGHT_EYE_INDEX = 5


@dataclass(frozen=True)
class EyeMeasurement:
    gap_px: float
    left_xy: tuple[int, int]
    right_xy: tuple[int, int]


class ToFUserXSource:
    """ToF service distance를 base 좌표계의 사용자 X로 변환한다."""

    def __init__(
        self,
        sensor_service,
        sensor_origin_x_m: float,
        minimum_user_x_m: float,
        maximum_user_x_m: float,
    ):
        self.sensor_service = sensor_service
        self.sensor_origin_x_m = float(sensor_origin_x_m)
        self.minimum_user_x_m = float(minimum_user_x_m)
        self.maximum_user_x_m = float(maximum_user_x_m)

        if self.minimum_user_x_m > self.maximum_user_x_m:
            raise ValueError("ToF 사용자 X 최소값이 최대값보다 큽니다.")

    def open(self) -> bool:
        return bool(self.sensor_service.open())

    def close(self) -> None:
        self.sensor_service.close()

    def read_user_x_m(self) -> float:
        range_m = float(self.sensor_service.read_distance_m())
        user_x_m = self.sensor_origin_x_m + range_m
        return self.validate_user_x_m(user_x_m)

    def validate_user_x_m(self, user_x_m: float) -> float:
        user_x_m = float(user_x_m)

        if not math.isfinite(user_x_m):
            raise ValueError("ToF 사용자 X가 유한한 값이 아닙니다.")

        if not self.minimum_user_x_m <= user_x_m <= self.maximum_user_x_m:
            raise ValueError(
                f"ToF 사용자 X {user_x_m:.3f}m가 허용범위 "
                f"{self.minimum_user_x_m:.3f}~{self.maximum_user_x_m:.3f}m 밖입니다."
            )

        return user_x_m


class EyeGapVisionDistanceEstimator:
    """눈 간격을 기준으로 핀홀 카메라의 반비례 거리값을 계산한다.

    같은 사람과 같은 카메라에서는 ``눈 간격 픽셀값 * 실제 거리``가 거의
    일정하다는 팀원 standalone의 계산 원리를 그대로 사용한다. 첫 유효
    측정에서 ToF 거리를 기준값으로 보정하고 이후 EMA 필터를 적용한다.
    """

    def __init__(
        self,
        minimum_eye_gap_px: float,
        minimum_distance_m: float,
        maximum_distance_m: float,
        filter_alpha: float = 0.25,
    ):
        self.minimum_eye_gap_px = float(minimum_eye_gap_px)
        self.minimum_distance_m = float(minimum_distance_m)
        self.maximum_distance_m = float(maximum_distance_m)
        self.filter_alpha = float(filter_alpha)

        if self.minimum_eye_gap_px <= 0.0:
            raise ValueError("minimum_eye_gap_px는 0보다 커야 합니다.")

        if self.minimum_distance_m >= self.maximum_distance_m:
            raise ValueError("비전 최소 거리는 최대 거리보다 작아야 합니다.")

        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("vision filter_alpha는 0 초과 1 이하여야 합니다.")

        self.reference_gap_px: float | None = None
        self.reference_distance_m: float | None = None
        self.filtered_distance_m: float | None = None

    @property
    def calibrated(self) -> bool:
        return self.reference_gap_px is not None

    def reset(self) -> None:
        self.reference_gap_px = None
        self.reference_distance_m = None
        self.filtered_distance_m = None

    def calibrate(
        self,
        eye_gap_px: float,
        reference_distance_m: float,
    ) -> None:
        gap = float(eye_gap_px)
        distance = float(reference_distance_m)

        if gap < self.minimum_eye_gap_px:
            raise ValueError(
                f"눈 간격 {gap:.1f}px가 너무 작아 기준 보정을 할 수 없습니다."
            )

        if not self.minimum_distance_m <= distance <= self.maximum_distance_m:
            raise ValueError(
                f"비전 기준거리 {distance:.3f}m가 허용범위 "
                f"{self.minimum_distance_m:.3f}~"
                f"{self.maximum_distance_m:.3f}m 밖입니다."
            )

        self.reference_gap_px = gap
        self.reference_distance_m = distance
        self.filtered_distance_m = distance

    def estimate_distance_m(
        self,
        eye_gap_px: float,
    ) -> float:
        if (
            not self.calibrated
            or self.reference_distance_m is None
        ):
            raise ValueError(
                "비전 눈 간격 기준이 아직 보정되지 않았습니다."
            )

        gap = float(eye_gap_px)

        if (
            not math.isfinite(gap)
            or gap < self.minimum_eye_gap_px
        ):
            raise ValueError(
                f"유효하지 않은 눈 간격입니다: {gap:.1f}px"
            )

        distance = (
            self.reference_distance_m
            * self.reference_gap_px
            / gap
        )

        if not self.minimum_distance_m <= distance <= self.maximum_distance_m:
            raise ValueError(
                f"비전 거리 {distance:.3f}m가 허용범위 "
                f"{self.minimum_distance_m:.3f}~"
                f"{self.maximum_distance_m:.3f}m 밖입니다."
            )

        if self.filtered_distance_m is None:
            self.filtered_distance_m = distance
        else:
            alpha = self.filter_alpha
            self.filtered_distance_m = (
                alpha * distance
                + (1.0 - alpha)
                * self.filtered_distance_m
            )

        return self.filtered_distance_m


class UserXFusion:
    """ToF와 Vision의 base-user X를 가중 평균한다."""

    def __init__(
        self,
        tof_weight: float = 0.7,
        vision_weight: float = 0.3,
    ):
        self.tof_weight = float(tof_weight)
        self.vision_weight = float(vision_weight)

        if (
            self.tof_weight < 0.0
            or self.vision_weight < 0.0
        ):
            raise ValueError(
                "센서 융합 가중치는 음수일 수 없습니다."
            )

        total = self.tof_weight + self.vision_weight

        if total <= 0.0:
            raise ValueError(
                "센서 융합 가중치 합은 0보다 커야 합니다."
            )

        self.tof_weight /= total
        self.vision_weight /= total

    def fuse(
        self,
        tof_user_x_m: float,
        vision_user_x_m: float | None,
    ) -> float:
        tof_x = float(tof_user_x_m)

        if not math.isfinite(tof_x):
            raise ValueError(
                "ToF 사용자 X가 유한한 값이 아닙니다."
            )

        # Vision landmark가 순간적으로 불안정할 경우
        # 팀원 원본과 동일하게 ToF 단독값으로 fallback한다.
        if vision_user_x_m is None:
            return tof_x

        vision_x = float(vision_user_x_m)

        if not math.isfinite(vision_x):
            return tof_x

        return (
            self.tof_weight * tof_x
            + self.vision_weight * vision_x
        )


def measure_pose_eye_gap(
    landmarks,
    width: int,
    height: int,
) -> EyeMeasurement | None:
    """POCO 직렬 Pose landmark에서 팀원 코드와 같은 눈 간격(px)을 계산한다.

    PoseProcess는 MediaPipe 객체가 아니라 ``[x, y, z, visibility]`` 배열을
    HardwareProcess로 전달한다.

    팀원 standalone의 presence 검사는 값이 없을 경우 기본 1.0을 사용하므로,
    여기서는 직렬 데이터에 실제로 존재하는 visibility만 동일 기준(0.5)으로
    검사한다.
    """
    if (
        landmarks is None
        or len(landmarks) <= RIGHT_EYE_INDEX
    ):
        return None

    left = landmarks[LEFT_EYE_INDEX]
    right = landmarks[RIGHT_EYE_INDEX]

    for landmark in (left, right):
        if (
            not isinstance(landmark, (list, tuple))
            or len(landmark) < 2
        ):
            return None

        visibility = (
            float(landmark[3])
            if len(landmark) > 3
            else 1.0
        )

        if visibility < 0.5:
            return None

    left_xy = (
        round(float(left[0]) * int(width)),
        round(float(left[1]) * int(height)),
    )

    right_xy = (
        round(float(right[0]) * int(width)),
        round(float(right[1]) * int(height)),
    )

    gap_px = math.hypot(
        left_xy[0] - right_xy[0],
        left_xy[1] - right_xy[1],
    )

    return EyeMeasurement(
        gap_px=gap_px,
        left_xy=left_xy,
        right_xy=right_xy,
    )
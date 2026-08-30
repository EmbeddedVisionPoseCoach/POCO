"""Temporary ToF user-X source for the Motor1/2 monitor-arm controller.

The current project uses a fixed distance as a stand-in for a real ToF sensor.
Only ``read_range_m()`` needs to change when the actual ToF driver is connected;
the base-coordinate conversion and valid user-X range checks stay the same.
"""

from __future__ import annotations

import math


class FixedToFUserXSource:
    """Return user X from a temporary fixed ToF range value."""

    def __init__(
        self,
        sensor_origin_x_m: float,
        fixed_range_m: float,
        minimum_user_x_m: float,
        maximum_user_x_m: float,
    ):
        self.sensor_origin_x_m = float(sensor_origin_x_m)
        self.fixed_range_m = float(fixed_range_m)
        self.minimum_user_x_m = float(minimum_user_x_m)
        self.maximum_user_x_m = float(maximum_user_x_m)

        if self.minimum_user_x_m > self.maximum_user_x_m:
            raise ValueError("ToF 사용자 X 최소값이 최대값보다 큽니다.")

    def read_range_m(self) -> float:
        """Return fixed data until the real ToF driver is connected.

        실제 ToF 센서를 연결할 때 이 메서드만 센서 측정값을 반환하도록
        교체하고, ``read_user_x_m()``의 좌표 변환/범위 검사는 그대로 유지한다.
        """
        return self.fixed_range_m

    def read_user_x_m(self) -> float:
        range_m = float(self.read_range_m())
        user_x_m = self.sensor_origin_x_m + range_m

        if not math.isfinite(user_x_m):
            raise ValueError("ToF 사용자 X가 유한한 값이 아닙니다.")

        if not self.minimum_user_x_m <= user_x_m <= self.maximum_user_x_m:
            raise ValueError(
                f"ToF 사용자 X {user_x_m:.3f}m가 허용범위 "
                f"{self.minimum_user_x_m:.3f}~{self.maximum_user_x_m:.3f}m 밖입니다."
            )

        return user_x_m
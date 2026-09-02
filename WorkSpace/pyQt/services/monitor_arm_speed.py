"""Shared fixed/adaptive speed policy for Servo 1 and 2 commands."""

from __future__ import annotations


ABSOLUTE_SPEED_CAP = 1000
FIXED_SPEED_MODE = "fixed"
ADAPTIVE_SPEED_MODE = "adaptive"


def validate_speed_profile(
    mode: str,
    maximum_speed: int,
    minimum_speed: int,
    full_speed_error_deg: float,
) -> None:
    mode = str(mode)
    maximum_speed = int(maximum_speed)
    minimum_speed = int(minimum_speed)
    full_speed_error_deg = float(full_speed_error_deg)
    if mode not in (FIXED_SPEED_MODE, ADAPTIVE_SPEED_MODE):
        raise ValueError(f"알 수 없는 속도 모드입니다: {mode}")
    if not 1 <= maximum_speed <= ABSOLUTE_SPEED_CAP:
        raise ValueError(f"최대 Speed 허용범위는 1~{ABSOLUTE_SPEED_CAP}입니다.")
    if not 1 <= minimum_speed <= maximum_speed:
        raise ValueError("최소 Speed는 1 이상이고 최대 Speed 이하여야 합니다.")
    if not 1.0 <= full_speed_error_deg <= 180.0:
        raise ValueError("최대속도 도달 오차는 1~180° 범위여야 합니다.")


def calculate_adaptive_speed(
    minimum_speed: int,
    maximum_speed: int,
    error_deg: float,
    full_speed_error_deg: float,
) -> int:
    """Linearly map current-target joint error to a speed no greater than 1000."""
    maximum_speed = min(int(maximum_speed), ABSOLUTE_SPEED_CAP)
    minimum_speed = min(int(minimum_speed), maximum_speed)
    ratio = min(1.0, max(0.0, float(error_deg)) / float(full_speed_error_deg))
    speed = round(minimum_speed + (maximum_speed - minimum_speed) * ratio)
    return min(max(speed, 1), ABSOLUTE_SPEED_CAP)


def select_speed(
    mode: str,
    maximum_speed: int,
    minimum_speed: int,
    full_speed_error_deg: float,
    error_deg: float,
) -> int:
    """Return fixed speed or an error-proportional adaptive speed."""
    validate_speed_profile(
        mode,
        maximum_speed,
        minimum_speed,
        full_speed_error_deg,
    )
    if mode == FIXED_SPEED_MODE:
        return int(maximum_speed)
    return calculate_adaptive_speed(
        minimum_speed,
        maximum_speed,
        error_deg,
        full_speed_error_deg,
    )

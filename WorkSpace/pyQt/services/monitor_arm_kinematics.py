"""Two-joint monitor-arm kinematics and motion safety checks.

Only shoulder_lift (servo 1) and elbow_flex (servo 2) exist in this model.
The monitor center is modelled as being MONITOR_OFFSET_M farther along the
second link from its end motor centre.  Wrist joints are intentionally absent.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT_DIR = Path(__file__).resolve().parent
DEFAULT_SETTINGS_PATH = ROOT_DIR / "monitor_arm_settings.json"


class KinematicsError(ValueError):
    """The requested monitor position has no safe two-joint solution."""


class MotionSafetyError(ValueError):
    """The endpoint is valid, but the commanded path violates a safety rule."""


@dataclass(frozen=True)
class ArmGeometry:
    """Planar geometry in metres, based on the SO-101 URDF axis transforms."""

    # The fixed base-to-shoulder link is mounted vertically. Its original
    # 13.5606 cm length is preserved while its X component is removed.
    shoulder_x_m: float = 0.0
    shoulder_z_m: float = math.hypot(0.0692345, 0.1166)
    upper_link_m: float = math.hypot(0.028, 0.11257)
    lower_link_m: float = math.hypot(0.1349, 0.0052)
    upper_zero_angle_rad: float = math.atan2(0.11257, 0.028)
    lower_zero_angle_rad: float = math.atan2(0.0052, 0.1349)
    monitor_offset_m: float = 0.07

    @property
    def effective_lower_link_m(self) -> float:
        return self.lower_link_m + self.monitor_offset_m

    @classmethod
    def from_settings(cls, settings: dict) -> "ArmGeometry":
        source = settings.get("geometry", {})
        defaults = cls()
        return cls(
            shoulder_x_m=float(source.get("shoulder_x_m", defaults.shoulder_x_m)),
            shoulder_z_m=float(source.get("shoulder_z_m", defaults.shoulder_z_m)),
            upper_link_m=float(source.get("upper_link_m", defaults.upper_link_m)),
            lower_link_m=float(source.get("lower_link_m", defaults.lower_link_m)),
            upper_zero_angle_rad=float(
                source.get("upper_zero_angle_rad", defaults.upper_zero_angle_rad)
            ),
            lower_zero_angle_rad=float(
                source.get("lower_zero_angle_rad", defaults.lower_zero_angle_rad)
            ),
            monitor_offset_m=float(
                source.get("monitor_offset_m", defaults.monitor_offset_m)
            ),
        )


@dataclass(frozen=True)
class JointCommand:
    """Angles in the team-facing motor_control convention, in degrees."""

    shoulder_lift_deg: float
    elbow_flex_deg: float

    def interpolate(self, other: "JointCommand", ratio: float) -> "JointCommand":
        ratio = float(ratio)
        return JointCommand(
            self.shoulder_lift_deg
            + (other.shoulder_lift_deg - self.shoulder_lift_deg) * ratio,
            self.elbow_flex_deg
            + (other.elbow_flex_deg - self.elbow_flex_deg) * ratio,
        )


@dataclass(frozen=True)
class MonitorPose:
    x_m: float
    z_m: float


def monitor_target_from_user(
    user_x_m: float,
    user_monitor_distance_m: float,
    monitor_z_m: float,
) -> MonitorPose:
    """Convert user coordinates to a monitor-centre target.

    Coordinate convention:
      origin: clamp-to-robot-base connection
      +X: from the base toward the user
      +Z: vertically upward from the ground/base origin

    The monitor is kept between the base and user, so its X coordinate is the
    user's X coordinate minus the requested user-to-monitor distance.
    """
    user_x_m = float(user_x_m)
    distance_m = float(user_monitor_distance_m)
    monitor_z_m = float(monitor_z_m)
    if not all(math.isfinite(value) for value in (user_x_m, distance_m, monitor_z_m)):
        raise ValueError("사용자 X, 고정거리, 모니터 Z는 유한한 숫자여야 합니다.")
    if distance_m <= 0.0:
        raise ValueError("사용자-모니터 고정거리는 0보다 커야 합니다.")
    if user_x_m <= distance_m:
        raise ValueError("사용자 X는 사용자-모니터 고정거리보다 커야 합니다.")
    if monitor_z_m < 0.0:
        raise ValueError("모니터 Z는 0 이상이어야 합니다.")
    return MonitorPose(x_m=user_x_m - distance_m, z_m=monitor_z_m)


@dataclass(frozen=True)
class SafetyLimits:
    shoulder_min_deg: float
    shoulder_max_deg: float
    elbow_min_deg: float
    elbow_max_deg: float
    vertical_tolerance_m: float
    max_joint_step_deg: float
    path_samples: int

    @classmethod
    def from_settings(cls, settings: dict) -> "SafetyLimits":
        source = settings["safety"]
        soft = source["soft_joint_limits_deg"]
        return cls(
            shoulder_min_deg=float(soft["shoulder_lift"]["min"]),
            shoulder_max_deg=float(soft["shoulder_lift"]["max"]),
            elbow_min_deg=float(soft["elbow_flex"]["min"]),
            elbow_max_deg=float(soft["elbow_flex"]["max"]),
            vertical_tolerance_m=float(source["vertical_tolerance_m"]),
            max_joint_step_deg=float(source["max_joint_step_deg"]),
            path_samples=max(2, int(source["path_samples"])),
        )


class TwoJointMonitorArm:
    """Forward/inverse kinematics for servos 1 and 2 only."""

    # Kept in sync with motor_control/config.py. Both TEAM + directions map to
    # URDF -, while final raw/zero conversion remains CalibrationManager's job.
    SHOULDER_COMMAND_TO_URDF = -1.0
    ELBOW_COMMAND_TO_URDF = -1.0

    def __init__(self, geometry: ArmGeometry | None = None):
        self.geometry = geometry or ArmGeometry()

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(value, maximum))

    def command_to_urdf(self, command: JointCommand) -> tuple[float, float]:
        shoulder = math.radians(
            command.shoulder_lift_deg * self.SHOULDER_COMMAND_TO_URDF
        )
        elbow = math.radians(command.elbow_flex_deg * self.ELBOW_COMMAND_TO_URDF)
        return shoulder, elbow

    def urdf_to_command(self, shoulder_rad: float, elbow_rad: float) -> JointCommand:
        return JointCommand(
            math.degrees(shoulder_rad) / self.SHOULDER_COMMAND_TO_URDF,
            math.degrees(elbow_rad) / self.ELBOW_COMMAND_TO_URDF,
        )

    def forward(self, command: JointCommand) -> MonitorPose:
        shoulder, elbow = self.command_to_urdf(command)
        g = self.geometry
        upper_world = g.upper_zero_angle_rad - shoulder
        lower_world = g.lower_zero_angle_rad - shoulder - elbow
        x_m = (
            g.shoulder_x_m
            + g.upper_link_m * math.cos(upper_world)
            + g.effective_lower_link_m * math.cos(lower_world)
        )
        z_m = (
            g.shoulder_z_m
            + g.upper_link_m * math.sin(upper_world)
            + g.effective_lower_link_m * math.sin(lower_world)
        )
        return MonitorPose(x_m=x_m, z_m=z_m)

    def inverse(self, target_x_m: float, target_z_m: float) -> JointCommand:
        """Solve the non-folded monitor-arm branch for a target monitor centre."""
        g = self.geometry
        dx = float(target_x_m) - g.shoulder_x_m
        dz = float(target_z_m) - g.shoulder_z_m
        l1 = g.upper_link_m
        l2 = g.effective_lower_link_m
        radius = math.hypot(dx, dz)
        minimum_reach = abs(l1 - l2)
        maximum_reach = l1 + l2

        if radius < minimum_reach or radius > maximum_reach:
            raise KinematicsError(
                f"목표가 2축 가동범위 밖입니다: shoulder 거리={radius:.3f}m, "
                f"허용={minimum_reach:.3f}~{maximum_reach:.3f}m"
            )

        cosine = (radius * radius - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        relative_world = -math.acos(self._clamp(cosine, -1.0, 1.0))
        upper_world = math.atan2(dz, dx) - math.atan2(
            l2 * math.sin(relative_world),
            l1 + l2 * math.cos(relative_world),
        )

        shoulder_urdf = g.upper_zero_angle_rad - upper_world
        elbow_urdf = (
            g.lower_zero_angle_rad - g.upper_zero_angle_rad - relative_world
        )
        return self.urdf_to_command(shoulder_urdf, elbow_urdf)

    def validate_motion(
        self,
        current: JointCommand,
        target: JointCommand,
        reference_z_m: float,
        limits: SafetyLimits,
        calibration_ranges: dict[str, tuple[float, float]] | None = None,
        enforce_step_limit: bool = True,
    ) -> None:
        """Reject joint-limit or vertical-path unsafe commands.

        ``enforce_step_limit`` keeps the legacy per-command joint-step guard for
        manual/stepped control.  Direct-target control disables only that guard;
        joint limits and the complete interpolated Z path are still checked.
        """
        ranges = {
            "shoulder_lift": (limits.shoulder_min_deg, limits.shoulder_max_deg),
            "elbow_flex": (limits.elbow_min_deg, limits.elbow_max_deg),
        }

        if calibration_ranges is not None:
            for name in ranges:
                soft_min, soft_max = ranges[name]
                hard_min, hard_max = calibration_ranges[name]
                ranges[name] = (max(soft_min, hard_min), min(soft_max, hard_max))

        targets = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        for name, angle in targets.items():
            minimum, maximum = ranges[name]
            if minimum > maximum or not minimum <= angle <= maximum:
                raise MotionSafetyError(
                    f"{name} 목표 {angle:+.2f}°가 안전범위 "
                    f"{minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                )

        joint_steps = (
            abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(target.elbow_flex_deg - current.elbow_flex_deg),
        )
        if enforce_step_limit and max(joint_steps) > limits.max_joint_step_deg:
            raise MotionSafetyError(
                f"한 주기 관절 변화 {max(joint_steps):.2f}°가 제한 "
                f"{limits.max_joint_step_deg:.2f}°를 초과합니다."
            )

        for index in range(limits.path_samples + 1):
            ratio = index / limits.path_samples
            sample = current.interpolate(target, ratio)
            pose = self.forward(sample)
            vertical_error = abs(pose.z_m - reference_z_m)
            if vertical_error > limits.vertical_tolerance_m:
                raise MotionSafetyError(
                    f"예상 이동경로의 수직 편차 {vertical_error:.3f}m가 제한 "
                    f"{limits.vertical_tolerance_m:.3f}m를 초과합니다. 자세 유지."
                )


def load_settings(path: Path = DEFAULT_SETTINGS_PATH) -> dict:
    with Path(path).open("r", encoding="utf-8") as file:
        return json.load(file)


def save_settings(data: dict, path: Path = DEFAULT_SETTINGS_PATH) -> None:
    """Atomically save settings edited by the manual limit UI."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=2, ensure_ascii=False)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def iter_joint_values(command: JointCommand) -> Iterable[tuple[str, float]]:
    yield "shoulder_lift", command.shoulder_lift_deg
    yield "elbow_flex", command.elbow_flex_deg

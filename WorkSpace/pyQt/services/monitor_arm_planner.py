"""Motor1/2 monitor-arm target planner extracted from the teammate standalone flow.

This service keeps the original two-joint IK, distance tracking, motion-safety,
and working-pose recovery calculations. It does not own cameras, ToF hardware,
serial access, or motor command timing; those responsibilities stay in the
existing POCO process/service layers.
"""

from __future__ import annotations

import time

from services.monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    MotionSafetyError,
    SafetyLimits,
    TwoJointMonitorArm,
    monitor_target_from_user,
)


class RecoveryTimeoutError(MotionSafetyError):
    """Raised when working-pose recovery cannot settle before its deadline."""


class MonitorArmPlanner:
    """Calculate safe shoulder/elbow targets from the current pose and user X."""

    def __init__(self, settings: dict):
        self.settings = settings
        self.kinematics = TwoJointMonitorArm(ArmGeometry.from_settings(settings))
        self.limits = SafetyLimits.from_settings(settings)
        distance = settings["distance"]
        self.desired_distance_m = float(distance["desired_user_monitor_distance_m"])
        self.deadband_m = float(distance["deadband_m"])
        self.max_x_step_m = float(distance["max_monitor_x_step_m"])
        control = settings.get("control", {})
        self.joint_command_mode = str(
            control.get("pose_joint_command_mode", "direct")
        ).strip().lower()
        if self.joint_command_mode not in {"direct", "stepped"}:
            raise ValueError(
                "control.pose_joint_command_mode는 direct 또는 stepped여야 합니다."
            )
        self.reference_z_m: float | None = None
        cartesian = settings.get("manual_cartesian", {})
        self.working_z_min_m = float(cartesian.get("monitor_z_min_m", 0.20))
        self.working_z_max_m = float(cartesian.get("monitor_z_max_m", 0.30))
        self.default_working_z_m = float(
            cartesian.get("default_monitor_z_m", 0.2560722511328793)
        )
        postures = settings.get("postures", {})
        working = postures.get("working", {})
        self.working_command = JointCommand(
            float(working.get("shoulder_lift_deg", 0.0)),
            float(working.get("elbow_flex_deg", 0.0)),
        )
        self.working_start_arrival_tolerance_deg = max(
            0.01,
            float(control.get("working_start_arrival_tolerance_deg", 1.0)),
        )
        self.working_start_stable_samples = max(
            1,
            int(control.get("working_start_stable_samples", 3)),
        )
        self.working_start_timeout_sec = max(
            0.1,
            float(control.get("working_start_timeout_sec", 25.0)),
        )
        self.recovery_active = False
        self.recovery_started_at: float | None = None
        self.recovery_stable_sample_count = 0
        self.recovery_largest_error_deg: float | None = None

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(value, maximum))

    @staticmethod
    def _outside_distance(value: float, minimum: float, maximum: float) -> float:
        if value < minimum:
            return minimum - value
        if value > maximum:
            return value - maximum
        return 0.0

    def set_vertical_reference(self, current: JointCommand) -> float:
        current_z_m = self.kinematics.forward(current).z_m
        if self.working_z_min_m <= current_z_m <= self.working_z_max_m:
            self.reference_z_m = current_z_m
            self.cancel_working_pose_recovery()
        else:
            self.reference_z_m = self.default_working_z_m
            self.request_working_pose_recovery()
        return self.reference_z_m

    def request_working_pose_recovery(self) -> None:
        """Latch recovery until the configured working posture is reached."""
        self.reference_z_m = self.default_working_z_m
        self.recovery_active = True
        self.recovery_started_at = time.monotonic()
        self.recovery_stable_sample_count = 0
        self.recovery_largest_error_deg = None

    def cancel_working_pose_recovery(self) -> None:
        """Stop recovery without treating the current pose as an arrival."""
        self.recovery_active = False
        self.recovery_started_at = None
        self.recovery_stable_sample_count = 0

    def _effective_joint_ranges(
        self,
        calibration_ranges: dict[str, tuple[float, float]] | None,
    ) -> dict[str, tuple[float, float]]:
        ranges = {
            "shoulder_lift": (
                self.limits.shoulder_min_deg,
                self.limits.shoulder_max_deg,
            ),
            "elbow_flex": (
                self.limits.elbow_min_deg,
                self.limits.elbow_max_deg,
            ),
        }
        if calibration_ranges is not None:
            for joint, (soft_min, soft_max) in tuple(ranges.items()):
                hard_min, hard_max = calibration_ranges[joint]
                ranges[joint] = (
                    max(soft_min, float(hard_min)),
                    min(soft_max, float(hard_max)),
                )
        return ranges

    def _validate_recovery_step(
        self,
        current: JointCommand,
        target: JointCommand,
        calibration_ranges: dict[str, tuple[float, float]] | None,
    ) -> None:
        ranges = self._effective_joint_ranges(calibration_ranges)
        current_angles = {
            "shoulder_lift": current.shoulder_lift_deg,
            "elbow_flex": current.elbow_flex_deg,
        }
        target_angles = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        for joint, (minimum, maximum) in ranges.items():
            current_outside = self._outside_distance(
                current_angles[joint], minimum, maximum
            )
            target_outside = self._outside_distance(
                target_angles[joint], minimum, maximum
            )
            if current_outside <= 1e-6 and target_outside > 1e-6:
                raise MotionSafetyError(
                    f"복구 스텝 {joint}={target_angles[joint]:+.2f}°가 "
                    f"안전범위 {minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                )
            if current_outside > 1e-6 and target_outside >= current_outside - 1e-6:
                raise MotionSafetyError(
                    f"복구 스텝이 {joint} 안전범위에서 더 멀어집니다."
                )

        current_pose = self.kinematics.forward(current)
        target_pose = self.kinematics.forward(target)
        current_z_outside = self._outside_distance(
            current_pose.z_m,
            self.working_z_min_m,
            self.working_z_max_m,
        )
        for index in range(self.limits.path_samples + 1):
            ratio = index / self.limits.path_samples
            pose = self.kinematics.forward(current.interpolate(target, ratio))
            sample_z_outside = self._outside_distance(
                pose.z_m,
                self.working_z_min_m,
                self.working_z_max_m,
            )
            if current_z_outside <= 1e-4 and sample_z_outside > 1e-4:
                raise MotionSafetyError("복구 중 안전 Z 범위 밖으로 나가는 경로입니다.")
            if (
                current_z_outside > 1e-4
                and sample_z_outside
                > current_z_outside + self.limits.vertical_tolerance_m
            ):
                raise MotionSafetyError(
                    "복구 중 Z가 현재 이탈량보다 vertical_tolerance 이상 "
                    "더 벗어납니다."
                )
            expected_z_m = current_pose.z_m + (
                target_pose.z_m - current_pose.z_m
            ) * ratio
            if abs(pose.z_m - expected_z_m) > self.limits.vertical_tolerance_m:
                raise MotionSafetyError("복구 중 예상 Z 경로 편차가 너무 큽니다.")

    def _plan_working_pose_recovery(
        self,
        current: JointCommand,
        calibration_ranges: dict[str, tuple[float, float]] | None,
    ) -> JointCommand | None:
        largest_joint_change = max(
            abs(self.working_command.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(self.working_command.elbow_flex_deg - current.elbow_flex_deg),
        )
        self.recovery_largest_error_deg = largest_joint_change
        if self.recovery_started_at is None:
            self.recovery_started_at = time.monotonic()
        elapsed_sec = time.monotonic() - self.recovery_started_at
        if elapsed_sec >= self.working_start_timeout_sec:
            raise RecoveryTimeoutError(
                "작업자세 복구 시간 초과: "
                f"{elapsed_sec:.1f}초 경과, 최대 관절 오차 "
                f"{largest_joint_change:.2f}° "
                f"(허용 {self.working_start_arrival_tolerance_deg:.2f}°)."
            )

        if largest_joint_change <= self.working_start_arrival_tolerance_deg:
            self.recovery_stable_sample_count += 1
            if (
                self.recovery_stable_sample_count
                >= self.working_start_stable_samples
            ):
                self.recovery_active = False
                self.recovery_started_at = None
                return None
            return None
        else:
            self.recovery_stable_sample_count = 0
        ratio = min(1.0, self.limits.max_joint_step_deg / largest_joint_change)
        target = current.interpolate(self.working_command, ratio)
        self._validate_recovery_step(current, target, calibration_ranges)
        return target

    def plan(
        self,
        current: JointCommand,
        user_x_m: float,
        calibration_ranges: dict[str, tuple[float, float]] | None = None,
    ) -> JointCommand | None:
        if self.reference_z_m is None:
            self.set_vertical_reference(current)

        if self.recovery_active:
            return self._plan_working_pose_recovery(current, calibration_ranges)

        current_pose = self.kinematics.forward(current)
        requested_pose = monitor_target_from_user(
            user_x_m=float(user_x_m),
            user_monitor_distance_m=self.desired_distance_m,
            monitor_z_m=self.reference_z_m,
        )
        monitor_x_error = requested_pose.x_m - current_pose.x_m
        if abs(monitor_x_error) <= self.deadband_m:
            return None

        x_step = self._clamp(
            monitor_x_error,
            -self.max_x_step_m,
            self.max_x_step_m,
        )
        full_target = self.kinematics.inverse(
            current_pose.x_m + x_step,
            self.reference_z_m,
        )

        target = full_target
        if self.joint_command_mode == "stepped":
            largest_joint_change = max(
                abs(full_target.shoulder_lift_deg - current.shoulder_lift_deg),
                abs(full_target.elbow_flex_deg - current.elbow_flex_deg),
            )
            if largest_joint_change > self.limits.max_joint_step_deg:
                ratio = self.limits.max_joint_step_deg / largest_joint_change
                target = current.interpolate(full_target, ratio)

        self.kinematics.validate_motion(
            current=current,
            target=target,
            reference_z_m=self.reference_z_m,
            limits=self.limits,
            calibration_ranges=calibration_ranges,
            enforce_step_limit=self.joint_command_mode == "stepped",
        )
        return target
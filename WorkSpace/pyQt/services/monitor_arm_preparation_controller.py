"""Hardware-process controller for the PyQt monitor-arm preparation dialog."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

from services.monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    SafetyLimits,
    TwoJointMonitorArm,
    load_settings,
    monitor_target_from_user,
)
from services.monitor_arm_speed import select_speed


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_SETTINGS_PATH = (
    WORKSPACE_DIR / "config" / "monitor_arm_settings.json"
)
MOTOR12_JOINTS = ("shoulder_lift", "elbow_flex")
GIMBAL_JOINTS = ("wrist_flex", "wrist_roll")


class MonitorArmPreparationController:
    """Own setup-only Motor1~4 commands while MotorService owns the serial bus."""

    def __init__(self, motor_service, motor12, motor34, settings_path=DEFAULT_SETTINGS_PATH):
        self.motor = motor_service
        self.motor12 = motor12
        self.motor34 = motor34
        self.settings_path = Path(settings_path)
        self.settings = load_settings(self.settings_path)
        self.kinematics = TwoJointMonitorArm(
            ArmGeometry.from_settings(self.settings)
        )
        self.limits = SafetyLimits.from_settings(self.settings)
        self.active = False
        self.connected = False
        self.working_start_completed = False
        self.current: JointCommand | None = None
        self.current_pose = None
        self.target: JointCommand | None = None
        self.target_pose = None
        self.target_reason: str | None = None
        self.recovery_active = False
        self.last_command_at = 0.0
        self.last_telemetry_at = 0.0
        self.last_error: str | None = None
        self.movement_status = "idle"
        self.movement_started_at: float | None = None
        self.arrival_stable_sample_count = 0
        self.working_start_max_error_deg: float | None = None
        self.motor12_hold_reason: str | None = None
        self.motor12_last_error: str | None = None
        self._normal_working_command = (
            self.motor12.planner.working_command
            if getattr(self.motor12, "planner", None) is not None
            else None
        )
        self.motor_angles: dict[str, float | None] = {
            joint: None for joint in MOTOR12_JOINTS + GIMBAL_JOINTS
        }
        # Jog 중에는 실제 모터가 아직 목표를 따라오는 중이어도 누적 목표가
        # 매 tick 전진해야 한다. 현재각을 매번 다시 기준으로 삼으면 작은 + 입력이
        # 같은 목표로 반복되어 한쪽 방향이 멈춘 것처럼 보일 수 있다.
        self.gimbal_jog_targets: dict[str, float | None] = {
            joint: None for joint in GIMBAL_JOINTS
        }

        control = self.settings.get("control", {})
        self.command_hz = max(1.0, float(control.get("command_hz", 20.0)))
        self.command_interval = 1.0 / self.command_hz
        self.maximum_speed = min(
            1000,
            max(1, int(control.get("vertical_ik_speed", 800))),
        )
        self.minimum_speed = min(
            self.maximum_speed,
            max(1, int(control.get("vertical_ik_variable_min_speed", 150))),
        )
        self.full_speed_error_deg = max(
            1.0,
            float(control.get("vertical_ik_variable_full_speed_error_deg", 30.0)),
        )
        self.speed_mode = str(control.get("vertical_ik_speed_mode", "adaptive"))
        self.acc = min(254, max(0, int(control.get("vertical_ik_acc", 30))))
        planner = getattr(self.motor12, "planner", None)
        self.working_start_arrival_tolerance_deg = (
            planner.working_start_arrival_tolerance_deg if planner else 1.0
        )
        self.working_start_stable_samples = (
            planner.working_start_stable_samples if planner else 3
        )
        self.working_start_timeout_sec = (
            planner.working_start_timeout_sec if planner else 25.0
        )
        # Smooth working/manual IK moves are sent as one synchronized Servo goal,
        # so the Planner recovery counter is not active for these movements.
        # Use the same proven arrival criteria locally instead of the old 0.5°
        # one-sample check, which could leave the preparation UI moving forever.
        self.arrival_tolerance_deg = max(
            0.1,
            float(
                control.get(
                    "preparation_arrival_tolerance_deg",
                    self.working_start_arrival_tolerance_deg,
                )
            ),
        )
        self.arrival_stable_samples = max(
            1,
            int(
                control.get(
                    "preparation_arrival_stable_samples",
                    self.working_start_stable_samples,
                )
            ),
        )
        self.movement_timeout_sec = max(
            0.1,
            float(
                control.get(
                    "preparation_movement_timeout_sec",
                    self.working_start_timeout_sec,
                )
            ),
        )

    def begin(self) -> None:
        self.active = True
        self.last_error = None
        self.movement_status = "idle"
        self.movement_started_at = None
        self.arrival_stable_sample_count = 0
        self.working_start_max_error_deg = None
        self.motor12_hold_reason = None
        self.motor12_last_error = None

    def end(self) -> None:
        self.active = False
        self.recovery_active = False
        self.target_reason = None
        self.movement_started_at = None
        self.arrival_stable_sample_count = 0
        if (
            self._normal_working_command is not None
            and getattr(self.motor12, "planner", None) is not None
        ):
            self.motor12.planner.cancel_working_pose_recovery()
            self.motor12.planner.working_command = self._normal_working_command

    def connect_all(self) -> dict[str, Any]:
        if not self.motor.available and not self.motor.open():
            raise RuntimeError(self.motor.last_error or "모터 포트 연결 실패")
        # 시작 때 얻은 ready 값만 재사용하지 않고 버튼/프로필 적용 시점의
        # Servo1~4를 다시 Ping해 실제 연결 상태를 확인한다.
        motor12_ok = self.motor12.initialize()
        motor34_ok = self.motor34.initialize()
        if not motor12_ok or not motor34_ok:
            errors = []
            if not motor12_ok:
                errors.append(f"Motor1/2: {self.motor12.last_error}")
            if not motor34_ok:
                errors.append(f"Motor3/4: {self.motor34.last_error}")
            raise RuntimeError(" / ".join(errors))
        self.connected = True
        self.active = True
        self.refresh_telemetry(force=True)
        for joint in GIMBAL_JOINTS:
            self.gimbal_jog_targets[joint] = self.motor_angles[joint]
        return self.snapshot()

    def _read_motor12(self) -> JointCommand:
        shoulder = self.motor.get_joint_angle("shoulder_lift")
        elbow = self.motor.get_joint_angle("elbow_flex")
        if shoulder is None or elbow is None:
            raise RuntimeError("Servo1/2 현재 각도를 읽지 못했습니다.")
        return JointCommand(float(shoulder), float(elbow))

    def refresh_telemetry(self, force=False) -> None:
        if not self.connected:
            return
        now = time.monotonic()
        if not force and now - self.last_telemetry_at < 0.20:
            return
        self.last_telemetry_at = now
        try:
            self.current = self._read_motor12()
            self.current_pose = self.kinematics.forward(self.current)
            for joint in GIMBAL_JOINTS:
                value = self.motor.get_joint_angle(joint)
                self.motor_angles[joint] = None if value is None else float(value)
            self.motor_angles["shoulder_lift"] = self.current.shoulder_lift_deg
            self.motor_angles["elbow_flex"] = self.current.elbow_flex_deg

            if self.target is not None:
                error = self._largest_delta(self.current, self.target)
                self.working_start_max_error_deg = error
                if (
                    not self.recovery_active
                    and self.target_reason in {"manual_ik", "rest", "working_start"}
                ):
                    movement_reason = self.target_reason
                    if error <= self.arrival_tolerance_deg:
                        self.arrival_stable_sample_count += 1
                        if (
                            self.arrival_stable_sample_count
                            >= self.arrival_stable_samples
                        ):
                            if movement_reason == "working_start":
                                self.working_start_completed = True
                            self.target_reason = None
                            self.movement_started_at = None
                            self.movement_status = "completed"
                        else:
                            self.movement_status = "stabilizing"
                    else:
                        self.arrival_stable_sample_count = 0
                        self.movement_status = "moving"

                    if (
                        self.target_reason is not None
                        and self.movement_started_at is not None
                        and now - self.movement_started_at
                        >= self.movement_timeout_sec
                    ):
                        if movement_reason == "working_start":
                            self.working_start_completed = False
                        self.last_error = (
                            f"{self.movement_timeout_sec:.1f}초 내 목표에 "
                            f"도착하지 못했습니다. 현재 최대 관절 오차 "
                            f"{error:.2f}° (허용 {self.arrival_tolerance_deg:.2f}°)."
                        )
                        self.target_reason = None
                        self.movement_started_at = None
                        self.arrival_stable_sample_count = 0
                        self.movement_status = "timeout"
                        if (
                            movement_reason == "working_start"
                            and self._normal_working_command is not None
                            and getattr(self.motor12, "planner", None) is not None
                        ):
                            self.motor12.planner.working_command = (
                                self._normal_working_command
                            )
            if self.movement_status == "telemetry_error":
                self.movement_status = (
                    "moving"
                    if self.recovery_active or self.target_reason is not None
                    else "idle"
                )
            if self.movement_status not in {
                "timeout",
                "safety_error",
                "command_error",
                "telemetry_error",
            }:
                self.last_error = None
        except Exception as error:
            self.last_error = str(error)
            self.movement_status = "telemetry_error"

    def request_working_start(self) -> JointCommand:
        self._require_connected()
        cartesian = self.settings.get("manual_cartesian", {})
        distance = float(
            self.settings["distance"]["desired_user_monitor_distance_m"]
        )
        # +X가 사용자 방향이므로 user_x 최소값에서 고정거리만큼 뺀 위치가
        # 준비 단계의 '사용자에게서 가장 먼' 작업 시작점이다.
        user_x_m = float(cartesian.get("user_x_min_m", 0.6007655))
        monitor_z_m = float(cartesian.get("default_monitor_z_m", 0.256))
        requested_pose = monitor_target_from_user(
            user_x_m=user_x_m,
            user_monitor_distance_m=distance,
            monitor_z_m=monitor_z_m,
        )
        target = self.kinematics.inverse(requested_pose.x_m, requested_pose.z_m)
        self._validate_final_target(target)
        self.motor12.planner.working_command = target
        result = self.motor12.move_to_working_smooth(target)
        if not result.get("accepted", False):
            if self._normal_working_command is not None:
                self.motor12.planner.working_command = self._normal_working_command
            raise RuntimeError(result.get("error", "작업자세 복구 시작 실패"))
        self.target = target
        self.target_pose = self.kinematics.forward(target)
        self.target_reason = "working_start"
        self.recovery_active = False
        self.working_start_completed = False
        self.movement_started_at = time.monotonic()
        self.arrival_stable_sample_count = 0
        self.last_error = None
        self.movement_status = "moving"
        self.working_start_max_error_deg = None
        self.motor12_hold_reason = "WORKING_SMOOTH"
        self.motor12_last_error = None
        return target

    def request_rest(self) -> dict[str, Any]:
        """현재 작업 자세에서 확인된 Motor1/2 휴식 자세로 이동한다."""
        self._require_connected()
        result = self.motor12.move_to_rest()
        if not result.get("accepted", False):
            raise RuntimeError(result.get("error", "휴식자세 이동 실패"))
        self.working_start_completed = False
        self.recovery_active = False
        self.target = self.motor12.rest_command
        self.target_pose = self.kinematics.forward(self.target)
        self.target_reason = "rest"
        self.movement_started_at = time.monotonic()
        self.arrival_stable_sample_count = 0
        self.last_error = None
        self.movement_status = "moving"
        self.working_start_max_error_deg = None
        return result

    def command_manual_ik(
        self,
        user_x_m: float,
        user_monitor_distance_m: float,
        monitor_z_m: float,
    ) -> JointCommand:
        self._require_connected()
        if not self.working_start_completed:
            raise RuntimeError("먼저 '휴식 → 작업 시작 위치 이동'을 완료해주세요.")
        result = self.motor12.move_manual_user_target(
            user_x_m=float(user_x_m),
            user_monitor_distance_m=float(user_monitor_distance_m),
            monitor_z_m=float(monitor_z_m),
        )
        if not result.get("accepted", False):
            raise RuntimeError(result.get("error", "Servo1/2 IK 명령 실패"))
        target = JointCommand(
            float(result["target"]["shoulder_lift"]),
            float(result["target"]["elbow_flex"]),
        )
        self.target = target
        self.target_pose = self.kinematics.forward(target)
        self.target_reason = "manual_ik"
        self.movement_started_at = time.monotonic()
        self.arrival_stable_sample_count = 0
        self.last_error = None
        self.movement_status = "moving"
        self.working_start_max_error_deg = None
        return target

    def jog_gimbal(self, joint: str, delta_deg: float, speed: int = 100) -> float:
        self._require_connected()
        if joint not in GIMBAL_JOINTS:
            raise ValueError(f"Gimbal 조그 대상이 아닙니다: {joint}")
        current = self.motor.get_joint_angle(joint)
        if current is None:
            raise RuntimeError(f"{joint} 현재 각도를 읽지 못했습니다.")
        safe_range = self.motor.get_safe_angle_range(joint)
        if safe_range is None:
            raise RuntimeError(f"{joint} 안전각 범위를 읽지 못했습니다.")
        minimum, maximum = safe_range
        base = self.gimbal_jog_targets.get(joint)
        if base is None or abs(float(base) - float(current)) > 15.0:
            base = float(current)
        requested = float(base) + float(delta_deg)
        target = max(minimum, min(requested, maximum))
        if abs(target - float(base)) < 1e-6:
            direction = "+" if delta_deg > 0 else "-"
            raise RuntimeError(
                f"{joint} {direction} 조그가 안전한계에 도달했습니다 "
                f"({minimum:+.2f}~{maximum:+.2f}°)."
            )
        max_speed = self.motor.get_max_speed(joint) or 1000
        command_speed = max(1, min(int(speed), int(max_speed), 1000))
        if not self.motor.move_joint(
            joint,
            angle=target,
            speed=command_speed,
            acc=10,
            wait=False,
        ):
            raise RuntimeError(self.motor.last_error or f"{joint} 조그 실패")
        self.gimbal_jog_targets[joint] = target
        self.motor_angles[joint] = target
        return target

    def stop_gimbal_jog(self, joint: str | None = None) -> None:
        """버튼을 놓은 시점의 실제각을 다음 조그의 새 기준으로 사용한다."""
        joints = GIMBAL_JOINTS if joint is None else (joint,)
        for name in joints:
            if name not in GIMBAL_JOINTS:
                continue
            current = self.motor.get_joint_angle(name)
            self.gimbal_jog_targets[name] = (
                None if current is None else float(current)
            )

    def record_movement_error(
        self,
        error: Exception | str,
        status: str = "command_error",
    ) -> None:
        """Persist a stopped movement error so UI polling cannot overwrite it."""
        self.last_error = str(error)
        self.movement_status = status
        self.recovery_active = False
        self.working_start_completed = False
        self.target_reason = None
        self.movement_started_at = None
        self.arrival_stable_sample_count = 0
        planner = getattr(self.motor12, "planner", None)
        if planner is not None and planner.recovery_active:
            planner.cancel_working_pose_recovery()
        if self._normal_working_command is not None and planner is not None:
            planner.working_command = self._normal_working_command

    def update(self, now: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if now is None else float(now)
        if self.connected:
            self.refresh_telemetry()

        planner = getattr(self.motor12, "planner", None)
        self.motor12_hold_reason = getattr(self.motor12, "hold_reason", None)
        live_motor12_error = getattr(self.motor12, "last_error", None)

        if self.active and self.connected and self.recovery_active:
            if self.current is not None and self.target is not None:
                self.working_start_max_error_deg = self._largest_delta(
                    self.current, self.target
                )

            failure_statuses = {
                "RECOVERY_TIMEOUT": "timeout",
                "RECOVERY_SAFETY_ERROR": "safety_error",
                "RECOVERY_COMMAND_ERROR": "command_error",
                "RECOVERY_ERROR": "command_error",
                "NOT_READY": "command_error",
                "DISABLED": "command_error",
                "SAFE_HOLD": "command_error",
            }
            failure_status = failure_statuses.get(self.motor12_hold_reason)
            if failure_status is not None:
                self.motor12_last_error = live_motor12_error
                self.record_movement_error(
                    live_motor12_error or self.motor12_hold_reason,
                    failure_status,
                )
            elif planner is None:
                self.record_movement_error(
                    "MonitorArmPlanner가 준비 중 사라졌습니다.",
                    "command_error",
                )
            elif not planner.recovery_active:
                if self.motor12_hold_reason == "RECOVERY_COMPLETE":
                    self.recovery_active = False
                    self.working_start_completed = True
                    self.target_reason = None
                    self.movement_status = "completed"
                    self.last_error = None
                    self.motor12_last_error = None
                    if self._normal_working_command is not None:
                        planner.working_command = self._normal_working_command
                else:
                    self.record_movement_error(
                        "작업자세 Recovery가 완료 상태 없이 중단되었습니다.",
                        "command_error",
                    )
            elif planner.recovery_stable_sample_count > 0:
                self.movement_status = "stabilizing"
            else:
                self.movement_status = "moving"

        if self.connected and self.motor12.rest_mode:
            self.working_start_completed = False
            self.recovery_active = False
        return self.snapshot()

    def _validate_final_target(self, target: JointCommand) -> None:
        ranges = self._motor12_ranges()
        soft = {
            "shoulder_lift": (
                self.limits.shoulder_min_deg,
                self.limits.shoulder_max_deg,
            ),
            "elbow_flex": (
                self.limits.elbow_min_deg,
                self.limits.elbow_max_deg,
            ),
        }
        for joint, angle in (
            ("shoulder_lift", target.shoulder_lift_deg),
            ("elbow_flex", target.elbow_flex_deg),
        ):
            minimum = max(ranges[joint][0], soft[joint][0])
            maximum = min(ranges[joint][1], soft[joint][1])
            if not minimum <= angle <= maximum:
                raise ValueError(
                    f"{joint} IK={angle:+.2f}°가 안전범위 "
                    f"{minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                )

    def _validate_recovery_step(self, current: JointCommand, target: JointCommand) -> None:
        ranges = self._motor12_ranges()
        for joint, current_angle, target_angle in (
            (
                "shoulder_lift",
                current.shoulder_lift_deg,
                target.shoulder_lift_deg,
            ),
            ("elbow_flex", current.elbow_flex_deg, target.elbow_flex_deg),
        ):
            minimum, maximum = ranges[joint]
            current_outside = self._outside_distance(current_angle, minimum, maximum)
            target_outside = self._outside_distance(target_angle, minimum, maximum)
            if current_outside <= 1e-6 and target_outside > 1e-6:
                raise ValueError(f"{joint} 복구 스텝이 안전범위 밖으로 나갑니다.")
            if current_outside > 1e-6 and target_outside >= current_outside - 1e-6:
                raise ValueError(f"{joint} 복구 스텝이 안전범위 안쪽으로 향하지 않습니다.")

        current_pose = self.kinematics.forward(current)
        target_pose = self.kinematics.forward(target)
        z_min = float(
            self.settings.get("manual_cartesian", {}).get("monitor_z_min_m", 0.20)
        )
        z_max = float(
            self.settings.get("manual_cartesian", {}).get("monitor_z_max_m", 0.30)
        )
        current_outside = self._outside_distance(current_pose.z_m, z_min, z_max)
        target_outside = self._outside_distance(target_pose.z_m, z_min, z_max)
        if current_outside <= 1e-6 and target_outside > self.limits.vertical_tolerance_m:
            raise ValueError("복구 스텝이 작업 Z 범위 밖으로 나갑니다.")
        if (
            current_outside > 1e-6
            and target_outside > current_outside + self.limits.vertical_tolerance_m
        ):
            raise ValueError("복구 스텝의 Z가 작업범위에서 더 멀어집니다.")

    def _motor12_ranges(self) -> dict[str, tuple[float, float]]:
        return {
            joint: tuple(self.motor.get_safe_angle_range(joint))
            for joint in MOTOR12_JOINTS
        }

    def _speed_for_error(self, error_deg: float) -> int:
        return select_speed(
            self.speed_mode,
            self.maximum_speed,
            self.minimum_speed,
            self.full_speed_error_deg,
            error_deg,
        )

    def _require_connected(self) -> None:
        if not self.connected or not self.motor12.ready or not self.motor34.ready:
            raise RuntimeError("먼저 모터 1~4 연결 확인을 완료해주세요.")

    @staticmethod
    def _largest_delta(current: JointCommand, target: JointCommand) -> float:
        return max(
            abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(target.elbow_flex_deg - current.elbow_flex_deg),
        )

    @staticmethod
    def _outside_distance(value: float, minimum: float, maximum: float) -> float:
        if value < minimum:
            return minimum - value
        if value > maximum:
            return value - maximum
        return 0.0

    def calibration_metadata(self) -> dict[str, Any]:
        self.refresh_telemetry(force=True)
        if self.current_pose is None:
            raise RuntimeError("현재 모니터 자세를 읽지 못했습니다.")
        return {
            "monitor_x_m": float(self.current_pose.x_m),
            "monitor_z_m": float(self.current_pose.z_m),
            "motor_angles_deg": {
                joint: value for joint, value in self.motor_angles.items()
            },
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "active": bool(self.active),
            "connected": bool(self.connected),
            "all_motors_ready": bool(
                self.connected and self.motor12.ready and self.motor34.ready
            ),
            "working_start_completed": bool(self.working_start_completed),
            "recovery_active": bool(self.recovery_active),
            "movement_active": bool(
                self.recovery_active
                or (self.target is not None and self.target_reason is not None)
            ),
            "target_reason": self.target_reason,
            "movement_status": self.movement_status,
            "current_target_max_error_deg": self.working_start_max_error_deg,
            "arrival_tolerance_deg": self.arrival_tolerance_deg,
            "stable_samples": (
                0
                if getattr(self.motor12, "planner", None) is None
                else (
                    self.motor12.planner.recovery_stable_sample_count
                    if self.recovery_active
                    else self.arrival_stable_sample_count
                )
            ),
            "required_stable_samples": self.arrival_stable_samples,
            "timeout_sec": self.movement_timeout_sec,
            "motor12_hold_reason": self.motor12_hold_reason,
            "motor12_last_error": self.motor12_last_error,
            "current_angles_deg": {
                "shoulder_lift": (
                    self.current.shoulder_lift_deg if self.current else None
                ),
                "elbow_flex": self.current.elbow_flex_deg if self.current else None,
            },
            "motor_angles_deg": dict(self.motor_angles),
            "current_pose": {
                "x_m": self.current_pose.x_m if self.current_pose else None,
                "z_m": self.current_pose.z_m if self.current_pose else None,
            },
            "target_angles_deg": {
                "shoulder_lift": self.target.shoulder_lift_deg if self.target else None,
                "elbow_flex": self.target.elbow_flex_deg if self.target else None,
            },
            "target_pose": {
                "x_m": self.target_pose.x_m if self.target_pose else None,
                "z_m": self.target_pose.z_m if self.target_pose else None,
            },
            "settings_path": str(self.settings_path),
            "last_error": self.last_error,
            "timestamp": time.time(),
        }

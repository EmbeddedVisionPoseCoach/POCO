"""Initial-preparation sensor averaging and runtime distance-fusion helpers.

MediaPipe still runs only in the pose process and physical I2C remains owned by
the hardware process.  During the explicit preparation capture, the hardware
process combines the latest unique eye frames with unique ToF samples and
persists one baseline record.
"""

from __future__ import annotations

import math
import json
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from typing import Any


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_PREPARATION_CALIBRATION_PATH = (
    WORKSPACE_DIR / "data" / "settings" / "monitor_arm_user_calibration.json"
)


@dataclass(frozen=True)
class MonitorArmDistanceEstimate:
    """One runtime fusion result in the base X coordinate system."""

    tof_user_x_m: float
    vision_user_x_m: float
    fused_user_x_m: float
    vision_user_monitor_distance_m: float


class MonitorArmPreparationCalibrationService:
    """Collect and persist ToF/eye averages during the preparation dialog."""

    def __init__(
        self,
        path: Path = DEFAULT_PREPARATION_CALIBRATION_PATH,
        duration_sec: float = 5.0,
        minimum_tof_samples: int = 60,
        minimum_eye_samples: int = 60,
    ):
        self.path = Path(path)
        self.duration_sec = max(1.0, float(duration_sec))
        self.minimum_tof_samples = max(1, int(minimum_tof_samples))
        self.minimum_eye_samples = max(1, int(minimum_eye_samples))
        self.running = False
        self.session_ready = False
        self.started_at: float | None = None
        self.tof_samples_m: list[float] = []
        self.eye_gap_samples_px: list[float] = []
        self._last_tof_timestamp: float | None = None
        self._last_pose_frame_id: int | None = None
        self.record = self._empty_record()

    def _empty_record(self) -> dict[str, Any]:
        return {
            "ready": False,
            "session_ready": False,
            "running": False,
            "duration_sec": self.duration_sec,
            "remain_sec": self.duration_sec,
            "tof_user_x_baseline_m": None,
            "tof_range_baseline_m": None,
            "eye_gap_baseline_px": None,
            "tof_sample_count": 0,
            "eye_sample_count": 0,
            "monitor_x_baseline_m": None,
            "monitor_z_baseline_m": None,
            "user_monitor_distance_baseline_m": None,
            "motor_angles_deg": {},
            "calibrated_at": None,
            "path": str(self.path),
            "last_error": None,
        }

    def start(self) -> dict[str, Any]:
        self.running = True
        self.session_ready = False
        self.started_at = time.monotonic()
        self.tof_samples_m.clear()
        self.eye_gap_samples_px.clear()
        self._last_tof_timestamp = None
        self._last_pose_frame_id = None
        self.record = self._empty_record()
        self.record["running"] = True
        return self.snapshot()

    def cancel(self) -> dict[str, Any]:
        self.running = False
        self.started_at = None
        self.tof_samples_m.clear()
        self.eye_gap_samples_px.clear()
        self._last_tof_timestamp = None
        self._last_pose_frame_id = None
        self.record = self._empty_record()
        return self.snapshot()

    def add_tof_state(self, tof_state: dict[str, Any] | None) -> bool:
        if not self.running or not isinstance(tof_state, dict):
            return False
        if not tof_state.get("valid", False):
            return False
        try:
            value = float(
                tof_state.get("filtered_distance_m", tof_state.get("distance_m"))
            )
            timestamp = float(tof_state.get("timestamp"))
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value) or value <= 0.0 or not math.isfinite(timestamp):
            return False
        if self._last_tof_timestamp is not None and timestamp <= self._last_tof_timestamp:
            return False
        self._last_tof_timestamp = timestamp
        self.tof_samples_m.append(value)
        return True

    def add_pose_state(self, pose_state: dict[str, Any] | None) -> bool:
        if not self.running or not isinstance(pose_state, dict):
            return False
        if not pose_state.get("eye_gap_valid", False):
            return False
        try:
            frame_id = int(pose_state.get("frame_id"))
            eye_gap = float(pose_state.get("eye_gap_px"))
        except (TypeError, ValueError):
            return False
        if not math.isfinite(eye_gap) or eye_gap <= 0.0:
            return False
        if self._last_pose_frame_id is not None and frame_id <= self._last_pose_frame_id:
            return False
        self._last_pose_frame_id = frame_id
        self.eye_gap_samples_px.append(eye_gap)
        return True

    @property
    def elapsed_sec(self) -> float:
        if not self.running or self.started_at is None:
            return 0.0
        return max(0.0, time.monotonic() - self.started_at)

    @property
    def finished_by_time(self) -> bool:
        return bool(self.running and self.elapsed_sec >= self.duration_sec)

    def finish(self, pose_metadata: dict[str, Any]) -> dict[str, Any]:
        self.running = False
        tof_count = len(self.tof_samples_m)
        eye_count = len(self.eye_gap_samples_px)
        tof_avg = fmean(self.tof_samples_m) if self.tof_samples_m else None
        eye_avg = fmean(self.eye_gap_samples_px) if self.eye_gap_samples_px else None
        success = bool(
            tof_avg is not None
            and eye_avg is not None
            and tof_count >= self.minimum_tof_samples
            and eye_count >= self.minimum_eye_samples
        )
        errors = []
        if tof_count < self.minimum_tof_samples:
            errors.append(f"ToF 샘플 부족({tof_count}/{self.minimum_tof_samples})")
        if eye_count < self.minimum_eye_samples:
            errors.append(f"눈 간격 샘플 부족({eye_count}/{self.minimum_eye_samples})")

        metadata = dict(pose_metadata or {})
        monitor_x_m = metadata.get("monitor_x_m")
        user_monitor_distance_m = (
            float(tof_avg) - float(monitor_x_m)
            if success and monitor_x_m is not None
            else None
        )
        if success and (
            user_monitor_distance_m is None or user_monitor_distance_m <= 0.0
        ):
            success = False
            self.session_ready = False
            errors.append("ToF 사용자 X가 현재 모니터 X보다 가깝습니다.")
        self.session_ready = success
        self.record = {
            "version": 1,
            "ready": success,
            "session_ready": success,
            "running": False,
            "duration_sec": self.duration_sec,
            "remain_sec": 0.0,
            "tof_user_x_baseline_m": tof_avg if success else None,
            "tof_range_baseline_m": tof_avg if success else None,
            "eye_gap_baseline_px": eye_avg if success else None,
            "tof_sample_count": tof_count,
            "eye_sample_count": eye_count,
            "monitor_x_baseline_m": monitor_x_m,
            "monitor_z_baseline_m": metadata.get("monitor_z_m"),
            "user_monitor_distance_baseline_m": (
                user_monitor_distance_m if success else None
            ),
            "motor_angles_deg": dict(metadata.get("motor_angles_deg") or {}),
            "calibrated_at": time.time() if success else None,
            "path": str(self.path),
            "last_error": " / ".join(errors) if errors else None,
        }
        if success:
            self._save_record(self.record)
        return self.snapshot()

    def _save_record(self, record: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.",
            suffix=".tmp",
            dir=str(self.path.parent),
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as file:
                json.dump(record, file, indent=2, ensure_ascii=False)
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary_name, self.path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass
            raise

    def snapshot(self) -> dict[str, Any]:
        state = dict(self.record)
        state["running"] = self.running
        state["session_ready"] = self.session_ready
        state["tof_sample_count"] = len(self.tof_samples_m) if self.running else state.get(
            "tof_sample_count", 0
        )
        state["eye_sample_count"] = len(self.eye_gap_samples_px) if self.running else state.get(
            "eye_sample_count", 0
        )
        state["remain_sec"] = (
            max(0.0, self.duration_sec - self.elapsed_sec)
            if self.running
            else float(state.get("remain_sec", 0.0))
        )
        return state


class CalibratedMonitorArmFusion:
    """Use the 5-second averages for ToF 0.7 + vision 0.3 user-X fusion.

    The eye gap measures the camera-to-user distance, not base-to-user X.
    Therefore the monitor X at calibration time is supplied when the motor-arm
    controller activates this baseline.
    """

    def __init__(self, tof_weight: float = 0.7, vision_weight: float = 0.3):
        tof_weight = float(tof_weight)
        vision_weight = float(vision_weight)
        total = tof_weight + vision_weight
        if tof_weight < 0.0 or vision_weight < 0.0 or total <= 0.0:
            raise ValueError("센서 융합 가중치는 0 이상이고 합은 0보다 커야 합니다.")
        self.tof_weight = tof_weight / total
        self.vision_weight = vision_weight / total
        self.tof_user_x_baseline_m: float | None = None
        self.eye_gap_baseline_px: float | None = None
        self.monitor_x_baseline_m: float | None = None

    @property
    def calibrated(self) -> bool:
        return all(
            value is not None
            for value in (
                self.tof_user_x_baseline_m,
                self.eye_gap_baseline_px,
                self.monitor_x_baseline_m,
            )
        )

    def apply_calibration(
        self,
        calibration: dict[str, Any],
        monitor_x_baseline_m: float,
    ) -> None:
        if not isinstance(calibration, dict) or not calibration.get("ready", False):
            raise ValueError("현재 세션의 모니터암 보정값이 준비되지 않았습니다.")
        tof_x = float(calibration["tof_user_x_baseline_m"])
        eye_gap = float(calibration["eye_gap_baseline_px"])
        monitor_x = float(monitor_x_baseline_m)
        reference_distance = tof_x - monitor_x
        if not all(math.isfinite(v) for v in (tof_x, eye_gap, monitor_x)):
            raise ValueError("모니터암 보정값에 NaN 또는 Inf가 있습니다.")
        if eye_gap <= 0.0 or reference_distance <= 0.0:
            raise ValueError("눈 간격 또는 기준 사용자-모니터 거리가 유효하지 않습니다.")
        self.tof_user_x_baseline_m = tof_x
        self.eye_gap_baseline_px = eye_gap
        self.monitor_x_baseline_m = monitor_x

    def estimate(
        self,
        tof_user_x_m: float,
        eye_gap_px: float,
        current_monitor_x_m: float,
    ) -> MonitorArmDistanceEstimate:
        if not self.calibrated:
            raise RuntimeError("5초 모니터암 보정을 먼저 완료해야 합니다.")
        tof_x = float(tof_user_x_m)
        gap = float(eye_gap_px)
        monitor_x = float(current_monitor_x_m)
        if not all(math.isfinite(v) for v in (tof_x, gap, monitor_x)) or gap <= 0.0:
            raise ValueError("실시간 ToF/눈 간격/모니터 X 값이 유효하지 않습니다.")

        reference_distance = (
            self.tof_user_x_baseline_m - self.monitor_x_baseline_m
        )
        vision_distance = reference_distance * self.eye_gap_baseline_px / gap
        vision_user_x = monitor_x + vision_distance
        fused_user_x = (
            self.tof_weight * tof_x + self.vision_weight * vision_user_x
        )
        return MonitorArmDistanceEstimate(
            tof_user_x_m=tof_x,
            vision_user_x_m=vision_user_x,
            fused_user_x_m=fused_user_x,
            vision_user_monitor_distance_m=vision_distance,
        )

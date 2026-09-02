"""Standalone POCO Motor1/2 monitor-arm performance evaluator.

The tool reuses the production ToF, MotorService, Motor12Controller,
MonitorArmPlanner, kinematics, and safety supervisor.  It does not import or
start mainpyQt.py.  Opening the application never moves a motor: connection,
working-pose movement, tracking, rest movement, and safety trials all require
separate confirmed button actions.

Scope
-----
* Static user-to-monitor distance MAE (external ruler value entered by user)
* Dynamic settling time and intermediate stop count
* Repetition precision from servo telemetry + production forward kinematics
* Safety scenario pass rate

Motor3/4 IMU gimbal control is intentionally excluded so the Motor1/2 distance
tracking performance can be measured independently.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


PYQT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = PYQT_DIR.parent
SETTINGS_PATH = WORKSPACE_DIR / "config" / "monitor_arm_settings.json"
CURRENT_CALIBRATION_PATH = (
    WORKSPACE_DIR / "data" / "settings" / "monitor_arm_user_calibration.json"
)
USER_PROFILES_DIR = WORKSPACE_DIR / "data" / "user_profiles"
if str(PYQT_DIR) not in sys.path:
    sys.path.insert(0, str(PYQT_DIR))
if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))

from services.monitor_arm_kinematics import JointCommand
from services.monitor_arm_safety_supervisor import MonitorArmSafetySupervisor
from services.monitor_arm_user_x import ToFUserXSource
from services.motor12_controller import Motor12Controller
from services.motor_service import MotorService
from services.tof_service import create_tof_service


def _read_working_angles(calibration_path: Path) -> dict[str, float] | None:
    try:
        payload = json.loads(calibration_path.read_text(encoding="utf-8"))
        if not bool(payload.get("ready")):
            return None
        values = dict(payload.get("motor_angles_deg") or {})
        shoulder = float(values["shoulder_lift"])
        elbow = float(values["elbow_flex"])
        if not math.isfinite(shoulder) or not math.isfinite(elbow):
            return None
        return {
            "shoulder_lift": shoulder,
            "elbow_flex": elbow,
        }
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


def load_working_pose_options() -> list[dict[str, Any]]:
    """Load the current calibration and all valid saved profile work poses."""
    options: list[dict[str, Any]] = []
    current = _read_working_angles(CURRENT_CALIBRATION_PATH)
    if current is not None:
        options.append(
            {
                "label": (
                    "현재 보정값  "
                    f"M1 {current['shoulder_lift']:.1f}° / "
                    f"M2 {current['elbow_flex']:.1f}°"
                ),
                "source": "현재 보정값",
                "angles": current,
                "path": str(CURRENT_CALIBRATION_PATH),
            }
        )

    for slot in range(1, 5):
        slot_dir = USER_PROFILES_DIR / f"slot_{slot}"
        angles = _read_working_angles(slot_dir / "monitor_arm_calibration.json")
        if angles is None:
            continue
        name = f"프로필 {slot}"
        try:
            metadata = json.loads(
                (slot_dir / "profile.json").read_text(encoding="utf-8")
            )
            profile_name = str(metadata.get("name") or "").strip()
            if profile_name:
                name = f"프로필 {slot} · {profile_name}"
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        options.append(
            {
                "label": (
                    f"{name}  M1 {angles['shoulder_lift']:.1f}° / "
                    f"M2 {angles['elbow_flex']:.1f}°"
                ),
                "source": name,
                "angles": angles,
                "path": str(slot_dir / "monitor_arm_calibration.json"),
            }
        )
    return options


class InstrumentedMotor12Controller(Motor12Controller):
    """Production controller with a non-invasive actual-angle telemetry cache.

    Normal tracking already reads both servo angles at the 20 Hz command gate.
    Reusing that sample for evaluation avoids adding another pair of serial reads
    at 10 Hz.  During one-shot working/rest moves, where ``update()`` does not read
    angles, ``latest_arm_state()`` falls back to a real hardware read.
    """

    def __init__(self, *args, **kwargs):
        self._evaluation_angles: JointCommand | None = None
        self._evaluation_angles_at = 0.0
        super().__init__(*args, **kwargs)

    def _read_current_angles(self):
        angles = super()._read_current_angles()
        self._evaluation_angles = angles
        self._evaluation_angles_at = time.monotonic()
        return angles

    def latest_arm_state(self, max_cache_age_sec: float = 0.12):
        now = time.monotonic()
        if (
            self._evaluation_angles is None
            or now - self._evaluation_angles_at > float(max_cache_age_sec)
        ):
            angles = self._read_current_angles()
        else:
            angles = self._evaluation_angles
        pose = self.planner.kinematics.forward(angles)
        return angles, pose


TELEMETRY_FIELDS = [
    "timestamp",
    "elapsed_sec",
    "tracking_enabled",
    "tof_available",
    "tof_valid",
    "tof_filtered_distance_m",
    "raw_user_x_m",
    "control_user_x_m",
    "control_saturated",
    "shoulder_lift_deg",
    "elbow_flex_deg",
    "monitor_x_m",
    "monitor_z_m",
    "estimated_user_monitor_distance_m",
    "distance_error_m",
    "motor12_hold_reason",
    "motor12_command_speed",
    "safety_state",
    "safety_reason",
    "tracking_allowed",
    "simulated_landmark_valid",
    "simulated_posture",
    "dynamic_trial_id",
    "safety_trial_id",
]

STATIC_FIELDS = [
    "trial_id",
    "timestamp",
    "nominal_user_x_m",
    "approach_direction",
    "external_actual_distance_m",
    "target_distance_m",
    "absolute_error_m",
    "tof_raw_user_x_m",
    "tof_filtered_distance_m",
    "estimated_internal_distance_m",
    "shoulder_lift_deg",
    "elbow_flex_deg",
    "monitor_x_m",
    "monitor_z_m",
]

DYNAMIC_FIELDS = [
    "trial_id",
    "started_at",
    "ended_at",
    "success",
    "initial_user_x_m",
    "final_user_x_m",
    "user_step_m",
    "settling_time_sec",
    "intermediate_stop_count",
    "final_distance_error_m",
    "timeout_sec",
    "message",
]

SAFETY_FIELDS = [
    "trial_id",
    "scenario",
    "started_at",
    "ended_at",
    "success",
    "hold_latency_sec",
    "action_latency_sec",
    "expected_action",
    "message",
]


def _safe_div(numerator: float, denominator: float) -> float:
    if denominator <= 0:
        return 0.0
    return float(numerator) / float(denominator)


def _round_or_none(value: Any, digits: int = 4):
    if value is None:
        return None
    return round(float(value), digits)


def calculate_arm_metrics(
    static_trials: list[dict[str, Any]],
    dynamic_trials: list[dict[str, Any]],
    safety_trials: list[dict[str, Any]],
) -> dict[str, Any]:
    static_errors = [
        abs(float(row["external_actual_distance_m"]) - float(row["target_distance_m"]))
        for row in static_trials
        if row.get("external_actual_distance_m") is not None
    ]
    repeatability: dict[str, dict[str, Any]] = {}
    nominal_values = sorted(
        {
            round(float(row["nominal_user_x_m"]), 3)
            for row in static_trials
            if row.get("nominal_user_x_m") is not None
            and row.get("monitor_x_m") is not None
        }
    )
    for nominal in nominal_values:
        positions = [
            float(row["monitor_x_m"])
            for row in static_trials
            if row.get("monitor_x_m") is not None
            and abs(float(row["nominal_user_x_m"]) - nominal) < 0.0005
        ]
        repeatability[f"{nominal:.3f}m"] = {
            "sample_count": len(positions),
            "mean_monitor_x_m": _round_or_none(np.mean(positions), 5)
            if positions
            else None,
            "std_monitor_x_m": _round_or_none(np.std(positions, ddof=1), 5)
            if len(positions) >= 2
            else None,
            "range_monitor_x_m": _round_or_none(max(positions) - min(positions), 5)
            if positions
            else None,
        }

    completed_dynamic = [row for row in dynamic_trials if bool(row.get("success"))]
    settling_times = [
        float(row["settling_time_sec"])
        for row in completed_dynamic
        if row.get("settling_time_sec") is not None
    ]
    stop_counts = [
        int(row.get("intermediate_stop_count", 0)) for row in dynamic_trials
    ]
    safety_passes = sum(bool(row.get("success")) for row in safety_trials)

    return {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "static_distance": {
            "sample_count": len(static_errors),
            "target_distance_m": (
                _round_or_none(static_trials[0].get("target_distance_m"), 4)
                if static_trials
                else None
            ),
            "mae_m": _round_or_none(np.mean(static_errors), 5)
            if static_errors
            else None,
            "mae_mm": _round_or_none(np.mean(static_errors) * 1000.0, 2)
            if static_errors
            else None,
            "rmse_m": _round_or_none(
                math.sqrt(float(np.mean(np.square(static_errors)))), 5
            )
            if static_errors
            else None,
            "max_absolute_error_m": _round_or_none(max(static_errors), 5)
            if static_errors
            else None,
        },
        "repeatability": repeatability,
        "dynamic": {
            "trial_count": len(dynamic_trials),
            "success_count": len(completed_dynamic),
            "success_rate": round(
                _safe_div(len(completed_dynamic), len(dynamic_trials)), 4
            ),
            "mean_settling_time_sec": _round_or_none(np.mean(settling_times), 3)
            if settling_times
            else None,
            "p95_settling_time_sec": _round_or_none(
                np.percentile(settling_times, 95), 3
            )
            if settling_times
            else None,
            "total_intermediate_stops": sum(stop_counts),
            "mean_intermediate_stops": _round_or_none(np.mean(stop_counts), 3)
            if stop_counts
            else None,
            "zero_stop_trial_rate": round(
                _safe_div(sum(count == 0 for count in stop_counts), len(stop_counts)), 4
            ),
        },
        "safety": {
            "trial_count": len(safety_trials),
            "success_count": safety_passes,
            "success_rate": round(_safe_div(safety_passes, len(safety_trials)), 4),
            "by_scenario": _safety_by_scenario(safety_trials),
        },
    }


def _safety_by_scenario(rows: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    scenarios = sorted({str(row.get("scenario", "")) for row in rows})
    for scenario in scenarios:
        selected = [row for row in rows if str(row.get("scenario")) == scenario]
        passed = sum(bool(row.get("success")) for row in selected)
        output[scenario] = {
            "trial_count": len(selected),
            "success_count": passed,
            "success_rate": round(_safe_div(passed, len(selected)), 4),
        }
    return output


def format_arm_metrics(metrics: dict[str, Any]) -> str:
    static = metrics.get("static_distance", {})
    dynamic = metrics.get("dynamic", {})
    safety = metrics.get("safety", {})
    lines = [
        "[목표거리 정상상태 정확도]",
        f"표본: {static.get('sample_count', 0)}",
        f"MAE: {static.get('mae_mm')} mm",
        f"RMSE: {static.get('rmse_m')} m",
        f"최대 절대오차: {static.get('max_absolute_error_m')} m",
        "",
        "[동적 추종]",
        (
            f"성공: {dynamic.get('success_count', 0)}/"
            f"{dynamic.get('trial_count', 0)} "
            f"({float(dynamic.get('success_rate', 0.0)) * 100:.1f}%)"
        ),
        f"평균 정착시간: {dynamic.get('mean_settling_time_sec')} s",
        f"P95 정착시간: {dynamic.get('p95_settling_time_sec')} s",
        f"중간 정지 총합: {dynamic.get('total_intermediate_stops', 0)}회",
        (
            "무정지 이동 비율: "
            f"{float(dynamic.get('zero_stop_trial_rate', 0.0)) * 100:.1f}%"
        ),
        "",
        "[반복 정밀도: 같은 사용자 위치에서 최종 monitor X]",
    ]
    repeatability = metrics.get("repeatability", {})
    if not repeatability:
        lines.append("아직 반복 표본이 없습니다.")
    for nominal, values in repeatability.items():
        std_m = values.get("std_monitor_x_m")
        std_mm = None if std_m is None else float(std_m) * 1000.0
        lines.append(
            f"사용자 {nominal}: N={values.get('sample_count', 0)}, "
            f"표준편차={None if std_mm is None else round(std_mm, 2)} mm, "
            f"범위={values.get('range_monitor_x_m')} m"
        )
    lines.extend(
        [
            "",
            "[안전 시나리오]",
            (
                f"성공: {safety.get('success_count', 0)}/"
                f"{safety.get('trial_count', 0)} "
                f"({float(safety.get('success_rate', 0.0)) * 100:.1f}%)"
            ),
        ]
    )
    for scenario, values in safety.get("by_scenario", {}).items():
        lines.append(
            f"{scenario}: {values.get('success_count', 0)}/"
            f"{values.get('trial_count', 0)}"
        )
    return "\n".join(lines)


def _write_csv_atomic(path: Path, fields: list[str], rows: list[dict[str, Any]]):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _write_json_atomic(path: Path, value: dict[str, Any]):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


class MonitorArmEvaluationWorker(QThread):
    status_changed = pyqtSignal(str)
    state_changed = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    tracking_changed = pyqtSignal(bool)
    motion_finished = pyqtSignal(str, bool, str)
    dynamic_activity_changed = pyqtSignal(bool)
    dynamic_finished = pyqtSignal(dict)
    safety_activity_changed = pyqtSignal(bool)
    safety_finished = pyqtSignal(dict)
    metrics_changed = pyqtSignal(dict)
    exported = pyqtSignal(str)
    fatal_error = pyqtSignal(str)

    def __init__(self, session_dir: Path, fixed_tof_m: float | None = None, parent=None):
        super().__init__(parent)
        self.session_dir = Path(session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.fixed_tof_m = fixed_tof_m
        self.commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.running = False
        self.connected = False
        self.tracking_enabled = False
        self.settings: dict[str, Any] = {}
        self.tof_service = None
        self.tof_source = None
        self.motor_service = None
        self.motor12 = None
        self.safety = None

        self.started_monotonic = time.monotonic()
        self.last_telemetry_time = 0.0
        self.last_state_emit = 0.0
        self.latest_state: dict[str, Any] = {}
        self.latest_tof_state: dict[str, Any] = {}
        self.latest_raw_user_x_m: float | None = None
        self.latest_control_user_x_m: float | None = None
        self.latest_control_saturated = False
        self.latest_angles: JointCommand | None = None
        self.latest_monitor_pose = None
        self.latest_safety_state: dict[str, Any] = {}
        self.latest_motor_state: dict[str, Any] = {}

        self.simulated_landmark_valid = True
        self.simulated_posture = "Optimal"
        self.configured_working_target: JointCommand | None = None
        self.working_reference: JointCommand | None = None

        self.motion_kind: str | None = None
        self.motion_target: JointCommand | None = None
        self.motion_started_at: float | None = None
        self.motion_stable_samples = 0
        self.motion_tolerance_deg = 3.0
        self.motion_required_samples = 2
        self.motion_timeout_sec = 25.0

        self.dynamic_trial: dict[str, Any] | None = None
        self.dynamic_next_id = 1
        self.dynamic_change_threshold_m = 0.04
        self.dynamic_tolerance_m = 0.03
        self.dynamic_stable_sec = 1.0
        self.dynamic_timeout_sec = 20.0

        self.safety_trial: dict[str, Any] | None = None
        self.safety_next_id = 1

        self.telemetry_rows: list[dict[str, Any]] = []
        self.static_trials: list[dict[str, Any]] = []
        self.dynamic_trials: list[dict[str, Any]] = []
        self.safety_trials: list[dict[str, Any]] = []

    def send(self, command: str, payload=None):
        self.commands.put((command, payload))

    def request_connect(self, working_angles: dict[str, Any] | None = None):
        self.send("CONNECT", working_angles)

    def request_set_working_target(self, working_angles: dict[str, Any] | None):
        self.send("SET_WORKING_TARGET", working_angles)

    def request_working(self):
        self.send("MOVE_WORKING")

    def request_rest(self):
        self.send("MOVE_REST")

    def request_tracking(self, enabled: bool):
        self.send("TRACKING", bool(enabled))

    def request_static_sample(self, payload: dict[str, Any]):
        self.send("STATIC_SAMPLE", dict(payload))

    def request_dynamic_start(self):
        self.send("DYNAMIC_START")

    def request_dynamic_cancel(self):
        self.send("DYNAMIC_CANCEL")

    def request_safety_start(self, scenario: str):
        self.send("SAFETY_START", str(scenario))

    def request_inputs_normal(self):
        self.send("INPUTS_NORMAL")

    def request_export(self):
        self.send("EXPORT")

    def request_shutdown(self):
        self.send("SHUTDOWN")

    def _load_settings(self):
        with SETTINGS_PATH.open("r", encoding="utf-8") as file:
            self.settings = json.load(file)
        control = self.settings.get("control", {})
        self.motion_tolerance_deg = float(
            control.get("working_start_arrival_tolerance_deg", 3.0)
        )
        self.motion_required_samples = int(
            control.get("working_start_stable_samples", 2)
        )
        self.motion_timeout_sec = float(
            control.get("working_start_timeout_sec", 25.0)
        )

    def _set_working_target(self, payload: dict[str, Any] | None) -> bool:
        if not isinstance(payload, dict):
            self.configured_working_target = None
            self.status_changed.emit(
                "저장된 작업자세가 없습니다. 작업자세 이동을 실행하지 않습니다."
            )
            return False
        try:
            shoulder = float(payload["shoulder_lift"])
            elbow = float(payload["elbow_flex"])
            if not math.isfinite(shoulder) or not math.isfinite(elbow):
                raise ValueError("관절각이 유한한 값이 아닙니다.")
        except (KeyError, TypeError, ValueError) as error:
            self.configured_working_target = None
            self.status_changed.emit(f"작업자세 보정각이 유효하지 않습니다: {error}")
            return False
        self.configured_working_target = JointCommand(shoulder, elbow)
        self.status_changed.emit(
            f"작업자세 목표 적용: M1 {shoulder:.2f}°, M2 {elbow:.2f}°"
        )
        return True

    def _connect_hardware(self):
        if self.connected:
            self.connection_changed.emit(True, "이미 연결되어 있습니다.")
            return
        try:
            self._load_settings()
            tof_cfg = self.settings["tof"]
            self.tof_service = create_tof_service(
                tof_cfg, fixed_range_override_m=self.fixed_tof_m
            )
            self.tof_source = ToFUserXSource(
                self.tof_service,
                sensor_origin_x_m=float(tof_cfg.get("sensor_origin_x_m", 0.0)),
                minimum_user_x_m=float(tof_cfg["minimum_user_x_m"]),
                maximum_user_x_m=float(tof_cfg["maximum_user_x_m"]),
            )
            self.motor_service = MotorService()
            self.motor12 = InstrumentedMotor12Controller(
                self.motor_service, settings=self.settings
            )
            safety_cfg = self.settings.get("safety", {})
            self.safety = MonitorArmSafetySupervisor(
                absence_timeout_sec=safety_cfg.get("absence_timeout_sec", 5.0),
                reacquire_stable_sec=safety_cfg.get("reacquire_stable_sec", 1.0),
                posture_confidence=safety_cfg.get("posture_confidence_min", 0.7),
                posture_stale_sec=safety_cfg.get("posture_stale_sec", 1.0),
            )
            tof_ok = self.tof_source.open()
            motor_ok = self.motor_service.open()
            motor12_ok = bool(motor_ok and self.motor12.initialize())
            if not (tof_ok and motor_ok and motor12_ok):
                errors = [
                    getattr(self.tof_service, "last_error", None),
                    getattr(self.motor_service, "last_error", None),
                    getattr(self.motor12, "last_error", None),
                ]
                raise RuntimeError(" / ".join(str(value) for value in errors if value))
            self.connected = True
            self.latest_tof_state = dict(self.tof_service.update(force=True))
            message = "ToF와 Motor1/2 연결 완료. 아직 모터는 움직이지 않았습니다."
            self.connection_changed.emit(True, message)
            self.status_changed.emit(message)
        except Exception as error:
            self._close_hardware()
            message = f"하드웨어 연결 실패: {error}"
            self.connection_changed.emit(False, message)
            self.status_changed.emit(message)

    def _close_hardware(self):
        self.tracking_enabled = False
        self.connected = False
        if self.motor12 is not None:
            self.motor12.close()
        if self.motor_service is not None:
            self.motor_service.close()
        if self.tof_source is not None:
            self.tof_source.close()
        self.motor12 = None
        self.motor_service = None
        self.tof_source = None
        self.tof_service = None
        self.safety = None

    def _start_motion(
        self, kind: str, target: JointCommand, result: dict[str, Any]
    ) -> bool:
        if not result.get("accepted"):
            self.motion_finished.emit(kind, False, str(result.get("error", "명령 거부")))
            return False
        self.motion_kind = kind
        self.motion_target = target
        self.motion_started_at = time.monotonic()
        self.motion_stable_samples = 0
        self.status_changed.emit(f"{kind} 이동 중입니다.")
        return True

    def _move_working(self):
        if not self.connected or self.motor12 is None:
            self.motion_finished.emit("WORKING", False, "먼저 하드웨어를 연결하세요.")
            return
        if self.configured_working_target is None:
            self.motion_finished.emit(
                "WORKING",
                False,
                "저장된 사용자 작업자세가 없어 영점 이동을 차단했습니다.",
            )
            return
        self.tracking_enabled = False
        self.tracking_changed.emit(False)
        target = self.configured_working_target
        result = self.motor12.move_to_working_smooth(target)
        self._start_motion("WORKING", target, result)

    def _move_rest(self, safety_trial: bool = False):
        if not self.connected or self.motor12 is None:
            self.motion_finished.emit("REST", False, "먼저 하드웨어를 연결하세요.")
            if safety_trial and self.safety_trial is not None:
                self._finish_safety_trial(
                    False, "휴식자세 명령 전 하드웨어가 연결되지 않았습니다.", time.monotonic()
                )
            return
        self.tracking_enabled = False
        self.tracking_changed.emit(False)
        target = self.motor12.rest_command
        result = self.motor12.move_to_rest()
        accepted = self._start_motion(
            "SAFETY_REST" if safety_trial else "REST", target, result
        )
        if safety_trial and not accepted and self.safety_trial is not None:
            self._finish_safety_trial(
                False,
                f"휴식자세 명령 거부: {result.get('error', '원인 미상')}",
                time.monotonic(),
            )

    def _read_tof_input(self):
        self.latest_raw_user_x_m = None
        self.latest_control_user_x_m = None
        self.latest_control_saturated = False
        if self.tof_service is None or self.tof_source is None:
            return
        self.latest_tof_state = dict(self.tof_service.update())
        try:
            raw = self.tof_source.read_raw_user_x_m()
            control = self.tof_source.clamp_user_x_m(raw)
            self.latest_raw_user_x_m = raw
            self.latest_control_user_x_m = control
            self.latest_control_saturated = abs(raw - control) > 1e-9
        except (TypeError, ValueError):
            pass

    def _tof_presence_valid(self) -> bool:
        state = self.latest_tof_state
        value = state.get("filtered_distance_m")
        if not state.get("valid") or value is None:
            return False
        safety_cfg = self.settings.get("safety", {})
        minimum = float(safety_cfg.get("presence_min_m", 0.3))
        maximum = float(safety_cfg.get("presence_max_m", 1.5))
        try:
            return minimum < float(value) < maximum
        except (TypeError, ValueError):
            return False

    def _update_safety_and_control(self, now: float):
        if not self.connected or self.motor12 is None or self.safety is None:
            return
        tof_valid = self._tof_presence_valid()
        inference = {
            "posture_type": self.simulated_posture,
            "confidence": 1.0,
            "timestamp": time.time(),
        }
        safety_active = self.tracking_enabled or self.safety_trial is not None
        if safety_active:
            self.latest_safety_state = self.safety.update(
                tof_valid,
                self.simulated_landmark_valid,
                inference,
                now=now,
            )
        else:
            self.safety.reset()
            self.latest_safety_state = self.safety.snapshot(now)

        if self.latest_safety_state.get("request_return"):
            target = self.working_reference or self.configured_working_target
            if target is None:
                message = "사용자 복귀용 작업자세 보정각이 없어 영점 이동을 차단했습니다."
                self.status_changed.emit(message)
                if self.safety_trial is not None:
                    self._finish_safety_trial(False, message, now)
                return
            return_result = self.motor12.move_to_working_smooth(target)
            if self.safety_trial is not None:
                self.safety_trial["return_requested_at"] = now
                self.safety_trial["return_accepted"] = bool(
                    return_result.get("accepted")
                )
                if return_result.get("accepted"):
                    self._start_motion("SAFETY_RETURN", target, return_result)
                else:
                    self._finish_safety_trial(
                        False,
                        f"작업 초기위치 복귀 명령 거부: "
                        f"{return_result.get('error', '원인 미상')}",
                        now,
                    )

        input_state = {
            "valid": self.latest_control_user_x_m is not None,
            "user_x_m": self.latest_control_user_x_m,
            "last_error": None
            if self.latest_control_user_x_m is not None
            else "ToF 사용자 X가 유효하지 않습니다.",
        }
        control_active = bool(
            self.tracking_enabled
            and self.motion_kind is None
            and self.latest_safety_state.get("tracking_allowed", False)
            and input_state["valid"]
        )
        self.latest_motor_state = self.motor12.update(
            {
                "now": now,
                "motor12": {
                    "control_active": control_active,
                    "input": input_state,
                },
            }
        )

    def _poll_arm(self, now: float):
        if not self.connected or self.motor12 is None:
            return
        if now - self.last_telemetry_time < 0.10:
            return
        self.last_telemetry_time = now
        try:
            cache_age = (
                0.12
                if self.tracking_enabled and self.motion_kind is None
                else 0.0
            )
            self.latest_angles, self.latest_monitor_pose = self.motor12.latest_arm_state(
                max_cache_age_sec=cache_age
            )
        except Exception as error:
            self.status_changed.emit(f"모터 현재각 읽기 실패: {error}")
            return
        self._update_motion_arrival(now)
        self._update_dynamic_trial(now)
        self._append_telemetry(now)

    def _update_motion_arrival(self, now: float):
        if self.motion_kind is None or self.motion_target is None:
            return
        assert self.latest_angles is not None
        error = max(
            abs(
                self.latest_angles.shoulder_lift_deg
                - self.motion_target.shoulder_lift_deg
            ),
            abs(self.latest_angles.elbow_flex_deg - self.motion_target.elbow_flex_deg),
        )
        if error <= self.motion_tolerance_deg:
            self.motion_stable_samples += 1
        else:
            self.motion_stable_samples = 0
        elapsed = now - float(self.motion_started_at or now)
        complete = self.motion_stable_samples >= self.motion_required_samples
        timed_out = elapsed >= self.motion_timeout_sec
        if not complete and not timed_out:
            return

        kind = self.motion_kind
        success = bool(complete)
        message = (
            f"{kind} 이동 완료: 최대 관절오차 {error:.2f}°, {elapsed:.2f}초"
            if success
            else f"{kind} 이동 timeout: 최대 관절오차 {error:.2f}°"
        )
        if kind == "WORKING" and success:
            self.working_reference = JointCommand(
                self.latest_angles.shoulder_lift_deg,
                self.latest_angles.elbow_flex_deg,
            )
        self.motion_kind = None
        self.motion_target = None
        self.motion_started_at = None
        self.motion_stable_samples = 0
        self.motion_finished.emit(kind, success, message)
        self.status_changed.emit(message)
        if kind in ("SAFETY_REST", "SAFETY_RETURN") and self.safety_trial is not None:
            self._finish_safety_trial(success, message, now)

    def _current_estimated_distance(self) -> float | None:
        if self.latest_raw_user_x_m is None or self.latest_monitor_pose is None:
            return None
        return float(self.latest_raw_user_x_m) - float(self.latest_monitor_pose.x_m)

    def _append_telemetry(self, now: float):
        estimated_distance = self._current_estimated_distance()
        desired = (
            self.motor12.planner.desired_distance_m
            if self.motor12 is not None
            else None
        )
        row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "elapsed_sec": round(now - self.started_monotonic, 4),
            "tracking_enabled": int(self.tracking_enabled),
            "tof_available": int(bool(self.latest_tof_state.get("available"))),
            "tof_valid": int(bool(self.latest_tof_state.get("valid"))),
            "tof_filtered_distance_m": self.latest_tof_state.get(
                "filtered_distance_m"
            ),
            "raw_user_x_m": self.latest_raw_user_x_m,
            "control_user_x_m": self.latest_control_user_x_m,
            "control_saturated": int(self.latest_control_saturated),
            "shoulder_lift_deg": self.latest_angles.shoulder_lift_deg
            if self.latest_angles
            else None,
            "elbow_flex_deg": self.latest_angles.elbow_flex_deg
            if self.latest_angles
            else None,
            "monitor_x_m": self.latest_monitor_pose.x_m
            if self.latest_monitor_pose
            else None,
            "monitor_z_m": self.latest_monitor_pose.z_m
            if self.latest_monitor_pose
            else None,
            "estimated_user_monitor_distance_m": estimated_distance,
            "distance_error_m": (
                None
                if estimated_distance is None or desired is None
                else estimated_distance - desired
            ),
            "motor12_hold_reason": self.latest_motor_state.get("hold_reason"),
            "motor12_command_speed": self.latest_motor_state.get("command_speed"),
            "safety_state": self.latest_safety_state.get("state"),
            "safety_reason": self.latest_safety_state.get("reason"),
            "tracking_allowed": int(
                bool(self.latest_safety_state.get("tracking_allowed"))
            ),
            "simulated_landmark_valid": int(self.simulated_landmark_valid),
            "simulated_posture": self.simulated_posture,
            "dynamic_trial_id": self.dynamic_trial.get("trial_id")
            if self.dynamic_trial
            else None,
            "safety_trial_id": self.safety_trial.get("trial_id")
            if self.safety_trial
            else None,
        }
        self.telemetry_rows.append(row)
        self.latest_state = dict(row)
        self.latest_state["connected"] = self.connected
        self.latest_state["motion_kind"] = self.motion_kind
        self.state_changed.emit(self.latest_state)

    def _record_static(self, payload: dict[str, Any]):
        if (
            not self.connected
            or not self.tracking_enabled
            or self.motion_kind is not None
            or self.latest_monitor_pose is None
            or self.latest_raw_user_x_m is None
        ):
            self.status_changed.emit(
                "정적 기록은 유효한 ToF 값으로 자동추종 중이며 별도 자세 이동이 없을 때만 가능합니다."
            )
            return
        external = float(payload["external_actual_distance_m"])
        desired = float(self.motor12.planner.desired_distance_m)
        trial = {
            "trial_id": len(self.static_trials) + 1,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "nominal_user_x_m": float(payload["nominal_user_x_m"]),
            "approach_direction": str(payload["approach_direction"]),
            "external_actual_distance_m": external,
            "target_distance_m": desired,
            "absolute_error_m": abs(external - desired),
            "tof_raw_user_x_m": self.latest_raw_user_x_m,
            "tof_filtered_distance_m": self.latest_tof_state.get(
                "filtered_distance_m"
            ),
            "estimated_internal_distance_m": self._current_estimated_distance(),
            "shoulder_lift_deg": self.latest_angles.shoulder_lift_deg,
            "elbow_flex_deg": self.latest_angles.elbow_flex_deg,
            "monitor_x_m": self.latest_monitor_pose.x_m,
            "monitor_z_m": self.latest_monitor_pose.z_m,
        }
        self.static_trials.append(trial)
        self.status_changed.emit(
            f"정적 시험 #{trial['trial_id']} 기록: 외부거리 {external * 100:.1f}cm, "
            f"절대오차 {trial['absolute_error_m'] * 1000:.1f}mm"
        )
        self._export_results()

    def _start_dynamic(self):
        if not self.connected or not self.tracking_enabled:
            self.status_changed.emit("하드웨어 연결과 자동추종을 먼저 활성화하세요.")
            return
        if self.dynamic_trial is not None:
            self.status_changed.emit("이미 동적 시험 중입니다.")
            return
        if self.latest_raw_user_x_m is None:
            self.status_changed.emit("유효한 ToF 사용자 위치가 없습니다.")
            return
        now = time.monotonic()
        self.dynamic_trial = {
            "trial_id": self.dynamic_next_id,
            "started_at": datetime.now().isoformat(timespec="milliseconds"),
            "start_monotonic": now,
            "initial_user_x_m": float(self.latest_raw_user_x_m),
            "step_detected_at": None,
            "step_user_x_m": None,
            "settle_started_at": None,
            "movement_seen": False,
            "low_speed_started_at": None,
            "in_stop": False,
            "stop_count": 0,
            "previous_pose_x": self.latest_monitor_pose.x_m
            if self.latest_monitor_pose
            else None,
            "previous_pose_at": now,
        }
        self.dynamic_next_id += 1
        self.dynamic_activity_changed.emit(True)
        self.status_changed.emit(
            f"동적 시험 #{self.dynamic_trial['trial_id']} 대기 — 사용자가 4cm 이상 이동하세요."
        )

    def _cancel_dynamic(self, message="사용자가 동적 시험을 취소했습니다."):
        if self.dynamic_trial is None:
            return
        now = time.monotonic()
        self._finish_dynamic(False, message, now)

    def _update_dynamic_trial(self, now: float):
        trial = self.dynamic_trial
        if trial is None or self.latest_raw_user_x_m is None:
            return
        initial = float(trial["initial_user_x_m"])
        current_user = float(self.latest_raw_user_x_m)
        if trial["step_detected_at"] is None:
            if abs(current_user - initial) >= self.dynamic_change_threshold_m:
                trial["step_detected_at"] = now
                trial["step_user_x_m"] = current_user
                self.status_changed.emit(
                    f"사용자 이동 검출: {initial:.3f}m → {current_user:.3f}m. 정착을 측정합니다."
                )
            elif now - float(trial["start_monotonic"]) >= 30.0:
                self._finish_dynamic(False, "30초 안에 4cm 이동을 검출하지 못했습니다.", now)
            return

        if self.latest_monitor_pose is None:
            return
        previous_x = trial.get("previous_pose_x")
        previous_at = float(trial.get("previous_pose_at") or now)
        dt = max(1e-6, now - previous_at)
        speed = (
            0.0
            if previous_x is None
            else abs(float(self.latest_monitor_pose.x_m) - float(previous_x)) / dt
        )
        trial["previous_pose_x"] = float(self.latest_monitor_pose.x_m)
        trial["previous_pose_at"] = now

        if speed >= 0.006:
            trial["movement_seen"] = True
            trial["low_speed_started_at"] = None
            trial["in_stop"] = False
        elif trial["movement_seen"] and speed < 0.003:
            if trial["low_speed_started_at"] is None:
                trial["low_speed_started_at"] = now

        estimated = self._current_estimated_distance()
        desired = self.motor12.planner.desired_distance_m
        error = None if estimated is None else abs(estimated - desired)
        within = error is not None and error <= self.dynamic_tolerance_m

        if (
            trial["movement_seen"]
            and not within
            and trial["low_speed_started_at"] is not None
            and now - float(trial["low_speed_started_at"]) >= 0.30
            and not trial["in_stop"]
        ):
            trial["stop_count"] += 1
            trial["in_stop"] = True

        if within:
            if trial["settle_started_at"] is None:
                trial["settle_started_at"] = now
            if now - float(trial["settle_started_at"]) >= self.dynamic_stable_sec:
                self._finish_dynamic(True, "허용오차 내 1초 연속 정착", now)
                return
        else:
            trial["settle_started_at"] = None

        if now - float(trial["step_detected_at"]) >= self.dynamic_timeout_sec:
            self._finish_dynamic(False, "동적 정착 timeout", now)

    def _finish_dynamic(self, success: bool, message: str, now: float):
        trial = self.dynamic_trial
        if trial is None:
            return
        step_at = trial.get("step_detected_at")
        estimated = self._current_estimated_distance()
        desired = self.motor12.planner.desired_distance_m
        final_user = self.latest_raw_user_x_m
        settled_at = trial.get("settle_started_at") if success else now
        result = {
            "trial_id": trial["trial_id"],
            "started_at": trial["started_at"],
            "ended_at": datetime.now().isoformat(timespec="milliseconds"),
            "success": bool(success),
            "initial_user_x_m": trial["initial_user_x_m"],
            "final_user_x_m": final_user,
            "user_step_m": (
                None
                if final_user is None
                else float(final_user) - float(trial["initial_user_x_m"])
            ),
            "settling_time_sec": (
                None
                if step_at is None
                else round(float(settled_at or now) - float(step_at), 3)
            ),
            "intermediate_stop_count": int(trial["stop_count"]),
            "final_distance_error_m": (
                None if estimated is None else round(estimated - desired, 5)
            ),
            "timeout_sec": self.dynamic_timeout_sec,
            "message": message,
        }
        self.dynamic_trials.append(result)
        self.dynamic_trial = None
        self.dynamic_activity_changed.emit(False)
        self.dynamic_finished.emit(result)
        self.status_changed.emit(
            f"동적 시험 #{result['trial_id']} {'성공' if success else '실패'}: "
            f"정착 {result['settling_time_sec']}초, 중간정지 {result['intermediate_stop_count']}회"
        )
        self._export_results()

    def _start_safety(self, scenario: str):
        if not self.connected:
            self.status_changed.emit("먼저 하드웨어를 연결하세요.")
            return
        if self.safety_trial is not None:
            self.status_changed.emit("이미 안전 시나리오 시험 중입니다.")
            return
        now = time.monotonic()
        if scenario == "REACQUIRE" and not self.safety.return_requested:
            self.status_changed.emit(
                "REACQUIRE는 LANDMARK_LOSS_RETURN 또는 TOF_LOSS_RETURN 완료 후 실행하세요."
            )
            return
        self.safety_trial = {
            "trial_id": self.safety_next_id,
            "scenario": scenario,
            "started_at": datetime.now().isoformat(timespec="milliseconds"),
            "started_monotonic": now,
            "loss_detected_at": None,
            "hold_detected_at": None,
            "return_requested_at": None,
            "return_accepted": None,
        }
        self.safety_next_id += 1
        self.safety_activity_changed.emit(True)
        if scenario == "LANDMARK_LOSS_RETURN":
            self.simulated_landmark_valid = False
        elif scenario == "POSTURE_HOLD":
            self.simulated_posture = "Forward Head"
        elif scenario == "REACQUIRE":
            self.simulated_landmark_valid = True
            self.simulated_posture = "Optimal"
        elif scenario == "REST_RETURN":
            self._move_rest(safety_trial=True)
        self.status_changed.emit(f"안전 시험 시작: {scenario}")

    def _update_safety_trial(self, now: float):
        trial = self.safety_trial
        if trial is None:
            return
        scenario = trial["scenario"]
        state = str(self.latest_safety_state.get("state", ""))
        if scenario == "TOF_LOSS_RETURN":
            if trial["loss_detected_at"] is None and not self._tof_presence_valid():
                trial["loss_detected_at"] = now
            if trial["loss_detected_at"] is not None and state in (
                MonitorArmSafetySupervisor.SENSOR_GRACE,
                MonitorArmSafetySupervisor.ABSENT,
            ):
                trial["hold_detected_at"] = trial["hold_detected_at"] or now
            if (
                trial.get("return_requested_at") is not None
                and not trial.get("return_accepted")
            ):
                self._finish_safety_trial(False, "ToF 미검출 후 복귀 명령 거부", now)
            elif now - float(trial["started_monotonic"]) >= (
                self.safety.absence_timeout_sec + self.motion_timeout_sec + 1.0
            ):
                self._finish_safety_trial(False, "ToF 미검출 복귀 도착 timeout", now)
        elif scenario == "LANDMARK_LOSS_RETURN":
            trial["loss_detected_at"] = trial["loss_detected_at"] or trial[
                "started_monotonic"
            ]
            if state in (
                MonitorArmSafetySupervisor.SENSOR_GRACE,
                MonitorArmSafetySupervisor.ABSENT,
            ):
                trial["hold_detected_at"] = trial["hold_detected_at"] or now
            if (
                trial.get("return_requested_at") is not None
                and not trial.get("return_accepted")
            ):
                self._finish_safety_trial(False, "Landmark 미검출 후 복귀 명령 거부", now)
            elif now - float(trial["started_monotonic"]) >= (
                self.safety.absence_timeout_sec + self.motion_timeout_sec + 1.0
            ):
                self._finish_safety_trial(False, "Landmark 미검출 복귀 도착 timeout", now)
        elif scenario == "POSTURE_HOLD":
            if state == MonitorArmSafetySupervisor.POSTURE:
                trial["hold_detected_at"] = now
                self._finish_safety_trial(True, "비정상 자세 자동추종 HOLD", now)
            elif now - float(trial["started_monotonic"]) >= 2.0:
                self._finish_safety_trial(False, "2초 안에 POSTURE_HOLD 진입 실패", now)
        elif scenario == "REACQUIRE":
            if state == MonitorArmSafetySupervisor.TRACKING:
                self._finish_safety_trial(True, "재검출 안정화 후 자동추종 재개", now)
            elif now - float(trial["started_monotonic"]) >= 3.0:
                self._finish_safety_trial(False, "3초 안에 자동추종 재개 실패", now)
        elif scenario == "REST_RETURN":
            # Completion is handled by the actual-angle arrival poll.
            if now - float(trial["started_monotonic"]) >= self.motion_timeout_sec + 1.0:
                self._finish_safety_trial(False, "휴식자세 도착 timeout", now)

    def _finish_safety_trial(self, success: bool, message: str, now: float):
        trial = self.safety_trial
        if trial is None:
            return
        loss_at = trial.get("loss_detected_at") or trial.get("started_monotonic")
        hold_at = trial.get("hold_detected_at")
        result = {
            "trial_id": trial["trial_id"],
            "scenario": trial["scenario"],
            "started_at": trial["started_at"],
            "ended_at": datetime.now().isoformat(timespec="milliseconds"),
            "success": bool(success),
            "hold_latency_sec": (
                None if hold_at is None else round(float(hold_at) - float(loss_at), 3)
            ),
            "action_latency_sec": round(now - float(trial["started_monotonic"]), 3),
            "expected_action": {
                "TOF_LOSS_RETURN": "즉시 HOLD 후 약 5초에 복귀 명령, 실제 작업 초기위치 도착",
                "LANDMARK_LOSS_RETURN": "즉시 HOLD 후 약 5초에 복귀 명령, 실제 작업 초기위치 도착",
                "POSTURE_HOLD": "비정상 자세에서 자동추종 정지",
                "REACQUIRE": "약 1초 안정화 후 자동추종 재개",
                "REST_RETURN": "실제 관절각이 휴식자세 허용오차에 도착",
            }.get(trial["scenario"], ""),
            "message": message,
        }
        self.safety_trials.append(result)
        self.safety_trial = None
        self.safety_activity_changed.emit(False)
        self.safety_finished.emit(result)
        self.status_changed.emit(
            f"안전 시험 #{result['trial_id']} {'성공' if success else '실패'}: {message}"
        )
        self._export_results()

    def _inputs_normal(self):
        self.simulated_landmark_valid = True
        self.simulated_posture = "Optimal"
        self.status_changed.emit("모의 Landmark/Posture 입력을 정상으로 복원했습니다.")

    def _handle_commands(self) -> bool:
        while True:
            try:
                command, payload = self.commands.get_nowait()
            except queue.Empty:
                return True
            if command == "CONNECT":
                self._set_working_target(payload)
                self._connect_hardware()
            elif command == "SET_WORKING_TARGET":
                self._set_working_target(payload)
            elif command == "MOVE_WORKING":
                self._move_working()
            elif command == "MOVE_REST":
                self._move_rest()
            elif command == "TRACKING":
                self.tracking_enabled = bool(payload) and self.connected
                if not self.tracking_enabled and self.dynamic_trial is not None:
                    self._cancel_dynamic("자동추종이 정지되어 동적 시험을 취소했습니다.")
                self.tracking_changed.emit(self.tracking_enabled)
                self.status_changed.emit(
                    "ToF 자동추종 활성" if self.tracking_enabled else "자동추종 정지"
                )
            elif command == "STATIC_SAMPLE":
                self._record_static(payload)
            elif command == "DYNAMIC_START":
                self._start_dynamic()
            elif command == "DYNAMIC_CANCEL":
                self._cancel_dynamic()
            elif command == "SAFETY_START":
                self._start_safety(payload)
            elif command == "INPUTS_NORMAL":
                self._inputs_normal()
            elif command == "EXPORT":
                self._export_results()
            elif command == "SHUTDOWN":
                self.tracking_enabled = False
                self._export_results()
                return False

    def _export_results(self):
        try:
            metrics = calculate_arm_metrics(
                self.static_trials, self.dynamic_trials, self.safety_trials
            )
            metrics.update(
                {
                    "session_dir": str(self.session_dir),
                    "settings_path": str(SETTINGS_PATH),
                    "scope": "Motor1/2 ToF fore-aft tracking",
                    "fixed_tof_override_m": self.fixed_tof_m,
                    "configured_working_target_deg": (
                        None
                        if self.configured_working_target is None
                        else {
                            "shoulder_lift": self.configured_working_target.shoulder_lift_deg,
                            "elbow_flex": self.configured_working_target.elbow_flex_deg,
                        }
                    ),
                    "measured_working_reference_deg": (
                        None
                        if self.working_reference is None
                        else {
                            "shoulder_lift": self.working_reference.shoulder_lift_deg,
                            "elbow_flex": self.working_reference.elbow_flex_deg,
                        }
                    ),
                    "dynamic_definition": {
                        "user_step_threshold_m": self.dynamic_change_threshold_m,
                        "settling_tolerance_m": self.dynamic_tolerance_m,
                        "stable_duration_sec": self.dynamic_stable_sec,
                        "timeout_sec": self.dynamic_timeout_sec,
                        "stop_speed_enter_m_s": 0.003,
                        "stop_speed_exit_m_s": 0.006,
                        "stop_min_duration_sec": 0.30,
                    },
                }
            )
            _write_csv_atomic(
                self.session_dir / "telemetry.csv", TELEMETRY_FIELDS, self.telemetry_rows
            )
            _write_csv_atomic(
                self.session_dir / "static_trials.csv", STATIC_FIELDS, self.static_trials
            )
            _write_csv_atomic(
                self.session_dir / "dynamic_trials.csv", DYNAMIC_FIELDS, self.dynamic_trials
            )
            _write_csv_atomic(
                self.session_dir / "safety_trials.csv", SAFETY_FIELDS, self.safety_trials
            )
            _write_json_atomic(self.session_dir / "summary.json", metrics)
            self.metrics_changed.emit(metrics)
            self.exported.emit(str(self.session_dir))
        except Exception as error:
            self.status_changed.emit(f"평가 결과 저장 실패: {error}")

    def run(self):
        self.running = True
        try:
            self.status_changed.emit(
                "대기 중 — 하드웨어 연결 버튼은 연결만 수행하며 모터를 움직이지 않습니다."
            )
            while self.running:
                if not self._handle_commands():
                    break
                now = time.monotonic()
                if self.connected:
                    self._read_tof_input()
                    self._update_safety_and_control(now)
                    self._poll_arm(now)
                    self._update_safety_trial(now)
                if now - self.last_state_emit >= 0.25:
                    self.last_state_emit = now
                    if not self.connected:
                        self.state_changed.emit({"connected": False})
                self.msleep(5)
        except Exception as error:
            detail = traceback.format_exc()
            print(detail)
            self.fatal_error.emit(f"{error}\n\n{detail}")
        finally:
            self.running = False
            self._close_hardware()


class MonitorArmEvaluationWindow(QMainWindow):
    SAFETY_OPTIONS = {
        "ToF 실제 미검출 → 5초 복귀": "TOF_LOSS_RETURN",
        "Landmark 모의 미검출 → 5초 복귀": "LANDMARK_LOSS_RETURN",
        "비정상 자세 모의 HOLD": "POSTURE_HOLD",
        "사용자/센서 재검출": "REACQUIRE",
        "측정 종료 휴식자세": "REST_RETURN",
    }

    def __init__(self, fixed_tof_m: float | None = None):
        super().__init__()
        session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = WORKSPACE_DIR / "data" / "monitor_arm_evaluation" / session_name
        self.worker = MonitorArmEvaluationWorker(
            self.session_dir, fixed_tof_m=fixed_tof_m, parent=self
        )
        self.connected = False
        self.tracking = False
        self.motion_active = False
        self.dynamic_active = False
        self.safety_active = False
        self.closing = False
        self.working_pose_options = load_working_pose_options()
        self.setWindowTitle("POCO 모니터암 Motor1/2 성능평가")
        self.compact_display = self._fit_to_display()
        self._build_ui()
        self._connect_worker()
        self.worker.start()

    def _fit_to_display(self) -> bool:
        """Fit 800x480/1024x600 Raspberry Pi displays without clipping."""
        screen = QApplication.primaryScreen()
        if screen is None:
            self.resize(1024, 600)
            return True
        available = screen.availableGeometry()
        compact = available.width() <= 1024 or available.height() <= 600
        if compact:
            self.setGeometry(available)
        else:
            self.resize(
                min(1180, available.width()),
                min(760, available.height()),
            )
        return compact

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        margin = 5 if self.compact_display else 10
        outer.setContentsMargins(margin, margin, margin, margin)
        outer.setSpacing(6 if self.compact_display else 10)
        if self.compact_display:
            central.setStyleSheet(
                "QWidget{font-size:11px;}"
                "QGroupBox{font-weight:600;margin-top:7px;}"
                "QGroupBox::title{subcontrol-origin:margin;left:7px;}"
                "QPushButton{min-height:28px;padding:2px 5px;}"
            )

        controls_scroll = QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        controls_scroll.setFrameShape(QScrollArea.NoFrame)
        controls_widget = QWidget()
        controls = QVBoxLayout(controls_widget)
        controls.setContentsMargins(5, 5, 5, 5)
        controls.setSpacing(5)
        warning = QLabel(
            "주의: 실제 Motor1/2와 ToF를 제어합니다. 충돌물을 제거하고 모니터를 지지하세요. "
            "연결만으로는 모터가 움직이지 않습니다."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet(
            f"background:#fef2f2;border:2px solid #ef4444;color:#991b1b;"
            f"padding:{5 if self.compact_display else 9}px;font-weight:700;"
        )
        controls.addWidget(warning)

        hardware_group = QGroupBox("1. 하드웨어와 자세")
        hardware_layout = QGridLayout(hardware_group)
        self.working_pose_combo = QComboBox()
        if self.working_pose_options:
            for option in self.working_pose_options:
                self.working_pose_combo.addItem(option["label"], option)
        else:
            self.working_pose_combo.addItem("저장된 작업자세 없음", None)
        self.working_pose_combo.currentIndexChanged.connect(
            self._working_pose_changed
        )
        self.connect_button = QPushButton("ToF + Motor1/2 연결")
        self.connect_button.clicked.connect(self._request_connect)
        self.working_button = QPushButton(
            "작업자세 이동" if self.compact_display else "휴식/현재 → 작업자세"
        )
        self.working_button.setEnabled(False)
        self.working_button.clicked.connect(self._move_working)
        self.rest_button = QPushButton(
            "휴식자세 이동" if self.compact_display else "작업 → 휴식자세"
        )
        self.rest_button.setEnabled(False)
        self.rest_button.clicked.connect(self._move_rest)
        self.tracking_button = QPushButton("ToF 자동추종 시작")
        self.tracking_button.setEnabled(False)
        self.tracking_button.clicked.connect(self._toggle_tracking)
        hardware_layout.addWidget(QLabel("작업자세 기준"), 0, 0)
        hardware_layout.addWidget(self.working_pose_combo, 0, 1)
        hardware_layout.addWidget(self.connect_button, 1, 0, 1, 2)
        hardware_layout.addWidget(self.working_button, 2, 0)
        hardware_layout.addWidget(self.rest_button, 2, 1)
        hardware_layout.addWidget(self.tracking_button, 3, 0, 1, 2)
        controls.addWidget(hardware_group)

        static_group = QGroupBox("2. 정적 MAE + 반복 정밀도")
        static_layout = QGridLayout(static_group)
        self.nominal_user_x = QDoubleSpinBox()
        self.nominal_user_x.setRange(50.0, 100.0)
        self.nominal_user_x.setValue(70.0)
        self.nominal_user_x.setDecimals(1)
        self.nominal_user_x.setSuffix(" cm")
        self.external_distance = QDoubleSpinBox()
        self.external_distance.setRange(20.0, 100.0)
        self.external_distance.setValue(50.0)
        self.external_distance.setDecimals(1)
        self.external_distance.setSuffix(" cm")
        self.approach_direction = QComboBox()
        self.approach_direction.addItems(["가까운 쪽에서 접근", "먼 쪽에서 접근"])
        self.static_button = QPushButton("현재 안정값 기록")
        self.static_button.setEnabled(False)
        self.static_button.clicked.connect(self._record_static)
        static_layout.addWidget(QLabel("명목 사용자 X"), 0, 0)
        static_layout.addWidget(self.nominal_user_x, 0, 1)
        static_layout.addWidget(
            QLabel(
                "실제 사용자-모니터 거리"
                if self.compact_display
                else "줄자로 잰 실제 사용자-모니터 거리"
            ),
            1,
            0,
        )
        static_layout.addWidget(self.external_distance, 1, 1)
        static_layout.addWidget(QLabel("접근 방향"), 2, 0)
        static_layout.addWidget(self.approach_direction, 2, 1)
        static_layout.addWidget(self.static_button, 3, 0, 1, 2)
        controls.addWidget(static_group)

        dynamic_group = QGroupBox(
            "3. 동적 응답" if self.compact_display else "3. 동적 정착시간 + 이동 중 정지 횟수"
        )
        dynamic_layout = QVBoxLayout(dynamic_group)
        dynamic_instruction = QLabel(
            "자동추종을 켠 안정 상태에서 시작한 뒤 사용자가 앞/뒤로 4cm 이상 이동합니다.\n"
            "내부 추정거리 오차 ±3cm가 1초 유지되면 자동 완료됩니다."
        )
        dynamic_instruction.setWordWrap(True)
        dynamic_layout.addWidget(dynamic_instruction)
        dynamic_actions = QHBoxLayout()
        self.dynamic_start_button = QPushButton("동적 시험 시작")
        self.dynamic_start_button.setEnabled(False)
        self.dynamic_start_button.clicked.connect(self.worker.request_dynamic_start)
        self.dynamic_cancel_button = QPushButton("동적 시험 취소")
        self.dynamic_cancel_button.setEnabled(False)
        self.dynamic_cancel_button.clicked.connect(self.worker.request_dynamic_cancel)
        dynamic_actions.addWidget(self.dynamic_start_button)
        dynamic_actions.addWidget(self.dynamic_cancel_button)
        dynamic_layout.addLayout(dynamic_actions)
        controls.addWidget(dynamic_group)

        safety_group = QGroupBox("4. 안전 시나리오")
        safety_layout = QGridLayout(safety_group)
        self.safety_combo = QComboBox()
        self.safety_combo.addItems(list(self.SAFETY_OPTIONS))
        self.safety_start_button = QPushButton("선택 안전 시험 시작")
        self.safety_start_button.setEnabled(False)
        self.safety_start_button.clicked.connect(self._start_safety)
        self.inputs_normal_button = QPushButton("모의 입력 정상화")
        self.inputs_normal_button.setEnabled(False)
        self.inputs_normal_button.clicked.connect(self.worker.request_inputs_normal)
        safety_layout.addWidget(self.safety_combo, 0, 0, 1, 2)
        safety_layout.addWidget(self.safety_start_button, 1, 0)
        safety_layout.addWidget(self.inputs_normal_button, 1, 1)
        controls.addWidget(safety_group)

        self.status_label = QLabel("초기화 중...")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(
            "background:#fff7ed;border:1px solid #fdba74;padding:8px;font-weight:600;"
        )
        self.output_label = QLabel(f"저장 폴더: {self.session_dir}")
        self.output_label.setWordWrap(True)
        self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        controls.addWidget(self.status_label)
        controls.addWidget(self.output_label)
        controls.addStretch(1)
        controls_scroll.setWidget(controls_widget)
        outer.addWidget(controls_scroll, 1)

        monitor_tabs = QTabWidget()
        monitor_tabs.setDocumentMode(True)
        live_group = QGroupBox("실시간 ToF / IK / Safety / Motor 상태")
        live_layout = QVBoxLayout(live_group)
        self.live_text = QPlainTextEdit()
        self.live_text.setReadOnly(True)
        self.live_text.setPlainText("하드웨어 연결 대기")
        live_layout.addWidget(self.live_text)
        monitor_tabs.addTab(live_group, "실시간 상태")
        metrics_group = QGroupBox("자동 계산 결과")
        metrics_layout = QVBoxLayout(metrics_group)
        self.metrics_text = QPlainTextEdit()
        self.metrics_text.setReadOnly(True)
        self.metrics_text.setPlainText(
            format_arm_metrics(calculate_arm_metrics([], [], []))
        )
        self.export_button = QPushButton("현재 결과 저장")
        self.export_button.clicked.connect(self.worker.request_export)
        metrics_layout.addWidget(self.metrics_text)
        metrics_layout.addWidget(self.export_button)
        monitor_tabs.addTab(metrics_group, "측정 결과")
        outer.addWidget(monitor_tabs, 1)

    def _connect_worker(self):
        self.worker.status_changed.connect(self.status_label.setText)
        self.worker.state_changed.connect(self._update_live)
        self.worker.connection_changed.connect(self._on_connection)
        self.worker.tracking_changed.connect(self._on_tracking)
        self.worker.motion_finished.connect(self._on_motion_finished)
        self.worker.dynamic_activity_changed.connect(self._on_dynamic_activity)
        self.worker.dynamic_finished.connect(self._on_dynamic_finished)
        self.worker.safety_activity_changed.connect(self._on_safety_activity)
        self.worker.safety_finished.connect(self._on_safety_finished)
        self.worker.metrics_changed.connect(
            lambda metrics: self.metrics_text.setPlainText(format_arm_metrics(metrics))
        )
        self.worker.exported.connect(
            lambda path: self.output_label.setText(f"저장 완료: {path}")
        )
        self.worker.fatal_error.connect(self._on_fatal_error)

    def _on_connection(self, success: bool, message: str):
        self.connected = bool(success)
        self._refresh_controls()
        if not success:
            QMessageBox.warning(self, "하드웨어 연결", message)

    def _selected_working_option(self) -> dict[str, Any] | None:
        option = self.working_pose_combo.currentData()
        return dict(option) if isinstance(option, dict) else None

    def _selected_working_angles(self) -> dict[str, float] | None:
        option = self._selected_working_option()
        if option is None or not isinstance(option.get("angles"), dict):
            return None
        return dict(option["angles"])

    def _request_connect(self):
        angles = self._selected_working_angles()
        if angles is None:
            QMessageBox.warning(
                self,
                "작업자세 없음",
                "저장된 보정 작업자세를 찾지 못했습니다. 메인 프로그램에서 보정 또는 프로필 저장을 먼저 완료해주세요.",
            )
            return
        self.worker.request_connect(angles)

    def _working_pose_changed(self, _index: int | None = None):
        self._refresh_controls()
        if self.connected:
            self.worker.request_set_working_target(self._selected_working_angles())

    def _refresh_controls(self):
        idle = bool(
            self.connected
            and not self.motion_active
            and not self.dynamic_active
            and not self.safety_active
        )
        self.connect_button.setEnabled(not self.connected)
        self.working_pose_combo.setEnabled(
            not self.motion_active
            and not self.dynamic_active
            and not self.safety_active
        )
        self.working_button.setEnabled(
            idle and self._selected_working_angles() is not None
        )
        self.rest_button.setEnabled(idle)
        self.tracking_button.setEnabled(idle)
        self.static_button.setEnabled(idle and self.tracking)
        self.dynamic_start_button.setEnabled(idle and self.tracking)
        self.dynamic_cancel_button.setEnabled(self.connected and self.dynamic_active)
        self.safety_start_button.setEnabled(idle)
        self.inputs_normal_button.setEnabled(self.connected and not self.safety_active)

    def _move_working(self):
        option = self._selected_working_option()
        angles = self._selected_working_angles()
        if option is None or angles is None:
            QMessageBox.warning(
                self,
                "작업자세 없음",
                "저장된 사용자 작업자세가 없어 영점 이동을 차단했습니다.",
            )
            return
        answer = QMessageBox.question(
            self,
            "작업자세 이동 확인",
            f"'{option.get('source', '선택 보정값')}' 작업자세로 이동합니다.\n"
            f"M1 {angles['shoulder_lift']:.2f}° / M2 {angles['elbow_flex']:.2f}°\n\n"
            "주변 충돌물을 제거하고 모니터를 지지했습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.motion_active = True
            self._refresh_controls()
            self.worker.request_working()

    def _move_rest(self):
        answer = QMessageBox.question(
            self,
            "휴식자세 이동 확인",
            "모터 1·2를 휴식자세로 이동합니다. 모니터를 지지하고 충돌물이 없는지 확인했습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.motion_active = True
            self._refresh_controls()
            self.worker.request_rest()

    def _on_motion_finished(self, kind: str, success: bool, message: str):
        self.motion_active = False
        self._refresh_controls()
        if not success and not kind.startswith("SAFETY_"):
            QMessageBox.warning(self, "모터 이동", message)

    def _toggle_tracking(self):
        if not self.tracking:
            answer = QMessageBox.question(
                self,
                "자동추종 시작 확인",
                "사용자가 ToF 제어범위에 있고 주변이 안전한지 확인했습니까?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.worker.request_tracking(not self.tracking)

    def _on_tracking(self, enabled: bool):
        self.tracking = bool(enabled)
        self.tracking_button.setText(
            "ToF 자동추종 정지" if enabled else "ToF 자동추종 시작"
        )
        self.tracking_button.setStyleSheet(
            "background:#dc2626;color:white;font-weight:700;" if enabled else ""
        )
        self._refresh_controls()

    def _record_static(self):
        self.worker.request_static_sample(
            {
                "nominal_user_x_m": self.nominal_user_x.value() / 100.0,
                "external_actual_distance_m": self.external_distance.value() / 100.0,
                "approach_direction": self.approach_direction.currentText(),
            }
        )

    def _on_dynamic_activity(self, active: bool):
        self.dynamic_active = bool(active)
        self._refresh_controls()

    def _on_dynamic_finished(self, result: dict[str, Any]):
        if not result.get("success"):
            QMessageBox.warning(self, "동적 시험", str(result.get("message")))

    def _start_safety(self):
        display = self.safety_combo.currentText()
        scenario = self.SAFETY_OPTIONS[display]
        answer = QMessageBox.question(
            self,
            "안전 시험 확인",
            f"'{display}' 시험을 시작합니다. 모니터를 지지하고 주변 안전을 확인했습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.worker.request_safety_start(scenario)

    def _on_safety_activity(self, active: bool):
        self.safety_active = bool(active)
        self._refresh_controls()

    def _on_safety_finished(self, result: dict[str, Any]):
        if not result.get("success"):
            QMessageBox.warning(self, "안전 시험", str(result.get("message")))

    def _update_live(self, state: dict[str, Any]):
        if not state.get("connected"):
            self.live_text.setPlainText("하드웨어 연결 대기")
            return
        def fmt(value, digits=3, suffix=""):
            try:
                return f"{float(value):.{digits}f}{suffix}"
            except (TypeError, ValueError):
                return "--"
        self.live_text.setPlainText(
            "\n".join(
                [
                    f"Tracking: {bool(state.get('tracking_enabled'))}",
                    f"ToF filtered: {fmt(state.get('tof_filtered_distance_m'), 3, ' m')}",
                    f"User X raw/control: {fmt(state.get('raw_user_x_m'))} / {fmt(state.get('control_user_x_m'))} m",
                    f"Motor S/E: {fmt(state.get('shoulder_lift_deg'), 2, '°')} / {fmt(state.get('elbow_flex_deg'), 2, '°')}",
                    f"Monitor X/Z(FK): {fmt(state.get('monitor_x_m'))} / {fmt(state.get('monitor_z_m'))} m",
                    f"Estimated user-monitor: {fmt(state.get('estimated_user_monitor_distance_m'))} m",
                    f"Distance error: {fmt(state.get('distance_error_m'), 3, ' m')}",
                    f"Motor hold: {state.get('motor12_hold_reason')}",
                    f"Safety: {state.get('safety_state')} / {state.get('safety_reason')}",
                    f"Sim landmark/posture: {bool(state.get('simulated_landmark_valid'))} / {state.get('simulated_posture')}",
                    f"Motion: {state.get('motion_kind') or '-'}",
                ]
            )
        )

    def _on_fatal_error(self, message: str):
        if not self.closing:
            QMessageBox.critical(self, "모니터암 평가 오류", message)

    def closeEvent(self, event: QCloseEvent):
        if self.connected:
            answer = QMessageBox.question(
                self,
                "평가 도구 종료",
                "종료하면 자동추종과 모터 통신을 정지하지만 현재 자세에서 자동으로 휴식자세로 이동하지 않습니다.\n"
                "필요하면 취소 후 '작업 → 휴식자세'를 먼저 실행하세요. 그대로 종료할까요?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self.closing = True
        if self.worker.isRunning():
            self.worker.blockSignals(True)
            self.worker.request_shutdown()
            if not self.worker.wait(8000):
                self.worker.blockSignals(False)
                self.closing = False
                QMessageBox.warning(
                    self, "종료 지연", "하드웨어 Worker가 아직 종료되지 않았습니다. 다시 시도해주세요."
                )
                event.ignore()
                return
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(
        description="POCO standalone Motor1/2 monitor-arm performance evaluator"
    )
    parser.add_argument(
        "--fixed-tof",
        type=float,
        default=None,
        metavar="METRES",
        help="Use FixedToF stub instead of physical ToF. Motors remain physical.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    app = QApplication(sys.argv[:1])
    window = MonitorArmEvaluationWindow(fixed_tof_m=args.fixed_tof)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

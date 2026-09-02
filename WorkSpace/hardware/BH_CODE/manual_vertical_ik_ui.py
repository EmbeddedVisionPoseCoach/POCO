#!/usr/bin/env python3
"""Manual Cartesian UI for constant user-to-monitor distance.

Coordinate convention
---------------------
origin : clamp-to-robot-base connection
+X     : from the robot base toward the user
+Z     : vertically upward from the ground/base origin

The gauge represents the user's X coordinate. The monitor target is always:

    monitor_x = user_x - user_monitor_distance
    monitor_z = operator_entered_constant_height

Only shoulder_lift (servo 1) and elbow_flex (servo 2) are solved and sent.
Changing the user-X gauge starts real-time target tracking. Distance and height
edits remain pending until the operator explicitly applies those constants.
"""

from __future__ import annotations

import math
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from manual_motor12_limit_ui import ManualMotor12Bus
from monitor_arm_speed import (
    ABSOLUTE_SPEED_CAP,
    ADAPTIVE_SPEED_MODE as ADAPTIVE_SPEED_KEY,
    FIXED_SPEED_MODE as FIXED_SPEED_KEY,
    calculate_adaptive_speed,
)
from monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    KinematicsError,
    MonitorPose,
    TwoJointMonitorArm,
    load_settings,
    monitor_target_from_user,
    save_settings,
)


ROOT_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = ROOT_DIR / "servo_calibration_result.json"
SETTINGS_PATH = ROOT_DIR / "monitor_arm_settings.json"
JOINTS = ("shoulder_lift", "elbow_flex")
REST_POSE = JointCommand(shoulder_lift_deg=107.75, elbow_flex_deg=-92.55)
REST_SPEED_CAP = 200
FIXED_SPEED_MODE = "고정 속도"
ADAPTIVE_SPEED_MODE = "거리 비례 가변 속도"


class CartesianIKWindow:
    CANVAS_WIDTH = 650
    CANVAS_HEIGHT = 520

    def __init__(self, root: tk.Tk):
        self.root = root
        self.settings = load_settings(SETTINGS_PATH)
        self.geometry = ArmGeometry.from_settings(self.settings)
        self.kinematics = TwoJointMonitorArm(self.geometry)
        self.bus = ManualMotor12Bus(CALIBRATION_PATH)
        self.closing = False

        self.current = JointCommand(0.0, 0.0)
        self.current_pose = self.kinematics.forward(self.current)
        self.target: JointCommand | None = None
        self.target_pose: MonitorPose | None = None
        self.realtime_after_id: str | None = None
        self.realtime_active = False

        distance = self.settings["distance"]
        cartesian = self.settings.get("manual_cartesian", {})
        control = self.settings["control"]

        default_distance_m = float(distance["desired_user_monitor_distance_m"])
        default_z_m = float(
            cartesian.get("default_monitor_z_m", self.current_pose.z_m)
        )
        default_user_x_m = self.current_pose.x_m + default_distance_m

        self.user_x_min_cm_var = tk.DoubleVar(
            value=float(cartesian.get("user_x_min_m", 0.6007655)) * 100.0
        )
        self.user_x_max_cm_var = tk.DoubleVar(
            value=float(cartesian.get("user_x_max_m", 0.8307655)) * 100.0
        )
        self.monitor_z_min_cm_var = tk.DoubleVar(
            value=float(cartesian.get("monitor_z_min_m", 0.20)) * 100.0
        )
        self.monitor_z_max_cm_var = tk.DoubleVar(
            value=float(cartesian.get("monitor_z_max_m", 0.30)) * 100.0
        )

        default_user_x_cm = self._clamp(
            default_user_x_m * 100.0,
            self.user_x_min_cm_var.get(),
            self.user_x_max_cm_var.get(),
        )
        default_z_cm = self._clamp(
            default_z_m * 100.0,
            self.monitor_z_min_cm_var.get(),
            self.monitor_z_max_cm_var.get(),
        )

        self.user_x_cm_var = tk.DoubleVar(value=default_user_x_cm)
        self.distance_cm_var = tk.DoubleVar(value=default_distance_m * 100.0)
        self.monitor_z_cm_var = tk.DoubleVar(value=default_z_cm)
        self.applied_distance_cm = default_distance_m * 100.0
        self.applied_monitor_z_cm = default_z_cm
        self.speed_var = tk.IntVar(
            value=int(control.get("vertical_ik_speed", control["speed"]))
        )
        self.acc_var = tk.IntVar(
            value=int(control.get("vertical_ik_acc", control["acc"]))
        )
        configured_mode = str(control.get("vertical_ik_speed_mode", "fixed"))
        self.speed_mode_var = tk.StringVar(
            value=(
                ADAPTIVE_SPEED_MODE
                if configured_mode == "adaptive"
                else FIXED_SPEED_MODE
            )
        )
        self.variable_min_speed_var = tk.IntVar(
            value=int(control.get("vertical_ik_variable_min_speed", 10))
        )
        self.variable_full_speed_error_deg_var = tk.DoubleVar(
            value=float(
                control.get("vertical_ik_variable_full_speed_error_deg", 30.0)
            )
        )
        self.control_interval_ms = max(
            20,
            round(1000.0 / max(float(control.get("command_hz", 5.0)), 1.0)),
        )
        self.arrival_tolerance_deg = 0.25

        self.coordinate_text = tk.StringVar(value="좌표 계산 대기")
        self.current_text = tk.StringVar(value="현재 자세: 연결 전")
        self.ik_text = tk.StringVar(value="IK 목표: -")
        self.constant_status_text = tk.StringVar()
        self.status_text = tk.StringVar(
            value="X 게이지는 연결 후 실시간 제어됩니다. 먼저 포트를 연결하세요."
        )
        self.update_constant_status()

        self.root.title("POCO 모니터암 — 사용자 X / 고정거리 / 고정높이 IK")
        self.root.geometry("1180x850")
        self.root.minsize(1060, 760)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._build_ui()
        self.distance_cm_var.trace_add(
            "write", lambda *_args: self.mark_constants_pending()
        )
        self.monitor_z_cm_var.trace_add(
            "write", lambda *_args: self.mark_constants_pending()
        )
        self.preview_target()
        self.root.after(350, self.refresh_current_state)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(float(value), maximum))

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=2)
        outer.columnconfigure(1, weight=3)
        outer.rowconfigure(0, weight=1)

        controls = ttk.Frame(outer)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        preview = ttk.LabelFrame(outer, text="X-Z 좌표와 현재/목표 자세", padding=8)
        preview.grid(row=0, column=1, sticky="nsew")

        connection = ttk.LabelFrame(controls, text="1. 모터 연결", padding=10)
        connection.pack(fill="x", pady=(0, 9))
        ttk.Button(connection, text="Servo 1·2 포트 연결", command=self.connect).pack(
            side="left"
        )
        ttk.Button(connection, text="현재 위치 Hold", command=self.hold).pack(
            side="left", padx=6
        )
        ttk.Button(connection, text="1·2 토크 OFF", command=self.torque_off).pack(
            side="left"
        )

        x_box = ttk.LabelFrame(
            controls,
            text="2. 베이스 ToF → 사용자 몸통 X 모의값",
            padding=10,
        )
        x_box.pack(fill="x", pady=(0, 9))
        ttk.Label(
            x_box,
            text=(
                "원점은 클램프-로봇팔 연결점입니다. 실제 ToF 연결 전까지 이 게이지가 "
                "명치 부근을 향한 센서의 사용자 X 입력을 대신합니다."
            ),
            wraplength=430,
        ).pack(anchor="w")
        self.user_x_scale = ttk.Scale(
            x_box,
            from_=self.user_x_min_cm_var.get(),
            to=self.user_x_max_cm_var.get(),
            variable=self.user_x_cm_var,
            command=lambda _value: self.on_user_x_changed(),
        )
        self.user_x_scale.pack(fill="x", pady=(10, 4))
        x_exact = ttk.Frame(x_box)
        x_exact.pack(fill="x")
        ttk.Label(x_exact, text="ToF 사용자 X").pack(side="left")
        user_x_spin = ttk.Spinbox(
            x_exact,
            from_=self.user_x_min_cm_var.get(),
            to=self.user_x_max_cm_var.get(),
            increment=0.1,
            width=9,
            textvariable=self.user_x_cm_var,
            command=self.on_user_x_changed,
        )
        user_x_spin.pack(side="left", padx=5)
        user_x_spin.bind("<Return>", lambda _event: self.on_user_x_changed())
        ttk.Label(x_exact, text="cm").pack(side="left")
        ttk.Button(
            x_exact,
            text="현재 모니터 위치에서 사용자 X 역산",
            command=self.use_current_coordinates,
        ).pack(side="right")

        constants = ttk.LabelFrame(
            controls,
            text="3. 일정하게 유지할 값",
            padding=10,
        )
        constants.pack(fill="x", pady=(0, 9))

        distance_row = ttk.Frame(constants)
        distance_row.pack(fill="x", pady=3)
        ttk.Label(distance_row, text="사용자 ↔ 모니터 X 거리", width=28).pack(side="left")
        distance_spin = ttk.Spinbox(
            distance_row,
            from_=20.0,
            to=100.0,
            increment=0.5,
            width=9,
            textvariable=self.distance_cm_var,
            command=self.mark_constants_pending,
        )
        distance_spin.pack(side="left", padx=5)
        distance_spin.bind("<Return>", lambda _event: self.mark_constants_pending())
        ttk.Label(distance_row, text="cm (X 이동 중 항상 유지)").pack(side="left")

        z_row = ttk.Frame(constants)
        z_row.pack(fill="x", pady=3)
        ttk.Label(z_row, text="지면/베이스 원점 → 모니터 Z", width=28).pack(side="left")
        z_spin = ttk.Spinbox(
            z_row,
            from_=self.monitor_z_min_cm_var.get(),
            to=self.monitor_z_max_cm_var.get(),
            increment=0.1,
            width=9,
            textvariable=self.monitor_z_cm_var,
            command=self.mark_constants_pending,
        )
        z_spin.pack(side="left", padx=5)
        z_spin.bind("<Return>", lambda _event: self.mark_constants_pending())
        ttk.Label(z_row, text="cm (X 이동 중 고정)").pack(side="left")
        self.monitor_z_spin = z_spin
        ttk.Label(
            constants,
            textvariable=self.constant_status_text,
            foreground="#775500",
            wraplength=430,
        ).pack(anchor="w", pady=(6, 3))
        ttk.Button(
            constants,
            text="일정값 적용 및 현재 게이지 위치로 이동",
            command=self.apply_constants_and_move,
        ).pack(fill="x", pady=(3, 0))

        ranges = ttk.LabelFrame(controls, text="4. 수동 좌표 한계", padding=10)
        ranges.pack(fill="x", pady=(0, 9))
        self._range_row(
            ranges,
            "사용자 X 범위",
            self.user_x_min_cm_var,
            self.user_x_max_cm_var,
        )
        self._range_row(
            ranges,
            "모니터 Z 범위",
            self.monitor_z_min_cm_var,
            self.monitor_z_max_cm_var,
        )
        ttk.Button(
            ranges,
            text="좌표 한계 저장 및 적용 (모터 명령 없음)",
            command=self.save_and_apply_ranges,
        ).pack(fill="x", pady=(7, 0))

        command = ttk.LabelFrame(controls, text="5. 계산된 자세 명령", padding=10)
        command.pack(fill="x", pady=(0, 9))
        mode_row = ttk.Frame(command)
        mode_row.pack(fill="x", pady=(0, 5))
        ttk.Label(mode_row, text="속도 모드").pack(side="left")
        ttk.Combobox(
            mode_row,
            state="readonly",
            width=19,
            values=(FIXED_SPEED_MODE, ADAPTIVE_SPEED_MODE),
            textvariable=self.speed_mode_var,
        ).pack(side="left", padx=5)
        speed_row = ttk.Frame(command)
        speed_row.pack(fill="x")
        ttk.Label(speed_row, text="고정/최대 Speed").pack(side="left")
        ttk.Spinbox(
            speed_row,
            from_=1,
            to=ABSOLUTE_SPEED_CAP,
            width=7,
            textvariable=self.speed_var,
        ).pack(side="left", padx=5)
        ttk.Label(speed_row, text="Acc").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            speed_row,
            from_=0,
            to=30,
            width=7,
            textvariable=self.acc_var,
        ).pack(side="left", padx=5)
        variable_row = ttk.Frame(command)
        variable_row.pack(fill="x", pady=(5, 0))
        ttk.Label(variable_row, text="가변 최소 Speed").pack(side="left")
        ttk.Spinbox(
            variable_row,
            from_=1,
            to=ABSOLUTE_SPEED_CAP,
            width=7,
            textvariable=self.variable_min_speed_var,
        ).pack(side="left", padx=5)
        ttk.Label(variable_row, text="최대속도 도달 오차").pack(
            side="left", padx=(10, 2)
        )
        ttk.Spinbox(
            variable_row,
            from_=1.0,
            to=180.0,
            increment=1.0,
            width=7,
            textvariable=self.variable_full_speed_error_deg_var,
        ).pack(side="left", padx=5)
        ttk.Label(
            command,
            text=(
                "가변 모드: 현재-목표 관절각 오차가 클수록 최대 Speed에 가까워지고, "
                "목표 근처에서는 최소 Speed로 감속합니다."
            ),
            wraplength=430,
        ).pack(anchor="w", pady=(5, 0))
        ttk.Button(
            command,
            text="속도 모드/값 저장 (모터 명령 없음)",
            command=self.save_speed_settings,
        ).pack(fill="x", pady=(6, 0))
        ttk.Button(
            command,
            text="현재 X 게이지 목표 다시 추종",
            command=self.request_realtime_control,
        ).pack(fill="x", pady=(9, 0))
        zero_row = ttk.Frame(command)
        zero_row.pack(fill="x", pady=(7, 0))
        ttk.Button(
            zero_row,
            text="ID 1 원점 (0°)",
            command=lambda: self.move_zero("shoulder_lift"),
        ).pack(side="left", fill="x", expand=True)
        ttk.Button(
            zero_row,
            text="ID 2 원점 (0°)",
            command=lambda: self.move_zero("elbow_flex"),
        ).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(
            zero_row,
            text="ID 1·2 동시 원점",
            command=lambda: self.move_zero(None),
        ).pack(side="left", fill="x", expand=True)
        ttk.Button(
            command,
            text="휴식자세로 이동 (S +107.75° / E -92.55°)",
            command=self.move_rest_pose,
        ).pack(fill="x", pady=(7, 0))

        information = ttk.LabelFrame(controls, text="계산 결과", padding=10)
        information.pack(fill="both", expand=True)
        ttk.Label(information, textvariable=self.coordinate_text, wraplength=435).pack(
            anchor="w", pady=2
        )
        ttk.Label(information, textvariable=self.current_text, wraplength=435).pack(
            anchor="w", pady=2
        )
        ttk.Label(information, textvariable=self.ik_text, wraplength=435).pack(
            anchor="w", pady=2
        )
        ttk.Separator(information).pack(fill="x", pady=7)
        ttk.Label(
            information,
            textvariable=self.status_text,
            foreground="#a33",
            wraplength=435,
        ).pack(anchor="w")

        self.canvas = tk.Canvas(
            preview,
            width=self.CANVAS_WIDTH,
            height=self.CANVAS_HEIGHT,
            background="#111820",
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)
        ttk.Label(
            preview,
            text=(
                "회색: 현재 팔   파랑: 목표 팔   주황: 목표 모니터   초록: 사용자\n"
                "노랑: 일정하게 유지할 사용자-모니터 X 거리"
            ),
            justify="center",
        ).pack(pady=(6, 0))

    @staticmethod
    def _range_row(parent, label: str, minimum_var, maximum_var) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, width=20).pack(side="left")
        ttk.Spinbox(
            row,
            from_=0.0,
            to=200.0,
            increment=0.5,
            width=8,
            textvariable=minimum_var,
        ).pack(side="left", padx=4)
        ttk.Label(row, text="~").pack(side="left")
        ttk.Spinbox(
            row,
            from_=0.0,
            to=200.0,
            increment=0.5,
            width=8,
            textvariable=maximum_var,
        ).pack(side="left", padx=4)
        ttk.Label(row, text="cm").pack(side="left")

    def cartesian_limits_cm(self) -> tuple[float, float, float, float]:
        user_min = float(self.user_x_min_cm_var.get())
        user_max = float(self.user_x_max_cm_var.get())
        z_min = float(self.monitor_z_min_cm_var.get())
        z_max = float(self.monitor_z_max_cm_var.get())
        if not 0.0 <= user_min < user_max:
            raise ValueError("사용자 X 범위는 0 ≤ min < max여야 합니다.")
        if not 0.0 <= z_min < z_max:
            raise ValueError("모니터 Z 범위는 0 ≤ min < max여야 합니다.")
        return user_min, user_max, z_min, z_max

    def update_constant_status(self, pending: bool = False) -> None:
        applied = (
            f"적용 중: 거리 {self.applied_distance_cm:.1f}cm / "
            f"모니터 Z {self.applied_monitor_z_cm:.1f}cm"
        )
        if pending:
            applied += "  —  입력값은 아직 미적용"
        self.constant_status_text.set(applied)

    def mark_constants_pending(self) -> None:
        """Do not let a spinbox edit alter the active IK constants."""
        self.update_constant_status(pending=True)
        self.status_text.set(
            "고정거리/높이 입력만 변경됐습니다. "
            "'일정값 적용'을 누르기 전까지 모터와 IK에 반영되지 않습니다."
        )

    def requested_coordinates(self) -> tuple[float, float, float, MonitorPose]:
        user_min, user_max, z_min, z_max = self.cartesian_limits_cm()
        user_x_cm = float(self.user_x_cm_var.get())
        distance_cm = self.applied_distance_cm
        monitor_z_cm = self.applied_monitor_z_cm
        if not user_min <= user_x_cm <= user_max:
            raise ValueError(f"사용자 X 허용범위: {user_min:.1f}~{user_max:.1f}cm")
        if not z_min <= monitor_z_cm <= z_max:
            raise ValueError(f"모니터 Z 허용범위: {z_min:.1f}~{z_max:.1f}cm")
        target_pose = monitor_target_from_user(
            user_x_m=user_x_cm / 100.0,
            user_monitor_distance_m=distance_cm / 100.0,
            monitor_z_m=monitor_z_cm / 100.0,
        )
        return user_x_cm, distance_cm, monitor_z_cm, target_pose

    def hard_ranges(self) -> dict[str, tuple[float, float]]:
        return {
            joint: self.bus.calibration.get_safe_angle_range(joint)
            for joint in JOINTS
        }

    def validate_joint_limits(self, target: JointCommand) -> None:
        soft = self.settings["safety"]["soft_joint_limits_deg"]
        requested = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        hard = self.hard_ranges()
        for joint, angle in requested.items():
            hard_min, hard_max = hard[joint]
            minimum = max(float(soft[joint]["min"]), hard_min)
            maximum = min(float(soft[joint]["max"]), hard_max)
            if not minimum <= angle <= maximum:
                raise KinematicsError(
                    f"{joint} IK={angle:+.2f}°가 joint 제한 "
                    f"{minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                )

    @staticmethod
    def _outside_distance(value: float, minimum: float, maximum: float) -> float:
        if value < minimum:
            return minimum - value
        if value > maximum:
            return value - maximum
        return 0.0

    def validate_recovery_step_limits(
        self,
        current: JointCommand,
        target: JointCommand,
    ) -> bool:
        """Allow an out-of-soft-range step only when it moves strictly inward.

        The final IK target is still checked by validate_joint_limits().  This
        exception only prevents a physical rest pose outside the configured
        soft/calibration range from becoming a state that cannot move back in.
        """
        soft = self.settings["safety"]["soft_joint_limits_deg"]
        hard = self.hard_ranges()
        current_angles = {
            "shoulder_lift": current.shoulder_lift_deg,
            "elbow_flex": current.elbow_flex_deg,
        }
        target_angles = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        recovering = False
        for joint in JOINTS:
            minimum = max(float(soft[joint]["min"]), hard[joint][0])
            maximum = min(float(soft[joint]["max"]), hard[joint][1])
            current_outside = self._outside_distance(
                current_angles[joint], minimum, maximum
            )
            target_outside = self._outside_distance(
                target_angles[joint], minimum, maximum
            )
            if current_outside <= 1e-6:
                if target_outside > 1e-6:
                    raise KinematicsError(
                        f"{joint} 스텝 {target_angles[joint]:+.2f}°가 안전범위 "
                        f"{minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                    )
            else:
                recovering = True
                if target_outside >= current_outside - 1e-6:
                    raise KinematicsError(
                        f"{joint}이 안전범위 밖에서 더 멀어지는 명령을 차단했습니다. "
                        f"현재 이탈={current_outside:.2f}°, "
                        f"다음 이탈={target_outside:.2f}°"
                    )
        return recovering

    def validate_interpolated_path(self, target: JointCommand) -> bool:
        """Validate a normal Z path or a bounded recovery from rest Z.

        ``monitor_z_min/max`` define valid working heights.  A folded rest pose
        can legitimately begin below that interval, so rejecting sample zero
        would make recovery impossible.  While outside, only a path that stays
        within one vertical_tolerance of the current violation is accepted.
        """
        _user_min, _user_max, z_min_cm, z_max_cm = self.cartesian_limits_cm()
        samples = max(2, int(self.settings["safety"]["path_samples"]))
        target_pose = self.kinematics.forward(target)
        tolerance_m = float(self.settings["safety"]["vertical_tolerance_m"])
        z_min_m = z_min_cm / 100.0
        z_max_m = z_max_cm / 100.0
        current_violation_m = self._outside_distance(
            self.current_pose.z_m, z_min_m, z_max_m
        )
        recovering = current_violation_m > 1e-4
        for index in range(samples + 1):
            ratio = index / samples
            sample = self.current.interpolate(target, ratio)
            pose = self.kinematics.forward(sample)
            z_cm = pose.z_m * 100.0
            sample_violation_m = self._outside_distance(
                pose.z_m, z_min_m, z_max_m
            )
            if recovering:
                if sample_violation_m > current_violation_m + tolerance_m:
                    raise KinematicsError(
                        f"휴식자세 복구 중 Z={z_cm:.1f}cm가 현재 안전범위 이탈보다 "
                        f"{tolerance_m * 100:.1f}cm 이상 더 벗어납니다."
                    )
            elif sample_violation_m > 1e-4:
                raise KinematicsError(
                    f"이동 중간경로 Z={z_cm:.1f}cm가 설정범위 "
                    f"{z_min_cm:.1f}~{z_max_cm:.1f}cm 밖입니다."
                )
            expected_z_m = self.current_pose.z_m + (
                target_pose.z_m - self.current_pose.z_m
            ) * ratio
            vertical_error_m = abs(pose.z_m - expected_z_m)
            if vertical_error_m > tolerance_m:
                raise KinematicsError(
                    f"이동 중 예상 Z 편차 {vertical_error_m * 100:.1f}cm가 "
                    f"vertical_tolerance {tolerance_m * 100:.1f}cm를 초과합니다."
                )
        return recovering

    def motion_parameters(self, largest_delta_deg: float | None = None) -> tuple[int, int]:
        selected_speed = int(self.speed_var.get())
        acc = int(self.acc_var.get())
        if not 1 <= selected_speed <= ABSOLUTE_SPEED_CAP:
            raise ValueError(
                f"고정/최대 Speed 허용범위는 1~{ABSOLUTE_SPEED_CAP}입니다."
            )
        if not 0 <= acc <= 30:
            raise ValueError("Acc 허용범위는 0~30입니다.")

        mode = self.speed_mode_var.get()
        if mode == FIXED_SPEED_MODE:
            return selected_speed, acc
        if mode != ADAPTIVE_SPEED_MODE:
            raise ValueError(f"알 수 없는 속도 모드입니다: {mode}")

        minimum_speed = int(self.variable_min_speed_var.get())
        full_speed_error_deg = float(self.variable_full_speed_error_deg_var.get())
        if not 1 <= minimum_speed <= selected_speed:
            raise ValueError(
                "가변 최소 Speed는 1 이상이고 고정/최대 Speed 이하여야 합니다."
            )
        if not 1.0 <= full_speed_error_deg <= 180.0:
            raise ValueError("최대속도 도달 오차는 1~180° 범위여야 합니다.")

        speed = calculate_adaptive_speed(
            minimum_speed,
            selected_speed,
            float(largest_delta_deg or 0.0),
            full_speed_error_deg,
        )
        return speed, acc

    def _store_speed_settings(self) -> None:
        """Copy this UI's speed controls into dedicated persisted keys."""
        self.motion_parameters()
        control = self.settings["control"]
        control["vertical_ik_speed"] = int(self.speed_var.get())
        control["vertical_ik_acc"] = int(self.acc_var.get())
        control["vertical_ik_speed_mode"] = (
            ADAPTIVE_SPEED_KEY
            if self.speed_mode_var.get() == ADAPTIVE_SPEED_MODE
            else FIXED_SPEED_KEY
        )
        control["vertical_ik_variable_min_speed"] = int(
            self.variable_min_speed_var.get()
        )
        control["vertical_ik_variable_full_speed_error_deg"] = float(
            self.variable_full_speed_error_deg_var.get()
        )

    def save_speed_settings(self) -> None:
        try:
            self._store_speed_settings()
            save_settings(self.settings, SETTINGS_PATH)
            self.status_text.set(
                "manual_vertical_ik_ui 전용 속도 모드/값을 저장했습니다. "
                "모터 명령은 보내지 않았습니다."
            )
        except Exception as error:
            messagebox.showerror("속도 설정 저장 실패", str(error))

    def connect(self) -> None:
        try:
            self.bus.open()
            self.current = self.bus.read_angles()
            self.current_pose = self.kinematics.forward(self.current)
            self.use_current_coordinates()
            self.status_text.set(
                "연결 완료. 현재 모니터 X에서 사용자 X를 역산했습니다. "
                "게이지를 움직이면 실시간 추종을 시작합니다."
            )
        except Exception as error:
            messagebox.showerror("연결 실패", str(error))

    def use_current_coordinates(self) -> None:
        distance_m = self.applied_distance_cm / 100.0
        user_min, user_max, _z_min, _z_max = self.cartesian_limits_cm()
        user_x_cm = (self.current_pose.x_m + distance_m) * 100.0
        self.user_x_cm_var.set(self._clamp(user_x_cm, user_min, user_max))
        self.preview_target()

    def on_user_x_changed(self) -> None:
        """Preview immediately and track the new gauge target when connected."""
        self.preview_target()
        self.request_realtime_control(show_warning=False)

    def preview_target(self) -> None:
        try:
            user_x_cm, distance_cm, monitor_z_cm, requested_pose = (
                self.requested_coordinates()
            )
            target = self.kinematics.inverse(requested_pose.x_m, requested_pose.z_m)
            self.validate_joint_limits(target)
            solved_pose = self.kinematics.forward(target)

            self.target = target
            self.target_pose = solved_pose
            self.coordinate_text.set(
                f"사용자 X={user_x_cm:.1f}cm / 고정거리={distance_cm:.1f}cm\n"
                f"→ 모니터 목표 X={requested_pose.x_m * 100:.1f}cm, "
                f"고정 Z={monitor_z_cm:.1f}cm"
            )
            self.ik_text.set(
                f"IK — shoulder_lift={target.shoulder_lift_deg:+.2f}°, "
                f"elbow_flex={target.elbow_flex_deg:+.2f}°"
            )
            self.status_text.set(
                "적용 중인 고정거리/높이로 IK 및 joint limit 검사 통과"
            )
        except (KinematicsError, ValueError) as error:
            self.target = None
            self.target_pose = None
            self.coordinate_text.set("좌표 계산 불가")
            self.ik_text.set("IK 목표: 전송 불가")
            self.status_text.set(f"전송 차단: {error}")
        self.draw_scene()

    def apply_constants_and_move(self) -> None:
        """Commit pending distance/Z values, save them, then track once."""
        try:
            user_min, user_max, z_min, z_max = self.cartesian_limits_cm()
            user_x_cm = float(self.user_x_cm_var.get())
            pending_distance_cm = float(self.distance_cm_var.get())
            pending_z_cm = float(self.monitor_z_cm_var.get())
            if not user_min <= user_x_cm <= user_max:
                raise ValueError(
                    f"사용자 X는 {user_min:.1f}~{user_max:.1f}cm 범위여야 합니다."
                )
            if pending_distance_cm <= 0.0 or pending_distance_cm >= user_min:
                raise ValueError("고정거리는 0보다 크고 사용자 X 최소값보다 작아야 합니다.")
            if not z_min <= pending_z_cm <= z_max:
                raise ValueError(
                    f"모니터 Z는 {z_min:.1f}~{z_max:.1f}cm 범위여야 합니다."
                )

            requested_pose = monitor_target_from_user(
                user_x_m=user_x_cm / 100.0,
                user_monitor_distance_m=pending_distance_cm / 100.0,
                monitor_z_m=pending_z_cm / 100.0,
            )
            pending_target = self.kinematics.inverse(
                requested_pose.x_m, requested_pose.z_m
            )
            self.validate_joint_limits(pending_target)

            self.applied_distance_cm = pending_distance_cm
            self.applied_monitor_z_cm = pending_z_cm
            self.settings["distance"]["desired_user_monitor_distance_m"] = (
                pending_distance_cm / 100.0
            )
            self.settings.setdefault("manual_cartesian", {})[
                "default_monitor_z_m"
            ] = pending_z_cm / 100.0
            save_settings(self.settings, SETTINGS_PATH)
            self.update_constant_status(pending=False)
            self.preview_target()
            self.request_realtime_control(show_warning=True)
        except Exception as error:
            messagebox.showerror("일정값 적용 실패", str(error))

    def request_realtime_control(self, show_warning: bool = True) -> None:
        """Start or retarget the stepped IK tracking loop."""
        self.preview_target()
        if self.target is None:
            if show_warning:
                messagebox.showwarning("제어 불가", "현재 좌표에 안전한 IK 해가 없습니다.")
            return
        if self.bus.driver is None:
            if show_warning:
                messagebox.showwarning("연결 필요", "먼저 Servo 1·2 포트를 연결하세요.")
            else:
                self.status_text.set(
                    "X 게이지 목표는 갱신됐지만 포트 연결 전이라 모터 명령은 없습니다."
                )
            return

        self.realtime_active = True
        if self.realtime_after_id is None:
            self.realtime_after_id = self.root.after(0, self.realtime_control_tick)

    def realtime_control_tick(self) -> None:
        """Approach the latest gauge target in max_joint_step-sized commands."""
        self.realtime_after_id = None
        if self.closing or not self.realtime_active or self.bus.driver is None:
            return

        try:
            self.current = self.bus.read_angles()
            self.current_pose = self.kinematics.forward(self.current)
            self.preview_target()
            if self.target is None:
                self.realtime_active = False
                return

            self.validate_joint_limits(self.target)
            largest_delta = max(
                abs(self.target.shoulder_lift_deg - self.current.shoulder_lift_deg),
                abs(self.target.elbow_flex_deg - self.current.elbow_flex_deg),
            )
            if largest_delta <= self.arrival_tolerance_deg:
                self.realtime_active = False
                self.status_text.set(
                    f"실시간 IK 목표 도달 (최대 각도 오차 {largest_delta:.2f}°)"
                )
                return

            configured_step = max(
                0.1, float(self.settings["safety"]["max_joint_step_deg"])
            )
            ratio = min(1.0, configured_step / largest_delta)
            step_target = self.current.interpolate(self.target, ratio)
            recovering_joint = self.validate_recovery_step_limits(
                self.current, step_target
            )
            recovering_z = self.validate_interpolated_path(step_target)

            speed, acc = self.motion_parameters(largest_delta)
            self.bus.move_recovery_aware(self.current, step_target, speed, acc)
            mode_label = (
                "가변"
                if self.speed_mode_var.get() == ADAPTIVE_SPEED_MODE
                else "고정"
            )
            if recovering_joint or recovering_z:
                self.status_text.set(
                    f"휴식자세 → 안전 작업범위 복귀 중 — Speed={speed}({mode_label}), "
                    f"남은 관절각 최대 {largest_delta:.2f}°"
                )
            else:
                self.status_text.set(
                    f"실시간 X 게이지 추종 중 — Speed={speed}({mode_label}), "
                    f"남은 관절각 최대 {largest_delta:.2f}°, "
                    f"이번 명령 스텝 ≤ {configured_step:.2f}°"
                )
        except Exception as error:
            self.realtime_active = False
            self.status_text.set(f"실시간 제어 중지: {error}")
            return

        if self.realtime_active and not self.closing:
            self.realtime_after_id = self.root.after(
                self.control_interval_ms, self.realtime_control_tick
            )

    def cancel_realtime_control(self) -> None:
        self.realtime_active = False
        if self.realtime_after_id is not None:
            self.root.after_cancel(self.realtime_after_id)
            self.realtime_after_id = None

    def save_and_apply_ranges(self) -> None:
        try:
            user_min, user_max, z_min, z_max = self.cartesian_limits_cm()
            if self.applied_distance_cm <= 0.0 or self.applied_distance_cm >= user_min:
                raise ValueError("고정거리는 0보다 크고 사용자 X 최소값보다 작아야 합니다.")
            if not z_min <= self.applied_monitor_z_cm <= z_max:
                raise ValueError("현재 적용 중인 모니터 Z가 새 Z 범위 밖입니다.")

            self.cancel_realtime_control()
            self.settings["distance"]["desired_user_monitor_distance_m"] = (
                self.applied_distance_cm / 100.0
            )
            self.settings["manual_cartesian"] = {
                "user_x_min_m": user_min / 100.0,
                "user_x_max_m": user_max / 100.0,
                "monitor_z_min_m": z_min / 100.0,
                "monitor_z_max_m": z_max / 100.0,
                "default_monitor_z_m": self.applied_monitor_z_cm / 100.0,
            }
            self._store_speed_settings()
            save_settings(self.settings, SETTINGS_PATH)

            self.user_x_scale.configure(from_=user_min, to=user_max)
            self.monitor_z_spin.configure(from_=z_min, to=z_max)
            self.user_x_cm_var.set(
                self._clamp(self.user_x_cm_var.get(), user_min, user_max)
            )
            self.preview_target()
            self.status_text.set(
                "좌표 한계와 이 UI 전용 속도 설정을 저장/적용했습니다. "
                "모터 명령은 보내지 않았습니다."
            )
        except Exception as error:
            messagebox.showerror("설정 저장 실패", str(error))

    def refresh_current_state(self) -> None:
        try:
            if self.bus.driver is not None:
                states = self.bus.read_states()
                self.current = JointCommand(
                    states["shoulder_lift"]["angle"],
                    states["elbow_flex"]["angle"],
                )
                self.current_pose = self.kinematics.forward(self.current)
                self.current_text.set(
                    f"현재 — S={self.current.shoulder_lift_deg:+.2f}°, "
                    f"E={self.current.elbow_flex_deg:+.2f}°\n"
                    f"모니터 X={self.current_pose.x_m * 100:.1f}cm, "
                    f"Z={self.current_pose.z_m * 100:.1f}cm\n"
                    f"Load — S={states['shoulder_lift']['load_percent']}%, "
                    f"E={states['elbow_flex']['load_percent']}%"
                )
                self.draw_scene()
        except Exception as error:
            self.status_text.set(f"상태 읽기 실패: {error}")
        finally:
            if not self.closing:
                self.root.after(350, self.refresh_current_state)

    def hold(self) -> None:
        self.cancel_realtime_control()
        try:
            self.bus.hold()
            self.status_text.set("Servo 1·2 현재 위치 Hold")
        except Exception as error:
            messagebox.showerror("Hold 실패", str(error))

    def torque_off(self) -> None:
        if not messagebox.askyesno(
            "Servo 1·2 토크 OFF",
            "팔과 모니터를 확실히 지지했습니까? Servo 1·2만 토크 해제됩니다.",
        ):
            return
        self.cancel_realtime_control()
        try:
            self.bus.torque_off_1_and_2()
            self.status_text.set("Servo 1·2 Torque OFF")
        except Exception as error:
            messagebox.showerror("Torque OFF 실패", str(error))

    def move_zero(self, joint: str | None) -> None:
        """Move one calibrated command axis, or both axes, to command angle 0°."""
        if self.bus.driver is None:
            messagebox.showwarning("연결 필요", "먼저 Servo 1·2 포트를 연결하세요.")
            return

        self.cancel_realtime_control()
        try:
            current = self.bus.read_angles()
            if joint == "shoulder_lift":
                target = JointCommand(0.0, current.elbow_flex_deg)
                delta = abs(current.shoulder_lift_deg)
                label = "ID 1 shoulder_lift"
            elif joint == "elbow_flex":
                target = JointCommand(current.shoulder_lift_deg, 0.0)
                delta = abs(current.elbow_flex_deg)
                label = "ID 2 elbow_flex"
            elif joint is None:
                target = JointCommand(0.0, 0.0)
                delta = max(
                    abs(current.shoulder_lift_deg), abs(current.elbow_flex_deg)
                )
                label = "ID 1·2"
            else:
                raise ValueError(f"원점 대상 관절 오류: {joint}")

            self.validate_joint_limits(target)
            configured_step = float(self.settings["safety"]["max_joint_step_deg"])
            if delta > configured_step and not messagebox.askyesno(
                "원점 복귀 확인",
                f"{label}이(가) 최대 {delta:.1f}° 원점으로 이동합니다.\n"
                "팔과 모니터를 지지하고 충돌물이 없는지 확인했습니까?",
            ):
                return

            speed, acc = self.motion_parameters(delta)
            if joint is None:
                self.bus.move(target, speed, acc)
            else:
                self.bus.move_joint(joint, 0.0, speed, acc)
            self.status_text.set(
                f"{label}에 calibration 기준 원점(0°) 명령을 보냈습니다 "
                f"(Speed={speed}). "
                "X 게이지 실시간 추종은 중지된 상태입니다."
            )
        except Exception as error:
            messagebox.showerror("원점 복귀 실패", str(error))

    def move_rest_pose(self) -> None:
        """Move IDs 1 and 2 to the explicitly requested folded rest pose."""
        if self.bus.driver is None:
            messagebox.showwarning("연결 필요", "먼저 Servo 1·2 포트를 연결하세요.")
            return

        self.cancel_realtime_control()
        try:
            current = self.bus.read_angles()
            delta = max(
                abs(REST_POSE.shoulder_lift_deg - current.shoulder_lift_deg),
                abs(REST_POSE.elbow_flex_deg - current.elbow_flex_deg),
            )
            shoulder_range = self.hard_ranges()["shoulder_lift"]
            elbow_range = self.hard_ranges()["elbow_flex"]
            if not messagebox.askyesno(
                "휴식자세 이동 확인",
                (
                    "팔과 모니터를 지지하고 베이스 주변 충돌물이 없는지 확인하세요.\n\n"
                    f"목표: shoulder {REST_POSE.shoulder_lift_deg:+.2f}°, "
                    f"elbow {REST_POSE.elbow_flex_deg:+.2f}°\n"
                    f"calibration 안전범위: shoulder {shoulder_range[0]:+.2f}~"
                    f"{shoulder_range[1]:+.2f}°, elbow {elbow_range[0]:+.2f}~"
                    f"{elbow_range[1]:+.2f}°\n\n"
                    "이 휴식자세는 calibration 안전범위 밖의 사용자가 지정한 "
                    "예외 자세입니다. 계속 이동합니까?"
                ),
            ):
                self.status_text.set("휴식자세 이동을 취소했습니다.")
                return

            requested_speed, requested_acc = self.motion_parameters(delta)
            speed = min(requested_speed, REST_SPEED_CAP)
            acc = min(requested_acc, 10)
            self.bus.move_confirmed_rest_pose(REST_POSE, speed, acc)
            self.status_text.set(
                f"휴식자세 명령 전송 — S={REST_POSE.shoulder_lift_deg:+.2f}°, "
                f"E={REST_POSE.elbow_flex_deg:+.2f}°, Speed={speed}, Acc={acc}. "
                "휴식 이동은 충격 방지를 위해 Speed≤200, Acc≤10으로 제한됩니다."
            )
        except Exception as error:
            messagebox.showerror("휴식자세 이동 실패", str(error))

    def world_to_canvas(
        self,
        x_m: float,
        z_m: float,
        scale: float,
    ) -> tuple[float, float]:
        canvas_height = max(self.canvas.winfo_height(), self.CANVAS_HEIGHT)
        origin_x = 55.0
        origin_y = canvas_height - 48.0
        return origin_x + x_m * scale, origin_y - z_m * scale

    def arm_points(self, command: JointCommand) -> list[tuple[float, float]]:
        shoulder_urdf, elbow_urdf = self.kinematics.command_to_urdf(command)
        upper_world = self.geometry.upper_zero_angle_rad - shoulder_urdf
        lower_world = self.geometry.lower_zero_angle_rad - shoulder_urdf - elbow_urdf
        base = (0.0, 0.0)
        shoulder = (self.geometry.shoulder_x_m, self.geometry.shoulder_z_m)
        elbow = (
            shoulder[0] + self.geometry.upper_link_m * math.cos(upper_world),
            shoulder[1] + self.geometry.upper_link_m * math.sin(upper_world),
        )
        monitor = (
            elbow[0] + self.geometry.effective_lower_link_m * math.cos(lower_world),
            elbow[1] + self.geometry.effective_lower_link_m * math.sin(lower_world),
        )
        return [base, shoulder, elbow, monitor]

    def draw_arm(self, command: JointCommand, color: str, width: int, scale: float) -> None:
        points = [self.world_to_canvas(x, z, scale) for x, z in self.arm_points(command)]
        for start, end in zip(points, points[1:]):
            self.canvas.create_line(*start, *end, fill=color, width=width)
        for index, point in enumerate(points):
            radius = 7 if index == len(points) - 1 else 5
            fill = "#ff9f43" if index == len(points) - 1 else color
            self.canvas.create_oval(
                point[0] - radius,
                point[1] - radius,
                point[0] + radius,
                point[1] + radius,
                fill=fill,
                outline="",
            )

    def draw_scene(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), self.CANVAS_WIDTH)
        height = max(self.canvas.winfo_height(), self.CANVAS_HEIGHT)
        try:
            user_min, user_max, _z_min, z_max = self.cartesian_limits_cm()
            user_x_m = max(float(self.user_x_cm_var.get()) / 100.0, user_max / 100.0)
            max_z_m = max(z_max / 100.0, self.geometry.shoulder_z_m, 0.40)
        except (ValueError, tk.TclError):
            user_x_m = 0.84
            max_z_m = 0.40
        scale = min((width - 110.0) / max(user_x_m, 0.2), (height - 95.0) / max_z_m)

        origin = self.world_to_canvas(0.0, 0.0, scale)
        x_end = self.world_to_canvas(max(user_x_m, 0.2), 0.0, scale)
        z_end = self.world_to_canvas(0.0, max_z_m, scale)
        self.canvas.create_line(*origin, *x_end, fill="#8492a0", width=2, arrow=tk.LAST)
        self.canvas.create_line(*origin, *z_end, fill="#8492a0", width=2, arrow=tk.LAST)
        self.canvas.create_text(x_end[0] - 8, x_end[1] - 14, text="+X 사용자 방향", fill="white", anchor="e")
        self.canvas.create_text(z_end[0] + 8, z_end[1], text="+Z 위", fill="white", anchor="w")
        self.canvas.create_text(origin[0] + 5, origin[1] + 15, text="BASE (0,0)", fill="white", anchor="w")

        self.draw_arm(self.current, "#7f8c98", 5, scale)
        if self.target is not None:
            self.draw_arm(self.target, "#4ea5ff", 3, scale)

        try:
            user_x_cm, distance_cm, monitor_z_cm, requested_pose = self.requested_coordinates()
            monitor_point = self.world_to_canvas(requested_pose.x_m, requested_pose.z_m, scale)
            user_point = self.world_to_canvas(user_x_cm / 100.0, monitor_z_cm / 100.0, scale)
            self.canvas.create_line(
                *monitor_point,
                *user_point,
                fill="#ffd166",
                width=3,
                dash=(7, 4),
            )
            self.canvas.create_text(
                (monitor_point[0] + user_point[0]) / 2,
                monitor_point[1] - 16,
                text=f"고정 {distance_cm:.1f}cm",
                fill="#ffd166",
            )
            radius = 10
            self.canvas.create_oval(
                user_point[0] - radius,
                user_point[1] - radius,
                user_point[0] + radius,
                user_point[1] + radius,
                fill="#43c59e",
                outline="",
            )
            self.canvas.create_text(
                user_point[0],
                user_point[1] - 24,
                text=f"사용자 X={user_x_cm:.1f}cm",
                fill="#43c59e",
            )
            self.canvas.create_text(
                monitor_point[0],
                monitor_point[1] + 22,
                text=f"모니터 ({requested_pose.x_m * 100:.1f}, {monitor_z_cm:.1f})cm",
                fill="#ffb36b",
            )
        except (ValueError, tk.TclError):
            pass

    def close(self) -> None:
        self.closing = True
        self.cancel_realtime_control()
        self.bus.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    CartesianIKWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()

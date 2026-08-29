#!/usr/bin/env python3
"""Tkinter X-Z visualizer shared by the automatic pose controller."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import messagebox, ttk

from monitor_arm_kinematics import JointCommand, TwoJointMonitorArm


def calculate_arm_points(
    kinematics: TwoJointMonitorArm,
    command: JointCommand,
) -> list[tuple[float, float]]:
    """Return base, shoulder, elbow and monitor-centre points in X-Z metres."""
    geometry = kinematics.geometry
    shoulder_urdf, elbow_urdf = kinematics.command_to_urdf(command)
    upper_world = geometry.upper_zero_angle_rad - shoulder_urdf
    lower_world = (
        geometry.lower_zero_angle_rad - shoulder_urdf - elbow_urdf
    )
    base = (0.0, 0.0)
    shoulder = (geometry.shoulder_x_m, geometry.shoulder_z_m)
    elbow = (
        shoulder[0] + geometry.upper_link_m * math.cos(upper_world),
        shoulder[1] + geometry.upper_link_m * math.sin(upper_world),
    )
    monitor = (
        elbow[0] + geometry.effective_lower_link_m * math.cos(lower_world),
        elbow[1] + geometry.effective_lower_link_m * math.sin(lower_world),
    )
    return [base, shoulder, elbow, monitor]


class PoseIKVisualizer:
    """Live Tk window for current/target two-joint monitor-arm poses."""

    CANVAS_WIDTH = 720
    CANVAS_HEIGHT = 590

    def __init__(self, kinematics: TwoJointMonitorArm):
        self.kinematics = kinematics
        self.geometry = kinematics.geometry
        self.closed = False
        self.exit_requested = False
        self.rest_requested = False
        self.current = JointCommand(0.0, 0.0)
        self.target: JointCommand | None = None
        self.user_x_m: float | None = None
        self.desired_distance_m = 0.5
        self.reference_z_m = self.kinematics.forward(self.current).z_m

        self.root = tk.Tk()
        self.root.title("POCO Pose IK 실시간 X-Z 시각화")
        self.root.geometry("820x760")
        self.root.minsize(720, 650)
        self.root.protocol("WM_DELETE_WINDOW", self._request_exit)

        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        self.mode_var = tk.StringVar(value="SIMULATION")
        self.distance_var = tk.StringVar(value="ToF 사용자 X: 입력 대기")
        self.current_var = tk.StringVar(value="현재 자세: -")
        self.target_var = tk.StringVar(value="IK 목표: -")
        self.status_var = tk.StringVar(value="ToF/Pose 입력 시작 대기")

        heading = ttk.Frame(outer)
        heading.pack(fill="x")
        ttk.Label(
            heading,
            text="ToF 사용자 X 기반 2축 IK",
            font=("TkDefaultFont", 14, "bold"),
        ).pack(side="left")
        ttk.Label(heading, textvariable=self.mode_var).pack(side="right")
        ttk.Button(
            heading,
            text="휴식자세 요청",
            command=self._request_rest,
        ).pack(side="right", padx=10)

        ttk.Label(outer, textvariable=self.distance_var).pack(anchor="w", pady=(8, 2))
        ttk.Label(outer, textvariable=self.current_var).pack(anchor="w", pady=2)
        ttk.Label(outer, textvariable=self.target_var).pack(anchor="w", pady=2)
        ttk.Label(
            outer,
            textvariable=self.status_var,
            foreground="#a33",
            wraplength=760,
        ).pack(anchor="w", pady=(2, 8))

        self.canvas = tk.Canvas(
            outer,
            width=self.CANVAS_WIDTH,
            height=self.CANVAS_HEIGHT,
            background="#111820",
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)
        ttk.Label(
            outer,
            text=(
                "회색: 현재/명령 전   파랑: 최신 IK 목표   "
                "주황: 모니터   초록: 사용자\n"
                "노랑: 사용자-모니터 거리   청록 점선: 기준 Z"
            ),
            justify="center",
        ).pack(pady=(7, 0))
        self.draw_scene()
        self.pump_events()

    def _request_exit(self) -> None:
        self.exit_requested = True
        self.close()

    def _request_rest(self) -> None:
        if messagebox.askyesno(
            "휴식자세 요청",
            "팔과 모니터를 지지하고 베이스 주변 충돌물이 없는지 확인했습니까?\n"
            "자동 거리 제어를 일시정지하고 휴식자세로 이동합니다.",
        ):
            self.rest_requested = True

    def consume_rest_request(self) -> bool:
        requested = self.rest_requested
        self.rest_requested = False
        return requested

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def pump_events(self) -> bool:
        """Process Tk events without taking control away from the camera loop."""
        if self.closed:
            return not self.exit_requested
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.closed = True
            self.exit_requested = True
        return not self.exit_requested

    def update_state(
        self,
        *,
        current: JointCommand,
        target: JointCommand | None,
        user_x_m: float | None,
        desired_distance_m: float,
        reference_z_m: float,
        status: str,
        mode_text: str,
    ) -> bool:
        if self.closed:
            return not self.exit_requested

        self.current = current
        self.target = target
        self.user_x_m = user_x_m
        self.desired_distance_m = float(desired_distance_m)
        self.reference_z_m = float(reference_z_m)
        self.mode_var.set(mode_text)

        current_pose = self.kinematics.forward(current)
        self.current_var.set(
            f"현재/명령 전 — S={current.shoulder_lift_deg:+.2f}°, "
            f"E={current.elbow_flex_deg:+.2f}° / "
            f"X={current_pose.x_m * 100:.1f}cm, Z={current_pose.z_m * 100:.1f}cm"
        )

        if target is None:
            self.target_var.set("IK 목표: 현재 자세 유지")
        else:
            target_pose = self.kinematics.forward(target)
            self.target_var.set(
                f"IK 목표 — S={target.shoulder_lift_deg:+.2f}°, "
                f"E={target.elbow_flex_deg:+.2f}° / "
                f"X={target_pose.x_m * 100:.1f}cm, Z={target_pose.z_m * 100:.1f}cm"
            )

        if user_x_m is None:
            self.distance_var.set(
                f"ToF 사용자 X: 입력 대기 / "
                f"유지 목표 {self.desired_distance_m * 100:.1f}cm"
            )
        else:
            actual_distance_m = float(user_x_m) - current_pose.x_m
            error_cm = (actual_distance_m - self.desired_distance_m) * 100.0
            self.distance_var.set(
                f"ToF 사용자 X {float(user_x_m) * 100:.1f}cm / "
                f"사용자–모니터 {actual_distance_m * 100:.1f}cm / "
                f"유지 목표 {self.desired_distance_m * 100:.1f}cm / "
                f"오차 {error_cm:+.1f}cm"
            )
        self.status_var.set(status)
        self.draw_scene()
        return self.pump_events()

    def _world_to_canvas(
        self,
        x_m: float,
        z_m: float,
        scale: float,
    ) -> tuple[float, float]:
        canvas_height = max(self.canvas.winfo_height(), self.CANVAS_HEIGHT)
        origin_x = 58.0
        origin_y = canvas_height - 54.0
        return origin_x + x_m * scale, origin_y - z_m * scale

    def _draw_arm(
        self,
        command: JointCommand,
        color: str,
        width: int,
        scale: float,
    ) -> None:
        points = [
            self._world_to_canvas(x_m, z_m, scale)
            for x_m, z_m in calculate_arm_points(self.kinematics, command)
        ]
        for start, end in zip(points, points[1:]):
            self.canvas.create_line(*start, *end, fill=color, width=width)
        for index, point in enumerate(points):
            radius = 8 if index == len(points) - 1 else 5
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
        if self.closed:
            return
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), self.CANVAS_WIDTH)
        height = max(self.canvas.winfo_height(), self.CANVAS_HEIGHT)
        current_pose = self.kinematics.forward(self.current)
        target_pose = (
            self.kinematics.forward(self.target) if self.target is not None else None
        )
        user_x_m = self.user_x_m or 0.0
        max_x_m = max(
            0.62,
            current_pose.x_m,
            target_pose.x_m if target_pose is not None else 0.0,
            user_x_m,
        )
        max_z_m = max(
            0.42,
            self.reference_z_m,
            current_pose.z_m,
            target_pose.z_m if target_pose is not None else 0.0,
        )
        scale = min((width - 120.0) / max_x_m, (height - 105.0) / max_z_m)

        origin = self._world_to_canvas(0.0, 0.0, scale)
        x_end = self._world_to_canvas(max_x_m, 0.0, scale)
        z_end = self._world_to_canvas(0.0, max_z_m, scale)
        self.canvas.create_line(*origin, *x_end, fill="#8492a0", width=2, arrow=tk.LAST)
        self.canvas.create_line(*origin, *z_end, fill="#8492a0", width=2, arrow=tk.LAST)
        self.canvas.create_text(
            x_end[0] - 8, x_end[1] - 15, text="+X 사용자 방향", fill="white", anchor="e"
        )
        self.canvas.create_text(
            z_end[0] + 8, z_end[1], text="+Z 위", fill="white", anchor="w"
        )
        self.canvas.create_text(
            origin[0] + 5, origin[1] + 16, text="BASE (0,0)", fill="white", anchor="w"
        )

        reference_start = self._world_to_canvas(0.0, self.reference_z_m, scale)
        reference_end = self._world_to_canvas(max_x_m, self.reference_z_m, scale)
        self.canvas.create_line(
            *reference_start,
            *reference_end,
            fill="#45d6d0",
            width=2,
            dash=(8, 5),
        )
        self.canvas.create_text(
            reference_end[0] - 5,
            reference_end[1] - 12,
            text=f"기준 Z={self.reference_z_m * 100:.1f}cm",
            fill="#45d6d0",
            anchor="e",
        )

        self._draw_arm(self.current, "#7f8c98", 6, scale)
        if self.target is not None:
            self._draw_arm(self.target, "#4ea5ff", 3, scale)

        if self.user_x_m is not None:
            actual_distance_m = user_x_m - current_pose.x_m
            monitor_point = self._world_to_canvas(
                current_pose.x_m, current_pose.z_m, scale
            )
            user_point = self._world_to_canvas(
                user_x_m, current_pose.z_m, scale
            )
            self.canvas.create_line(
                *monitor_point,
                *user_point,
                fill="#ffd166",
                width=3,
                dash=(7, 4),
            )
            self.canvas.create_text(
                (monitor_point[0] + user_point[0]) / 2,
                monitor_point[1] - 17,
                text=f"거리 {actual_distance_m * 100:.1f}cm",
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
                user_point[1] - 26,
                text=f"사용자 X={user_x_m * 100:.1f}cm",
                fill="#43c59e",
            )

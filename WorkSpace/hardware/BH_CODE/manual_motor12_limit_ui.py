#!/usr/bin/env python3
"""Press-and-hold jog UI for shoulder_lift (ID 1) and elbow_flex (ID 2).

Pressing + or - continuously advances that joint's target angle. Releasing
the button immediately stops creating new targets, so an idle UI sends no
position packets. Calibration hard limits and editable software limits are
always applied. Servo IDs 3 and 4 are never included in a packet.
"""

from __future__ import annotations

import time
import tkinter as tk
import sys
from pathlib import Path
from tkinter import messagebox, ttk

from monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    TwoJointMonitorArm,
    load_settings,
    save_settings,
)
ROOT_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = ROOT_DIR / "servo_calibration_result.json"
SETTINGS_PATH = ROOT_DIR / "monitor_arm_settings.json"
JOINTS = ("shoulder_lift", "elbow_flex")
JOG_INTERVAL_MS = 75
STATE_INTERVAL_MS = 300

# The repository contains the SDK environment used by the original motor code.
# A normally installed pyserial still takes precedence; this is a local fallback.
VENDORED_SITE_PACKAGES = (
    ROOT_DIR / "STServo_Python" / "stservo-env" / "Lib" / "site-packages"
)
if VENDORED_SITE_PACKAGES.is_dir() and str(VENDORED_SITE_PACKAGES) not in sys.path:
    sys.path.append(str(VENDORED_SITE_PACKAGES))

from motor_control.calibration import CalibrationError, CalibrationManager
from motor_control.config import (
    COMMAND_TO_URDF_DIRECTION,
    POSITION_PER_DEGREE,
    STS_POSITION_MAX,
    STS_POSITION_MIN,
)
from motor_control.servo_driver import ServoDriver


class ManualMotor12Bus:
    """Low-level adapter restricted by construction to servo IDs 1 and 2."""

    def __init__(self, calibration_path: Path):
        self.calibration = CalibrationManager(str(calibration_path))
        self.driver: ServoDriver | None = None
        self.joint_ids = {
            joint: int(self.calibration.get_joint(joint)["servo_id"])
            for joint in JOINTS
        }
        if self.joint_ids != {"shoulder_lift": 1, "elbow_flex": 2}:
            raise RuntimeError(f"Servo 1·2 매핑이 예상과 다릅니다: {self.joint_ids}")

    def open(self) -> None:
        if self.driver is None:
            self.driver = ServoDriver(
                device=self.calibration.device,
                baudrate=self.calibration.baudrate,
            )

    def close(self) -> None:
        if self.driver is not None:
            self.driver.close()
            self.driver = None

    def read_angles(self) -> JointCommand:
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        values = {}
        for joint in JOINTS:
            position = self.driver.read_position(self.joint_ids[joint])
            if position is None:
                raise RuntimeError(f"{joint} 현재 위치 읽기 실패")
            values[joint] = self.calibration.position_to_command_angle(joint, position)
        return JointCommand(values["shoulder_lift"], values["elbow_flex"])

    def read_states(self) -> dict:
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        states = {}
        for joint in JOINTS:
            raw_state = self.driver.read_state(self.joint_ids[joint])
            if raw_state is None:
                raise RuntimeError(f"{joint} 상태 읽기 실패")
            raw_state["angle"] = self.calibration.position_to_command_angle(
                joint, raw_state["position"]
            )
            states[joint] = raw_state
        return states

    def move_joint(self, joint: str, angle_deg: float, speed: int, acc: int) -> None:
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        if joint not in JOINTS:
            raise ValueError(f"수동 조그 대상이 아닙니다: {joint}")
        servo_id = self.joint_ids[joint]
        position = self.calibration.command_angle_to_position(joint, angle_deg)
        if not self.driver.write_position(servo_id, position, int(speed), int(acc)):
            raise RuntimeError(f"{joint} 조그 명령 실패")

    def move(self, target: JointCommand, speed: int, acc: int) -> None:
        """Compatibility API used by manual_vertical_ik_ui.py."""
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        commands = {}
        requested = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        for joint in JOINTS:
            servo_id = self.joint_ids[joint]
            commands[servo_id] = {
                "position": self.calibration.command_angle_to_position(
                    joint, requested[joint]
                ),
                "speed": int(speed),
                "acc": int(acc),
            }
        if not self.driver.sync_write_positions(commands):
            raise RuntimeError("Servo 1·2 SyncWrite 실패")

    def _unchecked_position(self, joint: str, angle_deg: float) -> int:
        """Convert an angle without the calibration *safe-range* check.

        This remains private and still enforces the STS absolute raw range.  It
        exists only for two narrow UI operations: returning from an already
        out-of-range physical rest pose, and entering the explicitly confirmed
        named rest pose.  Normal jog/IK commands continue to use the calibrated
        conversion above.
        """
        servo = self.calibration.require_position_calibrated(joint)
        urdf_angle_deg = float(angle_deg) * int(COMMAND_TO_URDF_DIRECTION[joint])
        position = int(
            round(
                int(servo["zero_position"])
                + int(servo["direction"])
                * urdf_angle_deg
                * POSITION_PER_DEGREE
            )
        )
        if not STS_POSITION_MIN <= position <= STS_POSITION_MAX:
            raise CalibrationError(
                f"{joint} 휴식/복구 Position={position}이 STS 절대범위 "
                f"{STS_POSITION_MIN}~{STS_POSITION_MAX} 밖입니다."
            )
        return position

    @staticmethod
    def _outside_distance(angle: float, minimum: float, maximum: float) -> float:
        if angle < minimum:
            return minimum - angle
        if angle > maximum:
            return angle - maximum
        return 0.0

    def move_recovery_aware(
        self,
        current: JointCommand,
        target: JointCommand,
        speed: int,
        acc: int,
    ) -> None:
        """Move normally, or permit only an inward step from outside safe range."""
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")

        commands = {}
        current_angles = {
            "shoulder_lift": current.shoulder_lift_deg,
            "elbow_flex": current.elbow_flex_deg,
        }
        target_angles = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        for joint in JOINTS:
            target_angle = target_angles[joint]
            try:
                position = self.calibration.command_angle_to_position(
                    joint, target_angle
                )
            except CalibrationError:
                safe_min, safe_max = self.calibration.get_safe_angle_range(joint)
                current_outside = self._outside_distance(
                    current_angles[joint], safe_min, safe_max
                )
                target_outside = self._outside_distance(
                    target_angle, safe_min, safe_max
                )
                if current_outside <= 0.0 or target_outside >= current_outside:
                    raise
                position = self._unchecked_position(joint, target_angle)

            commands[self.joint_ids[joint]] = {
                "position": position,
                "speed": int(speed),
                "acc": int(acc),
            }

        if not self.driver.sync_write_positions(commands):
            raise RuntimeError("Servo 1·2 복구 SyncWrite 실패")

    def move_confirmed_rest_pose(
        self,
        target: JointCommand,
        speed: int,
        acc: int,
    ) -> None:
        """Send the UI's explicitly confirmed named rest pose only."""
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        requested = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        commands = {
            self.joint_ids[joint]: {
                "position": self._unchecked_position(joint, requested[joint]),
                "speed": int(speed),
                "acc": int(acc),
            }
            for joint in JOINTS
        }
        if not self.driver.sync_write_positions(commands):
            raise RuntimeError("Servo 1·2 휴식자세 SyncWrite 실패")

    def hold(self) -> None:
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        commands = {}
        for joint in JOINTS:
            servo_id = self.joint_ids[joint]
            position = self.driver.read_position(servo_id)
            if position is None:
                raise RuntimeError(f"{joint} 현재 위치 읽기 실패")
            commands[servo_id] = {"position": position, "speed": 10, "acc": 3}
        if not self.driver.sync_write_positions(commands):
            raise RuntimeError("현재 위치 Hold 실패")

    def hold_joint(self, joint: str) -> float:
        """Stop one jog axis near its present position without touching the other."""
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        if joint not in JOINTS:
            raise ValueError(f"수동 조그 대상이 아닙니다: {joint}")
        servo_id = self.joint_ids[joint]
        position = self.driver.read_position(servo_id)
        if position is None:
            raise RuntimeError(f"{joint} 현재 위치 읽기 실패")
        if not self.driver.write_position(servo_id, position, 10, 3):
            raise RuntimeError(f"{joint} Hold 실패")
        return self.calibration.position_to_command_angle(joint, position)

    def torque_off_1_and_2(self) -> None:
        if self.driver is None:
            raise RuntimeError("먼저 모터 포트를 연결하세요.")
        for servo_id in (1, 2):
            if not self.driver.set_torque(servo_id, False):
                raise RuntimeError(f"Servo {servo_id} Torque OFF 실패")


class ManualJogWindow:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.settings = load_settings(SETTINGS_PATH)
        self.bus = ManualMotor12Bus(CALIBRATION_PATH)
        self.kinematics = TwoJointMonitorArm(ArmGeometry.from_settings(self.settings))
        self.current = JointCommand(0.0, 0.0)
        self.jog_targets = {joint: 0.0 for joint in JOINTS}
        self.jog_directions = {joint: 0 for joint in JOINTS}
        self.last_jog_at = {joint: None for joint in JOINTS}
        self.closing = False

        control = self.settings["control"]
        soft = self.settings["safety"]["soft_joint_limits_deg"]
        self.speed_var = tk.IntVar(value=int(control["speed"]))
        self.acc_var = tk.IntVar(value=int(control["acc"]))
        self.jog_rate_var = tk.DoubleVar(value=8.0)
        self.status_var = tk.StringVar(
            value="연결 전 — +/− 버튼을 누르고 있는 동안만 명령이 전송됩니다."
        )
        self.pose_var = tk.StringVar(value="모니터 중심 위치: -")
        self.angle_vars = {joint: tk.StringVar(value="현재 각도: -") for joint in JOINTS}
        self.state_vars = {joint: tk.StringVar(value="상태: -") for joint in JOINTS}
        self.soft_min_vars = {
            joint: tk.DoubleVar(value=float(soft[joint]["min"])) for joint in JOINTS
        }
        self.soft_max_vars = {
            joint: tk.DoubleVar(value=float(soft[joint]["max"])) for joint in JOINTS
        }

        self.root.title("POCO Servo 1·2 실시간 +/− 수동 조그")
        self.root.geometry("860x510")
        self.root.minsize(780, 470)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<FocusOut>", lambda _event: self.stop_all_jogs())

        self._build_ui()
        self.root.after(JOG_INTERVAL_MS, self.jog_tick)
        self.root.after(STATE_INTERVAL_MS, self.refresh_state)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=16)
        outer.pack(fill="both", expand=True)

        top = ttk.Frame(outer)
        top.pack(fill="x")
        ttk.Button(top, text="포트 연결", command=self.connect).pack(side="left")
        ttk.Button(top, text="현재 위치 Hold", command=self.hold).pack(side="left", padx=6)
        ttk.Button(top, text="1·2 토크 OFF", command=self.torque_off).pack(side="left")

        ttk.Label(top, text="조그 속도").pack(side="left", padx=(20, 4))
        ttk.Spinbox(
            top,
            from_=0.5,
            to=30.0,
            increment=0.5,
            width=7,
            textvariable=self.jog_rate_var,
        ).pack(side="left")
        ttk.Label(top, text="°/s").pack(side="left", padx=(2, 10))

        ttk.Label(top, text="Servo Speed").pack(side="left")
        ttk.Spinbox(
            top,
            from_=1,
            to=self.settings["control"]["manual_test_speed_cap"],
            width=7,
            textvariable=self.speed_var,
        ).pack(side="left", padx=4)
        ttk.Label(top, text="Acc").pack(side="left", padx=(8, 2))
        ttk.Spinbox(top, from_=0, to=30, width=6, textvariable=self.acc_var).pack(
            side="left"
        )

        ttk.Label(outer, textvariable=self.status_var, foreground="#a33").pack(
            fill="x", pady=(12, 4)
        )
        ttk.Label(outer, textvariable=self.pose_var).pack(fill="x", pady=(0, 8))

        for joint, servo_id in (("shoulder_lift", 1), ("elbow_flex", 2)):
            hard_min, hard_max = self.bus.calibration.get_safe_angle_range(joint)
            box = ttk.LabelFrame(outer, text=f"ID {servo_id}  {joint}", padding=12)
            box.pack(fill="x", pady=6)

            button_row = ttk.Frame(box)
            button_row.pack(fill="x")
            minus = tk.Button(
                button_row,
                text="−  아래",
                font=("TkDefaultFont", 24, "bold"),
                width=7,
                height=1,
                bg="#d85b5b",
                activebackground="#bd4444",
                fg="white",
            )
            minus.pack(side="left", fill="x", expand=True, padx=(0, 8))
            plus = tk.Button(
                button_row,
                text="+  위",
                font=("TkDefaultFont", 24, "bold"),
                width=7,
                height=1,
                bg="#3a9d6f",
                activebackground="#2e805a",
                fg="white",
            )
            plus.pack(side="left", fill="x", expand=True)
            self.bind_jog_button(minus, joint, -1)
            self.bind_jog_button(plus, joint, +1)

            readout = ttk.Frame(box)
            readout.pack(fill="x", pady=(8, 2))
            ttk.Label(readout, textvariable=self.angle_vars[joint]).pack(side="left")
            ttk.Label(readout, textvariable=self.state_vars[joint]).pack(side="right")

            limits = ttk.Frame(box)
            limits.pack(fill="x", pady=(4, 0))
            ttk.Label(limits, text=f"Hard {hard_min:+.1f}° ~ {hard_max:+.1f}° | Soft").pack(
                side="left"
            )
            ttk.Spinbox(
                limits,
                from_=hard_min,
                to=hard_max,
                increment=0.5,
                width=8,
                textvariable=self.soft_min_vars[joint],
            ).pack(side="left", padx=4)
            ttk.Label(limits, text="~").pack(side="left")
            ttk.Spinbox(
                limits,
                from_=hard_min,
                to=hard_max,
                increment=0.5,
                width=8,
                textvariable=self.soft_max_vars[joint],
            ).pack(side="left", padx=4)
            ttk.Button(
                limits,
                text="현재각→min",
                command=lambda name=joint: self.use_current_as_limit(name, "min"),
            ).pack(side="left", padx=(10, 3))
            ttk.Button(
                limits,
                text="현재각→max",
                command=lambda name=joint: self.use_current_as_limit(name, "max"),
            ).pack(side="left")

        ttk.Button(
            outer,
            text="Soft limit / Speed / Acc 저장",
            command=self.save_settings_from_ui,
        ).pack(fill="x", pady=(10, 0))

    def bind_jog_button(self, button: tk.Button, joint: str, direction: int) -> None:
        button.bind("<ButtonPress-1>", lambda _event: self.start_jog(joint, direction))
        button.bind("<ButtonRelease-1>", lambda _event: self.stop_jog(joint))
        button.bind("<Leave>", lambda _event: self.stop_jog(joint))

    def connect(self) -> None:
        try:
            self.stop_all_jogs()
            self.bus.open()
            self.current = self.bus.read_angles()
            self.sync_jog_targets_to_current()
            self.status_var.set(
                "연결 완료 — 버튼을 누르는 동안만 해당 관절에 새 위치를 전송합니다."
            )
        except Exception as error:
            messagebox.showerror("연결 실패", str(error))

    def sync_jog_targets_to_current(self) -> None:
        self.jog_targets["shoulder_lift"] = self.current.shoulder_lift_deg
        self.jog_targets["elbow_flex"] = self.current.elbow_flex_deg

    def start_jog(self, joint: str, direction: int) -> None:
        if self.bus.driver is None:
            messagebox.showwarning("연결 필요", "먼저 모터 포트를 연결하세요.")
            return
        try:
            latest = self.bus.read_angles()
            self.current = latest
            self.jog_targets[joint] = (
                latest.shoulder_lift_deg
                if joint == "shoulder_lift"
                else latest.elbow_flex_deg
            )
            self.jog_directions[joint] = 1 if direction > 0 else -1
            self.last_jog_at[joint] = time.monotonic()
            sign = "+" if direction > 0 else "−"
            self.status_var.set(f"{joint} {sign} 조그 중")
        except Exception as error:
            self.stop_jog(joint)
            messagebox.showerror("조그 시작 실패", str(error))

    def stop_jog(self, joint: str, hold: bool = True) -> None:
        was_active = self.jog_directions[joint] != 0
        self.jog_directions[joint] = 0
        self.last_jog_at[joint] = None

        if was_active and hold and self.bus.driver is not None:
            try:
                self.jog_targets[joint] = self.bus.hold_joint(joint)
            except Exception as error:
                self.status_var.set(f"{joint} 버튼 해제 Hold 실패: {error}")
                return

        if not any(self.jog_directions.values()):
            self.status_var.set("버튼 해제 — 현재 위치 Hold 후 새 모터 명령 없음")

    def stop_all_jogs(self, hold: bool = True) -> None:
        for joint in JOINTS:
            self.stop_jog(joint, hold=hold)

    def effective_range(self, joint: str) -> tuple[float, float]:
        hard_min, hard_max = self.bus.calibration.get_safe_angle_range(joint)
        soft_min = float(self.soft_min_vars[joint].get())
        soft_max = float(self.soft_max_vars[joint].get())
        minimum = max(hard_min, soft_min)
        maximum = min(hard_max, soft_max)
        if minimum >= maximum:
            raise ValueError(f"{joint} soft min은 soft max보다 작아야 합니다.")
        return minimum, maximum

    def motion_parameters(self) -> tuple[float, int, int]:
        jog_rate = float(self.jog_rate_var.get())
        speed = int(self.speed_var.get())
        acc = int(self.acc_var.get())
        speed_cap = int(self.settings["control"]["manual_test_speed_cap"])
        if not 0.5 <= jog_rate <= 30.0:
            raise ValueError("조그 속도 허용범위는 0.5~30.0°/s입니다.")
        if not 1 <= speed <= speed_cap:
            raise ValueError(f"Servo Speed 허용범위는 1~{speed_cap}입니다.")
        if not 0 <= acc <= 30:
            raise ValueError("Acc 허용범위는 0~30입니다.")
        return jog_rate, speed, acc

    def jog_tick(self) -> None:
        try:
            if self.bus.driver is not None:
                now = time.monotonic()
                jog_rate, speed, acc = self.motion_parameters()
                for joint in JOINTS:
                    direction = self.jog_directions[joint]
                    previous_at = self.last_jog_at[joint]
                    if direction == 0 or previous_at is None:
                        continue

                    dt = min(max(now - previous_at, 0.0), 0.15)
                    self.last_jog_at[joint] = now
                    minimum, maximum = self.effective_range(joint)
                    old_target = self.jog_targets[joint]
                    next_target = old_target + direction * jog_rate * dt
                    next_target = max(minimum, min(next_target, maximum))

                    if abs(next_target - old_target) < 1e-6:
                        self.status_var.set(
                            f"{joint} {'최대' if direction > 0 else '최소'} soft limit 도달"
                        )
                        continue

                    self.bus.move_joint(
                        joint,
                        next_target,
                        speed,
                        acc,
                    )
                    self.jog_targets[joint] = next_target
        except Exception as error:
            self.stop_all_jogs(hold=False)
            self.status_var.set(f"조그 중단: {error}")
        finally:
            if not self.closing:
                self.root.after(JOG_INTERVAL_MS, self.jog_tick)

    def refresh_state(self) -> None:
        try:
            if self.bus.driver is not None:
                states = self.bus.read_states()
                self.current = JointCommand(
                    states["shoulder_lift"]["angle"],
                    states["elbow_flex"]["angle"],
                )
                pose = self.kinematics.forward(self.current)
                self.pose_var.set(
                    f"모니터 중심 X={pose.x_m * 100:.1f}cm, Z={pose.z_m * 100:.1f}cm"
                )
                for joint in JOINTS:
                    state = states[joint]
                    self.angle_vars[joint].set(
                        f"현재 {state['angle']:+.2f}° / 조그 목표 {self.jog_targets[joint]:+.2f}°"
                    )
                    self.state_vars[joint].set(
                        f"Load {state['load_percent']}% | Temp {state['temperature']}°C"
                    )
        except Exception as error:
            self.stop_all_jogs(hold=False)
            self.status_var.set(f"상태 읽기 실패: {error}")
        finally:
            if not self.closing:
                self.root.after(STATE_INTERVAL_MS, self.refresh_state)

    def use_current_as_limit(self, joint: str, side: str) -> None:
        angle = (
            self.current.shoulder_lift_deg
            if joint == "shoulder_lift"
            else self.current.elbow_flex_deg
        )
        variable = self.soft_min_vars[joint] if side == "min" else self.soft_max_vars[joint]
        variable.set(round(angle, 1))

    def save_settings_from_ui(self) -> None:
        try:
            self.stop_all_jogs()
            for joint in JOINTS:
                minimum, maximum = self.effective_range(joint)
                self.settings["safety"]["soft_joint_limits_deg"][joint] = {
                    "min": minimum,
                    "max": maximum,
                }
            self.settings["control"]["speed"] = int(self.speed_var.get())
            self.settings["control"]["acc"] = int(self.acc_var.get())
            save_settings(self.settings, SETTINGS_PATH)
            self.status_var.set(f"설정 저장 완료: {SETTINGS_PATH.name}")
        except Exception as error:
            messagebox.showerror("설정 저장 실패", str(error))

    def hold(self) -> None:
        self.stop_all_jogs()
        try:
            self.bus.hold()
            self.status_var.set("Servo 1·2 현재 위치 Hold")
        except Exception as error:
            messagebox.showerror("Hold 실패", str(error))

    def torque_off(self) -> None:
        self.stop_all_jogs()
        if not messagebox.askyesno(
            "Servo 1·2 토크 OFF",
            "팔과 모니터를 확실히 지지했습니까? Servo 1·2만 토크가 해제됩니다.",
        ):
            return
        try:
            self.bus.torque_off_1_and_2()
            self.status_var.set("Servo 1·2 Torque OFF")
        except Exception as error:
            messagebox.showerror("Torque OFF 실패", str(error))

    def close(self) -> None:
        self.closing = True
        self.stop_all_jogs()
        self.bus.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ManualJogWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()

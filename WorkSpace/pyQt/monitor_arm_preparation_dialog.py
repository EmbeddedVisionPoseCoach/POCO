"""PyQt dialog used by POCO's '초기값 준비' step."""

from __future__ import annotations

import math
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from services.monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    TwoJointMonitorArm,
    load_settings,
)


WORKSPACE_DIR = Path(__file__).resolve().parent.parent
SETTINGS_PATH = (
    WORKSPACE_DIR / "config" / "monitor_arm_settings.json"
)


def _fmt(value, digits=2, suffix=""):
    if value is None:
        return "--"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return "--"


class ArmPreparationCanvas(QWidget):
    """Small X-Z visualization of current and requested Servo1/2 poses."""

    def __init__(self, kinematics, parent=None):
        super().__init__(parent)
        self.kinematics = kinematics
        self.current = None
        self.target = None
        self.user_x_m = None
        self.setMinimumSize(260, 210)

    def set_state(self, preparation, user_x_m):
        current = preparation.get("current_angles_deg", {})
        target = preparation.get("target_angles_deg", {})
        self.current = self._command(current)
        self.target = self._command(target)
        self.user_x_m = user_x_m
        self.update()

    @staticmethod
    def _command(values):
        try:
            shoulder = values.get("shoulder_lift")
            elbow = values.get("elbow_flex")
            if shoulder is None or elbow is None:
                return None
            return JointCommand(float(shoulder), float(elbow))
        except (TypeError, ValueError, AttributeError):
            return None

    def _points(self, command):
        k = self.kinematics
        g = k.geometry
        shoulder_urdf, elbow_urdf = k.command_to_urdf(command)
        upper_world = g.upper_zero_angle_rad - shoulder_urdf
        lower_world = g.lower_zero_angle_rad - shoulder_urdf - elbow_urdf
        shoulder = (g.shoulder_x_m, g.shoulder_z_m)
        elbow = (
            shoulder[0] + g.upper_link_m * math.cos(upper_world),
            shoulder[1] + g.upper_link_m * math.sin(upper_world),
        )
        monitor = (
            elbow[0] + g.effective_lower_link_m * math.cos(lower_world),
            elbow[1] + g.effective_lower_link_m * math.sin(lower_world),
        )
        return [(0.0, 0.0), shoulder, elbow, monitor]

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f7f9fc"))
        origin_x = 45.0
        origin_y = self.height() - 45.0
        scale = min((self.width() - 80.0) / 0.9, (self.height() - 80.0) / 0.45)

        def world(point):
            return origin_x + point[0] * scale, origin_y - point[1] * scale

        painter.setPen(QPen(QColor("#9aa4b2"), 1))
        painter.drawLine(int(origin_x), 20, int(origin_x), int(origin_y))
        painter.drawLine(int(origin_x), int(origin_y), self.width() - 20, int(origin_y))
        painter.drawText(12, 25, "+Z")
        painter.drawText(self.width() - 45, self.height() - 15, "+X 사용자")

        def draw_arm(command, color, width):
            if command is None:
                return
            points = [world(point) for point in self._points(command)]
            painter.setPen(QPen(QColor(color), width))
            for start, end in zip(points, points[1:]):
                painter.drawLine(
                    int(start[0]), int(start[1]), int(end[0]), int(end[1])
                )
            painter.setBrush(QColor(color))
            for x, y in points:
                painter.drawEllipse(int(x - 4), int(y - 4), 8, 8)

        draw_arm(self.target, "#f39c12", 3)
        draw_arm(self.current, "#1565c0", 5)

        if self.user_x_m is not None:
            user_x, user_y = world((float(self.user_x_m), 0.0))
            painter.setPen(QPen(QColor("#16a085"), 2, Qt.DashLine))
            painter.drawLine(int(user_x), 25, int(user_x), int(user_y))
            painter.drawText(int(user_x - 28), 42, "USER")

        painter.setPen(QColor("#1565c0"))
        painter.drawText(15, self.height() - 20, "파랑: 현재")
        painter.setPen(QColor("#f39c12"))
        painter.drawText(105, self.height() - 20, "주황: 목표")
        painter.end()


class MonitorArmPreparationDialog(QDialog):
    preparation_finished = pyqtSignal(bool, str)

    def __init__(self, send_command, get_hardware_state, parent=None, manual_only=False):
        super().__init__(parent)
        self.send_command = send_command
        self.get_hardware_state = get_hardware_state
        self.settings = load_settings(SETTINGS_PATH)
        self.kinematics = TwoJointMonitorArm(
            ArmGeometry.from_settings(self.settings)
        )
        self._accepted_close = False
        self._jog_joint = None
        self._jog_direction = 0
        self._latest_preparation = {}
        self._latest_calibration = {}
        self.manual_only = bool(manual_only)

        cartesian = self.settings.get("manual_cartesian", {})
        distance = self.settings.get("distance", {})
        self.user_min_cm = float(cartesian.get("user_x_min_m", 0.6007655)) * 100
        self.user_max_cm = float(cartesian.get("user_x_max_m", 0.8307655)) * 100
        self.default_z_cm = float(cartesian.get("default_monitor_z_m", 0.256)) * 100
        self.distance_cm = float(
            distance.get("desired_user_monitor_distance_m", 0.5)
        ) * 100
        self.applied_distance_cm = self.distance_cm
        self.applied_height_cm = self.default_z_cm

        self.setWindowTitle(
            "POCO 모니터암 수동조작"
            if self.manual_only else
            "POCO 모니터암 초기 준비"
        )
        self.resize(780, 440)
        self.setMinimumSize(720, 400)
        self.setMaximumSize(800, 470)
        self.setModal(False)
        self._build_ui()

        self.state_timer = QTimer(self)
        self.state_timer.timeout.connect(self.refresh_state)
        self.state_timer.start(100)
        self.ik_debounce = QTimer(self)
        self.ik_debounce.setSingleShot(True)
        self.ik_debounce.timeout.connect(self.send_manual_ik)
        self.jog_timer = QTimer(self)
        self.jog_timer.timeout.connect(self._send_jog_step)

    def _build_ui(self):
        self.setStyleSheet(
            "QGroupBox{font-weight:600;margin-top:6px;}"
            "QGroupBox::title{subcontrol-origin:margin;left:8px;}"
            "QPushButton{min-height:28px;} QLabel{font-size:12px;}"
        )
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(5)
        tabs = QTabWidget()
        preparation_tab = QWidget()
        preparation_controls = QVBoxLayout(preparation_tab)
        manual_tab = QWidget()
        manual_layout = QHBoxLayout(manual_tab)
        manual_controls = QVBoxLayout()
        manual_layout.addLayout(manual_controls, 5)
        tabs.addTab(preparation_tab, "자세 전환 · 센서")
        tabs.addTab(manual_tab, "모터 수동조작")
        outer.addWidget(tabs, 1)

        connection = QGroupBox("1. 모터 1~4 연결 및 작업 시작 위치")
        connection_layout = QGridLayout(connection)
        self.connect_button = QPushButton("모터 1~4 연결 확인")
        self.connect_button.clicked.connect(
            lambda: self._send({"type": "MONITOR_ARM_CONNECT_ALL"})
        )
        self.working_button = QPushButton("휴식 → 작업 시작 위치 이동")
        self.working_button.setEnabled(False)
        self.working_button.clicked.connect(
            lambda: self._send({"type": "MONITOR_ARM_MOVE_WORKING_START"})
        )
        self.rest_button = QPushButton("작업자세 → 휴식자세 이동")
        self.rest_button.setEnabled(False)
        self.rest_button.clicked.connect(self.request_rest_pose)
        self.connection_label = QLabel("연결 전")
        self.connection_label.setWordWrap(True)
        connection_layout.addWidget(self.connect_button, 0, 0)
        connection_layout.addWidget(self.working_button, 0, 1)
        connection_layout.addWidget(self.rest_button, 0, 2)
        connection_layout.addWidget(self.connection_label, 1, 0, 1, 3)
        preparation_controls.addWidget(connection)

        ik_group = QGroupBox("2. Servo 1·2 — 사용자 X 게이지 기반 수동 IK")
        ik_layout = QGridLayout(ik_group)
        self.user_slider = QSlider(Qt.Horizontal)
        self.user_slider.setRange(round(self.user_min_cm * 10), round(self.user_max_cm * 10))
        self.user_slider.setValue(round(self.user_min_cm * 10))
        self.user_slider.setEnabled(False)
        self.user_slider.valueChanged.connect(self._on_ik_value_changed)
        self.user_value_label = QLabel()
        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(20.0, 100.0)
        self.distance_spin.setDecimals(1)
        self.distance_spin.setValue(self.distance_cm)
        self.distance_spin.setSuffix(" cm")
        self.height_spin = QDoubleSpinBox()
        self.height_spin.setRange(
            float(self.settings["manual_cartesian"]["monitor_z_min_m"]) * 100,
            float(self.settings["manual_cartesian"]["monitor_z_max_m"]) * 100,
        )
        self.height_spin.setDecimals(1)
        self.height_spin.setValue(self.default_z_cm)
        self.height_spin.setSuffix(" cm")
        self.distance_spin.valueChanged.connect(self._mark_constants_pending)
        self.height_spin.valueChanged.connect(self._mark_constants_pending)
        ik_layout.addWidget(QLabel("베이스 → 사용자 X"), 0, 0)
        ik_layout.addWidget(self.user_slider, 0, 1)
        ik_layout.addWidget(self.user_value_label, 0, 2)
        ik_layout.addWidget(QLabel("사용자 ↔ 모니터 고정거리"), 1, 0)
        ik_layout.addWidget(self.distance_spin, 1, 1)
        ik_layout.addWidget(QLabel("모니터 고정 Z"), 2, 0)
        ik_layout.addWidget(self.height_spin, 2, 1)
        self.apply_constants_button = QPushButton("고정거리·높이 적용 후 현재 X로 이동")
        self.apply_constants_button.setEnabled(False)
        self.apply_constants_button.clicked.connect(self.apply_constants)
        ik_layout.addWidget(self.apply_constants_button, 2, 2)
        self.ik_label = QLabel("현재/목표 IK: --")
        self.ik_label.setWordWrap(True)
        ik_layout.addWidget(self.ik_label, 3, 0, 1, 3)
        manual_controls.addWidget(ik_group)

        gimbal = QGroupBox("3. Servo 3·4 조그 — 버튼을 누르는 동안만 이동")
        gimbal_layout = QGridLayout(gimbal)
        self.jog_speed = QSpinBox()
        self.jog_speed.setRange(1, 1000)
        self.jog_speed.setValue(100)
        self.jog_step = QDoubleSpinBox()
        self.jog_step.setRange(0.1, 3.0)
        self.jog_step.setValue(0.5)
        self.jog_step.setSuffix("°/tick")
        gimbal_layout.addWidget(QLabel("Speed"), 0, 0)
        gimbal_layout.addWidget(self.jog_speed, 0, 1)
        gimbal_layout.addWidget(QLabel("조그 변화량"), 0, 2)
        gimbal_layout.addWidget(self.jog_step, 0, 3)
        self.gimbal_labels = {}
        for row, (joint, title) in enumerate(
            (("wrist_flex", "Motor3 wrist_flex"), ("wrist_roll", "Motor4 wrist_roll")),
            start=1,
        ):
            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setMinimumHeight(42)
            plus.setMinimumHeight(42)
            minus.pressed.connect(lambda name=joint: self._start_jog(name, -1))
            plus.pressed.connect(lambda name=joint: self._start_jog(name, +1))
            minus.released.connect(self._stop_jog)
            plus.released.connect(self._stop_jog)
            label = QLabel(f"{title}: --")
            self.gimbal_labels[joint] = label
            gimbal_layout.addWidget(label, row, 0)
            gimbal_layout.addWidget(minus, row, 1)
            gimbal_layout.addWidget(plus, row, 2)
        manual_controls.addWidget(gimbal)

        sensor = QGroupBox("4. ToF + MediaPipe 눈 간격 기준값 저장")
        sensor_layout = QGridLayout(sensor)
        self.live_sensor_label = QLabel("ToF: -- / Eye: --")
        self.capture_button = QPushButton("5초 평균 측정 시작")
        self.capture_button.setEnabled(False)
        self.capture_button.clicked.connect(
            lambda: self._send({"type": "START_MONITOR_ARM_SENSOR_CAPTURE"})
        )
        self.capture_progress = QProgressBar()
        self.capture_progress.setRange(0, 50)
        self.capture_result_label = QLabel("아직 저장된 준비 기준값이 없습니다.")
        self.capture_result_label.setWordWrap(True)
        sensor_layout.addWidget(self.live_sensor_label, 0, 0, 1, 2)
        sensor_layout.addWidget(self.capture_button, 1, 0)
        sensor_layout.addWidget(self.capture_progress, 1, 1)
        sensor_layout.addWidget(self.capture_result_label, 2, 0, 1, 2)
        preparation_controls.addWidget(sensor)
        preparation_controls.addStretch(1)

        self.status_label = QLabel("먼저 모터 1~4 연결 확인을 눌러주세요.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color:#b53a2d; font-weight:600;")
        outer.addWidget(self.status_label)

        bottom = QHBoxLayout()
        self.finish_button = QPushButton("준비 완료 후 종료")
        self.finish_button.setEnabled(False)
        self.finish_button.clicked.connect(
            lambda: self._send({"type": "FINISH_MONITOR_ARM_PREPARATION"})
        )
        cancel = QPushButton("닫기" if self.manual_only else "취소")
        cancel.clicked.connect(self.cancel_preparation)
        bottom.addWidget(self.finish_button)
        bottom.addWidget(cancel)
        outer.addLayout(bottom)

        self.canvas = ArmPreparationCanvas(self.kinematics)
        manual_layout.addWidget(self.canvas, 4)
        if self.manual_only:
            sensor.hide()
            self.finish_button.hide()
            tabs.setCurrentIndex(1)
        self._update_user_label()

    def _send(self, message):
        if not self.send_command(message):
            self.status_label.setText("하드웨어 프로세스에 명령을 전달하지 못했습니다.")
            return False
        return True

    def _on_ik_value_changed(self, _value=None):
        self._update_user_label()
        if self.user_slider.isEnabled():
            self.ik_debounce.start(100)

    def _update_user_label(self):
        self.user_value_label.setText(f"{self.user_slider.value() / 10.0:.1f} cm")

    def _mark_constants_pending(self, _value=None):
        self.status_label.setText(
            "고정거리/높이 변경은 '고정거리·높이 적용'을 눌러야 반영됩니다."
        )

    def apply_constants(self):
        self.applied_distance_cm = float(self.distance_spin.value())
        self.applied_height_cm = float(self.height_spin.value())
        self.send_manual_ik()

    def send_manual_ik(self):
        if not self._latest_preparation.get("working_start_completed", False):
            return
        self._send(
            {
                "type": "MONITOR_ARM_MANUAL_IK_TARGET",
                "user_x_m": self.user_slider.value() / 1000.0,
                "user_monitor_distance_m": self.applied_distance_cm / 100.0,
                "monitor_z_m": self.applied_height_cm / 100.0,
            }
        )

    def _start_jog(self, joint, direction):
        if self._latest_calibration.get("running", False):
            self.status_label.setText("센서 평균 측정 중에는 모터를 조작할 수 없습니다.")
            return
        if not self._latest_preparation.get("all_motors_ready", False):
            self.status_label.setText("먼저 모터 1~4 연결 확인을 완료해주세요.")
            return
        self._jog_joint = joint
        self._jog_direction = direction
        self._send_jog_step()
        self.jog_timer.start(120)

    def _stop_jog(self):
        joint = self._jog_joint
        self.jog_timer.stop()
        self._jog_joint = None
        self._jog_direction = 0
        if joint is not None:
            self._send(
                {
                    "type": "MONITOR_ARM_GIMBAL_JOG_STOP",
                    "joint": joint,
                }
            )

    def request_rest_pose(self):
        answer = QMessageBox.question(
            self,
            "휴식자세 이동 확인",
            "Motor1/2를 휴식자세(S +107.75°, E -92.55°)로 이동합니다.\n"
            "모니터와 팔을 지지하고 주변에 충돌물이 없는지 확인했습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self._send(
            {
                "type": "MONITOR_ARM_MOVE_REST",
                "confirmed": True,
            }
        )

    def _send_jog_step(self):
        if self._jog_joint is None:
            return
        self._send(
            {
                "type": "MONITOR_ARM_GIMBAL_JOG",
                "joint": self._jog_joint,
                "delta_deg": self._jog_direction * self.jog_step.value(),
                "speed": self.jog_speed.value(),
            }
        )

    def refresh_state(self):
        state = self.get_hardware_state() or {}
        monitor_arm = state.get("monitor_arm", {})
        preparation = monitor_arm.get("preparation", {})
        calibration = monitor_arm.get("calibration", {})
        self._latest_preparation = preparation
        self._latest_calibration = calibration

        connected = bool(preparation.get("all_motors_ready", False))
        working = bool(preparation.get("working_start_completed", False))
        moving = bool(preparation.get("movement_active", False))
        movement_status = str(preparation.get("movement_status", "idle"))
        capture_running = bool(calibration.get("running", False))
        capture_ready = bool(calibration.get("session_ready", False))
        self.connect_button.setEnabled(not capture_running)
        self.working_button.setEnabled(connected and not moving and not capture_running)
        self.rest_button.setEnabled(connected and not moving and not capture_running)
        self.user_slider.setEnabled(connected and working and not capture_running)
        self.distance_spin.setEnabled(connected and working and not capture_running)
        self.height_spin.setEnabled(connected and working and not capture_running)
        self.apply_constants_button.setEnabled(
            connected and working and not capture_running
        )
        self.capture_button.setEnabled(
            connected and working and not moving and not capture_running
        )
        self.finish_button.setEnabled(connected and working and capture_ready)

        angles = preparation.get("motor_angles_deg", {})
        current_pose = preparation.get("current_pose", {})
        target_pose = preparation.get("target_pose", {})
        self.connection_label.setText(
            f"Motor 1~4: {'READY' if connected else '연결/확인 필요'} | "
            f"S={_fmt(angles.get('shoulder_lift'), 2, '°')}  "
            f"E={_fmt(angles.get('elbow_flex'), 2, '°')} | "
            f"현재 X={_fmt(current_pose.get('x_m') * 100 if current_pose.get('x_m') is not None else None, 1, 'cm')} "
            f"Z={_fmt(current_pose.get('z_m') * 100 if current_pose.get('z_m') is not None else None, 1, 'cm')}"
        )
        for joint in self.gimbal_labels:
            title = "Motor3 wrist_flex" if joint == "wrist_flex" else "Motor4 wrist_roll"
            self.gimbal_labels[joint].setText(
                f"{title}: {_fmt(angles.get(joint), 2, '°')}"
            )
        target_angles = preparation.get("target_angles_deg", {})
        self.ik_label.setText(
            f"현재 S/E={_fmt(angles.get('shoulder_lift'), 2, '°')} / "
            f"{_fmt(angles.get('elbow_flex'), 2, '°')} | "
            f"목표 S/E={_fmt(target_angles.get('shoulder_lift'), 2, '°')} / "
            f"{_fmt(target_angles.get('elbow_flex'), 2, '°')} | "
            f"목표 X/Z={_fmt(target_pose.get('x_m') * 100 if target_pose.get('x_m') is not None else None, 1, 'cm')} / "
            f"{_fmt(target_pose.get('z_m') * 100 if target_pose.get('z_m') is not None else None, 1, 'cm')}\n"
            f"최대 관절 오차={_fmt(preparation.get('current_target_max_error_deg'), 2, '°')} | "
            f"도착 허용={_fmt(preparation.get('arrival_tolerance_deg'), 2, '°')} | "
            f"안정화={preparation.get('stable_samples', 0)}/"
            f"{preparation.get('required_stable_samples', '--')}"
        )

        tof = monitor_arm.get("live_tof_user_x_m")
        eye = monitor_arm.get("live_eye_gap_px")
        self.live_sensor_label.setText(
            f"실시간 ToF 사용자 X: {_fmt(tof * 100 if tof is not None else None, 1, 'cm')} | "
            f"실시간 눈 간격: {_fmt(eye, 2, 'px')} | "
            f"샘플 ToF={calibration.get('tof_sample_count', 0)}, "
            f"Eye={calibration.get('eye_sample_count', 0)}"
        )
        remain = float(calibration.get("remain_sec", 0.0) or 0.0)
        self.capture_progress.setValue(round((5.0 - remain) * 10) if capture_running else (50 if capture_ready else 0))
        if capture_ready:
            self.capture_result_label.setText(
                f"저장 완료 — ToF 평균 {_fmt(calibration.get('tof_user_x_baseline_m'), 3, 'm')}, "
                f"눈 간격 평균 {_fmt(calibration.get('eye_gap_baseline_px'), 2, 'px')}, "
                f"사용자-모니터 {_fmt(calibration.get('user_monitor_distance_baseline_m'), 3, 'm')}\n"
                f"파일: {calibration.get('path', '--')}"
            )
        elif capture_running:
            self.capture_result_label.setText(f"평균 측정 중 — 남은 시간 {remain:.1f}초")

        self.canvas.set_state(
            preparation,
            self.user_slider.value() / 1000.0,
        )

        error = preparation.get("last_error")
        if movement_status == "timeout":
            self.status_label.setText(f"작업 시작 위치 이동 시간 초과: {error}")
        elif movement_status == "safety_error":
            self.status_label.setText(f"Recovery 안전 검사 중단: {error}")
        elif movement_status == "command_error":
            self.status_label.setText(f"모터 명령 실패: {error}")
        elif movement_status == "telemetry_error":
            self.status_label.setText(f"현재각 읽기 실패: {error}")
        elif error:
            self.status_label.setText(f"제어 오류: {error}")
        elif movement_status == "stabilizing":
            self.status_label.setText(
                "목표 오차 범위에 도착했습니다. 연속 안정화 확인 중입니다."
            )
        elif moving:
            self.status_label.setText("작업 시작/IK 목표로 이동 중입니다.")
        elif connected and not working:
            self.status_label.setText("휴식 → 작업 시작 위치 이동 버튼을 눌러주세요.")
        elif working and not capture_ready and not capture_running:
            self.status_label.setText(
                "목표 도착 및 안정화 완료. 자세 조정 후 ToF/눈 간격 5초 평균을 측정해주세요."
            )
        elif capture_ready:
            self.status_label.setText("준비 완료. '준비 완료 후 종료'를 누르세요.")

    def handle_hardware_event(self, event):
        if not isinstance(event, dict):
            return
        event_type = str(event.get("type", "")).upper()
        relevant = event_type.startswith("MONITOR_ARM_")
        if not relevant:
            return
        message = str(event.get("message", ""))
        if message:
            self.status_label.setText(message)
        if event.get("success") is False and event_type != "MONITOR_ARM_GIMBAL_JOG_ACK":
            QMessageBox.warning(self, "모니터암 초기 준비", message or "명령 실패")
        if event_type == "MONITOR_ARM_PREPARATION_FINISHED" and event.get("success"):
            self._accepted_close = True
            self.preparation_finished.emit(True, message)
            self.accept()

    def cancel_preparation(self):
        self._stop_jog()
        self._send({"type": "CANCEL_MONITOR_ARM_PREPARATION"})
        self._accepted_close = True
        self.preparation_finished.emit(False, "모니터암 초기 준비를 취소했습니다.")
        self.reject()

    def closeEvent(self, event):
        self._stop_jog()
        if not self._accepted_close:
            self._send({"type": "CANCEL_MONITOR_ARM_PREPARATION"})
            self.preparation_finished.emit(False, "모니터암 초기 준비 창을 닫았습니다.")
        event.accept()

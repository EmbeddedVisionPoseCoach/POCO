import os
import copy
import sys
import faulthandler
from pathlib import Path

import platform
import shutil
import subprocess
import webbrowser

from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer, QCoreApplication, QEventLoop
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QPushButton

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR))

import modules.config as config
from camera_worker_profile_all import CameraWorker
from monitor_arm_preparation_dialog import MonitorArmPreparationDialog
from user_profile_dialog import UserProfileDialog
from managers.vision_process_manager_profile import PROFILE_MODE
from modules.app_settings import SettingsManager, AlarmSettings
from services.hardware_config_service import HardwareConfigService
from services.hardware_state_store import get_hardware_runtime_state_store
from services.user_profile_service import UserProfileService

import warnings

warnings.filterwarnings(
    "ignore",
    message="SymbolDatabase.GetPrototype.*"
)

warnings.filterwarnings(
    "ignore",
    category=UserWarning,
    module="google.protobuf.symbol_database"
)


class MainWindow(QMainWindow):
    """
    PyQt 메인 윈도우.

    이 클래스는 UI만 담당한다.
    카메라 처리, 랜드마크 탐지, 캘리브레이션 계산은 직접 하지 않는다.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        ui_path = Path(__file__).parent / "ui" / "pocoApplication_Qss.ui"
        # ui_path = Path(__file__).parent / "ui" / "pocoApplication.ui"
        uic.loadUi(str(ui_path), self)

        self.camera_worker = None
        self._camera_shutdown_in_progress = False
        self._app_closing = False
        self.streamlit_process = None
        self.streamlit_port = 8501

        # 설정 저장 경로
        self.settings_manager = SettingsManager(
            ROOT_DIR / "data" / "settings" / "alarm_settings.json"
        )

        # Hardware 제어 설정(JSON)과 Runtime Sensor State는 성격이 다르므로 분리한다.
        # - ConfigService: PID/LPF/Deadband 설정 + 마지막 IMU X/Y 기준값 기록
        # - StateStore: IMU/Motor의 실시간 최신값(메모리 only)
        self.hardware_config_service = HardwareConfigService()
        self.hardware_state_store = get_hardware_runtime_state_store()
        self.latest_hardware_config = self.hardware_config_service.load()

        # Vision / Hardware는 별도 Process로 실행된다.
        # Main Process는 UI 표시용 최신 상태만 보관한다.
        self.latest_pose_state = None
        self.latest_face_state = None
        self.latest_hardware_state = None
        self.latest_hardware_event = None
        self.monitor_arm_preparation_dialog = None
        self.monitor_arm_preparation_ready = False
        self.user_profile_service = UserProfileService()
        self.active_profile_slot = None
        self._pending_profile_slot = None
        self._measurement_stop_pending = False
        self._last_safety_message = None

        self.current_alarm_settings = None

        self.btnCalibration.clicked.connect(self.on_calibration_clicked)
        self.btnCalibrationStart.clicked.connect(self.on_calibration_start_clicked)
        self.btnCamOn.clicked.connect(self.on_camera_on_clicked)
        self.btnCamOff.clicked.connect(self.on_camera_off_clicked)

        self.btnUserProfile = QPushButton("프로필", self)
        self.btnManualArm = QPushButton("수동조작", self)
        self.btnUserProfile.clicked.connect(self.on_user_profile_clicked)
        self.btnManualArm.clicked.connect(self.on_manual_arm_clicked)
        if hasattr(self, "headerLayout"):
            self.headerLayout.addWidget(self.btnUserProfile)
            self.headerLayout.addWidget(self.btnManualArm)

        if hasattr(self, "btnReport"):
            self.btnReport.clicked.connect(self.on_report_clicked)

        if hasattr(self, "btnSaveSettings"):
            self.btnSaveSettings.clicked.connect(self.on_save_settings_clicked)

        if hasattr(self, "checkAlarmEnabled"):
            self.checkAlarmEnabled.toggled.connect(self.on_alarm_enabled_toggled)

        self.initialize_camera_label()
        self.initialize_button_state()
        self.initialize_realtime_labels()
        self.initialize_settings_ui()

    # ---------------------------------------------------------
    # Initialize
    # ---------------------------------------------------------
    def initialize_camera_label(self):
        self.label.setText("Camera Off")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("""
            QLabel {
                background-color: black;
                color: white;
                border: 1px solid #555;
            }
        """)

    def initialize_button_state(self):
        self.btnCalibration.setEnabled(True)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOff.setEnabled(False)

        # 디스크의 전역 baseline만으로 이전 사용자를 암묵적으로 선택하지 않는다.
        # 이번 실행에서 보정을 마치거나 프로필을 명시적으로 불러온 뒤 활성화한다.
        self.btnCamOn.setEnabled(False)
        if any(item.get("occupied") for item in self.user_profile_service.list_profiles()):
            self.set_status("사용자 프로필을 선택하거나 새 보정을 진행해주세요.")
        else:
            self.set_status("초기값 설정을 먼저 진행해주세요.")

    def initialize_realtime_labels(self):
        self.set_label_text("label_Rank", "불안정 자세 TOP 3\n\n1위  -\n2위  -\n3위  -")
        self.set_label_text(["label_totalsec", "label_totalset"], "00:00:00")
        self.set_label_text("label_CurrentPose", "현재 자세\n-")
        # self.set_label_text("label_CurrentFatigue", "현재 피로도\n-")

    # ---------------------------------------------------------
    # Path
    # ---------------------------------------------------------
    def has_baseline(self):
        """현재 활성화된 Vision Process의 baseline만 검사한다."""
        mode = str(PROFILE_MODE).upper()

        if mode in ("POSE_ONLY", "BOTH"):
            pose_baseline = self.resolve_workspace_path(config.BASELINE_PATH)
            if not pose_baseline.exists():
                return False

        if mode in ("FACE_ONLY", "BOTH"):
            face_baseline = self.resolve_workspace_path(config.FACE_BASELINE_PATH)
            if not face_baseline.exists():
                return False

        return True

    def resolve_workspace_path(self, path_text):
        path = Path(path_text)

        if path.is_absolute():
            return path

        return ROOT_DIR / path

    # ---------------------------------------------------------
    # Worker
    # ---------------------------------------------------------
    def ensure_camera_worker(self):
        # Cam Off에서는 worker/Process를 파괴하지 않고 카메라 QThread만 멈춘다.
        # 따라서 기존 worker가 있으면 그대로 재사용한다.
        if self.camera_worker is not None:
            if self.camera_worker.isRunning():
                return

            if not getattr(self.camera_worker, "_vision_resources_shutdown", False):
                self._camera_shutdown_in_progress = False
                self.camera_worker.start()
                return

        self.camera_worker = CameraWorker(parent=self)

        self.camera_worker.frame_changed.connect(self.update_camera_view)
        self.camera_worker.status_changed.connect(self.on_status_changed)
        self.camera_worker.calibration_finished.connect(self.on_calibration_finished)
        self.camera_worker.measurement_started.connect(self.on_measurement_started)
        self.camera_worker.result_changed.connect(self.on_result_changed)
        self.camera_worker.pose_state_changed.connect(self.on_pose_state_changed)
        self.camera_worker.face_state_changed.connect(self.on_face_state_changed)
        self.camera_worker.hardware_changed.connect(self.on_hardware_changed)
        self.camera_worker.hardware_event_changed.connect(self.on_hardware_event_changed)

        self._camera_shutdown_in_progress = False
        self.camera_worker.start()

    def stop_camera_worker(self, full_shutdown=False):
        worker = self.camera_worker
        if worker is None:
            return

        self._camera_shutdown_in_progress = True

        if full_shutdown:
            print("[SHUTDOWN] Main -> CameraWorker 전체 종료")
            # 앱 종료 단계에서는 새 worker signal을 막는다.
            worker.blockSignals(True)
            worker.shutdown()
            # self.camera_worker = None 으로 즉시 파괴하지 않는다.
            # MainWindow child로 유지해 Qt가 정상적인 객체 수명 순서로 정리하게 한다.
            return

        print("[CAM OFF] 카메라 QThread만 종료 시작")
        worker.stop_camera_only()

        # stop() 동안 Main event loop에 쌓였던 queued signal/meta-call을
        # worker 객체가 살아 있는 상태에서 한 번 처리한다. 각 slot은 아래 flag로 무시된다.
        QCoreApplication.processEvents(QEventLoop.ExcludeUserInputEvents, 50)

        self._camera_shutdown_in_progress = False
        print("[CAM OFF] 카메라 QThread 종료 완료 - Vision Process 유지")

    def on_pose_state_changed(self, state):
        """Pose Process -> Main Process 최신 landmark/feature/state 수신 지점."""
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        self.latest_pose_state = state

    def on_face_state_changed(self, state):
        """Face Process -> Main Process 최신 landmark/feature/state 수신 지점."""
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        self.latest_face_state = state

    def on_hardware_changed(self, state):
        """Hardware Process -> Main 최신 State. IMU/Motor 공용 저장소도 갱신한다."""
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        self.latest_hardware_state = state
        self.hardware_state_store.update(state)
        safety = state.get("monitor_arm", {}).get("safety", {}) if isinstance(state, dict) else {}
        if str(state.get("main_mode", "")).upper() == "MEASURING" and safety:
            message = safety.get("tof_alert") or safety.get("reason")
            if message and message != self._last_safety_message:
                self._last_safety_message = message
                self.set_status(str(message))

    def on_hardware_event_changed(self, event):
        """Hardware Process -> Main 순서 보장 Event/ACK 수신 지점."""
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        self.latest_hardware_event = event

        dialog = self.monitor_arm_preparation_dialog
        if dialog is not None and dialog.isVisible():
            dialog.handle_hardware_event(event)

        if isinstance(event, dict):
            event_type = str(event.get("type", "")).upper()
            if event_type in (
                "HARDWARE_CONFIG_STATE",
                "HARDWARE_CONFIG_UPDATED",
                "HARDWARE_CONFIG_RELOADED",
                "HARDWARE_CONFIG_RESET",
                "IMU_OFFSET_SAVED",
            ):
                config_data = event.get("config")
                if isinstance(config_data, dict):
                    self.latest_hardware_config = config_data
            if event_type == "USER_PROFILE_APPLIED":
                if event.get("success"):
                    self.active_profile_slot = self._pending_profile_slot
                    self.monitor_arm_preparation_ready = True
                    self.btnCamOn.setEnabled(True)
                    self.set_status("프로필 적용 완료. 측정 시작 버튼을 눌러주세요.")
                else:
                    QMessageBox.warning(self, "프로필 적용 실패", str(event.get("message", "")))
                self._pending_profile_slot = None
            elif event_type == "MEASUREMENT_STOP_AND_REST_ACK" and self._measurement_stop_pending:
                if event.get("success") is False:
                    QMessageBox.warning(self, "종료 자세 이동", str(event.get("message", "")))
                QTimer.singleShot(1500, self._finish_camera_off)

    def send_hardware_command(self, message):
        """Main Process -> Hardware Process IPC 전송 지점."""
        if self.camera_worker is None:
            return False

        manager = getattr(self.camera_worker, "vision_manager", None)
        if manager is None:
            return False

        return manager.send_hardware_command(message)

    # ---------------------------------------------------------
    # Hardware Runtime State Access
    # ---------------------------------------------------------
    def get_latest_hardware_state(self):
        return self.hardware_state_store.get_hardware_state()

    def get_latest_imu_state(self):
        return self.hardware_state_store.get_imu_state()

    def get_latest_motor_state(self):
        return self.hardware_state_store.get_motor_state()

    # ---------------------------------------------------------
    # Hardware Config API - 향후 PyQt PID/LPF 설정 UI에서 사용
    # ---------------------------------------------------------
    def get_hardware_config(self, reload_from_disk=False):
        if reload_from_disk:
            self.latest_hardware_config = self.hardware_config_service.reload()
        return copy.deepcopy(self.latest_hardware_config)

    def save_hardware_control_config(self, control_patch):
        """
        실행 중이면 Hardware Process가 JSON write owner가 된다.
        Worker가 아직 없으면 Main에서 직접 저장하고 다음 Hardware 시작 시 적용한다.
        """
        if not isinstance(control_patch, dict):
            raise TypeError("control_patch는 dict여야 합니다.")

        worker = self.camera_worker
        manager = getattr(worker, "vision_manager", None) if worker is not None else None
        resources_alive = bool(
            worker is not None
            and manager is not None
            and not getattr(worker, "_vision_resources_shutdown", False)
        )

        if resources_alive:
            return self.send_hardware_command({
                "type": "UPDATE_CONFIG",
                "control": control_patch,
            })

        self.latest_hardware_config = self.hardware_config_service.update_control(
            control_patch
        )
        return True

    def request_hardware_config(self):
        if self.send_hardware_command({"type": "GET_CONFIG"}):
            return True
        self.latest_hardware_config = self.hardware_config_service.reload()
        return False

    def reload_hardware_config(self):
        if self.send_hardware_command({"type": "RELOAD_CONFIG"}):
            return True
        self.latest_hardware_config = self.hardware_config_service.reload()
        return False

    # ---------------------------------------------------------
    # Button Events
    # ---------------------------------------------------------
    def on_user_profile_clicked(self):
        dialog = UserProfileDialog(self.user_profile_service, parent=self)
        if dialog.exec_() != dialog.Accepted:
            return
        slot = dialog.selected_slot
        try:
            mode = str(PROFILE_MODE).upper()
            bundle = self.user_profile_service.activate_profile(
                slot,
                self.resolve_workspace_path(config.BASELINE_PATH),
                self.resolve_workspace_path(config.FACE_BASELINE_PATH),
                mode in ("POSE_ONLY", "BOTH"),
                mode in ("FACE_ONLY", "BOTH"),
            )
            self.ensure_camera_worker()
            self._pending_profile_slot = slot
            self.btnCamOn.setEnabled(False)
            if not self.send_hardware_command({"type": "APPLY_USER_PROFILE", "slot": slot, "profile": bundle}):
                raise RuntimeError("하드웨어 프로세스에 프로필을 전달하지 못했습니다.")
            self.set_status("사용자 프로필을 적용하는 중입니다.")
        except Exception as error:
            self._pending_profile_slot = None
            QMessageBox.warning(self, "프로필 불러오기 실패", str(error))

    def on_manual_arm_clicked(self):
        self.ensure_camera_worker()
        self.camera_worker.start_monitor_arm_preparation()
        self.send_hardware_command({"type": "START_MONITOR_ARM_PREPARATION"})
        dialog = MonitorArmPreparationDialog(
            self.send_hardware_command, self.get_latest_hardware_state,
            parent=self, manual_only=True,
        )
        self.monitor_arm_preparation_dialog = dialog
        dialog.preparation_finished.connect(self.on_manual_arm_finished)
        dialog.show()

    def on_manual_arm_finished(self, _success, _message):
        if self.camera_worker is not None:
            self.camera_worker.finish_monitor_arm_preparation()
        self.monitor_arm_preparation_dialog = None

    def on_camera_on_clicked(self):
        """
        Cam On 버튼:
        baseline.pkl을 기준으로 실시간 자세 측정을 시작한다.
        """

        if not self.has_baseline():
            QMessageBox.warning(
                self,
                "측정 시작 불가",
                "먼저 Calibration을 완료해주세요."
            )
            return

        self.ensure_camera_worker()

        self.btnCalibration.setEnabled(False)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOn.setEnabled(False)
        self.btnCamOff.setEnabled(True)

        self.initialize_realtime_labels()
        self.set_status("실시간 자세 측정을 준비합니다.")

        self.camera_worker.start_measurement()

    # ------------------------
    # calibration
    # ------------------------
    def on_calibration_clicked(self):
        """
        초기값 준비 버튼: 카메라/Pose AI와 모니터암 준비 창을 시작한다.
        """

        self.ensure_camera_worker()

        if self.camera_worker is not None:
            self.camera_worker.start_monitor_arm_preparation()

        self.monitor_arm_preparation_ready = False
        self.send_hardware_command({"type": "START_MONITOR_ARM_PREPARATION"})

        old_dialog = self.monitor_arm_preparation_dialog
        if old_dialog is not None:
            old_dialog.close()

        self.monitor_arm_preparation_dialog = MonitorArmPreparationDialog(
            send_command=self.send_hardware_command,
            get_hardware_state=self.get_latest_hardware_state,
            parent=self,
        )
        self.monitor_arm_preparation_dialog.preparation_finished.connect(
            self.on_monitor_arm_preparation_finished
        )
        self.monitor_arm_preparation_dialog.show()

        self.btnCalibration.setEnabled(False)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOn.setEnabled(False)
        self.btnCamOff.setEnabled(True)

        self.set_status(
            "준비 창에서 모터 자세를 조정하고 ToF/눈 간격 평균을 저장해주세요."
        )

    def on_monitor_arm_preparation_finished(self, success, message):
        if self.camera_worker is not None:
            self.camera_worker.finish_monitor_arm_preparation()
        self.monitor_arm_preparation_ready = bool(success)
        self.btnCalibration.setEnabled(not success)
        self.btnCalibrationStart.setEnabled(bool(success))
        self.btnCamOn.setEnabled(False)
        self.btnCamOff.setEnabled(True)
        self.set_status(
            "모니터암 초기 준비 완료. 이제 초기값 측정시작을 눌러주세요."
            if success
            else message
        )

    def on_calibration_start_clicked(self):
        """
        Calibration Start 버튼:
        실제 baseline feature 수집 시작.
        """

        if self.camera_worker is None or not self.camera_worker.isRunning():
            QMessageBox.warning(
                self,
                "Calibration",
                "먼저 초기값 준비 버튼을 눌러 카메라를 실행해주세요."
            )
            return

        hardware_state = self.get_latest_hardware_state()
        arm_calibration = (
            hardware_state.get("monitor_arm", {}).get("calibration", {})
            if isinstance(hardware_state, dict)
            else {}
        )
        if not (
            self.monitor_arm_preparation_ready
            and arm_calibration.get("session_ready", False)
        ):
            QMessageBox.warning(
                self,
                "초기값 측정 시작 불가",
                "먼저 초기값 준비 창에서 모터 자세 조정과 ToF/눈 간격 5초 평균 "
                "저장을 완료해주세요.",
            )
            return

        self.btnCalibration.setEnabled(False)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOn.setEnabled(False)
        self.btnCamOff.setEnabled(True)

        self.set_status("초기 자세 측정을 시작합니다.")
        self.camera_worker.start_calibration()

    # ------------------------
    # measurement
    # ------------------------
    def on_measurement_started(self, success, message):
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        if success:
            self.btnCalibration.setEnabled(False)
            self.btnCalibrationStart.setEnabled(False)
            self.btnCamOn.setEnabled(False)
            self.btnCamOff.setEnabled(True)
            self.btnManualArm.setEnabled(False)
            self.btnUserProfile.setEnabled(False)

            self.initialize_realtime_labels()
            self.set_status(message)
            return

        QMessageBox.warning(
            self,
            "측정 시작 실패",
            message
        )

        self.btnCalibration.setEnabled(True)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOn.setEnabled(self.has_baseline())
        self.btnCamOff.setEnabled(True)

        self.set_status(message)

    
    def on_result_changed(self, result):
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        posture_type = result.get("posture_type", "-")
        confidence = result.get("confidence", 0.0)
        # fatigue_label = result.get("fatigue_label", "Normal")
        # fatigue_probability = result.get("fatigue_probability", 0.0)
        elapsed_sec = result.get("elapsed_sec", 0)
        rank_text = result.get("rank_text", self.get_empty_rank_text())

        self.set_label_text("label_Rank", rank_text)

        self.set_label_text(
            ["label_totalsec", "label_totalset"],
            self.format_elapsed_time(elapsed_sec)
        )

        self.set_label_text(
            "label_CurrentPose",
            f"현재 자세\n{posture_type}\n{confidence * 100:.1f}%"
        )

        # self.set_label_text(
        #     "label_CurrentFatigue",
        #     f"현재 피로도\n{fatigue_label}\n{fatigue_probability * 100:.1f}%"
        # )

    def get_empty_rank_text(self):
        return (
            "불안정 자세 TOP 3\n\n"
            "1위  -\n"
            "2위  -\n"
            "3위  -"
        )


    def format_elapsed_time(self, elapsed_sec):
        elapsed_sec = int(elapsed_sec)

        hours = elapsed_sec // 3600
        minutes = (elapsed_sec % 3600) // 60
        seconds = elapsed_sec % 60

        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


    def on_camera_off_clicked(self):
        dialog = self.monitor_arm_preparation_dialog
        if dialog is not None and dialog.isVisible():
            dialog.cancel_preparation()
        self.monitor_arm_preparation_dialog = None
        self.monitor_arm_preparation_ready = False
        if self.camera_worker is not None:
            self.camera_worker.stop_measurement()
        self._measurement_stop_pending = True
        self.btnCamOff.setEnabled(False)
        self.set_status("모니터암을 종료 자세로 이동하는 중입니다.")
        if self.send_hardware_command({"type": "MEASUREMENT_STOP_AND_REST"}):
            QTimer.singleShot(15000, self._finish_camera_off)
            return
        self._finish_camera_off()

    def _finish_camera_off(self):
        if not self._measurement_stop_pending:
            return
        self._measurement_stop_pending = False
        self.stop_camera_worker()

        self.label.clear()
        self.label.setText("Camera Off")
        self.label.setAlignment(Qt.AlignCenter)

        self.btnCalibration.setEnabled(True)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOn.setEnabled(self.has_baseline())
        self.btnCamOff.setEnabled(False)
        self.btnManualArm.setEnabled(True)
        self.btnUserProfile.setEnabled(True)

        if self.has_baseline():
            self.set_status("카메라가 종료되었습니다.")
        else:
            self.set_status("초기값설정 먼저 진행해주세요.")

    # ---------------------------------------------------------
    # Worker Signals
    # ---------------------------------------------------------
    def update_camera_view(self, q_image):
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        pixmap = QPixmap.fromImage(q_image)

        self.label.setPixmap(
            pixmap.scaled(
                self.label.width(),
                self.label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        )

    def on_status_changed(self, message):
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        self.set_status(message)

    def on_calibration_finished(self, success, message):
        if self._camera_shutdown_in_progress or self._app_closing:
            return
        if success:
            QMessageBox.information(
                self,
                "초기값 측정 완료",
                message
            )

            self.btnCalibration.setEnabled(True)
            self.btnCalibrationStart.setEnabled(False)
            self.btnCamOn.setEnabled(True)
            self.btnCamOff.setEnabled(True)

            self.set_status("초기값 설정이 완료되었습니다.")
            answer = QMessageBox.question(
                self,
                "프로필 저장",
                "이번 보정 정보를 사용자 프로필로 저장할까요?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if answer == QMessageBox.Yes:
                self.save_current_profile()

        else:
            QMessageBox.warning(
                self,
                "Calibration 실패",
                message
            )

            self.btnCalibration.setEnabled(True)
            self.btnCalibrationStart.setEnabled(True)
            # Calibration 실패 시 기존 baseline은 덮어쓰지 않으므로
            # 이전 정상 baseline이 남아 있으면 측정은 계속 가능하다.
            self.btnCamOn.setEnabled(self.has_baseline())
            self.btnCamOff.setEnabled(True)

            self.set_status("초기값 설정이 실패했습니다. 기존 기준값이 있으면 계속 사용할 수 있습니다.")

    def save_current_profile(self):
        dialog = UserProfileDialog(self.user_profile_service, save_mode=True, parent=self)
        if dialog.exec_() != dialog.Accepted:
            return
        try:
            mode = str(PROFILE_MODE).upper()
            metadata = self.user_profile_service.save_profile(
                dialog.selected_slot,
                dialog.selected_name,
                self.resolve_workspace_path(config.BASELINE_PATH),
                self.resolve_workspace_path(config.FACE_BASELINE_PATH),
                self.get_latest_hardware_state(),
                mode in ("POSE_ONLY", "BOTH"),
                mode in ("FACE_ONLY", "BOTH"),
            )
            self.active_profile_slot = dialog.selected_slot
            QMessageBox.information(self, "프로필 저장 완료", f"'{metadata['name']}' 프로필을 저장했습니다.")
        except Exception as error:
            QMessageBox.warning(self, "프로필 저장 실패", str(error))

    # ---------------------------------------------------------
    # UI Helper
    # ---------------------------------------------------------
    def set_label_text(self, object_names, text):
        if isinstance(object_names, str):
            object_names = [object_names]

        for object_name in object_names:
            widget = getattr(self, object_name, None)

            if widget is not None:
                widget.setText(text)
                widget.setAlignment(Qt.AlignCenter)
                return True

        return False

    def set_status(self, message):
        print(message)

        if hasattr(self, "labelStatus") and not self._app_closing:
            print("[STATUS] labelStatus.setText 호출 전")
            self.labelStatus.setText(message)
            print("[STATUS] labelStatus.setText 호출 후")

    def closeEvent(self, event):
        print("[SHUTDOWN] closeEvent 진입")
        dialog = self.monitor_arm_preparation_dialog
        if dialog is not None and dialog.isVisible():
            dialog.close()
        self._app_closing = True
        self.stop_camera_worker(full_shutdown=True)

        if self.streamlit_process is not None:
            if self.streamlit_process.poll() is None:
                self.streamlit_process.terminate()

        print("[SHUTDOWN] closeEvent 종료")
        event.accept()

    # ---------------------------------------------------------
    # Settings
    # ---------------------------------------------------------
    def initialize_settings_ui(self):
        """
        프로그램 시작 시 설정 JSON을 불러와서 설정 탭 UI에 반영한다.
        JSON 파일이 없으면 기본값으로 JSON 파일을 자동 생성한다.
        """

        settings = self.settings_manager.load()
        self.current_alarm_settings = settings

        self.apply_settings_to_ui(settings)

        if hasattr(self, "checkAlarmEnabled"):
            self.on_alarm_enabled_toggled(self.checkAlarmEnabled.isChecked())


    def apply_settings_to_ui(self, settings):
        """
        AlarmSettings 값을 실제 UI 위젯에 반영한다.
        """

        if hasattr(self, "checkAlarmEnabled"):
            self.checkAlarmEnabled.setChecked(settings.alarm_enabled)

        self.set_spinbox_value(
            "spinBadPostureDurationSec",
            settings.bad_posture_duration_sec
        )

        # self.set_spinbox_value(
        #     "spinFatigueDurationSec",
        #     settings.fatigue_duration_sec
        # )


        ## 추가
        # posture_Hardware_count 자세 LED/부저 반복 횟수
        self.set_spinbox_value(
            "spinPostureAlertCount",
            settings.posture_Hardware_count
        )

        # fatigue_Hardware_count 졸음 LED/부저 반복 횟수
        # self.set_spinbox_value(
        #     "spinDrowsyAlertCount",
        #     settings.fatigue_Hardware_count
        # )

        # posture_Strong_limit 자세 강함 알림 횟수
        self.set_spinbox_value(
            "spinPostureStrongLimit",
            settings.posture_Strong_limit
        )

        # fatigue_Strong_limit 졸음 강함 알림 횟수
        # self.set_spinbox_value(
        #     "spinDrowsyStrongLimit",
        #     settings.fatigue_Strong_limit
        # )


        # strong_alert_cooldown_min 강한 알림 후 쿨타임
        self.set_spinbox_value(
            "spinStrongAlertCooldownMin",
            settings.strong_alert_cooldown_min
        )





    def collect_settings_from_ui(self):
        """
        현재 설정 탭 UI 값을 읽어서 AlarmSettings 객체로 만든다.
        """

        alarm_enabled = True

        if hasattr(self, "checkAlarmEnabled"):
            alarm_enabled = self.checkAlarmEnabled.isChecked()

        return AlarmSettings(
            alarm_enabled=alarm_enabled,
            bad_posture_duration_sec=self.get_spinbox_value(
                "spinBadPostureDurationSec",
                default_value=5
            ),
            # fatigue_duration_sec=self.get_spinbox_value(
            #     "spinFatigueDurationSec",
            #     default_value=5
            # ),

            ## 추가
            posture_Hardware_count=self.get_spinbox_value(
                "spinPostureAlertCount",
                default_value=5
            ),
            # fatigue_Hardware_count=self.get_spinbox_value(
            #     "spinDrowsyAlertCount",
            #     default_value=3
            # ),
            posture_Strong_limit=self.get_spinbox_value(
                "spinPostureStrongLimit",
                default_value=3
            ),
            # fatigue_Strong_limit=self.get_spinbox_value(
            #     "spinDrowsyStrongLimit",
            #     default_value=2
            # ),
            strong_alert_cooldown_min=self.get_spinbox_value(
                "spinStrongAlertCooldownMin",
                default_value=5
            ),
        )


    def on_save_settings_clicked(self):
        settings = self.collect_settings_from_ui()

        # 기존 POCO 방식 그대로 JSON에는 항상 저장한다.
        self.settings_manager.save(
            settings
        )

        self.current_alarm_settings = settings

        # Hardware Process가 이미 실행 중이면
        # 같은 값을 IPC로 보내 Runtime에도 즉시 반영한다.
        #
        # Hardware Process가 아직 실행되지 않았다면
        # send_hardware_command()는 False를 반환할 뿐
        # CameraWorker를 새로 실행하지 않는다.
        #
        # 이후 Hardware Process가 시작될 때
        # alarm_settings.json을 읽으므로 저장값은 정상 적용된다.
        self.send_hardware_command(
            {
                "type": "UPDATE_ALARM_SETTINGS",
                "settings": settings.to_dict(),
            }
        )

        self.set_status(
            "설정이 저장되었습니다."
        )

        # 절대 여기서 ensure_camera_worker() 호출하지 않기
        # 설정 저장만 했는데 카메라가 켜지는 원인이 됨

        QMessageBox.information(
            self,
            "설정 저장",
            "알림 설정이 저장되었습니다."
        )


    def on_alarm_enabled_toggled(self, checked):
        """
        알림 사용 체크박스 상태에 따라
        알림 관련 SpinBox들을 활성화/비활성화한다.
        """

        spinbox_names = [
            "spinBadPostureDurationSec",
            # "spinFatigueDurationSec",
            "spinRepeatAlarmSec",
            "spinPostureAlertCount",
            # "spinDrowsyAlertCount",
            "spinPostureStrongLimit",
            # "spinDrowsyStrongLimit",
            "spinStrongAlertCooldownMin"
        ]

        for object_name in spinbox_names:
            widget = getattr(self, object_name, None)

            if widget is not None:
                widget.setEnabled(checked)


    def set_spinbox_value(self, object_name, value):
        """
        SpinBox에 값을 넣을 때 UI에 설정된 min/max 범위를 벗어나지 않게 보정한다.
        """

        widget = getattr(self, object_name, None)

        if widget is None:
            return

        value = int(value)
        value = max(widget.minimum(), min(widget.maximum(), value))

        widget.setValue(value)


    def get_spinbox_value(self, object_name, default_value):
        """
        SpinBox 값을 안전하게 가져온다.
        """

        widget = getattr(self, object_name, None)

        if widget is None:
            return default_value

        return int(widget.value())


    def get_current_alarm_settings(self):
        """
        다른 로직에서 현재 알림 설정이 필요할 때 사용한다.
        """

        if self.current_alarm_settings is None:
            self.current_alarm_settings = self.settings_manager.load()

        return self.current_alarm_settings


    # ------------------------------------------------------------------
    # Streamlit Report
    # ------------------------------------------------------------------
    def get_streamlit_app_path(self):
        """
        현재 프로젝트 구조 기준:

        WorkSpace/
        pyQt/
            mainpyQt.py
        streamlit/
            app.py
        """
        return ROOT_DIR / "streamlit" / "app.py"


    def get_streamlit_url(self):
        return f"http://localhost:{self.streamlit_port}"


    def on_report_clicked(self):
        """
        Report 버튼 클릭 시 Streamlit 리포트 서버를 실행하고 브라우저를 연다.

        이미 Streamlit 서버가 실행 중이면 서버를 또 띄우지 않고
        브라우저만 다시 연다.
        """

        app_path = self.get_streamlit_app_path()

        if not app_path.exists():
            QMessageBox.warning(
                self,
                "Report Error",
                f"app.py 파일을 찾을 수 없습니다:\n{app_path}"
            )
            return

        # 이미 Streamlit 서버가 켜져 있으면 브라우저만 다시 열기
        if self.streamlit_process is not None and self.streamlit_process.poll() is None:
            self.open_streamlit_browser()
            return

        command = [
            sys.executable,
            "-m",
            "streamlit",
            "run",
            str(app_path),
            "--server.headless=true",
            f"--server.port={self.streamlit_port}",
            "--browser.gatherUsageStats=false",
        ]

        try:
            self.streamlit_process = subprocess.Popen(
                command,
                cwd=str(app_path.parent),
                creationflags=self.get_subprocess_creationflags()
            )

        except Exception as e:
            QMessageBox.warning(
                self,
                "Report Error",
                f"Streamlit 실행에 실패했습니다:\n{e}"
            )
            return

        # Streamlit 서버가 뜰 시간을 조금 준 뒤 브라우저 열기
        QTimer.singleShot(2500, self.open_streamlit_browser)


    def open_streamlit_browser(self):
        url = self.get_streamlit_url()
        current_os = platform.system()

        if current_os == "Windows":
            self.open_windows_browser(url)

        elif current_os == "Linux":
            self.open_linux_browser(url)

        else:
            webbrowser.open_new(url)


    def open_windows_browser(self, url):
        """
        Windows에서는 Chrome 또는 Edge가 있으면 앱 창 형태로 열고,
        못 찾으면 기본 브라우저로 연다.
        """

        browser_path = self.find_windows_browser()

        if browser_path is not None:
            subprocess.Popen(
                [browser_path, f"--app={url}"],
                creationflags=self.get_subprocess_creationflags()
            )
            return

        webbrowser.open_new(url)


    def find_windows_browser(self):
        candidates = [
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
        ]

        for path in candidates:
            if Path(path).exists():
                return path

        return None


    def open_linux_browser(self, url):
        """
        Raspberry Pi/Linux에서는 Chromium을 전체화면 키오스크 모드로 실행한다.
        """

        browser_path = (
            shutil.which("chromium-browser")
            or shutil.which("chromium")
            or shutil.which("google-chrome")
        )

        if browser_path is None:
            QMessageBox.warning(
                self,
                "Report Error",
                "Chromium 브라우저를 찾을 수 없습니다.\n"
                "라즈베리파이에서 아래 명령어로 설치해주세요.\n\n"
                "sudo apt install -y chromium-browser\n"
                "또는\n"
                "sudo apt install -y chromium"
            )
            return

        env = os.environ.copy()

        # SSH에서 실행하더라도 라즈베리파이 GUI 화면에 띄우기 위한 기본값
        if "DISPLAY" not in env:
            env["DISPLAY"] = ":0"

        subprocess.Popen(
            [
                browser_path,
                "--kiosk",
                "--noerrdialogs",
                "--disable-infobars",
                "--disable-session-crashed-bubble",
                url
            ],
            env=env
        )


    def get_subprocess_creationflags(self):
        if platform.system() == "Windows":
            return subprocess.CREATE_NO_WINDOW

        return 0

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()


if __name__ == "__main__":
    # native C/C++ 계층에서 Segmentation fault가 재발하면 Python stack을 stderr에 남긴다.
    faulthandler.enable(all_threads=True)

    app = QApplication(sys.argv)

    window = MainWindow()

    if platform.system() == "Linux":
        window.showFullScreen()   # 라즈베리파이에서는 전체화면
    else:
        window.show()             # 윈도우 개발환경에서는 일반 창

    # sys.exit(app.exec_())
    exit_code = app.exec_()

    print("[SHUTDOWN] 16. QApplication event loop 종료")
    print("[SHUTDOWN] exit_code =", exit_code)

    sys.exit(exit_code)

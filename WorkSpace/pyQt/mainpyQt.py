import os
import sys
from pathlib import Path

import platform
import shutil
import subprocess
import webbrowser

from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR))

import modules.config as config
from camera_worker import CameraWorker
from modules.app_settings import SettingsManager, AlarmSettings
from services.hardware_controller import HardwareController

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

    def __init__(self, hardware_controller=None, parent=None):
        super().__init__(parent)

        ui_path = Path(__file__).parent / "ui" / "pocoApplication_Qss.ui"
        # ui_path = Path(__file__).parent / "ui" / "pocoApplication.ui"
        uic.loadUi(str(ui_path), self)

        self.camera_worker = None
        self.streamlit_process = None
        self.streamlit_port = 8501

        # 설정 저장 경로
        self.settings_manager = SettingsManager(
            ROOT_DIR / "data" / "settings" / "alarm_settings.json"
        )

        self.hardware_controller = HardwareController(
            enabled=config.HARDWARE_ENABLED,
            serial_port=config.HARDWARE_SERIAL_PORT,
            baud_rate=config.HARDWARE_BAUD_RATE,
            timeout=config.HARDWARE_TIMEOUT,
        )

        self.current_alarm_settings = None

        self.btnCalibration.clicked.connect(self.on_calibration_clicked)
        self.btnCalibrationStart.clicked.connect(self.on_calibration_start_clicked)
        self.btnCamOn.clicked.connect(self.on_camera_on_clicked)
        self.btnCamOff.clicked.connect(self.on_camera_off_clicked)

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

        self.btnCamOn.setEnabled(self.has_baseline())

        if self.has_baseline():
            self.set_status("자세 측정을 시작할 수 있습니다.")
        else:
            self.set_status("초기값 설정을 먼저 진행해주세요.")

    def initialize_realtime_labels(self):
        self.set_label_text("label_Rank", "불안정 자세 TOP 3\n\n1위  -\n2위  -\n3위  -")
        self.set_label_text(["label_totalsec", "label_totalset"], "00:00:00")
        self.set_label_text("label_CurrentPose", "현재 자세\n-")
        self.set_label_text("label_CurrentFatigue", "현재 피로도\n-")

    # ---------------------------------------------------------
    # Path
    # ---------------------------------------------------------
    def has_baseline(self):
        return self.resolve_workspace_path(config.BASELINE_PATH).exists()

    def resolve_workspace_path(self, path_text):
        path = Path(path_text)

        if path.is_absolute():
            return path

        return ROOT_DIR / path

    # ---------------------------------------------------------
    # Worker
    # ---------------------------------------------------------
    def ensure_camera_worker(self):
        if self.camera_worker is not None and self.camera_worker.isRunning():
            return

        self.camera_worker = CameraWorker(
            hardware_controller=self.hardware_controller
        )

        self.camera_worker.frame_changed.connect(self.update_camera_view)
        self.camera_worker.status_changed.connect(self.on_status_changed)
        self.camera_worker.calibration_finished.connect(self.on_calibration_finished)
        self.camera_worker.measurement_started.connect(self.on_measurement_started)
        self.camera_worker.result_changed.connect(self.on_result_changed)

        self.camera_worker.start()

    def stop_camera_worker(self):
        if self.camera_worker is not None:
            self.camera_worker.stop()
            self.camera_worker = None

    # ---------------------------------------------------------
    # Button Events
    # ---------------------------------------------------------
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
        Calibration 버튼:
        카메라 프리뷰를 켜고 사용자가 자세를 준비하는 단계.
        """

        self.ensure_camera_worker()

        if self.camera_worker is not None:
            self.camera_worker.start_preview()

        self.btnCalibration.setEnabled(False)
        self.btnCalibrationStart.setEnabled(True)
        self.btnCamOn.setEnabled(False)
        self.btnCamOff.setEnabled(True)

        self.set_status("바른 자세를 잡은 뒤 초기값준비 버튼을 눌러주세요.")

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
        if success:
            self.btnCalibration.setEnabled(False)
            self.btnCalibrationStart.setEnabled(False)
            self.btnCamOn.setEnabled(False)
            self.btnCamOff.setEnabled(True)

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
        posture_type = result.get("posture_type", "-")
        confidence = result.get("confidence", 0.0)
        fatigue_label = result.get("fatigue_label", "Normal")
        fatigue_probability = result.get("fatigue_probability", 0.0)
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

        self.set_label_text(
            "label_CurrentFatigue",
            f"현재 피로도\n{fatigue_label}\n{fatigue_probability * 100:.1f}%"
        )

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
        self.stop_camera_worker()

        self.label.clear()
        self.label.setText("Camera Off")
        self.label.setAlignment(Qt.AlignCenter)

        self.btnCalibration.setEnabled(True)
        self.btnCalibrationStart.setEnabled(False)
        self.btnCamOn.setEnabled(self.has_baseline())
        self.btnCamOff.setEnabled(False)

        if self.has_baseline():
            self.set_status("카메라가 종료되었습니다.")
        else:
            self.set_status("초기값설정 먼저 진행해주세요.")

    # ---------------------------------------------------------
    # Worker Signals
    # ---------------------------------------------------------
    def update_camera_view(self, q_image):
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
        self.set_status(message)

    def on_calibration_finished(self, success, message):
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

        else:
            QMessageBox.warning(
                self,
                "Calibration 실패",
                message
            )

            self.btnCalibration.setEnabled(True)
            self.btnCalibrationStart.setEnabled(True)
            self.btnCamOn.setEnabled(False)
            self.btnCamOff.setEnabled(True)

            self.set_status("초기값 설정이 실패했습니다.")

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

        if hasattr(self, "labelStatus"):
            self.labelStatus.setText(message)

    def closeEvent(self, event):
        self.stop_camera_worker()

        if self.hardware_controller is not None:
            self.hardware_controller.close()

        if self.streamlit_process is not None:
            if self.streamlit_process.poll() is None:
                self.streamlit_process.terminate()

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

        self.set_spinbox_value(
            "spinFatigueDurationSec",
            settings.fatigue_duration_sec
        )


        ## 추가
        # posture_Hardware_count 자세 LED/부저 반복 횟수
        self.set_spinbox_value(
            "spinPostureAlertCount",
            settings.posture_Hardware_count
        )

        # fatigue_Hardware_count 졸음 LED/부저 반복 횟수
        self.set_spinbox_value(
            "spinDrowsyAlertCount",
            settings.fatigue_Hardware_count
        )

        # posture_Strong_limit 자세 강함 알림 횟수
        self.set_spinbox_value(
            "spinPostureStrongLimit",
            settings.posture_Strong_limit
        )

        # fatigue_Strong_limit 졸음 강함 알림 횟수
        self.set_spinbox_value(
            "spinDrowsyStrongLimit",
            settings.fatigue_Strong_limit
        )


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
            fatigue_duration_sec=self.get_spinbox_value(
                "spinFatigueDurationSec",
                default_value=5
            ),

            ## 추가
            posture_Hardware_count=self.get_spinbox_value(
                "spinPostureAlertCount",
                default_value=5
            ),
            fatigue_Hardware_count=self.get_spinbox_value(
                "spinDrowsyAlertCount",
                default_value=3
            ),
            posture_Strong_limit=self.get_spinbox_value(
                "spinPostureStrongLimit",
                default_value=3
            ),
            fatigue_Strong_limit=self.get_spinbox_value(
                "spinDrowsyStrongLimit",
                default_value=2
            ),
            strong_alert_cooldown_min=self.get_spinbox_value(
                "spinStrongAlertCooldownMin",
                default_value=5
            ),
        )


    def on_save_settings_clicked(self):
        settings = self.collect_settings_from_ui()

        self.settings_manager.save(settings)
        self.current_alarm_settings = settings

        self.set_status("설정이 저장되었습니다.")

        # 절대 여기서 ensure_camera_worker() 호출하지 않기
        # 설정 저장만 했는데 카메라가 켜지는 원인이 됨

        if self.hardware_controller is not None:
            print("[MainWindow] 하드웨어 컨트롤러에 새로운 설정값 적용")
            self.hardware_controller.set_hardware_Values(settings)

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
            "spinFatigueDurationSec",
            "spinRepeatAlarmSec",
            "spinPostureAlertCount",
            "spinDrowsyAlertCount",
            "spinPostureStrongLimit",
            "spinDrowsyStrongLimit",
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

        profile_dir = Path.home() / ".config" / "visionposecoach-chromium"
        profile_dir.mkdir(parents=True, exist_ok=True)

        subprocess.Popen(
            [
                browser_path,
                "--kiosk",
                "--no-first-run",
                "--no-default-browser-check",
                "--noerrdialogs",
                "--disable-infobars",
                "--disable-session-crashed-bubble",
                "--password-store=basic",
                f"--user-data-dir={profile_dir}",
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
    app = QApplication(sys.argv)

    window = MainWindow()

    if platform.system() == "Linux":
        window.showFullScreen()   # 라즈베리파이에서는 전체화면
    else:
        window.show()             # 윈도우 개발환경에서는 일반 창

    sys.exit(app.exec_())
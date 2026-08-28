import copy
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from services.hardware_constants import (
    IMU_DEADBAND_DEG,
    IMU_LPF_ALPHA,
    PID_DERIVATIVE_LPF_ALPHA,
    PID_INTEGRAL_LIMIT_RAD_SEC,
    PID_OUTPUT_LIMIT_DEG_S,
    PID_OUTPUT_LPF_ALPHA,
    PITCH_KD,
    PITCH_KI,
    PITCH_KP,
    ROLL_KD,
    ROLL_KI,
    ROLL_KP,
)


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = WORKSPACE_DIR / "data" / "settings" / "hardware_control.json"


# 하드웨어 배선/통신 기본값(IR pin, I2C bus/address, sample Hz 등)은
# hardware_constants.py에만 둔다. JSON에는 PyQt에서 실제로 조절할 값만 저장한다.
DEFAULT_HARDWARE_CONFIG = {
    "version": 3,
    "calibration": {
        "imu": {
            "pitch_offset_deg": 0.0,
            "roll_offset_deg": 0.0,
            "sample_count": 0,
            "calibrated_at": None,
        }
    },
    "control": {
        "imu": {
            "lpf_alpha": IMU_LPF_ALPHA,
            "deadband_deg": IMU_DEADBAND_DEG,
        },
        "pid": {
            "pitch": {"kp": PITCH_KP, "ki": PITCH_KI, "kd": PITCH_KD},
            "roll": {"kp": ROLL_KP, "ki": ROLL_KI, "kd": ROLL_KD},
            "output_limit_deg_s": PID_OUTPUT_LIMIT_DEG_S,
            "integral_limit_rad_sec": PID_INTEGRAL_LIMIT_RAD_SEC,
            "derivative_lpf_alpha": PID_DERIVATIVE_LPF_ALPHA,
            "output_lpf_alpha": PID_OUTPUT_LPF_ALPHA,
        },
        "motor": {
            "command_hz": 10.0,
            "pid_speed_deadband_deg_s": 0.25,
            "motor3": {
                "enabled": True,
                "direction_sign": 1.0,
            },
            "motor4": {
                "enabled": True,
                "direction_sign": 1.0,
            },
        },
    },
}


def _deep_merge(base, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def _clamp(value, minimum, maximum):
    return max(minimum, min(float(value), maximum))


def _normalize_legacy_loaded(data):
    """예전 JSON을 읽어도 새 구조로 자동 정리한다.

    예전 control.ir 및 control.imu의 bus/address/sample_hz/calibration_sec는
    이제 코드 상수가 기준이므로 버린다. lpf/deadband만 유지한다.
    """
    if not isinstance(data, dict):
        return {}

    data = copy.deepcopy(data)
    control = data.get("control")
    if not isinstance(control, dict):
        return data

    old_imu = control.get("imu")
    if isinstance(old_imu, dict):
        control["imu"] = {
            key: old_imu[key]
            for key in ("lpf_alpha", "deadband_deg")
            if key in old_imu
        }

    control.pop("ir", None)

    # v2까지 Motor4는 미구현(pass) 상태였으므로, v3에서는 실제 제어가
    # 기본 활성화되도록 자동 마이그레이션한다. 사용자가 v3 이후 직접
    # disabled로 저장한 값은 그대로 존중한다.
    version = int(data.get("version", 0) or 0)
    motor = control.get("motor")
    if isinstance(motor, dict):
        motor4 = motor.get("motor4")
        if isinstance(motor4, dict):
            if version < 3 or motor4.get("pass") is True:
                motor4["enabled"] = True
            motor4.pop("pass", None)
            motor4.setdefault("direction_sign", 1.0)

    return data


class HardwareConfigService:
    """PID/Filter/Motor tuning + IMU Calibration 기록 전용 JSON 관리자.

    구분
      - hardware_constants.py: 배선/통신/샘플링 등 개발자 고정값
      - hardware_control.json: PyQt에서 변경 가능한 튜닝값
      - servo_calibration_result.json: Servo 기계적 Calibration
    """

    def __init__(self, path=DEFAULT_CONFIG_PATH):
        self.path = Path(path)
        self._cache = None

    def create_default(self):
        return copy.deepcopy(DEFAULT_HARDWARE_CONFIG)

    def load(self, create_if_missing=True):
        if not self.path.exists():
            data = self.create_default()
            self._validate(data)
            self._cache = data
            if create_if_missing:
                self.save(data)
            return copy.deepcopy(data)

        with self.path.open("r", encoding="utf-8") as file:
            loaded = json.load(file)

        loaded = _normalize_legacy_loaded(loaded)
        data = self.create_default()
        if isinstance(loaded, dict):
            _deep_merge(data, loaded)

        self._validate(data)
        self._cache = data
        return copy.deepcopy(data)

    def save(self, data):
        data = copy.deepcopy(data)
        self._validate(data)

        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temp_name = tempfile.mkstemp(
            prefix=self.path.stem + "_",
            suffix=".tmp",
            dir=str(self.path.parent),
            text=True,
        )

        try:
            with os.fdopen(fd, "w", encoding="utf-8") as file:
                json.dump(data, file, indent=4, ensure_ascii=False)
                file.flush()
                os.fsync(file.fileno())
            os.replace(temp_name, self.path)
        finally:
            if os.path.exists(temp_name):
                try:
                    os.remove(temp_name)
                except OSError:
                    pass

        self._cache = copy.deepcopy(data)
        return copy.deepcopy(data)

    def reload(self):
        return self.load(create_if_missing=True)

    def snapshot(self):
        if self._cache is None:
            return self.load()
        return copy.deepcopy(self._cache)

    def get_control(self):
        return copy.deepcopy(self.snapshot()["control"])

    def get_imu_calibration(self):
        return copy.deepcopy(self.snapshot()["calibration"]["imu"])

    def update(self, patch):
        if not isinstance(patch, dict):
            raise TypeError("Hardware config patch는 dict여야 합니다.")
        data = self.load()
        _deep_merge(data, _normalize_legacy_loaded(patch))
        return self.save(data)

    def update_control(self, patch):
        if not isinstance(patch, dict):
            raise TypeError("control patch는 dict여야 합니다.")
        return self.update({"control": patch})

    def update_imu_calibration(self, pitch_offset_deg, roll_offset_deg, sample_count=0):
        data = self.load()
        calibration = data["calibration"]["imu"]
        calibration["pitch_offset_deg"] = float(pitch_offset_deg)
        calibration["roll_offset_deg"] = float(roll_offset_deg)
        calibration["sample_count"] = int(sample_count)
        calibration["calibrated_at"] = datetime.now(timezone.utc).isoformat()
        return self.save(data)

    def clear_imu_calibration_record(self):
        data = self.load()
        data["calibration"]["imu"] = copy.deepcopy(
            DEFAULT_HARDWARE_CONFIG["calibration"]["imu"]
        )
        return self.save(data)

    def reset_defaults(self, preserve_imu_calibration=True):
        previous = self.load()
        data = self.create_default()
        if preserve_imu_calibration:
            data["calibration"]["imu"] = copy.deepcopy(
                previous.get("calibration", {}).get("imu", data["calibration"]["imu"])
            )
        return self.save(data)

    def _validate(self, data):
        if not isinstance(data, dict):
            raise ValueError("Hardware config root는 object여야 합니다.")

        data["version"] = 3

        calibration = data.setdefault("calibration", {}).setdefault("imu", {})
        calibration["pitch_offset_deg"] = float(calibration.get("pitch_offset_deg", 0.0))
        calibration["roll_offset_deg"] = float(calibration.get("roll_offset_deg", 0.0))
        calibration["sample_count"] = max(0, int(calibration.get("sample_count", 0)))
        calibrated_at = calibration.get("calibrated_at")
        calibration["calibrated_at"] = calibrated_at if calibrated_at else None

        control = data.setdefault("control", {})
        control.pop("ir", None)

        imu = control.setdefault("imu", {})
        imu["lpf_alpha"] = _clamp(imu.get("lpf_alpha", IMU_LPF_ALPHA), 0.0, 1.0)
        imu["deadband_deg"] = max(0.0, float(imu.get("deadband_deg", IMU_DEADBAND_DEG)))
        for key in ("bus", "address", "sample_hz", "calibration_sec"):
            imu.pop(key, None)

        pid = control.setdefault("pid", {})
        for axis, defaults in (
            ("pitch", {"kp": PITCH_KP, "ki": PITCH_KI, "kd": PITCH_KD}),
            ("roll", {"kp": ROLL_KP, "ki": ROLL_KI, "kd": ROLL_KD}),
        ):
            axis_cfg = pid.setdefault(axis, {})
            axis_cfg["kp"] = float(axis_cfg.get("kp", defaults["kp"]))
            axis_cfg["ki"] = float(axis_cfg.get("ki", defaults["ki"]))
            axis_cfg["kd"] = float(axis_cfg.get("kd", defaults["kd"]))

        pid["output_limit_deg_s"] = max(
            0.1, float(pid.get("output_limit_deg_s", PID_OUTPUT_LIMIT_DEG_S))
        )
        pid["integral_limit_rad_sec"] = max(
            0.0, float(pid.get("integral_limit_rad_sec", PID_INTEGRAL_LIMIT_RAD_SEC))
        )
        pid["derivative_lpf_alpha"] = _clamp(
            pid.get("derivative_lpf_alpha", PID_DERIVATIVE_LPF_ALPHA), 0.0, 1.0
        )
        pid["output_lpf_alpha"] = _clamp(
            pid.get("output_lpf_alpha", PID_OUTPUT_LPF_ALPHA), 0.0, 1.0
        )

        motor = control.setdefault("motor", {})
        motor["command_hz"] = max(1.0, float(motor.get("command_hz", 10.0)))
        motor["pid_speed_deadband_deg_s"] = max(
            0.0, float(motor.get("pid_speed_deadband_deg_s", 0.25))
        )

        motor3 = motor.setdefault("motor3", {})
        motor3["enabled"] = bool(motor3.get("enabled", True))
        direction_sign = float(motor3.get("direction_sign", 1.0))
        motor3["direction_sign"] = 1.0 if direction_sign >= 0.0 else -1.0

        motor4 = motor.setdefault("motor4", {})
        motor4["enabled"] = bool(motor4.get("enabled", True))
        direction_sign = float(motor4.get("direction_sign", 1.0))
        motor4["direction_sign"] = 1.0 if direction_sign >= 0.0 else -1.0
        motor4.pop("pass", None)

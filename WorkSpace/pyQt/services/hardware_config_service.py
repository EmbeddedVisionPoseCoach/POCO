import copy
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from services.hardware_constants import (
    IMU_ADDRESS,
    IMU_BUS,
    IMU_CALIBRATION_SEC,
    IMU_DEADBAND_DEG,
    IMU_LPF_ALPHA,
    IMU_SAMPLE_HZ,
    IR_ACTIVE_LOW,
    IR_CHECK_TIMEOUT_SEC,
    IR_LOST_GRACE_SEC,
    IR_PIN,
    IR_SAMPLE_HZ,
    IR_STABLE_DETECT_SEC,
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


DEFAULT_HARDWARE_CONFIG = {
    "version": 1,
    "calibration": {
        "imu": {
            "pitch_offset_deg": 0.0,
            "roll_offset_deg": 0.0,
            "sample_count": 0,
            "calibrated_at": None,
        }
    },
    "control": {
        "ir": {
            "pin": IR_PIN,
            "active_low": IR_ACTIVE_LOW,
            "sample_hz": IR_SAMPLE_HZ,
            "stable_detect_sec": IR_STABLE_DETECT_SEC,
            "lost_grace_sec": IR_LOST_GRACE_SEC,
            "check_timeout_sec": IR_CHECK_TIMEOUT_SEC,
        },
        "imu": {
            "bus": IMU_BUS,
            "address": IMU_ADDRESS,
            "sample_hz": IMU_SAMPLE_HZ,
            "calibration_sec": IMU_CALIBRATION_SEC,
            "lpf_alpha": IMU_LPF_ALPHA,
            "deadband_deg": IMU_DEADBAND_DEG,
        },
        "pid": {
            "pitch": {
                "kp": PITCH_KP,
                "ki": PITCH_KI,
                "kd": PITCH_KD,
            },
            "roll": {
                "kp": ROLL_KP,
                "ki": ROLL_KI,
                "kd": ROLL_KD,
            },
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
                "enabled": False,
                "pass": True,
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


class HardwareConfigService:
    """Hardware 제어/Calibration JSON의 단일 관리 계층.

    원칙
      - PID/LPF/Deadband/IR/Motor 제어 설정과 IMU Offset 기록을 한 파일에 저장한다.
      - Runtime sensor 값(IR detected 등)은 저장하지 않는다.
      - 실행 중에는 Hardware Process가 write owner가 되고, PyQt는 IPC로 update를 요청한다.
      - save()는 temporary file + os.replace()로 원자적으로 교체한다.
      - 새 설정 키가 추가돼도 기존 JSON을 유지할 수 있도록 deep merge한다.
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
        _deep_merge(data, patch)
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

        data["version"] = int(data.get("version", 1))

        calibration = data.setdefault("calibration", {}).setdefault("imu", {})
        calibration["pitch_offset_deg"] = float(calibration.get("pitch_offset_deg", 0.0))
        calibration["roll_offset_deg"] = float(calibration.get("roll_offset_deg", 0.0))
        calibration["sample_count"] = max(0, int(calibration.get("sample_count", 0)))
        calibrated_at = calibration.get("calibrated_at")
        calibration["calibrated_at"] = calibrated_at if calibrated_at else None

        control = data.setdefault("control", {})

        ir = control.setdefault("ir", {})
        ir["pin"] = int(ir.get("pin", IR_PIN))
        ir["active_low"] = bool(ir.get("active_low", IR_ACTIVE_LOW))
        ir["sample_hz"] = max(1.0, float(ir.get("sample_hz", IR_SAMPLE_HZ)))
        ir["stable_detect_sec"] = max(0.0, float(ir.get("stable_detect_sec", IR_STABLE_DETECT_SEC)))
        ir["lost_grace_sec"] = max(0.0, float(ir.get("lost_grace_sec", IR_LOST_GRACE_SEC)))
        ir["check_timeout_sec"] = max(0.5, float(ir.get("check_timeout_sec", IR_CHECK_TIMEOUT_SEC)))

        imu = control.setdefault("imu", {})
        imu["bus"] = int(imu.get("bus", IMU_BUS))
        imu["address"] = int(imu.get("address", IMU_ADDRESS))
        imu["sample_hz"] = max(1.0, float(imu.get("sample_hz", IMU_SAMPLE_HZ)))
        imu["calibration_sec"] = max(0.1, float(imu.get("calibration_sec", IMU_CALIBRATION_SEC)))
        imu["lpf_alpha"] = _clamp(imu.get("lpf_alpha", IMU_LPF_ALPHA), 0.0, 1.0)
        imu["deadband_deg"] = max(0.0, float(imu.get("deadband_deg", IMU_DEADBAND_DEG)))

        pid = control.setdefault("pid", {})
        for axis, defaults in (
            ("pitch", {"kp": PITCH_KP, "ki": PITCH_KI, "kd": PITCH_KD}),
            ("roll", {"kp": ROLL_KP, "ki": ROLL_KI, "kd": ROLL_KD}),
        ):
            axis_cfg = pid.setdefault(axis, {})
            axis_cfg["kp"] = float(axis_cfg.get("kp", defaults["kp"]))
            axis_cfg["ki"] = float(axis_cfg.get("ki", defaults["ki"]))
            axis_cfg["kd"] = float(axis_cfg.get("kd", defaults["kd"]))

        pid["output_limit_deg_s"] = max(0.1, float(pid.get("output_limit_deg_s", PID_OUTPUT_LIMIT_DEG_S)))
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
        motor4["enabled"] = bool(motor4.get("enabled", False))
        # 현재 구현은 무조건 pass. UI에서 true를 저장해도 실제 제어는 아직 활성화하지 않는다.
        motor4["pass"] = True

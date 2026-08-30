import copy
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from services.hardware_constants import (
    IMU_DEADBAND_G,
    IMU_LPF_ALPHA,
    MOTOR34_AUTO_ACC,
    MOTOR34_AUTO_SPEED,
    MOTOR34_COMMAND_HZ,
    MOTOR3_IMU_Y_DIRECTION_SIGN,
    MOTOR3_IMU_Y_KD,
    MOTOR3_IMU_Y_KI,
    MOTOR3_IMU_Y_KP,
    MOTOR4_IMU_X_DIRECTION_SIGN,
    MOTOR4_IMU_X_KD,
    MOTOR4_IMU_X_KI,
    MOTOR4_IMU_X_KP,
    PID_DERIVATIVE_LPF_ALPHA,
    PID_INTEGRAL_LIMIT_G_SEC,
    PID_OUTPUT_LIMIT_DEG_S,
)


WORKSPACE_DIR = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = WORKSPACE_DIR / "data" / "settings" / "hardware_control.json"


# ============================================================
# Version 4 = Direct IMU X/Y -> Motor3/4
# ============================================================
DEFAULT_HARDWARE_CONFIG = {
    "version": 4,
    "calibration": {
        "imu": {
            # 기록용. 새 Process 시작 시 자동 활성화하지 않는다.
            "x_reference_g": 0.0,
            "y_reference_g": 0.0,
            "x_reference_raw": 0.0,
            "y_reference_raw": 0.0,
            "sample_count": 0,
            "calibrated_at": None,
        }
    },
    "control": {
        "imu": {
            "lpf_alpha": IMU_LPF_ALPHA,
            "deadband_g": IMU_DEADBAND_G,
        },
        "pid": {
            "motor3_imu_y": {
                "kp": MOTOR3_IMU_Y_KP,
                "ki": MOTOR3_IMU_Y_KI,
                "kd": MOTOR3_IMU_Y_KD,
            },
            "motor4_imu_x": {
                "kp": MOTOR4_IMU_X_KP,
                "ki": MOTOR4_IMU_X_KI,
                "kd": MOTOR4_IMU_X_KD,
            },
            "output_limit_deg_s": PID_OUTPUT_LIMIT_DEG_S,
            "integral_limit_g_s": PID_INTEGRAL_LIMIT_G_SEC,
            "derivative_lpf_alpha": PID_DERIVATIVE_LPF_ALPHA,
        },
        "motor": {
            "command_hz": MOTOR34_COMMAND_HZ,
            "auto_speed": MOTOR34_AUTO_SPEED,
            "auto_acc": MOTOR34_AUTO_ACC,
            "motor3": {
                "enabled": True,
                "direction_sign": MOTOR3_IMU_Y_DIRECTION_SIGN,
            },
            "motor4": {
                "enabled": True,
                "direction_sign": MOTOR4_IMU_X_DIRECTION_SIGN,
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


def _normalize_loaded(data):
    """예전 Pitch/Roll config를 Direct X/Y v4 구조로 안전하게 마이그레이션.

    중요
    ----
    예전 Pitch/Roll PID gain은 입력 단위가 rad/deg 계열이고,
    새 Direct IMU PID는 입력 단위가 g이므로 숫자를 그대로 옮기지 않는다.
    v4 이전 파일은 검증된 V1.9 기본 gain/sign/rate로 시작한다.
    """
    if not isinstance(data, dict):
        return {}

    data = copy.deepcopy(data)
    version = int(data.get("version", 0) or 0)

    if version >= 4:
        return data

    migrated = copy.deepcopy(DEFAULT_HARDWARE_CONFIG)

    # 기존 LPF alpha는 같은 의미라 유지할 수 있다.
    old_control = data.get("control", {})
    old_imu = old_control.get("imu", {}) if isinstance(old_control, dict) else {}
    if isinstance(old_imu, dict) and "lpf_alpha" in old_imu:
        migrated["control"]["imu"]["lpf_alpha"] = old_imu["lpf_alpha"]

    # Motor enabled 상태만 보존한다.
    # direction sign은 제어 의미가 바뀌었으므로 Direct IMU 검증값(-1/-1) 사용.
    old_motor = old_control.get("motor", {}) if isinstance(old_control, dict) else {}
    if isinstance(old_motor, dict):
        for motor_key in ("motor3", "motor4"):
            old_axis = old_motor.get(motor_key)
            if isinstance(old_axis, dict) and "enabled" in old_axis:
                migrated["control"]["motor"][motor_key]["enabled"] = bool(
                    old_axis["enabled"]
                )

    # Calibration은 입력 의미가 바뀌었으므로 이전 Pitch/Roll offset을
    # Direct X/Y reference로 재사용하지 않는다.
    return migrated


class HardwareConfigService:
    """Direct IMU PID / Filter / Motor3/4 tuning + Calibration 기록 JSON 관리자."""

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

        loaded = _normalize_loaded(loaded)
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
        _deep_merge(data, _normalize_loaded(patch) if "version" in patch else patch)
        return self.save(data)

    def update_control(self, patch):
        if not isinstance(patch, dict):
            raise TypeError("control patch는 dict여야 합니다.")
        return self.update({"control": patch})

    def update_imu_calibration(
        self,
        x_reference_g,
        y_reference_g,
        sample_count=0,
        x_reference_raw=0.0,
        y_reference_raw=0.0,
    ):
        data = self.load()
        calibration = data["calibration"]["imu"]
        calibration["x_reference_g"] = float(x_reference_g)
        calibration["y_reference_g"] = float(y_reference_g)
        calibration["x_reference_raw"] = float(x_reference_raw)
        calibration["y_reference_raw"] = float(y_reference_raw)
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
                previous.get("calibration", {}).get(
                    "imu",
                    data["calibration"]["imu"],
                )
            )
        return self.save(data)

    def _validate(self, data):
        if not isinstance(data, dict):
            raise ValueError("Hardware config root는 object여야 합니다.")

        data["version"] = 4

        calibration = data.setdefault("calibration", {}).setdefault("imu", {})
        calibration["x_reference_g"] = float(calibration.get("x_reference_g", 0.0))
        calibration["y_reference_g"] = float(calibration.get("y_reference_g", 0.0))
        calibration["x_reference_raw"] = float(calibration.get("x_reference_raw", 0.0))
        calibration["y_reference_raw"] = float(calibration.get("y_reference_raw", 0.0))
        calibration["sample_count"] = max(0, int(calibration.get("sample_count", 0)))
        calibrated_at = calibration.get("calibrated_at")
        calibration["calibrated_at"] = calibrated_at if calibrated_at else None

        # 과거 Pitch/Roll calibration key가 섞여 있으면 제거.
        for old_key in ("pitch_offset_deg", "roll_offset_deg"):
            calibration.pop(old_key, None)

        control = data.setdefault("control", {})
        control.pop("ir", None)

        imu = control.setdefault("imu", {})
        imu["lpf_alpha"] = _clamp(
            imu.get("lpf_alpha", IMU_LPF_ALPHA),
            0.01,
            1.0,
        )
        imu["deadband_g"] = max(
            0.0,
            float(imu.get("deadband_g", IMU_DEADBAND_G)),
        )
        # 과거 단위가 다른 키 제거.
        imu.pop("deadband_deg", None)
        for key in ("bus", "address", "sample_hz", "calibration_sec"):
            imu.pop(key, None)

        pid = control.setdefault("pid", {})
        for axis, defaults in (
            (
                "motor3_imu_y",
                {
                    "kp": MOTOR3_IMU_Y_KP,
                    "ki": MOTOR3_IMU_Y_KI,
                    "kd": MOTOR3_IMU_Y_KD,
                },
            ),
            (
                "motor4_imu_x",
                {
                    "kp": MOTOR4_IMU_X_KP,
                    "ki": MOTOR4_IMU_X_KI,
                    "kd": MOTOR4_IMU_X_KD,
                },
            ),
        ):
            axis_cfg = pid.setdefault(axis, {})
            axis_cfg["kp"] = float(axis_cfg.get("kp", defaults["kp"]))
            axis_cfg["ki"] = float(axis_cfg.get("ki", defaults["ki"]))
            axis_cfg["kd"] = float(axis_cfg.get("kd", defaults["kd"]))

        pid["output_limit_deg_s"] = max(
            0.1,
            float(pid.get("output_limit_deg_s", PID_OUTPUT_LIMIT_DEG_S)),
        )
        pid["integral_limit_g_s"] = max(
            0.0,
            float(pid.get("integral_limit_g_s", PID_INTEGRAL_LIMIT_G_SEC)),
        )
        pid["derivative_lpf_alpha"] = _clamp(
            pid.get("derivative_lpf_alpha", PID_DERIVATIVE_LPF_ALPHA),
            0.0,
            1.0,
        )

        # 과거 Pitch/Roll PID key 제거.
        for old_key in (
            "pitch",
            "roll",
            "integral_limit_rad_sec",
            "output_lpf_alpha",
        ):
            pid.pop(old_key, None)

        motor = control.setdefault("motor", {})
        motor["command_hz"] = max(
            1.0,
            float(motor.get("command_hz", MOTOR34_COMMAND_HZ)),
        )
        motor["auto_speed"] = max(
            1,
            int(motor.get("auto_speed", MOTOR34_AUTO_SPEED)),
        )
        motor["auto_acc"] = max(
            1,
            int(motor.get("auto_acc", MOTOR34_AUTO_ACC)),
        )
        motor.pop("pid_speed_deadband_deg_s", None)

        motor3 = motor.setdefault("motor3", {})
        motor3["enabled"] = bool(motor3.get("enabled", True))
        sign = float(
            motor3.get("direction_sign", MOTOR3_IMU_Y_DIRECTION_SIGN)
        )
        motor3["direction_sign"] = 1.0 if sign >= 0.0 else -1.0

        motor4 = motor.setdefault("motor4", {})
        motor4["enabled"] = bool(motor4.get("enabled", True))
        sign = float(
            motor4.get("direction_sign", MOTOR4_IMU_X_DIRECTION_SIGN)
        )
        motor4["direction_sign"] = 1.0 if sign >= 0.0 else -1.0
        motor4.pop("pass", None)

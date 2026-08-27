import math
import time

from services.hardware_constants import (
    IMU_ADDRESS,
    IMU_BUS,
    IMU_CALIBRATION_SEC,
    IMU_DEADBAND_DEG,
    IMU_LPF_ALPHA,
    IMU_SAMPLE_HZ,
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


# 이전 코드/테스트 호환 alias
ADXL345_BUS = IMU_BUS
ADXL345_ADDR = IMU_ADDRESS

REG_BW_RATE = 0x2C
REG_POWER_CTL = 0x2D
REG_DATA_FORMAT = 0x31
REG_DATAX0 = 0x32

BW_RATE_100HZ = 0x0A
DATA_FORMAT_FULL_RES_2G = 0x08
MEASURE_MODE = 0x08


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def normalize_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def low_pass_filter(current, previous, alpha):
    return alpha * current + (1.0 - alpha) * previous


def low_pass_angle(current, previous, alpha):
    delta = normalize_angle(current - previous)
    return normalize_angle(previous + alpha * delta)


def to_signed(low, high):
    value = (high << 8) | low
    if value & 0x8000:
        value -= 65536
    return value


class PIDController:
    def __init__(
        self,
        kp,
        ki,
        kd,
        output_limit,
        integral_limit,
        deadband_deg=0.5,
        derivative_alpha=0.15,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.deadband = math.radians(float(deadband_deg))
        self.derivative_alpha = float(derivative_alpha)
        self.reset()

    def configure(
        self,
        kp,
        ki,
        kd,
        output_limit,
        integral_limit,
        deadband_deg,
        derivative_alpha,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.deadband = math.radians(float(deadband_deg))
        self.derivative_alpha = float(derivative_alpha)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        if abs(error) <= self.deadband:
            self.integral = 0.0
            self.previous_error = error
            self.filtered_derivative = 0.0
            return 0.0

        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = normalize_angle(error - self.previous_error) / dt

        self.filtered_derivative = (
            self.derivative_alpha * derivative
            + (1.0 - self.derivative_alpha) * self.filtered_derivative
        )

        integral_candidate = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )

        unsaturated_output = (
            self.kp * error
            + self.ki * integral_candidate
            + self.kd * self.filtered_derivative
        )

        allow_integral = False
        if abs(unsaturated_output) <= self.output_limit:
            allow_integral = True
        elif unsaturated_output > self.output_limit and error < 0.0:
            allow_integral = True
        elif unsaturated_output < -self.output_limit and error > 0.0:
            allow_integral = True

        if allow_integral:
            self.integral = integral_candidate

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )

        self.previous_error = error
        return clamp(output, -self.output_limit, self.output_limit)


class ADXL345IMUService:
    """ADXL345 -> Pitch/Roll Offset -> LPF -> PID -> Output LPF.

    Offset은 IR 사전 확인을 통과한 Calibration에서만 새로 잡는다.
    저장된 JSON Offset은 기록용이며 새 Process 시작 시 자동 활성화하지 않는다.
    """

    def __init__(
        self,
        bus_number=ADXL345_BUS,
        address=ADXL345_ADDR,
        sample_hz=IMU_SAMPLE_HZ,
        calibration_sec=IMU_CALIBRATION_SEC,
        imu_alpha=IMU_LPF_ALPHA,
        deadband_deg=IMU_DEADBAND_DEG,
        pitch_pid=None,
        roll_pid=None,
        output_limit_deg_s=PID_OUTPUT_LIMIT_DEG_S,
        integral_limit_rad_sec=PID_INTEGRAL_LIMIT_RAD_SEC,
        derivative_alpha=PID_DERIVATIVE_LPF_ALPHA,
        output_alpha=PID_OUTPUT_LPF_ALPHA,
        bus_factory=None,
    ):
        self.bus_number = int(bus_number)
        self.address = int(address)
        self.sample_hz = float(sample_hz)
        self.sample_interval = 1.0 / max(self.sample_hz, 1.0)
        self.calibration_sec = max(float(calibration_sec), 0.1)
        self.imu_alpha = float(imu_alpha)
        self.deadband_deg = max(0.0, float(deadband_deg))
        self.output_limit_deg_s = max(0.1, float(output_limit_deg_s))
        self.integral_limit_rad_sec = max(0.0, float(integral_limit_rad_sec))
        self.derivative_alpha = float(derivative_alpha)
        self.output_alpha = float(output_alpha)
        self.bus_factory = bus_factory

        pitch_pid = pitch_pid or {
            "kp": PITCH_KP,
            "ki": PITCH_KI,
            "kd": PITCH_KD,
        }
        roll_pid = roll_pid or {
            "kp": ROLL_KP,
            "ki": ROLL_KI,
            "kd": ROLL_KD,
        }

        output_limit = math.radians(self.output_limit_deg_s)
        self.pitch_pid = PIDController(
            pitch_pid.get("kp", PITCH_KP),
            pitch_pid.get("ki", PITCH_KI),
            pitch_pid.get("kd", PITCH_KD),
            output_limit,
            self.integral_limit_rad_sec,
            self.deadband_deg,
            self.derivative_alpha,
        )
        self.roll_pid = PIDController(
            roll_pid.get("kp", ROLL_KP),
            roll_pid.get("ki", ROLL_KI),
            roll_pid.get("kd", ROLL_KD),
            output_limit,
            self.integral_limit_rad_sec,
            self.deadband_deg,
            self.derivative_alpha,
        )

        self.bus = None
        self.available = False
        self.last_error = None
        self.last_update_time = None

        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self.filtered_pitch_output = 0.0
        self.filtered_roll_output = 0.0
        self.filter_initialized = False

        self.calibrating = False
        self.calibrated_session = False
        self.calibration_started_at = None
        self.calibration_pitch_samples = []
        self.calibration_roll_samples = []
        self._pending_calibration_result = None

        self.latest_state = self._empty_state()

    def _empty_state(self):
        return {
            "available": False,
            "calibrating": False,
            "calibrated": False,
            "calibration_remaining_sec": 0.0,
            "calibration_sample_count": 0,
            "raw_x": 0,
            "raw_y": 0,
            "raw_z": 0,
            "absolute_pitch_deg": 0.0,
            "absolute_roll_deg": 0.0,
            "pitch_offset_deg": 0.0,
            "roll_offset_deg": 0.0,
            # 이전 코드와의 호환용 alias. 신규 코드는 offset 이름을 사용한다.
            "reference_pitch_deg": 0.0,
            "reference_roll_deg": 0.0,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
            "correction_pitch_deg": 0.0,
            "correction_roll_deg": 0.0,
            "correction_pitch_speed_deg_s": 0.0,
            "correction_roll_speed_deg_s": 0.0,
            "last_error": None,
            "timestamp": time.time(),
        }

    @property
    def calibrated(self):
        return bool(self.available and self.calibrated_session and not self.calibrating)

    def apply_control_config(self, imu_config, pid_config):
        imu_config = imu_config if isinstance(imu_config, dict) else {}
        pid_config = pid_config if isinstance(pid_config, dict) else {}

        old_bus = self.bus_number
        old_address = self.address
        was_available = self.available

        self.bus_number = int(imu_config.get("bus", self.bus_number))
        self.address = int(imu_config.get("address", self.address))
        self.sample_hz = max(1.0, float(imu_config.get("sample_hz", self.sample_hz)))
        self.sample_interval = 1.0 / self.sample_hz
        self.calibration_sec = max(
            0.1, float(imu_config.get("calibration_sec", self.calibration_sec))
        )
        self.imu_alpha = clamp(imu_config.get("lpf_alpha", self.imu_alpha), 0.0, 1.0)
        self.deadband_deg = max(
            0.0, float(imu_config.get("deadband_deg", self.deadband_deg))
        )

        self.output_limit_deg_s = max(
            0.1, float(pid_config.get("output_limit_deg_s", self.output_limit_deg_s))
        )
        self.integral_limit_rad_sec = max(
            0.0,
            float(pid_config.get("integral_limit_rad_sec", self.integral_limit_rad_sec)),
        )
        self.derivative_alpha = clamp(
            pid_config.get("derivative_lpf_alpha", self.derivative_alpha), 0.0, 1.0
        )
        self.output_alpha = clamp(
            pid_config.get("output_lpf_alpha", self.output_alpha), 0.0, 1.0
        )

        pitch_cfg = pid_config.get("pitch", {})
        roll_cfg = pid_config.get("roll", {})
        output_limit = math.radians(self.output_limit_deg_s)

        self.pitch_pid.configure(
            pitch_cfg.get("kp", self.pitch_pid.kp),
            pitch_cfg.get("ki", self.pitch_pid.ki),
            pitch_cfg.get("kd", self.pitch_pid.kd),
            output_limit,
            self.integral_limit_rad_sec,
            self.deadband_deg,
            self.derivative_alpha,
        )
        self.roll_pid.configure(
            roll_cfg.get("kp", self.roll_pid.kp),
            roll_cfg.get("ki", self.roll_pid.ki),
            roll_cfg.get("kd", self.roll_pid.kd),
            output_limit,
            self.integral_limit_rad_sec,
            self.deadband_deg,
            self.derivative_alpha,
        )

        self.filtered_pitch_output = 0.0
        self.filtered_roll_output = 0.0
        self.last_update_time = None

        hardware_changed = old_bus != self.bus_number or old_address != self.address
        if hardware_changed and was_available:
            self.close()
            self.invalidate_calibration()
            return self.open()
        return True

    def open(self):
        try:
            if self.bus_factory is None:
                from smbus2 import SMBus
                factory = SMBus
            else:
                factory = self.bus_factory

            self.bus = factory(self.bus_number)
            self.bus.write_byte_data(self.address, REG_BW_RATE, BW_RATE_100HZ)
            self.bus.write_byte_data(self.address, REG_DATA_FORMAT, DATA_FORMAT_FULL_RES_2G)
            self.bus.write_byte_data(self.address, REG_POWER_CTL, MEASURE_MODE)

            self.available = True
            self.last_error = None
            self.invalidate_calibration()
            print(
                f"[IMU] ADXL345 준비 완료 bus={self.bus_number} "
                f"addr=0x{self.address:02X} sample={self.sample_hz:.0f}Hz"
            )
            return True

        except Exception as error:
            self.available = False
            self.last_error = str(error)
            self.latest_state = self._empty_state()
            self.latest_state["last_error"] = self.last_error
            print(f"[IMU] ADXL345 초기화 실패: {error}")
            self.close()
            return False

    def close(self):
        bus = self.bus
        self.bus = None
        self.available = False
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass

    def invalidate_calibration(self):
        self.calibrating = False
        self.calibrated_session = False
        self.calibration_started_at = None
        self.calibration_pitch_samples.clear()
        self.calibration_roll_samples.clear()
        self._pending_calibration_result = None
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self.filtered_pitch_output = 0.0
        self.filtered_roll_output = 0.0
        self.filter_initialized = False
        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.last_update_time = None

    def start_calibration(self):
        if not self.available:
            return False

        self.calibrating = True
        self.calibrated_session = False
        self.calibration_started_at = time.monotonic()
        self.calibration_pitch_samples.clear()
        self.calibration_roll_samples.clear()
        self._pending_calibration_result = None

        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.filtered_pitch_output = 0.0
        self.filtered_roll_output = 0.0
        self.filter_initialized = False
        self.last_update_time = None

        print(f"[IMU] Offset Calibration 시작 ({self.calibration_sec:.1f}초)")
        return True

    def cancel_calibration(self):
        self.invalidate_calibration()
        print("[IMU] Offset Calibration 취소")

    def consume_calibration_result(self):
        result = self._pending_calibration_result
        self._pending_calibration_result = None
        return dict(result) if isinstance(result, dict) else None

    def _read_raw(self):
        data = self.bus.read_i2c_block_data(self.address, REG_DATAX0, 6)
        x = to_signed(data[0], data[1])
        y = to_signed(data[2], data[3])
        z = to_signed(data[4], data[5])
        return x, y, z

    @staticmethod
    def _calculate_pitch_roll(x, y, z):
        pitch = math.atan2(-x, math.sqrt(y * y + z * z))
        roll = math.atan2(y, z)
        return pitch, roll

    @staticmethod
    def _circular_mean(values):
        if not values:
            return 0.0
        sin_sum = sum(math.sin(value) for value in values)
        cos_sum = sum(math.cos(value) for value in values)
        return math.atan2(sin_sum, cos_sum)

    def _finish_calibration(self):
        if not self.calibration_pitch_samples:
            raise RuntimeError("IMU Offset Calibration sample이 없습니다.")

        self.pitch_offset = self._circular_mean(self.calibration_pitch_samples)
        self.roll_offset = self._circular_mean(self.calibration_roll_samples)
        self.calibrating = False
        self.calibrated_session = True

        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self.filtered_pitch_output = 0.0
        self.filtered_roll_output = 0.0
        self.filter_initialized = True
        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.last_update_time = None

        self._pending_calibration_result = {
            "pitch_offset_deg": math.degrees(self.pitch_offset),
            "roll_offset_deg": math.degrees(self.roll_offset),
            "sample_count": len(self.calibration_pitch_samples),
        }

        print(
            "[IMU] Offset Calibration 완료 "
            f"samples={len(self.calibration_pitch_samples)} "
            f"pitch_offset={math.degrees(self.pitch_offset):+.2f}deg "
            f"roll_offset={math.degrees(self.roll_offset):+.2f}deg"
        )

    def _state_common(self, x, y, z, absolute_pitch, absolute_roll):
        pitch_offset_deg = math.degrees(self.pitch_offset)
        roll_offset_deg = math.degrees(self.roll_offset)
        return {
            "available": True,
            "raw_x": x,
            "raw_y": y,
            "raw_z": z,
            "absolute_pitch_deg": math.degrees(absolute_pitch),
            "absolute_roll_deg": math.degrees(absolute_roll),
            "pitch_offset_deg": pitch_offset_deg,
            "roll_offset_deg": roll_offset_deg,
            "reference_pitch_deg": pitch_offset_deg,
            "reference_roll_deg": roll_offset_deg,
            "last_error": None,
            "timestamp": time.time(),
        }

    def update(self):
        now_monotonic = time.monotonic()

        if not self.available or self.bus is None:
            state = self._empty_state()
            state["last_error"] = self.last_error or "IMU unavailable"
            self.latest_state = state
            return state

        try:
            x, y, z = self._read_raw()
            absolute_pitch, absolute_roll = self._calculate_pitch_roll(x, y, z)
            common = self._state_common(x, y, z, absolute_pitch, absolute_roll)

            if self.calibrating:
                self.calibration_pitch_samples.append(absolute_pitch)
                self.calibration_roll_samples.append(absolute_roll)

                elapsed = now_monotonic - self.calibration_started_at
                remaining = max(0.0, self.calibration_sec - elapsed)

                if elapsed >= self.calibration_sec:
                    self._finish_calibration()
                    common = self._state_common(x, y, z, absolute_pitch, absolute_roll)

                state = {
                    **common,
                    "calibrating": self.calibrating,
                    "calibrated": self.calibrated,
                    "calibration_remaining_sec": remaining if self.calibrating else 0.0,
                    "calibration_sample_count": len(self.calibration_pitch_samples),
                    "pitch_deg": 0.0,
                    "roll_deg": 0.0,
                    "correction_pitch_deg": 0.0,
                    "correction_roll_deg": 0.0,
                    "correction_pitch_speed_deg_s": 0.0,
                    "correction_roll_speed_deg_s": 0.0,
                }
                self.latest_state = state
                return state

            if not self.calibrated:
                state = {
                    **common,
                    "calibrating": False,
                    "calibrated": False,
                    "calibration_remaining_sec": 0.0,
                    "calibration_sample_count": 0,
                    "pitch_deg": 0.0,
                    "roll_deg": 0.0,
                    "correction_pitch_deg": 0.0,
                    "correction_roll_deg": 0.0,
                    "correction_pitch_speed_deg_s": 0.0,
                    "correction_roll_speed_deg_s": 0.0,
                }
                self.latest_state = state
                return state

            relative_pitch = normalize_angle(absolute_pitch - self.pitch_offset)
            relative_roll = normalize_angle(absolute_roll - self.roll_offset)

            if not self.filter_initialized:
                self.filtered_pitch = relative_pitch
                self.filtered_roll = relative_roll
                self.filter_initialized = True
            else:
                self.filtered_pitch = low_pass_angle(
                    relative_pitch, self.filtered_pitch, self.imu_alpha
                )
                self.filtered_roll = low_pass_angle(
                    relative_roll, self.filtered_roll, self.imu_alpha
                )

            deadband_rad = math.radians(self.deadband_deg)
            filtered_pitch_for_output = (
                0.0 if abs(self.filtered_pitch) <= deadband_rad else self.filtered_pitch
            )
            filtered_roll_for_output = (
                0.0 if abs(self.filtered_roll) <= deadband_rad else self.filtered_roll
            )

            if self.last_update_time is None:
                dt = self.sample_interval
            else:
                dt = max(0.0, now_monotonic - self.last_update_time)
            self.last_update_time = now_monotonic

            pitch_error = -filtered_pitch_for_output
            roll_error = -filtered_roll_for_output

            pitch_pid_output = self.pitch_pid.update(pitch_error, dt)
            roll_pid_output = self.roll_pid.update(roll_error, dt)

            if filtered_pitch_for_output == 0.0:
                self.filtered_pitch_output = 0.0
            else:
                self.filtered_pitch_output = low_pass_filter(
                    pitch_pid_output, self.filtered_pitch_output, self.output_alpha
                )

            if filtered_roll_for_output == 0.0:
                self.filtered_roll_output = 0.0
            else:
                self.filtered_roll_output = low_pass_filter(
                    roll_pid_output, self.filtered_roll_output, self.output_alpha
                )

            state = {
                **common,
                "calibrating": False,
                "calibrated": True,
                "calibration_remaining_sec": 0.0,
                "calibration_sample_count": len(self.calibration_pitch_samples),
                "pitch_deg": math.degrees(filtered_pitch_for_output),
                "roll_deg": math.degrees(filtered_roll_for_output),
                "correction_pitch_deg": math.degrees(-filtered_pitch_for_output),
                "correction_roll_deg": math.degrees(-filtered_roll_for_output),
                "correction_pitch_speed_deg_s": math.degrees(self.filtered_pitch_output),
                "correction_roll_speed_deg_s": math.degrees(self.filtered_roll_output),
            }

            self.last_error = None
            self.latest_state = state
            return state

        except Exception as error:
            self.last_error = str(error)
            state = dict(self.latest_state)
            state["available"] = False
            state["last_error"] = self.last_error
            state["timestamp"] = time.time()
            self.latest_state = state
            return state

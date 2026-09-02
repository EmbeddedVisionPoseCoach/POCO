import math
import time

from services.hardware_constants import (
    IMU_ADDRESS,
    IMU_BUS,
    IMU_CALIBRATION_MIN_SAMPLES,
    IMU_CALIBRATION_SEC,
    IMU_DEADBAND_G,
    IMU_G_PER_LSB,
    IMU_LPF_ALPHA,
    IMU_SAMPLE_HZ,
    MOTOR3_IMU_Y_KD,
    MOTOR3_IMU_Y_KI,
    MOTOR3_IMU_Y_KP,
    MOTOR4_IMU_X_KD,
    MOTOR4_IMU_X_KI,
    MOTOR4_IMU_X_KP,
    PID_DERIVATIVE_LPF_ALPHA,
    PID_INTEGRAL_LIMIT_G_SEC,
    PID_OUTPUT_LIMIT_DEG_S,
)


# 이전 테스트/코드 호환 alias
ADXL345_BUS = IMU_BUS
ADXL345_ADDR = IMU_ADDRESS

REG_BW_RATE = 0x2C
REG_POWER_CTL = 0x2D
REG_DATA_FORMAT = 0x31
REG_DATAX0 = 0x32

BW_RATE_100HZ = 0x0A
DATA_FORMAT_FULL_RES_2G = 0x08
MEASURE_MODE = 0x08


def clamp(value, minimum, maximum):
    return max(minimum, min(float(value), maximum))


def low_pass_filter(current, previous, alpha):
    return alpha * current + (1.0 - alpha) * previous


def to_signed(low, high):
    value = (high << 8) | low
    if value & 0x8000:
        value -= 65536
    return value


class DirectIMUPIDController:
    """IMU X/Y 오차[g] -> Motor 목표각 변화속도[deg/s].

    imu_xy_direct_pid_tuner V1.9에서 검증한 PID 식을 그대로 사용한다.
    중요한 점은 PID 입력이 Pitch/Roll 각도가 아니라 Calibration 기준 대비
    filtered X/Y 중력가속도 오차라는 것이다.
    """

    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        output_limit_deg_s=PID_OUTPUT_LIMIT_DEG_S,
        integral_limit_g_s=PID_INTEGRAL_LIMIT_G_SEC,
        deadband_g=IMU_DEADBAND_G,
        derivative_alpha=PID_DERIVATIVE_LPF_ALPHA,
    ):
        self.configure(
            kp=kp,
            ki=ki,
            kd=kd,
            output_limit_deg_s=output_limit_deg_s,
            integral_limit_g_s=integral_limit_g_s,
            deadband_g=deadband_g,
            derivative_alpha=derivative_alpha,
        )
        self.reset()

    def configure(
        self,
        kp,
        ki,
        kd,
        output_limit_deg_s,
        integral_limit_g_s,
        deadband_g,
        derivative_alpha,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit_deg_s))
        self.integral_limit = abs(float(integral_limit_g_s))
        self.deadband = abs(float(deadband_g))
        self.derivative_alpha = clamp(derivative_alpha, 0.0, 1.0)

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0
        self.last = {
            "error": 0.0,
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "derivative": 0.0,
            "output": 0.0,
        }

    def update(self, imu_error_g, dt):
        dt = max(float(dt), 1e-4)

        # Tuner V1.9와 동일한 부호.
        # 이후 Motor Controller의 direction_sign이 실제 Servo 방향을 결정한다.
        error = -float(imu_error_g)

        if abs(error) <= self.deadband:
            error = 0.0

        if self.previous_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (error - self.previous_error) / dt

        a = self.derivative_alpha
        self.filtered_derivative = (
            a * raw_derivative
            + (1.0 - a) * self.filtered_derivative
        )

        integral_candidate = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )

        p = self.kp * error
        i_candidate = self.ki * integral_candidate
        d = self.kd * self.filtered_derivative
        unsaturated = p + i_candidate + d

        allow_integral = False
        if abs(unsaturated) <= self.output_limit:
            allow_integral = True
        elif unsaturated > self.output_limit and error < 0.0:
            allow_integral = True
        elif unsaturated < -self.output_limit and error > 0.0:
            allow_integral = True

        if allow_integral:
            self.integral = integral_candidate

        i = self.ki * self.integral
        output = clamp(
            p + i + d,
            -self.output_limit,
            self.output_limit,
        )

        self.previous_error = error
        self.last = {
            "error": error,
            "p": p,
            "i": i,
            "d": d,
            "derivative": self.filtered_derivative,
            "output": output,
        }
        return dict(self.last)


# 이전 import 이름 호환
PIDController = DirectIMUPIDController


class ADXL345IMUService:
    """ADXL345 Direct X/Y Gimbal Service.

    실제 제어 경로
    ------------
    ADXL345 Raw X/Y/Z
      -> g 변환
      -> X/Y/Z Low Pass Filter
      -> Calibration에서 filtered X/Y 기준값 저장
      -> Y Error -> PID -> Motor3 목표속도[deg/s]
      -> X Error -> PID -> Motor4 목표속도[deg/s]

    Pitch/Roll은 상태 확인용으로만 계산하며 Motor3/4 PID 입력에는 사용하지 않는다.
    """

    def __init__(
        self,
        bus_number=ADXL345_BUS,
        address=ADXL345_ADDR,
        sample_hz=IMU_SAMPLE_HZ,
        calibration_sec=IMU_CALIBRATION_SEC,
        calibration_min_samples=IMU_CALIBRATION_MIN_SAMPLES,
        imu_alpha=IMU_LPF_ALPHA,
        deadband_g=IMU_DEADBAND_G,
        motor3_pid=None,
        motor4_pid=None,
        output_limit_deg_s=PID_OUTPUT_LIMIT_DEG_S,
        integral_limit_g_s=PID_INTEGRAL_LIMIT_G_SEC,
        derivative_alpha=PID_DERIVATIVE_LPF_ALPHA,
        bus_factory=None,
    ):
        self.bus_number = int(bus_number)
        self.address = int(address)
        self.sample_hz = max(1.0, float(sample_hz))
        self.sample_interval = 1.0 / self.sample_hz
        self.calibration_sec = max(float(calibration_sec), 0.1)
        self.calibration_min_samples = max(1, int(calibration_min_samples))
        self.imu_alpha = clamp(imu_alpha, 0.01, 1.0)
        self.deadband_g = max(0.0, float(deadband_g))
        self.output_limit_deg_s = max(0.1, float(output_limit_deg_s))
        self.integral_limit_g_s = max(0.0, float(integral_limit_g_s))
        self.derivative_alpha = clamp(derivative_alpha, 0.0, 1.0)
        self.bus_factory = bus_factory

        motor3_pid = motor3_pid or {
            "kp": MOTOR3_IMU_Y_KP,
            "ki": MOTOR3_IMU_Y_KI,
            "kd": MOTOR3_IMU_Y_KD,
        }
        motor4_pid = motor4_pid or {
            "kp": MOTOR4_IMU_X_KP,
            "ki": MOTOR4_IMU_X_KI,
            "kd": MOTOR4_IMU_X_KD,
        }

        self.motor3_imu_y_pid = DirectIMUPIDController(
            motor3_pid.get("kp", MOTOR3_IMU_Y_KP),
            motor3_pid.get("ki", MOTOR3_IMU_Y_KI),
            motor3_pid.get("kd", MOTOR3_IMU_Y_KD),
            self.output_limit_deg_s,
            self.integral_limit_g_s,
            self.deadband_g,
            self.derivative_alpha,
        )
        self.motor4_imu_x_pid = DirectIMUPIDController(
            motor4_pid.get("kp", MOTOR4_IMU_X_KP),
            motor4_pid.get("ki", MOTOR4_IMU_X_KI),
            motor4_pid.get("kd", MOTOR4_IMU_X_KD),
            self.output_limit_deg_s,
            self.integral_limit_g_s,
            self.deadband_g,
            self.derivative_alpha,
        )

        self.bus = None
        self.available = False
        self.last_error = None
        self.last_update_time = None

        # Filtered gravity vector [g]
        self.filtered_x_g = 0.0
        self.filtered_y_g = 0.0
        self.filtered_z_g = 1.0
        self.filter_initialized = False

        # Calibration reference
        self.imu_x_reference_g = 0.0
        self.imu_y_reference_g = 0.0
        self.imu_x_reference_raw = 0.0
        self.imu_y_reference_raw = 0.0
        self.pitch_reference_deg = 0.0
        self.roll_reference_deg = 0.0

        self.calibrating = False
        self.calibrated_session = False
        self.calibration_started_at = None
        self.calibration_x_g_samples = []
        self.calibration_y_g_samples = []
        self.calibration_x_raw_samples = []
        self.calibration_y_raw_samples = []
        self.calibration_pitch_samples = []
        self.calibration_roll_samples = []
        self._pending_calibration_result = None

        self.latest_state = self._empty_state()

    @property
    def calibrated(self):
        return bool(
            self.available
            and self.calibrated_session
            and not self.calibrating
        )

    def _empty_state(self):
        return {
            "available": False,
            "calibrating": False,
            "calibrated": False,
            "calibration_remaining_sec": 0.0,
            "calibration_sample_count": 0,
            "calibration_min_samples": self.calibration_min_samples,
            "raw_x": 0,
            "raw_y": 0,
            "raw_z": 0,
            "ax_g": 0.0,
            "ay_g": 0.0,
            "az_g": 0.0,
            "filtered_x_g": 0.0,
            "filtered_y_g": 0.0,
            "filtered_z_g": 0.0,
            "norm_g": 0.0,
            "imu_x_reference_g": 0.0,
            "imu_y_reference_g": 0.0,
            "imu_x_reference_raw": 0.0,
            "imu_y_reference_raw": 0.0,
            "imu_x_error_g": 0.0,
            "imu_y_error_g": 0.0,
            "motor3_imu_y_pid": dict(self.motor3_imu_y_pid.last),
            "motor4_imu_x_pid": dict(self.motor4_imu_x_pid.last),
            "motor3_correction_speed_deg_s": 0.0,
            "motor4_correction_speed_deg_s": 0.0,
            "output_limit_deg_s": self.output_limit_deg_s,
            # Pitch/Roll은 진단/기존 UI 호환용
            "absolute_pitch_deg": 0.0,
            "absolute_roll_deg": 0.0,
            "pitch_reference_deg": 0.0,
            "roll_reference_deg": 0.0,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
            "pitch_offset_deg": 0.0,
            "roll_offset_deg": 0.0,
            "reference_pitch_deg": 0.0,
            "reference_roll_deg": 0.0,
            "correction_pitch_deg": 0.0,
            "correction_roll_deg": 0.0,
            "correction_pitch_speed_deg_s": 0.0,
            "correction_roll_speed_deg_s": 0.0,
            "last_error": None,
            "timestamp": time.time(),
        }

    def apply_control_config(self, imu_config, pid_config):
        """hardware_control.json의 Direct IMU 튜닝값을 즉시 반영한다."""
        imu_config = imu_config if isinstance(imu_config, dict) else {}
        pid_config = pid_config if isinstance(pid_config, dict) else {}

        self.imu_alpha = clamp(
            imu_config.get("lpf_alpha", self.imu_alpha),
            0.01,
            1.0,
        )
        self.deadband_g = max(
            0.0,
            float(imu_config.get("deadband_g", self.deadband_g)),
        )

        self.output_limit_deg_s = max(
            0.1,
            float(pid_config.get("output_limit_deg_s", self.output_limit_deg_s)),
        )
        self.integral_limit_g_s = max(
            0.0,
            float(pid_config.get("integral_limit_g_s", self.integral_limit_g_s)),
        )
        self.derivative_alpha = clamp(
            pid_config.get("derivative_lpf_alpha", self.derivative_alpha),
            0.0,
            1.0,
        )

        motor3_cfg = pid_config.get("motor3_imu_y", {})
        motor4_cfg = pid_config.get("motor4_imu_x", {})

        self.motor3_imu_y_pid.configure(
            motor3_cfg.get("kp", self.motor3_imu_y_pid.kp),
            motor3_cfg.get("ki", self.motor3_imu_y_pid.ki),
            motor3_cfg.get("kd", self.motor3_imu_y_pid.kd),
            self.output_limit_deg_s,
            self.integral_limit_g_s,
            self.deadband_g,
            self.derivative_alpha,
        )
        self.motor4_imu_x_pid.configure(
            motor4_cfg.get("kp", self.motor4_imu_x_pid.kp),
            motor4_cfg.get("ki", self.motor4_imu_x_pid.ki),
            motor4_cfg.get("kd", self.motor4_imu_x_pid.kd),
            self.output_limit_deg_s,
            self.integral_limit_g_s,
            self.deadband_g,
            self.derivative_alpha,
        )

        self.motor3_imu_y_pid.reset()
        self.motor4_imu_x_pid.reset()
        self.last_update_time = None
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
            self.bus.write_byte_data(
                self.address,
                REG_DATA_FORMAT,
                DATA_FORMAT_FULL_RES_2G,
            )
            self.bus.write_byte_data(self.address, REG_POWER_CTL, MEASURE_MODE)

            self.available = True
            self.last_error = None
            self.invalidate_calibration()
            print(
                f"[IMU] ADXL345 Direct XY 준비 완료 "
                f"bus={self.bus_number} addr=0x{self.address:02X} "
                f"sample={self.sample_hz:.0f}Hz alpha={self.imu_alpha:.2f}"
            )
            print(
                "[IMU CONTROL] "
                f"M3<-Y Kp={self.motor3_imu_y_pid.kp:.1f} "
                f"Ki={self.motor3_imu_y_pid.ki:.1f} "
                f"Kd={self.motor3_imu_y_pid.kd:.1f} | "
                f"M4<-X Kp={self.motor4_imu_x_pid.kp:.1f} "
                f"Ki={self.motor4_imu_x_pid.ki:.1f} "
                f"Kd={self.motor4_imu_x_pid.kd:.1f} | "
                f"deadband={self.deadband_g:.3f}g "
                f"limit=±{self.output_limit_deg_s:.1f}deg/s "
                f"Dalpha={self.derivative_alpha:.2f}"
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

        self.calibration_x_g_samples.clear()
        self.calibration_y_g_samples.clear()
        self.calibration_x_raw_samples.clear()
        self.calibration_y_raw_samples.clear()
        self.calibration_pitch_samples.clear()
        self.calibration_roll_samples.clear()
        self._pending_calibration_result = None

        self.imu_x_reference_g = 0.0
        self.imu_y_reference_g = 0.0
        self.imu_x_reference_raw = 0.0
        self.imu_y_reference_raw = 0.0
        self.pitch_reference_deg = 0.0
        self.roll_reference_deg = 0.0

        self.filtered_x_g = 0.0
        self.filtered_y_g = 0.0
        self.filtered_z_g = 1.0
        self.filter_initialized = False

        self.motor3_imu_y_pid.reset()
        self.motor4_imu_x_pid.reset()
        self.last_update_time = None

    def restore_calibration(self, record):
        """Restore a previously saved user reference for this process session."""
        if not self.available:
            raise RuntimeError("IMU가 연결되지 않아 프로필을 적용할 수 없습니다.")
        record = record if isinstance(record, dict) else {}
        required = {
            "imu_x_reference_g": "x_reference_g",
            "imu_y_reference_g": "y_reference_g",
            "imu_x_reference_raw": "x_reference_raw",
            "imu_y_reference_raw": "y_reference_raw",
            "pitch_reference_deg": "pitch_reference_deg",
            "roll_reference_deg": "roll_reference_deg",
        }
        values = {}
        for attribute, key in required.items():
            if record.get(key) is None:
                raise ValueError(f"IMU 프로필 값이 없습니다: {key}")
            values[attribute] = float(record[key])
        self.calibrating = False
        self.calibrated_session = True
        self.calibration_started_at = None
        for attribute, value in values.items():
            setattr(self, attribute, value)
        self.motor3_imu_y_pid.reset()
        self.motor4_imu_x_pid.reset()
        self.last_update_time = None
        self._pending_calibration_result = None
        state = dict(self.latest_state) if isinstance(self.latest_state, dict) else self._empty_state()
        state.update({
            "available": True,
            "calibrating": False,
            "calibrated": True,
            "imu_x_reference_g": self.imu_x_reference_g,
            "imu_y_reference_g": self.imu_y_reference_g,
            "imu_x_reference_raw": self.imu_x_reference_raw,
            "imu_y_reference_raw": self.imu_y_reference_raw,
            "pitch_reference_deg": self.pitch_reference_deg,
            "roll_reference_deg": self.roll_reference_deg,
            "timestamp": time.time(),
        })
        self.latest_state = state
        return dict(state)

    def start_calibration(self):
        if not self.available:
            return False

        self.calibrating = True
        self.calibrated_session = False
        self.calibration_started_at = time.monotonic()

        self.calibration_x_g_samples.clear()
        self.calibration_y_g_samples.clear()
        self.calibration_x_raw_samples.clear()
        self.calibration_y_raw_samples.clear()
        self.calibration_pitch_samples.clear()
        self.calibration_roll_samples.clear()
        self._pending_calibration_result = None

        # 현재 자세 기준을 새로 잡으므로 Filter/PID도 새 세션으로 시작한다.
        self.filter_initialized = False
        self.motor3_imu_y_pid.reset()
        self.motor4_imu_x_pid.reset()
        self.last_update_time = None

        state = dict(self.latest_state) if isinstance(self.latest_state, dict) else self._empty_state()
        state.update({
            "available": bool(self.available),
            "calibrating": True,
            "calibrated": False,
            "calibration_remaining_sec": float(self.calibration_sec),
            "calibration_sample_count": 0,
            "calibration_min_samples": self.calibration_min_samples,
            "imu_x_error_g": 0.0,
            "imu_y_error_g": 0.0,
            "motor3_correction_speed_deg_s": 0.0,
            "motor4_correction_speed_deg_s": 0.0,
            "last_error": None,
            "timestamp": time.time(),
        })
        self.latest_state = state

        print(
            f"[IMU] X/Y Reference Calibration 시작 "
            f"({self.calibration_sec:.1f}초, min={self.calibration_min_samples} samples)"
        )
        return True

    def cancel_calibration(self):
        self.invalidate_calibration()
        print("[IMU] X/Y Reference Calibration 취소")

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
    def _calculate_pitch_roll(filtered_x_g, filtered_y_g, filtered_z_g):
        # Tuner와 같은 진단식. PID에는 사용하지 않는다.
        pitch_deg = math.degrees(
            math.atan2(filtered_y_g, filtered_z_g)
        )
        roll_deg = math.degrees(
            math.atan2(
                -filtered_x_g,
                math.sqrt(
                    filtered_y_g * filtered_y_g
                    + filtered_z_g * filtered_z_g
                ),
            )
        )
        return pitch_deg, roll_deg

    def _read_filtered_sample(self):
        raw_x, raw_y, raw_z = self._read_raw()

        ax_g = raw_x * IMU_G_PER_LSB
        ay_g = raw_y * IMU_G_PER_LSB
        az_g = raw_z * IMU_G_PER_LSB

        if not self.filter_initialized:
            self.filtered_x_g = ax_g
            self.filtered_y_g = ay_g
            self.filtered_z_g = az_g
            self.filter_initialized = True
        else:
            a = self.imu_alpha
            self.filtered_x_g = low_pass_filter(ax_g, self.filtered_x_g, a)
            self.filtered_y_g = low_pass_filter(ay_g, self.filtered_y_g, a)
            self.filtered_z_g = low_pass_filter(az_g, self.filtered_z_g, a)

        norm_g = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        pitch_deg, roll_deg = self._calculate_pitch_roll(
            self.filtered_x_g,
            self.filtered_y_g,
            self.filtered_z_g,
        )

        return {
            "raw_x": raw_x,
            "raw_y": raw_y,
            "raw_z": raw_z,
            "ax_g": ax_g,
            "ay_g": ay_g,
            "az_g": az_g,
            "filtered_x_g": self.filtered_x_g,
            "filtered_y_g": self.filtered_y_g,
            "filtered_z_g": self.filtered_z_g,
            "norm_g": norm_g,
            "absolute_pitch_deg": pitch_deg,
            "absolute_roll_deg": roll_deg,
        }

    def _finish_calibration(self):
        sample_count = len(self.calibration_x_g_samples)
        if sample_count < self.calibration_min_samples:
            raise RuntimeError(
                "IMU X/Y Calibration sample 부족 "
                f"{sample_count}/{self.calibration_min_samples}"
            )

        self.imu_x_reference_g = sum(self.calibration_x_g_samples) / sample_count
        self.imu_y_reference_g = sum(self.calibration_y_g_samples) / sample_count
        self.imu_x_reference_raw = sum(self.calibration_x_raw_samples) / sample_count
        self.imu_y_reference_raw = sum(self.calibration_y_raw_samples) / sample_count
        self.pitch_reference_deg = sum(self.calibration_pitch_samples) / sample_count
        self.roll_reference_deg = sum(self.calibration_roll_samples) / sample_count

        self.calibrating = False
        self.calibrated_session = True
        self.motor3_imu_y_pid.reset()
        self.motor4_imu_x_pid.reset()
        self.last_update_time = None

        self._pending_calibration_result = {
            "x_reference_g": self.imu_x_reference_g,
            "y_reference_g": self.imu_y_reference_g,
            "x_reference_raw": self.imu_x_reference_raw,
            "y_reference_raw": self.imu_y_reference_raw,
            "pitch_reference_deg": self.pitch_reference_deg,
            "roll_reference_deg": self.roll_reference_deg,
            "sample_count": sample_count,
        }

        print(
            "[IMU] X/Y Reference Calibration 완료 "
            f"samples={sample_count} "
            f"Xref={self.imu_x_reference_g:+.5f}g "
            f"Yref={self.imu_y_reference_g:+.5f}g"
        )

    def _build_state_from_sample(
        self,
        sample,
        imu_x_error_g=0.0,
        imu_y_error_g=0.0,
        motor3_pid=None,
        motor4_pid=None,
        remaining=0.0,
    ):
        motor3_pid = motor3_pid or dict(self.motor3_imu_y_pid.last)
        motor4_pid = motor4_pid or dict(self.motor4_imu_x_pid.last)

        current_pitch = float(sample.get("absolute_pitch_deg", 0.0))
        current_roll = float(sample.get("absolute_roll_deg", 0.0))
        delta_pitch = current_pitch - self.pitch_reference_deg
        delta_roll = current_roll - self.roll_reference_deg

        # 아래 old field는 기존 UI/debug를 깨지 않기 위한 호환 alias일 뿐이다.
        # 실제 Motor34Controller는 motor3_correction_speed_deg_s / motor4_...만 사용한다.
        return {
            "available": True,
            "calibrating": bool(self.calibrating),
            "calibrated": bool(self.calibrated),
            "calibration_remaining_sec": float(remaining),
            "calibration_sample_count": len(self.calibration_x_g_samples),
            "calibration_min_samples": self.calibration_min_samples,
            **sample,
            "imu_x_reference_g": self.imu_x_reference_g,
            "imu_y_reference_g": self.imu_y_reference_g,
            "imu_x_reference_raw": self.imu_x_reference_raw,
            "imu_y_reference_raw": self.imu_y_reference_raw,
            "imu_x_error_g": float(imu_x_error_g),
            "imu_y_error_g": float(imu_y_error_g),
            "motor3_imu_y_pid": dict(motor3_pid),
            "motor4_imu_x_pid": dict(motor4_pid),
            "motor3_correction_speed_deg_s": float(motor3_pid.get("output", 0.0)),
            "motor4_correction_speed_deg_s": float(motor4_pid.get("output", 0.0)),
            "output_limit_deg_s": self.output_limit_deg_s,
            # Diagnostic angle fields
            "pitch_reference_deg": self.pitch_reference_deg,
            "roll_reference_deg": self.roll_reference_deg,
            "pitch_deg": delta_pitch if self.calibrated else 0.0,
            "roll_deg": delta_roll if self.calibrated else 0.0,
            # Legacy alias
            "pitch_offset_deg": self.pitch_reference_deg,
            "roll_offset_deg": self.roll_reference_deg,
            "reference_pitch_deg": self.pitch_reference_deg,
            "reference_roll_deg": self.roll_reference_deg,
            "correction_pitch_deg": -delta_pitch if self.calibrated else 0.0,
            "correction_roll_deg": -delta_roll if self.calibrated else 0.0,
            "correction_pitch_speed_deg_s": float(motor3_pid.get("output", 0.0)),
            "correction_roll_speed_deg_s": float(motor4_pid.get("output", 0.0)),
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
            sample = self._read_filtered_sample()

            if self.calibrating:
                self.calibration_x_g_samples.append(sample["filtered_x_g"])
                self.calibration_y_g_samples.append(sample["filtered_y_g"])
                self.calibration_x_raw_samples.append(sample["raw_x"])
                self.calibration_y_raw_samples.append(sample["raw_y"])
                self.calibration_pitch_samples.append(sample["absolute_pitch_deg"])
                self.calibration_roll_samples.append(sample["absolute_roll_deg"])

                elapsed = now_monotonic - self.calibration_started_at
                remaining = max(0.0, self.calibration_sec - elapsed)
                sample_count = len(self.calibration_x_g_samples)

                if (
                    elapsed >= self.calibration_sec
                    and sample_count >= self.calibration_min_samples
                ):
                    self._finish_calibration()
                    remaining = 0.0

                state = self._build_state_from_sample(
                    sample,
                    remaining=remaining if self.calibrating else 0.0,
                )
                self.latest_state = state
                return state

            if not self.calibrated:
                state = self._build_state_from_sample(sample)
                self.latest_state = state
                return state

            imu_x_error_g = sample["filtered_x_g"] - self.imu_x_reference_g
            imu_y_error_g = sample["filtered_y_g"] - self.imu_y_reference_g

            if self.last_update_time is None:
                dt = self.sample_interval
            else:
                dt = clamp(
                    now_monotonic - self.last_update_time,
                    0.001,
                    0.2,
                )
            self.last_update_time = now_monotonic

            # 핵심 Direct IMU 매핑
            # Motor3 <- IMU Y
            # Motor4 <- IMU X
            motor3_pid = self.motor3_imu_y_pid.update(imu_y_error_g, dt)
            motor4_pid = self.motor4_imu_x_pid.update(imu_x_error_g, dt)

            state = self._build_state_from_sample(
                sample,
                imu_x_error_g=imu_x_error_g,
                imu_y_error_g=imu_y_error_g,
                motor3_pid=motor3_pid,
                motor4_pid=motor4_pid,
            )

            self.last_error = None
            self.latest_state = state
            return state

        except Exception as error:
            self.last_error = str(error)
            state = dict(self.latest_state) if isinstance(self.latest_state, dict) else self._empty_state()
            state["available"] = False
            state["last_error"] = self.last_error
            state["timestamp"] = time.time()
            self.latest_state = state
            return state

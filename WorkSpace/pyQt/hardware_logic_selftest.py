"""RPi 실물 없이 Config/State/IMU/Motor 핵심 계산을 검증하는 간단 self-test."""
import json
import tempfile
import time
from pathlib import Path

from services.hardware_config_service import HardwareConfigService
from services.hardware_constants import (
    IMU_ADDRESS,
    IMU_BUS,
    IMU_CALIBRATION_SEC,
    IMU_DEADBAND_DEG,
    IMU_LPF_ALPHA,
    IMU_SAMPLE_HZ,
    IR_ACTIVE_LOW,
    IR_LOST_GRACE_SEC,
    IR_PIN,
    IR_SAMPLE_HZ,
    IR_STABLE_DETECT_SEC,
)
from services.hardware_state_store import HardwareRuntimeStateStore
from services.imu_service import ADXL345IMUService
from services.ir_service import IRSensorService
from services.motor_service import MonitorMotorService


def _pack_signed(value):
    if value < 0:
        value += 65536
    return [value & 0xFF, (value >> 8) & 0xFF]


class FakeBus:
    def __init__(self, _bus_number):
        self.raw = (0, 0, 256)

    def write_byte_data(self, *_args):
        pass

    def read_i2c_block_data(self, *_args):
        x, y, z = self.raw
        return _pack_signed(x) + _pack_signed(y) + _pack_signed(z)

    def close(self):
        pass


class FakeCalibration:
    def get_safe_angle_range(self, _joint):
        return -20.0, 20.0


class FakeController:
    def __init__(self, calibration_file=None):
        self.calibration = FakeCalibration()
        self.read_count = 0
        self.moves = []

    def get_joint_angle(self, _joint):
        self.read_count += 1
        return 0.0

    def move_joint(self, joint, angle, speed, wait=False):
        self.moves.append((joint, angle, speed, wait))
        return True

    def close(self):
        pass


def main():
    # Production 생성자는 JSON이 아니라 코드 상수 기본값을 사용한다.
    ir_defaults = IRSensorService(gpio_module=object())
    assert ir_defaults.pin == IR_PIN
    assert ir_defaults.active_low == IR_ACTIVE_LOW
    assert ir_defaults.sample_hz == IR_SAMPLE_HZ
    assert ir_defaults.stable_detect_sec == IR_STABLE_DETECT_SEC
    assert ir_defaults.lost_grace_sec == IR_LOST_GRACE_SEC

    imu_defaults = ADXL345IMUService(bus_factory=lambda _number: None)
    assert imu_defaults.bus_number == IMU_BUS
    assert imu_defaults.address == IMU_ADDRESS
    assert imu_defaults.sample_hz == IMU_SAMPLE_HZ
    assert imu_defaults.calibration_sec == IMU_CALIBRATION_SEC
    assert imu_defaults.imu_alpha == IMU_LPF_ALPHA
    assert imu_defaults.deadband_deg == IMU_DEADBAND_DEG

    with tempfile.TemporaryDirectory() as td:
        config = HardwareConfigService(Path(td) / "hardware_control.json")
        data = config.load()
        data = config.update_control({"pid": {"pitch": {"kp": 8.5}}})
        assert data["control"]["pid"]["pitch"]["kp"] == 8.5
        assert data["control"]["pid"]["pitch"]["ki"] == 0.0
        config.update_imu_calibration(1.2, -0.7, 100)

    store = HardwareRuntimeStateStore()
    store.update({"ir": {"available": True, "detected": True}})
    assert store.is_ir_detected()

    holder = {}

    def bus_factory(number):
        bus = FakeBus(number)
        holder["bus"] = bus
        return bus

    imu = ADXL345IMUService(
        calibration_sec=0.1,
        sample_hz=50,
        bus_factory=bus_factory,
    )
    assert imu.open()
    assert not imu.calibrated
    assert imu.start_calibration()

    start = time.monotonic()
    while time.monotonic() - start < 0.13:
        imu_state = imu.update()
        time.sleep(0.01)
    assert imu_state["calibrated"]

    holder["bus"].raw = (-45, 0, 252)
    for _ in range(12):
        imu_state = imu.update()
    assert abs(imu_state["correction_pitch_speed_deg_s"]) > 0.0

    with tempfile.TemporaryDirectory() as td:
        servo_file = Path(td) / "servo.json"
        servo_file.write_text(
            json.dumps({"servos": {"3": {"joint": "wrist_flex", "max_speed": 100}}}),
            encoding="utf-8",
        )
        made = {}

        def controller_factory(calibration_file=None):
            controller = FakeController(calibration_file)
            made["controller"] = controller
            return controller

        motor = MonitorMotorService(
            calibration_file=servo_file,
            command_hz=1000,
            controller_factory=controller_factory,
        )
        motor.apply_control_config({
            "command_hz": 1000,
            "pid_speed_deadband_deg_s": 0.25,
            "motor3": {"enabled": True, "direction_sign": 1.0},
            "motor4": {"enabled": False},
        })
        assert motor.open()

        zero = {
            "available": True,
            "calibrated": True,
            "calibrating": False,
            "correction_pitch_speed_deg_s": 0.0,
            "correction_roll_speed_deg_s": 0.0,
        }
        motor.update(zero, True, 30.0)
        assert made["controller"].read_count == 0
        assert len(made["controller"].moves) == 0

        time.sleep(0.002)
        moving = dict(zero)
        moving["correction_pitch_speed_deg_s"] = 10.0
        motor.update(moving, True, 30.0)
        assert made["controller"].read_count == 1
        assert len(made["controller"].moves) == 1

    print("Hardware logic self-test: PASS")


if __name__ == "__main__":
    main()

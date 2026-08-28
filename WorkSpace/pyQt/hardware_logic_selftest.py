"""RPi 실물 없이 Config/State/IMU/Motor 핵심 계산을 검증하는 self-test."""
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
from services.motor_service import MotorService
from services.motor12_controller import Motor12Controller
from services.motor34_controller import Motor34Controller


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
    device = "/dev/fake"
    baudrate = 1000000

    def get_joint(self, joint):
        table = {
            "shoulder_lift": {"servo_id": 1, "joint": "shoulder_lift", "max_speed": 100},
            "elbow_flex": {"servo_id": 2, "joint": "elbow_flex", "max_speed": 100},
            "wrist_flex": {"servo_id": 3, "joint": "wrist_flex", "max_speed": 100},
            "wrist_roll": {"servo_id": 4, "joint": "wrist_roll", "max_speed": 100},
        }
        return table[joint]

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

    def move_joint_relative(self, joint, delta_angle, speed, wait=False):
        self.moves.append((joint, delta_angle, speed, wait))
        return True

    def close(self):
        pass


def main():
    # --------------------------------------------------------
    # Hardware constructor는 JSON이 아니라 코드 상수 사용
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # JSON은 PID/Filter/Motor tuning만 보관
    # --------------------------------------------------------
    with tempfile.TemporaryDirectory() as td:
        config = HardwareConfigService(Path(td) / "hardware_control.json")
        data = config.load()
        assert data["version"] == 3
        assert "ir" not in data["control"]
        assert set(data["control"]["imu"].keys()) == {"lpf_alpha", "deadband_deg"}
        assert data["control"]["motor"]["motor3"]["enabled"] is True
        assert data["control"]["motor"]["motor4"]["enabled"] is True
        assert "pass" not in data["control"]["motor"]["motor4"]

        data = config.update_control({"pid": {"pitch": {"kp": 8.5}}})
        assert data["control"]["pid"]["pitch"]["kp"] == 8.5
        assert data["control"]["pid"]["pitch"]["ki"] == 0.0
        config.update_imu_calibration(1.2, -0.7, 100)

    store = HardwareRuntimeStateStore()
    store.update({"ir": {"available": True, "detected": True}})
    assert store.is_ir_detected()

    # --------------------------------------------------------
    # IMU Offset / PID
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # Shared MotorService + Motor12 + Motor34
    # --------------------------------------------------------
    with tempfile.TemporaryDirectory() as td:
        servo_file = Path(td) / "servo.json"
        servo_file.write_text(json.dumps({"servos": {"3": {}}}), encoding="utf-8")
        made = {}

        def controller_factory(calibration_file=None):
            controller = FakeController(calibration_file)
            made["controller"] = controller
            return controller

        motor = MotorService(
            calibration_file=servo_file,
            controller_factory=controller_factory,
        )
        assert motor.open()

        motor12 = Motor12Controller(motor, update_hz=1000)
        motor34 = Motor34Controller(motor)
        motor34.apply_control_config({
            "command_hz": 1000,
            "pid_speed_deadband_deg_s": 0.25,
            "motor3": {"enabled": True, "direction_sign": 1.0},
            "motor4": {"enabled": True, "direction_sign": 1.0},
        })
        assert motor34.initialize()

        zero_imu = {
            "available": True,
            "calibrated": True,
            "calibrating": False,
            "pitch_deg": 0.0,
            "correction_pitch_speed_deg_s": 0.0,
            "correction_roll_speed_deg_s": 0.0,
        }
        context = {
            "now": time.monotonic(),
            "imu": zero_imu,
            "motor34_control_active": True,
            "pid_limit_deg_s": 30.0,
        }

        # 1/2번은 현재 pass -> packet 없음
        motor12.update(context)
        # Motor3(Roll), Motor4(Pitch) PID=0 -> read/move packet 없음
        motor34.update(context)
        assert made["controller"].read_count == 0
        assert len(made["controller"].moves) == 0

        # Roll 오차 -> Motor3(wrist_flex)만 움직여야 한다.
        time.sleep(0.002)
        moving_roll = dict(zero_imu)
        moving_roll["correction_roll_speed_deg_s"] = 10.0
        context["now"] = time.monotonic()
        context["imu"] = moving_roll
        state = motor34.update(context)
        assert made["controller"].read_count == 1
        assert len(made["controller"].moves) == 1
        assert made["controller"].moves[0][0] == "wrist_flex"
        assert state["motor3"]["axis"] == "roll"

        # Pitch 오차 -> Motor4(wrist_roll)만 새로 움직여야 한다.
        time.sleep(0.002)
        moving_pitch = dict(zero_imu)
        moving_pitch["correction_pitch_speed_deg_s"] = 10.0
        context["now"] = time.monotonic()
        context["imu"] = moving_pitch
        state = motor34.update(context)
        assert made["controller"].read_count == 2
        assert len(made["controller"].moves) == 2
        assert made["controller"].moves[1][0] == "wrist_roll"
        assert state["motor4"]["axis"] == "pitch"

        motor.close()

    print("Hardware logic self-test: PASS")


if __name__ == "__main__":
    main()

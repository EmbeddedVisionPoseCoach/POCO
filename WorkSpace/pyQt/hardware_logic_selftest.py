"""RPi 실물 없이 Direct IMU / Motor3/4 통합 핵심 계산을 검증하는 self-test."""
import json
import tempfile
import time
from pathlib import Path

from services.hardware_config_service import HardwareConfigService
from services.hardware_constants import (
    IMU_ADDRESS,
    IMU_BUS,
    IMU_CALIBRATION_SEC,
    IMU_DEADBAND_G,
    IMU_LPF_ALPHA,
    IMU_SAMPLE_HZ,
    MOTOR34_AUTO_ACC,
    MOTOR34_AUTO_SPEED,
    MOTOR34_COMMAND_HZ,
    MOTOR3_IMU_Y_KP,
    MOTOR4_IMU_X_KP,
    PID_OUTPUT_LIMIT_DEG_S,
)
from services.hardware_state_store import HardwareRuntimeStateStore
from services.imu_service import ADXL345IMUService
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
            "shoulder_lift": {"servo_id": 1, "joint": "shoulder_lift", "max_speed": 1000},
            "elbow_flex": {"servo_id": 2, "joint": "elbow_flex", "max_speed": 1000},
            "wrist_flex": {"servo_id": 3, "joint": "wrist_flex", "max_speed": 1000},
            "wrist_roll": {"servo_id": 4, "joint": "wrist_roll", "max_speed": 1000},
        }
        return table[joint]

    def get_safe_angle_range(self, _joint):
        return -90.0, 90.0


class FakeController:
    def __init__(self, calibration_file=None):
        self.calibration = FakeCalibration()
        self.read_count = 0
        self.moves = []

    def get_joint_angle(self, _joint):
        self.read_count += 1
        return 0.0

    def move_joint(self, joint, angle, speed, acc=10, wait=False):
        self.moves.append((joint, angle, speed, acc, wait))
        return True

    def move_joint_relative(self, joint, delta_angle, speed, acc=10, wait=False):
        self.moves.append((joint, delta_angle, speed, acc, wait))
        return True

    def close(self):
        pass


def main():
    # --------------------------------------------------------
    # Hardware constants
    # --------------------------------------------------------

    imu_defaults = ADXL345IMUService(bus_factory=lambda _number: None)
    assert imu_defaults.bus_number == IMU_BUS
    assert imu_defaults.address == IMU_ADDRESS
    assert imu_defaults.sample_hz == IMU_SAMPLE_HZ == 100.0
    assert imu_defaults.calibration_sec == IMU_CALIBRATION_SEC
    assert imu_defaults.imu_alpha == IMU_LPF_ALPHA == 0.08
    assert imu_defaults.deadband_g == IMU_DEADBAND_G == 0.010
    assert imu_defaults.motor3_imu_y_pid.kp == MOTOR3_IMU_Y_KP == 120.0
    assert imu_defaults.motor4_imu_x_pid.kp == MOTOR4_IMU_X_KP == 120.0
    assert imu_defaults.output_limit_deg_s == PID_OUTPUT_LIMIT_DEG_S == 24.0

    # --------------------------------------------------------
    # JSON schema v5 / stale v4 migration
    # --------------------------------------------------------
    with tempfile.TemporaryDirectory() as td:
        config = HardwareConfigService(Path(td) / "hardware_control.json")
        data = config.load()
        assert data["version"] == 5
        assert set(data["control"]["imu"].keys()) == {"lpf_alpha", "deadband_g"}
        assert data["control"]["pid"]["motor3_imu_y"]["kp"] == 120.0
        assert data["control"]["pid"]["motor4_imu_x"]["kp"] == 120.0
        assert data["control"]["motor"]["command_hz"] == MOTOR34_COMMAND_HZ
        assert data["control"]["motor"]["auto_speed"] == MOTOR34_AUTO_SPEED
        assert data["control"]["motor"]["auto_acc"] == MOTOR34_AUTO_ACC
        assert data["control"]["motor"]["motor3"]["direction_sign"] == +1.0
        assert data["control"]["motor"]["motor4"]["direction_sign"] == +1.0

        data = config.update_control({
            "pid": {"motor3_imu_y": {"kp": 135.0}}
        })
        assert data["control"]["pid"]["motor3_imu_y"]["kp"] == 135.0
        assert data["control"]["pid"]["motor3_imu_y"]["ki"] == 0.0

        config.update_imu_calibration(
            0.012,
            -0.034,
            100,
            x_reference_raw=3.0,
            y_reference_raw=-9.0,
        )
        saved = config.get_imu_calibration()
        assert saved["x_reference_g"] == 0.012
        assert saved["y_reference_g"] == -0.034

    # 예전 v4 JSON의 alpha=.20/sign=-1이 새 런타임을 덮어쓰지 않는지 확인.
    with tempfile.TemporaryDirectory() as td:
        stale_path = Path(td) / "hardware_control.json"
        stale_path.write_text(json.dumps({
            "version": 4,
            "control": {
                "imu": {"lpf_alpha": 0.20, "deadband_g": 0.02},
                "pid": {
                    "motor3_imu_y": {"kp": 60.0, "ki": 1.0, "kd": 2.0},
                    "motor4_imu_x": {"kp": 70.0, "ki": 1.0, "kd": 2.0},
                    "output_limit_deg_s": 10.0,
                },
                "motor": {
                    "command_hz": 50.0,
                    "auto_speed": 100,
                    "auto_acc": 5,
                    "motor3": {"enabled": True, "direction_sign": -1.0},
                    "motor4": {"enabled": True, "direction_sign": -1.0},
                },
            },
        }), encoding="utf-8")
        migrated = HardwareConfigService(stale_path).load()
        assert migrated["version"] == 5
        assert migrated["control"]["imu"]["lpf_alpha"] == 0.08
        assert migrated["control"]["imu"]["deadband_g"] == 0.010
        assert migrated["control"]["pid"]["motor3_imu_y"]["kp"] == 120.0
        assert migrated["control"]["pid"]["motor4_imu_x"]["kp"] == 120.0
        assert migrated["control"]["pid"]["output_limit_deg_s"] == 24.0
        assert migrated["control"]["motor"]["command_hz"] == 100.0
        assert migrated["control"]["motor"]["auto_speed"] == 500
        assert migrated["control"]["motor"]["auto_acc"] == 12
        assert migrated["control"]["motor"]["motor3"]["direction_sign"] == +1.0
        assert migrated["control"]["motor"]["motor4"]["direction_sign"] == +1.0

    store = HardwareRuntimeStateStore()
    store.update({"imu": {"available": True}, "motor": {"ready": True}})
    assert store.get_imu_state().get("available") is True
    assert store.get_motor_state().get("ready") is True

    # --------------------------------------------------------
    # Direct IMU Calibration / PID
    # --------------------------------------------------------
    holder = {}

    def bus_factory(number):
        bus = FakeBus(number)
        holder["bus"] = bus
        return bus

    imu = ADXL345IMUService(
        calibration_sec=0.10,
        calibration_min_samples=5,
        sample_hz=100,
        bus_factory=bus_factory,
    )
    assert imu.open()
    assert not imu.calibrated
    assert imu.start_calibration()

    start = time.monotonic()
    while time.monotonic() - start < 0.13:
        imu_state = imu.update()
        time.sleep(0.005)
    assert imu_state["calibrated"]
    assert abs(imu_state["imu_x_reference_g"]) < 1e-9
    assert abs(imu_state["imu_y_reference_g"]) < 1e-9

    # Y 변화 -> Motor3 PID만 유의미하게 출력
    holder["bus"].raw = (0, 30, 254)
    for _ in range(25):
        imu_state = imu.update()
        time.sleep(0.001)

    assert abs(imu_state["imu_y_error_g"]) > IMU_DEADBAND_G
    assert abs(imu_state["motor3_correction_speed_deg_s"]) > 0.0
    assert abs(imu_state["motor4_correction_speed_deg_s"]) < 1e-9

    imu.close()

    # --------------------------------------------------------
    # Shared MotorService + Motor12 untouched + Motor34 Direct
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
            "auto_speed": 500,
            "auto_acc": 12,
            "motor3": {"enabled": True, "direction_sign": +1.0},
            "motor4": {"enabled": True, "direction_sign": +1.0},
        })
        assert motor34.initialize()

        zero_imu = {
            "available": True,
            "calibrated": True,
            "calibrating": False,
            "imu_x_error_g": 0.0,
            "imu_y_error_g": 0.0,
            "motor3_correction_speed_deg_s": 0.0,
            "motor4_correction_speed_deg_s": 0.0,
        }
        context = {
            "now": time.monotonic(),
            "imu": zero_imu,
            "motor34_control_active": True,
        }

        # 1/2번은 현재 pass. 기존 로직을 건드리지 않는다.
        motor12.update(context)
        assert len(made["controller"].moves) == 0

        # Closed loop 첫 진입은 M3/M4 현재각을 각각 1회 읽고,
        # V1.9처럼 고정 Speed/Acc로 현재 target을 write한다.
        motor34.update(context)
        assert made["controller"].read_count == 2
        assert len(made["controller"].moves) == 2
        assert made["controller"].moves[0][0] == "wrist_flex"
        assert made["controller"].moves[1][0] == "wrist_roll"
        assert made["controller"].moves[0][2] == 500
        assert made["controller"].moves[0][3] == 12

        # IMU Y PID -> Motor3 target 변화. Motor4는 같은 target 유지.
        time.sleep(0.002)
        moving_y = dict(zero_imu)
        moving_y["imu_y_error_g"] = 0.05
        moving_y["motor3_correction_speed_deg_s"] = -6.0
        context["now"] = time.monotonic()
        context["imu"] = moving_y
        state = motor34.update(context)

        assert len(made["controller"].moves) == 4
        m3_move = made["controller"].moves[-2]
        m4_move = made["controller"].moves[-1]
        assert m3_move[0] == "wrist_flex"
        assert m4_move[0] == "wrist_roll"
        assert m3_move[1] < 0.0  # TEAM angle: output -6 * sign +1 => - target
        assert abs(m4_move[1]) < 1e-9
        assert state["motor3"]["axis"] == "imu_y"
        assert state["motor4"]["axis"] == "imu_x"

        # IMU X PID -> Motor4 target 변화.
        time.sleep(0.002)
        moving_x = dict(zero_imu)
        moving_x["imu_x_error_g"] = 0.05
        moving_x["motor4_correction_speed_deg_s"] = -6.0
        context["now"] = time.monotonic()
        context["imu"] = moving_x
        state = motor34.update(context)

        assert len(made["controller"].moves) == 6
        m4_move = made["controller"].moves[-1]
        assert m4_move[0] == "wrist_roll"
        assert m4_move[1] < 0.0
        assert state["axis_mapping"] == {"motor3": "imu_y", "motor4": "imu_x"}

        motor.close()

    print("Hardware logic self-test: PASS")


if __name__ == "__main__":
    main()

import sys
import unittest
from pathlib import Path


BH_CODE_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = BH_CODE_DIR.parent.parent
PYQT_DIR = WORKSPACE_DIR / "pyQt"
for path in (BH_CODE_DIR, PYQT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from services.tof_service import FixedToFSensorService, ToFSensorService
from pose_monitor_arm_controller import (
    EyeGapVisionDistanceEstimator,
    ToFUserXSource,
    UserXFusion,
)


class FakeI2C:
    def __init__(self, bus_number):
        self.bus_number = bus_number
        self.deinitialized = False

    def deinit(self):
        self.deinitialized = True


class FakeVL53L0X:
    def __init__(self, i2c, address, io_timeout_s):
        self.i2c = i2c
        self.address = address
        self.io_timeout_s = io_timeout_s
        self.values = iter((700, 900, 2500))
        self.started = False
        self.stopped = False

    def start_continuous(self):
        self.started = True

    def stop_continuous(self):
        self.stopped = True

    @property
    def range(self):
        return next(self.values)


class ToFSensorServiceTests(unittest.TestCase):
    def setUp(self):
        self.sensor_holder = {}

        def sensor_factory(*args, **kwargs):
            sensor = FakeVL53L0X(*args, **kwargs)
            self.sensor_holder["sensor"] = sensor
            return sensor

        self.service = ToFSensorService(
            bus_number=3,
            address=0x29,
            minimum_range_m=0.03,
            maximum_range_m=2.0,
            filter_alpha=0.5,
            i2c_factory=FakeI2C,
            sensor_factory=sensor_factory,
        )

    def test_i2c3_millimetres_are_converted_and_filtered(self):
        self.assertTrue(self.service.open())
        first = self.service.update(force=True)
        second = self.service.update(force=True)
        self.assertEqual(first["device_path"], "/dev/i2c-3")
        self.assertEqual(first["address"], 0x29)
        self.assertAlmostEqual(first["filtered_distance_m"], 0.7)
        self.assertAlmostEqual(second["filtered_distance_m"], 0.8)

    def test_out_of_range_sensor_value_is_invalid(self):
        self.service.open()
        self.service.update(force=True)
        self.service.update(force=True)
        invalid = self.service.update(force=True)
        self.assertFalse(invalid["valid"])
        self.assertIn("허용범위", invalid["last_error"])
        self.assertAlmostEqual(invalid["last_valid_distance_m"], 0.8)


class FusionTests(unittest.TestCase):
    def test_tof_user_x_uses_sensor_origin(self):
        service = FixedToFSensorService(0.70)
        source = ToFUserXSource(service, 0.02, 0.60, 0.83)
        source.open()
        self.assertAlmostEqual(source.read_user_x_m(), 0.72)

    def test_eye_gap_distance_is_inverse_proportional(self):
        estimator = EyeGapVisionDistanceEstimator(5.0, 0.2, 1.2, filter_alpha=1.0)
        estimator.calibrate(60.0, 0.5)
        self.assertAlmostEqual(estimator.estimate_distance_m(75.0), 0.4)
        self.assertAlmostEqual(estimator.estimate_distance_m(50.0), 0.6)

    def test_fusion_is_tof_70_percent_and_vision_30_percent(self):
        fusion = UserXFusion(0.7, 0.3)
        self.assertAlmostEqual(fusion.fuse(0.70, 0.80), 0.73)

    def test_missing_vision_falls_back_to_tof(self):
        fusion = UserXFusion(0.7, 0.3)
        self.assertAlmostEqual(fusion.fuse(0.70, None), 0.70)


if __name__ == "__main__":
    unittest.main()

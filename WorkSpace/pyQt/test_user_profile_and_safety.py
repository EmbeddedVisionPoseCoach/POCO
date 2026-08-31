import json
import tempfile
import unittest
from pathlib import Path

from services.monitor_arm_safety_supervisor import MonitorArmSafetySupervisor
from services.user_profile_service import UserProfileService


class MonitorArmSafetySupervisorTest(unittest.TestCase):
    def test_either_sensor_missing_returns_once_after_timeout(self):
        safety = MonitorArmSafetySupervisor(absence_timeout_sec=5.0)
        first = safety.update(False, True, None, now=10.0)
        self.assertEqual(first["state"], safety.SENSOR_GRACE)
        at_timeout = safety.update(False, True, None, now=15.0)
        self.assertTrue(at_timeout["request_return"])
        repeated = safety.update(False, True, None, now=16.0)
        self.assertFalse(repeated["request_return"])

    def test_bad_posture_holds_and_optimal_tracks(self):
        safety = MonitorArmSafetySupervisor(reacquire_stable_sec=0.0)
        bad = safety.update(
            True, True,
            {"posture_type": "Forward Head", "confidence": 0.95},
            now=1.0,
        )
        self.assertEqual(bad["state"], safety.POSTURE)
        good = safety.update(
            True, True,
            {"posture_type": "Optimal", "confidence": 0.95},
            now=2.0,
        )
        self.assertTrue(good["tracking_allowed"])


class UserProfileServiceTest(unittest.TestCase):
    def test_four_slots_and_bundle_save_load(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            pose = root / "baseline.pkl"
            pose.write_bytes(b"pose")
            service = UserProfileService(root / "profiles")
            hardware = {
                "monitor_arm": {"calibration": {
                    "ready": True,
                    "session_ready": True,
                    "tof_user_x_baseline_m": 0.7,
                    "eye_gap_baseline_px": 42.0,
                    "monitor_x_baseline_m": 0.2,
                    "motor_angles_deg": {
                        "shoulder_lift": 1.0, "elbow_flex": 2.0,
                        "wrist_flex": 3.0, "wrist_roll": 4.0,
                    },
                }},
                "imu": {
                    "calibrated": True,
                    "imu_x_reference_g": 0.1,
                    "imu_y_reference_g": 0.2,
                    "imu_x_reference_raw": 10,
                    "imu_y_reference_raw": 20,
                    "pitch_reference_deg": 1.5,
                    "roll_reference_deg": 2.5,
                },
            }
            saved = service.save_profile(1, "사용자", pose, None, hardware, True, False)
            self.assertEqual(saved["name"], "사용자")
            self.assertEqual(len(service.list_profiles()), 4)
            self.assertTrue(service.list_profiles()[0]["occupied"])
            bundle = service.load_profile(1)
            self.assertEqual(bundle["motor_angles_deg"]["wrist_roll"], 4.0)
            with self.assertRaises(ValueError):
                service.slot_dir(5)


if __name__ == "__main__":
    unittest.main()

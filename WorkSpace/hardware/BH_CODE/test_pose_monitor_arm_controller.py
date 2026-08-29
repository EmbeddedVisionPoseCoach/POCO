import unittest
from pathlib import Path

from manual_motor12_limit_ui import ManualMotor12Bus
from manual_vertical_ik_ui import REST_POSE, calculate_adaptive_speed
from monitor_arm_motor_process import TwoMotorHardware
from monitor_arm_visualizer import calculate_arm_points
from monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    MotionSafetyError,
    SafetyLimits,
    TwoJointMonitorArm,
    load_settings,
    monitor_target_from_user,
)
from pose_monitor_arm_controller import (
    FixedToFUserXSource,
    MonitorArmPlanner,
)


ROOT_DIR = Path(__file__).resolve().parent


class FakeManualDriver:
    def __init__(self):
        self.writes = []
        self.sync_writes = []
        self.positions = {1: 1652, 2: 2010}

    def write_position(self, servo_id, position, speed, acc):
        self.writes.append((servo_id, position, speed, acc))
        self.positions[servo_id] = position
        return True

    def read_position(self, servo_id):
        return self.positions[servo_id]

    def sync_write_positions(self, commands):
        self.sync_writes.append(commands)
        for servo_id, command in commands.items():
            self.positions[servo_id] = command["position"]
        return True


class ManualJogBusTests(unittest.TestCase):
    def test_single_joint_jog_packet_contains_only_selected_servo(self):
        bus = ManualMotor12Bus(ROOT_DIR / "servo_calibration_result.json")
        fake = FakeManualDriver()
        bus.driver = fake
        bus.move_joint("shoulder_lift", 1.0, speed=10, acc=3)
        self.assertEqual([item[0] for item in fake.writes], [1])

    def test_button_release_hold_contains_only_selected_servo(self):
        bus = ManualMotor12Bus(ROOT_DIR / "servo_calibration_result.json")
        fake = FakeManualDriver()
        bus.driver = fake
        held_angle = bus.hold_joint("elbow_flex")
        self.assertEqual([item[0] for item in fake.writes], [2])
        self.assertAlmostEqual(held_angle, 0.0)

    def test_two_joint_zero_uses_only_ids_1_and_2_calibrated_origins(self):
        bus = ManualMotor12Bus(ROOT_DIR / "servo_calibration_result.json")
        fake = FakeManualDriver()
        bus.driver = fake
        bus.move(JointCommand(0.0, 0.0), speed=10, acc=3)
        commands = fake.sync_writes[-1]
        self.assertEqual(set(commands), {1, 2})
        self.assertEqual(commands[1]["position"], 1652)
        self.assertEqual(commands[2]["position"], 2010)

    def test_confirmed_rest_pose_uses_only_ids_1_and_2(self):
        bus = ManualMotor12Bus(ROOT_DIR / "servo_calibration_result.json")
        fake = FakeManualDriver()
        bus.driver = fake
        bus.move_confirmed_rest_pose(REST_POSE, speed=200, acc=10)
        commands = fake.sync_writes[-1]
        self.assertEqual(set(commands), {1, 2})
        self.assertEqual(commands[1]["position"], 426)
        self.assertEqual(commands[2]["position"], 3063)

    def test_recovery_step_must_move_toward_calibration_range(self):
        bus = ManualMotor12Bus(ROOT_DIR / "servo_calibration_result.json")
        fake = FakeManualDriver()
        bus.driver = fake
        inward = JointCommand(102.75, -88.0)
        bus.move_recovery_aware(REST_POSE, inward, speed=100, acc=5)
        self.assertEqual(set(fake.sync_writes[-1]), {1, 2})

        outward = JointCommand(110.0, -88.0)
        with self.assertRaises(Exception):
            bus.move_recovery_aware(REST_POSE, outward, speed=100, acc=5)


class AdaptiveSpeedTests(unittest.TestCase):
    def test_speed_increases_with_joint_error(self):
        self.assertEqual(calculate_adaptive_speed(50, 500, 0.0, 30.0), 50)
        self.assertEqual(calculate_adaptive_speed(50, 500, 15.0, 30.0), 275)
        self.assertEqual(calculate_adaptive_speed(50, 500, 30.0, 30.0), 500)

    def test_speed_never_exceeds_absolute_cap(self):
        self.assertEqual(calculate_adaptive_speed(50, 2000, 180.0, 30.0), 1000)

    def test_motor_process_selects_speed_from_actual_joint_error(self):
        class FakeArm:
            def __init__(self):
                self.last_speed = None

            def get_joint_angle(self, joint):
                return 0.0

            def move_joints(self, angles, speed, acc, wait):
                self.last_speed = speed
                return True

        hardware = TwoMotorHardware(
            ROOT_DIR / "servo_calibration_result.json",
            speed=500,
            acc=5,
            speed_mode="adaptive",
            minimum_speed=50,
            full_speed_error_deg=30.0,
        )
        hardware.arm = FakeArm()
        hardware.calibration_ranges = {
            "shoulder_lift": (-92.0, 94.0),
            "elbow_flex": (-89.0, 89.0),
        }
        result = hardware.move(JointCommand(15.0, -10.0))
        self.assertEqual(result["speed"], 275)
        self.assertEqual(hardware.arm.last_speed, 275)


class FixedToFUserXSourceTests(unittest.TestCase):
    def test_sensor_origin_plus_range_becomes_user_x(self):
        source = FixedToFUserXSource(0.02, 0.70, 0.60, 0.83)
        self.assertAlmostEqual(source.read_user_x_m(), 0.72)

    def test_out_of_range_user_x_is_rejected(self):
        source = FixedToFUserXSource(0.0, 0.90, 0.60, 0.83)
        with self.assertRaises(ValueError):
            source.read_user_x_m()

    def test_fixed_tof_user_x_converges_to_configured_monitor_distance(self):
        settings = load_settings()
        planner = MonitorArmPlanner(settings)
        current = JointCommand(0.0, 0.0)
        planner.set_vertical_reference(current)
        user_x_m = 0.78

        for _step in range(20):
            target = planner.plan(current, user_x_m)
            if target is None:
                break
            current = target

        actual_distance_m = user_x_m - planner.kinematics.forward(current).x_m
        self.assertLessEqual(
            abs(actual_distance_m - planner.desired_distance_m),
            planner.deadband_m,
        )


class TwoJointKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_settings()
        self.arm = TwoJointMonitorArm(ArmGeometry.from_settings(self.settings))

    def test_monitor_offset_is_seven_centimetres(self):
        self.assertAlmostEqual(self.arm.geometry.monitor_offset_m, 0.07)

    def test_fixed_base_link_is_vertical_and_preserves_original_length(self):
        self.assertAlmostEqual(self.arm.geometry.shoulder_x_m, 0.0)
        self.assertAlmostEqual(
            self.arm.geometry.shoulder_z_m,
            0.13560595853519858,
        )

    def test_user_x_keeps_user_monitor_distance_constant(self):
        for user_x_m in (0.60, 0.72, 0.88):
            target = monitor_target_from_user(user_x_m, 0.50, 0.237)
            self.assertAlmostEqual(user_x_m - target.x_m, 0.50)
            self.assertAlmostEqual(target.z_m, 0.237)

    def test_user_coordinate_target_round_trips_through_ik(self):
        target_pose = monitor_target_from_user(0.80, 0.50, 0.237)
        command = self.arm.inverse(target_pose.x_m, target_pose.z_m)
        solved_pose = self.arm.forward(command)
        self.assertAlmostEqual(solved_pose.x_m, target_pose.x_m)
        self.assertAlmostEqual(solved_pose.z_m, target_pose.z_m)

    def test_forward_inverse_round_trip_uses_only_two_joint_command(self):
        for command in (
            JointCommand(0.0, 0.0),
            JointCommand(8.0, -12.0),
            JointCommand(-10.0, 15.0),
        ):
            with self.subTest(command=command):
                pose = self.arm.forward(command)
                solved = self.arm.inverse(pose.x_m, pose.z_m)
                self.assertAlmostEqual(solved.shoulder_lift_deg, command.shoulder_lift_deg)
                self.assertAlmostEqual(solved.elbow_flex_deg, command.elbow_flex_deg)

    def test_visualizer_monitor_point_matches_forward_kinematics(self):
        command = JointCommand(8.0, -12.0)
        points = calculate_arm_points(self.arm, command)
        pose = self.arm.forward(command)
        self.assertEqual(len(points), 4)
        self.assertAlmostEqual(points[-1][0], pose.x_m)
        self.assertAlmostEqual(points[-1][1], pose.z_m)

    def test_vertical_gauge_target_keeps_connected_x(self):
        current_pose = self.arm.forward(JointCommand(0.0, 0.0))
        for vertical_change_m in (-0.03, -0.01, 0.01, 0.03):
            with self.subTest(vertical_change_m=vertical_change_m):
                solved = self.arm.inverse(
                    current_pose.x_m,
                    current_pose.z_m + vertical_change_m,
                )
                solved_pose = self.arm.forward(solved)
                self.assertAlmostEqual(solved_pose.x_m, current_pose.x_m)
                self.assertAlmostEqual(
                    solved_pose.z_m,
                    current_pose.z_m + vertical_change_m,
                )

    def test_vertical_path_guard_holds_unsafe_motion(self):
        source = self.settings["safety"]
        limits = SafetyLimits(
            shoulder_min_deg=-70.0,
            shoulder_max_deg=70.0,
            elbow_min_deg=-70.0,
            elbow_max_deg=70.0,
            vertical_tolerance_m=0.000001,
            max_joint_step_deg=10.0,
            path_samples=int(source["path_samples"]),
        )
        current = JointCommand(0.0, 0.0)
        target = JointCommand(5.0, -5.0)
        with self.assertRaises(MotionSafetyError):
            self.arm.validate_motion(
                current,
                target,
                self.arm.forward(current).z_m,
                limits,
            )

    def test_planner_keeps_reference_height_within_configured_tolerance(self):
        planner = MonitorArmPlanner(self.settings)
        current = JointCommand(0.0, 0.0)
        reference_z = planner.set_vertical_reference(current)
        current_x = planner.kinematics.forward(current).x_m
        target = planner.plan(current, user_x_m=current_x + 0.52)
        self.assertIsNotNone(target)
        target_pose = planner.kinematics.forward(target)
        self.assertLessEqual(
            abs(target_pose.z_m - reference_z),
            planner.limits.vertical_tolerance_m,
        )

    def test_distance_error_moves_monitor_in_correct_x_direction(self):
        current = JointCommand(0.0, 0.0)
        current_x = self.arm.forward(current).x_m

        farther_planner = MonitorArmPlanner(self.settings)
        farther_target = farther_planner.plan(current, user_x_m=current_x + 0.60)
        self.assertIsNotNone(farther_target)
        self.assertGreater(self.arm.forward(farther_target).x_m, current_x)

        closer_planner = MonitorArmPlanner(self.settings)
        closer_target = closer_planner.plan(current, user_x_m=current_x + 0.40)
        self.assertIsNotNone(closer_target)
        self.assertLess(self.arm.forward(closer_target).x_m, current_x)

    def test_direct_pose_mode_sends_full_ik_target_without_five_degree_clipping(self):
        planner = MonitorArmPlanner(self.settings)
        current = JointCommand(0.0, 0.0)
        current_pose = planner.kinematics.forward(current)

        target = planner.plan(current, user_x_m=current_pose.x_m + 0.60)

        self.assertIsNotNone(target)
        expected = planner.kinematics.inverse(
            current_pose.x_m + planner.max_x_step_m,
            current_pose.z_m,
        )
        self.assertAlmostEqual(target.shoulder_lift_deg, expected.shoulder_lift_deg)
        self.assertAlmostEqual(target.elbow_flex_deg, expected.elbow_flex_deg)
        largest_step = max(
            abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(target.elbow_flex_deg - current.elbow_flex_deg),
        )
        self.assertGreater(largest_step, planner.limits.max_joint_step_deg)

    def test_legacy_stepped_pose_mode_remains_available(self):
        settings = load_settings(ROOT_DIR / "monitor_arm_settings.json")
        settings["control"]["pose_joint_command_mode"] = "stepped"
        planner = MonitorArmPlanner(settings)
        current = JointCommand(0.0, 0.0)

        current_x = planner.kinematics.forward(current).x_m
        target = planner.plan(current, user_x_m=current_x + 0.60)

        self.assertIsNotNone(target)
        largest_step = max(
            abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(target.elbow_flex_deg - current.elbow_flex_deg),
        )
        self.assertAlmostEqual(largest_step, planner.limits.max_joint_step_deg)

    def test_pose_planner_recovers_from_folded_rest_before_distance_control(self):
        planner = MonitorArmPlanner(self.settings)
        current = REST_POSE
        calibration_ranges = {
            "shoulder_lift": (-92.988, 94.746),
            "elbow_flex": (-89.297, 89.121),
        }
        self.assertAlmostEqual(
            planner.set_vertical_reference(current),
            self.settings["manual_cartesian"]["default_monitor_z_m"],
        )
        self.assertTrue(planner.recovery_active)

        for _ in range(30):
            target = planner.plan(
                current,
                user_x_m=0.73,
                calibration_ranges=calibration_ranges,
            )
            if target is None:
                break
            largest_step = max(
                abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
                abs(target.elbow_flex_deg - current.elbow_flex_deg),
            )
            self.assertLessEqual(largest_step, planner.limits.max_joint_step_deg)
            current = target
        else:
            self.fail("휴식자세 복구가 제한 스텝 안에 끝나지 않았습니다.")

        self.assertFalse(planner.recovery_active)
        self.assertAlmostEqual(current.shoulder_lift_deg, 0.0)
        self.assertAlmostEqual(current.elbow_flex_deg, 0.0)


if __name__ == "__main__":
    unittest.main()

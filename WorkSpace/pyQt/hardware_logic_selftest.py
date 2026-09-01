"""RPi 실물 없이 IMU / ToF-Vision / Motor1~4 / Rest-Recovery 핵심 경로를 검증하는 self-test."""
import json
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

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
from services.monitor_arm_kinematics import JointCommand
from services.monitor_arm_planner import MonitorArmPlanner
from services.monitor_arm_preparation_controller import (
    MonitorArmPreparationController,
)
from services.monitor_arm_user_x import (
    EyeGapVisionDistanceEstimator,
    ToFUserXSource,
    UserXFusion,
    measure_pose_eye_gap,
)
from services.motor_service import MotorService
from services.motor12_controller import Motor12Controller
from services.motor34_controller import Motor34Controller
from services.tof_service import FixedToFSensorService
from processes.pose_process_profile import pose_control_landmark_quality
from processes.hardware_process import _read_joint_arrival


WORKSPACE_DIR = Path(__file__).resolve().parents[1]
MONITOR_ARM_SETTINGS_FILE = (
    WORKSPACE_DIR
    / "config"
    / "monitor_arm_settings.json"
)


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
    """실제 Servo 없이 Motor1~4 명령 경계를 검증하는 Fake Controller."""

    def __init__(self, calibration_file=None):
        self.calibration = FakeCalibration()

        self.read_count = 0

        # Motor3/4 단일 Joint 명령
        self.moves = []

        # Motor1/2 일반 SyncWrite
        self.sync_moves = []

        # Motor1/2 Rest / Recovery 특수 SyncWrite
        self.special_sync_moves = []

        # 실제 Servo의 현재 TEAM 각도를 흉내 낸다.
        self.angles = {
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
        }

    def reset_trace(self):
        """현재 각도는 유지하고 명령/읽기 기록만 초기화한다."""
        self.read_count = 0
        self.moves.clear()
        self.sync_moves.clear()
        self.special_sync_moves.clear()

    def get_joint_angle(self, joint):
        self.read_count += 1
        return self.angles[joint]

    def move_joint(
        self,
        joint,
        angle,
        speed,
        acc=10,
        wait=False,
    ):
        angle = float(angle)

        self.moves.append(
            (
                joint,
                angle,
                speed,
                acc,
                wait,
            )
        )

        self.angles[joint] = angle
        return True

    def move_joint_relative(
        self,
        joint,
        delta_angle,
        speed,
        acc=10,
        wait=False,
    ):
        delta_angle = float(delta_angle)

        self.moves.append(
            (
                joint,
                delta_angle,
                speed,
                acc,
                wait,
            )
        )

        self.angles[joint] += delta_angle
        return True

    def move_joints(
        self,
        targets,
        speed,
        acc=10,
        wait=False,
    ):
        """Motor1/2 일반 pair SyncWrite를 기록한다."""
        normalized = {
            str(joint): float(angle)
            for joint, angle
            in dict(targets).items()
        }

        self.sync_moves.append(
            {
                "targets": normalized,
                "speed": int(speed),
                "acc": int(acc),
                "wait": bool(wait),
            }
        )

        # Fake에서는 명령을 받으면 즉시 목표에 도달했다고 가정한다.
        self.angles.update(normalized)

        return True

    def move_joints_special(
        self,
        targets,
        speed,
        acc=10,
        wait=False,
    ):
        """Motor1/2 Rest/Recovery 특수 SyncWrite를 기록한다."""
        normalized = {
            str(joint): float(angle)
            for joint, angle
            in dict(targets).items()
        }

        self.special_sync_moves.append(
            {
                "targets": normalized,
                "speed": int(speed),
                "acc": int(acc),
                "wait": bool(wait),
            }
        )

        self.angles.update(normalized)

        return True

    def close(self):
        pass


def main():
    # --------------------------------------------------------
    # Motor1/2 고정 프로젝트 설정
    # --------------------------------------------------------
    monitor_arm_settings = json.loads(
        MONITOR_ARM_SETTINGS_FILE.read_text(
            encoding="utf-8"
        )
    )

    # 현재 POCO 통합 기준값이 바뀌지 않았는지 먼저 확인한다.
    assert (
        monitor_arm_settings["control"]["command_hz"]
        == 20.0
    )
    assert monitor_arm_settings["control"]["trajectory_reference_hz"] == 5.0
    assert (
        monitor_arm_settings["control"]["pose_speed"]
        == 800
    )
    assert (
        monitor_arm_settings["control"]["pose_acc"]
        == 20
    )
    assert (
        monitor_arm_settings["control"]["working_start_arrival_tolerance_deg"]
        == 1.0
    )
    assert (
        monitor_arm_settings["control"]["working_start_stable_samples"]
        == 3
    )
    assert (
        monitor_arm_settings["control"]["working_start_timeout_sec"]
        == 25.0
    )
    assert (
        monitor_arm_settings["control"]["preparation_arrival_tolerance_deg"]
        == 1.0
    )
    assert (
        monitor_arm_settings["control"]["preparation_arrival_stable_samples"]
        == 3
    )
    assert (
        monitor_arm_settings["control"]["preparation_movement_timeout_sec"]
        == 25.0
    )

    # 0.25°보다 크지만 설정 허용치 안인 실제 정지 오차도
    # 연속 3개 샘플에서만 도착으로 확정해야 한다.
    arrival_planner = MonitorArmPlanner(monitor_arm_settings)
    arrival_planner.working_command = JointCommand(0.0, 0.0)
    arrival_planner.request_working_pose_recovery()
    near_target = JointCommand(0.8, -0.7)
    outside_target = JointCommand(1.2, -0.7)
    assert arrival_planner.plan(near_target, user_x_m=0.73) is None
    assert arrival_planner.recovery_active is True
    assert arrival_planner.recovery_stable_sample_count == 1
    assert arrival_planner.plan(outside_target, user_x_m=0.73) is not None
    assert arrival_planner.recovery_stable_sample_count == 0
    assert arrival_planner.plan(near_target, user_x_m=0.73) is None
    assert arrival_planner.recovery_active is True
    assert arrival_planner.plan(near_target, user_x_m=0.73) is None
    assert arrival_planner.recovery_active is True
    assert arrival_planner.plan(near_target, user_x_m=0.73) is None
    assert arrival_planner.recovery_active is False
    assert arrival_planner.recovery_stable_sample_count == 3

    # --------------------------------------------------------
    # ToF / Vision / User-X Fusion
    # --------------------------------------------------------
    # 앉은 사용자의 골반이 책상에 가려져도 실제 제어에 필요한 양쪽
    # 눈/어깨가 충분히 보이면 presence-valid여야 한다.
    pose_landmarks = [
        SimpleNamespace(visibility=0.95)
        for _ in range(33)
    ]
    pose_landmarks[23].visibility = 0.10
    pose_landmarks[24].visibility = 0.10
    pose_results = SimpleNamespace(
        pose_landmarks=SimpleNamespace(landmark=pose_landmarks)
    )
    landmark_quality, landmark_valid = pose_control_landmark_quality(pose_results)
    assert landmark_valid is True
    assert abs(landmark_quality - 0.95) < 1e-9

    # 반대로 실제 거리 제어에 필요한 한쪽 눈이 임계값 아래이면
    # 검출 자체가 있어도 안전 게이트에서는 미검출로 처리한다.
    pose_landmarks[2].visibility = 0.59
    landmark_quality, landmark_valid = pose_control_landmark_quality(pose_results)
    assert landmark_valid is False
    assert abs(landmark_quality - 0.59) < 1e-9

    # 실제 I2C 없이 fixed ToF로 Sensor -> user X 변환을 검증한다.
    tof_service = FixedToFSensorService(
        0.70
    )

    tof_source = ToFUserXSource(
        tof_service,
        sensor_origin_x_m=0.02,
        minimum_user_x_m=0.60,
        maximum_user_x_m=0.83,
    )

    assert tof_source.open()

    # sensor origin 0.02m + ToF 0.70m
    assert abs(
        tof_source.read_user_x_m()
        - 0.72
    ) < 1e-9

    # POCO PoseProcess와 같은 [x, y, z, visibility] landmark 형식.
    landmarks = [
        [0.0, 0.0, 0.0, 1.0]
        for _ in range(6)
    ]

    landmarks[2] = [
        0.40,
        0.45,
        0.0,
        1.0,
    ]

    landmarks[5] = [
        0.60,
        0.45,
        0.0,
        1.0,
    ]

    eye = measure_pose_eye_gap(
        landmarks,
        320,
        240,
    )

    assert eye is not None

    # 320px * (0.60 - 0.40)
    assert abs(
        eye.gap_px
        - 64.0
    ) < 1e-9

    estimator = (
        EyeGapVisionDistanceEstimator(
            minimum_eye_gap_px=5.0,
            minimum_distance_m=0.20,
            maximum_distance_m=1.20,
            filter_alpha=1.0,
        )
    )

    # 기준: 64px = 0.5m
    estimator.calibrate(
        64.0,
        0.50,
    )

    # 눈 간격이 80px로 커지면
    # 0.5 * 64 / 80 = 0.4m
    assert abs(
        estimator.estimate_distance_m(80.0)
        - 0.40
    ) < 1e-9

    fusion = UserXFusion(
        0.7,
        0.3,
    )

    # ToF 70% + Vision 30%
    assert abs(
        fusion.fuse(
            0.70,
            0.80,
        )
        - 0.73
    ) < 1e-9

    # Vision이 없으면 ToF 단독값.
    assert abs(
        fusion.fuse(
            0.70,
            None,
        )
        - 0.70
    ) < 1e-9

    tof_source.close()


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
    # Shared MotorService
    # Motor12 Auto Tracking / Rest / Recovery
    # Motor34 Direct
    # --------------------------------------------------------
    with tempfile.TemporaryDirectory() as td:
        servo_file = (Path(td) / "servo.json")

        # FakeController가 자체 FakeCalibration을 사용하므로
        # 실제 Calibration 내용은 필요 없고 파일 존재만 만족시킨다.
        servo_file.write_text(
            json.dumps({"servos": {}, }) , encoding="utf-8", )

        made = {}

        def controller_factory(calibration_file=None, ):
            controller = FakeController(calibration_file)

            made["controller"] = (controller)

            return controller

        motor = MotorService(
            calibration_file=servo_file,
            controller_factory=controller_factory,
        )

        assert motor.open()

        # ====================================================
        # Motor1/2
        # ====================================================
        motor12 = Motor12Controller(
            motor,
            settings=monitor_arm_settings,
            update_hz=1000,
        )

        # ====================================================
        # Motor3/4
        # ====================================================
        motor34 = Motor34Controller(motor)
        preparation = MonitorArmPreparationController(
            motor,
            motor12,
            motor34,
            settings_path=MONITOR_ARM_SETTINGS_FILE,
        )

        motor34.apply_control_config(
            {
                "command_hz": 1000,
                "auto_speed": 500,
                "auto_acc": 12,
                "motor3": {
                    "enabled": True,
                    "direction_sign": +1.0,
                },
                "motor4": {
                    "enabled": True,
                    "direction_sign": +1.0,
                },
            }
        )

        # Motor1~4 모두 같은 MotorService를 공유하면서
        # 각자의 Calibration / Servo ID / Ping / 범위를 검사한다.
        assert motor12.initialize()
        assert motor34.initialize()

        fake = made["controller"]

        # ====================================================
        # Motor1/2 정상 자동추종
        # ====================================================
        monitor_input = {
            "available": True,
            "valid": True,
            "tof_user_x_m": 0.80,
            "vision_user_x_m": None,
            "user_x_m": 0.80,
            "fusion_mode": "TOF_ONLY",
            "eye_gap_px": None,
            "last_error": None,
        }

        now = 10.0

        motor12_context = {
            "now": now,
            "motor12": {
                "control_active": True,
                "input": monitor_input,
            },
        }

        state12 = motor12.update(motor12_context)

        assert (state12["control_active"] is True)

        # shoulder + elbow는 하나의 SyncWrite.
        assert len(fake.sync_moves) == 1

        normal_move = (fake.sync_moves[-1])

        assert set(normal_move["targets"]) == {
            "shoulder_lift",
            "elbow_flex",
        }

        # 현재 설정의 adaptive 범위.
        assert (150 <= normal_move["speed"] <= 800)

        assert (normal_move["acc"] == 20)

        assert (normal_move["wait"] is False)

        # ====================================================
        # Motor12 command_hz = 20Hz independent trajectory
        # ====================================================
        initial_pose = motor12.planner.kinematics.forward(
            JointCommand(0.0, 0.0)
        )
        first_target = JointCommand(
            normal_move["targets"]["shoulder_lift"],
            normal_move["targets"]["elbow_flex"],
        )
        first_pose = motor12.planner.kinematics.forward(first_target)
        assert abs((first_pose.x_m - initial_pose.x_m) - 0.005) < 1e-6
        assert abs(state12["trajectory_max_x_step_m"] - 0.005) < 1e-9
        assert state12["trajectory_speed_error_scale"] == 4.0
        legacy_target = motor12.planner.plan(
            JointCommand(0.0, 0.0),
            user_x_m=0.80,
        )
        legacy_speed, _legacy_delta = motor12._select_tracking_speed(
            JointCommand(0.0, 0.0),
            legacy_target,
        )
        assert normal_move["speed"] == legacy_speed

        # 기존 5Hz 1주기(0.2초)의 2cm 이동을 20Hz 4개 목표로 분할한다.
        for offset in (0.051, 0.102, 0.153):
            motor12_context["now"] = now + offset
            motor12.update(motor12_context)

        assert len(fake.sync_moves) == 4
        fourth_target = JointCommand(
            fake.sync_moves[-1]["targets"]["shoulder_lift"],
            fake.sync_moves[-1]["targets"]["elbow_flex"],
        )
        fourth_pose = motor12.planner.kinematics.forward(fourth_target)
        assert abs((fourth_pose.x_m - initial_pose.x_m) - 0.020) < 1e-6

        # ====================================================
        # ToF/Fusion invalid -> SAFE_HOLD
        # ====================================================
        invalid_input = dict(monitor_input)

        invalid_input.update(
            {
                "valid": False,
                "user_x_m": None,
                "last_error": (
                    "ToF unavailable"
                ),
            }
        )

        motor12_context["now"] = (now + 0.30)

        motor12_context["motor12"]["input"] = invalid_input

        state12 = motor12.update(motor12_context)

        assert (state12["hold_reason"] == "SAFE_HOLD")

        # 잘못된 센서 입력으로 추가 이동하면 안 된다.
        assert len(fake.sync_moves) == 4

        # ====================================================
        # Rest 특수 자세
        # ====================================================
        motor12_context["motor12"]["input"] = monitor_input

        rest_result = (motor12.move_to_rest())

        assert (rest_result["accepted"] is True)

        assert len(fake.special_sync_moves) == 1

        rest_move = (fake.special_sync_moves[-1])

        assert (rest_move["targets"] == {
                "shoulder_lift": 107.75,
                "elbow_flex": -92.55,
            }
        )

        assert (rest_move["speed"] == 200)

        assert (rest_move["acc"] == 10)

        assert (rest_move["wait"] is False)

        # ====================================================
        # Rest latch
        # ====================================================
        regular_before_rest_hold = len(fake.sync_moves)

        special_before_rest_hold = len(fake.special_sync_moves)

        motor12_context["now"] = (now + 0.60)

        state12 = motor12.update(motor12_context)

        assert (state12["hold_reason"] == "REST")

        # ToF가 유효해도 Rest 중에는 추종하지 않는다.
        assert len(fake.sync_moves) == regular_before_rest_hold

        assert len(fake.special_sync_moves) == special_before_rest_hold

        # ====================================================
        # Resume -> inward-only Recovery
        # ====================================================
        resume_result = (motor12.resume_from_rest())

        assert (resume_result["accepted"] is True)

        # Timeout은 성공으로 처리하지 않고 Recovery 명령을 즉시 중단한다.
        motor12.planner.recovery_started_at = (
            time.monotonic()
            - motor12.planner.working_start_timeout_sec
            - 1.0
        )
        motor12_context["now"] = (now + 0.90)
        timeout_state = motor12.update(motor12_context)
        assert timeout_state["hold_reason"] == "RECOVERY_TIMEOUT"
        assert timeout_state["recovery_active"] is False
        assert "시간 초과" in timeout_state["last_error"]

        preparation.begin()
        preparation.connected = True
        preparation.recovery_active = True
        preparation.target = motor12.planner.working_command
        preparation.target_pose = preparation.kinematics.forward(
            preparation.target
        )
        preparation.target_reason = "working_start"
        preparation_state = preparation.update(now + 0.91)
        assert preparation_state["movement_active"] is False
        assert preparation_state["working_start_completed"] is False
        assert preparation_state["movement_status"] == "timeout"
        assert preparation_state["motor12_hold_reason"] == "RECOVERY_TIMEOUT"
        assert "시간 초과" in preparation_state["motor12_last_error"]
        assert preparation_state["current_target_max_error_deg"] > 1.0

        # 명시적으로 다시 요청한 뒤에만 정상 Recovery를 재개한다.
        resume_result = motor12.resume_from_rest()
        assert resume_result["accepted"] is True

        regular_before_recovery = len(fake.sync_moves)

        special_before_recovery = len(fake.special_sync_moves)

        recovery_complete = False
        stabilizing_command_counts = []
        recovery_now = (now + 0.90)

        # Fake에서는 각 명령을 받으면 즉시 목표에 도달한다고
        # 가정하므로 최대 60 command 안에서 작업자세로 돌아와야 한다.
        for _ in range(60):
            motor12_context["now"] = recovery_now

            state12 = motor12.update(motor12_context)

            recovery_now += 0.25

            if state12["hold_reason"] == "RECOVERY_STABILIZING":
                stabilizing_command_counts.append(
                    len(fake.sync_moves) + len(fake.special_sync_moves)
                )

            if (state12["hold_reason"] == "RECOVERY_COMPLETE"):
                recovery_complete = True
                break

        assert recovery_complete
        assert len(stabilizing_command_counts) == 2
        assert len(set(stabilizing_command_counts)) == 1

        # Calibration 밖에서는 특수 SyncWrite가 실제 사용돼야 한다.
        assert len(fake.special_sync_moves) > special_before_recovery

        # Calibration 안으로 들어온 뒤에는 일반 SyncWrite로 전환돼야 한다.
        assert len(fake.sync_moves) > regular_before_recovery

        assert (state12["recovery_active"] is False)
        assert state12["recovery_stable_samples"] == 3
        assert state12["recovery_required_stable_samples"] == 3
        assert state12["recovery_arrival_tolerance_deg"] == 1.0

        assert abs(fake.angles["shoulder_lift"]) <= 0.25

        assert abs(fake.angles["elbow_flex"]) <= 0.25

        # ====================================================
        # Motor3/4 기존 Direct IMU 테스트
        # ====================================================
        # Motor12 테스트가 남긴 read/write trace만 제거한다.
        # 각도 상태 자체는 그대로 유지한다.
        fake.reset_trace()

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

        # Closed loop 첫 진입은 M3/M4 현재각을 각각 1회 읽는다.
        motor34.update(context)

        assert (fake.read_count == 2)

        assert len(fake.moves) == 2

        assert (fake.moves[0][0] == "wrist_flex")

        assert (fake.moves[1][0] == "wrist_roll")

        assert (fake.moves[0][2] == 500)

        assert (fake.moves[0][3] == 12)

        # ----------------------------------------------------
        # IMU Y PID -> Motor3
        # ----------------------------------------------------
        time.sleep(0.002)

        moving_y = dict(zero_imu)

        moving_y["imu_y_error_g"] = 0.05

        moving_y["motor3_correction_speed_deg_s"] = -6.0

        context["now"] = (time.monotonic())

        context["imu"] = (moving_y)

        state = motor34.update(context)

        assert len(fake.moves) == 4

        m3_move = (fake.moves[-2])

        m4_move = (fake.moves[-1])

        assert (m3_move[0] == "wrist_flex")

        assert (m4_move[0] == "wrist_roll")

        assert (m3_move[1] < 0.0)

        assert abs(m4_move[1]) < 1e-9

        assert (state["motor3"]["axis"] == "imu_y")

        assert (state["motor4"]["axis"] == "imu_x")

        # ----------------------------------------------------
        # IMU X PID -> Motor4
        # ----------------------------------------------------
        time.sleep(0.002)

        moving_x = dict(zero_imu)

        moving_x["imu_x_error_g"] = 0.05

        moving_x["motor4_correction_speed_deg_s"] = -6.0

        context["now"] = (time.monotonic())

        context["imu"] = (moving_x)

        state = motor34.update(context)

        assert len(fake.moves) == 6

        m4_move = (fake.moves[-1])

        assert (m4_move[0] == "wrist_roll")

        assert (m4_move[1] < 0.0)

        assert (state["axis_mapping"] == {
                "motor3": "imu_y",
                "motor4": "imu_x",
            }
        )

        # ====================================================
        # Smooth preparation arrival / timeout regression
        # ====================================================
        # Smooth working/manual IK is sent as one synchronized goal, so it does
        # not use Planner recovery counters. A realistic 0.8° stop error must
        # complete after three telemetry samples (the old 0.5° logic stuck).
        preparation.active = True
        preparation.connected = True
        preparation.target = JointCommand(0.0, 0.0)
        preparation.target_reason = "working_start"
        preparation.recovery_active = False
        preparation.working_start_completed = False
        preparation.movement_started_at = time.monotonic()
        preparation.arrival_stable_sample_count = 0
        preparation.movement_status = "moving"
        fake.angles["shoulder_lift"] = 0.8
        fake.angles["elbow_flex"] = -0.7
        for _ in range(3):
            preparation.refresh_telemetry(force=True)
        preparation_state = preparation.snapshot()
        assert preparation_state["working_start_completed"] is True
        assert preparation_state["movement_active"] is False
        assert preparation_state["movement_status"] == "completed"
        assert preparation_state["stable_samples"] == 3

        # An unreachable goal must stop with a visible timeout instead of
        # leaving the UI in an endless "moving" state.
        preparation.target = JointCommand(10.0, 10.0)
        preparation.target_reason = "manual_ik"
        preparation.movement_started_at = (
            time.monotonic() - preparation.movement_timeout_sec - 1.0
        )
        preparation.arrival_stable_sample_count = 0
        preparation.movement_status = "moving"
        preparation.refresh_telemetry(force=True)
        preparation_state = preparation.snapshot()
        assert preparation_state["movement_active"] is False
        assert preparation_state["movement_status"] == "timeout"
        assert "도착하지 못했습니다" in preparation_state["last_error"]

        # ====================================================
        # Saved profile -> reconnect/Ping -> Motor1~4 initial pose
        # ====================================================
        # 프로필 선택 경로는 기존 ready 플래그만 믿지 않고 네 축을 다시
        # 초기화한 뒤, 저장된 작업각을 한 번에 명령하고 실제각 도달을 확인한다.
        preparation.end()
        preparation.begin()
        connected_state = preparation.connect_all()
        assert connected_state["all_motors_ready"] is True

        profile_targets = {
            "shoulder_lift": 8.0,
            "elbow_flex": -7.0,
            "wrist_flex": 6.0,
            "wrist_roll": -5.0,
        }
        result12 = motor12.move_to_working_smooth(
            JointCommand(
                profile_targets["shoulder_lift"],
                profile_targets["elbow_flex"],
            )
        )
        result34 = motor34.move_to_neutral(profile_targets)
        assert result12["accepted"] is True
        assert result34["accepted"] is True

        actual, errors, reached, max_error = _read_joint_arrival(
            motor,
            profile_targets,
            tolerance_deg=1.0,
        )
        assert reached is True
        assert actual == profile_targets
        assert all(error == 0.0 for error in errors.values())
        assert max_error == 0.0

        fake.angles["wrist_roll"] = profile_targets["wrist_roll"] + 1.1
        _actual, _errors, reached, max_error = _read_joint_arrival(
            motor,
            profile_targets,
            tolerance_deg=1.0,
        )
        assert reached is False
        assert abs(max_error - 1.1) < 1e-9
        preparation.end()

        motor.close()

    print(
    "Hardware logic self-test: PASS "
    "(IMU / ToF-Vision / Motor1~4 / Rest-Recovery / Profile-Auto-Ready)"
)


if __name__ == "__main__":
    main()

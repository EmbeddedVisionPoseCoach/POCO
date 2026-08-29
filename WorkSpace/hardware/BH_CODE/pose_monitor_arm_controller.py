#!/usr/bin/env python3
"""ToF user-X + MediaPipe Pose based two-motor monitor-arm controller.

Runtime flow
------------
1. Pose Landmarker processes camera frames for the future posture gate.
2. A ToF source supplies the user's absolute +X coordinate from the base.
3. Monitor X is user X minus the configured user-monitor distance.
4. A two-joint IK target is calculated for shoulder_lift and elbow_flex only.
5. Calibration hard limits, editable soft limits, and the complete interpolated
   vertical path are checked before a command is sent. Normal tracking can send
   the final IK target directly; the legacy maximum-step mode remains selectable.
6. In motor mode, only the resulting two angles cross into a motor process;
   that child process exclusively owns serial reads and writes.

The script starts in simulation mode.  Pass --enable-motor only after manual
limit testing and after max_speed has been calibrated for servos 1 and 2.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

from monitor_arm_kinematics import (
    ArmGeometry,
    JointCommand,
    KinematicsError,
    MotionSafetyError,
    SafetyLimits,
    TwoJointMonitorArm,
    load_settings,
    monitor_target_from_user,
)
from monitor_arm_motor_process import MotorControlProcessClient


ROOT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ROOT_DIR / "tasks" / "pose_landmarker_heavy.task"
DEFAULT_CALIBRATION_PATH = ROOT_DIR / "servo_calibration_result.json"
LEFT_EYE_INDEX = 2
RIGHT_EYE_INDEX = 5


@dataclass(frozen=True)
class EyeMeasurement:
    gap_px: float
    left_xy: tuple[int, int]
    right_xy: tuple[int, int]


class FixedToFUserXSource:
    """Temporary ToF stand-in returning user X from a fixed sensor range.

    A real ToF adapter only needs to replace ``read_range_m()``. The coordinate
    conversion and range validation can remain unchanged.
    """

    def __init__(
        self,
        sensor_origin_x_m: float,
        fixed_range_m: float,
        minimum_user_x_m: float,
        maximum_user_x_m: float,
    ):
        self.sensor_origin_x_m = float(sensor_origin_x_m)
        self.fixed_range_m = float(fixed_range_m)
        self.minimum_user_x_m = float(minimum_user_x_m)
        self.maximum_user_x_m = float(maximum_user_x_m)
        if self.minimum_user_x_m > self.maximum_user_x_m:
            raise ValueError("ToF 사용자 X 최소값이 최대값보다 큽니다.")

    def read_range_m(self) -> float:
        """Return fixed data until the real ToF driver is connected."""
        return self.fixed_range_m

    def read_user_x_m(self) -> float:
        range_m = float(self.read_range_m())
        user_x_m = self.sensor_origin_x_m + range_m
        if not math.isfinite(user_x_m):
            raise ValueError("ToF 사용자 X가 유한한 값이 아닙니다.")
        if not self.minimum_user_x_m <= user_x_m <= self.maximum_user_x_m:
            raise ValueError(
                f"ToF 사용자 X {user_x_m:.3f}m가 허용범위 "
                f"{self.minimum_user_x_m:.3f}~{self.maximum_user_x_m:.3f}m 밖입니다."
            )
        return user_x_m


class MonitorArmPlanner:
    def __init__(self, settings: dict):
        self.settings = settings
        self.kinematics = TwoJointMonitorArm(ArmGeometry.from_settings(settings))
        self.limits = SafetyLimits.from_settings(settings)
        distance = settings["distance"]
        self.desired_distance_m = float(distance["desired_user_monitor_distance_m"])
        self.deadband_m = float(distance["deadband_m"])
        self.max_x_step_m = float(distance["max_monitor_x_step_m"])
        control = settings.get("control", {})
        self.joint_command_mode = str(
            control.get("pose_joint_command_mode", "direct")
        ).strip().lower()
        if self.joint_command_mode not in {"direct", "stepped"}:
            raise ValueError(
                "control.pose_joint_command_mode는 direct 또는 stepped여야 합니다."
            )
        self.reference_z_m: float | None = None
        cartesian = settings.get("manual_cartesian", {})
        self.working_z_min_m = float(cartesian.get("monitor_z_min_m", 0.20))
        self.working_z_max_m = float(cartesian.get("monitor_z_max_m", 0.30))
        self.default_working_z_m = float(
            cartesian.get("default_monitor_z_m", 0.2560722511328793)
        )
        postures = settings.get("postures", {})
        working = postures.get("working", {})
        self.working_command = JointCommand(
            float(working.get("shoulder_lift_deg", 0.0)),
            float(working.get("elbow_flex_deg", 0.0)),
        )
        self.recovery_active = False

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(value, maximum))

    @staticmethod
    def _outside_distance(value: float, minimum: float, maximum: float) -> float:
        if value < minimum:
            return minimum - value
        if value > maximum:
            return value - maximum
        return 0.0

    def set_vertical_reference(self, current: JointCommand) -> float:
        current_z_m = self.kinematics.forward(current).z_m
        if self.working_z_min_m <= current_z_m <= self.working_z_max_m:
            self.reference_z_m = current_z_m
            self.recovery_active = False
        else:
            self.reference_z_m = self.default_working_z_m
            self.recovery_active = True
        return self.reference_z_m

    def request_working_pose_recovery(self) -> None:
        """Latch recovery until the configured working posture is reached."""
        self.reference_z_m = self.default_working_z_m
        self.recovery_active = True

    def _effective_joint_ranges(
        self,
        calibration_ranges: dict[str, tuple[float, float]] | None,
    ) -> dict[str, tuple[float, float]]:
        ranges = {
            "shoulder_lift": (
                self.limits.shoulder_min_deg,
                self.limits.shoulder_max_deg,
            ),
            "elbow_flex": (
                self.limits.elbow_min_deg,
                self.limits.elbow_max_deg,
            ),
        }
        if calibration_ranges is not None:
            for joint, (soft_min, soft_max) in tuple(ranges.items()):
                hard_min, hard_max = calibration_ranges[joint]
                ranges[joint] = (
                    max(soft_min, float(hard_min)),
                    min(soft_max, float(hard_max)),
                )
        return ranges

    def _validate_recovery_step(
        self,
        current: JointCommand,
        target: JointCommand,
        calibration_ranges: dict[str, tuple[float, float]] | None,
    ) -> None:
        ranges = self._effective_joint_ranges(calibration_ranges)
        current_angles = {
            "shoulder_lift": current.shoulder_lift_deg,
            "elbow_flex": current.elbow_flex_deg,
        }
        target_angles = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        for joint, (minimum, maximum) in ranges.items():
            current_outside = self._outside_distance(
                current_angles[joint], minimum, maximum
            )
            target_outside = self._outside_distance(
                target_angles[joint], minimum, maximum
            )
            if current_outside <= 1e-6 and target_outside > 1e-6:
                raise MotionSafetyError(
                    f"복구 스텝 {joint}={target_angles[joint]:+.2f}°가 "
                    f"안전범위 {minimum:+.2f}~{maximum:+.2f}° 밖입니다."
                )
            if current_outside > 1e-6 and target_outside >= current_outside - 1e-6:
                raise MotionSafetyError(
                    f"복구 스텝이 {joint} 안전범위에서 더 멀어집니다."
                )

        current_pose = self.kinematics.forward(current)
        target_pose = self.kinematics.forward(target)
        current_z_outside = self._outside_distance(
            current_pose.z_m,
            self.working_z_min_m,
            self.working_z_max_m,
        )
        for index in range(self.limits.path_samples + 1):
            ratio = index / self.limits.path_samples
            pose = self.kinematics.forward(current.interpolate(target, ratio))
            sample_z_outside = self._outside_distance(
                pose.z_m,
                self.working_z_min_m,
                self.working_z_max_m,
            )
            if current_z_outside <= 1e-4 and sample_z_outside > 1e-4:
                raise MotionSafetyError("복구 중 안전 Z 범위 밖으로 나가는 경로입니다.")
            if (
                current_z_outside > 1e-4
                and sample_z_outside
                > current_z_outside + self.limits.vertical_tolerance_m
            ):
                raise MotionSafetyError(
                    "복구 중 Z가 현재 이탈량보다 vertical_tolerance 이상 "
                    "더 벗어납니다."
                )
            expected_z_m = current_pose.z_m + (
                target_pose.z_m - current_pose.z_m
            ) * ratio
            if abs(pose.z_m - expected_z_m) > self.limits.vertical_tolerance_m:
                raise MotionSafetyError("복구 중 예상 Z 경로 편차가 너무 큽니다.")

    def _plan_working_pose_recovery(
        self,
        current: JointCommand,
        calibration_ranges: dict[str, tuple[float, float]] | None,
    ) -> JointCommand | None:
        largest_joint_change = max(
            abs(self.working_command.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(self.working_command.elbow_flex_deg - current.elbow_flex_deg),
        )
        if largest_joint_change <= 0.25:
            self.recovery_active = False
            return None
        ratio = min(1.0, self.limits.max_joint_step_deg / largest_joint_change)
        target = current.interpolate(self.working_command, ratio)
        self._validate_recovery_step(current, target, calibration_ranges)
        return target

    def plan(
        self,
        current: JointCommand,
        user_x_m: float,
        calibration_ranges: dict[str, tuple[float, float]] | None = None,
    ) -> JointCommand | None:
        if self.reference_z_m is None:
            self.set_vertical_reference(current)

        if self.recovery_active:
            return self._plan_working_pose_recovery(current, calibration_ranges)

        current_pose = self.kinematics.forward(current)
        requested_pose = monitor_target_from_user(
            user_x_m=float(user_x_m),
            user_monitor_distance_m=self.desired_distance_m,
            monitor_z_m=self.reference_z_m,
        )
        monitor_x_error = requested_pose.x_m - current_pose.x_m
        if abs(monitor_x_error) <= self.deadband_m:
            return None

        x_step = self._clamp(
            monitor_x_error,
            -self.max_x_step_m,
            self.max_x_step_m,
        )
        full_target = self.kinematics.inverse(
            current_pose.x_m + x_step,
            self.reference_z_m,
        )

        target = full_target
        if self.joint_command_mode == "stepped":
            largest_joint_change = max(
                abs(full_target.shoulder_lift_deg - current.shoulder_lift_deg),
                abs(full_target.elbow_flex_deg - current.elbow_flex_deg),
            )
            if largest_joint_change > self.limits.max_joint_step_deg:
                ratio = self.limits.max_joint_step_deg / largest_joint_change
                target = current.interpolate(full_target, ratio)

        self.kinematics.validate_motion(
            current=current,
            target=target,
            reference_z_m=self.reference_z_m,
            limits=self.limits,
            calibration_ranges=calibration_ranges,
            enforce_step_limit=self.joint_command_mode == "stepped",
        )
        return target


def measure_pose_eye_gap(landmarks, width: int, height: int) -> EyeMeasurement | None:
    if landmarks is None or len(landmarks) <= RIGHT_EYE_INDEX:
        return None
    left = landmarks[LEFT_EYE_INDEX]
    right = landmarks[RIGHT_EYE_INDEX]

    for landmark in (left, right):
        visibility = getattr(landmark, "visibility", 1.0)
        presence = getattr(landmark, "presence", 1.0)
        if visibility < 0.5 or presence < 0.5:
            return None

    left_xy = (round(left.x * width), round(left.y * height))
    right_xy = (round(right.x * width), round(right.y * height))
    gap_px = math.hypot(left_xy[0] - right_xy[0], left_xy[1] - right_xy[1])
    return EyeMeasurement(gap_px=gap_px, left_xy=left_xy, right_xy=right_xy)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="OpenCV webcam index")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--settings", type=Path, default=ROOT_DIR / "monitor_arm_settings.json")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument(
        "--enable-motor",
        action="store_true",
        help="Actually open /dev/ttyACM0 and command servos 1 and 2",
    )
    parser.add_argument(
        "--tof-user-x-m",
        type=float,
        help="Override the fixed ToF stub user X coordinate for testing",
    )
    parser.add_argument(
        "--no-ik-visualizer",
        action="store_true",
        help="Disable the live Tkinter X-Z IK window",
    )
    parser.add_argument(
        "--allow-uncalibrated-speed",
        action="store_true",
        help=(
            "Allow motor testing while servo calibration max_speed is null. "
            "The configured absolute cap of 1000 still applies."
        ),
    )
    return parser.parse_args()


def run() -> None:
    args = parse_args()
    settings = load_settings(args.settings)
    if not args.model.exists():
        raise FileNotFoundError(f"Pose task 파일이 없습니다: {args.model}")

    try:
        import cv2
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
    except ImportError as error:
        raise RuntimeError(
            "opencv-python과 mediapipe를 설치한 Python 환경에서 실행하세요."
        ) from error

    distance_cfg = settings["distance"]
    tof_cfg = settings["tof"]
    if str(tof_cfg.get("mode", "fixed_stub")) != "fixed_stub":
        raise ValueError("현재 구현된 ToF mode는 fixed_stub뿐입니다.")
    sensor_origin_x_m = float(tof_cfg.get("sensor_origin_x_m", 0.0))
    fixed_range_m = float(tof_cfg["fixed_range_m"])
    if args.tof_user_x_m is not None:
        fixed_range_m = float(args.tof_user_x_m) - sensor_origin_x_m
    tof_source = FixedToFUserXSource(
        sensor_origin_x_m=sensor_origin_x_m,
        fixed_range_m=fixed_range_m,
        minimum_user_x_m=float(tof_cfg["minimum_user_x_m"]),
        maximum_user_x_m=float(tof_cfg["maximum_user_x_m"]),
    )
    initial_user_x_m = tof_source.read_user_x_m()
    posture_cfg = settings.get("postures", {})
    rest_cfg = posture_cfg.get("rest", {})
    rest_command = JointCommand(
        shoulder_lift_deg=float(rest_cfg.get("shoulder_lift_deg", 107.75)),
        elbow_flex_deg=float(rest_cfg.get("elbow_flex_deg", -92.55)),
    )
    rest_speed_cap = int(rest_cfg.get("speed_cap", 200))
    rest_acc_cap = int(rest_cfg.get("acc_cap", 10))
    planner = MonitorArmPlanner(settings)

    motor_process: MotorControlProcessClient | None = None
    calibration_ranges = None
    if args.enable_motor:
        control = settings["control"]
        motor_process = MotorControlProcessClient(
            calibration_path=args.calibration,
            speed=control.get(
                "pose_speed", control.get("vertical_ik_speed", control["speed"])
            ),
            acc=control.get(
                "pose_acc", control.get("vertical_ik_acc", control["acc"])
            ),
            speed_mode=control.get(
                "pose_speed_mode", control.get("vertical_ik_speed_mode", "fixed")
            ),
            minimum_speed=control.get(
                "pose_variable_min_speed",
                control.get("vertical_ik_variable_min_speed", 1),
            ),
            full_speed_error_deg=control.get(
                "pose_variable_full_speed_error_deg",
                min(
                    float(control.get("vertical_ik_variable_full_speed_error_deg", 30.0)),
                    float(settings["safety"]["max_joint_step_deg"]),
                ),
            ),
            allow_uncalibrated_speed=args.allow_uncalibrated_speed,
        )
        motor_process.open()
        if motor_process.initial_angles is None:
            motor_process.close()
            raise RuntimeError("모터 프로세스에서 초기 관절각을 받지 못했습니다.")
        current = motor_process.initial_angles
        calibration_ranges = motor_process.calibration_ranges
        speed_mode_text = str(
            control.get(
                "pose_speed_mode", control.get("vertical_ik_speed_mode", "fixed")
            )
        ).upper()
        mode_text = f"MOTOR 1+2 ENABLED / {speed_mode_text} SPEED"
    else:
        current = JointCommand(0.0, 0.0)
        mode_text = "FIXED TOF SIMULATION"

    reference_z = planner.set_vertical_reference(current)
    command_mode_text = (
        "DIRECT IK TARGET"
        if planner.joint_command_mode == "direct"
        else f"STEPPED IK ({planner.limits.max_joint_step_deg:g} DEG)"
    )
    mode_text = f"{mode_text} / {command_mode_text}"
    print(f"[{mode_text}] vertical reference={reference_z:.3f}m")
    print(
        f"고정 ToF 사용자 X={initial_user_x_m:.3f}m, "
        f"유지 거리={planner.desired_distance_m:.3f}m. "
        "q=종료, h=휴식, a=자동제어 재개, "
        "IK 창 닫기=종료"
    )

    visualizer = None
    if not args.no_ik_visualizer:
        try:
            from monitor_arm_visualizer import PoseIKVisualizer

            visualizer = PoseIKVisualizer(planner.kinematics)
        except Exception as error:
            print(f"[IK VISUALIZER DISABLED] Tkinter 창 생성 실패: {error}")

    options = vision.PoseLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=str(args.model)),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened():
        if visualizer is not None:
            visualizer.close()
        if motor_process is not None:
            motor_process.close()
        raise RuntimeError(f"웹캠 index {args.camera}를 열 수 없습니다.")

    command_interval = 1.0 / max(float(settings["control"]["command_hz"]), 1.0)
    last_command_at = 0.0
    last_timestamp_ms = -1
    status = "고정 ToF 입력 기반 제어 시작 대기"
    visual_current = current
    visual_target: JointCommand | None = None
    auto_control_enabled = True
    rest_confirmation_deadline = 0.0

    try:
        with vision.PoseLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = capture.read()
                if not ok or frame is None:
                    status = "카메라 프레임 읽기 실패"
                    if visualizer is not None and not visualizer.update_state(
                        current=visual_current,
                        target=visual_target,
                        user_x_m=initial_user_x_m,
                        desired_distance_m=planner.desired_distance_m,
                        reference_z_m=reference_z,
                        status=status,
                        mode_text=mode_text,
                    ):
                        break
                    continue

                frame = cv2.flip(frame, 1)
                height, width = frame.shape[:2]
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                timestamp_ms = max(int(time.monotonic() * 1000), last_timestamp_ms + 1)
                last_timestamp_ms = timestamp_ms
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                landmarks = result.pose_landmarks[0] if result.pose_landmarks else None
                eye = measure_pose_eye_gap(landmarks, width, height)
                if eye is not None:
                    cv2.circle(frame, eye.left_xy, 5, (0, 255, 255), -1)
                    cv2.circle(frame, eye.right_xy, 5, (0, 255, 255), -1)
                    cv2.line(frame, eye.left_xy, eye.right_xy, (0, 255, 255), 2)

                try:
                    user_x_m = tof_source.read_user_x_m()
                except ValueError as error:
                    user_x_m = None
                    status = f"SAFE HOLD: {error}"

                now = time.monotonic()
                if (
                    auto_control_enabled
                    and user_x_m is not None
                    and now - last_command_at >= command_interval
                ):
                    last_command_at = now
                    try:
                        if motor_process is not None:
                            current = motor_process.read_angles()
                        command_start = current
                        target = planner.plan(
                            current,
                            user_x_m,
                            calibration_ranges=calibration_ranges,
                        )
                        if target is None:
                            visual_current = current
                            visual_target = None
                            status = "ToF 목표 X deadband - 현재 자세 유지"
                        else:
                            target_pose = planner.kinematics.forward(target)
                            move_result = None
                            if motor_process is not None:
                                move_result = motor_process.move(target)
                            visual_current = command_start
                            visual_target = target
                            current = target
                            speed_text = (
                                ""
                                if move_result is None
                                else f" Speed={int(move_result['speed'])}"
                            )
                            recovery_text = (
                                " RECOVERY"
                                if planner.recovery_active
                                or (
                                    move_result is not None
                                    and bool(move_result.get("recovering"))
                                )
                                else ""
                            )
                            status = (
                                f"target S={target.shoulder_lift_deg:+.1f} "
                                f"E={target.elbow_flex_deg:+.1f} "
                                f"X={target_pose.x_m:.3f} Z={target_pose.z_m:.3f}m"
                                f"{speed_text}{recovery_text}"
                            )
                    except (
                        KinematicsError,
                        MotionSafetyError,
                        RuntimeError,
                        ValueError,
                    ) as error:
                        visual_current = current
                        visual_target = None
                        status = f"SAFE HOLD: {error}"

                if user_x_m is None:
                    tof_text = "tof_user_x=--  user-monitor=--"
                else:
                    current_pose = planner.kinematics.forward(current)
                    user_monitor_distance_m = user_x_m - current_pose.x_m
                    tof_text = (
                        f"tof_user_x={user_x_m * 100:.1f}cm  "
                        f"user-monitor={user_monitor_distance_m * 100:.1f}cm"
                    )
                pose_text = "pose=--" if landmarks is None else "pose=OK"
                cv2.putText(
                    frame,
                    f"{mode_text}  {pose_text}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    tof_text,
                    (12, 54),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (0, 220, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    status[:110],
                    (12, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (255, 255, 255),
                    1,
                )

                if visualizer is not None and not visualizer.update_state(
                    current=visual_current,
                    target=visual_target,
                    user_x_m=user_x_m,
                    desired_distance_m=planner.desired_distance_m,
                    reference_z_m=reference_z,
                    status=status,
                    mode_text=mode_text,
                ):
                    break
                rest_requested = (
                    visualizer is not None and visualizer.consume_rest_request()
                )
                cv2.imshow("POCO two-motor pose monitor arm", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("h"):
                    if time.monotonic() <= rest_confirmation_deadline:
                        rest_requested = True
                        rest_confirmation_deadline = 0.0
                    else:
                        rest_confirmation_deadline = time.monotonic() + 5.0
                        status = (
                            "휴식자세 확인: 팔/모니터를 지지한 뒤 5초 안에 "
                            "h를 한 번 더 누르세요."
                        )
                if rest_requested:
                    auto_control_enabled = False
                    planner.request_working_pose_recovery()
                    if motor_process is not None:
                        rest_result = motor_process.move_rest(
                            rest_command,
                            rest_speed_cap=rest_speed_cap,
                            rest_acc_cap=rest_acc_cap,
                        )
                        status = (
                            "휴식자세 이동/자동제어 일시정지 — "
                            f"Speed={int(rest_result['speed'])}, "
                            f"Acc={int(rest_result['acc'])}; a=자동제어 재개"
                        )
                    else:
                        current = rest_command
                        status = "가상 휴식자세/자동제어 일시정지 — a=자동제어 재개"
                    visual_current = current
                    visual_target = rest_command
                if key == ord("a"):
                    if motor_process is not None:
                        current = motor_process.read_angles()
                    planner.request_working_pose_recovery()
                    auto_control_enabled = True
                    status = "자동제어 재개 — 작업자세 복귀 후 ToF X 추종"
    finally:
        capture.release()
        cv2.destroyAllWindows()
        if visualizer is not None:
            visualizer.close()
        if motor_process is not None:
            motor_process.close()


if __name__ == "__main__":
    run()

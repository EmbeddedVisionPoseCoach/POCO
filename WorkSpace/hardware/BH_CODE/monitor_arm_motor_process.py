#!/usr/bin/env python3
"""Servo 1/2 motor-process boundary for the pose monitor-arm controller.

The main process sends only calculated joint angles.  This module owns the
serial port, calibration conversion, servo reads and servo writes in a child
process so the same message boundary can later be moved into POCO hardware.
"""

from __future__ import annotations

import json
import multiprocessing as mp
import queue
import sys
from pathlib import Path
from typing import Any

PYQT_DIR = Path(__file__).resolve().parents[2] / "pyQt"
if str(PYQT_DIR) not in sys.path:
    sys.path.insert(0, str(PYQT_DIR))

from services.monitor_arm_kinematics import JointCommand
from services.monitor_arm_speed import (
    ABSOLUTE_SPEED_CAP,
    FIXED_SPEED_MODE,
    select_speed,
    validate_speed_profile,
)


class TwoMotorHardware:
    """Hardware adapter whose public movement API contains only motors 1 and 2."""

    def __init__(
        self,
        calibration_path: Path,
        speed: int,
        acc: int,
        speed_mode: str = FIXED_SPEED_MODE,
        minimum_speed: int = 1,
        full_speed_error_deg: float = 30.0,
        allow_uncalibrated_speed: bool = False,
    ):
        self.calibration_path = Path(calibration_path)
        self.maximum_speed = int(speed)
        self.acc = int(acc)
        self.speed_mode = str(speed_mode)
        self.minimum_speed = int(minimum_speed)
        self.full_speed_error_deg = float(full_speed_error_deg)
        self.allow_uncalibrated_speed = bool(allow_uncalibrated_speed)
        self.arm = None
        self.calibration_ranges: dict[str, tuple[float, float]] = {}
        validate_speed_profile(
            self.speed_mode,
            self.maximum_speed,
            self.minimum_speed,
            self.full_speed_error_deg,
        )
        if not 0 <= self.acc <= 30:
            raise ValueError("통합 테스트 Acc 허용범위는 0~30입니다.")

    def open(self) -> None:
        with self.calibration_path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        for servo_id, expected_joint in (
            ("1", "shoulder_lift"),
            ("2", "elbow_flex"),
        ):
            servo = data.get("servos", {}).get(servo_id, {})
            if servo.get("joint") != expected_joint:
                raise RuntimeError(
                    f"Servo {servo_id} 매핑 불일치: expected={expected_joint}, "
                    f"actual={servo.get('joint')}"
                )
            max_speed = servo.get("max_speed")
            if max_speed is None:
                if not self.allow_uncalibrated_speed:
                    raise RuntimeError(
                        f"Servo {servo_id}({expected_joint}) max_speed가 null입니다. "
                        "수동 시험 후 calibration JSON에 안전 속도를 기록하거나, "
                        "위험을 이해한 테스트에서만 --allow-uncalibrated-speed를 "
                        "추가하세요."
                    )
            elif self.maximum_speed > int(max_speed):
                raise RuntimeError(
                    f"요청 최대 speed={self.maximum_speed}, "
                    f"Servo {servo_id} max_speed={max_speed}"
                )

        vendored_site_packages = (
            Path(__file__).resolve().parent
            / "STServo_Python"
            / "stservo-env"
            / "Lib"
            / "site-packages"
        )
        if (
            vendored_site_packages.is_dir()
            and str(vendored_site_packages) not in sys.path
        ):
            sys.path.append(str(vendored_site_packages))

        from motor_control import MotorController

        self.arm = MotorController(calibration_file=str(self.calibration_path))
        if self.allow_uncalibrated_speed:
            # Explicit test-only override.  Mutate the child process's in-memory
            # calibration so MotorController's normal validation still enforces
            # our configured maximum; the JSON file is never modified here.
            for joint in ("shoulder_lift", "elbow_flex"):
                servo = self.arm.calibration.get_joint(joint)
                if servo.get("max_speed") is None:
                    servo["max_speed"] = self.maximum_speed
        self.calibration_ranges = {
            "shoulder_lift": self.arm.calibration.get_safe_angle_range(
                "shoulder_lift"
            ),
            "elbow_flex": self.arm.calibration.get_safe_angle_range("elbow_flex"),
        }

    def read_angles(self) -> JointCommand:
        if self.arm is None:
            raise RuntimeError("Motor port가 열리지 않았습니다.")
        shoulder = self.arm.get_joint_angle("shoulder_lift")
        elbow = self.arm.get_joint_angle("elbow_flex")
        if shoulder is None or elbow is None:
            raise RuntimeError("Servo 1·2 현재 각도를 읽지 못했습니다.")
        return JointCommand(float(shoulder), float(elbow))

    @staticmethod
    def _outside_distance(value: float, minimum: float, maximum: float) -> float:
        if value < minimum:
            return minimum - value
        if value > maximum:
            return value - maximum
        return 0.0

    def _unchecked_position(self, joint: str, angle_deg: float) -> int:
        """Convert only a confirmed rest/recovery angle, enforcing raw 0..4095."""
        servo = self.arm.calibration.require_position_calibrated(joint)
        # shoulder_lift and elbow_flex both use TEAM angle = -URDF angle.
        urdf_angle_deg = -float(angle_deg)
        position = int(
            round(
                int(servo["zero_position"])
                + int(servo["direction"])
                * urdf_angle_deg
                * (4096.0 / 360.0)
            )
        )
        if not 0 <= position <= 4095:
            raise RuntimeError(
                f"{joint} 예외 자세 Position={position}이 STS 절대범위 밖입니다."
            )
        return position

    def _move_recovery_aware(
        self,
        current: JointCommand,
        target: JointCommand,
        speed: int,
    ) -> bool:
        """Return True when a calibration-outside inward recovery was used."""
        current_angles = {
            "shoulder_lift": current.shoulder_lift_deg,
            "elbow_flex": current.elbow_flex_deg,
        }
        target_angles = {
            "shoulder_lift": target.shoulder_lift_deg,
            "elbow_flex": target.elbow_flex_deg,
        }
        needs_recovery = any(
            self._outside_distance(target_angles[joint], *self.calibration_ranges[joint])
            > 1e-6
            for joint in target_angles
        )
        if not needs_recovery:
            success = self.arm.move_joints(
                target_angles,
                speed=speed,
                acc=self.acc,
                wait=False,
            )
            if not success:
                raise RuntimeError("Servo 1·2 동기 이동 명령이 거부되었습니다.")
            return False

        commands = {}
        for joint, target_angle in target_angles.items():
            minimum, maximum = self.calibration_ranges[joint]
            current_outside = self._outside_distance(
                current_angles[joint], minimum, maximum
            )
            target_outside = self._outside_distance(target_angle, minimum, maximum)
            if target_outside > 1e-6:
                if current_outside <= 1e-6 or target_outside >= current_outside - 1e-6:
                    raise RuntimeError(
                        f"{joint} calibration 범위 밖에서 안쪽으로 복구되지 않는 "
                        "명령을 차단했습니다."
                    )
                position = self._unchecked_position(joint, target_angle)
            else:
                position = self.arm.calibration.command_angle_to_position(
                    joint, target_angle
                )
            servo_id = int(self.arm.calibration.get_joint(joint)["servo_id"])
            commands[servo_id] = {
                "position": position,
                "speed": speed,
                "acc": self.acc,
            }
        if not self.arm.driver.sync_write_positions(commands):
            raise RuntimeError("Servo 1·2 휴식자세 복구 SyncWrite 실패")
        return True

    def move(self, target: JointCommand) -> dict[str, Any]:
        if self.arm is None:
            raise RuntimeError("Motor port가 열리지 않았습니다.")
        current = self.read_angles()
        largest_delta = max(
            abs(target.shoulder_lift_deg - current.shoulder_lift_deg),
            abs(target.elbow_flex_deg - current.elbow_flex_deg),
        )
        speed = select_speed(
            self.speed_mode,
            self.maximum_speed,
            self.minimum_speed,
            self.full_speed_error_deg,
            largest_delta,
        )
        recovering = self._move_recovery_aware(current, target, speed)
        return {
            "accepted": True,
            "speed": speed,
            "largest_delta_deg": largest_delta,
            "recovering": recovering,
        }

    def move_rest(
        self,
        target: JointCommand,
        rest_speed_cap: int,
        rest_acc_cap: int,
    ) -> dict[str, Any]:
        """Move to the explicit rest exception at conservative speed/acc caps."""
        if self.arm is None:
            raise RuntimeError("Motor port가 열리지 않았습니다.")
        speed = min(self.maximum_speed, int(rest_speed_cap), ABSOLUTE_SPEED_CAP)
        acc = min(self.acc, int(rest_acc_cap))
        commands = {}
        for joint, angle in (
            ("shoulder_lift", target.shoulder_lift_deg),
            ("elbow_flex", target.elbow_flex_deg),
        ):
            servo_id = int(self.arm.calibration.get_joint(joint)["servo_id"])
            commands[servo_id] = {
                "position": self._unchecked_position(joint, angle),
                "speed": speed,
                "acc": acc,
            }
        if not self.arm.driver.sync_write_positions(commands):
            raise RuntimeError("Servo 1·2 휴식자세 SyncWrite 실패")
        return {"accepted": True, "speed": speed, "acc": acc}

    def close(self) -> None:
        if self.arm is not None:
            self.arm.close()
            self.arm = None


def _serialize_angles(command: JointCommand) -> dict[str, float]:
    return {
        "shoulder_lift_deg": float(command.shoulder_lift_deg),
        "elbow_flex_deg": float(command.elbow_flex_deg),
    }


def _deserialize_angles(payload: dict[str, Any]) -> JointCommand:
    return JointCommand(
        shoulder_lift_deg=float(payload["shoulder_lift_deg"]),
        elbow_flex_deg=float(payload["elbow_flex_deg"]),
    )


def _respond(response_queue, request_id: int, *, result=None, error=None) -> None:
    response_queue.put(
        {
            "request_id": int(request_id),
            "ok": error is None,
            "result": result,
            "error": None if error is None else str(error),
        }
    )


def motor_process_worker(
    request_queue,
    response_queue,
    calibration_path: str,
    speed: int,
    acc: int,
    speed_mode: str,
    minimum_speed: int,
    full_speed_error_deg: float,
    allow_uncalibrated_speed: bool,
) -> None:
    """Child entrypoint. All serial and motor_control calls stay in this process."""
    hardware = TwoMotorHardware(
        Path(calibration_path),
        speed,
        acc,
        speed_mode,
        minimum_speed,
        full_speed_error_deg,
        allow_uncalibrated_speed,
    )
    try:
        hardware.open()
        initial = hardware.read_angles()
        _respond(
            response_queue,
            0,
            result={
                "angles": _serialize_angles(initial),
                "calibration_ranges": hardware.calibration_ranges,
            },
        )
    except Exception as error:
        _respond(response_queue, 0, error=error)
        hardware.close()
        return

    try:
        while True:
            request = request_queue.get()
            request_id = int(request["request_id"])
            request_type = request["type"]
            try:
                if request_type == "read_angles":
                    angles = hardware.read_angles()
                    _respond(
                        response_queue,
                        request_id,
                        result={"angles": _serialize_angles(angles)},
                    )
                elif request_type == "move":
                    target = _deserialize_angles(request["angles"])
                    move_result = hardware.move(target)
                    _respond(response_queue, request_id, result=move_result)
                elif request_type == "move_rest":
                    target = _deserialize_angles(request["angles"])
                    move_result = hardware.move_rest(
                        target,
                        int(request["rest_speed_cap"]),
                        int(request["rest_acc_cap"]),
                    )
                    _respond(response_queue, request_id, result=move_result)
                elif request_type == "shutdown":
                    _respond(response_queue, request_id, result={"closed": True})
                    break
                else:
                    raise ValueError(f"알 수 없는 모터 프로세스 메시지: {request_type}")
            except Exception as error:
                _respond(response_queue, request_id, error=error)
    finally:
        hardware.close()


class MotorControlProcessClient:
    """Main-process proxy exposing angle telemetry and angle-command messages."""

    def __init__(
        self,
        calibration_path: Path,
        speed: int,
        acc: int,
        speed_mode: str = FIXED_SPEED_MODE,
        minimum_speed: int = 1,
        full_speed_error_deg: float = 30.0,
        allow_uncalibrated_speed: bool = False,
        response_timeout_s: float = 5.0,
    ):
        self.calibration_path = Path(calibration_path)
        self.speed = int(speed)
        self.acc = int(acc)
        self.speed_mode = str(speed_mode)
        self.minimum_speed = int(minimum_speed)
        self.full_speed_error_deg = float(full_speed_error_deg)
        self.allow_uncalibrated_speed = bool(allow_uncalibrated_speed)
        self.response_timeout_s = float(response_timeout_s)
        self.context = mp.get_context("spawn")
        self.request_queue = None
        self.response_queue = None
        self.process = None
        self.next_request_id = 1
        self.calibration_ranges: dict[str, tuple[float, float]] = {}
        self.initial_angles: JointCommand | None = None

    def open(self) -> None:
        if self.process is not None:
            return
        self.request_queue = self.context.Queue()
        self.response_queue = self.context.Queue()
        self.process = self.context.Process(
            target=motor_process_worker,
            args=(
                self.request_queue,
                self.response_queue,
                str(self.calibration_path),
                self.speed,
                self.acc,
                self.speed_mode,
                self.minimum_speed,
                self.full_speed_error_deg,
                self.allow_uncalibrated_speed,
            ),
            name="poco-monitor-arm-motor12",
        )
        self.process.start()
        try:
            result = self._receive(0)
        except Exception:
            self.close()
            raise
        self.initial_angles = _deserialize_angles(result["angles"])
        self.calibration_ranges = {
            name: (float(values[0]), float(values[1]))
            for name, values in result["calibration_ranges"].items()
        }

    def _receive(self, expected_request_id: int) -> dict[str, Any]:
        if self.response_queue is None:
            raise RuntimeError("모터 프로세스가 시작되지 않았습니다.")
        try:
            response = self.response_queue.get(timeout=self.response_timeout_s)
        except queue.Empty as error:
            raise RuntimeError(
                f"모터 프로세스 응답 시간 초과: request={expected_request_id}"
            ) from error
        if int(response.get("request_id", -1)) != expected_request_id:
            raise RuntimeError(
                f"모터 응답 ID 불일치: expected={expected_request_id}, "
                f"actual={response.get('request_id')}"
            )
        if not response.get("ok"):
            raise RuntimeError(str(response.get("error", "모터 프로세스 오류")))
        return dict(response.get("result") or {})

    def _request(self, request_type: str, **payload) -> dict[str, Any]:
        if self.process is None or self.request_queue is None:
            raise RuntimeError("모터 프로세스가 열리지 않았습니다.")
        if not self.process.is_alive():
            raise RuntimeError("모터 프로세스가 종료되었습니다.")
        request_id = self.next_request_id
        self.next_request_id += 1
        self.request_queue.put(
            {"request_id": request_id, "type": request_type, **payload}
        )
        return self._receive(request_id)

    def read_angles(self) -> JointCommand:
        result = self._request("read_angles")
        return _deserialize_angles(result["angles"])

    def move(self, target: JointCommand) -> dict[str, Any]:
        return self._request("move", angles=_serialize_angles(target))

    def move_rest(
        self,
        target: JointCommand,
        rest_speed_cap: int = 200,
        rest_acc_cap: int = 10,
    ) -> dict[str, Any]:
        return self._request(
            "move_rest",
            angles=_serialize_angles(target),
            rest_speed_cap=int(rest_speed_cap),
            rest_acc_cap=int(rest_acc_cap),
        )

    def close(self) -> None:
        process = self.process
        if process is None:
            return
        if process.is_alive() and self.request_queue is not None:
            try:
                self._request("shutdown")
            except Exception:
                pass
        process.join(timeout=2.0)
        if process.is_alive():
            process.terminate()
            process.join(timeout=1.0)
        for channel in (self.request_queue, self.response_queue):
            if channel is not None:
                channel.close()
                channel.join_thread()
        self.process = None
        self.request_queue = None
        self.response_queue = None

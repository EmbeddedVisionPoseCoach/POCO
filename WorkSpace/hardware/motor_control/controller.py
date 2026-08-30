"""
motor_control/controller.py

[파이프라인에서의 역할]
다른 팀원이 실제 AI/센서/조건 로직에서 사용하는 상위 Motor API이다.

팀원이 기본적으로 지정하는 값:
- joint_name
- angle 또는 delta_angle
- speed

선택 인자:
- acc (기본 10)
- wait (기본 True)
- timeout

[팀원용 방향]
- shoulder_lift : + = 위,  - = 아래
- elbow_flex    : + = 위,  - = 아래
- wrist_flex    : + = 위,  - = 아래
- wrist_roll    : + = CW,  - = CCW

[두 종류의 정지 기능]
1) emergency_stop()
   - 기존 팀원 호환성을 위해 이름/동작을 그대로 유지한다.
   - 개발/점검 상황에서 전체 Servo Torque를 OFF한다.
   - Torque OFF 후 로봇팔이 중력으로 떨어질 수 있다.

2) user_stop()
   - 최종 사용자용 정지 기능이다.
   - Torque를 OFF하지 않는다.
   - 4축 현재 Position을 읽고 한 번의 SyncWrite로 각 축 현재 위치를 Goal로 설정한다.
   - 별도 user-stop latch를 ON하여 이후 모든 이동 API를 차단한다.
   - resume_user_stop()은 latch만 해제하며 자체 이동 명령은 보내지 않는다.

[동시성 / 재호출]
- 실제 이동 명령의 read/calculate/write 임계구역은 _command_lock으로 보호한다.
- wait=True 중 다른 Thread가 같은 Servo에 새 목표를 보내면 Servo별 command generation이
  변경되므로 기존 wait는 Timeout까지 기다리지 않고 '새 명령으로 대체됨'을 감지해 종료한다.
- wait polling 중에는 _command_lock을 잡지 않으므로 다른 Thread의 새 명령/정지 요청이 가능하다.
"""

import threading
import time

from .config import (
    DEFAULT_ACC,
    DEFAULT_WAIT,
    DEFAULT_TIMEOUT_SEC,
    POSITION_TOLERANCE,
    POLL_INTERVAL_SEC,
    USER_STOP_HOLD_SPEED,
    USER_STOP_HOLD_ACC,
)
from .calibration import CalibrationManager, CalibrationError
from .servo_driver import ServoDriver


class MotorController:
    """팀원용 상위 모터 제어 API."""

    def __init__(self, calibration_file=None):
        # ----------------------------------------------------
        # Calibration Load
        # ----------------------------------------------------
        if calibration_file is None:
            self.calibration = CalibrationManager()
        else:
            self.calibration = CalibrationManager(calibration_file)

        # ----------------------------------------------------
        # STServo Driver
        # ----------------------------------------------------
        self.driver = ServoDriver(
            device=self.calibration.device,
            baudrate=self.calibration.baudrate,
        )

        # ----------------------------------------------------
        # Stop latch
        # ----------------------------------------------------
        # 기존 emergency_stop() 상태:
        #   개발/점검용 전체 Torque OFF + 이후 이동 차단.
        self._emergency_event = threading.Event()

        # 새 user_stop() 상태:
        #   Torque ON 상태로 현재 위치 Hold + 이후 이동 차단.
        self._user_stop_event = threading.Event()

        # ----------------------------------------------------
        # Command serialization
        # ----------------------------------------------------
        # 실제 Goal write / relative read->calculate->write / user_stop Hold /
        # emergency_stop Torque OFF가 서로 경합하지 않도록 보호한다.
        # wait 구간에서는 이 Lock을 잡지 않으므로 다른 Thread의 재호출/정지가 가능하다.
        self._command_lock = threading.RLock()

        # ----------------------------------------------------
        # Servo별 command generation
        # ----------------------------------------------------
        # wait=True인 이전 명령이 다른 Thread의 새 명령으로 대체되었는지 판단한다.
        # generation은 실제 Goal 전송이 성공한 뒤에만 증가한다.
        self._generation_lock = threading.Lock()
        self._command_generation = {
            int(servo_id): 0
            for servo_id in self.calibration.servos_by_id.keys()
        }

    # ========================================================
    # 공통 Error / Stop 상태 검사
    # ========================================================

    @staticmethod
    def _print_error(error):
        print(f"[MOTOR ERROR] {error}")

    def _check_emergency_state(self):
        """
        기존 emergency_stop() latch만 검사한다.

        기존 내부 동작/호환성을 유지하기 위해 별도 메서드로 남겨둔다.
        일반 이동 API에서는 user_stop까지 함께 검사하는
        _check_motion_allowed()를 사용한다.
        """
        if self._emergency_event.is_set():
            self._print_error(
                "Emergency Stop 상태입니다. "
                "모든 모터 이동 명령이 차단되어 있습니다."
            )
            return False

        return True

    def _check_motion_allowed(self):
        """기존 Emergency Stop과 사용자 Stop을 모두 검사한다."""
        if not self._check_emergency_state():
            return False

        if self._user_stop_event.is_set():
            self._print_error(
                "User Stop 상태입니다. "
                "resume_user_stop() 호출 전까지 모든 모터 이동 명령이 차단됩니다."
            )
            return False

        return True

    def is_emergency_stopped(self):
        """기존 emergency_stop() latch 상태를 반환한다."""
        return self._emergency_event.is_set()

    def is_user_stopped(self):
        """새 사용자용 user_stop() latch 상태를 반환한다."""
        return self._user_stop_event.is_set()

    # ========================================================
    # Command generation 내부 관리
    # ========================================================

    def _bump_command_generations(self, servo_ids):
        """
        실제 Goal 명령이 성공한 Servo의 generation을 증가시키고
        증가 후 값을 snapshot으로 반환한다.
        """
        snapshot = {}

        with self._generation_lock:
            for servo_id in servo_ids:
                servo_id = int(servo_id)
                current = self._command_generation.get(servo_id, 0)
                current += 1
                self._command_generation[servo_id] = current
                snapshot[servo_id] = current

        return snapshot

    def _generations_are_current(self, expected_generations):
        """wait가 기억한 generation이 아직 최신 명령인지 확인한다."""
        if not expected_generations:
            return True

        with self._generation_lock:
            for servo_id, expected in expected_generations.items():
                current = self._command_generation.get(int(servo_id), 0)
                if current != int(expected):
                    return False

        return True

    # ========================================================
    # 기존 Emergency Stop - 팀원 호환성 유지
    # ========================================================

    def emergency_stop(self):
        """
        모든 Servo Torque를 OFF하고 이후 모든 이동 명령을 차단한다.

        중요:
        - 기존 팀원들이 사용 중인 함수이므로 이름과 Torque OFF 동작을 유지한다.
        - 이 함수는 최종 사용자용 자세 유지 정지가 아니다.
        - Torque OFF 순간 로봇팔이 중력으로 떨어질 수 있다.
        - 현재 버전은 의도적으로 자동 reset/Torque ON API를 제공하지 않는다.
        """
        # Torque OFF 전부터 새로운 이동 명령을 막는다.
        self._emergency_event.set()

        servo_ids = sorted(self.calibration.servos_by_id.keys())

        try:
            with self._command_lock:
                success = self.driver.disable_torque_all_sync(servo_ids)
        except Exception as error:
            # 통신 예외가 발생해도 Emergency latch는 절대 자동 해제하지 않는다.
            self._print_error(
                f"Emergency Stop Torque OFF 전송 중 오류: {error}"
            )
            return False

        if not success:
            # 송신 실패여도 latch는 유지한다.
            self._print_error(
                "Emergency Stop Torque OFF 패킷 전송에 실패했습니다. "
                "Emergency 상태는 계속 유지됩니다."
            )
            return False

        print(
            "[EMERGENCY STOP] 모든 Servo에 Torque OFF 명령을 전송했습니다. "
            "이후 이동 명령은 차단됩니다."
        )
        return True

    # ========================================================
    # 사용자용 Stop - Torque ON 현재 위치 Hold
    # ========================================================

    @staticmethod
    def _user_stop_speed_for_servo(servo):
        """
        user_stop Hold SyncWrite에 넣을 Speed를 정한다.

        기본값은 실제 단축 테스트에서 사용한 USER_STOP_HOLD_SPEED이며,
        Calibration max_speed가 더 낮으면 그 이하로 제한한다.
        user_stop은 Safe Angle 계산이 아니라 현재 raw Position Hold이므로
        현재 위치 자체를 별도로 clamp하지 않는다.
        """
        max_speed = servo.get("max_speed")

        try:
            max_speed = int(max_speed)
        except (TypeError, ValueError):
            max_speed = USER_STOP_HOLD_SPEED

        if max_speed <= 0:
            max_speed = USER_STOP_HOLD_SPEED

        return max(1, min(int(USER_STOP_HOLD_SPEED), max_speed))

    def user_stop(self):
        """
        최종 사용자용 정지.

        동작 순서:
        1. user-stop latch를 즉시 ON해 새 이동 요청을 먼저 차단한다.
        2. _command_lock을 획득한다.
        3. 4축 Present Position만 빠르게 읽는다.
        4. 한 번의 SyncWrite로 각 축 현재 Position을 새 Goal로 덮어쓴다.
        5. Torque는 변경하지 않는다.
        6. 성공한 Hold 명령으로 기존 목표가 대체되므로 4축 generation을 갱신한다.

        실패 정책:
        - Position 읽기/SyncWrite가 실패해도 user-stop latch는 자동 해제하지 않는다.
        - 따라서 이후 이동 명령은 계속 software block 상태로 남는다.
        """
        if self._emergency_event.is_set():
            self._print_error(
                "기존 Emergency Stop(Torque OFF) 상태이므로 "
                "user_stop() 현재 위치 Hold를 수행할 수 없습니다."
            )
            return False

        # command_lock을 기다리는 동안에도 새 이동 요청을 즉시 막기 위해
        # latch를 먼저 켠다.
        self._user_stop_event.set()

        servo_ids = sorted(self.calibration.servos_by_id.keys())

        try:
            with self._command_lock:
                self._user_stop_event.set()  # 대기 중 resume가 먼저 실행된 race도 최종적으로 다시 latch
                if self._emergency_event.is_set():
                    self._print_error(
                        "user_stop() 처리 중 Emergency Stop(Torque OFF)이 발생했습니다. "
                        "User Stop latch는 유지됩니다."
                    )
                    return False

                # 전체 상태가 아니라 Position만 읽는다.
                positions = self.driver.read_positions(servo_ids)

                # 통신 일시 오류일 가능성에 대비해 실패했을 때만 한 번 재시도한다.
                if positions is None:
                    positions = self.driver.read_positions(servo_ids)

                if positions is None:
                    self._print_error(
                        "User Stop 현재 Position 읽기에 실패했습니다. "
                        "이동 차단 latch는 계속 유지됩니다."
                    )
                    return False

                sync_commands = {}
                for servo_id in servo_ids:
                    servo = self.calibration.servos_by_id[int(servo_id)]
                    sync_commands[int(servo_id)] = {
                        "position": int(positions[int(servo_id)]),
                        "speed": self._user_stop_speed_for_servo(servo),
                        "acc": int(USER_STOP_HOLD_ACC),
                    }

                # Position Read 이후 Emergency Stop 요청이 들어왔다면
                # Torque OFF보다 늦게 Goal을 덮어쓰지 않도록 다시 검사한다.
                if self._emergency_event.is_set():
                    self._print_error(
                        "User Stop Hold 전 Emergency Stop(Torque OFF)이 감지되어 "
                        "Position Hold 전송을 중단합니다."
                    )
                    return False

                # 4축 모두 한 번의 SyncWrite로 현재 위치를 Goal로 설정한다.
                success = self.driver.sync_write_positions(sync_commands)

                if not success:
                    self._print_error(
                        "User Stop 4축 현재 위치 Hold SyncWrite에 실패했습니다. "
                        "이동 차단 latch는 계속 유지됩니다."
                    )
                    return False

                # Hold Goal이 기존 각 축 Goal을 대체했으므로 모든 generation 무효화.
                self._bump_command_generations(servo_ids)

        except Exception as error:
            self._print_error(
                f"User Stop 처리 중 오류: {error}. "
                "이동 차단 latch는 계속 유지됩니다."
            )
            return False

        print(
            "[USER STOP] Torque를 유지한 채 4축 현재 위치를 Goal로 Hold했습니다. "
            "resume_user_stop() 전까지 이동 명령은 차단됩니다."
        )
        return True

    def resume_user_stop(self):
        """
        user_stop latch만 해제한다.

        중요:
        - Torque를 변경하지 않는다.
        - Zero/이전 목표/새 목표로 이동하지 않는다.
        - 현재 Hold 위치를 그대로 유지한다.
        - 실제 움직임은 이후 별도의 명시적 이동 명령이 들어올 때만 시작된다.
        """
        # user_stop()이 Hold를 전송하는 중이라면 그 작업이 끝난 뒤 latch를 해제한다.
        with self._command_lock:
            was_stopped = self._user_stop_event.is_set()
            self._user_stop_event.clear()

        if was_stopped:
            if self._emergency_event.is_set():
                print(
                    "[USER STOP] User Stop latch만 해제했습니다. "
                    "기존 Emergency Stop(Torque OFF) 상태는 계속 유지됩니다."
                )
            else:
                print(
                    "[USER STOP] User Stop latch를 해제했습니다. "
                    "해제 자체로는 어떤 이동 명령도 전송하지 않습니다."
                )

        return True

    # ========================================================
    # 1. Zero 기준 절대각도 제어
    # ========================================================

    def move_joint(
        self,
        joint_name,
        angle,
        speed,
        acc=DEFAULT_ACC,
        wait=DEFAULT_WAIT,
        timeout=DEFAULT_TIMEOUT_SEC,
    ):
        """
        Zero Position을 0°로 보고 최종 목표 각도로 이동한다.

        예:
            move_joint("shoulder_lift", 30, 100)
            -> 현재 위치와 관계없이 팀원 기준 +30° 위치로 이동

            move_joint("wrist_roll", 30, 100)
            -> CW +30° 위치로 이동
        """
        if not self._check_motion_allowed():
            return False

        try:
            speed = self.calibration.validate_speed(joint_name, speed)
            acc = self.calibration.validate_acc(acc)
            target_position = self.calibration.command_angle_to_position(
                joint_name,
                angle,
            )
            servo = self.calibration.get_joint(joint_name)
            servo_id = int(servo["servo_id"])

        except CalibrationError as error:
            self._print_error(error)
            return False

        # 검증 후 실제 쓰기 직전에도 stop 상태를 다시 확인한다.
        # 확인 + 실제 Goal write + generation 갱신을 같은 command lock 안에서 처리한다.
        with self._command_lock:
            if not self._check_motion_allowed():
                return False

            success = self.driver.write_position(
                servo_id=servo_id,
                position=target_position,
                speed=speed,
                acc=acc,
            )

            if success:
                expected_generations = self._bump_command_generations(
                    [servo_id]
                )
            else:
                expected_generations = None

        if not success:
            self._print_error(f"{joint_name} 이동 명령 실패")
            return False

        if not wait:
            return True

        return self._wait_for_targets(
            {servo_id: target_position},
            timeout=timeout,
            expected_generations=expected_generations,
        )

    # ========================================================
    # 2. 현재 위치 기준 상대각도 제어
    # ========================================================

    def move_joint_relative(
        self,
        joint_name,
        delta_angle,
        speed,
        acc=DEFAULT_ACC,
        wait=DEFAULT_WAIT,
        timeout=DEFAULT_TIMEOUT_SEC,
    ):
        """
        현재 실제 위치를 기준으로 delta_angle만큼 추가 이동한다.

        read -> calculate -> write 전체를 _command_lock 안에서 처리한다.
        따라서 다른 Thread가 중간에 새 Goal을 삽입해 오래된 Position을 기준으로
        상대 목표를 계산하는 문제를 방지한다.
        """
        if not self._check_motion_allowed():
            return False

        try:
            servo = self.calibration.require_position_calibrated(joint_name)
            servo_id = int(servo["servo_id"])
            speed = self.calibration.validate_speed(joint_name, speed)
            acc = self.calibration.validate_acc(acc)
            delta_angle = float(delta_angle)

        except (CalibrationError, TypeError, ValueError) as error:
            self._print_error(error)
            return False

        # 상대이동의 핵심 원자 구간:
        # 현재값 읽기 -> 목표 계산 -> 안전검사 -> 실제 write가 모두 같은 lock 안에 있다.
        with self._command_lock:
            if not self._check_motion_allowed():
                return False

            current_position = self.driver.read_position(servo_id)
            if current_position is None:
                self._print_error(f"{joint_name} 현재 Position 읽기 실패")
                return False

            try:
                current_angle = self.calibration.position_to_command_angle(
                    joint_name,
                    current_position,
                )
                target_angle = current_angle + delta_angle
                target_position = self.calibration.command_angle_to_position(
                    joint_name,
                    target_angle,
                )
            except (CalibrationError, TypeError, ValueError) as error:
                self._print_error(error)
                return False

            # Position read/계산 중 user_stop()이 latch를 먼저 켰을 수 있으므로
            # 실제 write 직전에 한 번 더 확인한다.
            if not self._check_motion_allowed():
                return False

            success = self.driver.write_position(
                servo_id=servo_id,
                position=target_position,
                speed=speed,
                acc=acc,
            )

            if success:
                expected_generations = self._bump_command_generations(
                    [servo_id]
                )
            else:
                expected_generations = None

        if not success:
            self._print_error(f"{joint_name} 상대 이동 명령 실패")
            return False

        if not wait:
            return True

        return self._wait_for_targets(
            {servo_id: target_position},
            timeout=timeout,
            expected_generations=expected_generations,
        )

    # ========================================================
    # 3. 여러 Joint 동시 제어
    # ========================================================

    def move_joints(
        self,
        targets,
        speed,
        acc=DEFAULT_ACC,
        wait=DEFAULT_WAIT,
        timeout=DEFAULT_TIMEOUT_SEC,
    ):
        """
        여러 Joint 목표를 모두 먼저 검증한 뒤 SyncWrite로 같이 시작한다.

        하나라도 Calibration / Speed / Safe Range 검증에 실패하면
        실제 Servo에는 아무 명령도 보내지 않는다.
        """
        if not self._check_motion_allowed():
            return False

        if not isinstance(targets, dict):
            self._print_error("targets는 dict여야 합니다.")
            return False

        if not targets:
            self._print_error("targets가 비어 있습니다.")
            return False

        sync_commands = {}
        wait_targets = {}

        try:
            acc = self.calibration.validate_acc(acc)

            for joint_name, angle in targets.items():
                joint_speed = self.calibration.validate_speed(
                    joint_name,
                    speed,
                )

                target_position = self.calibration.command_angle_to_position(
                    joint_name,
                    angle,
                )

                servo = self.calibration.get_joint(joint_name)
                servo_id = int(servo["servo_id"])

                sync_commands[servo_id] = {
                    "position": target_position,
                    "speed": joint_speed,
                    "acc": acc,
                }

                wait_targets[servo_id] = target_position

        except CalibrationError as error:
            self._print_error(error)
            return False

        # 모든 계산/검증이 끝난 직후에도 stop 상태를 재확인한다.
        # 확인 + SyncWrite + generation 갱신을 같은 command lock 안에서 처리한다.
        with self._command_lock:
            if not self._check_motion_allowed():
                return False

            success = self.driver.sync_write_positions(sync_commands)

            if success:
                expected_generations = self._bump_command_generations(
                    wait_targets.keys()
                )
            else:
                expected_generations = None

        if not success:
            self._print_error("여러 Joint 동기 이동 명령 실패")
            return False

        if not wait:
            return True

        return self._wait_for_targets(
            wait_targets,
            timeout=timeout,
            expected_generations=expected_generations,
        )

    def move_joints_special(
        self,
        targets,
        speed,
        acc=DEFAULT_ACC,
        wait=DEFAULT_WAIT,
        timeout=DEFAULT_TIMEOUT_SEC,
    ):
        """Motor1/2의 검증된 Rest/Recovery 예외 자세를 SyncWrite한다.

        일반 ``move_joints()``와 달리 Calibration Safe Range만 우회한다.

        반드시 유지하는 안전장치:
        - Emergency Stop / User Stop latch
        - Motor1/2 이외 Joint 차단
        - Speed max_speed 검사
        - Acc 검사
        - Position Calibration 완료 검사
        - STS 절대 Position 0~4095 검사
        - command lock
        - command generation 갱신

        이 메서드는 일반 팀원 제어용 API가 아니라 Motor12Controller의
        Rest/Recovery 전용 내부 경로다.
        """
        if not self._check_motion_allowed():
            return False

        if not isinstance(targets, dict):
            self._print_error("targets는 dict여야 합니다.")
            return False

        required_joints = {
            "shoulder_lift",
            "elbow_flex",
        }

        # 예외 경로를 다른 Servo에 재사용하지 못하도록
        # Motor1/2 두 축을 함께 지정한 경우만 허용한다.
        if set(targets.keys()) != required_joints:
            self._print_error(
                "Special Position 예외 이동은 "
                "shoulder_lift + elbow_flex 두 Joint를 "
                "동시에 지정해야 합니다."
            )
            return False

        sync_commands = {}
        wait_targets = {}

        try:
            acc = self.calibration.validate_acc(acc)

            for joint_name, angle in targets.items():
                joint_speed = (
                    self.calibration.validate_speed(
                        joint_name,
                        speed,
                    )
                )

                # Safe Range만 생략하고 STS 절대범위는 검사한다.
                target_position = (
                    self.calibration
                    .command_angle_to_position_absolute_only(
                        joint_name,
                        angle,
                    )
                )

                servo = self.calibration.get_joint(joint_name)
                servo_id = int(servo["servo_id"])

                sync_commands[servo_id] = {
                    "position": target_position,
                    "speed": joint_speed,
                    "acc": acc,
                }

                wait_targets[servo_id] = (target_position)

        except CalibrationError as error:
            self._print_error(error)
            return False

        # 기존 move_joints()와 동일하게 실제 Write 직전에
        # Stop 상태를 다시 확인하고 command lock 안에서 전송한다.
        with self._command_lock:
            if not self._check_motion_allowed():
                return False

            success = (
                self.driver.sync_write_positions(
                    sync_commands
                )
            )

            if success:
                expected_generations = (
                    self._bump_command_generations(
                        wait_targets.keys()
                    )
                )
            else:
                expected_generations = None

        if not success:
            self._print_error(
                "Motor1/2 Special Position "
                "SyncWrite 실패"
            )
            return False

        if not wait:
            return True

        return self._wait_for_targets(
            wait_targets,
            timeout=timeout,
            expected_generations=(
                expected_generations
            ),
        )

    # ========================================================
    # 4. Zero 이동
    # ========================================================

    def move_to_zero(
        self,
        joint_name,
        speed,
        acc=DEFAULT_ACC,
        wait=DEFAULT_WAIT,
        timeout=DEFAULT_TIMEOUT_SEC,
    ):
        return self.move_joint(
            joint_name=joint_name,
            angle=0.0,
            speed=speed,
            acc=acc,
            wait=wait,
            timeout=timeout,
        )

    def move_all_to_zero(
        self,
        speed,
        acc=DEFAULT_ACC,
        wait=DEFAULT_WAIT,
        timeout=DEFAULT_TIMEOUT_SEC,
    ):
        if not self._check_motion_allowed():
            return False

        targets = {
            joint_name: 0.0
            for joint_name in self.calibration.servos_by_joint.keys()
        }

        return self.move_joints(
            targets=targets,
            speed=speed,
            acc=acc,
            wait=wait,
            timeout=timeout,
        )

    # ========================================================
    # 5. 현재 각도 / 상태 읽기
    # ========================================================
    # 상태 읽기는 두 종류의 Stop 상태에서도 허용한다.
    # 정지 후에도 현재 Position/Load/온도 등을 확인할 수 있어야 하기 때문이다.

    def get_joint_angle(self, joint_name):
        try:
            servo = self.calibration.require_position_calibrated(joint_name)
            servo_id = int(servo["servo_id"])

            position = self.driver.read_position(servo_id)
            if position is None:
                self._print_error(f"{joint_name} Position 읽기 실패")
                return None

            return self.calibration.position_to_command_angle(
                joint_name,
                position,
            )

        except CalibrationError as error:
            self._print_error(error)
            return None

    def get_joint_state(self, joint_name):
        try:
            servo = self.calibration.require_position_calibrated(joint_name)
            servo_id = int(servo["servo_id"])

            state = self.driver.read_state(servo_id)
            if state is None:
                self._print_error(f"{joint_name} 상태 읽기 실패")
                return None

            angle = self.calibration.position_to_command_angle(
                joint_name,
                state["position"],
            )

            return {
                "joint": joint_name,
                "angle": angle,
                "speed": state["speed"],
                "load": state["load"],
                "load_percent": state["load_percent"],
                "voltage": state["voltage"],
                "temperature": state["temperature"],
                "current_raw": state["current_raw"],
                "moving": (
                    None
                    if state["moving"] is None
                    else bool(state["moving"])
                ),
            }

        except CalibrationError as error:
            self._print_error(error)
            return None

    def get_all_states(self):
        """
        일반 모니터링용 전체 상태 읽기.

        각 Servo를 순서대로 읽으므로 4축이 완전히 같은 시각의 snapshot은 아니다.
        user_stop()은 이 함수를 사용하지 않고 Position-only 경량 read를 사용한다.
        """
        return {
            joint_name: self.get_joint_state(joint_name)
            for joint_name in self.calibration.servos_by_joint.keys()
        }

    def is_moving(self, joint_name):
        state = self.get_joint_state(joint_name)
        if state is None:
            return None
        return state["moving"]

    # ========================================================
    # 6. wait=True 목표 도착 대기
    # ========================================================

    def _wait_for_targets(
        self,
        targets_by_servo_id,
        timeout,
        expected_generations=None,
    ):
        """
        목표 도착을 기다린다.

        일반 전체 read_state() 대신 Position + Moving만 읽는 경량 경로를 사용한다.
        또한 Servo별 generation을 확인해 wait 중 새 목표가 들어오면 즉시 종료한다.
        """
        start_time = time.monotonic()

        while time.monotonic() - start_time < float(timeout):
            if self._emergency_event.is_set():
                self._print_error(
                    "Emergency Stop이 발생하여 목표 도착 대기를 중단합니다."
                )
                return False

            if self._user_stop_event.is_set():
                self._print_error(
                    "User Stop이 발생하여 목표 도착 대기를 중단합니다."
                )
                return False

            if not self._generations_are_current(expected_generations):
                self._print_error(
                    "기존 이동 명령이 더 새로운 명령으로 대체되어 "
                    "목표 도착 대기를 종료합니다."
                )
                return False

            all_arrived = True

            for servo_id, target_position in targets_by_servo_id.items():
                motion_state = self.driver.read_motion_state(servo_id)

                if motion_state is None:
                    self._print_error(
                        f"Servo ID {servo_id} Position/Moving 읽기 실패"
                    )
                    return False

                position_error = abs(
                    motion_state["position"] - target_position
                )
                moving = motion_state["moving"]

                if not (
                    moving == 0
                    and position_error <= POSITION_TOLERANCE
                ):
                    all_arrived = False

            if all_arrived:
                # 상태를 읽는 사이 새 명령이 들어온 경우 오래된 wait가 True를
                # 반환하지 않도록 성공 직전에 한 번 더 generation/stop을 확인한다.
                if self._emergency_event.is_set():
                    return False

                if self._user_stop_event.is_set():
                    return False

                if not self._generations_are_current(expected_generations):
                    self._print_error(
                        "목표 확인 직전에 더 새로운 명령이 들어와 "
                        "기존 목표 대기를 종료합니다."
                    )
                    return False

                return True

            time.sleep(POLL_INTERVAL_SEC)

        # Timeout 직전 새 명령/정지가 들어온 경우 원인을 Timeout으로 잘못 표시하지 않는다.
        if self._emergency_event.is_set():
            self._print_error(
                "Emergency Stop이 발생하여 목표 도착 대기를 중단합니다."
            )
            return False

        if self._user_stop_event.is_set():
            self._print_error(
                "User Stop이 발생하여 목표 도착 대기를 중단합니다."
            )
            return False

        if not self._generations_are_current(expected_generations):
            self._print_error(
                "기존 이동 명령이 더 새로운 명령으로 대체되어 "
                "목표 도착 대기를 종료합니다."
            )
            return False

        self._print_error(f"목표 도착 Timeout: {timeout} sec")
        return False

    # ========================================================
    # 7. 종료
    # ========================================================

    def close(self):
        """
        Serial Port를 닫는다.

        주의:
        close()는 어떤 Stop 기능도 아니며 Torque를 자동으로 OFF하지 않는다.
        """
        self.driver.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

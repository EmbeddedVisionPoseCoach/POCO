"""
POCO Passive Buzzer Terminal Integration Test

파이프라인
----------
Terminal Input
    ↓
MAIN_STATE / POSE_STATE
    ↓
Hardware Process의 자세 Alert 처리 구조 재현
    ↓
PostureAlertService
    ↓
BuzzerService
    ↓
Raspberry Pi GPIO18 PWM
    ↓
Passive Buzzer


목적
----
mainpyQt.py, Camera, Pose GRU, Motor/IMU/ToF 전체를 실행하기 어려운
상황에서 터미널 입력만으로 POCO 자세 부저 연동을 검증한다.

이 테스트는 다음 production 구조를 그대로 사용한다.

- MAIN_STATE["state"]
- POSE_STATE
- latest State Queue 정책
- UPDATE_ALARM_SETTINGS Event 구조
- MEASURING Gate
- Pose frame_id 중복 처리 방지
- PostureAlertService
- BuzzerService


중요
----
실제 run_hardware_process() 전체는 실행하지 않는다.

Hardware Process 전체를 실행하면
Motor / IMU / ToF까지 실제 하드웨어가 같이 열리기 때문이다.

set 명령으로 변경한 Alarm Settings는 테스트 Runtime에만 반영하고
alarm_settings.json 파일에는 저장하지 않는다.
"""

import sys
import time

from pathlib import Path
from queue import Queue


# ============================================================
# Path Setup
# ============================================================
# 이 파일은 WorkSpace/pyQt/ 아래에 위치한다.
#
# pyQt 내부의 services / ipc와
# WorkSpace/modules를 모두 import할 수 있도록
# __file__ 기준으로 경로를 등록한다.
PYQT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = PYQT_DIR.parent

for path in (
    PYQT_DIR,
    WORKSPACE_DIR,
):
    path_text = str(path)

    if path_text not in sys.path:
        sys.path.insert(
            0,
            path_text,
        )


from modules.app_settings import (
    AlarmSettings,
    SettingsManager,
)

from ipc.queue_utils import (
    drain_ordered,
    get_latest,
    put_latest,
    put_ordered,
)

from services.buzzer_service import (
    BuzzerService,
)

from services.posture_alert_service import (
    POSE_COMMANDS,
    PostureAlertService,
)


# ============================================================
# Test Constants
# ============================================================

# 실제 Hardware Process가 읽는 것과 같은 Alarm Settings 파일.
ALARM_SETTINGS_FILE = (
    WORKSPACE_DIR
    / "data"
    / "settings"
    / "alarm_settings.json"
)


# Main / Pose 상태에서 사용하는 mode.
VALID_MODES = {
    "IDLE",
    "PREPARING",
    "CALIBRATING",
    "WAITING",
    "MEASURING",
}


# 실제 Pose Process처럼 같은 자세도
# 계속 새로운 frame_id로 전달하기 위한 주기.
#
# 10Hz면 자세 유지시간 테스트에는 충분하다.
POSE_INTERVAL_SEC = 0.10


# BuzzerService는 non-blocking 상태머신이므로
# Hardware loop를 흉내 내어 지속적으로 update()한다.
BUZZER_TICK_SEC = 0.01


# ============================================================
# Terminal Integration Test
# ============================================================

class BuzzerTerminalTest:
    """
    Main / Pose -> Hardware -> PostureAlert -> Buzzer 흐름 중
    자세 부저와 직접 관련된 영역만 터미널에서 재현한다.
    """

    def __init__(self):

        # ====================================================
        # Alarm Settings
        # ====================================================
        # 실제 Hardware Process와 동일한 설정 파일을 읽는다.
        self.settings_manager = SettingsManager(
            ALARM_SETTINGS_FILE
        )

        self.alarm_settings = (
            self.settings_manager.load()
        )


        # ====================================================
        # Production Services
        # ====================================================

        # 실제 POCO 자세 판단 Service.
        self.posture_alert = (
            PostureAlertService()
        )

        self.posture_alert.apply_settings(
            self.alarm_settings
        )


        # 일반 자세 경고 발생 시
        # 물리적으로 부저를 몇 번 울릴지 결정한다.
        self.posture_alert_count = int(
            self.alarm_settings
            .posture_Hardware_count
        )


        # 실제 GPIO18 PWM을 담당하는 Service.
        self.buzzer = BuzzerService()


        # ====================================================
        # Main / Pose -> Hardware Queue
        # ====================================================

        # production State Queue처럼
        # 최신 데이터 하나만 유지한다.
        self.main_state_queue = Queue(
            maxsize=1
        )

        self.pose_state_queue = Queue(
            maxsize=1
        )


        # Alarm Settings 변경은
        # production과 동일한 Event 구조를 사용한다.
        self.main_event_queue = Queue(
            maxsize=32
        )


        # ====================================================
        # Hardware Alert Runtime State
        # ====================================================

        self.latest_main_state = None
        self.latest_pose_state = None

        self.latest_pose_frame_id = None


        # Hardware Process와 동일하게
        # 같은 Pose frame을 Alert 판단에
        # 여러 번 사용하지 않기 위한 상태.
        self.last_alert_pose_frame_id = None


        # 현재 PROFILE_MODE=POSE_ONLY 상황을 재현.
        self.enable_pose = True


        # 테스트에서 생성할 가상 Pose frame 번호.
        self.frame_id = 0


    # ========================================================
    # Main State
    # ========================================================

    def set_mode(
        self,
        mode,
    ):
        """
        Main -> Hardware State를 생성한다.

        Hardware Process는 실제로
        latest_main_state["state"]를 읽어
        MEASURING 여부를 판단한다.
        """

        mode = str(
            mode
        ).upper()


        if mode not in VALID_MODES:

            raise ValueError(
                "지원 mode: "
                + ", ".join(
                    sorted(
                        VALID_MODES
                    )
                )
            )


        main_state = {
            "type": "MAIN_STATE",
            "state": mode,
            "timestamp": time.time(),
        }


        put_latest(
            self.main_state_queue,
            main_state,
        )


        # State를 Queue에 넣은 뒤
        # Hardware loop 한 번 수행.
        self.hardware_tick()


        print(
            "[TEST] MAIN_STATE -> "
            f"{mode}"
        )


    def get_mode(self):
        """
        실제 Hardware Process와 동일하게
        latest_main_state["state"]를 읽는다.
        """

        if not isinstance(
            self.latest_main_state,
            dict,
        ):
            return ""


        return str(
            self.latest_main_state.get(
                "state",
                "",
            )
        ).upper()


    # ========================================================
    # Pose State
    # ========================================================

    def make_pose_state(
        self,
        pose_index,
    ):
        """
        pose_process_profile.py에서 생성하는
        POSE_STATE 구조를 재현한다.

        실제 production과 동일하게
        inference는 MEASURING에서만 생성한다.
        """

        pose_index = int(
            pose_index
        )


        if pose_index not in POSE_COMMANDS:

            raise ValueError(
                "pose_index는 "
                "0~3만 가능합니다."
            )


        self.frame_id += 1


        mode = self.get_mode()


        # 기본 POSE_STATE.
        pose_state = {

            "type": "POSE_STATE",

            "frame_id": (
                self.frame_id
            ),

            "timestamp_ns": (
                time.perf_counter_ns()
            ),

            "mode": mode,

            "landmark_valid": False,

            "landmarks": None,

            "eye_gap_valid": False,

            "eye_gap_px": None,

            "features": None,

            "inference": None,

            "calibration": None,

            "hardware_state": None,

            "hardware_event": None,
        }


        # 실제 Pose Process와 동일하게
        # GRU inference는 MEASURING에서만 존재한다.
        if mode == "MEASURING":

            pose_state["inference"] = {

                "posture_type": (
                    POSE_COMMANDS[
                        pose_index
                    ]
                ),

                "confidence": 1.0,

                "pose_index": (
                    pose_index
                ),

                "latency_ms": 0.0,
            }


        return pose_state


    def send_pose(
        self,
        pose_index,
        now=None,
        verbose=True,
    ):
        """
        새로운 frame_id를 가진 POSE_STATE
        하나를 Hardware State Queue로 전달한다.
        """

        if now is None:
            now = time.monotonic()


        pose_state = (
            self.make_pose_state(
                pose_index
            )
        )


        put_latest(
            self.pose_state_queue,
            pose_state,
        )


        alert_command = (
            self.hardware_tick(
                now=now
            )
        )


        if verbose:

            print(
                "[TEST] POSE_STATE "
                f"frame={pose_state['frame_id']} "
                f"mode={pose_state['mode']} "
                f"pose={pose_index}:"
                f"{POSE_COMMANDS[int(pose_index)]} "
                f"inference="
                f"{pose_state['inference'] is not None} "
                f"alert={alert_command}"
            )


        return alert_command


    # ========================================================
    # Main -> Hardware Event
    # ========================================================

    def process_main_events(self):
        """
        production Hardware Process의
        UPDATE_ALARM_SETTINGS Event 처리 부분을 재현한다.
        """

        for event in drain_ordered(
            self.main_event_queue
        ):

            event_type = str(
                event.get(
                    "type",
                    "",
                )
            ).upper()


            if (
                event_type
                != "UPDATE_ALARM_SETTINGS"
            ):
                continue


            settings_data = event.get(
                "settings",
                event.get(
                    "data",
                    {},
                ),
            )


            if not isinstance(
                settings_data,
                dict,
            ):

                raise ValueError(
                    "alarm settings는 "
                    "dict여야 합니다."
                )


            # production과 동일하게
            # from_dict() 범위 검사를 다시 거친다.
            self.alarm_settings = (
                AlarmSettings.from_dict(
                    settings_data
                )
            )


            self.posture_alert.apply_settings(
                self.alarm_settings
            )


            self.posture_alert_count = int(
                self.alarm_settings
                .posture_Hardware_count
            )


            print(
                "[TEST] "
                "UPDATE_ALARM_SETTINGS "
                "적용 완료 "
                "(Runtime only)"
            )


    # ========================================================
    # Hardware Process Alert Section
    # ========================================================

    def hardware_tick(
        self,
        now=None,
    ):
        """
        hardware_process.py의
        Pose Alert -> Passive Buzzer 구간을 재현한다.

        처리 순서
        --------
        1. Main 최신 State 수신
        2. Pose 최신 State 수신
        3. Main Event 처리
        4. MEASURING Gate
        5. frame_id 중복 검사
        6. PostureAlertService
        7. BuzzerService
        """

        if now is None:
            now = time.monotonic()


        # ====================================================
        # A. 최신 State 수신
        # ====================================================

        self.latest_main_state = (
            get_latest(
                self.main_state_queue,
                self.latest_main_state,
            )
        )


        self.latest_pose_state = (
            get_latest(
                self.pose_state_queue,
                self.latest_pose_state,
            )
        )


        if isinstance(
            self.latest_pose_state,
            dict,
        ):

            self.latest_pose_frame_id = (
                self.latest_pose_state.get(
                    "frame_id"
                )
            )


        # ====================================================
        # B. Main Event 처리
        # ====================================================

        self.process_main_events()


        main_mode = self.get_mode()


        # ====================================================
        # F-2. Pose 자세 Alert -> Passive Buzzer
        # ====================================================

        measuring_for_alert = bool(
            self.enable_pose
            and main_mode == "MEASURING"
        )


        alert_command = None


        if measuring_for_alert:

            pose_inference = None


            if isinstance(
                self.latest_pose_state,
                dict,
            ):

                pose_inference = (
                    self.latest_pose_state.get(
                        "inference"
                    )
                )


            # Hardware Process는 Pose Process보다
            # 훨씬 빠르게 반복되므로
            # 같은 frame_id는 한 번만 처리한다.
            if (
                isinstance(
                    pose_inference,
                    dict,
                )
                and self.latest_pose_frame_id
                is not None
                and self.latest_pose_frame_id
                != self.last_alert_pose_frame_id
            ):

                self.last_alert_pose_frame_id = (
                    self.latest_pose_frame_id
                )


                pose_index = (
                    pose_inference.get(
                        "pose_index"
                    )
                )


                alert_command = (
                    self.posture_alert.update(
                        pose_index,
                        now=now,
                    )
                )


                if (
                    alert_command
                    is not None
                ):

                    self.buzzer.play_command(
                        alert_command,
                        self.posture_alert_count,
                    )


                    print(
                        "[TEST] "
                        "Hardware Alert -> "
                        f"{alert_command}"
                    )


        # MEASURING이 끝나도 이미 시작된 Alert는
        # production과 동일하게 끝까지 진행시킨다.
        self.buzzer.update(
            now=now
        )


        return alert_command


    # ========================================================
    # Continuous Pose Simulation
    # ========================================================

    def hold(
        self,
        pose_index,
        seconds,
    ):
        """
        실제 Pose Process처럼
        새로운 frame_id를 계속 만들면서
        특정 pose_index를 지정 시간 동안 유지한다.

        예:
            hold 2 6

        -> ForwardHead를 약 10Hz로 6초 동안 전달.
        """

        pose_index = int(
            pose_index
        )

        seconds = float(
            seconds
        )


        if pose_index not in POSE_COMMANDS:

            raise ValueError(
                "pose_index는 "
                "0~3만 가능합니다."
            )


        if seconds <= 0:

            raise ValueError(
                "seconds는 "
                "0보다 커야 합니다."
            )


        print(
            "\n[TEST] HOLD 시작 "
            f"pose={pose_index}:"
            f"{POSE_COMMANDS[pose_index]}, "
            f"duration={seconds:.2f}s, "
            f"mode={self.get_mode()}"
        )


        end_time = (
            time.monotonic()
            + seconds
        )


        next_pose_time = (
            time.monotonic()
        )


        frame_count = 0


        while (
            time.monotonic()
            < end_time
        ):

            now = time.monotonic()


            if (
                now
                >= next_pose_time
            ):

                self.send_pose(
                    pose_index,
                    now=now,
                    verbose=False,
                )


                frame_count += 1


                next_pose_time += (
                    POSE_INTERVAL_SEC
                )


            else:

                # 새로운 Pose frame이 없는 동안에도
                # Hardware처럼 Buzzer를 계속 update.
                self.hardware_tick(
                    now=now
                )


            time.sleep(
                BUZZER_TICK_SEC
            )


        # 터미널 input()은 blocking이므로
        # 이미 시작한 Buzzer Pattern은
        # 다음 입력 전에 끝까지 처리한다.
        self.drain_buzzer()


        print(
            "[TEST] HOLD 완료 "
            f"frames={frame_count}\n"
        )


    def drain_buzzer(self):
        """
        현재 동작 중이거나 Pending 상태인
        Buzzer Pattern을 모두 처리한다.
        """

        while self.buzzer.get_state().get(
            "busy",
            False,
        ):

            self.hardware_tick()

            time.sleep(
                BUZZER_TICK_SEC
            )


    # ========================================================
    # Runtime Alarm Settings
    # ========================================================

    def set_runtime(
        self,
        name,
        value,
    ):
        """
        테스트를 빠르게 하기 위해
        AlarmSettings를 Runtime에서만 변경한다.

        실제 JSON에는 저장하지 않는다.
        """

        field_map = {

            "hold":
                "bad_posture_duration_sec",

            "count":
                "posture_Hardware_count",

            "strong":
                "posture_Strong_limit",

            "cooldown":
                "strong_alert_cooldown_min",

            "alarm":
                "alarm_enabled",
        }


        name = str(
            name
        ).lower()


        if name not in field_map:

            raise ValueError(
                "set 항목: "
                "hold, count, strong, "
                "cooldown, alarm"
            )


        data = (
            self.alarm_settings.to_dict()
        )


        if name == "alarm":

            value_text = str(
                value
            ).lower()


            if value_text in {
                "on",
                "true",
                "1",
            }:

                parsed_value = True


            elif value_text in {
                "off",
                "false",
                "0",
            }:

                parsed_value = False


            else:

                raise ValueError(
                    "alarm은 "
                    "on/off를 사용하세요."
                )


        else:

            parsed_value = int(
                value
            )


        data[
            field_map[name]
        ] = parsed_value


        # Main -> Hardware와 동일한
        # UPDATE_ALARM_SETTINGS Event 생성.
        settings = (
            AlarmSettings.from_dict(
                data
            )
        )


        put_ordered(
            self.main_event_queue,
            {
                "type":
                    "UPDATE_ALARM_SETTINGS",

                "settings":
                    settings.to_dict(),
            },
        )


        self.hardware_tick()

        self.print_settings()


    def reload_settings(self):
        """
        실제 alarm_settings.json 값을
        다시 Runtime에 적용한다.
        """

        saved_settings = (
            self.settings_manager.load()
        )


        put_ordered(
            self.main_event_queue,
            {
                "type":
                    "UPDATE_ALARM_SETTINGS",

                "settings":
                    saved_settings.to_dict(),
            },
        )


        self.hardware_tick()


        print(
            "[TEST] "
            "alarm_settings.json "
            "다시 적용"
        )


        self.print_settings()


    # ========================================================
    # Display
    # ========================================================

    def print_settings(self):

        settings = (
            self.alarm_settings
        )


        print(
            "\n[Runtime Alarm Settings]"
        )

        print(
            " alarm_enabled             : "
            f"{settings.alarm_enabled}"
        )

        print(
            " bad_posture_duration_sec  : "
            f"{settings.bad_posture_duration_sec}"
        )

        print(
            " posture_Hardware_count    : "
            f"{settings.posture_Hardware_count}"
        )

        print(
            " posture_Strong_limit      : "
            f"{settings.posture_Strong_limit}"
        )

        print(
            " strong_alert_cooldown_min : "
            f"{settings.strong_alert_cooldown_min}"
        )

        print(
            " ※ set 명령 변경값은 "
            "JSON에 저장하지 않음\n"
        )


    def print_status(self):

        print(
            "\n========== STATUS =========="
        )

        print(
            "main_mode                : "
            f"{self.get_mode()}"
        )

        print(
            "latest_pose_frame_id     : "
            f"{self.latest_pose_frame_id}"
        )

        print(
            "last_alert_pose_frame_id : "
            f"{self.last_alert_pose_frame_id}"
        )

        print(
            "posture_alert            : "
            f"{self.posture_alert.get_state()}"
        )

        print(
            "buzzer                   : "
            f"{self.buzzer.get_state()}"
        )

        print(
            "============================\n"
        )


    @staticmethod
    def print_help():

        print(
            """
================ 사용 명령 ================

mode <STATE>

    STATE:
        IDLE
        PREPARING
        CALIBRATING
        WAITING
        MEASURING


pose <0~3>

    Pose frame 한 개 전달

    0 = Optimal
    1 = Asymmetric
    2 = ForwardHead
    3 = ChinPropping


hold <0~3> <seconds>

    같은 자세를 새로운 frame_id로
    계속 전달한다.

    예:
        hold 1 3.5


set hold <1~10>

    나쁜 자세 유지시간


set count <1~5>

    일반 Alert에서 실제 부저 반복 횟수


set strong <1~5>

    StrongAlert 승격 기준


set cooldown <1~5>

    StrongAlert Cooldown (분)


set alarm <on|off>

    자세 Alert 사용 여부

    ※ 모든 set 값은 Runtime only
    ※ alarm_settings.json은 수정하지 않음


reload

    실제 alarm_settings.json을 다시 읽음


settings

    현재 Runtime 설정 확인


status

    현재 Alert / Buzzer 상태 확인


help

    명령 목록


quit

    테스트 종료

============================================
"""
        )


    # ========================================================
    # Terminal Main Loop
    # ========================================================

    def run(self):

        # 실제 GPIO18 PWM Service Open.
        if not self.buzzer.open():

            print(
                "[TEST] "
                "Buzzer open 실패"
            )

            print(
                self.buzzer.get_state()
            )

            return 1


        # 시작 상태는 IDLE.
        self.set_mode(
            "IDLE"
        )


        self.print_settings()

        self.print_help()


        try:

            while True:

                try:

                    parts = input(
                        "poco-buzzer-test> "
                    ).strip().split()


                    if not parts:
                        continue


                    command = (
                        parts[0].lower()
                    )


                    # ----------------------------------------
                    # mode
                    # ----------------------------------------
                    if (
                        command == "mode"
                        and len(parts) == 2
                    ):

                        self.set_mode(
                            parts[1]
                        )


                    # ----------------------------------------
                    # pose
                    # ----------------------------------------
                    elif (
                        command == "pose"
                        and len(parts) == 2
                    ):

                        self.send_pose(
                            int(
                                parts[1]
                            )
                        )


                        self.drain_buzzer()


                    # ----------------------------------------
                    # hold
                    # ----------------------------------------
                    elif (
                        command == "hold"
                        and len(parts) == 3
                    ):

                        self.hold(
                            int(
                                parts[1]
                            ),
                            float(
                                parts[2]
                            ),
                        )


                    # ----------------------------------------
                    # set
                    # ----------------------------------------
                    elif (
                        command == "set"
                        and len(parts) == 3
                    ):

                        self.set_runtime(
                            parts[1],
                            parts[2],
                        )


                    # ----------------------------------------
                    # reload
                    # ----------------------------------------
                    elif (
                        command == "reload"
                        and len(parts) == 1
                    ):

                        self.reload_settings()


                    # ----------------------------------------
                    # settings
                    # ----------------------------------------
                    elif (
                        command == "settings"
                        and len(parts) == 1
                    ):

                        self.print_settings()


                    # ----------------------------------------
                    # status
                    # ----------------------------------------
                    elif (
                        command == "status"
                        and len(parts) == 1
                    ):

                        self.print_status()


                    # ----------------------------------------
                    # help
                    # ----------------------------------------
                    elif (
                        command == "help"
                        and len(parts) == 1
                    ):

                        self.print_help()


                    # ----------------------------------------
                    # quit
                    # ----------------------------------------
                    elif command in {
                        "quit",
                        "exit",
                    }:

                        break


                    else:

                        print(
                            "[입력 오류] "
                            "help를 입력해 "
                            "명령 형식을 확인하세요."
                        )


                except (
                    ValueError,
                    TypeError,
                ) as error:

                    print(
                        "[입력 오류] "
                        f"{error}"
                    )


                except KeyboardInterrupt:

                    print(
                        "\n[TEST] Ctrl+C"
                    )

                    break


        finally:

            # 종료할 때 GPIO/PWM을
            # 반드시 안전하게 닫는다.
            self.buzzer.close()


        return 0


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":

    raise SystemExit(
        BuzzerTerminalTest().run()
    )
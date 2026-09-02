"""
POCO Posture Alert Service.

파이프라인 위치
---------------
Pose Process
    ↓
POSE_STATE["inference"]["pose_index"]
    ↓
Hardware Process
    ↓
PostureAlertService
    ↓
Alert Command
    ↓
BuzzerService
    ↓
Raspberry Pi GPIO18 PWM
    ↓
Passive Buzzer


역할
----
이 파일은 '자세 알림 판단 로직'을
현재 Multiprocessing 구조에 맞게 분리한 Service이다.

이 Service는 실제 GPIO나 Buzzer를 제어하지 않는다.

담당하는 기능:
- AI pose_index -> 자세 command 변환
- 나쁜 자세 유지시간 판단
- 같은 자세가 계속될 경우 일정 시간마다 일반 경고 발생
- 같은 경고가 연속으로 발생하면 StrongAlert로 승격
- StrongAlert 이후 해당 자세만 Cooldown 적용
- PyQt AlarmSettings 값 즉시 반영

담당하지 않는 기능:
- GPIO 제어
- PWM 생성
- 실제 부저 ON/OFF
- Face / Drowsy 판단
- Motor 제어


기존 POCO 정책 유지
-------------------
Pose class:

    0 -> Optimal
    1 -> Asymmetric
    2 -> ForwardHead
    3 -> ChinPropping

예:
    bad_posture_duration_sec = 5

    ForwardHead 시작
        ↓
    5초 유지
        ↓
    ForwardHead 일반 경고
        ↓
    같은 자세 계속 유지
        ↓
    다시 5초
        ↓
    ForwardHead 일반 경고
        ↓
    posture_Strong_limit 도달
        ↓
    StrongAlert
        ↓
    해당 자세만 Cooldown


중요
----
Optimal이 감지되면:
- bad posture 유지시간은 초기화
- 현재 bad posture 종류는 초기화

하지만:
- last_alert_command
- continuous_alert_counts

는 Optimal만으로 초기화하지 않는다.

이 부분은 동작을 임의로 변경하지 않기 위한 의도적인 유지이다.
"""

import time


# ============================================================
# Pose Class -> POCO Alert Command
# ============================================================

POSE_COMMANDS = {
    0: "Optimal",
    1: "Asymmetric",
    2: "ForwardHead",
    3: "ChinPropping",
}


class PostureAlertService:
    """
    POCO 자세 경고 판단 Service.

    실제 부저 출력은 BuzzerService가 담당하고,
    이 클래스는 언제 어떤 Alert command를 발생시킬지만 판단한다.
    """

    def __init__(
        self,
        enabled=True,
        posture_hold_seconds=5,
        posture_strong_alert_limit=3,
        alert_cooldown_minutes=5,
    ):
        # ====================================================
        # PyQt Alarm Settings
        # ====================================================

        self.enabled = bool(enabled)

        self.posture_hold_seconds = self._clamp_int(
            posture_hold_seconds,
            min_value=1,
            max_value=10,
            default_value=5,
        )

        self.posture_strong_alert_limit = self._clamp_int(
            posture_strong_alert_limit,
            min_value=1,
            max_value=5,
            default_value=3,
        )

        self.alert_cooldown_minutes = self._clamp_int(
            alert_cooldown_minutes,
            min_value=1,
            max_value=5,
            default_value=5,
        )

        # ====================================================
        # get_posture_result_from_ai() 상태
        # ====================================================

        # Alert 대상으로 처리한 자세 index.
        self.last_sent_idx = None

        # 현재 나쁜 자세가 시작된 시간.
        self.bad_start_time = None

        # 현재 유지시간을 측정 중인 나쁜 자세 index.
        self.current_bad_idx = None

        # ====================================================
        # process_alert_with_cooldown() 상태
        # ====================================================

        # 마지막으로 일반 Alert 카운트에 사용된 command.
        #
        # 예:
        # "ForwardHead"
        self.last_alert_command = None

        # Alert 종류별 연속 반복 횟수.
        #
        # 예:
        # {
        #     "ForwardHead": 2,
        #     "ChinPropping": 1,
        # }
        self.continuous_alert_counts = {}

        # StrongAlert 발생 후 자세별 Cooldown 종료시간.
        #
        # monotonic clock을 사용하므로 시스템 시간 변경의
        # 영향을 받지 않는다.
        #
        # 예:
        # {
        #     "ForwardHead": 12345.67
        # }
        self.cooldown_until = {}

        # 마지막으로 반환된 실제 Alert command.
        # Hardware state / 디버깅 확인용이다.
        self.last_output_command = None

    # ========================================================
    # Settings
    # ========================================================
    def apply_settings(self, settings):
        """
        PyQt AlarmSettings 또는 dict 값을 Runtime에 반영한다.

        지원 입력
        --------
        AlarmSettings 객체

        또는

        {
            "alarm_enabled": True,
            "bad_posture_duration_sec": 5,
            "posture_Strong_limit": 3,
            "strong_alert_cooldown_min": 5,
        }

        주의
        ----
        posture_Hardware_count는 실제 부저 반복 횟수이므로
        BuzzerService에서 사용한다.

        이 Service는 '언제 Alert를 발생시킬지'만 판단하기 때문에
        posture_Hardware_count를 저장하지 않는다.
        """

        if settings is None:
            return False

        if isinstance(settings, dict):
            getter = settings.get
        else:
            getter = lambda name, default=None: getattr(
                settings,
                name,
                default,
            )

        self.enabled = bool(
            getter(
                "alarm_enabled",
                self.enabled,
            )
        )

        self.posture_hold_seconds = self._clamp_int(
            getter(
                "bad_posture_duration_sec",
                self.posture_hold_seconds,
            ),
            min_value=1,
            max_value=10,
            default_value=self.posture_hold_seconds,
        )

        self.posture_strong_alert_limit = self._clamp_int(
            getter(
                "posture_Strong_limit",
                self.posture_strong_alert_limit,
            ),
            min_value=1,
            max_value=5,
            default_value=self.posture_strong_alert_limit,
        )

        self.alert_cooldown_minutes = self._clamp_int(
            getter(
                "strong_alert_cooldown_min",
                self.alert_cooldown_minutes,
            ),
            min_value=1,
            max_value=5,
            default_value=self.alert_cooldown_minutes,
        )

        print(
            "[PostureAlertService] 설정 반영 "
            f"enabled={self.enabled}, "
            f"hold={self.posture_hold_seconds}s, "
            f"strong_limit={self.posture_strong_alert_limit}, "
            f"cooldown={self.alert_cooldown_minutes}min"
        )

        return True

    def set_enabled(self, enabled):
        """
        자세 알림 기능 ON/OFF.

        기존 HardwareController와 동일하게
        OFF라고 해서 AI 추론 자체를 중단하지 않는다.

        단지 Alert command를 발생시키지 않는다.
        """

        self.enabled = bool(enabled)

        print(
            "[PostureAlertService] 자세 알림 "
            + ("ON" if self.enabled else "OFF")
        )

    # ========================================================
    # Pose Mapping
    # ========================================================
    @staticmethod
    def convert_class_idx_to_command(class_idx):
        """
        기존 POCO pose_index를 Alert command로 변환한다.

        0 -> Optimal
        1 -> Asymmetric
        2 -> ForwardHead
        3 -> ChinPropping
        """

        try:
            class_idx = int(class_idx)
        except (TypeError, ValueError):
            return None

        return POSE_COMMANDS.get(class_idx)

    # ========================================================
    # Main Posture Update
    # ========================================================
    def update(self, class_idx, now=None):
        """
        현재 Pose AI 결과 하나를 처리한다.

        Parameters
        ----------
        class_idx:
            Pose Process에서 전달한 pose_index.

            0: Optimal
            1: Asymmetric
            2: ForwardHead
            3: ChinPropping

        now:
            테스트용 monotonic timestamp.
            실제 Runtime에서는 생략한다.

        Returns
        -------
        str | None

        가능한 결과:

            None
            "Optimal"
            "Asymmetric"
            "ForwardHead"
            "ChinPropping"
            "StrongAlert"
        """

        if not self.enabled:
            return None

        try:
            class_idx = int(class_idx)
        except (TypeError, ValueError):
            return None

        # 정의되지 않은 Pose class는 무시한다.
        if class_idx not in POSE_COMMANDS:
            return None

        if now is None:
            now = time.monotonic()

        # ====================================================
        # 0: Optimal
        # ====================================================
        if class_idx == 0:
            # 기존 POCO 코드 그대로:
            # 나쁜 자세 유지시간 측정 상태만 초기화한다.
            self.bad_start_time = None
            self.current_bad_idx = None

            # Optimal은 상태가 바뀐 경우에만 한 번 반환한다.
            if self.last_sent_idx != class_idx:
                self.last_sent_idx = class_idx
                self.last_output_command = "Optimal"

                return "Optimal"

            return None

        # ====================================================
        # 1~3: Bad Posture
        # ====================================================

        # 새로운 종류의 나쁜 자세가 감지됨.
        #
        # 예:
        # ForwardHead -> ChinPropping
        #
        # 새 자세의 유지시간을 처음부터 측정한다.
        if self.current_bad_idx != class_idx:
            self.current_bad_idx = class_idx
            self.bad_start_time = now

            return None

        # 방어 처리.
        #
        # 정상적으로는 위에서 bad_start_time이 항상 생성된다.
        if self.bad_start_time is None:
            self.bad_start_time = now
            return None

        elapsed_time = (
            now - self.bad_start_time
        )

        # 아직 PyQt에서 설정한 자세 유지시간을 충족하지 않았다.
        if elapsed_time < self.posture_hold_seconds:
            return None

        # ====================================================
        # 자세 유지시간 충족
        # ====================================================

        self.last_sent_idx = class_idx

        # 기존 POCO의 핵심 동작.
        #
        # Alert가 한 번 울린 뒤에도 같은 나쁜 자세가 계속되면
        # 다시 posture_hold_seconds 만큼 시간을 측정한다.
        #
        # 예:
        # ForwardHead 5초
        # -> Alert
        #
        # 계속 ForwardHead
        # -> 다시 5초
        # -> Alert
        self.bad_start_time = now

        command = self.convert_class_idx_to_command(
            class_idx
        )

        alert_command = self._process_alert_with_cooldown(
            command,
            self.posture_strong_alert_limit,
            now,
        )

        if alert_command is not None:
            self.last_output_command = (
                alert_command
            )

        return alert_command

    # ========================================================
    # StrongAlert / Cooldown
    # ========================================================
    def _process_alert_with_cooldown(
        self,
        command,
        strong_limit,
        now,
    ):
        """
        기존 POCO process_alert_with_cooldown() 로직.

        같은 command가 '연속으로 Alert 발생'했을 때만
        StrongAlert로 승격한다.

        예
        --
        strong_limit = 3

        ForwardHead
        ForwardHead
        ForwardHead

        -> 3번째에서 StrongAlert


        다른 Alert가 중간에 들어오면 새로 시작한다.

        ForwardHead
        ChinPropping
        ForwardHead

        -> 마지막 ForwardHead는 다시 1회차


        StrongAlert가 발생하면 해당 command만 Cooldown된다.
        """

        if command is None:
            return None

        # ====================================================
        # 1. 현재 command의 Cooldown 확인
        # ====================================================

        cooldown_end = self.cooldown_until.get(
            command
        )

        if (
            cooldown_end is not None
            and now < cooldown_end
        ):
            return None

        # ====================================================
        # 2. Cooldown 종료
        # ====================================================

        if (
            cooldown_end is not None
            and now >= cooldown_end
        ):
            del self.cooldown_until[command]

            self.continuous_alert_counts[
                command
            ] = 0

            if self.last_alert_command == command:
                self.last_alert_command = None

        # ====================================================
        # 3. 연속 Alert 횟수
        # ====================================================

        if self.last_alert_command == command:

            self.continuous_alert_counts[
                command
            ] = (
                self.continuous_alert_counts.get(
                    command,
                    0,
                )
                + 1
            )

        else:
            # 다른 종류의 Alert가 들어왔으므로
            # 현재 command는 1회차부터 새로 시작한다.
            self.continuous_alert_counts[
                command
            ] = 1

            self.last_alert_command = command

        current_count = (
            self.continuous_alert_counts[
                command
            ]
        )

        # ====================================================
        # 4. StrongAlert
        # ====================================================

        if current_count >= strong_limit:

            self.cooldown_until[
                command
            ] = (
                now
                + (
                    self.alert_cooldown_minutes
                    * 60.0
                )
            )

            # StrongAlert 이후 현재 command의
            # 연속 Alert 카운트를 초기화한다.
            self.continuous_alert_counts[
                command
            ] = 0

            # 기존 코드와 동일하게 마지막 Alert도 초기화.
            self.last_alert_command = None

            return "StrongAlert"

        # 아직 StrongAlert 기준 미달.
        return command

    # ========================================================
    # Measurement Tracking Reset
    # ========================================================
    def reset_tracking(
        self,
        preserve_cooldowns=True,
    ):
        """
        현재 자세 유지시간 및 연속 추적 상태를 초기화한다.

        Hardware Process에서 MEASURING이 끝났을 때 사용할 수 있다.

        기본적으로 StrongAlert Cooldown은 유지한다.

        이 함수는 자동으로 호출되지 않는다.
        실제 호출 시점은 Hardware Process 통합 단계에서 결정한다.
        """

        self.last_sent_idx = None
        self.bad_start_time = None
        self.current_bad_idx = None

        self.last_alert_command = None
        self.continuous_alert_counts.clear()

        self.last_output_command = None

        if not preserve_cooldowns:
            self.cooldown_until.clear()

    # ========================================================
    # Runtime State
    # ========================================================
    def get_state(self, now=None):
        """
        디버깅/HARDWARE_STATE용 현재 자세 Alert 상태.
        """

        if now is None:
            now = time.monotonic()

        bad_elapsed_sec = 0.0

        if self.bad_start_time is not None:
            bad_elapsed_sec = max(
                0.0,
                now - self.bad_start_time,
            )

        cooldown_remaining = {}

        for command, end_time in (
            self.cooldown_until.items()
        ):
            remaining = max(
                0.0,
                end_time - now,
            )

            if remaining > 0.0:
                cooldown_remaining[
                    command
                ] = remaining

        return {
            "enabled": bool(
                self.enabled
            ),

            "posture_hold_seconds": (
                self.posture_hold_seconds
            ),

            "posture_strong_alert_limit": (
                self.posture_strong_alert_limit
            ),

            "alert_cooldown_minutes": (
                self.alert_cooldown_minutes
            ),

            "current_bad_idx": (
                self.current_bad_idx
            ),

            "current_bad_command": (
                self.convert_class_idx_to_command(
                    self.current_bad_idx
                )
                if self.current_bad_idx
                is not None
                else None
            ),

            "bad_elapsed_sec": (
                bad_elapsed_sec
            ),

            "last_sent_idx": (
                self.last_sent_idx
            ),

            "last_alert_command": (
                self.last_alert_command
            ),

            "continuous_alert_counts": dict(
                self.continuous_alert_counts
            ),

            "cooldown_remaining_sec": (
                cooldown_remaining
            ),

            "last_output_command": (
                self.last_output_command
            ),
        }

    # ========================================================
    # Utility
    # ========================================================
    @staticmethod
    def _clamp_int(
        value,
        min_value,
        max_value,
        default_value,
    ):
        try:
            value = int(value)

        except (TypeError, ValueError):
            value = int(
                default_value
            )

        return max(
            min_value,
            min(
                value,
                max_value,
            ),
        )
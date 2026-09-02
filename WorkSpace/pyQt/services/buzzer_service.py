"""
POCO Raspberry Pi Passive Buzzer Service.

파이프라인 위치
---------------
Pose Process
    ↓
Posture Alert 판단
    ↓
Hardware Process
    ↓
BuzzerService
    ↓
Raspberry Pi BCM18 PWM
    ↓
NPN Transistor
    ↓
Passive Buzzer

역할
----
이 파일은 '언제 경고를 울릴지' 판단하지 않는다.

자세 유지시간, StrongAlert 횟수, Cooldown 같은 정책은
기존 POCO 자세 알림 로직에서 처리한다.

BuzzerService는
실제 부저 출력 부분만 Raspberry Pi에서 한다.

부저 패턴
----------------------
일반 자세 경고
- 200 ms ON
- 200 ms OFF
- posture_Hardware_count 만큼 반복

StrongAlert
- 100 ms ON
- 100 ms OFF
- 10회 반복

수동부저이므로 ON 상태에서는
2000 Hz / 50% Duty PWM을 출력한다.

중요
----
Hardware Process 안에서 time.sleep()으로 부저 패턴을 실행하면
IMU / ToF / Motor 제어까지 같이 멈출 수 있다.

따라서 이 Service는 non-blocking 상태머신으로 동작한다.
Hardware Process가 반복적으로 update()를 호출하면
시간이 되었을 때만 PWM ON/OFF 상태를 변경한다.
"""

import glob
import re
import time
from collections import deque

from services.hardware_constants import (
    BUZZER_BCM_PIN,
    BUZZER_DUTY_CYCLE,
    BUZZER_FREQUENCY_HZ,
)


# ============================================================
# 알림 패턴
# ============================================================

NORMAL_ALERT_COMMANDS = {
    "Asymmetric",
    "ForwardHead",
    "ChinPropping",
}

NORMAL_ALERT_ON_SEC = 0.200
NORMAL_ALERT_OFF_SEC = 0.200

STRONG_ALERT_REPEAT_COUNT = 10
STRONG_ALERT_ON_SEC = 0.100
STRONG_ALERT_OFF_SEC = 0.100


def _gpiochip_number(path):
    match = re.search(r"gpiochip(\d+)$", str(path))
    return None if match is None else int(match.group(1))


def _open_user_gpiochip(lgpio_module, pin, device_paths=None):
    """Open the gpiochip that owns the user-facing BCM header GPIOs.

    Raspberry Pi 5 exposes those lines through the RP1 chip.  Its /dev suffix
    is not stable across every kernel build, so selecting gpiochip0/4 by model
    is insufficient.  Inspect each accessible chip and select ``pinctrl-rp1``.
    Older Pi models fall back to another ``pinctrl-*`` chip with this line.
    """
    paths = list(device_paths) if device_paths is not None else glob.glob(
        "/dev/gpiochip*"
    )
    paths.sort(key=lambda path: _gpiochip_number(path) or -1)
    fallback = None
    errors = []

    for path in paths:
        chip = _gpiochip_number(path)
        if chip is None:
            continue
        handle = None
        try:
            handle = lgpio_module.gpiochip_open(chip)
            info = lgpio_module.gpio_get_chip_info(handle)
            line_count = int(info[1])
            label = str(info[3])
        except Exception as error:
            if handle is not None:
                try:
                    lgpio_module.gpiochip_close(handle)
                except Exception:
                    pass
            errors.append(f"gpiochip{chip}: {error}")
            continue

        if label == "pinctrl-rp1" and line_count > int(pin):
            if fallback is not None:
                lgpio_module.gpiochip_close(fallback[0])
            return handle, chip, label

        if fallback is None and label.startswith("pinctrl-") and line_count > int(pin):
            fallback = (handle, chip, label)
        else:
            lgpio_module.gpiochip_close(handle)

    if fallback is not None:
        return fallback

    detail = " / ".join(errors) if errors else "접근 가능한 gpiochip이 없습니다."
    raise RuntimeError(f"사용자 GPIO 칩을 찾지 못했습니다: {detail}")


class _LGPIOPWMDevice:
    """Small value-compatible adapter around lgpio software PWM."""

    def __init__(self, lgpio_module, handle, chip, label, pin, frequency_hz):
        self._lgpio = lgpio_module
        self._handle = handle
        self.chip = int(chip)
        self.label = str(label)
        self.pin = int(pin)
        self.frequency_hz = int(frequency_hz)
        self._value = 0.0
        self._closed = False
        # tx_pwm() 전에 명시적으로 LOW output으로 claim해야 Pi 5/RP1에서
        # PWM 정지 시 'bad PWM micros' 없이 안정적으로 Duty 0%를 적용한다.
        self._lgpio.gpio_claim_output(self._handle, self.pin, 0)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, requested):
        if self._closed:
            raise RuntimeError("GPIO PWM 장치가 닫혀 있습니다.")
        value = max(0.0, min(float(requested), 1.0))
        if value > 0.0:
            self._lgpio.tx_pwm(
                self._handle,
                self.pin,
                self.frequency_hz,
                value * 100.0,
            )
        else:
            self._lgpio.tx_pwm(
                self._handle,
                self.pin,
                self.frequency_hz,
                0,
            )
        self._value = value

    def close(self):
        if self._closed:
            return
        try:
            self.value = 0.0
        finally:
            self._closed = True
            try:
                self._lgpio.gpio_free(self._handle, self.pin)
            finally:
                self._lgpio.gpiochip_close(self._handle)


class BuzzerService:
    """
    Raspberry Pi 수동부저 PWM 출력 Service.

    GPIO/PWM만 담당한다.

    자세 판단이나 StrongAlert 판단은 이 클래스에서 하지 않는다.
    """

    def __init__(
        self,
        pin=BUZZER_BCM_PIN,
        frequency_hz=BUZZER_FREQUENCY_HZ,
        duty_cycle=BUZZER_DUTY_CYCLE,
    ):
        self.pin = int(pin)
        self.frequency_hz = int(frequency_hz)
        self.duty_cycle = float(duty_cycle)

        # lgpio PWM adapter. Raspberry Pi가 아닌 개발 PC에서도 import
        # 단계에서 전체 Hardware Process가 죽지 않도록 open()에서 생성한다.
        self._device = None
        self.gpiochip = None
        self.gpiochip_label = None

        # ----------------------------------------------------
        # Hardware 상태
        # ----------------------------------------------------
        self.available = False
        self.last_error = None

        # ----------------------------------------------------
        # 현재 부저 Pattern 상태
        # ----------------------------------------------------
        self._phase = "IDLE"

        self._active_command = None
        self._remaining_count = 0

        self._on_sec = 0.0
        self._off_sec = 0.0

        self._phase_deadline = 0.0

        self._output_on = False

        # ----------------------------------------------------
        # Arduino Serial 동작 호환용 Pending Pattern
        # ----------------------------------------------------
        # 예전에는 Raspberry Pi가 Arduino로 command를 보내면
        # Arduino가 이전 blinkFeedback()을 실행하는 동안
        # 다음 Serial command가 순서대로 대기할 수 있었다.
        #
        # Raspberry Pi 단독 구조에서도 이미 발생한 알림을
        # 임의로 버리지 않도록 같은 순서로 보관한다.
        self._pending_patterns = deque()

    # ========================================================
    # Hardware Open
    # ========================================================
    def open(self):
        """
        Raspberry Pi GPIO18 PWM Device를 연다.

        Raspberry Pi 5에서는 실제 ``pinctrl-rp1`` gpiochip을 찾아 lgpio로 연다.

        GPIO 초기화에 실패하더라도 예외를 Hardware Process 밖으로
        던지지 않고 False를 반환한다.
        """

        if self.available and self._device is not None:
            return True

        handle = None
        try:
            # Raspberry Pi가 아닌 PC에서 이 모듈을 import해도 바로 실패하지
            # 않도록 실제 GPIO import와 장치 탐색은 여기서 수행한다.
            import lgpio

            handle, chip, label = _open_user_gpiochip(lgpio, self.pin)
            self._device = _LGPIOPWMDevice(
                lgpio,
                handle,
                chip,
                label,
                self.pin,
                self.frequency_hz,
            )
            self.gpiochip = chip
            self.gpiochip_label = label

            self.available = True
            self.last_error = None

            self._phase = "IDLE"
            self._output_on = False

            print(
                "[BuzzerService] 준비 완료 "
                f"(gpiochip{chip}:{label}, BCM{self.pin}, "
                f"{self.frequency_hz}Hz, "
                f"Duty={self.duty_cycle:.2f})"
            )

            return True

        except Exception as error:
            if handle is not None and self._device is None:
                try:
                    lgpio.gpiochip_close(handle)
                except Exception:
                    pass
            self._device = None
            self.gpiochip = None
            self.gpiochip_label = None
            self.available = False
            self.last_error = str(error)

            print(
                "[BuzzerService] GPIO/PWM 초기화 실패: "
                f"{error}"
            )

            return False

    # ========================================================
    # Alert Command
    # ========================================================
    def play_command(
        self,
        command,
        posture_count,
    ):
        """
        기존 POCO Alert command를 실제 부저 Pattern으로 변환한다.

        Parameters
        ----------
        command:
            "Optimal"
            "Asymmetric"
            "ForwardHead"
            "ChinPropping"
            "StrongAlert"

        posture_count:
            PyQt의 posture_Hardware_count 값.
            일반 자세 경고 반복 횟수에 사용한다.

        Returns
        -------
        bool
            Pattern 요청이 정상적으로 처리되면 True.
        """

        if not self.available or self._device is None:
            return False

        if command is None:
            return False

        command = str(command)

        # ----------------------------------------------------
        # Optimal
        # ----------------------------------------------------
        # 기존 Arduino에서도 Optimal은 부저를 울리지 않았다.
        #
        # 현재 실행 중인 기존 경고까지 강제로 끄지는 않는다.
        # 이미 시작된 Arduino 경고가 끝까지 실행되던 기존 동작을
        # 그대로 유지하기 위함이다.
        if command == "Optimal":
            return True

        # ----------------------------------------------------
        # 일반 자세 경고
        # ----------------------------------------------------
        if command in NORMAL_ALERT_COMMANDS:
            try:
                repeat_count = int(posture_count)
            except (TypeError, ValueError):
                repeat_count = 1

            # 현재 AlarmSettings UI 범위도 1~5이지만,
            # Service에서도 한 번 더 안전하게 제한한다.
            repeat_count = max(
                1,
                min(repeat_count, 5),
            )

            pattern = {
                "command": command,
                "repeat_count": repeat_count,
                "on_sec": NORMAL_ALERT_ON_SEC,
                "off_sec": NORMAL_ALERT_OFF_SEC,
            }

        # ----------------------------------------------------
        # StrongAlert
        # ----------------------------------------------------
        elif command == "StrongAlert":
            pattern = {
                "command": command,
                "repeat_count": STRONG_ALERT_REPEAT_COUNT,
                "on_sec": STRONG_ALERT_ON_SEC,
                "off_sec": STRONG_ALERT_OFF_SEC,
            }

        else:
            self.last_error = (
                f"지원하지 않는 Buzzer command: {command}"
            )
            return False

        # 발생 순서를 유지한다.
        self._pending_patterns.append(pattern)

        # 현재 아무 Pattern도 실행 중이지 않으면 즉시 시작한다.
        if self._phase == "IDLE":
            return self._start_next_pattern(
                time.monotonic()
            )

        return True

    # ========================================================
    # Non-blocking Update
    # ========================================================
    def update(self, now=None):
        """
        현재 부저 Pattern을 한 단계 진행한다.

        Hardware Process main loop에서 계속 호출해야 한다.

        이 함수 내부에서는 sleep()을 사용하지 않는다.
        """

        if not self.available or self._device is None:
            return self.get_state()

        if now is None:
            now = time.monotonic()

        # Pattern이 끝났지만 Pending Pattern이 남아 있다면
        # 다음 Pattern을 시작한다.
        if self._phase == "IDLE":
            if self._pending_patterns:
                self._start_next_pattern(now)

            return self.get_state()

        # 아직 현재 ON/OFF 구간 시간이 끝나지 않았다.
        if now < self._phase_deadline:
            return self.get_state()

        try:
            # =================================================
            # ON -> OFF
            # =================================================
            if self._phase == "ON":
                self._set_output(False)

                # 한 번의 beep ON이 완료되었으므로
                # 남은 반복 횟수를 1 감소시킨다.
                self._remaining_count -= 1

                self._phase = "OFF"
                self._phase_deadline = (
                    now + self._off_sec
                )

            # =================================================
            # OFF -> 다음 ON 또는 Pattern 종료
            # =================================================
            elif self._phase == "OFF":

                # 마지막 OFF 구간까지 완료.
                if self._remaining_count <= 0:
                    self._finish_current_pattern()

                    # 이미 Pending Alert가 있다면
                    # 기존 Arduino Serial 순서처럼 바로 다음 Pattern 시작.
                    if self._pending_patterns:
                        self._start_next_pattern(now)

                else:
                    self._set_output(True)

                    self._phase = "ON"
                    self._phase_deadline = (
                        now + self._on_sec
                    )

        except Exception as error:
            self.last_error = str(error)

            print(
                "[BuzzerService] PWM 출력 오류: "
                f"{error}"
            )

            # 오류 발생 시 가능한 한 부저를 OFF 상태로 만든다.
            self.stop(clear_pending=True)

        return self.get_state()

    # ========================================================
    # Pattern Internal
    # ========================================================
    def _start_next_pattern(self, now):
        """
        Pending Pattern 중 가장 먼저 요청된 Pattern을 시작한다.

        실제 PWM 출력을 시작하는 순간 GPIO backend 오류가 발생해도
        Hardware Process 전체로 예외가 전파되지 않도록
        이 함수 내부에서 실패를 처리한다.
        """

        if not self._pending_patterns:
            return False

        pattern = self._pending_patterns.popleft()

        self._active_command = (
            pattern["command"]
        )

        self._remaining_count = int(
            pattern["repeat_count"]
        )

        self._on_sec = float(
            pattern["on_sec"]
        )

        self._off_sec = float(
            pattern["off_sec"]
        )

        try:
            # Pattern은 항상 ON부터 시작한다.
            self._set_output(True)

        except Exception as error:
            self.last_error = str(error)

            print(
                "[BuzzerService] PWM 시작 오류: "
                f"{error}"
            )

            # GPIO 오류가 발생하면 현재 Pattern과
            # 아직 실행되지 않은 Pending Pattern을 안전하게 제거한다.
            self.stop(
                clear_pending=True
            )

            return False

        self._phase = "ON"
        self._phase_deadline = (
            now + self._on_sec
        )

        print(
            "[BuzzerService] Alert 시작 "
            f"command={self._active_command}, "
            f"repeat={self._remaining_count}"
        )

        return True

    def _finish_current_pattern(self):
        """
        현재 Pattern 실행 상태를 초기화한다.
        """

        self._set_output(False)

        print(
            "[BuzzerService] Alert 완료 "
            f"command={self._active_command}"
        )

        self._phase = "IDLE"
        self._active_command = None

        self._remaining_count = 0

        self._on_sec = 0.0
        self._off_sec = 0.0

        self._phase_deadline = 0.0

    # ========================================================
    # GPIO Output
    # ========================================================
    def _set_output(self, enabled):
        """
        Passive Buzzer 실제 PWM ON/OFF.

        enabled=True
            GPIO18 -> 2000 Hz / 50% Duty PWM
            -> NPN ON/OFF 반복
            -> 부저 발음

        enabled=False
            Duty 0%
            -> NPN OFF
            -> 부저 정지
        """

        if self._device is None:
            return

        if enabled:
            self._device.value = (
                self.duty_cycle
            )
            self._output_on = True

        else:
            self._device.value = 0.0
            self._output_on = False

    # ========================================================
    # Stop / Close
    # ========================================================
    def stop(self, clear_pending=True):
        """
        현재 부저 출력을 즉시 OFF한다.

        clear_pending=True이면
        아직 실행되지 않은 Alert Pattern도 모두 제거한다.
        """

        try:
            if self._device is not None:
                self._device.value = 0.0

        except Exception:
            pass

        self._output_on = False

        self._phase = "IDLE"
        self._active_command = None

        self._remaining_count = 0

        self._on_sec = 0.0
        self._off_sec = 0.0

        self._phase_deadline = 0.0

        if clear_pending:
            self._pending_patterns.clear()

    def close(self):
        """
        Hardware Process 종료 시 GPIO/PWM Resource를 정리한다.
        """

        self.stop(clear_pending=True)

        device = self._device
        self._device = None

        if device is not None:
            try:
                device.close()
            except Exception:
                pass

        self.available = False
        self.gpiochip = None
        self.gpiochip_label = None

        print("[BuzzerService] 종료")

    # ========================================================
    # State
    # ========================================================
    def get_state(self):
        """
        HARDWARE_STATE에 넣을 수 있는 현재 Buzzer 상태.
        """

        return {
            "available": bool(
                self.available
            ),
            "active": bool(
                self._output_on
            ),
            "busy": bool(
                self._phase != "IDLE"
                or self._pending_patterns
            ),
            "pin": self.pin,
            "gpiochip": self.gpiochip,
            "gpiochip_label": self.gpiochip_label,
            "frequency_hz": self.frequency_hz,
            "duty_cycle": self.duty_cycle,
            "phase": self._phase,
            "active_command": self._active_command,
            "remaining_count": self._remaining_count,
            "pending_count": len(
                self._pending_patterns
            ),
            "last_error": self.last_error,
        }

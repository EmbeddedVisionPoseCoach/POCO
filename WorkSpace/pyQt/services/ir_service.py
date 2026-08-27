import time

from services.hardware_constants import (
    IR_ACTIVE_LOW,
    IR_LOST_GRACE_SEC,
    IR_PIN,
    IR_SAMPLE_HZ,
    IR_STABLE_DETECT_SEC,
)


class IRSensorService:
    """GPIO 적외선 센서 상태 관리.

    Runtime 값은 JSON에 쓰지 않는다.
    Hardware Process가 latest_state를 메모리에 유지하고 Main/Pose/Face 최신 State IPC로 배포한다.
    """

    def __init__(
        self,
        pin=IR_PIN,
        active_low=IR_ACTIVE_LOW,
        sample_hz=IR_SAMPLE_HZ,
        stable_detect_sec=IR_STABLE_DETECT_SEC,
        lost_grace_sec=IR_LOST_GRACE_SEC,
        gpio_module=None,
    ):
        self.pin = int(pin)
        self.active_low = bool(active_low)
        self.sample_hz = float(sample_hz)
        self.sample_interval = 1.0 / max(self.sample_hz, 1.0)
        self.stable_detect_sec = max(0.0, float(stable_detect_sec))
        self.lost_grace_sec = max(0.0, float(lost_grace_sec))
        self.gpio = gpio_module

        self.available = False
        self.last_error = None
        self.detected_since = None
        self.lost_since = None
        self.latest_state = self._empty_state()

    def _empty_state(self):
        return {
            "available": False,
            "pin": self.pin,
            "active_low": self.active_low,
            "raw_value": None,
            "detected": False,
            "stable_detected": False,
            "detected_duration_sec": 0.0,
            "lost_duration_sec": 0.0,
            "last_error": None,
            "timestamp": time.time(),
        }

    def apply_config(self, config):
        config = config if isinstance(config, dict) else {}
        new_pin = int(config.get("pin", self.pin))
        pin_changed = new_pin != self.pin
        was_available = self.available

        if pin_changed and was_available:
            self.close()

        self.pin = new_pin
        self.active_low = bool(config.get("active_low", self.active_low))
        self.sample_hz = max(1.0, float(config.get("sample_hz", self.sample_hz)))
        self.sample_interval = 1.0 / self.sample_hz
        self.stable_detect_sec = max(
            0.0, float(config.get("stable_detect_sec", self.stable_detect_sec))
        )
        self.lost_grace_sec = max(
            0.0, float(config.get("lost_grace_sec", self.lost_grace_sec))
        )
        self.reset_stability()

        if pin_changed and was_available:
            return self.open()
        return True

    def open(self):
        try:
            if self.gpio is None:
                import RPi.GPIO as GPIO
                self.gpio = GPIO

            self.gpio.setwarnings(False)
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setup(self.pin, self.gpio.IN)

            self.available = True
            self.last_error = None
            self.reset_stability()
            print(
                f"[IR] 준비 완료 BCM={self.pin} "
                f"active={'LOW' if self.active_low else 'HIGH'}"
            )
            return True
        except Exception as error:
            self.available = False
            self.last_error = str(error)
            self.latest_state = self._empty_state()
            self.latest_state["last_error"] = self.last_error
            print(f"[IR] 초기화 실패: {error}")
            return False

    def close(self):
        gpio = self.gpio
        old_pin = self.pin
        self.available = False
        if gpio is not None:
            try:
                gpio.cleanup(old_pin)
            except Exception:
                pass

    def reset_stability(self):
        self.detected_since = None
        self.lost_since = None

    def update(self):
        if not self.available or self.gpio is None:
            state = self._empty_state()
            state["last_error"] = self.last_error or "IR unavailable"
            self.latest_state = state
            return state

        try:
            raw = int(self.gpio.input(self.pin))
            detected = (
                raw == self.gpio.LOW
                if self.active_low
                else raw == self.gpio.HIGH
            )
            now = time.monotonic()

            if detected:
                if self.detected_since is None:
                    self.detected_since = now
                self.lost_since = None
            else:
                if self.lost_since is None:
                    self.lost_since = now
                self.detected_since = None

            detected_duration = (
                now - self.detected_since if self.detected_since is not None else 0.0
            )
            lost_duration = now - self.lost_since if self.lost_since is not None else 0.0

            state = {
                "available": True,
                "pin": self.pin,
                "active_low": self.active_low,
                "raw_value": raw,
                "detected": bool(detected),
                "stable_detected": bool(
                    detected and detected_duration >= self.stable_detect_sec
                ),
                "detected_duration_sec": detected_duration,
                "lost_duration_sec": lost_duration,
                "last_error": None,
                "timestamp": time.time(),
            }
            self.latest_state = state
            return state

        except Exception as error:
            self.last_error = str(error)
            state = self._empty_state()
            state["last_error"] = self.last_error
            self.latest_state = state
            return state

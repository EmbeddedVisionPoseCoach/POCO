"""HW-843(VL53L0X) ToF 거리 센서 서비스.

실제 하드웨어 import는 ``open()`` 시점까지 미룬다. 따라서 라즈베리파이가
아닌 개발 PC에서도 이 모듈과 단위 테스트를 import할 수 있다.
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable

from services.hardware_constants import (
    TOF_FILTER_ALPHA,
    TOF_I2C_ADDRESS,
    TOF_I2C_BUS,
    TOF_IO_TIMEOUT_SEC,
    TOF_MAX_RANGE_M,
    TOF_MIN_RANGE_M,
    TOF_SAMPLE_HZ,
)


class ToFSensorService:
    """I2C VL53L0X의 거리값을 m 단위 상태로 관리한다.

    ``i2c_factory``와 ``sensor_factory``는 실제 장비 없이 테스트할 때 쓰는
    의존성 주입 지점이다. 일반 실행에서는 둘 다 지정하지 않는다.
    """

    def __init__(
        self,
        bus_number: int = TOF_I2C_BUS,
        address: int = TOF_I2C_ADDRESS,
        sample_hz: float = TOF_SAMPLE_HZ,
        minimum_range_m: float = TOF_MIN_RANGE_M,
        maximum_range_m: float = TOF_MAX_RANGE_M,
        filter_alpha: float = TOF_FILTER_ALPHA,
        io_timeout_sec: float = TOF_IO_TIMEOUT_SEC,
        i2c_factory: Callable[[int], Any] | None = None,
        sensor_factory: Callable[..., Any] | None = None,
    ):
        self.bus_number = int(bus_number)
        self.address = int(address)
        self.sample_hz = max(1.0, float(sample_hz))
        self.sample_interval = 1.0 / self.sample_hz
        self.minimum_range_m = float(minimum_range_m)
        self.maximum_range_m = float(maximum_range_m)
        self.filter_alpha = float(filter_alpha)
        self.io_timeout_sec = max(0.01, float(io_timeout_sec))
        self.i2c_factory = i2c_factory
        self.sensor_factory = sensor_factory

        if self.bus_number < 0:
            raise ValueError("ToF I2C bus 번호는 0 이상이어야 합니다.")
        if not 0 <= self.address <= 0x7F:
            raise ValueError("ToF I2C 주소는 7-bit 범위(0x00~0x7F)여야 합니다.")
        if self.minimum_range_m >= self.maximum_range_m:
            raise ValueError("ToF 최소 측정거리는 최대 측정거리보다 작아야 합니다.")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("ToF filter_alpha는 0 초과 1 이하여야 합니다.")

        self.i2c = None
        self.sensor = None
        self.available = False
        self.last_error: str | None = None
        self.filtered_distance_m: float | None = None
        self.last_sample_monotonic = 0.0
        self.latest_state = self._empty_state()

    @property
    def device_path(self) -> str:
        return f"/dev/i2c-{self.bus_number}"

    def _empty_state(self) -> dict[str, Any]:
        return {
            "available": False,
            "valid": False,
            "bus_number": self.bus_number,
            "device_path": self.device_path,
            "address": self.address,
            "raw_range_mm": None,
            "distance_m": None,
            "filtered_distance_m": None,
            "last_valid_distance_m": self.filtered_distance_m,
            "last_error": None,
            "timestamp": time.time(),
        }

    def apply_config(self, config: dict[str, Any] | None) -> bool:
        """설정을 적용한다. bus/address 변경 시 열린 장치를 다시 연다."""
        config = config if isinstance(config, dict) else {}
        new_bus = int(config.get("i2c_bus", self.bus_number))
        raw_address = config.get("i2c_address", self.address)
        new_address = int(raw_address, 0) if isinstance(raw_address, str) else int(raw_address)
        connection_changed = new_bus != self.bus_number or new_address != self.address
        was_available = self.available
        if connection_changed and was_available:
            self.close()

        self.bus_number = new_bus
        self.address = new_address
        self.sample_hz = max(1.0, float(config.get("sample_hz", self.sample_hz)))
        self.sample_interval = 1.0 / self.sample_hz
        self.minimum_range_m = float(
            config.get("minimum_range_m", self.minimum_range_m)
        )
        self.maximum_range_m = float(
            config.get("maximum_range_m", self.maximum_range_m)
        )
        self.filter_alpha = float(config.get("filter_alpha", self.filter_alpha))
        self.io_timeout_sec = max(
            0.01, float(config.get("io_timeout_sec", self.io_timeout_sec))
        )
        if self.minimum_range_m >= self.maximum_range_m:
            raise ValueError("ToF 최소 측정거리는 최대 측정거리보다 작아야 합니다.")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("ToF filter_alpha는 0 초과 1 이하여야 합니다.")
        self.filtered_distance_m = None
        self.last_sample_monotonic = 0.0

        if connection_changed and was_available:
            return self.open()
        return True

    def open(self) -> bool:
        try:
            if self.i2c_factory is None:
                from adafruit_extended_bus import ExtendedI2C

                self.i2c_factory = ExtendedI2C
            if self.sensor_factory is None:
                import adafruit_vl53l0x

                self.sensor_factory = adafruit_vl53l0x.VL53L0X

            self.i2c = self.i2c_factory(self.bus_number)
            self.sensor = self.sensor_factory(
                self.i2c,
                address=self.address,
                io_timeout_s=self.io_timeout_sec,
            )
            start_continuous = getattr(self.sensor, "start_continuous", None)
            if callable(start_continuous):
                start_continuous()

            self.available = True
            self.last_error = None
            self.filtered_distance_m = None
            self.last_sample_monotonic = 0.0
            print(
                f"[ToF] 준비 완료 {self.device_path} address=0x{self.address:02X}"
            )
            return True
        except Exception as error:
            self.last_error = str(error)
            self.close()
            self.latest_state = self._empty_state()
            self.latest_state["last_error"] = self.last_error
            print(f"[ToF] 초기화 실패: {error}")
            return False

    def close(self) -> None:
        sensor = self.sensor
        i2c = self.i2c
        self.available = False
        self.sensor = None
        self.i2c = None
        if sensor is not None:
            try:
                stop_continuous = getattr(sensor, "stop_continuous", None)
                if callable(stop_continuous):
                    stop_continuous()
            except Exception:
                pass
        if i2c is not None:
            try:
                deinit = getattr(i2c, "deinit", None)
                if callable(deinit):
                    deinit()
            except Exception:
                pass

    def update(self, force: bool = False) -> dict[str, Any]:
        if not self.available or self.sensor is None:
            state = self._empty_state()
            state["last_error"] = self.last_error or "ToF unavailable"
            self.latest_state = state
            return state

        now_monotonic = time.monotonic()
        if (
            not force
            and self.latest_state.get("valid")
            and now_monotonic - self.last_sample_monotonic < self.sample_interval
        ):
            return self.latest_state

        try:
            raw_range_mm = int(self.sensor.range)
            distance_m = raw_range_mm / 1000.0
            if not math.isfinite(distance_m):
                raise ValueError("ToF 측정값이 유한한 수가 아닙니다.")
            if not self.minimum_range_m <= distance_m <= self.maximum_range_m:
                raise ValueError(
                    f"ToF 거리 {distance_m:.3f}m가 센서 허용범위 "
                    f"{self.minimum_range_m:.3f}~{self.maximum_range_m:.3f}m 밖입니다."
                )

            if self.filtered_distance_m is None:
                self.filtered_distance_m = distance_m
            else:
                alpha = self.filter_alpha
                self.filtered_distance_m = (
                    alpha * distance_m + (1.0 - alpha) * self.filtered_distance_m
                )
            self.last_sample_monotonic = now_monotonic
            self.last_error = None
            state = {
                "available": True,
                "valid": True,
                "bus_number": self.bus_number,
                "device_path": self.device_path,
                "address": self.address,
                "raw_range_mm": raw_range_mm,
                "distance_m": distance_m,
                "filtered_distance_m": self.filtered_distance_m,
                "last_valid_distance_m": self.filtered_distance_m,
                "last_error": None,
                "timestamp": time.time(),
            }
            self.latest_state = state
            return state
        except Exception as error:
            self.last_error = str(error)
            state = self._empty_state()
            state["available"] = True
            state["last_error"] = self.last_error
            self.latest_state = state
            return state

    def read_distance_m(self) -> float:
        state = self.update()
        if not state["valid"] or state["filtered_distance_m"] is None:
            raise ValueError(state["last_error"] or "유효한 ToF 측정값이 없습니다.")
        return float(state["filtered_distance_m"])


class FixedToFSensorService:
    """카메라/IK 개발 PC에서 쓰는 실제 ToF와 동일 인터페이스의 모의 센서."""

    def __init__(self, fixed_range_m: float):
        self.fixed_range_m = float(fixed_range_m)
        self.available = False
        self.last_error = None
        self.latest_state: dict[str, Any] = {}

    def open(self) -> bool:
        self.available = True
        self.last_error = None
        return True

    def close(self) -> None:
        self.available = False

    def update(self, force: bool = False) -> dict[str, Any]:
        del force
        valid = self.available and math.isfinite(self.fixed_range_m)
        self.latest_state = {
            "available": self.available,
            "valid": valid,
            "bus_number": None,
            "device_path": "fixed_stub",
            "address": None,
            "raw_range_mm": round(self.fixed_range_m * 1000) if valid else None,
            "distance_m": self.fixed_range_m if valid else None,
            "filtered_distance_m": self.fixed_range_m if valid else None,
            "last_valid_distance_m": self.fixed_range_m if valid else None,
            "last_error": None if valid else "Fixed ToF unavailable",
            "timestamp": time.time(),
        }
        return self.latest_state

    def read_distance_m(self) -> float:
        state = self.update()
        if not state["valid"]:
            raise ValueError(state["last_error"])
        return float(state["filtered_distance_m"])


def create_tof_service(
    config: dict[str, Any] | None,
    fixed_range_override_m: float | None = None,
) -> ToFSensorService | FixedToFSensorService:
    """설정의 mode에 따라 실제 I2C 센서 또는 모의 센서를 만든다."""
    config = config if isinstance(config, dict) else {}
    if fixed_range_override_m is not None:
        return FixedToFSensorService(fixed_range_override_m)

    mode = str(config.get("mode", "hardware")).strip().lower()
    if mode == "fixed_stub":
        return FixedToFSensorService(float(config["fixed_range_m"]))
    if mode != "hardware":
        raise ValueError("tof.mode는 hardware 또는 fixed_stub여야 합니다.")

    raw_address = config.get("i2c_address", TOF_I2C_ADDRESS)
    address = int(raw_address, 0) if isinstance(raw_address, str) else int(raw_address)
    return ToFSensorService(
        bus_number=int(config.get("i2c_bus", TOF_I2C_BUS)),
        address=address,
        sample_hz=float(config.get("sample_hz", TOF_SAMPLE_HZ)),
        minimum_range_m=float(config.get("minimum_range_m", TOF_MIN_RANGE_M)),
        maximum_range_m=float(config.get("maximum_range_m", TOF_MAX_RANGE_M)),
        filter_alpha=float(config.get("filter_alpha", TOF_FILTER_ALPHA)),
        io_timeout_sec=float(config.get("io_timeout_sec", TOF_IO_TIMEOUT_SEC)),
    )

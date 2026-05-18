import json
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass
class AlarmSettings:
    alarm_enabled: bool = True
    bad_posture_duration_sec: int = 5
    fatigue_duration_sec: int = 5
    posture_Hardware_count: int = 5
    fatigue_Hardware_count: int = 3
    posture_Strong_limit: int = 3
    fatigue_Strong_limit: int = 2
    strong_alert_cooldown_min: int = 5

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        """
        JSON 데이터가 일부 빠져있거나 타입이 이상해도
        기본값으로 안전하게 보정해서 AlarmSettings를 만든다.
        """

        default = cls()

        return cls(
            alarm_enabled=bool(
                data.get("alarm_enabled", default.alarm_enabled)
            ),
            bad_posture_duration_sec=cls._clamp_int(
                data.get("bad_posture_duration_sec", default.bad_posture_duration_sec),
                min_value=1,
                max_value=300,
                default_value=default.bad_posture_duration_sec
            ),
            fatigue_duration_sec=cls._clamp_int(
                data.get("fatigue_duration_sec", default.fatigue_duration_sec),
                min_value=1,
                max_value=300,
                default_value=default.fatigue_duration_sec
            ),

            ## 추가
            posture_Hardware_count=cls._clamp_int(
                data.get("posture_Hardware_count", default.posture_Hardware_count),
                min_value=1,
                max_value=30,
                default_value=default.posture_Hardware_count
            ),
            fatigue_Hardware_count=cls._clamp_int(
                data.get("fatigue_Hardware_count", default.fatigue_Hardware_count),
                min_value=1,
                max_value=30,
                default_value=default.fatigue_Hardware_count
            ),
            posture_Strong_limit=cls._clamp_int(
                data.get("posture_Strong_limit", default.posture_Strong_limit),
                min_value=1,
                max_value=30,
                default_value=default.posture_Strong_limit
            ),
            fatigue_Strong_limit=cls._clamp_int(
                data.get("fatigue_Strong_limit", default.fatigue_Strong_limit),
                min_value=1,
                max_value=30,
                default_value=default.fatigue_Strong_limit
            ),
            strong_alert_cooldown_min=cls._clamp_int(
                data.get("strong_alert_cooldown_min", default.strong_alert_cooldown_min),
                min_value=1,
                max_value=120,
                default_value=default.strong_alert_cooldown_min
            ),
        )

    @staticmethod
    def _clamp_int(value, min_value, max_value, default_value):
        try:
            value = int(value)
        except (TypeError, ValueError):
            value = default_value

        return max(min_value, min(value, max_value))


class SettingsManager:
    def __init__(self, settings_path):
        self.settings_path = Path(settings_path)

    def load(self):
        """
        설정 파일이 없으면 기본값 JSON을 만들고 기본값을 반환한다.
        설정 파일이 있으면 JSON 값을 읽어서 반환한다.
        JSON이 깨져있으면 기본값으로 다시 만든다.
        """

        if not self.settings_path.exists():
            default_settings = AlarmSettings()
            self.save(default_settings)
            return default_settings

        try:
            with open(self.settings_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            return AlarmSettings.from_dict(data)

        except Exception:
            default_settings = AlarmSettings()
            self.save(default_settings)
            return default_settings

    def save(self, settings):
        """
        설정 저장.
        폴더가 없으면 자동 생성한다.
        """

        self.settings_path.parent.mkdir(parents=True, exist_ok=True)

        with open(self.settings_path, "w", encoding="utf-8") as f:
            json.dump(
                settings.to_dict(),
                f,
                ensure_ascii=False,
                indent=4
            )
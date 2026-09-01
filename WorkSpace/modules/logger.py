import os
import csv
import time
from datetime import datetime
from dataclasses import dataclass, asdict, fields


@dataclass
class StudyLog:
    """Streamlit 리포트에서 사용하는 날짜별 측정 로그 구조."""
    timestamp: str
    elapsed_sec: int
    posture_type: str

    # 피로도 기능은 추후 다시 활성화할 예정이므로 컬럼을 유지한다.
    # Face Process가 비활성인 동안에는 Normal / 0.0을 저장한다.
    fatigue_label: str = "Normal"
    fatigue_probability: float = 0.0

    @classmethod
    def get_field_names(cls):
        return [f.name for f in fields(cls)]


class StudyLogger:
    def __init__(self, base_dir="data/session_log"):
        self.base_dir = base_dir
        self.start_time = time.monotonic()
        self.last_saved_elapsed_sec = None
        self._ensure_dir()

    def _ensure_dir(self):
        """저장 폴더가 없으면 생성한다."""
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    def _get_file_path(self):
        """현재 날짜에 맞는 CSV 경로를 반환한다."""
        today = datetime.now().strftime("%Y-%m-%d")
        return os.path.join(self.base_dir, f"posture_log_{today}.csv")

    def save(self, pose_data):
        """
        최신 측정 결과를 날짜별 CSV에 저장한다.

        ResultWorker에서는 새 AI 결과가 들어올 때마다 이 함수만 호출하면 된다.
        Logger 내부에서 elapsed_sec 기준으로 같은 초의 중복 저장을 막기 때문에
        정상적으로 결과가 들어오는 동안 최대 1초에 1row만 기록된다.
        """
        now = time.monotonic()
        elapsed_sec = int(now - self.start_time)

        if elapsed_sec == self.last_saved_elapsed_sec:
            return False

        file_path = self._get_file_path()
        file_exists = os.path.isfile(file_path)

        log_entry = StudyLog(
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            elapsed_sec=elapsed_sec,
            **pose_data
        )

        with open(file_path, mode="a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=StudyLog.get_field_names())

            if not file_exists:
                writer.writeheader()

            writer.writerow(asdict(log_entry))

        self.last_saved_elapsed_sec = elapsed_sec
        return True

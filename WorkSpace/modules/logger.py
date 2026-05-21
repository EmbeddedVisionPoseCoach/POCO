import os
import csv
import time
from datetime import datetime
from dataclasses import dataclass, asdict, fields

@dataclass
class StudyLog:
    """데이터 디테일 이미지 규격에 따른 자료 구조"""
    timestamp: str
    elapsed_sec: int
    posture_type: str
    
    # 얼굴 데이터 (현재 미구현 - 0으로 초기화)
    fatigue_label: str = "Normal"
    fatigue_probability: float = 0.0

    @classmethod
    def get_field_names(cls):
        return [f.name for f in fields(cls)]

class StudyLogger:
    def __init__(self, base_dir="data/session_log"):
        self.base_dir = base_dir
        self.start_time = time.time()
        self._ensure_dir()

    def _ensure_dir(self):
        """저장 폴더가 없으면 생성"""
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    def _get_file_path(self):
        """현재 날짜에 맞는 파일 경로 생성"""
        today = datetime.now().strftime("%Y-%m-%d")
        return os.path.join(self.base_dir, f"posture_log_{today}.csv")

    def save(self, pose_data):
        """데이터 저장 (파일이 없으면 헤더 생성, 있으면 어펜드)"""
        file_path = self._get_file_path()
        file_exists = os.path.isfile(file_path)
        
        # 로그 객체 생성
        log_entry = StudyLog(
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            elapsed_sec=int(time.time() - self.start_time),
            **pose_data
        )

        with open(file_path, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=StudyLog.get_field_names())
            
            # 파일이 새로 생성된 경우에만 헤더 작성
            if not file_exists:
                writer.writeheader()
            
            writer.writerow(asdict(log_entry))
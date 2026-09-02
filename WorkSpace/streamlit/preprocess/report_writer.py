import json
from pathlib import Path
from typing import Any, Dict, Optional


# ------------------------------------------------------------
# 경로 설정
# ------------------------------------------------------------

# 현재 파일:
# WorkSpace/streamlit/preprocess/report_writer.py
#
# parents[2] = WorkSpace
BASE_DIR = Path(__file__).resolve().parents[2]

REPORT_DIR = BASE_DIR / "data" / "reports"
DEFAULT_REPORT_PATH = REPORT_DIR / "report_summary.json"


# ------------------------------------------------------------
# summary_builder import
# ------------------------------------------------------------

try:
    from .data_loader import load_posture_log
    from .summary_builder import build_report_summary
except ImportError:
    from data_loader import load_posture_log
    from summary_builder import build_report_summary


# ------------------------------------------------------------
# JSON 저장/읽기 유틸
# ------------------------------------------------------------

def ensure_report_dir() -> None:
    """
    data/reports 폴더가 없으면 자동 생성한다.
    """
    REPORT_DIR.mkdir(parents=True, exist_ok=True)


def save_report_summary(
    summary: Dict[str, Any],
    output_path: Optional[str | Path] = None
) -> Path:
    """
    report_summary dict를 JSON 파일로 저장한다.

    반환:
    - 저장된 파일 경로
    """

    if output_path is None:
        output_path = DEFAULT_REPORT_PATH

    output_path = Path(output_path)

    # 저장할 폴더가 없으면 생성
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=4)

    return output_path


def load_report_summary(
    report_path: Optional[str | Path] = None
) -> Dict[str, Any]:
    """
    저장된 report_summary.json을 읽어서 dict로 반환한다.

    Streamlit app.py에서 이 함수를 사용하면 된다.
    """

    if report_path is None:
        report_path = DEFAULT_REPORT_PATH

    report_path = Path(report_path)

    if not report_path.exists():
        raise FileNotFoundError(f"리포트 파일을 찾을 수 없습니다: {report_path}")

    with report_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def generate_report_from_csv(
    csv_path: Optional[str | Path] = None,
    output_path: Optional[str | Path] = None
) -> Dict[str, Any]:
    """
    CSV 로그를 읽어서 report_summary.json을 생성한다.

    처리 순서:
    1. posture_log.csv 로드
    2. DataFrame 검증 및 타입 변환
    3. report summary 생성
    4. JSON 파일 저장
    5. summary dict 반환
    """

    df = load_posture_log(csv_path)
    summary = build_report_summary(df)

    saved_path = save_report_summary(
        summary=summary,
        output_path=output_path
    )

    print(f"리포트 저장 완료: {saved_path}")

    return summary


# ------------------------------------------------------------
# 테스트 실행
# ------------------------------------------------------------

if __name__ == "__main__":
    summary = generate_report_from_csv()

    print("--------------------")
    print("리포트 생성 성공")
    print("--------------------")

    print("세션 정보")
    print(summary["session"])

    print("--------------------")
    print("자세 요약")
    print(summary["posture_summary"])

    print("--------------------")
    print("피로도 요약")
    print(summary["fatigue_summary"])
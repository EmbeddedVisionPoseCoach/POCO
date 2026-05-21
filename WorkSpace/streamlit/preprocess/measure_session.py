# measure_session.py

from pathlib import Path
import argparse
import json
import time
from datetime import datetime

import pandas as pd


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session_dir", required=True)
    args = parser.parse_args()

    session_dir = Path(args.session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)

    raw_log_path = session_dir / "raw_log.csv"
    summary_path = session_dir / "summary.json"
    problem_stats_path = session_dir / "problem_stats.json"
    baseline_path = session_dir / "baseline.json"

    baseline = {
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "description": "임시 baseline 데이터",
        "forward_head_ratio_base": 0.12,
        "shoulder_angle_base": 2.5,
        "body_lean_angle_base": 3.0,
    }
    write_json(baseline_path, baseline)

    rows = []
    start_time = datetime.now()

    # 임시 측정: 30초 동안 1초 단위 로그 생성
    for elapsed_sec in range(1, 31):
        now = datetime.now()

        if elapsed_sec % 7 == 0:
            problem_type = "ForwardHead"
            score = 55
        elif elapsed_sec % 5 == 0:
            problem_type = "ShoulderImbalance"
            score = 63
        elif elapsed_sec % 9 == 0:
            problem_type = "BodyLean"
            score = 58
        else:
            problem_type = "Good"
            score = 86

        if score >= 80:
            posture_label = "Good"
        elif score >= 60:
            posture_label = "Warning"
        else:
            posture_label = "Bad"

        rows.append(
            {
                "timestamp": now.strftime("%Y-%m-%d %H:%M:%S"),
                "elapsed_sec": elapsed_sec,
                "posture_score": score,
                "posture_label": posture_label,
                "problem_type": problem_type,
                "good_posture": posture_label == "Good",
                "forward_head_ratio": 0.28 if problem_type == "ForwardHead" else 0.12,
                "shoulder_imbalance_angle": 8.5 if problem_type == "ShoulderImbalance" else 2.5,
                "body_lean_angle": 10.0 if problem_type == "BodyLean" else 3.0,
                "eye_closed_ratio": 0.05,
            }
        )

        df = pd.DataFrame(rows)
        df.to_csv(raw_log_path, index=False, encoding="utf-8-sig")

        time.sleep(1)

    df = pd.DataFrame(rows)

    total_seconds = len(df)
    good_seconds = int(df["good_posture"].sum())
    bad_seconds = int((df["posture_label"] == "Bad").sum())
    average_score = round(float(df["posture_score"].mean()), 1)
    max_score = int(df["posture_score"].max())
    min_score = int(df["posture_score"].min())

    lowest_row = df.loc[df["posture_score"].idxmin()]

    problem_df = df[df["problem_type"] != "Good"]
    main_problem = "None"

    if not problem_df.empty:
        main_problem = problem_df["problem_type"].value_counts().idxmax()

    summary = {
        "session_id": session_dir.name,
        "started_at": start_time.strftime("%Y-%m-%d %H:%M:%S"),
        "ended_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "total_seconds": total_seconds,
        "average_score": average_score,
        "max_score": max_score,
        "min_score": min_score,
        "good_ratio": round(good_seconds / total_seconds * 100, 1),
        "bad_seconds": bad_seconds,
        "main_problem": main_problem,
        "lowest_score_time": str(lowest_row["timestamp"]),
        "ai_feedback": "임시 측정 결과야. 실제 MediaPipe 연결 전까지는 이 구조로 Streamlit과 측정 프로세스 연결을 테스트하면 돼.",
    }

    write_json(summary_path, summary)

    problem_name_map = {
        "ForwardHead": "거북목 / 목 앞으로 나옴",
        "ShoulderImbalance": "어깨 비대칭",
        "BodyLean": "상체 기울어짐",
        "Fatigue": "피로 의심",
    }

    coach_map = {
        "ForwardHead": "턱을 살짝 당기고 모니터와 얼굴 사이 거리를 유지해봐.",
        "ShoulderImbalance": "양쪽 어깨 높이를 맞추고 한쪽으로 기대지 않도록 해봐.",
        "BodyLean": "골반을 의자 깊숙이 넣고 상체 중심을 가운데로 맞춰봐.",
        "Fatigue": "눈을 잠깐 쉬게 하고 먼 곳을 바라봐.",
    }

    problems = []

    for problem_type, count in problem_df["problem_type"].value_counts().items():
        problems.append(
            {
                "problem_type": problem_type,
                "display_name": problem_name_map.get(problem_type, problem_type),
                "seconds": int(count),
                "ratio": round(count / total_seconds * 100, 1),
                "coach": coach_map.get(problem_type, "자세를 다시 정렬해봐."),
            }
        )

    problem_stats = {
        "session_id": session_dir.name,
        "problems": problems,
    }

    write_json(problem_stats_path, problem_stats)


if __name__ == "__main__":
    main()
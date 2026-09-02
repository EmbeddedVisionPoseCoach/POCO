"""Tkinter viewer for POCO performance JSON logs.

Run on Raspberry Pi desktop:
    python multiprocess_performance_viewer.py

Headless quick check:
    python multiprocess_performance_viewer.py --print-latest
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_LOG_ROOT = ROOT_DIR / "data" / "performance"


def _load_json(path):
    path = Path(path)
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _values(data, key):
    if not data:
        return []
    result = []
    for sample in data.get("samples", []):
        value = sample.get(key)
        if isinstance(value, (int, float)):
            result.append(float(value))
    return result


def _avg(data, key):
    values = _values(data, key)
    return statistics.fmean(values) if values else None


def _max(data, key):
    values = _values(data, key)
    return max(values) if values else None


def _last(data, key):
    if not data:
        return None
    for sample in reversed(data.get("samples", [])):
        value = sample.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return None


def _fmt(value, suffix="", digits=2):
    if value is None:
        return "-"
    return f"{value:.{digits}f}{suffix}"


def load_run(run_dir):
    run_dir = Path(run_dir)
    return {
        "run_dir": run_dir,
        "main": _load_json(run_dir / "main_profile.json"),
        "pose": _load_json(run_dir / "pose_profile.json"),
        "summary": _load_json(run_dir / "summary.json"),
    }


def build_summary(run):
    main = run["main"]
    pose = run["pose"]
    saved = run.get("summary") or {}
    core = saved.get("core_metrics", {})
    diag = saved.get("diagnostic_metrics", {})
    counts = saved.get("sample_counts", {})

    # summary.json이 아직 생성되지 않은 구버전 로그도 그대로 열 수 있게
    # 원본 sample JSON에서 계산하는 fallback을 유지한다.
    return {
        "camera_fps": core.get("camera_fps_avg", _avg(main, "camera_fps")),
        "pose_fps": core.get("pose_fps_avg", _avg(pose, "pose_fps")),
        "pose_e2e_ms": core.get("pose_e2e_ms_avg", _avg(pose, "e2e_ms_avg")),
        "main_cpu_percent": core.get("main_cpu_percent_avg", _avg(main, "main_cpu_percent")),
        "pose_cpu_percent": core.get("pose_cpu_percent_avg", _avg(pose, "pose_cpu_percent")),
        "shared_memory_write_ms": diag.get("shared_memory_write_ms_avg", _avg(main, "shared_memory_write_ms_avg")),
        "queue_latency_ms": diag.get("queue_latency_ms_avg", _avg(pose, "queue_latency_ms_avg")),
        "mediapipe_ms": diag.get("mediapipe_ms_avg", _avg(pose, "mediapipe_ms_avg")),
        "pending_max": diag.get("ring_pending_max", _max(pose, "ring_pending")),
        "skip_final": diag.get("ring_skipped_final", _last(pose, "ring_skipped")),
        "overrun_final": diag.get("ring_overrun_final", _last(pose, "ring_overrun")),
        "main_samples": counts.get("main", len((main or {}).get("samples", []))),
        "pose_samples": counts.get("pose", len((pose or {}).get("samples", []))),
    }


def find_runs(log_root=DEFAULT_LOG_ROOT):
    log_root = Path(log_root)
    if not log_root.exists():
        return []
    runs = [p for p in log_root.iterdir() if p.is_dir() and p.name.startswith("multiprocess_")]
    return sorted(runs, key=lambda p: p.name, reverse=True)


def print_summary(run_dir):
    run = load_run(run_dir)
    s = build_summary(run)
    print(f"Run: {run_dir}")
    print(f"Camera FPS           : {_fmt(s['camera_fps'])}")
    print(f"Pose FPS             : {_fmt(s['pose_fps'])}")
    print(f"Pose E2E             : {_fmt(s['pose_e2e_ms'], ' ms')}")
    print(f"Main CPU             : {_fmt(s['main_cpu_percent'], ' %')}")
    print(f"Pose CPU             : {_fmt(s['pose_cpu_percent'], ' %')}")
    print(f"SharedMemory Write   : {_fmt(s['shared_memory_write_ms'], ' ms')}")
    print(f"Pose Queue Latency   : {_fmt(s['queue_latency_ms'], ' ms')}")
    print(f"Pose MediaPipe       : {_fmt(s['mediapipe_ms'], ' ms')}")
    print(f"Pending Max          : {_fmt(s['pending_max'], '', 0)}")
    print(f"Skip Final           : {_fmt(s['skip_final'], '', 0)}")
    print(f"Overrun Final        : {_fmt(s['overrun_final'], '', 0)}")


def launch_gui(log_root=DEFAULT_LOG_ROOT):
    import tkinter as tk
    from tkinter import filedialog, ttk

    root = tk.Tk()
    root.title("POCO Multi-Process Performance Viewer")
    root.geometry("1080x720")
    root.minsize(920, 620)

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass

    top = ttk.Frame(root, padding=12)
    top.pack(fill="x")
    ttk.Label(top, text="POCO 멀티프로세스 성능 평가", font=("Sans", 18, "bold")).pack(side="left")

    select_frame = ttk.Frame(root, padding=(12, 0, 12, 8))
    select_frame.pack(fill="x")
    run_var = tk.StringVar()
    combo = ttk.Combobox(select_frame, textvariable=run_var, state="readonly", width=52)
    combo.pack(side="left", padx=(0, 8))

    cards_frame = ttk.Frame(root, padding=(12, 6))
    cards_frame.pack(fill="x")
    card_vars = {}
    card_specs = [
        ("camera_fps", "Camera FPS"),
        ("pose_fps", "Pose FPS"),
        ("pose_e2e_ms", "자세 분석 완료 시간"),
        ("main_cpu_percent", "Main CPU"),
        ("pose_cpu_percent", "Pose CPU"),
    ]
    for index, (key, title) in enumerate(card_specs):
        card = ttk.LabelFrame(cards_frame, text=title, padding=12)
        card.grid(row=0, column=index, sticky="nsew", padx=4)
        cards_frame.columnconfigure(index, weight=1)
        var = tk.StringVar(value="-")
        card_vars[key] = var
        ttk.Label(card, textvariable=var, font=("Sans", 19, "bold")).pack()

    table_frame = ttk.Frame(root, padding=12)
    table_frame.pack(fill="both", expand=True)
    tree = ttk.Treeview(table_frame, columns=("metric", "value", "meaning"), show="headings", height=13)
    tree.heading("metric", text="평가 항목")
    tree.heading("value", text="측정값")
    tree.heading("meaning", text="확인 목적")
    tree.column("metric", width=220, anchor="w")
    tree.column("value", width=180, anchor="center")
    tree.column("meaning", width=560, anchor="w")
    tree.pack(fill="both", expand=True, side="left")
    scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=tree.yview)
    scrollbar.pack(side="right", fill="y")
    tree.configure(yscrollcommand=scrollbar.set)

    footer_var = tk.StringVar(value="JSON 로그를 선택하세요.")
    ttk.Label(root, textvariable=footer_var, padding=(12, 0, 12, 12)).pack(fill="x")

    state = {"runs": []}

    def refresh_runs(select_latest=True):
        state["runs"] = find_runs(log_root)
        combo["values"] = [p.name for p in state["runs"]]
        if state["runs"] and select_latest:
            combo.current(0)
            show_selected()

    def render(run_dir):
        run = load_run(run_dir)
        s = build_summary(run)
        card_vars["camera_fps"].set(_fmt(s["camera_fps"], " FPS"))
        card_vars["pose_fps"].set(_fmt(s["pose_fps"], " FPS"))
        card_vars["pose_e2e_ms"].set(_fmt(s["pose_e2e_ms"], " ms"))
        card_vars["main_cpu_percent"].set(_fmt(s["main_cpu_percent"], " %"))
        card_vars["pose_cpu_percent"].set(_fmt(s["pose_cpu_percent"], " %"))

        for item in tree.get_children():
            tree.delete(item)

        rows = [
            ("Camera 입력 FPS", _fmt(s["camera_fps"], " FPS"), "Main에서 카메라 Frame을 안정적으로 공급하는지"),
            ("Pose 처리 FPS", _fmt(s["pose_fps"], " FPS"), "Pose Process가 초당 실제 처리한 Frame 수"),
            ("Frame → Pose 완료(E2E)", _fmt(s["pose_e2e_ms"], " ms"), "카메라 Frame 생성부터 Pose 처리가 끝날 때까지의 응답시간"),
            ("Main Process CPU", _fmt(s["main_cpu_percent"], " %"), "카메라/UI/IPC를 포함한 Main Process CPU 사용량"),
            ("Pose Process CPU", _fmt(s["pose_cpu_percent"], " %"), "Pose 추론 Process CPU 사용량"),
            ("Shared Memory Write", _fmt(s["shared_memory_write_ms"], " ms"), "Main → Pose Frame 전달 비용"),
            ("Pose Frame 대기시간", _fmt(s["queue_latency_ms"], " ms"), "Frame 생성 후 Pose 처리가 시작될 때까지 기다린 시간"),
            ("MediaPipe Pose 시간", _fmt(s["mediapipe_ms"], " ms"), "Pose landmark 추론 자체의 처리시간"),
            ("대기 Frame 최대", _fmt(s["pending_max"], " frame", 0), "Ring Buffer 적체 정도"),
            ("Skip 누적", _fmt(s["skip_final"], " frame", 0), "Latest Frame 정책으로 의도적으로 건너뛴 Frame"),
            ("Overrun 누적", _fmt(s["overrun_final"], " frame", 0), "Ring이 가득 차 기록하지 못한 Frame"),
        ]
        for row in rows:
            tree.insert("", "end", values=row)
        footer_var.set(
            f"{run_dir}  |  Main samples={s['main_samples']}  Pose samples={s['pose_samples']}  "
            f"(각 sample은 약 2초 구간 평균)"
        )

    def show_selected(event=None):
        name = run_var.get()
        for run_dir in state["runs"]:
            if run_dir.name == name:
                render(run_dir)
                return

    def choose_folder():
        chosen = filedialog.askdirectory(initialdir=str(log_root), title="성능 로그 폴더 선택")
        if chosen:
            render(Path(chosen))

    ttk.Button(select_frame, text="새로고침", command=lambda: refresh_runs(True)).pack(side="left", padx=4)
    ttk.Button(select_frame, text="폴더 열기", command=choose_folder).pack(side="left", padx=4)
    combo.bind("<<ComboboxSelected>>", show_selected)

    refresh_runs(True)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--print-latest", action="store_true")
    parser.add_argument("--run", type=Path)
    args = parser.parse_args()

    if args.run:
        print_summary(args.run)
        return

    if args.print_latest:
        runs = find_runs(args.log_root)
        if not runs:
            print(f"성능 로그가 없습니다: {args.log_root}")
            return
        print_summary(runs[0])
        return

    launch_gui(args.log_root)


if __name__ == "__main__":
    main()

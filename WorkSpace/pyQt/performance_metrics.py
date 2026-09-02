"""POCO performance benchmark JSON storage utilities.

This module is intentionally dependency-free.  It stores multi-process
benchmark samples separately from normal application/session logs so the same
schema can later be reused by the legacy single-process benchmark.
"""

from __future__ import annotations

import json
import os
import statistics
import time
from datetime import datetime
from pathlib import Path


class ProcessCpuSampler:
    """Estimate process CPU usage for one profiling window.

    100% means roughly one logical CPU core was fully occupied during the
    window.  The same method can be used by the future single-process test.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.wall_start = time.perf_counter()
        self.cpu_start = time.process_time()

    def percent(self, wall_end=None, cpu_end=None):
        wall_end = time.perf_counter() if wall_end is None else float(wall_end)
        cpu_end = time.process_time() if cpu_end is None else float(cpu_end)
        wall = max(wall_end - self.wall_start, 1e-9)
        cpu = max(cpu_end - self.cpu_start, 0.0)
        return (cpu / wall) * 100.0


class JsonPerformanceLogger:
    """Append profiler snapshots and atomically persist them as JSON."""

    def __init__(
        self,
        output_path,
        *,
        session_id,
        architecture,
        component,
        metadata=None,
    ):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.data = {
            "schema_version": 2,
            "session_id": str(session_id),
            "architecture": str(architecture),
            "component": str(component),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "updated_at": None,
            "metadata": dict(metadata or {}),
            "samples": [],
        }
        self._write()

    def append(self, sample):
        row = dict(sample)
        row.setdefault("recorded_at", datetime.now().isoformat(timespec="milliseconds"))
        row.setdefault("sample_index", len(self.data["samples"]))
        self.data["samples"].append(row)
        self.data["updated_at"] = datetime.now().isoformat(timespec="seconds")
        self._write()

    def update_metadata(self, **metadata):
        self.data["metadata"].update(metadata)
        self.data["updated_at"] = datetime.now().isoformat(timespec="seconds")
        self._write()

    def _write(self):
        _atomic_write_json(self.output_path, self.data)


def _atomic_write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temp_path, path)


def _load_json(path):
    path = Path(path)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


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


def build_run_summary(run_dir):
    """Build one human-friendly summary from saved Main/Pose sample files."""
    run_dir = Path(run_dir)
    main = _load_json(run_dir / "main_profile.json")
    pose = _load_json(run_dir / "pose_profile.json")

    session_id = None
    if main:
        session_id = main.get("session_id")
    if session_id is None and pose:
        session_id = pose.get("session_id")

    summary = {
        "schema_version": 2,
        "session_id": session_id,
        "architecture": "MULTIPROCESS",
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "measurement_scope": "MEASURING samples only",
        "core_metrics": {
            "camera_fps_avg": _avg(main, "camera_fps"),
            "pose_fps_avg": _avg(pose, "pose_fps"),
            "pose_e2e_ms_avg": _avg(pose, "e2e_ms_avg"),
            "main_cpu_percent_avg": _avg(main, "main_cpu_percent"),
            "pose_cpu_percent_avg": _avg(pose, "pose_cpu_percent"),
        },
        "diagnostic_metrics": {
            "shared_memory_write_ms_avg": _avg(main, "shared_memory_write_ms_avg"),
            "queue_latency_ms_avg": _avg(pose, "queue_latency_ms_avg"),
            "mediapipe_ms_avg": _avg(pose, "mediapipe_ms_avg"),
            "ring_pending_max": _max(pose, "ring_pending"),
            "ring_skipped_final": _last(pose, "ring_skipped"),
            "ring_overrun_final": _last(pose, "ring_overrun"),
        },
        "sample_counts": {
            "main": len((main or {}).get("samples", [])),
            "pose": len((pose or {}).get("samples", [])),
        },
        "files": {
            "main_profile": "main_profile.json",
            "pose_profile": "pose_profile.json",
        },
    }
    return summary


def write_run_summary(run_dir):
    """Write/refresh summary.json for the performance viewer and reports."""
    run_dir = Path(run_dir)
    summary = build_run_summary(run_dir)
    _atomic_write_json(run_dir / "summary.json", summary)
    return summary

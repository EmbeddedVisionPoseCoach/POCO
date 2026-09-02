"""Lightweight JSON performance logging for POCO runtime benchmarks.

The logger intentionally has no third-party dependency.  It is used by the
main(CameraWorker) process and the Pose process so the same JSON schema can be
re-used later by the legacy single-process benchmark.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime
from pathlib import Path


class ProcessCpuSampler:
    """Estimate process CPU usage over one profiling window.

    100% means roughly one logical CPU core was fully occupied during the
    window.  This uses Python's process CPU clock, so no psutil installation is
    required and the same method can be used for the future single-process
    comparison.
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
    """Append profiler snapshots and atomically persist them as one JSON file."""

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
            "schema_version": 1,
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
        temp_path = self.output_path.with_suffix(self.output_path.suffix + ".tmp")
        temp_path.write_text(
            json.dumps(self.data, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temp_path, self.output_path)

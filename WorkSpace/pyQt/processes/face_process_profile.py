import os
import warnings
import logging
import faulthandler

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["GLOG_minloglevel"] = "2"

warnings.filterwarnings(
    "ignore",
    message=r".*SymbolDatabase\.GetPrototype\(\) is deprecated.*",
    category=UserWarning
)

logging.getLogger("tensorflow").setLevel(logging.ERROR)
faulthandler.enable(all_threads=True)

import sys
import time
from pathlib import Path
from queue import Empty

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

ROOT_DIR = Path(__file__).resolve().parents[2]
sys.path.append(str(ROOT_DIR))

import modules.config as config
from modules.features import calculate_face_features_for_window
from ipc.shared_frame_ring import SharedFrameReader
from services.calibration_service import CalibrationService
from services.face_gru_service import FaceGruService

MODE_IDLE = "IDLE"
MODE_CALIBRATING = "CALIBRATING"
MODE_WAITING = "WAITING"
MODE_MEASURING = "MEASURING"
PROFILE_INTERVAL_SEC = 2.0


class ProcessProfiler:
    def __init__(self, name, interval_sec=PROFILE_INTERVAL_SEC):
        self.name = name
        self.interval_sec = interval_sec
        self.reset()

    def reset(self):
        self.start_time = time.perf_counter()
        self.frame_count = 0
        self.samples = {
            "read": [],
            "queue": [],
            "color": [],
            "mp_image": [],
            "landmarker": [],
            "feature": [],
            "gru": [],
            "gru_result": [],
            "calibration": [],
            "process": [],
            "e2e": [],
        }

    def add(self, name, elapsed_ns):
        self.samples[name].append(elapsed_ns / 1_000_000.0)

    def add_ms(self, name, elapsed_ms):
        self.samples[name].append(float(elapsed_ms))

    def frame_done(self):
        self.frame_count += 1

    def ready(self):
        return time.perf_counter() - self.start_time >= self.interval_sec

    @staticmethod
    def _stats(values):
        if not values:
            return 0.0, 0.0, 0.0
        arr = np.asarray(values, dtype=np.float64)
        return float(np.mean(arr)), float(np.percentile(arr, 95)), float(np.max(arr))

    def build_report(self, mode, ring_resources):
        elapsed = max(time.perf_counter() - self.start_time, 1e-9)
        fps = self.frame_count / elapsed
        written = ring_resources["written_count"].value
        read = ring_resources["read_count"].value
        pending = written - read
        overrun = ring_resources["overrun_count"].value

        rows = [
            ("Read(wait+copy)", "read"),
            ("Queue latency", "queue"),
            ("BGR->RGB", "color"),
            ("mp.Image create", "mp_image"),
            ("FaceLandmarker", "landmarker"),
            ("Feature", "feature"),
            ("GRU update", "gru"),
            ("GRU result frames", "gru_result"),
            ("Calibration", "calibration"),
            ("Process excl.read", "process"),
            ("E2E frame->done", "e2e"),
        ]

        lines = [
            "",
            "================= FACE PROFILE =================",
            f"Mode={mode:<12} FPS={fps:6.2f} Frames={self.frame_count}",
            f"Ring Pending={pending} Overrun={overrun} Written={written} Read={read}",
            "------------------------------------------------",
            "Stage                 AVG(ms)    P95(ms)    MAX(ms)",
        ]

        for label, key in rows:
            avg, p95, maximum = self._stats(self.samples[key])
            lines.append(f"{label:<21} {avg:7.2f}    {p95:7.2f}    {maximum:7.2f}")

        lines.append("================================================")
        return "\n".join(lines)


def resolve_workspace_path(path):
    path = Path(path)
    return path if path.is_absolute() else ROOT_DIR / path


def create_face_detector():
    model_path = resolve_workspace_path(config.FACE_MODEL_PATH)
    if not model_path.exists():
        raise FileNotFoundError(f"FaceLandmarker 모델 없음 : {model_path}")

    base_options = python.BaseOptions(model_asset_path=str(model_path))
    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.IMAGE,
        output_face_blendshapes=True,
        num_faces=1,
        min_face_detection_confidence=0.5,
        min_face_presence_confidence=0.5,
        min_tracking_confidence=0.5
    )
    return vision.FaceLandmarker.create_from_options(options)


def build_face_features(results):
    if results is None or not results.face_blendshapes:
        return None

    features = calculate_face_features_for_window(results.face_blendshapes)
    if features is None:
        return None

    features = np.asarray(features, dtype=np.float32)
    if features.size != config.FACE_FEATURE_SIZE:
        return None

    return features


def run_face_process(stop_event, command_queue, result_queue, ring_resources):
    print("[FaceProcess:PROFILE] 시작")

    reader = None
    detector = None
    gru_service = None
    calibration_service = None

    mode = MODE_IDLE
    previous_frame_id = None
    processed_count = 0
    sequence_drop_count = 0
    calibration_missing_count = 0
    stat_start_time = time.monotonic()
    calibration_emit_time = 0.0
    profiler = ProcessProfiler("FACE")

    try:
        init_start = time.perf_counter_ns()
        reader = SharedFrameReader(ring_resources)

        detector_start = time.perf_counter_ns()
        detector = create_face_detector()
        detector_ms = (time.perf_counter_ns() - detector_start) / 1_000_000.0

        calibration_service = CalibrationService(
            baseline_path=resolve_workspace_path(config.FACE_BASELINE_PATH),
            duration=config.CALIBRATION_TIME,
            expected_fps=30,
            min_sample_ratio=0.6,
        )

        gru_service = FaceGruService()
        gru_start = time.perf_counter_ns()
        gru_service.load()
        gru_load_ms = (time.perf_counter_ns() - gru_start) / 1_000_000.0
        init_ms = (time.perf_counter_ns() - init_start) / 1_000_000.0

        print(
            f"[FACE INIT PROFILE] Detector={detector_ms:.2f}ms "
            f"GRU_Load={gru_load_ms:.2f}ms Total={init_ms:.2f}ms"
        )

        result_queue.put({"type": "FACE_READY", "success": True})
        print("[FaceProcess:PROFILE] 초기화 완료")

        while not stop_event.is_set():
            try:
                command = command_queue.get_nowait()

                if command == "START_CALIBRATION":
                    gru_service.stop()
                    calibration_result = calibration_service.start()
                    mode = MODE_CALIBRATING
                    previous_frame_id = None
                    processed_count = 0
                    sequence_drop_count = 0
                    calibration_missing_count = 0
                    stat_start_time = time.monotonic()
                    calibration_emit_time = 0.0
                    profiler.reset()
                    result_queue.put({
                        "type": "FACE_CALIBRATION_STARTED",
                        "message": calibration_result.message
                    })
                    print("[FaceProcess:PROFILE] Calibration 시작")

                elif command in ("START", "START_MEASUREMENT"):
                    calibration_service.cancel()
                    gru_service.baseline = gru_service.load_baseline()
                    gru_service.start()
                    mode = MODE_MEASURING
                    previous_frame_id = None
                    processed_count = 0
                    sequence_drop_count = 0
                    stat_start_time = time.monotonic()
                    profiler.reset()
                    print("[FaceProcess:PROFILE] 측정 시작")

                elif command == "STOP":
                    mode = MODE_IDLE
                    calibration_service.cancel()
                    gru_service.stop()
                    profiler.reset()
                    print("[FaceProcess:PROFILE] 정지")

                elif command == "SHUTDOWN":
                    break

            except Empty:
                pass

            if mode == MODE_IDLE:
                time.sleep(0.01)
                continue

            read_start = time.perf_counter_ns()
            frame_data = reader.read(timeout=0.1)
            read_end = time.perf_counter_ns()

            if frame_data is None:
                continue

            frame, frame_id, timestamp_ns = frame_data

            if mode == MODE_WAITING:
                continue

            process_start = time.perf_counter_ns()
            profiler.add("read", read_end - read_start)

            queue_latency_ms = (process_start - timestamp_ns) / 1_000_000.0
            profiler.add_ms("queue", queue_latency_ms)

            if previous_frame_id is not None:
                expected_frame_id = previous_frame_id + 1
                if frame_id != expected_frame_id:
                    sequence_drop_count += max(0, frame_id - expected_frame_id)
                    print(f"[FaceProcess:PROFILE] Frame Drop expected={expected_frame_id}, received={frame_id}")

            previous_frame_id = frame_id

            color_start = time.perf_counter_ns()
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            color_end = time.perf_counter_ns()
            profiler.add("color", color_end - color_start)

            image_start = time.perf_counter_ns()
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            image_end = time.perf_counter_ns()
            profiler.add("mp_image", image_end - image_start)

            landmarker_start = time.perf_counter_ns()
            results = detector.detect(mp_image)
            landmarker_end = time.perf_counter_ns()
            profiler.add("landmarker", landmarker_end - landmarker_start)

            feature_start = time.perf_counter_ns()
            features = build_face_features(results)
            feature_end = time.perf_counter_ns()
            profiler.add("feature", feature_end - feature_start)

            if mode == MODE_CALIBRATING:
                if features is None:
                    calibration_missing_count += 1

                cal_start = time.perf_counter_ns()
                calibration_result = calibration_service.update(features)
                cal_end = time.perf_counter_ns()
                profiler.add("calibration", cal_end - cal_start)

                if calibration_result.is_finished:
                    mode = MODE_WAITING
                    result_queue.put({
                        "type": "FACE_CALIBRATION_DONE",
                        "success": calibration_result.success,
                        "message": calibration_result.message,
                        "sample_count": calibration_result.sample_count,
                        "missing_count": calibration_missing_count,
                        "baseline_path": calibration_result.baseline_path
                    })
                    print(
                        f"[FaceProcess:PROFILE] Calibration 완료 success={calibration_result.success} "
                        f"samples={calibration_result.sample_count} missing={calibration_missing_count}"
                    )
                else:
                    now = time.monotonic()
                    if now - calibration_emit_time >= 0.25:
                        calibration_emit_time = now
                        result_queue.put({
                            "type": "FACE_CALIBRATION_PROGRESS",
                            "remain_time": calibration_result.remain_time,
                            "sample_count": calibration_result.sample_count,
                            "missing_count": calibration_missing_count,
                            "message": calibration_result.message
                        })

            elif mode == MODE_MEASURING:
                gru_start = time.perf_counter_ns()
                gru_result = gru_service.update(features)
                gru_end = time.perf_counter_ns()
                profiler.add("gru", gru_end - gru_start)

                if gru_result is not None:
                    profiler.add("gru_result", gru_end - gru_start)
                    total_latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0
                    result_queue.put({
                        "type": "FACE_RESULT",
                        "frame_id": frame_id,
                        "timestamp_ns": timestamp_ns,
                        "fatigue_label": gru_result["fatigue_label"],
                        "fatigue_probability": gru_result["fatigue_probability"],
                        "fatigue_index": gru_result["fatigue_index"],
                        "latency_ms": total_latency_ms
                    })

            process_end = time.perf_counter_ns()
            profiler.add("process", process_end - process_start)
            profiler.add("e2e", process_end - timestamp_ns)
            profiler.frame_done()

            processed_count += 1
            total_latency_ms = (process_end - timestamp_ns) / 1_000_000.0
            now = time.monotonic()
            elapsed = now - stat_start_time

            if elapsed >= 1.0:
                result_queue.put({
                    "type": "FACE_STATS",
                    "fps": processed_count / elapsed,
                    "processed": processed_count,
                    "sequence_drop": sequence_drop_count,
                    "last_frame_id": frame_id,
                    "queue_latency_ms": queue_latency_ms,
                    "total_latency_ms": total_latency_ms,
                    "mode": mode
                })
                processed_count = 0
                stat_start_time = now

            if profiler.ready():
                print(profiler.build_report(mode, ring_resources), flush=True)
                profiler.reset()

    except Exception as e:
        result_queue.put({
            "type": "FACE_ERROR",
            "success": False,
            "message": str(e)
        })
        raise

    finally:
        if calibration_service is not None:
            calibration_service.cancel()
        if gru_service is not None:
            gru_service.close()
        if detector is not None:
            detector.close()
        if reader is not None:
            reader.close()
        print("[FaceProcess:PROFILE] 종료")

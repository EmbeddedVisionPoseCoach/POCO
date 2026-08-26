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

ROOT_DIR = Path(__file__).resolve().parents[2]
sys.path.append(str(ROOT_DIR))

import modules.config as config
from modules.features import calculate_features
from ipc.shared_frame_ring import SharedFrameReader
from ipc.queue_utils import get_latest, drain_ordered, put_latest, put_ordered
from services.calibration_service import CalibrationService
from services.pose_gru_service import PoseGruService

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
            "mediapipe": [],
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
            ("MediaPipe Pose", "mediapipe"),
            ("Feature", "feature"),
            ("GRU update", "gru"),
            ("GRU result frames", "gru_result"),
            ("Calibration", "calibration"),
            ("Process excl.read", "process"),
            ("E2E frame->done", "e2e"),
        ]

        lines = [
            "",
            "================= POSE PROFILE =================",
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


def create_pose_detector():
    return mp.solutions.pose.Pose(
        model_complexity=0,
        min_detection_confidence=config.MIN_CONFIDENCE,
        min_tracking_confidence=config.MIN_CONFIDENCE
    )


def build_pose_features(results):
    if results is None or not results.pose_landmarks:
        return None

    features = calculate_features([results.pose_landmarks.landmark])
    if features is None:
        return None

    features = np.asarray(features, dtype=np.float32)
    if features.size != config.POSE_FEATURE_SIZE:
        return None

    return features


def serialize_pose_landmarks(results):
    if results is None or not results.pose_landmarks:
        return None

    return [
        [float(lm.x), float(lm.y), float(lm.z), float(lm.visibility)]
        for lm in results.pose_landmarks.landmark
    ]


def run_pose_process(
    stop_event,
    command_queue,
    result_queue,
    state_to_main_queue,
    ring_resources,
    pose_to_hw_state_queue,
    pose_to_hw_event_queue,
    hw_to_pose_state_queue,
    hw_to_pose_event_queue,
):
    print("[PoseProcess:PROFILE] 시작")

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
    latest_hardware_state = None
    latest_hardware_event = None
    profiler = ProcessProfiler("POSE")

    try:
        init_start = time.perf_counter_ns()
        reader = SharedFrameReader(ring_resources)

        detector_start = time.perf_counter_ns()
        detector = create_pose_detector()
        detector_ms = (time.perf_counter_ns() - detector_start) / 1_000_000.0

        # MediaPipe graph/TFLite delegate는 첫 process()에서 큰 지연이 생길 수 있다.
        # POSE_READY 전에 더미 프레임으로 미리 초기화해서 실제 Calibration/Measurement
        # 시작 직후 Shared Ring overrun이 발생하는 것을 줄인다.
        warmup_start = time.perf_counter_ns()
        dummy_rgb = np.zeros(
            (config.FRAME_HEIGHT, config.FRAME_WIDTH, 3),
            dtype=np.uint8
        )
        for _ in range(2):
            detector.process(dummy_rgb)
        warmup_ms = (time.perf_counter_ns() - warmup_start) / 1_000_000.0
        del dummy_rgb

        calibration_service = CalibrationService(
            baseline_path=resolve_workspace_path(config.BASELINE_PATH),
            duration=config.CALIBRATION_TIME,
            expected_fps=30,
            min_sample_ratio=0.6
        )

        # GRU는 Calibration에 필요하지 않으므로 여기서 로드하지 않는다.
        # 실제 측정 START_MEASUREMENT에서 lazy load한다.
        gru_service = PoseGruService()
        init_ms = (time.perf_counter_ns() - init_start) / 1_000_000.0

        print(
            f"[POSE INIT PROFILE] Detector={detector_ms:.2f}ms "
            f"Warmup={warmup_ms:.2f}ms GRU_Load=LAZY Total={init_ms:.2f}ms"
        )

        result_queue.put({"type": "POSE_READY", "success": True})
        print("[PoseProcess:PROFILE] 초기화 완료")

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

                    started_event = {
                        "type": "POSE_CALIBRATION_STARTED",
                        "message": calibration_result.message,
                        "timestamp": time.time(),
                    }
                    result_queue.put(started_event)
                    put_ordered(pose_to_hw_event_queue, started_event)
                    print("[PoseProcess:PROFILE] Calibration 시작")

                elif command in ("START", "START_MEASUREMENT"):
                    calibration_service.cancel()

                    try:
                        # 측정 시 새 Calibration을 하지 않는다.
                        # Calibration 버튼에서 저장된 baseline만 다시 로드한다.
                        gru_service.baseline = gru_service.load_baseline(required=True)
                        gru_service.start()
                        mode = MODE_MEASURING
                        previous_frame_id = None
                        processed_count = 0
                        sequence_drop_count = 0
                        stat_start_time = time.monotonic()
                        profiler.reset()

                        started_event = {
                            "type": "POSE_MEASUREMENT_STARTED",
                            "success": True,
                            "message": "Pose 저장 baseline 로드 완료",
                            "timestamp": time.time(),
                        }
                        result_queue.put(started_event)
                        put_ordered(pose_to_hw_event_queue, started_event)
                        print("[PoseProcess:PROFILE] 측정 시작")

                    except Exception as e:
                        mode = MODE_WAITING
                        gru_service.stop()
                        failed_event = {
                            "type": "POSE_MEASUREMENT_STARTED",
                            "success": False,
                            "message": str(e),
                            "timestamp": time.time(),
                        }
                        result_queue.put(failed_event)
                        put_ordered(pose_to_hw_event_queue, failed_event)
                        print(f"[PoseProcess:PROFILE] 측정 시작 실패: {e}")

                elif command == "STOP":
                    mode = MODE_IDLE
                    calibration_service.cancel()
                    gru_service.stop()
                    profiler.reset()
                    print("[PoseProcess:PROFILE] 정지")

                elif command == "SHUTDOWN":
                    break

            except Empty:
                pass

            # Hardware -> Pose
            # State는 최신값만, Event는 순서대로 모두 수신한다.
            latest_hardware_state = get_latest(
                hw_to_pose_state_queue,
                latest_hardware_state
            )
            for hardware_event in drain_ordered(hw_to_pose_event_queue):
                latest_hardware_event = hardware_event

            if mode == MODE_IDLE:
                time.sleep(0.01)
                continue

            read_start = time.perf_counter_ns()
            frame_data = reader.read_latest(timeout=0.1)
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
                    print(f"[PoseProcess:PROFILE] Frame Drop expected={expected_frame_id}, received={frame_id}")

            previous_frame_id = frame_id

            color_start = time.perf_counter_ns()
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            color_end = time.perf_counter_ns()
            profiler.add("color", color_end - color_start)

            mp_start = time.perf_counter_ns()
            results = detector.process(rgb_frame)
            mp_end = time.perf_counter_ns()
            profiler.add("mediapipe", mp_end - mp_start)

            feature_start = time.perf_counter_ns()
            features = build_pose_features(results)
            landmarks = serialize_pose_landmarks(results)
            feature_end = time.perf_counter_ns()
            profiler.add("feature", feature_end - feature_start)

            # Pose 최신 상태는 Main과 Hardware가 동일한 구조로 받는다.
            # Calibration 중에도 MediaPipe landmark / feature가 계속 포함된다.
            pose_state = {
                "type": "POSE_STATE",
                "frame_id": frame_id,
                "timestamp_ns": timestamp_ns,
                "mode": mode,
                "landmark_valid": landmarks is not None,
                "landmarks": landmarks,
                "features": features.tolist() if features is not None else None,
                "inference": None,
                "calibration": None,
                "hardware_state": latest_hardware_state,
                "hardware_event": latest_hardware_event,
            }

            if mode == MODE_CALIBRATING:
                if features is None:
                    calibration_missing_count += 1

                cal_start = time.perf_counter_ns()
                calibration_result = calibration_service.update(features)
                cal_end = time.perf_counter_ns()
                profiler.add("calibration", cal_end - cal_start)

                pose_state["calibration"] = {
                    "is_finished": calibration_result.is_finished,
                    "success": calibration_result.success,
                    "remain_time": calibration_result.remain_time,
                    "sample_count": calibration_result.sample_count,
                    "missing_count": calibration_missing_count,
                }

                if calibration_result.is_finished:
                    mode = MODE_WAITING
                    done_event = {
                        "type": "POSE_CALIBRATION_DONE",
                        "success": calibration_result.success,
                        "message": calibration_result.message,
                        "sample_count": calibration_result.sample_count,
                        "missing_count": calibration_missing_count,
                        "baseline_path": calibration_result.baseline_path,
                        "timestamp": time.time(),
                    }
                    result_queue.put(done_event)
                    put_ordered(pose_to_hw_event_queue, done_event)
                    print(
                        f"[PoseProcess:PROFILE] Calibration 완료 success={calibration_result.success} "
                        f"samples={calibration_result.sample_count} missing={calibration_missing_count}"
                    )
                else:
                    now = time.monotonic()
                    if now - calibration_emit_time >= 0.25:
                        calibration_emit_time = now
                        result_queue.put({
                            "type": "POSE_CALIBRATION_PROGRESS",
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
                    pose_result = {
                        "type": "POSE_RESULT",
                        "frame_id": frame_id,
                        "timestamp_ns": timestamp_ns,
                        "posture_type": gru_result["posture_type"],
                        "confidence": gru_result["confidence"],
                        "pose_index": gru_result["pose_index"],
                        "latency_ms": total_latency_ms,
                    }
                    result_queue.put(pose_result)

                    pose_state["inference"] = {
                        "posture_type": gru_result["posture_type"],
                        "confidence": gru_result["confidence"],
                        "pose_index": gru_result["pose_index"],
                        "latency_ms": total_latency_ms,
                    }

            # State는 항상 최신값 하나만 유지한다.
            put_latest(state_to_main_queue, pose_state)
            put_latest(pose_to_hw_state_queue, pose_state)

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
                    "type": "POSE_STATS",
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
        error_event = {
            "type": "POSE_ERROR",
            "success": False,
            "message": str(e),
            "timestamp": time.time(),
        }
        result_queue.put(error_event)
        put_ordered(pose_to_hw_event_queue, error_event)
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
        print("[PoseProcess:PROFILE] 종료")

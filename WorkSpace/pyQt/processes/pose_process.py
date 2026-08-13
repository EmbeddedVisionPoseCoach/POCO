import os
import warnings
import logging

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["GLOG_minloglevel"] = "2"

warnings.filterwarnings(
    "ignore",
    message=r".*SymbolDatabase\.GetPrototype\(\) is deprecated.*",
    category=UserWarning
)

logging.getLogger("tensorflow").setLevel(logging.ERROR)

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
from services.calibration_service import CalibrationService
from services.pose_gru_service import PoseGruService


MODE_IDLE = "IDLE"
MODE_CALIBRATING = "CALIBRATING"
MODE_WAITING = "WAITING"
MODE_MEASURING = "MEASURING"


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


def run_pose_process(stop_event, command_queue, result_queue, ring_resources):
    print("[PoseProcess] 시작")

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

    try:
        reader = SharedFrameReader(ring_resources)
        detector = create_pose_detector()

        calibration_service = CalibrationService(
            baseline_path=resolve_workspace_path(config.BASELINE_PATH),
            duration=config.CALIBRATION_TIME,
            expected_fps=30,
            min_sample_ratio=0.6
        )

        gru_service = PoseGruService()
        gru_service.load()

        result_queue.put({"type": "POSE_READY", "success": True})
        print("[PoseProcess] 초기화 완료")

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

                    result_queue.put({
                        "type": "POSE_CALIBRATION_STARTED",
                        "message": calibration_result.message
                    })

                    print("[PoseProcess] Calibration 시작")

                elif command in ("START", "START_MEASUREMENT"):
                    calibration_service.cancel()

                    # Calibration에서 새로 저장한 baseline 다시 로드
                    gru_service.baseline = gru_service.load_baseline()
                    gru_service.start()

                    mode = MODE_MEASURING
                    previous_frame_id = None
                    processed_count = 0
                    sequence_drop_count = 0
                    stat_start_time = time.monotonic()

                    print("[PoseProcess] 측정 시작")

                elif command == "STOP":
                    mode = MODE_IDLE
                    calibration_service.cancel()
                    gru_service.stop()

                    print("[PoseProcess] 정지")

                elif command == "SHUTDOWN":
                    break

            except Empty:
                pass

            if mode == MODE_IDLE:
                time.sleep(0.01)
                continue

            frame_data = reader.read(timeout=0.1)

            if frame_data is None:
                continue

            frame, frame_id, timestamp_ns = frame_data

            # Calibration 완료 후 Main이 다음 명령을 줄 때까지 Ring만 비워준다.
            if mode == MODE_WAITING:
                continue

            queue_latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0

            if previous_frame_id is not None:
                expected_frame_id = previous_frame_id + 1

                if frame_id != expected_frame_id:
                    sequence_drop_count += max(0, frame_id - expected_frame_id)
                    print(f"[PoseProcess] Frame Drop expected={expected_frame_id}, received={frame_id}")

            previous_frame_id = frame_id

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = detector.process(rgb_frame)
            features = build_pose_features(results)

            if mode == MODE_CALIBRATING:
                if features is None:
                    calibration_missing_count += 1

                calibration_result = calibration_service.update(features)

                if calibration_result.is_finished:
                    mode = MODE_WAITING

                    result_queue.put({
                        "type": "POSE_CALIBRATION_DONE",
                        "success": calibration_result.success,
                        "message": calibration_result.message,
                        "sample_count": calibration_result.sample_count,
                        "missing_count": calibration_missing_count,
                        "baseline_path": calibration_result.baseline_path
                    })

                    print(
                        f"[PoseProcess] Calibration 완료 "
                        f"success={calibration_result.success} "
                        f"samples={calibration_result.sample_count} "
                        f"missing={calibration_missing_count}"
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
                gru_result = gru_service.update(features)

                if gru_result is not None:
                    total_latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0

                    result_queue.put({
                        "type": "POSE_RESULT",
                        "frame_id": frame_id,
                        "timestamp_ns": timestamp_ns,
                        "posture_type": gru_result["posture_type"],
                        "confidence": gru_result["confidence"],
                        "pose_index": gru_result["pose_index"],
                        "latency_ms": total_latency_ms
                    })

            processed_count += 1
            total_latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0

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

    except Exception as e:
        result_queue.put({
            "type": "POSE_ERROR",
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

        print("[PoseProcess] 종료")
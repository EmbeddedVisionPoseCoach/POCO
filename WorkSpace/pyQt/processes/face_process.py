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
    print("[FaceProcess] 시작")

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
        detector = create_face_detector()

        calibration_service = CalibrationService(
            baseline_path=resolve_workspace_path(config.FACE_BASELINE_PATH),
            duration=config.CALIBRATION_TIME,
            expected_fps=30,
            min_sample_ratio=0.6,
        )

        gru_service = FaceGruService()
        gru_service.load()

        result_queue.put({"type": "FACE_READY", "success": True})
        print("[FaceProcess] 초기화 완료")

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
                        "type": "FACE_CALIBRATION_STARTED",
                        "message": calibration_result.message
                    })

                    print("[FaceProcess] Calibration 시작")

                elif command in ("START", "START_MEASUREMENT"):
                    calibration_service.cancel()

                    gru_service.baseline = gru_service.load_baseline()
                    gru_service.start()

                    mode = MODE_MEASURING
                    previous_frame_id = None
                    processed_count = 0
                    sequence_drop_count = 0
                    stat_start_time = time.monotonic()

                    print("[FaceProcess] 측정 시작")

                elif command == "STOP":
                    mode = MODE_IDLE
                    calibration_service.cancel()
                    gru_service.stop()

                    print("[FaceProcess] 정지")

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

            if mode == MODE_WAITING:
                continue

            queue_latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0

            if previous_frame_id is not None:
                expected_frame_id = previous_frame_id + 1

                if frame_id != expected_frame_id:
                    sequence_drop_count += max(0, frame_id - expected_frame_id)
                    print(f"[FaceProcess] Frame Drop expected={expected_frame_id}, received={frame_id}")

            previous_frame_id = frame_id

            # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # gray_rgb_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            results = detector.detect(mp_image)
            features = build_face_features(results)

            if mode == MODE_CALIBRATING:
                if features is None:
                    calibration_missing_count += 1

                calibration_result = calibration_service.update(features)

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
                        f"[FaceProcess] Calibration 완료 "
                        f"success={calibration_result.success} "
                        f"samples={calibration_result.sample_count} "
                        f"missing={calibration_missing_count}"
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
                gru_result = gru_service.update(features)

                if gru_result is not None:
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

            processed_count += 1
            total_latency_ms = (time.perf_counter_ns() - timestamp_ns) / 1_000_000.0

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

        print("[FaceProcess] 종료")
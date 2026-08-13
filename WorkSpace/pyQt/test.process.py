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

import time
from queue import Empty

import cv2

from managers.vision_process_manager import VisionProcessManager


def wait_process_ready(manager, timeout=30.0):
    pose_ready = False
    face_ready = False
    start_time = time.monotonic()

    while not (pose_ready and face_ready):
        if time.monotonic() - start_time > timeout:
            raise TimeoutError("Pose / Face Process 초기화 Timeout")

        try:
            result = manager.pose_result_queue.get_nowait()
            print(result)

            if result.get("type") == "POSE_READY":
                pose_ready = True

            elif result.get("type") == "POSE_ERROR":
                raise RuntimeError(result["message"])

        except Empty:
            pass

        try:
            result = manager.face_result_queue.get_nowait()
            print(result)

            if result.get("type") == "FACE_READY":
                face_ready = True

            elif result.get("type") == "FACE_ERROR":
                raise RuntimeError(result["message"])

        except Empty:
            pass

        time.sleep(0.01)


def print_pose_results(manager, camera_frame_id):
    calibration_done = None

    try:
        while True:
            result = manager.pose_result_queue.get_nowait()
            result_type = result.get("type")

            if result_type == "POSE_CALIBRATION_STARTED":
                print(f"[POSE CAL] {result['message']}")

            elif result_type == "POSE_CALIBRATION_PROGRESS":
                print(
                    f"[POSE CAL] Remain={result['remain_time']:.1f}s "
                    f"Samples={result['sample_count']} "
                    f"Missing={result['missing_count']}"
                )

            elif result_type == "POSE_CALIBRATION_DONE":
                calibration_done = result

                print(
                    f"[POSE CAL DONE] Success={result['success']} "
                    f"Samples={result['sample_count']} "
                    f"Missing={result['missing_count']} "
                    f"Path={result['baseline_path']}"
                )

            elif result_type == "POSE_RESULT":
                print(
                    f"[POSE RESULT] {result['posture_type']} "
                    f"{result['confidence'] * 100:.1f}% "
                    f"Frame={result['frame_id']} "
                    f"Latency={result['latency_ms']:.2f}ms"
                )

            elif result_type == "POSE_STATS":
                stats = manager.get_stats()
                frame_delay = camera_frame_id - result["last_frame_id"]

                print(
                    f"[POSE] Mode={result['mode']} FPS={result['fps']:.1f} "
                    f"Drop={result['sequence_drop']} DelayFrame={frame_delay} "
                    f"Pending={stats['pose_pending']} "
                    f"Queue={result['queue_latency_ms']:.2f}ms "
                    f"Total={result['total_latency_ms']:.2f}ms "
                    f"Overrun={stats['pose_overrun']}"
                )

            elif result_type == "POSE_ERROR":
                print(f"[POSE ERROR] {result['message']}")

    except Empty:
        pass

    return calibration_done


def print_face_results(manager, camera_frame_id):
    calibration_done = None

    try:
        while True:
            result = manager.face_result_queue.get_nowait()
            result_type = result.get("type")

            if result_type == "FACE_CALIBRATION_STARTED":
                print(f"[FACE CAL] {result['message']}")

            elif result_type == "FACE_CALIBRATION_PROGRESS":
                print(
                    f"[FACE CAL] Remain={result['remain_time']:.1f}s "
                    f"Samples={result['sample_count']} "
                    f"Missing={result['missing_count']}"
                )

            elif result_type == "FACE_CALIBRATION_DONE":
                calibration_done = result

                print(
                    f"[FACE CAL DONE] Success={result['success']} "
                    f"Samples={result['sample_count']} "
                    f"Missing={result['missing_count']} "
                    f"Path={result['baseline_path']}"
                )

            elif result_type == "FACE_RESULT":
                print(
                    f"[FACE RESULT] {result['fatigue_label']} "
                    f"{result['fatigue_probability'] * 100:.1f}% "
                    f"Frame={result['frame_id']} "
                    f"Latency={result['latency_ms']:.2f}ms"
                )

            elif result_type == "FACE_STATS":
                stats = manager.get_stats()
                frame_delay = camera_frame_id - result["last_frame_id"]

                print(
                    f"[FACE] Mode={result['mode']} FPS={result['fps']:.1f} "
                    f"Drop={result['sequence_drop']} DelayFrame={frame_delay} "
                    f"Pending={stats['face_pending']} "
                    f"Queue={result['queue_latency_ms']:.2f}ms "
                    f"Total={result['total_latency_ms']:.2f}ms "
                    f"Overrun={stats['face_overrun']}"
                )

            elif result_type == "FACE_ERROR":
                print(f"[FACE ERROR] {result['message']}")

    except Empty:
        pass

    return calibration_done


def main():
    frame_shape = (240, 320, 3)

    manager = VisionProcessManager(frame_shape=frame_shape, slot_count=32)
    camera = None

    try:
        manager.start()
        print("[Main] Process 시작")

        wait_process_ready(manager)
        print("[Main] Pose / Face 준비 완료")

        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        camera.set(cv2.CAP_PROP_FPS, 30)

        if not camera.isOpened():
            raise RuntimeError("Camera를 열 수 없습니다.")

        # Camera Warm-up
        for _ in range(10):
            camera.read()

        frame_id = 0
        pose_calibration_done = False
        face_calibration_done = False

        print("\n========== CALIBRATION START ==========")
        manager.start_calibration()

        while not (pose_calibration_done and face_calibration_done):
            ret, frame = camera.read()

            if not ret:
                continue

            frame = cv2.flip(frame, 1)

            if frame.shape[:2] != (240, 320):
                frame = cv2.resize(frame, (320, 240))

            frame_id += 1
            timestamp_ns = time.perf_counter_ns()

            pose_success, face_success = manager.write_frame(frame, frame_id, timestamp_ns)

            if not pose_success:
                print(f"[ERROR] Pose Ring Overrun : Frame={frame_id}")

            if not face_success:
                print(f"[ERROR] Face Ring Overrun : Frame={frame_id}")

            pose_result = print_pose_results(manager, frame_id)
            face_result = print_face_results(manager, frame_id)

            if pose_result is not None:
                if not pose_result["success"]:
                    raise RuntimeError(f"Pose Calibration 실패 : {pose_result['message']}")

                pose_calibration_done = True

            if face_result is not None:
                if not face_result["success"]:
                    raise RuntimeError(f"Face Calibration 실패 : {face_result['message']}")

                face_calibration_done = True

            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                return

        print("\n========== CALIBRATION COMPLETE ==========")
        print("Pose / Face Baseline 저장 완료")

        print("\n========== MEASUREMENT START ==========")
        manager.start_measurement()

        while True:
            ret, frame = camera.read()

            if not ret:
                continue

            frame = cv2.flip(frame, 1)

            if frame.shape[:2] != (240, 320):
                frame = cv2.resize(frame, (320, 240))

            frame_id += 1
            timestamp_ns = time.perf_counter_ns()

            pose_success, face_success = manager.write_frame(frame, frame_id, timestamp_ns)

            if not pose_success:
                print(f"[ERROR] Pose Ring Overrun : Frame={frame_id}")

            if not face_success:
                print(f"[ERROR] Face Ring Overrun : Frame={frame_id}")

            print_pose_results(manager, frame_id)
            print_face_results(manager, frame_id)

            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        if camera is not None:
            camera.release()

        cv2.destroyAllWindows()

        print("\n[Main] 최종 Ring 통계")
        print(manager.get_stats())

        manager.stop()


if __name__ == "__main__":
    main()
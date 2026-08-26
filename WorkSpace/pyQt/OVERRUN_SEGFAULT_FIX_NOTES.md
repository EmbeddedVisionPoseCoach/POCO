# Overrun / Segmentation fault 수정 메모

## 원인 1: 측정 시작 시 Ring Overrun
- Camera는 약 30 FPS로 Shared Frame Ring에 기록한다.
- Pose Process가 START_MEASUREMENT를 받은 뒤 baseline/scaler/TFLite 모델을 동기 로드하는 동안 frame을 읽지 못했다.
- Ring slot이 4개이므로 모델 로딩 동안 빠르게 가득 차고 overrun이 증가했다.
- 수정: START_MEASUREMENT 요청 시 `accept_frames=False`로 Ring 공급을 잠시 멈추고, Pose/Face START ACK가 성공한 뒤 `resume_measurement_frames()`로 공급을 재개한다.
- 카메라 capture와 PyQt preview는 계속 동작한다.

## 원인 2 후보: 종료 직후 Segmentation fault
기존에는 CameraWorker.run()의 finally, 즉 Camera QThread 내부에서 ResultWorker(QThread), multiprocessing Process, SharedMemory를 함께 정리했다.
수정 후에는:
1. Camera QThread capture 종료 및 camera.release()
2. Main thread의 CameraWorker.stop()에서 child Process 종료/join
3. ResultWorker 종료/wait
4. SharedMemory close/unlink
5. multiprocessing Queue close/join_thread
순으로 정리한다.

## 추가 진단
- 각 child Process의 exitcode를 종료 시 출력한다.
- `exitcode=0`인데 shell에서 Segmentation fault가 계속 발생하면 Picamera2/libcamera 또는 Qt native teardown 쪽을 다음 후보로 좁힐 수 있다.

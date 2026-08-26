# VisionPoseCoach PyQt 통합 수정본

기준: `pyQt(6).zip`

## 1. Cam Off와 App Shutdown 분리

기존에는 Cam Off 시 아래를 한 번에 모두 제거했다.

- Picamera2
- CameraWorker QThread
- ResultWorker QThread
- Pose / Hardware multiprocessing Process
- SharedMemory
- multiprocessing Queue
- CameraWorker Python/PyQt wrapper

로그상 모든 native/child 종료가 성공한 뒤 Qt UI event loop 복귀 시 Segmentation fault가 발생했기 때문에,
Cam Off에서는 카메라 QThread만 종료하고 Vision/Hardware Process는 IDLE 상태로 유지하도록 변경했다.

### Cam Off

- Camera capture loop 종료
- Picamera2 stop/close
- Pose inference STOP
- Vision/Hardware Process 유지
- ResultWorker 유지
- Queue/SharedMemory 유지
- CameraWorker QObject/QThread wrapper 유지

다음 Cam On에서는 같은 CameraWorker와 Vision Process를 재사용한다.

### App Close

앱 자체를 닫을 때만:

- Camera 종료
- Pose/Face/Hardware child process 종료
- ResultWorker 종료
- SharedMemory close/unlink
- Queue close/join_thread

을 수행한다.

Qt QObject/QThread wrapper는 `None`으로 즉시 파괴하지 않고 MainWindow child 수명에 맡긴다.

## 2. 종료 중 queued signal 방어

MainWindow에 다음 상태를 추가했다.

- `_camera_shutdown_in_progress`
- `_app_closing`

카메라가 멈추는 동안 이미 Qt event queue에 들어온 frame/status/result/state signal이 도착해도 UI slot에서 무시한다.
Cam Off가 끝난 뒤 `QCoreApplication.processEvents()`로 남은 queued event를 worker가 살아 있는 상태에서 한 번 소진한다.

## 3. MediaPipe Pose warm-up

Pose Process가 `POSE_READY`를 보내기 전에 320x240 dummy RGB frame을 2회 `detector.process()`한다.

목적:

- MediaPipe graph 초기화
- 내부 TFLite/XNNPACK delegate 첫 실행 지연을 측정 전에 소화
- 첫 실제 frame에서 300~400ms 이상 튀며 Ring overrun이 생기는 현상 완화

GRU TFLite model은 기존처럼 measurement 시작 시 lazy-load하고, ACK 전까지 Shared Ring frame 공급을 멈춘다.

## 4. Segfault 진단 유지

`mainpyQt.py`에서 `faulthandler.enable(all_threads=True)`를 활성화했다.
`start_pyqt.sh`도 `python -X faulthandler`로 실행한다.

재발 시 단순 `Segmentation fault` 외에 Python thread stack을 최대한 남기기 위한 목적이다.

## 5. Locale

`start_pyqt.sh` 실행 시에만:

```bash
LANG=C.UTF-8
LC_ALL=C.UTF-8
```

을 사용한다. 시스템 locale 설정을 강제로 변경하지 않는다.

## 테스트 포인트

1. `Cam On -> 측정 -> Cam Off` 후 앱이 계속 살아 있는지
2. Cam Off 후 다시 `Cam On` 했을 때 CameraWorker/Pose Process 재사용이 정상인지
3. 첫 측정에서 `Pose Ring Overrun`이 0 또는 기존보다 크게 감소하는지
4. 앱 X/종료 시 Pose/Hardware `exitcode=0`인지
5. Segmentation fault가 재발한다면 faulthandler 출력 전체 확인

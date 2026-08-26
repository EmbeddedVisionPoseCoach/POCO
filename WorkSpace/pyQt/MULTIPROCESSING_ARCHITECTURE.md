# Vision Pose Coach 멀티프로세싱 구조

## PROFILE_MODE

```python
PROFILE_MODE = "POSE_ONLY"
```

- `POSE_ONLY`: Main + Pose + Hardware
- `FACE_ONLY`: Main + Face + Hardware
- `BOTH`: Main + Pose + Face + Hardware

기능 비활성화는 코드 주석처리가 아니라 `PROFILE_MODE`로만 제어한다.

## 프로세스 역할

```text
                         Main Process
                 PyQt UI + Camera read()
                           |
                     Shared Frame
                    /             \
                   v               v
             Pose Process       Face Process
             (mode에 따라)       (mode에 따라)
                   <-----> Hardware <----->
                         Process
```

Main이 카메라를 한 번만 읽는다. 활성화된 Pose/Face Process는 Main이 기록한 동일 프레임을 각각 Shared Memory Ring에서 읽고 MediaPipe를 수행한다.

## Pose / Face -> Main / Hardware

Pose/Face에서 MediaPipe Landmark와 Feature를 생성한다.

```text
MediaPipe
   |
Landmark / Feature
   |\
   | +----> Main Process (UI/상태)
   |
   +------> Hardware Process
```

Main에서 MediaPipe를 수행하지 않는다.

## IPC 분리

### State Queue

`maxsize=1`이며 최신 상태만 유지한다.

예:
- Landmark
- Feature
- IR / IMU
- Hardware current state
- Process mode

### Event / Command Queue

순서대로 보존하며 기존 항목을 최신 데이터로 덮어쓰지 않는다.

예:
- START_CALIBRATION
- START_MEASUREMENT
- STOP
- CALIBRATION_DONE
- ERROR
- ACK

## Calibration

Calibration 버튼에서만 새 baseline을 만든다.

```text
Calibration button
 -> Pose/Face command
 -> MediaPipe
 -> Landmark
 -> Feature
 -> CalibrationService
 -> baseline file
```

Measurement에서는 저장된 baseline만 다시 불러온다.

```text
Measurement button
 -> load_baseline(required=True)
 -> GRU start
```

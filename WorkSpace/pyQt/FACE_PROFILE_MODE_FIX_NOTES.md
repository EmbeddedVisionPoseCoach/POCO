# Face 비활성화 / PROFILE_MODE 수정 정리

## 현재 실행 모드

```python
PROFILE_MODE = "POSE_ONLY"
```

현재는 다음 Process만 실제 실행된다.

- Main Process: PyQt + Camera read
- Pose Process: MediaPipe Pose + Calibration + GRU
- Hardware Process: IPC 골격
- Face Process: 실행하지 않음

Face 기능을 끄기 위해 코드나 Queue를 주석처리하지 않는다.
`PROFILE_MODE`로만 실행 여부를 결정한다.

## 이번 수정 이유

기존 `pyQt_2.zip`은 Face 관련 코드를 일부만 주석처리해 다음 문제가 있었다.

1. `CameraWorker.face_state_changed`는 제거했는데 Main에서는 signal connect를 계속 수행함.
2. `face_result_queue`를 제거했는데 ResultWorker는 계속 읽으려고 함.
3. Hardware Process 함수는 Face Queue 인자를 요구하지만 Process 생성부에서는 전달하지 않음.
4. Hardware Process 내부에서도 Face Queue를 일부만 주석처리해 함수 인터페이스가 깨짐.

## 수정 원칙

### 인터페이스는 항상 유지

다음 객체는 POSE_ONLY에서도 존재한다.

- `face_command_queue`
- `face_result_queue`
- `face_state_to_main_queue`
- `face_to_hw_state_queue`
- `face_to_hw_event_queue`
- `hw_to_face_state_queue`
- `hw_to_face_event_queue`
- `CameraWorker.face_state_changed`

하지만 `enable_face == False`이면 다음은 생성/실행하지 않는다.

- Face Shared Frame Ring
- Face Process
- Face MediaPipe
- Face GRU

따라서 현재 Pose-only 성능에는 Face MediaPipe 연산이 추가되지 않는다.

## Calibration

Calibration은 Calibration 버튼에서만 실행한다.

```text
Main Calibration 버튼
 -> START_CALIBRATION
 -> Pose Process
 -> MediaPipe Pose
 -> Landmark
 -> Feature
 -> CalibrationService
 -> baseline 저장
```

측정 시작에서는 Calibration을 다시 실행하지 않는다.

```text
측정 버튼
 -> START_MEASUREMENT
 -> 저장 baseline load(required=True)
 -> GRU start
```

## Face를 다시 켤 때

Pose + Face를 같이 사용할 때는 manager의 한 줄만 변경한다.

```python
PROFILE_MODE = "BOTH"
```

주석을 다시 풀거나 Queue/Signal 코드를 복구할 필요가 없다.

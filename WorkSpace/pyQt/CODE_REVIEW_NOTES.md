# 코드 리뷰 / 수정 메모

## 이번에 수정한 런타임 문제

### 1. Face 부분 주석처리로 Signal 인터페이스 단절

`CameraWorker.face_state_changed`는 제거되었지만 Main은 signal connect를 수행하고 있어 런타임 `AttributeError` 가능성이 있었다.

수정: signal은 항상 정의하고 Face Process 실행 여부만 `enable_face`로 결정한다.

### 2. Face Result Queue 단절

`face_result_queue`가 제거된 상태에서 ResultWorker가 계속 해당 Queue를 참조하고 있었다.

수정: Pose/Face IPC Queue 인터페이스는 항상 생성하고 ResultWorker는 활성 Process Queue만 drain한다.

### 3. Hardware Process 인자 불일치

Hardware 함수는 Face Queue 인자를 포함하지만 Process 생성부에서는 Face 인자를 제거하여 `TypeError`가 발생할 수 있었다.

수정: Main/Pose/Face <-> Hardware Queue 인터페이스를 항상 유지하고 Hardware Process에 일관된 인자 목록을 전달한다.

### 4. Face 비활성화 방식

기존 방식: Face 코드를 여러 파일에서 직접 주석처리.

현재 방식:

```python
PROFILE_MODE = "POSE_ONLY"
```

`enable_face=False`이므로 Face Process와 Face Shared Memory는 실제로 생성하지 않는다. 나중에 `BOTH`로 변경하면 기존 Face 코드가 그대로 실행된다.

### 5. Shared Frame 반환 타입 불일치

`write_frame()`이 Preview에서는 tuple, 분석 중에는 bool을 반환하는 불일치가 있었다.

수정: 항상 `(pose_success, face_success)`를 반환한다.

### 6. Shared Ring slot 수

CameraWorker가 Manager 기본값과 다르게 `slot_count=32`를 강제로 사용하고 있었다.

수정: Manager 기본값 `FRAME_RING_SLOT_COUNT=4`를 사용한다. Reader는 밀린 프레임을 건너뛰고 최신 프레임을 처리한다.

## 확인한 Calibration / Measurement 흐름

- Calibration 버튼 -> MediaPipe Landmark/Feature 수집 -> baseline 저장
- Measurement 버튼 -> 기존 baseline load -> GRU 시작
- Measurement 시작 시 새 Calibration을 실행하지 않음
- Calibration 중 Pose/Face State는 Main과 Hardware로 계속 전달 가능

## 테스트

- Python `compileall` 통과
- 주요 파일 AST parse 통과
- `POSE_ONLY`, `FACE_ONLY`, `BOTH` Profile resource gating 검사 통과
- Hardware Main/Pose/Face State/Event IPC spawn smoke test 통과
- POSE_ONLY Shared Frame 전달 검사 통과
- BOTH에서 Pose/Face가 동일 frame_id/timestamp의 Main frame을 받는 검사 통과

실제 Raspberry Pi Camera + MediaPipe model + TFLite model을 포함한 end-to-end 하드웨어 실행은 대상 장치에서 최종 확인해야 한다.

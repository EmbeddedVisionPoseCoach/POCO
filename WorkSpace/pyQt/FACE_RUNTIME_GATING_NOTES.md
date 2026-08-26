# Face Runtime Gating 수정

현재 기본 모드는 `PROFILE_MODE = "POSE_ONLY"`입니다.

## POSE_ONLY에서 실제 실행되는 항목

- Main Process / PyQt / Camera: 실행
- Pose Process / MediaPipe Pose: 실행
- Hardware Process: 실행
- Face Process / MediaPipe Face: 실행하지 않음
- Main -> Face Shared Frame: 전달하지 않음
- Hardware -> Face State/Event: 전달하지 않음
- Face -> Hardware State/Event: Hardware가 읽지 않음
- ResultWorker의 Face Queue polling: 실행하지 않음

Face용 Queue/Signal 객체 정의는 향후 `BOTH` 모드 확장을 위해 인터페이스로만 남아 있습니다.
`POSE_ONLY`에서는 이 객체들에 대한 런타임 송수신이 발생하지 않습니다.

## Hardware Process 변경

Hardware Process에 `enable_pose`, `enable_face`를 전달합니다.
따라서 비활성 Vision Profile의 State/Event Queue는 poll/broadcast/ACK 대상에서 제외됩니다.

`PROFILE_MODE = "BOTH"`로 바꾸면 같은 코드가 Pose/Face 양쪽 IPC를 다시 활성화합니다.

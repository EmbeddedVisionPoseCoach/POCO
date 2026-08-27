# IR → IMU Offset → Pose Calibration + Motor3 Gimbal

> 최신 전체 설명은 `README_TOTAL_HARDWARE_INTEGRATION_20260826.md`를 기준으로 한다.

```text
Calibration Start
→ IR stable 확인
→ IMU pitch_offset / roll_offset 측정
→ hardware_control.json에 Offset 기록
→ Pose Calibration 시작 + Motor3 Gimbal ON
→ Pose baseline 저장
→ Measurement에서도 같은 세션 Offset으로 Gimbal 유지
```

- IMU Offset 중 Motor3/4 OFF
- Pose Calibration + Measurement에서 Motor3 ON
- Motor4는 항상 `pass`
- IR lost 시 Motor3 gate OFF
- Pitch가 Deadband 안에서 0이면 Servo3 read/write packet 없음
- MediaPipe landmark는 Hardware에서 수신만 하고 현재 제어에는 사용하지 않음
- PID/LPF/Deadband/IR/Motor 제어 설정은 `data/settings/hardware_control.json`
- IR 실시간 값은 JSON이 아니라 Hardware State IPC로 Main/Pose/Face에 배포

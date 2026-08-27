# IMU + Motor3 Integration

> 최신 전체 설명은 `README_TOTAL_HARDWARE_INTEGRATION_20260826.md`를 기준으로 한다.

- Servo 3 = `wrist_flex`: 실제 제어
- Servo 4 = `wrist_roll`: `pass`
- IMU Pitch Offset 제거 → LPF → Deadband → PID → Output LPF → Motor3
- Motor3 방향 sign과 PID/LPF 값은 이제 `hardware_control.json`에서 변경 가능
- 실행 중 PyQt는 `UPDATE_CONFIG` IPC로 Hardware Process에 저장/반영 요청
- `servo_calibration_result.json`의 Servo3 `max_speed`가 `null`이면 안전상 실제 제어를 차단

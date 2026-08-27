# Motor / IMU Integration Notes

> 최신 기준: `README_TOTAL_HARDWARE_INTEGRATION_20260826.md`

```text
ADXL345 Pitch
→ pitch_offset 제거
→ IMU LPF
→ Deadband
→ Pitch PID
→ PID Output LPF
→ correction_pitch_speed_deg_s
→ dt 적분
→ Servo3 wrist_flex target
```

- Servo3 PID speed가 0이면 current-angle read와 move packet을 모두 보내지 않는다.
- Servo4 함수는 현재 `pass`다.
- Motor3 direction sign은 `data/settings/hardware_control.json`의 `control.motor.motor3.direction_sign`으로 변경한다.
- Pose landmark는 수신/보관만 하고 제어에는 아직 사용하지 않는다.

# Current Integration

현재 최신 통합 설명은 아래 문서를 기준으로 한다.

1. `README_TOTAL_HARDWARE_INTEGRATION_20260826.md` - 전체 Vision/Hardware 흐름
2. `README_MOTOR_CONSTANT_INIT_UPDATE_20260827.md` - 최신 Motor 반영 및 IR/IMU 코드 상수 초기화 변경

핵심 흐름:

```text
IR stable check
→ IMU Offset
→ Pose Calibration + Motor3 Gimbal
→ Measurement Gimbal
```

IR/IMU Service 생성자는 JSON 값을 직접 받지 않고 `hardware_constants.py`의 코드 상수로 기본 초기화한다.
`hardware_control.json`은 PyQt에서 저장한 Runtime tuning/override와 IMU calibration 기록을 관리한다.

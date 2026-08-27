# IMU Hardware Integration

현재 Hardware Process는 **ADXL345 IMU만 실제 동작**한다.

## 실행 흐름

1. Hardware Process 시작
2. ADXL345 I2C-1 / `0x53` 초기화
3. 3초간 기준 자세 Calibration
4. 50 Hz로 X/Y/Z 읽기
5. Pitch/Roll 계산
6. 기준 자세 대비 상대각 계산
7. IMU Low Pass Filter (`alpha=0.2`)
8. Dead Band (`0.5 deg`)
9. Pitch/Roll PID
10. PID 최종 출력 Low Pass Filter (`alpha=0.2`)
11. 최신 Hardware State를 Main/Pose에 20 Hz로 전달

## Hardware State 핵심 값

```python
state["imu"]["pitch_deg"]
state["imu"]["roll_deg"]
state["imu"]["correction_pitch_deg"]
state["imu"]["correction_roll_deg"]
state["imu"]["correction_pitch_speed_deg_s"]
state["imu"]["correction_roll_speed_deg_s"]
```

- `pitch_deg`, `roll_deg`: Calibration 기준 자세에서 현재 모니터가 얼마나 기울었는지
- `correction_*_deg`: 기준 자세로 돌아가기 위해 필요한 반대 방향 각도
- `correction_*_speed_deg_s`: PID가 만든 보정 속도
- **실제 모터 관절각/PWM 명령이 아니다.** 모터 제어 담당 계층이 이 값을 받아 최종 변환한다.

## MediaPipe Landmark

Pose Process에서 Hardware Process로 오는 최신 `POSE_STATE`에 이미 다음 값이 포함된다.

```python
pose_state["frame_id"]
pose_state["landmark_valid"]
pose_state["landmarks"]
```

Hardware Process는 이를:

```python
latest_pose_frame_id
latest_pose_landmarks
latest_pose_landmark_valid
```

에 저장한다.

현재는 **수신/보관만 하며 IMU/PID/모터 로직에는 사용하지 않는다.**

## IMU 재 Calibration

Main에서 필요할 경우:

```python
self.send_hardware_command({"type": "IMU_CALIBRATE"})
```

을 보내면 Hardware Process가 새 3초 Calibration을 시작한다.

## 현재 제외

- IR 센서
- Face 기반 Hardware 판단
- 모터/서보/PWM/Serial 제어
- MediaPipe landmark 기반 제어 조건
- PyBullet Jacobian/IK

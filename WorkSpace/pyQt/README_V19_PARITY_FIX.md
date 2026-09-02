# VisionPOCO Direct IMU V1.9 Parity Fix

이 패치는 standalone `imu_xy_direct_pid_tuner V1.9`와 POCO Motor3/4의 **실제 raw motion 방향과 튜닝값**을 맞추기 위한 수정입니다.

## 핵심 수정

- 매핑 유지: `Motor3 <- IMU Y`, `Motor4 <- IMU X`
- IMU PID 입력은 Pitch/Roll 각도가 아니라 filtered X/Y의 g 오차 그대로 사용
- POCO `MotorController`의 `COMMAND_TO_URDF_DIRECTION=-1`을 고려해 Motor34 TEAM-angle sign을 `+1/+1`로 보정
  - standalone tuner의 `-1/-1`과 **실제 raw 방향은 동일**
- hardware config version 5
  - 기존 v4에 남아 있던 `alpha=0.20`, `sign=-1` 등 stale control 값을 1회 V1.9 기준으로 초기화
  - calibration 기록과 motor enabled 상태는 보존
- V1.9 기본값
  - IMU 100Hz
  - LPF alpha 0.08
  - deadband 0.010g
  - M3(Y): Kp 120, Ki 0, Kd 0
  - M4(X): Kp 120, Ki 0, Kd 0
  - PID output limit ±24deg/s
  - Motor3/4 command 100Hz
  - Servo speed 500 / acc 12
- 시작 로그에 실제 런타임 PID/필터/모터 설정 출력 추가

## 정상 시작 로그

```text
[IMU] ... sample=100Hz alpha=0.08
[IMU CONTROL] M3<-Y Kp=120.0 Ki=0.0 Kd=0.0 | M4<-X Kp=120.0 Ki=0.0 Kd=0.0 | deadband=0.010g limit=±24.0deg/s Dalpha=0.15
[MOTOR34 CONTROL] command=100Hz speed=500 acc=12 M3<-Y TEAMsign=+1 M4<-X TEAMsign=+1
```

## 왜 sign이 tuner와 숫자가 다른가?

Standalone tuner는 `angle -> raw` 변환에 servo calibration direction만 사용합니다.
POCO는 `MotorController`에서 `TEAM angle -> URDF angle` 방향값 `-1`을 추가로 적용합니다.
따라서 raw motion을 동일하게 만들려면:

```text
standalone tuner sign = -1
POCO TEAM-angle sign  = +1
```

이어야 합니다.

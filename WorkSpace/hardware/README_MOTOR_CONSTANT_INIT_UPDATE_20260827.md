# Vision Pose Coach - Motor 최신본 + Hardware Constant Init 반영

## 1. 이번 변경의 핵심

기존 Hardware Process는 `hardware_control.json`을 먼저 읽은 뒤 아래처럼 JSON 값을 생성자에 직접 전달했다.

```python
IRSensorService(pin=..., active_low=..., sample_hz=...)
ADXL345IMUService(bus_number=..., address=..., sample_hz=...)
```

이번 버전부터는 이 구조를 제거했다.

```python
ir = IRSensorService()
imu = ADXL345IMUService()
motor = MonitorMotorService()
```

서비스 객체는 코드 상수로 먼저 생성되고, JSON은 객체가 생성된 이후 `apply_config()` / `apply_control_config()`에서 저장된 Runtime tuning/override를 적용한다.

## 2. Hardware 기본 상수

신규 파일:

```text
pyQt/services/hardware_constants.py
```

기본값:

```python
IR_PIN = 17
IR_ACTIVE_LOW = True
IR_SAMPLE_HZ = 20.0
IR_STABLE_DETECT_SEC = 0.5
IR_LOST_GRACE_SEC = 0.3
IR_CHECK_TIMEOUT_SEC = 5.0

IMU_BUS = 1
IMU_ADDRESS = 0x53
IMU_SAMPLE_HZ = 50.0
IMU_CALIBRATION_SEC = 3.0
IMU_LPF_ALPHA = 0.20
IMU_DEADBAND_DEG = 0.50
```

PID 기본값도 같은 파일에 정의해 기본값의 출처를 한 곳으로 모았다.

## 3. JSON의 역할

`data/settings/hardware_control.json`은 삭제하지 않는다.

이유는 향후 PyQt 설정 화면에서 PID/LPF/Deadband/Motor 설정 등을 수정하고 저장해야 하기 때문이다.

따라서 역할을 이렇게 분리한다.

```text
hardware_constants.py
  -> 코드가 가지는 Power-on 기본값 / fallback

hardware_control.json
  -> 사용자가 저장한 설정값 / runtime override / IMU offset 기록

Hardware Process
  -> Service를 상수 기본값으로 생성
  -> JSON load
  -> apply_config / apply_control_config
  -> open
```

즉 생성자는 JSON 구조를 알 필요가 없다.

## 4. Motor 최신화

사용자가 새로 제공한 `motor_control(1).zip`을 기준으로 다음 파일을 다시 반영했다.

```text
motor_control/
servo_calibration_result.json
servo_manual_control.py
multi_servo.py
servo_calibration.py
```

업로드된 최신본의 `motor_control/` 핵심 패키지는 이전 통합본과 동일한 API/내용이었으므로 `MonitorMotorService`의 호출 방식은 유지한다.

```python
arm.move_joint(
    "wrist_flex",
    angle=target_angle,
    speed=command_speed,
    wait=False,
)
```

Servo4 `wrist_roll`은 기존 요청대로 계속 `pass` 상태다.

## 5. 현재 Calibration / Measurement 흐름

```text
Calibration Start
  -> IR stable check
  -> IMU offset calibration
     Motor3/4 OFF
  -> IMU offset JSON 기록
  -> Pose Calibration
     IMU PID + Motor3 Gimbal ON
  -> Pose baseline 저장

Measurement
  -> 같은 세션 IMU offset 사용
  -> IMU PID + Motor3 Gimbal ON
  -> Deadband/PID output=0 이면 Servo packet 없음
```

MediaPipe landmark는 Pose Process에서 Hardware Process로 전달되지만 현재는 저장만 하고 제어에는 사용하지 않는다.

## 6. 검증

- 전체 `pyQt` compileall PASS
- Hardware logic self-test PASS
- IR 생성자 default == hardware_constants PASS
- IMU 생성자 default == hardware_constants PASS
- IMU calibration/PID fake test PASS
- Motor3 0 output에서 read/move packet 0건 PASS
- Motor3 correction 발생 시 read/move command 발생 PASS

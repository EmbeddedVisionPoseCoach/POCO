# Vision Pose Coach - Hardware / Vision Final Integration

기준: `VisionPoseCoach_IR_IMU_motor3_calibration_flow.zip` + 이후 대화에서 확정한 Config/IR Runtime State 구조.

## 1. 최종 전체 실행 구조

```text
Main Process (PyQt)
├─ CameraWorker / Picamera2
├─ HardwareRuntimeStateStore       # IR/IMU/Motor 최신값 메모리 저장
├─ HardwareConfigService           # JSON 설정 읽기 / Worker 미실행 시 저장
└─ VisionProcessManager
    ├─ Pose Process
    │   └─ MediaPipe -> landmark -> Pose→Hardware 최신 State
    ├─ Face Process                # POSE_ONLY에서는 Runtime OFF
    └─ Hardware Process
        ├─ IR Sensor BCM17
        ├─ ADXL345 IMU
        ├─ Pitch/Roll LPF + PID + PID Output LPF
        ├─ Motor3 wrist_flex       # 실제 제어
        └─ Motor4 wrist_roll       # pass
```

Camera frame은 Queue가 아니라 Shared Memory Ring을 유지한다.
Face 관련 인터페이스는 남아 있지만 `PROFILE_MODE="POSE_ONLY"`에서 Process/SharedMemory/Traffic은 비활성화된다.

---

## 2. Calibration 최종 순서

```text
[초기값 준비]
Camera Preview

[초기값 측정]
        ↓
1) IR CHECK
   BCM17 / LOW=감지
   stable_detect_sec 동안 연속 감지
        ↓
2) IMU OFFSET CALIBRATION
   Motor3 OFF
   Motor4 OFF
   pitch_offset / roll_offset 측정
        ↓
3) hardware_control.json에 마지막 Offset 기록
        ↓
4) Pose Calibration 시작
   MediaPipe baseline 수집
   + IMU PID 짐벌 동작
   + Motor3 wrist_flex 실제 제어
   + Motor4 pass
        ↓
5) Pose baseline 저장
```

IR이 IMU Offset 중 끊기면 Offset 측정을 취소한다.
Pose Calibration 중 IR이 끊기면 Motor3를 즉시 gate OFF하고 Pose Calibration도 실패 처리한다.

---

## 3. Measurement

Measurement에서는 IMU Offset을 다시 잡지 않는다.
현재 앱 실행 세션에서 IR→IMU Offset Calibration을 완료한 값을 그대로 사용한다.

```text
MediaPipe / GRU Measurement
        +
ADXL345 Pitch
  -> Offset 제거
  -> IMU LPF
  -> Deadband
  -> PID
  -> PID Output LPF
  -> Motor3
```

새 앱 실행에서는 JSON에 이전 Offset이 있어도 자동 활성화하지 않는다.
새 세션은 다시 IR→IMU Offset을 통과해야 Measurement가 허용된다.

---

## 4. 0도일 때 Motor Packet 없음

`pitch_deg`가 `deadband_deg` 안이면 0도로 취급한다.
이 순간 PID 출력과 PID Output LPF 잔류값도 0으로 만든다.

MotorService에서는 command speed가 0이면 즉시 return한다.

```text
Pitch == 0 (Deadband)
 -> PID Output = 0
 -> Servo current angle read 안 함
 -> move_joint() 안 함
 -> Motor3 Packet 없음
```

---

## 5. MediaPipe Landmark

Pose Process에서 Hardware Process로 최신 landmark가 전달된다.

```text
Pose Process
 -> frame_id
 -> landmark_valid
 -> landmarks
 -> Hardware Process latest_pose_landmarks
 -> pass
```

현재 IR/IMU/PID/Motor 조건에는 사용하지 않는다.
향후 자세 조건/안전 조건을 연결하기 위한 인터페이스만 준비되어 있다.

---

## 6. IR Runtime State 공유

IR은 실시간 센서이므로 JSON에 계속 저장하지 않는다.
20Hz 수준의 값을 JSON에 쓰면 파일 I/O와 SD Card write가 불필요하게 증가한다.

Hardware Process가 아래 State를 메모리로 유지한다.

```python
hardware_state["ir"] = {
    "available": True,
    "pin": 17,
    "raw_value": 0,
    "detected": True,
    "stable_detected": True,
    "detected_duration_sec": 1.2,
    "lost_duration_sec": 0.0,
    ...
}
```

같은 Hardware State를 `maxsize=1` 최신값 채널로 Main/Pose/Face에 배포한다.

Main에서는 `HardwareRuntimeStateStore`에 저장한다. Main Process 내부의 다른 UI/알림/리포트 모듈은 같은 singleton을 가져올 수 있다.

```python
from services.hardware_state_store import get_hardware_runtime_state_store

store = get_hardware_runtime_state_store()
ir_state = store.get_ir_state()
detected = store.is_ir_detected()
```

MainWindow에서도 편의 API를 제공한다.

```python
self.get_latest_ir_state()
self.is_ir_detected()
self.is_ir_detected(require_stable=True)
self.get_latest_imu_state()
self.get_latest_motor_state()
```

주의: 이 singleton은 **Main Process 안에서만 공유**된다. Pose/Face 같은 별도 Process에서는 메모리가 분리되므로 전달받은 `latest_hardware_state["ir"]`을 사용한다.
공통 accessor는 `pyQt/ipc/hardware_state_utils.py`에 있다.

---

## 7. Hardware Config JSON

파일:

```text
WorkSpace/data/settings/hardware_control.json
```

설정과 실시간 센서 상태를 분리한다.

### 저장되는 값

- 마지막 IMU pitch/roll Offset (기록용)
- IR pin / active-low / 감지 안정시간 / lost grace / timeout
- IMU sample Hz / Offset 측정시간 / LPF / Deadband
- Pitch PID P/I/D
- Roll PID P/I/D
- PID output limit
- PID derivative LPF
- PID output LPF
- Motor command Hz
- Motor PID speed deadband
- Motor3 enable / direction sign
- Motor4 설정 자리 (현재 실제 구현은 항상 pass)

### 저장하지 않는 Runtime 값

- IR detected 현재값
- PID integral 현재값
- previous error
- derivative 현재값
- PID output LPF 이전값
- 현재 Motor target / correction speed

이 값들은 Process 시작 시 Runtime 값으로 새로 만든다.

---

## 8. Config 클래스

`pyQt/services/hardware_config_service.py`

주요 API:

```python
service.load()
service.save(config)
service.get_control()
service.update_control(patch)
service.update_imu_calibration(pitch, roll, sample_count)
service.reload()
service.reset_defaults()
```

JSON 저장은 temp file을 만든 뒤 `os.replace()`로 교체한다.

---

## 9. PyQt에서 PID 값을 변경할 때

실행 중에는 Main Process와 Hardware Process가 JSON을 동시에 직접 쓰지 않는다.
Hardware Process를 write owner로 둔다.

예:

```python
self.save_hardware_control_config({
    "pid": {
        "pitch": {
            "kp": 8.5,
            "ki": 0.02,
            "kd": 0.12,
        }
    },
    "imu": {
        "lpf_alpha": 0.25,
        "deadband_deg": 0.4,
    }
})
```

실행 중 흐름:

```text
PyQt
 -> UPDATE_CONFIG IPC
 -> Hardware Process
 -> HardwareConfigService.update_control()
 -> hardware_control.json 저장
 -> IMU/PID/IR/Motor 설정 즉시 반영
 -> HARDWARE_CONFIG_UPDATED ACK
```

Hardware Process가 아직 시작되지 않았다면 Main의 ConfigService가 직접 저장하고 다음 Hardware 시작 때 읽는다.

---

## 10. Motor

### Servo 3

```text
ID 3 = wrist_flex
IMU Pitch PID -> correction speed -> dt 적분 -> target angle -> move_joint()
```

`servo_calibration_result.json`의 safe angle과 max_speed 규칙을 우회하지 않는다.
`max_speed=null`이면 Motor3를 열지 않고 Calibration을 차단한다.

### Servo 4

```python
def _control_motor4_wrist_roll(...):
    pass
```

현재 실제 패킷 없음.

---

## 11. LPF 위치

```text
ADXL345 Raw
 -> Pitch/Roll
 -> Offset 제거
 -> IMU LPF                  # 센서 노이즈 완화
 -> Deadband
 -> PID
 -> PID Output LPF           # 모터 속도 급변 완화
 -> Motor3
```

Deadband에서 0이 되면 PID Output LPF도 즉시 0으로 reset하여 잔류 출력으로 패킷이 나가는 것을 막는다.

---

## 12. 주요 신규 파일

```text
pyQt/services/hardware_config_service.py
pyQt/services/hardware_state_store.py
pyQt/ipc/hardware_state_utils.py
data/settings/hardware_control.json
```

기존 주요 파일 수정:

```text
pyQt/processes/hardware_process.py
pyQt/services/ir_service.py
pyQt/services/imu_service.py
pyQt/services/motor_service.py
pyQt/mainpyQt.py
pyQt/camera_worker_profile_all.py
pyQt/imu_debug.py
```

---

## 13. 검증 완료 항목

가짜 Hardware로 다음 로직을 검증했다.

- Config JSON 최초 생성
- JSON 부분 update 시 나머지 값 유지
- IMU Offset JSON 저장
- IR stable detection
- IR lost duration
- IMU 새 Process 시작 시 calibrated=False
- IMU Offset 후 calibrated=True
- 기울기 발생 시 PID output 발생
- 0도 복귀 후 PID output=0
- Motor3 0일 때 current-angle read 0건
- Motor3 0일 때 move command 0건
- Motor3 기울기 발생 시 read + move command 발생
- Motor4 pass
- 전체 Python compile / AST parse

실제 Raspberry Pi에서는 마지막으로 센서 축 방향과 Servo3 방향 sign, max_speed를 실기 확인해야 한다.

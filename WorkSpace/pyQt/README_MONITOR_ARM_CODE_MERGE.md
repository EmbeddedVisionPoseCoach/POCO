# POCO 모니터암 코드 병합 최종 README

> **대상 저장소**: `EmbeddedVisionPoseCoach/POCO`  
> **대상 브랜치**: `Feature_Multiprocessing`  
> **문서 작성 기준 HEAD**: `49f1069fa2806bcf76ce8827a7c4f1fa3bab7c37`  
> **작성 기준일**: 2026-08-30  
> **목적**: `WorkSpace/hardware/BH_CODE/`에 있던 팀원 모니터암 Standalone 코드를 POCO Multiprocessing 구조로 병합한 이력, 현재 최종 구조, 향후 수정 위치, BH_CODE 삭제 기준을 한 문서에서 추적할 수 있도록 정리한다.

---

## 0. 가장 중요한 결론

현재 POCO Runtime에서 **Motor1/2 모니터암 자동추종에 필요한 핵심 실행 로직은 `BH_CODE` 밖의 정식 POCO 구조로 통합되어 있다.**

현재 Runtime의 핵심 경로는 다음과 같다.

```text
PoseProcess
  └─ MediaPipe Pose landmark
       ↓
pose_to_hardware_q
       ↓
HardwareProcess
  ├─ ToFSensorService
  ├─ ToFUserXSource
  ├─ EyeGapVisionDistanceEstimator
  ├─ UserXFusion
  └─ 최종 fused user_x_m
       ↓
Motor12Controller
  ├─ 자동추종 상태/주기 제어
  ├─ SAFE_HOLD
  ├─ Rest
  └─ Recovery
       ↓
MonitorArmPlanner
  ├─ 목표 Monitor X 계산
  ├─ IK
  ├─ Deadband
  ├─ Joint/Path Safety
  └─ Recovery Planning
       ↓
MotorService
       ↓
MotorController
       ↓
Servo 1 + Servo 2 SyncWrite
```

따라서 `BH_CODE/pose_monitor_arm_controller.py` 또는 `BH_CODE/monitor_arm_motor_process.py`를 Runtime에서 직접 실행하거나 import하는 구조가 아니다.

단, **`BH_CODE` 폴더 전체를 삭제하기 전에 반드시 밖으로 옮겨야 하는 파일이 2개 있다.**

```text
WorkSpace/hardware/BH_CODE/requirements-monitor-arm.txt
WorkSpace/hardware/BH_CODE/TOF_HW843_SETUP.md
```

두 파일은 실행 코드 자체가 아니라 **새 Raspberry Pi 환경 재구축에 필요한 Python 의존성 및 HW-843 ToF 배선/I2C 설정 문서**이므로 먼저 보존해야 한다.

---

# 1. 병합의 기본 원칙

이번 병합은 팀원의 Standalone 프로그램을 POCO 안에서 그대로 한 번 더 실행하는 방식으로 하지 않았다.

다음 원칙으로 기능을 분리해서 기존 POCO 구조에 넣었다.

1. 실제 Serial/I2C 장치의 소유자는 `HardwareProcess` 하나로 유지한다.
2. Servo 1~4는 하나의 `MotorService`와 하나의 Motor Serial Bus를 공유한다.
3. Motor1/2는 두 관절이 하나의 IK 결과를 구성하므로 **순차 단일 명령이 아니라 SyncWrite 동시 명령**을 사용한다.
4. 센서 측정, 사용자 위치 융합, IK, Motor 전송 책임을 서로 분리한다.
5. 팀원 코드의 안전정책인 **ToF invalid → Vision 단독 제어 금지 → SAFE_HOLD**를 유지한다.
6. Rest는 정상 자동추종과 다른 **특수 자세 경로**로 분리한다.
7. Servo Calibration은 BH_CODE의 값을 사용하지 않고 POCO의 기존 Calibration JSON을 단일 기준으로 사용한다.
8. Motor3/4의 기존 짐벌 제어는 유지하면서 Motor1/2를 같은 HardwareProcess 안에 추가한다.

---

# 2. 현재 기준 핵심 파일

## 2.1 Runtime 통합 파일

| 역할 | 현재 사용 파일 |
|---|---|
| Hardware 전체 orchestration | `WorkSpace/pyQt/processes/hardware_process.py` |
| Motor1/2 상태·자동추종·Rest·Recovery | `WorkSpace/pyQt/services/motor12_controller.py` |
| 모니터암 목표/IK/Safety Planner | `WorkSpace/pyQt/services/monitor_arm_planner.py` |
| 2-Link Forward/Inverse Kinematics | `WorkSpace/pyQt/services/monitor_arm_kinematics.py` |
| Adaptive Speed 계산 | `WorkSpace/pyQt/services/monitor_arm_speed.py` |
| ToF user X / Vision 거리 / Fusion | `WorkSpace/pyQt/services/monitor_arm_user_x.py` |
| 실제/Fixed ToF Sensor Service | `WorkSpace/pyQt/services/tof_service.py` |
| Motor Hardware 전달 계층 | `WorkSpace/pyQt/services/motor_service.py` |
| 실제 STS3215 제어 및 SyncWrite | `WorkSpace/hardware/motor_control/controller.py` |
| 모니터암 고정 프로젝트 설정 | `WorkSpace/config/monitor_arm_settings.json` |
| Servo Calibration 단일 기준 | `WorkSpace/hardware/servo_calibration_result.json` |
| 최종 비실물 통합 테스트 | `WorkSpace/pyQt/hardware_logic_selftest.py` |

## 2.2 현재 Servo 기준

| Servo ID | Joint | Calibration 기준 파일 |
|---:|---|---|
| 1 | `shoulder_lift` | `WorkSpace/hardware/servo_calibration_result.json` |
| 2 | `elbow_flex` | `WorkSpace/hardware/servo_calibration_result.json` |
| 3 | `wrist_flex` | `WorkSpace/hardware/servo_calibration_result.json` |
| 4 | `wrist_roll` | `WorkSpace/hardware/servo_calibration_result.json` |

현재 통합 기준에서는 Servo 1~4의 `max_speed`를 `1000`으로 사용한다.

> **중요:** `BH_CODE`에 별도 Calibration 파일이나 과거 수치가 생기더라도 Runtime 기준으로 사용하지 않는다.  
> Servo Zero, Direction, Safe Angle Range, max_speed를 바꿔야 할 때는 반드시 `WorkSpace/hardware/servo_calibration_result.json`을 기준으로 확인한다.

---

# 3. BH_CODE 원본 → 현재 POCO 코드 대응 관계

이 절이 이번 병합에서 가장 중요한 추적표다.

---

## 3.1 `BH_CODE/pose_monitor_arm_controller.py`

이 파일은 팀원이 만든 **카메라 + Pose + ToF + Vision Fusion + Planner + Motor까지 한 프로그램에서 처리하는 Standalone Controller**였다.

### A. `JointCommand`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: class JointCommand
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_planner.py
Ctrl+F: class JointCommand
```

**병합 방식**

- 팀원 Planner가 반환하던 Shoulder/Elbow 목표각 표현을 별도 Planner 서비스의 결과 객체로 분리했다.
- Motor Hardware 접근은 포함하지 않는다.

---

### B. `MonitorArmPlanner`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: class MonitorArmPlanner
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_planner.py
Ctrl+F: class MonitorArmPlanner
```

**옮긴 핵심 기능**

- 사용자와 모니터 사이 목표 거리 적용
- `user_x_m`으로부터 목표 Monitor X 계산
- Deadband
- 한 번에 이동할 Monitor X 최대 Step 제한
- 현재 모니터 높이 유지
- 2-Link IK 호출
- Calibration Joint Range 검사
- Soft Joint Limit 검사
- 경로 Sample 검사
- Recovery Planning
- 작업 자세 복구 latch

**변경한 구조**

원본 Standalone에서는 Planner와 실행 Loop가 같은 파일에 있었지만, 현재는:

```text
Sensor/Fusion
    ↓
MonitorArmPlanner
    ↓
Motor12Controller
```

형태로 분리했다.

Planner는 **센서를 읽지 않고 Motor도 직접 움직이지 않는다.**

---

### C. `EyeMeasurement`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: class EyeMeasurement
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: class EyeMeasurement
```

눈 간격 측정 결과인 `gap_px`, 좌/우 눈 좌표를 저장하는 역할을 그대로 분리했다.

---

### D. `ToFUserXSource`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: class ToFUserXSource
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: class ToFUserXSource
```

**옮긴 기능**

```text
ToF Sensor range
+ sensor_origin_x_m
= POCO base 기준 user_x_m
```

그리고 계산된 `user_x_m`이 허용범위 안인지 검사한다.

현재 실제 거리 측정 자체는 이 클래스가 하지 않고:

```text
WorkSpace/pyQt/services/tof_service.py
```

의 Sensor Service를 감싸서 사용한다.

---

### E. `EyeGapVisionDistanceEstimator`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: class EyeGapVisionDistanceEstimator
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: class EyeGapVisionDistanceEstimator
```

**유지한 원리**

첫 유효 눈 간격을 ToF 기준 실제 거리와 매칭한 뒤, 눈 간격이 커지거나 작아지는 비율로 Vision 거리를 추정한다.

개념적으로:

```text
vision_distance
≈ reference_distance × reference_eye_gap / current_eye_gap
```

Vision 거리에는 EMA Filter가 적용된다.

---

### F. `UserXFusion`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: class UserXFusion
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: class UserXFusion
```

기본 설정:

```text
ToF    0.7
Vision 0.3
```

Vision 입력이 없으면 ToF 단독 값을 사용한다.

**ToF가 없을 때 Vision 단독으로 Motor를 움직이는 동작은 허용하지 않는다.**

---

### G. `measure_pose_eye_gap()`

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: def measure_pose_eye_gap
```

**현재 위치**

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: def measure_pose_eye_gap
```

**원본과 달라진 입력 경계**

팀원 Standalone은 MediaPipe Landmark 객체를 직접 받았지만, POCO는 Process 사이에 Landmark를 전달하므로 다음 직렬화 형식을 사용한다.

```python
[x, y, z, visibility]
```

현재 Pose 직렬화 위치:

```text
WorkSpace/pyQt/processes/pose_process.py
Ctrl+F: def serialize_pose_landmarks
```

현재 Eye Gap 계산은 Pose landmark index:

```text
LEFT_EYE_INDEX  = 2
RIGHT_EYE_INDEX = 5
```

를 사용한다.

Frame 크기는 현재 POCO 설정의:

```text
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
```

을 이용한다.

설정 위치:

```text
WorkSpace/modules/config.py
Ctrl+F: FRAME_WIDTH
Ctrl+F: FRAME_HEIGHT
```

---

### H. Standalone `main()` Loop

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
Ctrl+F: def main
```

이 부분은 **그대로 옮기지 않았다.**

이유는 POCO에 이미:

```text
MainProcess
PoseProcess
HardwareProcess
FaceProcess
```

구조가 있기 때문이다.

원본의 역할을 다음처럼 분산했다.

| 원본 Standalone 역할 | 현재 담당 |
|---|---|
| Camera/Pose landmark 획득 | `pose_process.py` |
| ToF 실제 장치 | `tof_service.py` |
| ToF + Vision Fusion | `hardware_process.py` + `monitor_arm_user_x.py` |
| IK / Safety Planning | `monitor_arm_planner.py` |
| Motor1/2 상태 및 명령주기 | `motor12_controller.py` |
| 실제 Servo 전송 | `motor_service.py` → `motor_control/controller.py` |

즉 `pose_monitor_arm_controller.py` 전체를 새 Process로 추가한 것이 아니라 **기능만 POCO 기존 Process 구조에 맞게 분해 병합**했다.

---

## 3.2 `BH_CODE/monitor_arm_motor_process.py`

이 파일은 팀원 Standalone에서 Motor1/2 명령을 별도 Multiprocessing Worker로 처리하던 코드다.

**원본 위치**

```text
WorkSpace/hardware/BH_CODE/monitor_arm_motor_process.py
```

현재는 이 파일 자체를 실행하지 않는다.

### A. Standalone Motor Process / Queue

**병합하지 않은 부분**

- 별도 Motor Process 생성
- 독자적인 Motor Queue
- 별도 Motor Serial 소유

**이유**

POCO에는 이미 `HardwareProcess`가 Hardware를 소유한다.

따라서 또 다른 Process가 `/dev/ttyACM0`을 열면 같은 Servo Bus에 중복 접근할 수 있다.

현재는:

```text
HardwareProcess
   └─ MotorService 1개
        ├─ Motor12Controller
        └─ Motor34Controller
```

구조다.

---

### B. Motor1/2 동시 이동

팀원 코드에서 Shoulder/Elbow를 한 Pair로 제어하던 의도는 다음 위치로 반영했다.

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def _move_normal_target
```

실제 전달:

```text
WorkSpace/pyQt/services/motor_service.py
Ctrl+F: def move_joints
```

최하단 Hardware:

```text
WorkSpace/hardware/motor_control/controller.py
Ctrl+F: def move_joints
```

최종적으로 Servo 1과 Servo 2는 **하나의 SyncWrite**로 목표가 전송된다.

Motor1과 Motor2를 다음처럼 따로 순차 호출하는 방식으로 되돌리면 안 된다.

```python
# 사용 금지 예시
move_joint("shoulder_lift", ...)
move_joint("elbow_flex", ...)
```

모니터암 IK 결과는 항상 Pair 단위로 취급한다.

---

### C. Adaptive Speed

현재 속도 계산 Helper:

```text
WorkSpace/pyQt/services/monitor_arm_speed.py
Ctrl+F: def select_speed
```

Motor12에서 실제 선택하는 위치:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def _select_tracking_speed
```

현재 정책은 두 관절 중 **현재각 → 목표각 오차가 더 큰 관절**을 기준으로 공통 Speed 하나를 고른다.

그리고 같은 Speed/Acc를 Shoulder/Elbow SyncWrite에 적용한다.

---

### D. Rest / 특수 자세 / Recovery

팀원 Motor Process의 일반 안전범위 밖 특수 자세 처리 개념은 현재 다음 위치로 분리했다.

**Motor12 상태/안전 판단**

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def move_to_rest
Ctrl+F: def resume_from_rest
Ctrl+F: def _move_recovery_target
```

**특수 SyncWrite 전달**

```text
WorkSpace/pyQt/services/motor_service.py
Ctrl+F: def move_joints_special
```

**실제 Servo 특수 명령**

```text
WorkSpace/hardware/motor_control/controller.py
Ctrl+F: def move_joints_special
```

Rest는 Calibration Safe Range 밖으로 갈 수 있으므로 일반 `move_joints()`와 섞지 않았다.

Recovery에서는:

- 목표가 아직 Calibration 밖이면 `move_joints_special()`
- 다시 Calibration Range 안으로 들어오면 일반 `move_joints()`

경로로 복귀한다.

특수 경로를 일반 자동추종에 사용하면 안 된다.

---

# 4. BH_CODE 밖에 원래부터 있던 팀원/공용 코드

다음 파일은 `BH_CODE`에서 새로 복사한 것이 아니다.

팀원 Standalone 코드가 이미 POCO Service를 import해서 사용하던 부분을 그대로 재사용했다.

---

## 4.1 `monitor_arm_kinematics.py`

```text
WorkSpace/pyQt/services/monitor_arm_kinematics.py
Ctrl+F: class TwoJointMonitorArm
```

담당:

- Forward Kinematics
- Inverse Kinematics
- 현재 Shoulder/Elbow 각도 ↔ Monitor 위치 계산

`MonitorArmPlanner`가 이 클래스를 사용한다.

IK 식을 수정해야 한다면 Planner가 아니라 이 파일이 핵심 위치다.

---

## 4.2 `monitor_arm_speed.py`

```text
WorkSpace/pyQt/services/monitor_arm_speed.py
Ctrl+F: def select_speed
Ctrl+F: def validate_speed_profile
```

Adaptive Speed 계산과 설정값 검증을 담당한다.

---

## 4.3 `tof_service.py`

팀원이 실제 HW-843/VL53L0X용 Service를 POCO Service 위치에 추가했고, 이번 병합에서 이를 HardwareProcess에 연결했다.

```text
WorkSpace/pyQt/services/tof_service.py
Ctrl+F: class ToFSensorService
Ctrl+F: class FixedToFSensorService
Ctrl+F: def create_tof_service
```

실제 Hardware import는 `open()` 안에서 지연 수행된다.

핵심 의존성:

```python
from adafruit_extended_bus import ExtendedI2C
import adafruit_vl53l0x
```

---

# 5. 전체 병합 작업 13단계 요약

## Step 1. MotorService 다축 SyncWrite 경로 추가

Motor1/2를 하나의 Pair로 명령하기 위해:

```text
motor12_controller
→ MotorService.move_joints()
→ MotorController.move_joints()
```

경로를 만들었다.

커밋 기준:

```text
Feat : MotorService 다축 동기 이동 기능 추가
```

---

## Step 2. Motor1/2 Readiness / Safety 검사 추가

Motor1과 Motor2 각각에 대해:

- Calibration 존재
- 예상 Servo ID
- `max_speed > 0`
- 유효한 Safe Angle Range
- Ping 성공

을 확인하고 **둘 다 정상이어야 Motor12 Ready**가 되도록 했다.

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def initialize
```

커밋:

```text
Feat : Motor1/2 하드웨어 준비 상태 및 안전검사 추가
```

---

## Step 3. Planner 분리

`BH_CODE/pose_monitor_arm_controller.py`의 Planner 계산을:

```text
WorkSpace/pyQt/services/monitor_arm_planner.py
```

로 분리했다.

커밋:

```text
Refactor : 모니터암 Planner 로직 서비스 계층으로 분리
```

---

## Step 4. ToF 사용자 X 계층 임시 분리

실제 ToF Service가 팀원 작업과 겹치던 시점에 충돌을 피하기 위해 임시:

```text
tof_service_2222.py
```

를 만들었다.

이 파일은 이후 실제 `tof_service.py` + `monitor_arm_user_x.py` 구조가 완성되면서 **최종 삭제 완료**했다.

커밋:

```text
Refactor : ToF 사용자 위치 입력 로직 서비스 계층으로 분리 (임시 파일)
```

최종 삭제 커밋:

```text
Remove : 임시 ToF 사용자 위치 서비스 제거
```

---

## Step 5. 모니터암 고정 설정 파일 분리

팀원 설정을 Runtime 고정 프로젝트 설정으로:

```text
WorkSpace/config/monitor_arm_settings.json
```

에 배치했다.

커밋:

```text
Chore : 모니터암 고정 설정 파일 추가
```

---

## Step 6. Motor12 ↔ Settings ↔ Planner 연결

`Motor12Controller`가 고정 설정을 받아 Planner와 제어 설정을 구성하도록 연결했다.

커밋:

```text
Feat : Motor12에 모니터암 고정 설정 및 Planner 연결
```

---

## Step 7. 일반 자동추종용 Pair 이동 구현

구현 사항:

- 현재 Motor1/2 각도 읽기
- 목표각과 현재각 오차 계산
- 큰 오차 기준 Adaptive Speed
- Calibration Range 검사
- Servo1+2 Pair SyncWrite

위치:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def _move_normal_target
```

커밋:

```text
Feat : Motor12 일반 추종 속도 및 동기 이동 경로 추가
```

---

## Step 8. Motor1~4 Hardware 필수 상태 통합

기존 Motor3/4뿐 아니라 Motor1/2도 Hardware 상태에서 동등하게 관리하도록 변경했다.

현재:

```text
HARDWARE_STATE["motor"]["motor12"]
HARDWARE_STATE["motor"]["motor34"]
HARDWARE_STATE["motor"]["motor1"]
HARDWARE_STATE["motor"]["motor2"]
HARDWARE_STATE["motor"]["motor3"]
HARDWARE_STATE["motor"]["motor4"]
```

를 제공한다.

전역:

```text
MOTOR_ENABLE
MOTOR_DISABLE
```

이 Motor1~4 모두에 적용된다.

커밋:

```text
Feat : Motor1~4 필수 하드웨어 준비 및 상태 통합
```

---

## Step 9. ToF + Vision 사용자 위치 Fusion 서비스 분리

새 파일:

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
```

으로 다음을 이동했다.

- `EyeMeasurement`
- `ToFUserXSource`
- `EyeGapVisionDistanceEstimator`
- `UserXFusion`
- `measure_pose_eye_gap()`

커밋:

```text
Refactor : 모니터암 사용자 위치 및 센서 융합 로직 서비스 분리
```

---

## Step 10. HardwareProcess에 실제 ToF + Vision Fusion 연결

`HardwareProcess`가 실제 ToF를 소유하고 PoseProcess의 최신 landmark를 받아 최종 `user_x_m`을 만들도록 연결했다.

위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: # F-1. Motor1/2 ToF + Vision 사용자 X 계산
```

안전정책:

```text
ToF 정상 + Vision 정상 → FUSED
ToF 정상 + Vision 없음 → TOF_ONLY
ToF 오류              → SAFE_HOLD
```

커밋:

```text
Feat : HardwareProcess에 ToF-Vision 사용자 위치 융합 연결
```

추가 보강:

```text
Fix : Motor12 Vision 입력 상태 변수 초기화
```

---

## Step 11. Motor12 자동추종 실제 연결

융합된 `user_x_m`을 Motor12Controller로 전달해 실제 자동추종을 활성화했다.

위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: motor12_requested =
```

현재 Motor1/2는 `CALIBRATING`, `MEASURING`에만 묶이지 않는다.

**유효한 ToF/Fusion user X가 존재하고 Motor12가 Ready이면 지속 추종한다.**

실제 Motor12 제어:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def update
```

현재:

```text
Controller 상태 계산 주기: 20 Hz
Servo 실제 명령:          5 Hz
```

커밋:

```text
Feat : Motor12 ToF-Vision 기반 자동추종 제어 연결
```

---

## Step 12. Rest / Recovery 특수 경로 연결

Rest가 Calibration 밖의 확인된 자세일 수 있기 때문에:

1. 일반 SyncWrite
2. 특수 SyncWrite
3. Recovery

를 분리했다.

관련 커밋:

```text
Feat : Motor12 특수 자세용 예외 SyncWrite 경로 추가
Feat : Motor12 Rest 및 안전 Recovery 제어 연결
```

현재 우선순위:

```text
1. Rest mode
2. ToF/Fusion invalid → SAFE_HOLD
3. Explicit Recovery
4. Normal Tracking
```

---

## Step 13. 임시 파일 제거 + 최종 통합 Self-test

임시:

```text
WorkSpace/pyQt/services/tof_service_2222.py
```

삭제 완료.

최종 Self-test:

```text
WorkSpace/pyQt/hardware_logic_selftest.py
```

현재 검증 범위:

- ToF → user X
- Pose Eye Gap
- Vision Distance
- ToF/Vision Fusion
- Motor1/2 Pair SyncWrite
- Motor12 5 Hz command gate
- SAFE_HOLD
- Rest 특수 SyncWrite
- Recovery
- Motor3/4 기존 경로
- 공통 MotorService 경계

최종 기준 커밋:

```text
49f1069fa2806bcf76ce8827a7c4f1fa3bab7c37
Test : Motor1~4 ToF-Vision 및 Rest Recovery 통합 self-test 보강
```

---

# 6. 현재 실제 Runtime 동작 순서

## 6.1 Pose 입력

PoseProcess:

```text
WorkSpace/pyQt/processes/pose_process.py
Ctrl+F: def serialize_pose_landmarks
```

에서 Landmark를:

```python
[x, y, z, visibility]
```

형태로 직렬화한다.

HardwareProcess가 최신 Pose State를 받는다.

---

## 6.2 ToF 측정

```text
WorkSpace/pyQt/services/tof_service.py
Ctrl+F: class ToFSensorService
```

설정:

```text
WorkSpace/config/monitor_arm_settings.json
"tof"
```

현재 기본:

```text
mode           = hardware
i2c_bus        = 3
i2c_address    = 41 (= 0x29)
sample_hz      = 20.0
filter_alpha   = 0.25
range          = 0.03 ~ 2.0 m
```

---

## 6.3 ToF user X 변환

```text
monitor_arm_user_x.py
Ctrl+F: class ToFUserXSource
```

계산:

```text
tof_user_x = sensor_origin_x_m + filtered_tof_range
```

그리고 사용자 X 안전범위를 확인한다.

---

## 6.4 Vision 거리 추정

```text
monitor_arm_user_x.py
Ctrl+F: class EyeGapVisionDistanceEstimator
```

첫 유효 Pose Eye Gap이 들어오면 당시:

```text
ToF user X - 현재 Monitor X
```

를 Vision Reference Distance로 설정한다.

이후 Eye Gap 변화로 거리 변화를 추정한다.

---

## 6.5 Fusion

```text
monitor_arm_user_x.py
Ctrl+F: class UserXFusion
```

기본:

```text
fused_user_x
= 0.7 × tof_user_x
+ 0.3 × vision_user_x
```

단, Vision이 없으면:

```text
fused_user_x = tof_user_x
```

ToF가 유효하지 않으면:

```text
fused_user_x = None
→ SAFE_HOLD
```

---

## 6.6 Planner

```text
monitor_arm_planner.py
Ctrl+F: def plan
```

입력:

```text
현재 shoulder/elbow 각도
fused user_x_m
Calibration Range
```

출력:

```text
JointCommand(
    shoulder_lift_deg,
    elbow_flex_deg
)
```

---

## 6.7 Motor1/2 명령

```text
motor12_controller.py
Ctrl+F: def _move_normal_target
```

에서 현재각과 목표각을 비교하고 Speed를 결정한 뒤:

```text
motor_service.py
Ctrl+F: def move_joints
```

를 호출한다.

최종적으로 Servo1/2가 같은 SyncWrite Packet으로 움직인다.

---

# 7. 추후 기능 수정 시 정확히 어디를 수정해야 하는가

라인 번호는 이후 커밋에서 바뀔 수 있으므로 **파일 경로 + Ctrl+F Anchor를 기준으로 찾는다.**

---

## 7.1 사용자와 모니터의 목표 거리를 바꾸고 싶을 때

파일:

```text
WorkSpace/config/monitor_arm_settings.json
```

찾을 항목:

```json
"distance": {
    "desired_user_monitor_distance_m": 0.5,
    "deadband_m": 0.015,
    "max_monitor_x_step_m": 0.02
}
```

- `desired_user_monitor_distance_m`: 사용자-모니터 목표 거리
- `deadband_m`: 이 정도 오차는 Motor를 움직이지 않음
- `max_monitor_x_step_m`: 한 Planner Cycle에서 Monitor X를 얼마나 이동시킬지 제한

**Planner 코드를 먼저 수정하지 말고 설정값으로 조절하는 것이 우선이다.**

---

## 7.2 ToF/Vision 비중을 바꾸고 싶을 때

파일:

```text
WorkSpace/config/monitor_arm_settings.json
```

항목:

```json
"fusion": {
    "tof_weight": 0.7,
    "vision_weight": 0.3
}
```

계산 구현:

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: class UserXFusion
```

단순 비율 변경이면 JSON만 수정한다.

---

## 7.3 Vision Eye Gap 민감도를 바꾸고 싶을 때

설정:

```text
WorkSpace/config/monitor_arm_settings.json
"fusion"
```

관련 값:

```text
minimum_eye_gap_px
minimum_vision_distance_m
maximum_vision_distance_m
vision_filter_alpha
```

계산 코드:

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
Ctrl+F: class EyeGapVisionDistanceEstimator
Ctrl+F: def measure_pose_eye_gap
```

---

## 7.4 눈 Landmark를 다른 index로 바꾸고 싶을 때

파일:

```text
WorkSpace/pyQt/services/monitor_arm_user_x.py
```

찾기:

```text
LEFT_EYE_INDEX
RIGHT_EYE_INDEX
```

현재:

```text
2 / 5
```

PoseProcess의 Landmark 형식이 바뀌면 반드시:

```text
WorkSpace/pyQt/processes/pose_process.py
Ctrl+F: def serialize_pose_landmarks
```

과 같이 확인한다.

---

## 7.5 ToF I2C Bus/주소/측정 범위를 바꾸고 싶을 때

먼저 설정:

```text
WorkSpace/config/monitor_arm_settings.json
"tof"
```

실제 Driver 변경이 필요하면:

```text
WorkSpace/pyQt/services/tof_service.py
Ctrl+F: class ToFSensorService
Ctrl+F: def open
Ctrl+F: def update
```

를 수정한다.

Planner나 Motor12Controller에는 ToF Hardware 코드를 넣지 않는다.

---

## 7.6 다른 거리센서로 교체하고 싶을 때

가장 안전한 방식은 기존 ToF Service의 외부 인터페이스를 유지하는 것이다.

현재 기대하는 핵심 형태:

```python
open()
close()
update()
read_distance_m()
get_state()
```

새 Sensor Service가 같은 의미의 인터페이스를 제공하도록 만든 뒤:

```text
WorkSpace/pyQt/services/tof_service.py
Ctrl+F: def create_tof_service
```

Factory에서 선택하도록 하는 것이 권장된다.

그렇게 하면:

```text
ToFUserXSource
UserXFusion
MonitorArmPlanner
Motor12Controller
```

는 수정하지 않아도 된다.

---

## 7.7 Motor1/2 자동추종 속도를 바꾸고 싶을 때

설정:

```text
WorkSpace/config/monitor_arm_settings.json
"control"
```

주요 값:

```text
command_hz
pose_speed
pose_acc
pose_speed_mode
pose_variable_min_speed
pose_variable_full_speed_error_deg
```

현재 핵심값:

```text
command_hz = 5.0
pose_speed = 800
pose_acc   = 20
```

Adaptive Speed 함수:

```text
WorkSpace/pyQt/services/monitor_arm_speed.py
Ctrl+F: def select_speed
```

Motor12 적용 위치:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def _select_tracking_speed
```

---

## 7.8 Motor1/2 실제 명령 빈도를 바꾸고 싶을 때

설정:

```text
monitor_arm_settings.json
control.command_hz
```

현재 `5.0 Hz`이다.

Motor12 update는 별도로 20 Hz이지만 실제 Servo Packet은 `command_hz` Gate를 통과할 때만 전송한다.

구현:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: 실제 Servo 명령은 5Hz gate
```

---

## 7.9 IK Geometry를 바꾸고 싶을 때

먼저 실제 치수 설정:

```text
WorkSpace/config/monitor_arm_settings.json
"geometry"
```

주요 값:

```text
shoulder_x_m
shoulder_z_m
upper_link_m
lower_link_m
upper_zero_angle_rad
lower_zero_angle_rad
monitor_offset_m
```

수학 자체 변경:

```text
WorkSpace/pyQt/services/monitor_arm_kinematics.py
Ctrl+F: class TwoJointMonitorArm
```

안전/목표 계획:

```text
WorkSpace/pyQt/services/monitor_arm_planner.py
Ctrl+F: class MonitorArmPlanner
```

실제 링크 길이만 바뀐 경우에는 가급적 JSON만 바꾼다.

---

## 7.10 Motor1/2 Safety Limit을 조정하고 싶을 때

프로젝트 Soft Limit:

```text
WorkSpace/config/monitor_arm_settings.json
"safety"
```

실제 Servo Calibration 기준:

```text
WorkSpace/hardware/servo_calibration_result.json
```

Planner Safety:

```text
WorkSpace/pyQt/services/monitor_arm_planner.py
Ctrl+F: soft_joint_limits
Ctrl+F: path_samples
```

> **주의:** Soft Limit을 넓힌다고 실제 Calibration Range까지 자동으로 넓어지는 것이 아니다.

---

## 7.11 Servo Zero / Direction / 실제 가동범위를 수정할 때

단일 기준:

```text
WorkSpace/hardware/servo_calibration_result.json
```

Motor 변환 구현:

```text
WorkSpace/hardware/motor_control/
```

`BH_CODE/monitor_arm_settings.json` 또는 BH_CODE 안의 과거 값으로 수정하지 않는다.

---

## 7.12 Rest 자세를 바꾸고 싶을 때

설정:

```text
WorkSpace/config/monitor_arm_settings.json
"postures" → "rest"
```

현재:

```text
shoulder_lift_deg = 107.75
elbow_flex_deg    = -92.55
speed_cap         = 200
acc_cap           = 10
```

실행:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def move_to_rest
```

Rest는 일반 Calibration 경로 밖일 수 있으므로:

```text
move_joints_special()
```

을 사용한다.

**일반 `move_joints()`로 단순 교체하지 않는다.**

---

## 7.13 Rest를 실제 UI/Main에서 요청할 때

Hardware Event:

```python
{
    "type": "MOTOR12_REST",
    "confirmed": True
}
```

처리 위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: if event_type == "MOTOR12_REST"
```

`confirmed=True`가 없으면 실제 Rest 이동을 수행하지 않고 확인 필요 Event를 반환한다.

이 확인은 Rest가 일반 Calibration Range 밖의 특수 자세일 수 있기 때문에 의도적으로 넣은 안전장치다.

---

## 7.14 Rest 후 자동추종으로 복귀할 때

Hardware Event:

```python
{
    "type": "MOTOR12_RESUME"
}
```

처리 위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: if event_type == "MOTOR12_RESUME"
```

Motor12:

```text
WorkSpace/pyQt/services/motor12_controller.py
Ctrl+F: def resume_from_rest
```

`resume_from_rest()` 자체는 즉시 Servo를 강제로 작업자세로 순간 이동시키지 않는다.

다음 `update()`부터 Planner가 **안전범위 방향으로 단계적으로 Recovery**한다.

---

## 7.15 Motor1~4 전체 제어 허용/차단

Hardware Event:

```python
{"type": "MOTOR_ENABLE"}
```

또는:

```python
{"type": "MOTOR_DISABLE"}
```

처리 위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: if event_type == "MOTOR_ENABLE"
Ctrl+F: if event_type == "MOTOR_DISABLE"
```

현재 두 Event는 Motor1/2와 Motor3/4를 모두 함께 변경한다.

---

## 7.16 Motor1/2 자동추종을 특정 상태에서만 켜고 싶을 때

현재는 유효한 Fusion 입력이 있으면 Main mode와 무관하게 Motor12 추종 요청을 만든다.

현재 위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: motor12_requested =
```

현재 개념:

```python
motor12_requested = bool(
    latest_monitor_arm_input_state.get("valid", False)
)
```

추후 예를 들어 특정 Main Mode, 사용자 버튼, 자세 상태를 Gate로 추가하려면 **이 지점 또는 Motor12에 전달하는 `context["motor12"]["control_active"]` 앞**에서 조건을 추가하는 것이 가장 명확하다.

Planner에 UI/Workflow 조건을 넣지 않는다.

---

## 7.17 추후 GRU/Posture 결과를 Motor1/2 Gate로 사용하고 싶을 때

현재 Motor1/2 자동추종은 GRU를 사용하지 않는다.

Pose State는 이미 HardwareProcess Context에:

```python
"context['pose']"
```

형태로 존재한다.

권장 추가 위치:

```text
WorkSpace/pyQt/processes/hardware_process.py
Ctrl+F: motor12_requested =
```

예:

```text
유효한 user_x
AND 원하는 posture/inference 상태
→ motor12_requested = True
```

처럼 Gate를 추가한다.

**GRU 결과를 `MonitorArmPlanner` 내부에 직접 넣지 않는 것이 좋다.**

Planner는 계속 순수한 위치/IK/Safety 계산 계층으로 유지한다.

---

# 8. Hardware State에서 확인해야 할 값

HardwareProcess는 최종 State에 다음을 제공한다.

```text
HARDWARE_STATE
├─ imu
├─ tof
├─ monitor_arm_input
└─ motor
    ├─ bus
    ├─ motor12
    ├─ motor34
    ├─ motor1
    ├─ motor2
    ├─ motor3
    └─ motor4
```

## 8.1 ToF/Fusion 문제 확인

```text
state["tof"]
state["monitor_arm_input"]
```

`monitor_arm_input`의 주요 값:

```text
available
valid
tof_user_x_m
vision_user_x_m
user_x_m
fusion_mode
eye_gap_px
last_error
```

`fusion_mode` 해석:

```text
FUSED     : ToF + Vision
TOF_ONLY  : ToF만 사용
SAFE_HOLD : Motor1/2 자동추종 금지
```

---

## 8.2 Motor1/2 문제 확인

```text
state["motor"]["motor12"]
```

에서:

```text
available
enabled
ready
control_active
hold_reason
last_error
```

및 Motor1/2 개별 상태를 확인한다.

대표 `hold_reason`:

```text
REST
SAFE_HOLD
DISABLED
NOT_READY
RECOVERY
RECOVERY_COMPLETE
ERROR
```

---

# 9. `BH_CODE` 파일별 최종 삭제 판단

현재 `BH_CODE`의 파일을 하나씩 분류하면 다음과 같다.

---

## 9.1 삭제 전에 반드시 밖으로 옮겨야 하는 파일

### 1) `requirements-monitor-arm.txt`

현재 위치:

```text
WorkSpace/hardware/BH_CODE/requirements-monitor-arm.txt
```

내용:

```text
mediapipe>=0.10
opencv-python>=4.8
pyserial>=3.5
adafruit-circuitpython-vl53l0x>=1.2
adafruit-extended-bus>=1.0
```

**판정: 지금 바로 삭제하면 안 됨. 먼저 이동 또는 프로젝트 requirements에 병합.**

추천 이동 위치:

```text
WorkSpace/hardware/requirements-monitor-arm.txt
```

또는 최종적으로 프로젝트 공용 requirements가 있다면 거기에 합친다.

특히 다음 두 패키지는 실제 ToF Service 재설치에 중요하다.

```text
adafruit-circuitpython-vl53l0x
adafruit-extended-bus
```

현재 `WorkSpace/hardware/STServo_Python/requirements.txt`는 `pyserial`만 포함하므로 이 파일을 아무 대책 없이 삭제하면 새 환경 구축 정보가 사라진다.

---

### 2) `TOF_HW843_SETUP.md`

현재 위치:

```text
WorkSpace/hardware/BH_CODE/TOF_HW843_SETUP.md
```

**판정: 지금 바로 삭제하면 안 됨. 먼저 이동 후 현재 구조에 맞게 경로 수정.**

추천:

```text
WorkSpace/hardware/TOF_HW843_SETUP.md
```

이 문서에는:

- Raspberry Pi 5 배선
- GPIO22 / GPIO23
- `/dev/i2c-3`
- `dtoverlay=i2c3-pi5,pins_22_23`
- I2C Address `0x29`
- `i2cdetect -y 3`
- Python package 설치
- ToF invalid 시 SAFE_HOLD 안전정책

이 들어 있으므로 Hardware 재구축 시 필요하다.

단, 현재 문서의 Standalone 실행 명령과 `BH_CODE` 경로는 최종 POCO 구조에 맞게 수정해야 한다.

---

## 9.2 Runtime에는 필요 없지만 문서 보존 여부를 팀이 결정할 파일

아래 3개는 **삭제해도 현재 Runtime은 깨지지 않는다.**

다만 과거 설계/학습 자료가 필요하면 `docs/archive/` 같은 곳에 보관할 수 있다.

### 3) `MONITOR_ARM_CODE_STUDY_GUIDE.md`

```text
WorkSpace/hardware/BH_CODE/MONITOR_ARM_CODE_STUDY_GUIDE.md
```

- 긴 학습/코드 해설 자료
- Runtime import 대상 아님
- 현재 최종 코드 위치는 이 README가 대신 설명함

**판정: 실행 관점 삭제 가능 / 학습 기록이 필요하면 Archive 권장**

---

### 4) `MONITOR_ARM_CONTROL.md`

```text
WorkSpace/hardware/BH_CODE/MONITOR_ARM_CONTROL.md
```

**판정: 실행 관점 삭제 가능 / 과거 설계 기록이 필요하면 Archive 가능**

---

### 5) `POSE_MONITOR_ARM_CONTROL.md`

```text
WorkSpace/hardware/BH_CODE/POSE_MONITOR_ARM_CONTROL.md
```

**판정: 실행 관점 삭제 가능 / 과거 설계 기록이 필요하면 Archive 가능**

---

## 9.3 병합 완료 후 삭제 가능한 Standalone 실행 코드

### 6) `pose_monitor_arm_controller.py`

```text
WorkSpace/hardware/BH_CODE/pose_monitor_arm_controller.py
```

핵심 기능은 다음으로 이동/통합됨:

```text
monitor_arm_planner.py
monitor_arm_user_x.py
tof_service.py
hardware_process.py
motor12_controller.py
```

Standalone `main()`은 POCO Process 구조와 중복되므로 사용하지 않는다.

**판정: 삭제 가능**

---

### 7) `monitor_arm_motor_process.py`

```text
WorkSpace/hardware/BH_CODE/monitor_arm_motor_process.py
```

기능은:

```text
Motor12Controller
MotorService
motor_control/controller.py
```

로 통합됨.

별도 Motor Process를 실행하면 오히려 공용 Serial Bus 소유 정책과 충돌할 수 있다.

**판정: 삭제 가능**

---

## 9.4 삭제 가능한 수동 Test UI / Visualizer

### 8) `manual_motor12_limit_ui.py`

```text
WorkSpace/hardware/BH_CODE/manual_motor12_limit_ui.py
```

팀원 Standalone 수동 테스트용 UI.

최종 POCO Runtime 필수 파일이 아니다.

**판정: 삭제 가능**

---

### 9) `manual_vertical_ik_ui.py`

```text
WorkSpace/hardware/BH_CODE/manual_vertical_ik_ui.py
```

Standalone IK/수동 테스트 도구.

**판정: 삭제 가능**

---

### 10) `monitor_arm_visualizer.py`

```text
WorkSpace/hardware/BH_CODE/monitor_arm_visualizer.py
```

Standalone 시각화 도구.

**판정: 삭제 가능**

> 단, 향후 IK 디버깅용 Tool로 계속 쓰고 싶다면 `WorkSpace/tools/` 같은 별도 위치에 Archive하는 것은 가능하다. Runtime에는 필요 없다.

---

## 9.5 중복 설정 파일

### 11) `monitor_arm_settings.json`

```text
WorkSpace/hardware/BH_CODE/monitor_arm_settings.json
```

현재 정식 기준:

```text
WorkSpace/config/monitor_arm_settings.json
```

문서 작성 시점에는 현재 정식 설정 파일에 동일한 프로젝트 기준값이 반영되어 있다.

**판정: 삭제 가능**

향후 설정 수정은 반드시:

```text
WorkSpace/config/monitor_arm_settings.json
```

에서 한다.

---

## 9.6 팀원 Standalone Test 파일

### 12) `test_pose_monitor_arm_controller.py`

```text
WorkSpace/hardware/BH_CODE/test_pose_monitor_arm_controller.py
```

Standalone Controller 기준 Test.

현재 통합 경로는:

```text
WorkSpace/pyQt/hardware_logic_selftest.py
```

에서 Motor1~4 / ToF-Vision / Rest-Recovery 핵심 경로를 검증한다.

**판정: Runtime 관점 삭제 가능**

---

### 13) `test_tof_service_and_fusion.py`

```text
WorkSpace/hardware/BH_CODE/test_tof_service_and_fusion.py
```

Standalone ToF/Fusion Test.

현재 통합 Self-test에서 Fixed ToF, Eye Gap, Vision Distance, Fusion을 함께 검증한다.

**판정: Runtime 관점 삭제 가능**

---

# 10. BH_CODE 삭제 전 최종 권장 절차

아래 순서로 하면 된다.

## 10.1 먼저 보존

```text
BH_CODE/requirements-monitor-arm.txt
→ WorkSpace/hardware/requirements-monitor-arm.txt
```

```text
BH_CODE/TOF_HW843_SETUP.md
→ WorkSpace/hardware/TOF_HW843_SETUP.md
```

`TOF_HW843_SETUP.md`의 다음 내용은 현재 구조에 맞게 수정한다.

```text
BH_CODE로 cd 하는 명령
pose_monitor_arm_controller.py 직접 실행 명령
Standalone Motor 실행 명령
```

---

## 10.2 선택적으로 Archive

필요하면:

```text
MONITOR_ARM_CODE_STUDY_GUIDE.md
MONITOR_ARM_CONTROL.md
POSE_MONITOR_ARM_CONTROL.md
```

을 별도 문서 보관 폴더로 이동한다.

필요 없으면 삭제해도 Runtime에는 영향이 없다.

---

## 10.3 BH_CODE 폴더 삭제

위 2개의 필수 보존 파일을 이동한 후 나머지가 불필요하다면:

```text
WorkSpace/hardware/BH_CODE/
```

전체를 삭제해도 현재 통합 Runtime 구조상 직접 실행 의존성이 없어야 한다.

---

## 10.4 삭제 후 Self-test

Repository Root에서:

```bash
python3 WorkSpace/pyQt/hardware_logic_selftest.py
```

를 실행한다.

이 Test는 실제 Servo/I2C 없이 Fake/Fixed Service를 사용해 핵심 통합 경로를 확인한다.

---

## 10.5 Raspberry Pi 실물에서는 추가 확인

Self-test 성공과 실제 Hardware 성공은 같은 의미가 아니다.

실물에서는 최소한 다음을 별도로 확인한다.

```text
1. /dev/ttyACM0 존재
2. Servo ID 1~4 Ping
3. 12V Servo 전원
4. /dev/i2c-3 존재
5. i2cdetect -y 3 에서 0x29 확인
6. VL53L0X Python package 설치
7. Pose landmark 정상 수신
8. HARDWARE_STATE["tof"] valid
9. monitor_arm_input fusion_mode 확인
10. Motor12 ready 확인
11. Motor Enable
12. 팔/모니터 주변 안전 확보 후 자동추종
13. Rest는 confirmed=True로만 시험
14. Resume/Recovery 확인
```

---

# 11. 현재 설정값의 책임 위치

설정 변경 시 아래 표를 먼저 확인한다.

| 바꾸려는 것 | 위치 |
|---|---|
| 사용자-모니터 목표 거리 | `config/monitor_arm_settings.json` → `distance` |
| ToF I2C / 범위 / EMA | `config/monitor_arm_settings.json` → `tof` |
| ToF/Vision 0.7/0.3 | `config/monitor_arm_settings.json` → `fusion` |
| Motor12 command_hz / Speed / Acc | `config/monitor_arm_settings.json` → `control` |
| Rest 각도 | `config/monitor_arm_settings.json` → `postures.rest` |
| Link 길이 / Zero Angle / Offset | `config/monitor_arm_settings.json` → `geometry` |
| Soft Joint Limit / Path Safety | `config/monitor_arm_settings.json` → `safety` |
| 실제 Servo Zero/Direction/Safe Range/max_speed | `hardware/servo_calibration_result.json` |
| IK 수학 | `pyQt/services/monitor_arm_kinematics.py` |
| Planner 정책 | `pyQt/services/monitor_arm_planner.py` |
| Eye Gap/Vision/Fusion 계산 | `pyQt/services/monitor_arm_user_x.py` |
| 실제 ToF Driver | `pyQt/services/tof_service.py` |
| Motor1/2 State/Timing/Rest/Recovery | `pyQt/services/motor12_controller.py` |
| Hardware 통합/Process 간 연결 | `pyQt/processes/hardware_process.py` |
| Motor Hardware 전달 | `pyQt/services/motor_service.py` |
| 실제 STS SyncWrite | `hardware/motor_control/controller.py` |

---

# 12. 수정하면 안 되는 책임 경계

추후 유지보수할 때 아래 경계를 지키는 것이 중요하다.

## `MonitorArmPlanner`에 넣지 말아야 할 것

- I2C 읽기
- Camera 읽기
- PyQt Event 처리
- Servo Serial
- GRU Workflow
- Motor Enable/Disable UI 조건

Planner는:

```text
현재 자세 + user_x + 안전범위
→ 목표 JointCommand
```

만 담당한다.

---

## `MotorService`에 넣지 말아야 할 것

- IK 계산
- ToF/Vision Fusion
- Adaptive Target 결정
- Rest인지 아닌지 판단
- Recovery가 안전한지 판단

MotorService는 **Controller가 결정한 명령을 Hardware Layer로 전달**한다.

---

## `tof_service.py`에 넣지 말아야 할 것

- Vision Fusion
- Monitor 목표거리
- IK
- Motor 제어

ToF Service는 센서 거리 측정과 상태만 책임진다.

---

## `HardwareProcess`가 소유해야 하는 것

실제 Hardware 자원:

```text
IMU I2C
ToF I2C
Motor Serial Bus
```

을 한 Process에서 관리한다.

별도 Standalone Motor/ToF Process가 같은 장치를 다시 열지 않도록 한다.

---

# 13. 장애 상황별 빠른 확인 위치

## Motor1/2가 안 움직임

순서:

```text
HARDWARE_STATE["motor"]["motor12"]["enabled"]
HARDWARE_STATE["motor"]["motor12"]["ready"]
HARDWARE_STATE["monitor_arm_input"]["valid"]
HARDWARE_STATE["monitor_arm_input"]["fusion_mode"]
HARDWARE_STATE["motor"]["motor12"]["hold_reason"]
HARDWARE_STATE["motor"]["motor12"]["last_error"]
```

---

## ToF가 안 잡힘

확인:

```text
/dev/i2c-3
i2cdetect -y 3
0x29
adafruit_extended_bus
adafruit_vl53l0x
```

코드:

```text
WorkSpace/pyQt/services/tof_service.py
Ctrl+F: def open
```

---

## Vision 값만 안 나옴

ToF가 정상이면 시스템은 `TOF_ONLY`로 계속 동작할 수 있다.

확인:

```text
pose_landmark_valid
pose_frame_id
eye_gap_px
vision_user_x_m
```

코드:

```text
monitor_arm_user_x.py
Ctrl+F: def measure_pose_eye_gap
```

---

## SAFE_HOLD가 걸림

가장 먼저:

```text
monitor_arm_input["last_error"]
tof["last_error"]
```

를 확인한다.

Vision만 정상이라고 SAFE_HOLD를 풀면 안 된다.

현재 안전정책은 의도적으로 **ToF 유효성을 Motor1/2 제어의 필수 기준**으로 둔다.

---

## Rest 후 자동추종이 재개되지 않음

단순 Motor Enable만 하지 말고:

```python
{"type": "MOTOR12_RESUME"}
```

Event가 들어갔는지 확인한다.

그리고:

```text
motor12_controller.py
Ctrl+F: def resume_from_rest
```

및 `hold_reason`을 확인한다.

---

# 14. 최종 테스트 기준

현재 최종 비실물 통합 Test:

```text
WorkSpace/pyQt/hardware_logic_selftest.py
```

문서 작성 기준 최신 커밋:

```text
49f1069fa2806bcf76ce8827a7c4f1fa3bab7c37
```

커밋 메시지:

```text
Test : Motor1~4 ToF-Vision 및 Rest Recovery 통합 self-test 보강
```

이 테스트는 병합 코드의 **계산과 Service 경계**를 확인하는 용도다.

실제 다음 항목은 Raspberry Pi 실물 테스트가 최종 기준이다.

```text
Servo 전원
Servo Ping
USB Serial
실제 Calibration
I2C3
HW-843
카메라 Landmark
실제 모니터 하중
Rest 물리 간섭
Recovery 물리 경로
```

---

# 15. 최종 정리

이번 병합에서 팀원 코드를 단순 Copy/Paste한 것이 아니라 다음처럼 POCO 구조로 재구성했다.

```text
BH_CODE Standalone
│
├─ Planner / IK 입력 정책
│    └─ monitor_arm_planner.py
│
├─ ToF user X / Vision / Fusion
│    └─ monitor_arm_user_x.py
│
├─ 실제 ToF Hardware
│    └─ tof_service.py
│
├─ Motor Pair / Adaptive Speed / State
│    └─ motor12_controller.py
│
├─ Rest / Recovery
│    ├─ motor12_controller.py
│    ├─ motor_service.py
│    └─ motor_control/controller.py
│
└─ Standalone main/process
     └─ 그대로 옮기지 않고 기존 HardwareProcess/PoseProcess 구조에 통합
```

최종 Runtime에서 기준으로 봐야 할 파일은 `BH_CODE`가 아니라:

```text
WorkSpace/config/monitor_arm_settings.json
WorkSpace/hardware/servo_calibration_result.json
WorkSpace/hardware/motor_control/
WorkSpace/pyQt/processes/hardware_process.py
WorkSpace/pyQt/services/motor12_controller.py
WorkSpace/pyQt/services/monitor_arm_planner.py
WorkSpace/pyQt/services/monitor_arm_kinematics.py
WorkSpace/pyQt/services/monitor_arm_speed.py
WorkSpace/pyQt/services/monitor_arm_user_x.py
WorkSpace/pyQt/services/tof_service.py
WorkSpace/pyQt/services/motor_service.py
WorkSpace/pyQt/hardware_logic_selftest.py
```

이다.

`BH_CODE`를 정리할 때는 다음 두 파일을 먼저 반드시 보존한다.

```text
requirements-monitor-arm.txt
TOF_HW843_SETUP.md
```

그 후 Standalone 실행 파일, 수동 Test UI, Visualizer, 중복 설정, Standalone Test는 현재 Runtime 기준으로 제거할 수 있다.

---

## 문서 유지 규칙

향후 구조가 변경되면 이 README도 함께 수정한다.

특히 다음이 바뀌면 반드시 갱신한다.

```text
1. Motor12 자동추종 활성 조건
2. ToF/Vision Fusion 정책
3. Rest/Recovery 정책
4. monitor_arm_settings.json 구조
5. servo_calibration_result.json 기준
6. MotorService / MotorController API
7. HardwareProcess의 State/Event 이름
8. ToF 설치 방법 또는 Raspberry Pi I2C 설정
```

**이 문서는 `Feature_Multiprocessing`의 HEAD `49f1069fa2806bcf76ce8827a7c4f1fa3bab7c37`을 기준으로 작성되었다.**

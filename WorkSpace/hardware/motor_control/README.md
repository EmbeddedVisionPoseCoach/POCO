# Motor Control Package 사용 가이드

`motor_control`은 STS3215 기반 4축 로봇팔을 **Joint 이름 + 각도 + Speed** 중심으로 제어하기 위한 Python 패키지입니다.

팀원은 Servo ID, raw Position, Zero Position, URDF 방향, STServo SDK 레지스터를 직접 다룰 필요가 없습니다.

이번 버전은 기존 팀원 API를 유지하면서 다음 안전/동시성 기능을 추가했습니다.

- 기존 `emergency_stop()` 이름과 Torque OFF 동작 유지
- 새 사용자용 `user_stop()` 추가
- `user_stop()` 중 모든 이동 API 차단
- `resume_user_stop()`은 latch만 해제하고 자체 이동하지 않음
- 4축 현재 Position을 읽어 **한 번의 SyncWrite**로 Hold
- `move_joint_relative()`의 `read → calculate → write` 전체를 command lock으로 보호
- Servo별 command generation으로 `wait=True` 중 새 목표 재호출 감지
- wait polling을 `Position + Moving` 경량 read로 변경
- 일반 상태 읽기 API는 기존 형태 유지

---

# 1. 실제 프로젝트에서의 위치

패키지 위치:

```text
POCO/
└─ WorkSpace/
   └─ hardware/
      ├─ servo_calibration.py
      ├─ servo_calibration_result.json
      ├─ servo_manual_control.py
      └─ motor_control/
         ├─ __init__.py
         ├─ config.py
         ├─ calibration.py
         ├─ servo_driver.py
         ├─ controller.py
         ├─ README.md
         ├─ README_ESTOP_ADDITION.md
         └─ VERIFICATION_REPORT.md
```

> 위 구조도는 이번 배포본에 포함한 **사용자 작성 모터 파일만** 표시합니다. 기존 프로젝트의 `hardware/__init__.py`, STServo SDK, 팀원 서비스/프로세스 파일은 수정하거나 포함하지 않습니다.

공식 통합 코드의 Import 기준은 `POCO/WorkSpace`이므로 `from hardware.motor_control import MotorController`를 사용합니다.

실제 통합 앱은 `multiprocessing` 구조이며 현재 프로젝트에서는 실제 모터 하드웨어 접근이 다음 경로로 모입니다.

```text
Main / Pose / Face Process
        ↓ IPC
Hardware Process
        ↓
MonitorMotorService
        ↓
MotorController
        ↓
ServoDriver
        ↓
/dev/ttyACM0
```

**통합 실행 규칙:** 실제 `/dev/ttyACM0`과 `MotorController`의 직접 소유자는 Hardware Process로 유지합니다. 다른 Process가 모터 동작을 요청할 때는 기존 IPC 구조를 통해 Hardware Process에 전달하는 방식을 사용합니다.

독립 하드웨어 테스트 파일은 통합 앱을 종료한 상태에서만 별도로 `MotorController()`를 생성해 사용합니다.

---

# 2. 구현 위치 빠른 찾기

아래 라인 번호는 **이 최종 패키지의 파일 기준**입니다.

## 공개 `MotorController` API

| 함수 | 구현 파일 / 위치 | 역할 |
|---|---|---|
| `MotorController()` | `controller.py` → `MotorController.__init__()` **62행** | Calibration/Driver/Stop latch/Lock/generation 초기화 |
| `emergency_stop()` | `controller.py` → **194행** | 기존 개발·점검용 4축 Torque OFF |
| `is_emergency_stopped()` | `controller.py` → **148행** | 기존 Emergency latch 확인 |
| `user_stop()` | `controller.py` → **259행** | Torque ON 유지 + 4축 현재 위치 Hold + 이동 차단 |
| `resume_user_stop()` | `controller.py` → **356행** | User Stop latch만 해제, 자체 이동 없음 |
| `is_user_stopped()` | `controller.py` → **152행** | User Stop latch 확인 |
| `move_joint()` | `controller.py` → **389행** | Zero 기준 절대각도 이동 |
| `move_joint_relative()` | `controller.py` → **462행** | 현재 위치 기준 상대각도 이동 |
| `move_joints()` | `controller.py` → **553행** | 여러 Joint SyncWrite 이동 |
| `move_to_zero()` | `controller.py` → **642행** | 한 Joint Zero 이동 |
| `move_all_to_zero()` | `controller.py` → **659행** | 전체 Joint Zero SyncWrite 이동 |
| `get_joint_angle()` | `controller.py` → **688행** | 현재 TEAM 기준 각도 읽기 |
| `get_joint_state()` | `controller.py` → **707행** | 한 Joint 전체 상태 읽기 |
| `get_all_states()` | `controller.py` → **742행** | 4축 일반 모니터링 상태 읽기 |
| `is_moving()` | `controller.py` → **754행** | 이동 여부 확인 |
| `close()` | `controller.py` → **867행** | Serial Port 닫기 |

## 핵심 내부 안전 로직

| 내부 기능 | 구현 파일 / 위치 | 역할 |
|---|---|---|
| `_check_motion_allowed()` | `controller.py` → **134행** | Emergency/User Stop 모두 확인 |
| `_bump_command_generations()` | `controller.py` → **160행** | 새 Goal 성공 후 Servo별 generation 증가 |
| `_generations_are_current()` | `controller.py` → **177행** | wait 중 명령 대체 여부 확인 |
| `_wait_for_targets()` | `controller.py` → **764행** | 목표 도착/Stop/재호출 감지 |
| `read_positions()` | `servo_driver.py` → **260행** | `user_stop()`용 다축 Position-only read |
| `read_motion_state()` | `servo_driver.py` → **291행** | wait용 Position + Moving 경량 read |
| `sync_write_positions()` | `servo_driver.py` → **156행** | 여러 Servo Goal을 한 SyncWrite로 전송 |
| `disable_torque_all_sync()` | `servo_driver.py` → **201행** | 기존 `emergency_stop()` Torque OFF SyncWrite |
| `read_state()` | `servo_driver.py` → **321행** | 일반 모니터링용 전체 상태 읽기 |

## 패키지 외부 모터 관리 파일

| 파일 / 함수 | 구현 위치 | 역할 |
|---|---|---|
| `servo_calibration.py` | `WorkSpace/hardware/servo_calibration.py` | Zero/Direction/Safe MIN·MAX/최대 속도 등 Calibration 생성·갱신 |
| `angle_to_position()` | `servo_calibration.py` → **380행** | URDF 각도 → raw Position 변환 |
| `read_servo_state()` | `servo_calibration.py` → **512행** | Calibration 중 Servo 상태 읽기 |
| `move_servo()` | `servo_calibration.py` → **763행** | Calibration 중 실제 Servo 이동 |
| `set_zero_position()` | `servo_calibration.py` → **1362행** | 현재 위치를 Zero로 설정 |
| `save_safe_min()` | `servo_calibration.py` → **1701행** | Safe MIN 저장 |
| `save_safe_max()` | `servo_calibration.py` → **1805행** | Safe MAX 저장 |
| `save_calibration_file()` | `servo_calibration.py` → **1905행** | `servo_calibration_result.json` 저장 |
| `servo_calibration_result.json` | `WorkSpace/hardware/servo_calibration_result.json` | ID 1~4의 최종 Calibration/방향/Safe Range/Max Speed 데이터 |
| `servo_manual_control.py` | `WorkSpace/hardware/servo_manual_control.py` | Calibration JSON을 읽어 안전 범위 내에서 터미널 수동제어 |
| `load_calibration_file()` | `servo_manual_control.py` → **310행** | Calibration JSON 읽기 |
| `read_servo_state()` | `servo_manual_control.py` → **580행** | 수동제어 중 Servo 상태 읽기 |
| `move_servo()` | `servo_manual_control.py` → **975행** | 수동제어 실제 Servo 이동 |
| `manual_joint_move()` | `servo_manual_control.py` → **1098행** | 선택 Joint를 URDF +/- 방향으로 수동 이동 |
| `move_to_zero()` | `servo_manual_control.py` → **1255행** | 저장/임시 Zero로 이동 |
| `hold_current_position()` | `servo_manual_control.py` → **1324행** | 현재 위치 Hold |

> `servo_calibration.py`와 `servo_manual_control.py`는 `motor_control` 패키지의 팀원용 API와 역할이 다릅니다. Calibration/점검용 도구이며, 통합 앱의 실제 모터 제어는 `MotorController`를 통해 수행합니다.
> 두 standalone 파일은 현재 SDK/JSON 경로를 실행 위치 기준 상대경로로 사용하므로, 기존 방식대로 **`POCO/WorkSpace/hardware`에서 실행**해야 합니다. 프로젝트 루트 이름이 `POCO`로 바뀐 것 자체는 이 상대경로 계산에 영향을 주지 않습니다.

## Calibration / 방향 / 안전범위

| 기능 | 구현 파일 / 위치 |
|---|---|
| 팀원 방향 설정 | `config.py` → `COMMAND_TO_URDF_DIRECTION` **79행** |
| 기본 `acc` | `config.py` → `DEFAULT_ACC` **117행** |
| wait Position tolerance | `config.py` → **125행** |
| wait polling interval | `config.py` → **126행** |
| User Stop Hold speed/acc | `config.py` → **149~150행** |
| Speed 검증 | `calibration.py` → `validate_speed()` **184행** |
| TEAM 각도 → raw Position | `calibration.py` → `command_angle_to_position()` **228행** |
| raw Position → TEAM 각도 | `calibration.py` → `position_to_command_angle()` **260행** |
| Safe Range 검증 | `calibration.py` → `validate_target_position()` **306행** |

---

# 3. 기본 Import

```python
from hardware.motor_control import MotorController

arm = MotorController()
```

사용이 끝나면:

```python
arm.close()
```

또는:

```python
from hardware.motor_control import MotorController

with MotorController() as arm:
    arm.move_joint(
        "shoulder_lift",
        angle=30,
        speed=100,
    )
```

---

# 4. Joint 이름과 TEAM 방향

| Joint | TEAM + | TEAM - |
|---|---|---|
| `shoulder_lift` | 위 | 아래 |
| `elbow_flex` | 위 | 아래 |
| `wrist_flex` | 위 | 아래 |
| `wrist_roll` | **CW** | **CCW** |

`wrist_roll` 관찰 기준은 **모니터가 위치한 정면에서 로봇팔을 바라보는 기준**입니다.

최종 실측 관계:

```text
RAW +  = CW
RAW -  = CCW
URDF + = CCW
TEAM + = CW
```

관련 구현:

```text
config.py → COMMAND_TO_URDF_DIRECTION (79행)
calibration.py → command_angle_to_position() (228행)
calibration.py → position_to_command_angle() (260행)
```

---

# 5. `move_joint()` - Zero 기준 절대각도

구현 위치:

```text
controller.py → MotorController.move_joint() → 389행
```

```python
arm.move_joint(
    "shoulder_lift",
    angle=30,
    speed=100,
)
```

현재 위치와 관계없이 Calibration Zero를 0°로 보고 TEAM +30°로 이동합니다.

`wrist_roll` 예:

```python
arm.move_joint("wrist_roll", 30, 100)
```

→ Zero 기준 **CW 30°**

```python
arm.move_joint("wrist_roll", -30, 100)
```

→ Zero 기준 **CCW 30°**

내부 순서:

```text
Stop 상태 1차 확인
→ Speed/Acc/Safe Range 검증
→ _command_lock 획득
→ Stop 상태 2차 확인
→ write_position()
→ 성공 시 해당 Servo command generation 증가
→ wait=True이면 목표 도착 대기
```

---

# 6. `move_joint_relative()` - 현재 위치 기준 상대각도

구현 위치:

```text
controller.py → MotorController.move_joint_relative() → 462행
```

```python
arm.move_joint_relative(
    "shoulder_lift",
    delta_angle=10,
    speed=100,
)
```

예를 들어 현재 TEAM 각도가 +20°이면 최종 목표는 약 +30°입니다.

이번 버전에서 중요한 변경점은 다음 전체 구간이 **하나의 `_command_lock` 안에서 처리**된다는 점입니다.

```text
_command_lock
   ↓
Stop 상태 확인
   ↓
현재 raw Position 읽기
   ↓
현재 TEAM 각도 계산
   ↓
delta_angle 적용
   ↓
Safe Range 검사
   ↓
Stop 상태 재확인
   ↓
실제 Goal write
   ↓
generation 갱신
```

따라서 다른 Thread가 `Position 읽기`와 `Goal write` 사이에 별도의 이동 명령을 끼워 넣어 오래된 Position 기준으로 상대각도를 계산하는 문제를 막습니다.

---

# 7. `move_joints()` - 여러 Joint 동시 이동

구현 위치:

```text
controller.py → MotorController.move_joints() → 553행
servo_driver.py → ServoDriver.sync_write_positions() → 156행
```

```python
arm.move_joints(
    {
        "shoulder_lift": 30,
        "elbow_flex": 20,
        "wrist_flex": -10,
        "wrist_roll": 15,
    },
    speed=100,
)
```

모든 Joint를 먼저 검증하고 전부 정상일 때만 SyncWrite합니다.

```text
모든 목표 계산/검증
→ 하나라도 실패: 아무 Servo에도 전송하지 않음
→ 모두 성공: 한 SyncWrite로 Goal 전송
```

SyncWrite가 성공한 Servo들의 command generation도 함께 갱신됩니다.

---

# 8. Zero 이동

구현 위치:

```text
controller.py → move_to_zero() → 642행
controller.py → move_all_to_zero() → 659행
```

단일 Joint:

```python
arm.move_to_zero(
    "shoulder_lift",
    speed=100,
)
```

전체 Joint:

```python
arm.move_all_to_zero(
    speed=100,
)
```

두 함수 모두 기존 이동 API를 재사용하므로 `user_stop()` 또는 `emergency_stop()` 상태에서는 자동으로 차단됩니다.

---

# 9. Speed / Acc

Speed 검증:

```text
calibration.py → CalibrationManager.validate_speed() → 184행
```

각 Servo의 `servo_calibration_result.json`에 저장된 `max_speed`를 초과하면 명령을 보내지 않습니다.

`acc` 기본값:

```text
config.py → DEFAULT_ACC = 10 → 117행
```

```python
arm.move_joint(
    "shoulder_lift",
    30,
    100,
    acc=20,
)
```

---

# 10. `wait=True / False`와 재호출

기본값은 `wait=True`입니다.

## `wait=True`

```python
result = arm.move_joint(
    "wrist_flex",
    50,
    80,
    wait=True,
)
```

목표 도착까지 기다립니다.

## `wait=False`

```python
arm.move_joint(
    "wrist_flex",
    50,
    80,
    wait=False,
)
```

Goal을 전송하고 즉시 반환합니다.

같은 Servo가 이동 중이어도 새 Goal을 다시 보낼 수 있습니다.

```python
arm.move_joint("wrist_flex", 50, 80, wait=False)
arm.move_joint("wrist_flex", 30, 80, wait=False)
arm.move_joint("wrist_flex", 25, 80, wait=False)
```

실제 하드웨어 테스트에서 `+50° → +30° → +25°` 두 번 재호출 후 마지막 목표로 이동하는 것을 확인했습니다.

## `wait=True` 중 다른 Thread가 새 명령을 보낸 경우

구현 위치:

```text
controller.py → _bump_command_generations() → 160행
controller.py → _generations_are_current() → 177행
controller.py → _wait_for_targets() → 764행
```

각 Servo는 command generation 번호를 가집니다.

```text
Thread A: ID3 +50° 전송 → generation 11 → wait
Thread B: ID3 +30° 전송 → generation 12
Thread A: 11 != 12 감지
→ 기존 +50° wait 즉시 종료
→ False 반환
```

따라서 새 목표로 이미 대체된 과거 목표를 Timeout까지 계속 기다리지 않습니다.

기존 API 호환성을 위해 반환형은 그대로 `bool`입니다.

```text
정상 도착         → True
새 명령으로 대체  → False
User Stop          → False
Emergency Stop     → False
Timeout            → False
```

---

# 11. 기존 `emergency_stop()` - 개발/점검용 Torque OFF

구현 위치:

```text
controller.py → MotorController.emergency_stop() → 194행
servo_driver.py → disable_torque_all_sync() → 201행
```

기존 팀원들이 이미 사용 중이므로 **함수 이름과 동작을 변경하지 않았습니다.**

```python
arm.emergency_stop()
```

동작:

```text
_emergency_event ON
→ 새로운 이동 명령 차단
→ Servo 1~4 Torque Enable = 0을 SyncWrite
→ Torque OFF 상태 유지
```

주의:

- Servo가 자세를 유지하지 않습니다.
- 로봇팔이 중력으로 떨어질 수 있습니다.
- 현재 버전에는 의도적으로 `reset_emergency_stop()`을 제공하지 않습니다.

상태 확인:

```python
arm.is_emergency_stopped()
```

---

# 12. 새 `user_stop()` - 최종 사용자용 Torque 유지 정지

구현 위치:

```text
controller.py → MotorController.user_stop() → 259행
servo_driver.py → ServoDriver.read_positions() → 260행
servo_driver.py → ServoDriver.sync_write_positions() → 156행
config.py → USER_STOP_HOLD_SPEED / ACC → 149~150행
```

호출:

```python
arm.user_stop()
```

정확한 내부 순서:

```text
1. _user_stop_event ON
   → command lock 대기 전부터 새 이동 요청을 software block

2. _command_lock 획득
   → lock 대기 중 resume가 먼저 실행된 경합까지 막기 위해 `_user_stop_event`를 다시 ON으로 확정

3. ID1~ID4 Present Position만 읽기
   예)
   ID1 = 1701
   ID2 = 2055
   ID3 = 2416
   ID4 = 2899

4. 한 번의 SyncWrite
   ID1 Goal = 1701
   ID2 Goal = 2055
   ID3 Goal = 2416
   ID4 Goal = 2899

5. Torque Enable은 변경하지 않음

6. 4축 command generation 갱신
   → 기존 wait 중인 과거 목표 무효화

7. _command_lock 해제
```

즉 **4축에 같은 Position을 보내는 것이 아니라, 각 축의 현재 Position을 각 축 Goal로 넣어 하나의 SyncWrite 패킷으로 전송**합니다.

### User Stop 중 차단되는 함수

```text
move_joint()
move_joint_relative()
move_joints()
move_to_zero()
move_all_to_zero()
```

Stop latch 확인 위치:

```text
controller.py → _check_motion_allowed() → 134행
```

모든 실제 이동 경로는 이 검사를 통과해야 Goal write까지 갈 수 있습니다.

### Position 읽기 실패 시

`user_stop()`은 Position 읽기를 실패했을 때만 한 번 재시도합니다.

두 번 모두 실패하면:

```text
user_stop() → False
_user_stop_event → 계속 ON
새 이동 명령 → 계속 BLOCK
부분 Hold Goal 전송 → 하지 않음
```

### SyncWrite 실패 시

```text
user_stop() → False
_user_stop_event → 계속 ON
새 이동 명령 → 계속 BLOCK
```

실패했다고 latch를 자동 해제하지 않습니다.

---

# 13. `resume_user_stop()` - 정지 해제

구현 위치:

```text
controller.py → MotorController.resume_user_stop() → 356행
```

```python
arm.resume_user_stop()
```

이 함수가 하는 일은 **오직 `_user_stop_event` latch 해제**입니다.

하지 않는 것:

```text
Torque ON/OFF 변경 ❌
Zero 이동 ❌
정지 전 목표 복원 ❌
새 Goal Position 전송 ❌
현재 Position 재전송 ❌
```

예:

```text
+50°로 이동 중 +13°에서 user_stop()
→ +13° Hold

resume_user_stop()
→ 여전히 +13° Hold

이후 move_joint(+30°)
→ 그때 새 이동 시작
```

현재 User Stop 상태:

```python
arm.is_user_stopped()
```

---

# 14. User Stop과 기존 Emergency Stop 우선순위

두 상태는 별도의 latch입니다.

```text
_emergency_event = 기존 Torque OFF 상태
_user_stop_event = Torque 유지 Hold 상태
```

기존 `emergency_stop()`이 이미 활성화되어 Torque OFF 상태이면 `user_stop()`은 현재 위치 Hold를 수행하지 않습니다.

`resume_user_stop()`을 호출해도 `_emergency_event`는 절대 변경하지 않습니다.

따라서 기존 Torque OFF 상태를 사용자 Stop 해제로 우회할 수 없습니다.

---

# 15. `_command_lock`의 책임

초기화 위치:

```text
controller.py → MotorController.__init__() → 62행
```

`_command_lock`은 다음 실제 모터 제어 작업끼리의 경합을 막습니다.

```text
move_joint() Goal write
move_joint_relative() read → calculate → write 전체
move_joints() SyncWrite
user_stop() Position read → Hold SyncWrite
emergency_stop() Torque OFF SyncWrite
```

반대로 `wait=True`의 목표 도착 polling 동안에는 `_command_lock`을 잡지 않습니다.

따라서 한 Thread가 목표 도착을 기다리는 동안 다른 Thread가:

```text
새 목표 재호출
user_stop()
emergency_stop()
```

을 실행할 수 있습니다.

이미 `_command_lock` 안에서 한 개의 패킷 전송이 시작된 순간까지 소급 취소할 수는 없습니다. `user_stop()`은 latch를 먼저 ON한 뒤 현재 전송이 끝나자마자 lock을 확보하여 현재 위치 Hold Goal로 덮어쓰는 구조입니다.

---

# 16. 상태 읽기 함수

## `get_joint_angle()`

구현:

```text
controller.py → 688행
```

```python
angle = arm.get_joint_angle("wrist_flex")
```

TEAM 방향 기준 각도(deg)를 반환합니다.

## `get_joint_state()`

구현:

```text
controller.py → 707행
servo_driver.py → read_state() → 321행
```

```python
state = arm.get_joint_state("wrist_flex")
```

반환 필드:

```python
{
    "joint": "wrist_flex",
    "angle": 12.3,
    "speed": 0,
    "load": 10,
    "load_percent": 1.0,
    "voltage": 12.2,
    "temperature": 30,
    "current_raw": 15,
    "moving": False,
}
```

`current_raw`는 mA 변환 전 raw 값입니다.

## `get_all_states()`

구현:

```text
controller.py → 742행
```

4축을 순서대로 읽습니다.

```python
states = arm.get_all_states()
```

일반 UI/로그/모니터링용으로 적합합니다. 각 Servo를 순차적으로 읽으므로 **완전히 같은 시각의 4축 snapshot은 아닙니다.**

## Stop 상태에서도 상태 읽기 허용

`emergency_stop()` 또는 `user_stop()` 상태에서도 상태 읽기는 허용합니다.

정지 후 Position, Load, Voltage, Temperature 등을 점검할 수 있어야 하기 때문입니다.

---

# 17. `user_stop()`과 wait가 일반 `get_all_states()`를 사용하지 않는 이유

일반 상태 함수는 다음 값을 모두 읽습니다.

```text
Position
Speed
Load
Voltage
Temperature
Current
Moving
```

사용자 정지에는 이 정보가 필요하지 않습니다.

그래서:

```text
user_stop()
→ servo_driver.py read_positions() (260행)
→ Position만 읽음

_wait_for_targets()
→ servo_driver.py read_motion_state() (291행)
→ Position + Moving만 읽음
```

으로 별도 경량 경로를 사용합니다.

장점:

```text
Serial 통신량 감소
wait polling 부담 감소
user_stop이 I/O lock을 얻기까지의 불필요한 지연 감소
```

---

# 18. Safe Range

구현:

```text
calibration.py → validate_target_position() → 306행
```

일반 이동 명령은 Calibration Safe Range를 벗어나면 자동 Clamp하지 않고 차단합니다.

```text
Safe Range 초과
→ 끝값으로 강제 보정 ❌
→ 그대로 전송 ❌
→ 이동 명령 BLOCK ✅
```

`user_stop()`은 **현재 실제 raw Position을 그대로 Hold**해야 하므로 현재 위치를 별도로 Safe Range 끝값으로 clamp하지 않습니다.

---

# 19. 반환값

기존 이동 함수의 반환 타입은 그대로 `bool`을 유지합니다.

일반적으로:

```text
True  = 요청한 명령/대기 정상 완료
False = 검증 실패 / Stop 차단 / 통신 실패 / 새 명령 대체 / Timeout 등
```

`wait=True`에서 `False`가 반환될 수 있는 원인이 늘어났지만 기존 팀원 코드의 호출 방식과 반환 타입은 변경되지 않았습니다.

---

# 20. 기존 팀원 코드 호환성

기존 함수 이름/인자 구조를 변경하지 않았습니다.

```text
MotorController()
move_joint()
move_joint_relative()
move_joints()
move_to_zero()
move_all_to_zero()
get_joint_angle()
get_joint_state()
get_all_states()
is_moving()
emergency_stop()
is_emergency_stopped()
close()
with MotorController()
```

따라서 기존 팀원들은 기존 호출 코드를 변경할 필요가 없습니다.

새 사용자 정지 기능을 사용하는 코드에서만:

```python
arm.user_stop()
arm.is_user_stopped()
arm.resume_user_stop()
```

을 추가하면 됩니다.

---

# 21. 기본 사용 예제

```python
from hardware.motor_control import MotorController

arm = MotorController()

try:
    arm.move_joint(
        "shoulder_lift",
        30,
        100,
    )

    arm.move_joint_relative(
        "elbow_flex",
        10,
        100,
    )

    arm.move_joints(
        {
            "shoulder_lift": 20,
            "elbow_flex": 15,
            "wrist_flex": -10,
            "wrist_roll": 20,  # TEAM + = CW
        },
        speed=100,
    )

    print(arm.get_all_states())

except KeyboardInterrupt:
    # 기존 개발/점검용 Torque OFF 함수
    arm.emergency_stop()

finally:
    arm.close()
```

사용자용 정지 예:

```python
# 사용자 STOP 입력
arm.user_stop()

# Stop 중 이동 요청은 False이며 Servo Goal을 보내지 않음
arm.move_joint("wrist_flex", 30, 80, wait=False)

# 사용자 RESUME
arm.resume_user_stop()

# resume 자체로는 움직이지 않음
# 새 명령을 명시적으로 호출할 때 다시 이동
arm.move_joint("wrist_flex", 30, 80, wait=False)
```

---

# 22. 최종 사용자용 User Stop 실물 검증 상태

현재 위치 Goal 덮어쓰기 방식은 실제 `wrist_flex` 단축 테스트에서 다음을 확인했습니다.

```text
Torque Enable = 1 유지
현재 Position을 Goal로 다시 Write
Moving = 0 전환
정지 후 자세 유지
Stop latch 중 재호출 차단
Resume 자체로 이동 없음
Resume 후 새 이동 명령 정상 수행
관측 최대 추가 이동 약 0.44°
```

해당 단축 실물 테스트는 `speed=80`, `acc=10`에서 수행했습니다.

이번 패키지는 같은 원리를 4축에 적용해 **각 축 현재 Position을 읽고 한 SyncWrite로 Hold**하도록 구현했습니다.

다만 코드/Mock 검증과 별개로 최종 사용자 안전 기능 승인 전에는 다음 실물 검증을 권장합니다.

```text
ID1 shoulder_lift 개별 정지
ID2 elbow_flex 개별 정지
ID3 wrist_flex 개별 정지 (단축 검증 완료)
ID4 wrist_roll 개별 정지
4축 동시 이동 중 전체 user_stop()
실제 부하/중력 조건에서 정지 후 추가 이동량 측정
실제 운용 최대 Speed 조건 확인
```

---

# 23. 내부 책임 분리

```text
controller.py
= 팀원용 API / Stop latch / command lock / generation / wait

servo_driver.py
= STServo raw 통신 / SyncWrite / Position read / 상태 read

calibration.py
= Calibration JSON / 각도↔Position / Safe Range / Speed 검증

config.py
= 방향 / 기본값 / Stop Hold 설정 / 레지스터 상수
```

일반 팀원은 가능하면 `controller.py`의 `MotorController` 공개 함수만 사용합니다.

```python
from hardware.motor_control import MotorController
```

`driver`, raw Position, Servo ID를 직접 제어하는 코드는 Calibration/하드웨어 점검용 테스트에서만 사용합니다.

# POCO 모니터암 코드 상세 학습 가이드

이 문서는 현재 `POCO_MonitorArm` 폴더에 구현된 모니터암 관련 코드를 처음부터 끝까지
공부하기 위한 자료다. 단순 사용법보다 다음 질문에 답하는 것을 목표로 한다.

- ToF 측정값이 어떻게 베이스 기준 사용자 X가 되는가?
- 사용자 X가 왜 모터 1·2의 각도로 바뀌는가?
- 정기구학과 역기구학은 코드에서 각각 어디에 쓰이는가?
- 영점과 모터 한계는 어느 계층에서 검사되는가?
- 자동 ToF 모의입력 제어와 수동 조그 UI는 무엇이 다른가?
- Tkinter 시각화는 OpenCV 카메라 루프와 어떻게 함께 실행되는가?
- 메인 프로세스와 모터 제어 프로세스는 어떤 메시지를 주고받는가?
- 나중에 VisionPoseCoach, 즉 포코의 메인/하드웨어 프로세스에 어떤 경계로 이식해야 하는가?

설명 수준은 파이썬 문법과 클래스 사용에 익숙한 중급자를 기준으로 한다. 다만
`dataclass`, Tkinter 이벤트 루프, 프로세스 Queue, `RLock`, 기구학처럼 일반적인
파이썬 프로그램에서는 자주 만나지 않는 개념은 별도로 풀어서 설명한다.

---

## 1. 먼저 알아둘 현재 범위

현재 모니터의 앞뒤 이동 계산과 실제 명령에는 다음 두 모터만 사용한다.

| Servo ID | Joint 이름 | 이 프로젝트에서의 역할 |
|---:|---|---|
| 1 | `shoulder_lift` | 상부 링크를 움직여 모니터 X/Z를 변경 |
| 2 | `elbow_flex` | 하부 링크를 움직여 모니터 X/Z를 변경 |
| 3 | `wrist_flex` | 이 코드에서는 사용하지 않음 |
| 4 | `wrist_roll` | 이 코드에서는 사용하지 않음 |

중요한 점은 “3·4번 명령을 계산했다가 버리는 것”이 아니라, 2축 기구학 모델 자체에
3·4번 관절이 존재하지 않는다는 것이다. 따라서 `JointCommand`에도 두 각도만 있고,
자동 모터 메시지에도 두 각도만 들어간다.

### 좌표계

모든 현재 코드가 사용하는 X-Z 좌표계는 다음과 같다.

```text
                 +Z, 위
                  ↑
                  |
BASE (0, 0) ------+----------------→ +X, 사용자 방향
클램프와 로봇팔이 연결되는 뿌리
```

- 원점: 클램프와 로봇팔 베이스가 연결되는 곳
- `+X`: 베이스에서 사용자를 향하는 방향
- `-X`: 사용자에게서 멀어지고 베이스로 돌아오는 방향
- `+Z`: 지면에서 수직으로 위쪽
- 거리 단위: 내부 계산은 metre, UI 표시는 주로 centimetre
- 각도 단위: 팀원용 모터 명령은 degree, 삼각함수 내부는 radian

ToF가 베이스 원점에서 사용자의 명치 방향을 측정한다고 보면 다음 관계가 성립한다.

```text
사용자 X = ToF 센서 원점 X + ToF range
목표 모니터 X = 사용자 X - 유지거리 50cm
```

카메라 Pose 결과는 거리 계산에 사용하지 않는다. 추후 얼굴만 앞으로 나온 거북목 등
나쁜 자세를 검출했을 때 ToF 자동 이동을 차단하는 입력으로 사용할 예정이다.

---

## 2. 파일 지도

### 핵심 실행 및 계산 파일

| 파일 | 한 문장 역할 |
|---|---|
| `pose_monitor_arm_controller.py` | ToF 사용자 X, MediaPipe 자세 입력, IK 계획, 안전검사를 수행하는 자동 제어 메인 |
| `monitor_arm_kinematics.py` | 2축 팔의 데이터 구조, 정기구학, 역기구학, 이동경로 안전검사 |
| `monitor_arm_motor_process.py` | 자동 모드의 별도 프로세스에서 시리얼 포트와 Servo 1·2 제어 전담 |
| `monitor_arm_visualizer.py` | 자동 제어의 현재 자세와 IK 목표를 Tkinter X-Z 화면에 그림 |
| `manual_motor12_limit_ui.py` | Servo 1·2를 `+/-` 버튼으로 개별 조그하고 soft limit을 측정 |
| `manual_vertical_ik_ui.py` | 사용자 X 게이지를 2축 IK로 변환하여 실시간으로 모터를 추종 |

### 데이터와 검증 파일

| 파일 | 역할 |
|---|---|
| `monitor_arm_settings.json` | 링크 길이, 목표 거리, 제어 주기, soft limit, 수직 허용편차 등 |
| `servo_calibration_result.json` | 실제 Servo ID, 영점 raw position, 하드 한계, 방향, 최대 속도 |
| `tasks/pose_landmarker_heavy.task` | MediaPipe Pose Landmarker 모델 바이너리 |
| `requirements-monitor-arm.txt` | MediaPipe, OpenCV, pyserial 의존성 |
| `test_pose_monitor_arm_controller.py` | ToF 좌표, IK, 안전경로, ID 1·2 제한을 검증하는 단위 테스트 |
| `POSE_MONITOR_ARM_CONTROL.md` | 실행법과 안전 주의사항을 빠르게 보는 운영 안내서 |

### 실제 모터 패키지

| 파일 | 계층 |
|---|---|
| `motor_control/config.py` | 상수와 각도 방향 정의 |
| `motor_control/calibration.py` | 팀원용 각도와 raw position 사이 변환 및 하드 한계 검사 |
| `motor_control/controller.py` | joint 이름과 각도를 받는 상위 모터 API |
| `motor_control/servo_driver.py` | STServo SDK에 raw 패킷을 보내는 저수준 드라이버 |
| `motor_control/__init__.py` | `MotorController`를 패키지 외부에 공개 |

`MONITOR_ARM_CONTROL.md`는 과거 4축 `monitor_arm_ui.py` 구조를 설명하는 레거시 문서다.
현재 1·2번 전용 Pose/IK 구조를 공부할 때는 이 학습 가이드와
`POSE_MONITOR_ARM_CONTROL.md`를 기준으로 보는 편이 정확하다.

---

## 3. 전체 아키텍처를 한 번에 보기

### 3.1 자동 시뮬레이션/실제 모터 공통 계산 흐름

```text
ToF range (현재는 고정 모의값)
    ↓
베이스 기준 사용자 X
    ↓
목표 monitor X = user X - 50cm
    ↓
현재 monitor X와 비교하여 한 주기 최대 2cm 변경
    ↓
고정 기준 Z와 새 X로 2축 역기구학
    ↓
soft limit / calibration hard limit / 관절 스텝 / Z 경로 검사
    ↓
shoulder_lift_deg, elbow_flex_deg
    ├─ 시뮬레이션: 가상 현재각으로 저장
    ├─ 시각화: Tkinter 창에 현재/목표 팔 그림
    └─ 실제 모드: Queue를 통해 모터 프로세스로 전달

카메라 → MediaPipe Pose는 위 흐름과 병행하며, 현재는 화면 표시만 하고 향후 자세
허용/차단 gate가 될 예정이다.
```

### 3.2 실제 모터 모드의 프로세스 경계

```text
┌──────────────────── 메인 프로세스 ────────────────────┐
│ ToF → 사용자 X → monitor X / Pose gate → IK → 안전검사│
│                                                       │
│ 결과: shoulder_lift_deg, elbow_flex_deg               │
└─────────────────────────┬─────────────────────────────┘
                          │ multiprocessing.Queue
                          ▼
┌────────────────── 모터 제어 프로세스 ─────────────────┐
│ CalibrationManager: 각도 → 안전한 raw position        │
│ MotorController: Servo 1·2 SyncWrite                   │
│ ServoDriver: /dev/ttyACM0 통신                         │
│ 현재 Servo 1·2 각도 읽어서 메인으로 응답               │
└───────────────────────────────────────────────────────┘
```

이 분리는 “메인에서 IK를 계산한다”와 “하드웨어 프로세스에서 그 각도를 실행한다”를
코드 구조로 강제한다. 포코로 이식할 때는 Queue 구현이 바뀌어도 메시지 의미인 두
관절각은 유지할 수 있다.

### 3.3 수동 조그 흐름

```text
사용자가 + 또는 - 버튼을 누름
    ↓
현재 실제 각도를 읽음
    ↓
경과시간 dt × 조그 속도(°/s)로 다음 각도 계산
    ↓
calibration hard limit ∩ UI soft limit으로 clamp
    ↓
해당 Servo 하나에만 position 명령
    ↓
버튼을 놓으면 해당 Servo 현재 위치를 한 번 읽어 Hold
    ↓
그 뒤에는 새 위치 패킷을 보내지 않음
```

### 3.4 수동 Cartesian IK 흐름

```text
ToF 사용자 X 모의 게이지
    ↓
monitor_x = user_x - 적용 중인 고정거리
monitor_z = 적용 중인 고정높이
    ↓
2축 IK
    ↓
하드/소프트 관절 한계와 예상 Z 경로 검사
    ↓
한 주기 최대 관절 스텝만큼 Servo 1·2 SyncWrite
    ↓
실제 각도를 다시 읽고 목표에 도달할 때까지 반복
```

---

## 4. 먼저 익혀둘 파이썬 개념

### 4.1 `@dataclass(frozen=True)`

`JointCommand`, `MonitorPose`, `ArmGeometry`, `SafetyLimits`, `EyeMeasurement`는
데이터를 담는 목적이 강한 클래스다. `@dataclass`는 생성자와 비교 메서드 등을 자동으로
만들어 준다.

```python
@dataclass(frozen=True)
class JointCommand:
    shoulder_lift_deg: float
    elbow_flex_deg: float
```

그러면 다음처럼 쓸 수 있다.

```python
command = JointCommand(10.0, -5.0)
print(command.shoulder_lift_deg)
```

`frozen=True`는 생성 후 필드를 직접 바꾸지 못하게 한다. 관절 명령을 중간에 몰래
수정하는 대신 새로운 `JointCommand`를 만들어야 하므로 계산 흐름을 추적하기 쉽다.

### 4.2 `float | None`

파이썬 3.10 이상의 union type hint다.

```python
self.reference_z_m: float | None = None
```

이 값이 아직 없으면 `None`, 기준 높이를 정한 후에는 `float`라는 뜻이다. 타입 힌트는 실행을
자동으로 막지는 않지만 IDE와 독자가 상태를 이해하는 데 도움을 준다.

### 4.3 `@property`, `@classmethod`, `@staticmethod`

- `@property`: 함수지만 필드처럼 읽는다. `geometry.effective_lower_link_m`
- `@classmethod`: 클래스 자체를 첫 인자 `cls`로 받는다. JSON에서 객체를 만드는
  `ArmGeometry.from_settings()`에 적합하다.
- `@staticmethod`: 객체 상태를 쓰지 않는 보조 함수다. `_clamp()`가 대표적이다.

### 4.4 예외와 `try/except/finally`

- `KinematicsError`: 목표 X/Z에 IK 해가 없을 때
- `MotionSafetyError`: IK 해는 있지만 이동경로가 안전 규칙을 위반할 때
- `CalibrationError`: 영점, 방향, 하드 한계, speed 검증 실패
- `RuntimeError`: 포트, 통신, 프로세스 응답 같은 실행 환경 문제

`finally`는 성공하거나 예외가 발생해도 반드시 실행된다. 카메라 release, Tk 창 종료,
시리얼 close처럼 자원 정리에 사용된다.

### 4.5 Tkinter 이벤트 루프와 `after()`

Tkinter GUI는 일반적인 `while True`로 버튼을 직접 검사하지 않는다. `mainloop()`가
마우스, 키보드, 창 다시 그리기 이벤트를 처리한다.

```python
root.after(75, self.jog_tick)
```

이 코드는 75ms 뒤 `jog_tick`을 한 번 호출한다. `jog_tick` 끝에서 다시 `after()`를
등록하면 반복 타이머가 된다. `sleep()`과 달리 GUI 전체를 멈추지 않는다.

### 4.6 Tkinter `StringVar`, `DoubleVar`, `trace_add`

Tkinter 변수는 위젯과 파이썬 값을 연결한다.

```python
self.distance_cm_var = tk.DoubleVar(value=50.0)
```

Spinbox가 값을 바꾸면 `get()`으로 새 값을 읽을 수 있다. `trace_add("write", callback)`는
값이 바뀌는 순간 callback을 부른다. Cartesian UI는 이를 이용해 고정거리/높이가
“입력만 바뀌고 아직 적용되지 않았다”는 상태를 표시한다.

### 4.7 `multiprocessing`, `spawn`, Queue

프로세스는 스레드와 달리 메모리를 기본적으로 공유하지 않는다. 따라서 명령과 응답을
Queue에 넣어 전달한다.

```python
context = multiprocessing.get_context("spawn")
request_queue = context.Queue()
```

`spawn`은 새 파이썬 인터프리터를 시작하고 필요한 객체를 직렬화해서 넘긴다. 그래서
프로세스 함수는 모듈 최상위에 있어야 하며, 실행 진입점은 다음 guard 안에 있어야 한다.

```python
if __name__ == "__main__":
    run()
```

### 4.8 `threading.RLock`과 `threading.Event`

`ServoDriver`는 하나의 시리얼 포트에 여러 패킷이 섞이지 않도록 `RLock`을 사용한다.
`RLock`은 같은 스레드가 이미 잡은 lock을 다시 잡을 수 있는 재진입 lock이다.

`MotorController`의 Emergency Stop 상태는 `Event`로 보관한다. 한 스레드가
`event.set()`을 호출하면 다른 스레드의 이동 함수도 즉시 같은 비상 상태를 확인한다.

---

## 5. 공통 데이터 모델: `monitor_arm_kinematics.py`

이 파일은 하드웨어나 GUI를 import하지 않는 순수 계산 계층이다. 따라서 가장 먼저
공부하기 좋다.

### 5.1 `KinematicsError`

`ValueError`를 상속한 사용자 정의 예외다. 목표가 팔의 최대 도달거리 밖이거나 너무
안쪽이라 2링크로 만들 수 없을 때 발생한다.

`ValueError`의 하위 클래스이므로 넓게는 숫자 입력 문제로 처리할 수도 있고,
`KinematicsError`만 따로 잡아 사용자에게 “IK 불가” 메시지를 보여줄 수도 있다.

### 5.2 `MotionSafetyError`

목표점 자체의 IK 계산은 됐지만 다음과 같은 안전조건에 실패할 때 사용한다.

- soft/hard 관절 한계 밖
- 한 제어 주기의 관절 변화량이 너무 큼
- 현재각과 목표각 사이 예상 경로의 Z 편차가 너무 큼

“수학적으로 갈 수 있다”와 “안전하게 명령해도 된다”를 구분한 예외다.

### 5.3 `ArmGeometry`

2축 평면 로봇팔의 기하 정보를 담는다.

| 필드 | 의미 |
|---|---|
| `shoulder_x_m` | 베이스 원점에서 shoulder 회전축까지 X 오프셋 |
| `shoulder_z_m` | 베이스 원점에서 shoulder 회전축까지 Z 오프셋 |
| `upper_link_m` | shoulder에서 elbow까지 길이 |
| `lower_link_m` | elbow에서 하부 링크 끝 모터 중심까지 길이 |
| `upper_zero_angle_rad` | 명령각 0°에서 상부 링크가 향하는 월드 각도 |
| `lower_zero_angle_rad` | 명령각 0°에서 하부 링크가 향하는 월드 각도 |
| `monitor_offset_m` | 하부 링크 끝 모터 중심에서 팔 방향으로 추가된 모니터 중심 7cm |

고정 베이스 링크는 기존 길이 약 13.56cm를 보존한 채 수직으로 세웠다. 따라서 shoulder
축은 `(X=0, Z=0.135606m)`에 있다. 이 링크는 움직이는 관절이 아니며 시각화에서도
베이스 원점에서 +Z로 곧게 그려진다.

#### `effective_lower_link_m`

```text
effective lower link = lower_link_m + monitor_offset_m
                     = 약 13.5cm + 7cm
                     = 약 20.5cm
```

IK가 계산하는 끝점은 하부 링크 끝 모터가 아니라 모니터 중심이다. 그래서 두 번째
링크의 유효 길이에 7cm를 더한다.

#### `from_settings(settings)`

JSON의 `geometry` 값을 읽어 `ArmGeometry`를 만든다. 키가 없으면 dataclass 기본값을
사용한다. 설정 파일과 계산 코드를 분리하기 위한 factory method다.

### 5.4 `JointCommand`

팀원이 이해하는 방향 기준의 두 관절각을 degree로 보관한다.

```python
JointCommand(
    shoulder_lift_deg=10.0,
    elbow_flex_deg=-5.0,
)
```

#### `interpolate(other, ratio)`

현재 명령과 다른 명령 사이의 중간값을 선형보간한다.

```text
sample = current + (target - current) × ratio
```

- `ratio=0`: current
- `ratio=0.5`: 정확히 중간
- `ratio=1`: target

관절 경로 샘플링과 한 번에 5°만 움직이는 step target 생성에 사용한다.

### 5.5 `MonitorPose`

모니터 중심의 평면 위치만 담는다.

```python
MonitorPose(x_m=0.30, z_m=0.237)
```

자세라는 이름이지만 현재 2축 모델에서는 회전 orientation을 별도로 담지 않는다.
모니터 중심의 X/Z 위치를 의미한다고 이해하면 된다.

### 5.6 `monitor_target_from_user()`

수동 Cartesian UI에서 사용자 좌표를 모니터 목표 좌표로 바꾼다.

```text
monitor_x = user_x - user_monitor_distance
monitor_z = requested constant z
```

예를 들어 사용자가 베이스에서 80cm, 유지 거리가 50cm라면 모니터 목표 X는 30cm다.
유한한 숫자인지, 거리가 양수인지, 사용자 X가 거리보다 큰지 등을 먼저 검증한다.

### 5.7 `SafetyLimits`

설정 파일의 안전 관련 값을 한 객체에 모은다.

| 필드 | 역할 |
|---|---|
| `shoulder_min/max_deg` | Servo 1 software limit |
| `elbow_min/max_deg` | Servo 2 software limit |
| `vertical_tolerance_m` | 예상 이동 중 기준 Z에서 벗어나도 되는 최대값 |
| `max_joint_step_deg` | 한 자동 제어 주기 최대 관절 변화 |
| `path_samples` | 현재각→목표각 경로를 몇 구간으로 검사할지 |

`from_settings()`는 JSON을 읽고 `path_samples`가 최소 2 이상이 되게 한다.

### 5.8 `TwoJointMonitorArm`

2축 정기구학과 역기구학의 중심 클래스다.

#### 각도 방향 상수

```python
SHOULDER_COMMAND_TO_URDF = -1.0
ELBOW_COMMAND_TO_URDF = -1.0
```

팀원용 `+` 각도와 URDF `+` 각도의 방향이 반대라서 `-1`을 곱한다. 이 변환은
raw encoder 방향 변환과 다르다. 방향 계층은 뒤에서 다시 정리한다.

#### `command_to_urdf(command)`

팀원용 degree를 URDF radian으로 바꾼다.

```text
URDF rad = radians(team degree × -1)
```

#### `urdf_to_command(shoulder_rad, elbow_rad)`

위 변환의 역연산이다. IK가 내부적으로 계산한 URDF radian을 팀원용 degree로 돌린다.

#### `forward(command)` — 정기구학

관절각을 알 때 모니터 X/Z를 구한다.

```text
upper_world = upper_zero_angle - shoulder_urdf
lower_world = lower_zero_angle - shoulder_urdf - elbow_urdf

x = shoulder_x
  + upper_link × cos(upper_world)
  + effective_lower_link × cos(lower_world)

z = shoulder_z
  + upper_link × sin(upper_world)
  + effective_lower_link × sin(lower_world)
```

주요 사용처는 다음과 같다.

- 현재 모터각으로 실제 모니터 위치 표시
- IK 결과가 목표 X/Z와 일치하는지 확인
- 시각화에서 링크 끝점 계산
- 가상 모니터가 얼마나 X로 움직였는지 계산
- 이동 중간경로의 Z 편차 검사

#### `inverse(target_x_m, target_z_m)` — 역기구학

원하는 모니터 X/Z를 만들 Servo 1·2 각도를 구한다.

1. shoulder 축을 기준으로 목표의 상대좌표 `dx`, `dz`를 구한다.
2. shoulder에서 목표까지 반지름 `radius = hypot(dx, dz)`를 구한다.
3. 두 링크 길이로 닿을 수 있는 범위인지 검사한다.
4. 코사인 법칙으로 두 링크의 상대각을 구한다.
5. `atan2`로 상부 링크 월드 각도를 구한다.
6. zero-angle 오프셋을 반영해 URDF 관절각을 구한다.
7. 팀원용 command degree로 변환한다.

도달 가능 조건은 다음과 같다.

```text
abs(L1 - L2) ≤ shoulder에서 목표까지 거리 ≤ L1 + L2
```

코사인 역함수에는 접힌 방향과 펴진 방향의 두 해가 생길 수 있다. 현재 코드는
`-acos(...)`를 사용해 모니터암에 맞춘 한 가지 non-folded branch만 선택한다.

#### `validate_motion()`

자동 제어의 최종 방어 함수다.

1. 설정의 soft limit을 기본 관절 범위로 만든다.
2. calibration hard range가 오면 두 범위의 교집합을 구한다.
3. 목표 두 각도가 교집합 안인지 검사한다.
4. 현재각과 목표각의 최대 변화가 `max_joint_step_deg` 이하인지 검사한다.
5. 현재각부터 목표각까지 `path_samples + 1`개 자세를 만든다.
6. 각 샘플의 모니터 Z가 기준 Z에서 `vertical_tolerance_m`보다 많이 벗어나면 차단한다.

여기서 검사하는 것은 실제 토크가 아니라 “관절을 직선 보간해서 이동한다고 가정한
기구학적 예상 Z 경로”다.

### 5.9 설정 파일 함수

#### `load_settings(path)`

JSON을 dict로 읽는다. 계산 함수가 특정 전역 설정에 강하게 묶이지 않게 path 인자를
받는다.

#### `save_settings(data, path)`

임시 파일에 JSON을 쓴 뒤 `os.replace()`로 원본을 교체한다. 저장 도중 프로그램이
종료되어 원본 JSON이 반만 써지는 위험을 줄이는 atomic save 패턴이다.

#### `iter_joint_values(command)`

두 관절 이름과 값을 순서대로 yield하는 generator다.

```python
for name, angle in iter_joint_values(command):
    ...
```

현재 핵심 실행 흐름에서 비중은 작지만, 두 관절 반복 코드를 단순화할 수 있는 도우미다.

---

## 6. 설정 파일: `monitor_arm_settings.json`

### 6.1 `geometry`

```json
"geometry": {
  "shoulder_x_m": 0.0,
  "shoulder_z_m": 0.1356059585,
  "upper_link_m": 0.1160,
  "lower_link_m": 0.1350,
  "upper_zero_angle_rad": 1.3270,
  "lower_zero_angle_rad": 0.0385,
  "monitor_offset_m": 0.07
}
```

실제 축 사이 거리와 0° 자세의 링크 방향을 정의한다. 링크 외형 길이가 아니라 회전축
중심 사이의 기하를 써야 한다.

### 6.2 `distance`와 `tof`

| 키 | 현재값 | 의미 |
|---|---:|---|
| `desired_user_monitor_distance_m` | 0.50 | 자동 제어 목표 거리 |
| `deadband_m` | 0.015 | 목표에서 ±1.5cm면 움직이지 않음 |
| `max_monitor_x_step_m` | 0.02 | 한 자동 계획에서 X를 최대 2cm 변경 |

| ToF 키 | 현재값 | 의미 |
|---|---:|---|
| `mode` | `fixed_stub` | 실제 센서 연결 전 고정값 모드 |
| `sensor_origin_x_m` | 0.0 | 베이스 원점 기준 ToF 발광점 X |
| `fixed_range_m` | 약 0.73285 | 임시 ToF range |
| `minimum_user_x_m` | 약 0.60077 | 허용할 사용자 X 하한 |
| `maximum_user_x_m` | 약 0.83077 | 허용할 사용자 X 상한 |

센서 원점 X가 0이므로 현재는 `fixed_range_m` 자체가 사용자 X다. 실제 센서가 베이스
원점보다 앞에 장착되면 그 장착 오프셋을 `sensor_origin_x_m`에 기록한다.

### 6.3 `control`

| 키 | 의미 |
|---|---|
| `command_hz` | 자동/Cartesian 반복 명령 주기, 현재 5Hz |
| `speed` | STServo SDK에 보내는 speed 값 |
| `acc` | STServo SDK에 보내는 acceleration 값 |
| `manual_test_speed_cap` | 수동 시험 UI가 허용하는 임시 speed 상한 |

주의할 점은 `speed`가 반드시 `degree/second`를 뜻하지는 않는다는 것이다. STServo SDK의
속도 단위다. 반면 수동 조그 UI의 `jog_rate`는 코드가 직접 적분하는 `degree/second`다.

### 6.4 `manual_cartesian`

수동 Cartesian UI에서 게이지와 Z 입력이 벗어나지 못하도록 하는 사용자 좌표 범위다.
이 직사각형 범위 안이라고 반드시 2링크 IK 해가 존재하는 것은 아니다. 최종적으로는
IK 도달범위와 관절 한계도 통과해야 한다.

### 6.5 `safety`

- `soft_joint_limits_deg`: 수동 시험으로 좁혀 가는 소프트웨어 관절 한계
- `vertical_tolerance_m`: 예상 경로가 기준 높이에서 벗어날 수 있는 허용량, 현재 3cm
- `max_joint_step_deg`: `stepped` 모드와 휴식 복귀에서 한 명령의 최대 5°
- `path_samples`: 예상 경로를 30구간으로 검사

`vertical_tolerance_m=0.03`은 “모니터 높이를 언제나 수학적으로 정확히 유지한다”는
뜻이 아니다. 관절 보간 중 기준 Z에서 최대 3cm까지 차이를 허용하고 그보다 크면 명령을
막는다는 뜻이다.

### 6.6 모델과 Python 의존성

`tasks/pose_landmarker_heavy.task`는 Python 소스가 아니라 MediaPipe가 읽는 학습 모델
바이너리다. 코드를 열어 공부하는 파일이 아니라 `BaseOptions(model_asset_path=...)`에
경로를 넘기는 실행 자원이다.

`requirements-monitor-arm.txt`의 역할은 다음과 같다.

- `mediapipe>=0.10`: Pose Landmarker runtime
- `opencv-python>=4.8`: webcam frame, 색상 변환, 화면 표시
- `pyserial>=3.5`: STServo SDK가 사용하는 serial 통신

Tkinter는 보통 Python 표준 배포 또는 OS의 `python3-tk` 패키지로 제공되므로 이 pip
requirements에는 들어 있지 않다.

---

## 7. 자동 제어 메인: `pose_monitor_arm_controller.py`

이 파일은 시스템의 orchestration 계층이다. 영상, 거리, IK, 시각화, 모터 프로세스를
순서대로 연결하지만 raw servo position 변환이나 시리얼 패킷은 직접 처리하지 않는다.

### 7.1 전역 상수

```python
DEFAULT_MODEL_PATH = tasks/pose_landmarker_heavy.task
DEFAULT_CALIBRATION_PATH = servo_calibration_result.json
LEFT_EYE_INDEX = 2
RIGHT_EYE_INDEX = 5
```

Face Landmarker를 별도로 사용하지 않고 Pose Landmarker의 눈 landmark 2와 5를 쓴다.

### 7.2 `EyeMeasurement`

한 프레임에서 계산한 눈 정보를 담는 immutable dataclass다.

| 필드 | 의미 |
|---|---|
| `gap_px` | 두 눈 사이 픽셀 거리 |
| `left_xy` | OpenCV 화면에 그릴 왼쪽 눈 픽셀 좌표 |
| `right_xy` | 오른쪽 눈 픽셀 좌표 |

픽셀 거리 계산과 화면 표시가 같은 좌표를 공유하도록 하나의 객체로 묶었다.

### 7.3 `FixedToFUserXSource`

#### 생성자

```python
FixedToFUserXSource(
    sensor_origin_x_m=0.0,
    fixed_range_m=0.732848,
    minimum_user_x_m=0.6007655,
    maximum_user_x_m=0.8307655,
)
```

실제 ToF 드라이버가 준비되기 전 사용하는 입력 객체다. 센서가 반환하는 ray 방향 거리와
센서 장착 X를 더해 베이스 원점 기준 사용자 X로 바꾼다.

#### `read_range_m()`

현재는 설정의 `fixed_range_m`을 그대로 반환한다. 실제 ToF 연결 시 이 함수 또는 같은
인터페이스의 하드웨어 클래스를 센서 read로 교체한다.

#### `read_user_x_m()`

계산은 다음과 같다.

```text
user_x = sensor_origin_x + range
```

NaN/무한대이거나 설정한 사용자 X 범위 밖이면 `ValueError`를 발생시키고 자동 제어는
`SAFE HOLD`한다. 얼굴이 아니라 명치 부근 몸통을 측정하므로 얼굴만 앞뒤로 움직이는
상황은 사용자 X 변화로 취급하지 않는 것이 의도다.

### 7.4 `MonitorArmPlanner`

ToF 사용자 X와 현재 관절각을 받아 이번 주기에 보낼 안전한 다음 관절각을 계산한다.

#### 생성자

- `TwoJointMonitorArm` 생성
- `SafetyLimits` 생성
- 목표거리, deadband, 최대 X step 로드
- `reference_z_m`은 아직 모르는 상태로 `None`

#### `set_vertical_reference(current)`

현재 관절각을 정기구학으로 변환하고 그 모니터 Z를 기준 높이로 저장한다. 자동 제어가
시작할 때 한 번 호출된다.

이 기준은 `manual_vertical_ik_ui.py`의 사용자가 입력하는 고정 Z와 다르다. 자동
Pose 제어는 시작 당시 실제 자세의 Z를 유지하려는 방식이다.

#### `plan(current, user_x_m, calibration_ranges=None)`

가장 중요한 계획 함수다.

1. 기준 Z가 없으면 현재 자세로 설정한다.
2. `target_monitor_x = user_x - desired_distance`를 계산한다.
3. 목표 monitor X와 현재 monitor X의 오차를 계산한다.
4. 오차 절댓값이 deadband 이하이면 `None`을 반환한다.
5. X 오차를 ±`max_monitor_x_step_m`로 clamp하고 Z는 기준값으로 둔다.
6. 그 X/Z에 대한 전체 IK 목표를 구한다.
7. 기본 `direct`에서는 전체 IK 목표를 그대로 사용한다. `stepped` 설정에서만 5°로 줄인다.
8. `validate_motion()`으로 관절 한계와 전체 보간 Z 경로를 검사한다.
9. 안전한 `JointCommand`를 반환한다. 휴식 복귀는 별도 로직으로 항상 5°씩 진행한다.

`None`은 오류가 아니다. “목표거리 근처이므로 움직일 필요 없음”이라는 정상 결과다.

#### ToF 사용자 X 예시

```text
user_x=0.78m, desired_distance=0.50m
→ target_monitor_x=0.28m

현재 monitor_x=0.23m이면 +X 이동
현재 monitor_x=0.31m이면 -X 이동
```

### 7.5 `measure_pose_eye_gap()`

MediaPipe landmark 목록을 받아 `EyeMeasurement`를 만든다.

1. landmark가 없거나 인덱스 5까지 없으면 `None`
2. 양쪽 눈의 `visibility`, `presence`가 0.5 미만이면 `None`
3. 정규화 좌표 0~1을 영상 width/height 픽셀 좌표로 변환
4. `hypot(dx, dy)`로 기울어진 두 눈 사이 유클리드 거리 계산

`getattr(landmark, "visibility", 1.0)`처럼 기본값을 주는 이유는 사용하는 MediaPipe
객체에 해당 속성이 없더라도 코드를 동작시키기 위해서다.

눈 점과 선은 Pose가 정상 처리되는지 보여주기 위한 시각 표시다. `gap_px`는 현재 ToF
사용자 X나 IK 목표 계산에 들어가지 않는다.

### 7.6 `parse_args()`

지원 모드는 다음과 같다.

```bash
python pose_monitor_arm_controller.py
python pose_monitor_arm_controller.py --tof-user-x-m 0.78
python pose_monitor_arm_controller.py --enable-motor
python pose_monitor_arm_controller.py --no-ik-visualizer
```

`--tof-user-x-m`은 실제 센서 연결 전 고정 사용자 X를 임시로 덮어쓰는 시험 옵션이다.
설정의 ToF 최소/최대 범위를 벗어나면 제어를 시작하지 않는다.

### 7.7 `run()` 전체 흐름

#### 1단계: 설정과 라이브러리 로드

- 명령행 인자 읽기
- JSON 설정 로드
- Pose model 파일 존재 확인
- OpenCV, MediaPipe import

MediaPipe를 함수 안에서 import하므로 단위 테스트가 ToF/IK 클래스만 import할 때 무거운
비전 런타임을 꼭 시작하지 않아도 된다.

#### 2단계: 계산 객체 생성

- `FixedToFUserXSource`
- `MonitorArmPlanner`

#### 3단계: 모드별 현재각 결정

- 기본 고정 ToF 시뮬레이션: `JointCommand(0, 0)`
- 실제 모터: `MotorControlProcessClient`를 열고 모터 프로세스가 읽은 초기각 사용

실제 모드에서도 메인 프로세스는 시리얼 포트를 직접 열지 않는다.

#### 4단계: 기준 Z와 ToF 사용자 X 설정

현재각의 정기구학 결과에서 `reference_z`를 저장한다. ToF 설정의 센서 원점과 range를
읽어 베이스 기준 사용자 X를 만든다.

#### 5단계: Tk 시각화와 Pose Landmarker 옵션 생성

시각화 창 생성에 실패하면 터미널에 오류를 출력하고 카메라 계산 자체는 계속한다.

Pose Landmarker는 `RunningMode.VIDEO`를 쓴다. VIDEO 모드는 프레임 timestamp가 계속
증가해야 하므로 다음 코드가 있다.

```python
timestamp_ms = max(monotonic_ms, last_timestamp_ms + 1)
```

시스템 시계가 아니라 `time.monotonic()`을 쓰는 이유는 시스템 시간이 조정되어 뒤로
가더라도 경과시간은 계속 증가하게 하기 위해서다.

#### 6단계: 카메라 프레임 루프

한 프레임마다 다음 일을 한다.

1. 프레임 읽기
2. 좌우 반전
3. BGR → RGB 변환
4. MediaPipe Image 생성
5. Pose 추론
6. Pose landmark 처리와 화면 표시
7. ToF source에서 사용자 X 읽기
8. `user_x - 유지거리`로 monitor X 계산
9. 제어 주기가 됐으면 IK 계획
10. 실제 모드라면 모터 프로세스에서 현재각 읽기
11. 목표가 있으면 시뮬레이션 상태 갱신 또는 모터 프로세스로 전송
12. OpenCV 텍스트와 Tkinter 시각화 갱신
13. `q`, `h`, `a` 키 처리

#### 7단계: 명령 주기 제한

카메라가 30FPS여도 모터 계획을 매 프레임 보내지 않는다.

```python
command_interval = 1.0 / command_hz
if now - last_command_at >= command_interval:
    ...
```

현재 5Hz이므로 약 0.2초마다 한 번만 새 관절 목표를 계산/전송한다.

#### 8단계: `user_x_m`과 실제 사용자-모니터 거리

- `user_x_m`: ToF가 제공하는 베이스 기준 사용자 몸통 X
- `actual_distance_m`: `user_x_m - current_monitor_x`
- `desired_distance_m`: 유지하려는 0.50m

플래너는 `actual_distance_m`을 카메라로 추정하지 않고 사용자 X에서 목표 monitor X를
직접 만든다. 따라서 고정 webcam 때문에 별도 가상 카메라 보정을 할 필요가 없다.

#### 9단계: 시뮬레이션의 `current = target`

모터가 없는 모드에서는 IK 목표를 실제로 실행할 하드웨어가 없으므로, 명령이 성공했다고
가정하고 target을 다음 주기의 current로 저장한다. 이것이 Tkinter 팔이 가상으로
움직이는 원리다.

#### 10단계: 정리

`finally`에서 다음 자원을 닫는다.

- OpenCV capture
- OpenCV 창
- Tkinter 시각화 창
- 모터 제어 프로세스

카메라 루프에서 예외가 생겨도 이 정리가 실행된다.

---

## 8. 자동 모터 프로세스: `monitor_arm_motor_process.py`

이 파일은 계산과 실제 하드웨어 실행 사이의 경계다.

### 8.1 왜 별도 프로세스인가?

- 시리얼 포트를 한 프로세스만 소유하게 한다.
- 카메라/AI 코드가 raw position이나 STServo SDK를 몰라도 된다.
- 포코의 하드웨어 프로세스로 이식할 때 angle message 계약을 유지할 수 있다.
- 모터 통신 오류를 메인 계산 상태와 분리할 수 있다.

현재 client는 각 요청의 응답을 기다리는 동기 RPC 방식이다. 즉 모터 I/O는 자식
프로세스에서 실행되지만 메인은 응답이 올 때까지 잠시 기다린다. 포코 통합에서 카메라
루프를 절대 막지 않아야 한다면, 목표각 Queue는 비동기 put으로 보내고 하드웨어의 최신
각도 telemetry를 별도 Queue/shared state로 읽는 구조로 확장할 수 있다.

### 8.2 `TwoMotorHardware`

모터 프로세스 내부에서만 사용되는 하드웨어 adapter다.

#### 생성자

캘리브레이션 경로, speed, acc를 저장한다. 아직 포트를 열지는 않는다.

#### `open()`

1. calibration JSON을 읽는다.
2. ID 1이 `shoulder_lift`, ID 2가 `elbow_flex`인지 검사한다.
3. 두 모터의 `max_speed`가 설정됐는지 검사한다.
4. 요청 speed가 각 max speed 이하인지 검사한다.
5. `MotorController`를 생성해 포트를 연다.
6. calibration hard angle range를 읽어 둔다.

현재 JSON의 Servo 1·2 `max_speed`는 `null`이므로 실제 자동 모드는 의도적으로 시작이
차단된다. 수동 실측 후 안전 속도를 기록해야 한다.

#### `read_angles()`

`MotorController.get_joint_angle()`로 Servo 1·2의 실제 각도를 읽고 `JointCommand`로
묶는다.

#### `move(target)`

다음 dict만 `move_joints()`에 보낸다.

```python
{
    "shoulder_lift": target.shoulder_lift_deg,
    "elbow_flex": target.elbow_flex_deg,
}
```

ID 3·4의 key가 만들어지지 않는다. `wait=False`라서 목표 도착까지 자식 프로세스를
붙잡고 있지는 않고 패킷 수락 여부까지만 확인한다.

#### `close()`

`MotorController.close()`로 시리얼 포트를 닫는다.

### 8.3 `_serialize_angles()`와 `_deserialize_angles()`

`JointCommand`를 Queue에 넣기 쉬운 단순 dict로 변환하거나 다시 객체로 만든다.

```json
{
  "shoulder_lift_deg": 12.3,
  "elbow_flex_deg": -4.5
}
```

이 dict가 포코로 이식할 때 유지해야 할 핵심 명령 계약이다.

### 8.4 `_respond()`

모든 응답 형식을 통일한다.

```python
{
    "request_id": 3,
    "ok": True,
    "result": {...},
    "error": None,
}
```

요청 ID가 있어 어떤 요청에 대한 응답인지 확인할 수 있다.

### 8.5 `motor_process_worker()`

자식 프로세스의 진입 함수다. 모듈 최상위 함수여야 `spawn`이 pickle/import할 수 있다.

초기화 성공 시 request ID 0으로 다음을 응답한다.

- 초기 Servo 1·2 각도
- calibration hard angle range

이후 무한 루프에서 Queue message를 기다린다.

| `type` | 동작 |
|---|---|
| `read_angles` | 현재 Servo 1·2 각도를 읽어 응답 |
| `move` | 두 목표각을 역직렬화하여 동기 이동 명령 |
| `shutdown` | 성공 응답 후 루프 종료 |

어떤 경로로 루프가 끝나도 `finally`에서 하드웨어를 닫는다.

### 8.6 `MotorControlProcessClient`

메인 프로세스가 자식 프로세스를 일반 객체처럼 사용하게 해 주는 proxy다.

#### 생성자

- `spawn` multiprocessing context 준비
- Queue와 Process는 아직 `None`
- 다음 request ID는 1
- 응답 timeout 기본 5초

#### `open()`

1. request/response Queue 생성
2. `motor_process_worker` Process 생성
3. 프로세스 시작
4. 초기화 응답 ID 0 대기
5. 초기 각도와 calibration range 저장

#### `_request()`

새 request ID를 붙이고 Queue에 message를 넣은 뒤 `_receive()`로 같은 ID의 응답을
기다린다.

#### `_receive()`

- timeout이면 `RuntimeError`
- response request ID가 다르면 오류
- `ok=False`면 자식의 error 문장을 `RuntimeError`로 변환
- 성공하면 result dict 반환

#### `read_angles()`, `move()`

메인 코드가 Queue 구조를 직접 알지 않게 하는 편의 API다. 이름은 하드웨어 함수와
비슷하지만 내부에서는 프로세스 메시지를 보낸다.

#### `close()`

1. 자식이 살아 있으면 shutdown 요청
2. 최대 2초 join
3. 계속 살아 있으면 terminate
4. Queue close 및 feeder thread join
5. 참조를 `None`으로 초기화

---

## 9. 자동 IK 시각화: `monitor_arm_visualizer.py`

### 9.1 `calculate_arm_points()`

관절 명령 하나를 받아 다음 네 월드 좌표를 반환한다.

```text
[base, shoulder, elbow, monitor_center]
```

마지막 점은 `effective_lower_link_m`, 즉 7cm 모니터 offset을 포함한다. 테스트에서는
이 마지막 좌표가 `TwoJointMonitorArm.forward()` 결과와 같은지 확인한다.

### 9.2 `PoseIKVisualizer`

자동 제어용 Tkinter 창 전체를 관리한다.

#### 생성자

- 현재/목표 관절 상태 초기화
- Tk root와 Label/Canvas 생성
- 창 닫기 callback 연결
- 최초 장면 그림
- `pump_events()` 한 번 실행

색상 의미는 다음과 같다.

| 색 | 의미 |
|---|---|
| 회색 | 현재 또는 명령 직전 팔 |
| 파랑 | 최신 IK 목표 팔 |
| 주황 | 모니터 중심 |
| 초록 | ToF 사용자 X 위치 |
| 노랑 | ToF 사용자 X와 모니터 사이 거리 |
| 청록 점선 | 자동 시작 시 고정한 reference Z |

#### 왜 `mainloop()`를 호출하지 않는가?

자동 프로그램의 주 루프는 OpenCV 카메라 `while True`가 소유한다. 여기서 Tk
`mainloop()`까지 호출하면 카메라 루프로 돌아오지 못한다. 대신 프레임마다 다음을 호출한다.

```python
root.update_idletasks()
root.update()
```

이것이 `pump_events()`다. Tk 이벤트를 조금씩 처리하고 다시 OpenCV 루프로 반환한다.

#### `_request_exit()`

사용자가 Tk 창의 X를 누르면 `exit_requested=True`로 만든 후 창을 닫는다. 메인 루프는
`update_state()`의 False 반환을 보고 카메라와 모터도 종료한다.

#### `close()`

중복 destroy를 피하기 위해 `closed` flag를 확인한다. 이미 사라진 Tk 창을 destroy할 때
발생할 수 있는 `TclError`도 처리한다.

#### `pump_events()`

Tk event 처리 후 프로그램을 계속할지 bool로 반환한다. 창이 비정상적으로 사라져도
전체 자동 프로그램이 종료 방향으로 가도록 `exit_requested=True`로 둔다.

#### `update_state()`

메인 루프가 매 프레임 호출한다. 입력은 다음과 같다.

- 현재/명령 전 `JointCommand`
- 최신 target 또는 `None`
- ToF 사용자 X
- 목표 거리
- 기준 Z
- 상태 문자열
- 모드 문자열

Label 문자열을 갱신하고 `draw_scene()` 후 Tk event를 처리한다.

#### `_world_to_canvas()`

월드 metre 좌표를 Canvas pixel 좌표로 바꾼다.

```text
canvas_x = origin_x + world_x × scale
canvas_y = origin_y - world_z × scale
```

Canvas는 아래로 갈수록 y가 증가하지만 월드 Z는 위로 갈수록 증가하므로 Z 앞에 minus가
붙는다.

#### `_draw_arm()`

네 점을 선으로 연결하고 관절 원을 그린다. 마지막 모니터 점은 주황색으로 그린다.

#### `draw_scene()`

1. Canvas 초기화
2. 현재팔, 목표팔, 사용자 위치까지 모두 들어오도록 scale 결정
3. X/Z 축 그림
4. 기준 Z 점선 그림
5. 현재팔과 목표팔 그림
6. ToF 사용자 X가 있으면 사용자 점과 노란 거리선 그림

시각화의 좌표 계산도 기구학 클래스와 같은 link/offset 값을 사용하므로 그림과 숫자
계산이 따로 놀지 않는다.

---

## 10. 개별 모터 수동 조그: `manual_motor12_limit_ui.py`

이 UI의 목적은 자동 제어가 아니라 실제 가동범위와 안전한 제어 파라미터를 천천히
찾는 것이다. `+/-` 버튼을 누르는 동안만 해당 모터에 새 목표를 보낸다.

### 10.1 상수와 pyserial fallback

```python
JOINTS = ("shoulder_lift", "elbow_flex")
JOG_INTERVAL_MS = 75
STATE_INTERVAL_MS = 300
```

이 tuple 때문에 대부분의 반복이 두 관절에만 한정된다.

프로젝트 안에 포함된 STServo 가상환경의 `site-packages`를 `sys.path` 뒤에 추가하는
fallback도 있다. 시스템에 정상 설치된 pyserial이 있으면 그것이 먼저 사용되고, 없을 때
저장소의 pyserial을 찾을 수 있게 한다.

### 10.2 `ManualMotor12Bus`

UI와 저수준 모터 코드 사이의 adapter다. 자동 모드의 `MotorController`보다 의도적으로
작고, Servo 1·2만 노출한다.

#### 생성자

1. `CalibrationManager`로 JSON 로드
2. `shoulder_lift`, `elbow_flex`의 Servo ID 조회
3. mapping이 정확히 `{shoulder_lift: 1, elbow_flex: 2}`인지 검사

이 검사 덕분에 calibration 파일의 ID mapping이 잘못됐을 때 엉뚱한 모터를 움직이지
않고 시작 단계에서 실패한다.

#### `open()` / `close()`

`ServoDriver`를 직접 생성하거나 닫는다. `ServoDriver` 생성자는 내부에서 포트까지 연다.

#### `read_angles()`

각 Servo raw position을 읽고 `CalibrationManager.position_to_command_angle()`로 팀원용
각도로 바꾼 뒤 `JointCommand`를 반환한다.

#### `read_states()`

position뿐 아니라 load, temperature 등의 상태를 읽고 `angle` 필드를 추가한다.

#### `move_joint(joint, angle_deg, speed, acc)`

한 관절만 움직인다.

1. joint가 `JOINTS` 안인지 검사
2. angle을 calibration-safe raw position으로 변환
3. 해당 Servo ID 하나에 `write_position()`

버튼 하나가 다른 Servo까지 포함하는 SyncWrite를 만들지 않게 설계됐다.

#### `move(target, speed, acc)`

두 관절 목표를 raw position으로 변환해 ID 1·2만 SyncWrite한다. 이 메서드는 Cartesian
IK UI가 공유해서 쓴다.

#### `hold()`

두 Servo의 현재 raw position을 읽고 그 위치를 다시 목표로 SyncWrite한다. 현재 위치에서
힘을 유지하라는 뜻이다.

#### `hold_joint(joint)`

한 Servo만 현재 위치를 읽어 그 위치로 다시 명령한다. 조그 버튼을 놓을 때 사용하며
다른 Servo에는 패킷을 보내지 않는다.

#### `torque_off_1_and_2()`

ID 1, 2 각각 Torque OFF를 보낸다. Hold와 다르게 모터가 힘을 주지 않으므로 중력으로
팔이 떨어질 수 있다.

### 10.3 `ManualJogWindow`

#### 생성자 상태

- `current`: 최근 읽은 실제 두 관절각
- `jog_targets`: 각 관절이 따라가는 누적 목표각
- `jog_directions`: `-1`, `0`, `+1`
- `last_jog_at`: 마지막 조그 계산 시각
- Tkinter 변수: speed, acc, jog rate, soft min/max, 상태 문자열

창이 focus를 잃으면 `stop_all_jogs()`를 호출한다. 마우스를 놓는 이벤트를 놓쳤을 때
계속 움직이는 위험을 줄인다.

#### `_build_ui()`

- 포트/Hold/Torque OFF 버튼
- 조그 속도, Servo speed, acc 입력
- 관절별 큰 `- 아래`, `+ 위` 버튼
- 실제각, 조그 목표각, load, temperature 표시
- calibration hard range와 수정 가능한 soft range 표시
- 현재각을 min/max 후보로 복사하는 버튼

#### `bind_jog_button()`

한 버튼에 세 이벤트를 묶는다.

| 이벤트 | 처리 |
|---|---|
| `<ButtonPress-1>` | 조그 시작 |
| `<ButtonRelease-1>` | 조그 정지 및 Hold |
| `<Leave>` | 누른 채 버튼 밖으로 나가도 정지 |

lambda의 `joint`, `direction`은 메서드 인자로 이미 전달된 지역값을 closure로 기억한다.

#### `connect()`

포트를 열고 현재각을 읽은 뒤 조그 목표를 실제각에 맞춘다. 목표를 0°로 가정하지 않기
때문에 연결 직후 갑자기 영점으로 이동하지 않는다.

#### `sync_jog_targets_to_current()`

두 `jog_targets`를 최근 실제각으로 맞춘다.

#### `start_jog()`

버튼을 누른 순간 현재각을 다시 읽는다. 오래된 UI 목표에서 시작하지 않고 실제 위치를
출발점으로 삼는다. 방향과 `time.monotonic()` 시각을 저장한다.

#### `stop_jog()`

방향을 0으로 만들고 타이머 상태를 지운다. 실제로 조그 중이었다면 `hold_joint()`를 한
번 호출해 현재 위치에 정지시킨다. 그 뒤 반복 tick은 방향 0을 보고 새 패킷을 만들지 않는다.

#### `stop_all_jogs()`

두 관절에 `stop_jog()`를 적용한다. 오류, focus out, 저장, 창 닫기 전에 사용한다.

#### `effective_range(joint)`

```text
effective min = max(calibration hard min, UI soft min)
effective max = min(calibration hard max, UI soft max)
```

하드 범위보다 넓게 soft limit을 입력해도 실제 범위가 넓어지지 않는다.

#### `motion_parameters()`

- jog rate: 0.5~30 degree/second
- Servo speed: 1~manual test cap
- acc: 0~30

값을 숫자로 변환하고 범위를 검사한다.

수동 UI는 안전 speed를 찾기 위한 도구이기 때문에 `CalibrationManager.validate_speed()`의
`max_speed` 검사를 직접 사용하지 않는다. 대신 임시 `manual_test_speed_cap`을 사용한다.
따라서 테스트가 끝난 후 확정한 안전 speed를 calibration JSON의 `max_speed`에 별도로
기록해야 자동 모드를 사용할 수 있다.

#### `jog_tick()`

75ms마다 실행되는 핵심 반복 함수다.

```text
dt = 현재시각 - 이전시각
next_target = old_target + direction × jog_rate × dt
```

`dt`는 최대 0.15초로 제한한다. GUI가 잠깐 멈췄다가 재개됐을 때 긴 경과시간이 한 번에
큰 각도 점프로 바뀌는 것을 방지한다.

계산된 목표를 effective range로 clamp하고, 변화가 있으면 해당 관절 하나에만 명령한다.
함수 마지막 `finally`에서 다시 75ms 뒤 자신을 예약한다.

#### `refresh_state()`

300ms마다 두 모터의 상태를 읽는다. 현재각으로 정기구학을 계산해 모니터 중심 X/Z도
표시한다. 상태 읽기에 실패하면 모든 조그 방향을 0으로 만든다.

#### `use_current_as_limit()`

최근 실제각을 선택한 soft min 또는 max 입력칸에 복사한다. 저장은 아직 하지 않는다.

#### `save_settings_from_ui()`

조그를 멈춘 뒤 effective soft limits, speed, acc를 settings JSON에 atomic save한다.

#### `hold()`, `torque_off()`, `close()`

모두 먼저 조그 반복을 멈춘다. Torque OFF는 사용자 확인창을 거친다. 창을 닫으면 타이머가
재등록되지 않도록 `closing=True`로 만들고 포트를 닫는다.

---

## 11. 사용자 X 기반 수동 IK: `manual_vertical_ik_ui.py`

이 UI는 관절을 따로 움직이지 않고 사용자/모니터 Cartesian 좌표로 Servo 1·2를
제어한다. 이름에 `vertical`이 남아 있지만 현재 주 게이지는 사용자 X다.

파일의 `CartesianIKWindow` 클래스가 설정 상태, Tkinter 위젯, 실시간 IK timer,
시각화를 모두 소유한다. `ManualJogWindow`와 마찬가지로 한 창의 상태를 한 객체에 모은
controller/view 역할의 클래스다.

### 11.1 핵심 상태 구분

이 UI에서 가장 중요한 설계는 입력값과 적용값을 구분하는 것이다.

```text
distance_cm_var / monitor_z_cm_var
    = 사용자가 편집 중인 값

applied_distance_cm / applied_monitor_z_cm
    = 현재 IK와 모터 제어에 실제 사용 중인 값
```

사용자가 고정거리나 Z 입력을 바꿔도 applied 값은 바뀌지 않는다. 반드시
`일정값 적용 및 현재 게이지 위치로 이동`을 눌러야 반영된다. 반면 사용자 X 게이지는
움직이는 즉시 applied 상수들과 조합되어 실시간 제어된다.

### 11.2 생성자

생성자가 준비하는 핵심 객체는 다음과 같다.

- settings와 `ArmGeometry`
- `TwoJointMonitorArm`
- `ManualMotor12Bus`
- 현재/목표 관절각과 pose
- pending/active 상수
- 실시간 타이머 ID와 active flag
- 제어 주기와 목표 도착 각도 tolerance 0.25°

고정거리와 Z `DoubleVar`에는 `trace_add()`를 연결해 사용자가 글자를 직접 입력해도
pending 표시가 즉시 바뀐다.

### 11.3 `_build_ui()`

UI는 다섯 영역으로 나뉜다.

1. Servo 연결, Hold, Torque OFF
2. 사용자 X 게이지와 정확한 수치 입력
3. 고정거리와 고정 Z pending 입력 및 적용 버튼
4. 사용자 X/Z 좌표 범위 설정
5. speed/acc, 재추종, ID 1/2 원점 버튼

오른쪽 Canvas에는 현재팔, 목표팔, 모니터, 사용자, 고정거리 선을 그린다.

### 11.4 `_range_row()`

min/max Spinbox가 반복되므로 한 행 생성 코드를 static helper로 분리했다.

### 11.5 `cartesian_limits_cm()`

네 UI 입력을 float로 읽어 다음 조건을 검사한다.

```text
0 ≤ user_min < user_max
0 ≤ z_min < z_max
```

성공하면 `(user_min, user_max, z_min, z_max)` tuple을 반환한다.

### 11.6 `update_constant_status()`와 `mark_constants_pending()`

- `update_constant_status()`: 현재 applied 거리/Z를 label에 표시
- `mark_constants_pending()`: 입력값이 아직 IK에 반영되지 않았음을 표시

pending 상태에서도 `requested_coordinates()`는 입력 변수 대신 applied 값을 읽는다.

### 11.7 `requested_coordinates()`

1. 사용자 X가 허용범위인지 검사
2. applied Z가 Z 범위인지 검사
3. `monitor_target_from_user()` 호출
4. 사용자 X, 거리, Z, `MonitorPose` 반환

이 함수는 “현재 UI가 실제로 요청하는 Cartesian 목표”의 단일 출처다. 미리보기와
시각화가 같은 함수를 사용하므로 서로 다른 상수를 표시하지 않는다.

### 11.8 `hard_ranges()`와 `validate_joint_limits()`

calibration JSON의 hard angle range와 settings soft limit의 교집합을 만든다. IK 목표가
교집합 밖이면 `KinematicsError`로 차단한다.

### 11.9 `validate_interpolated_path(target)`

현재 실제각에서 이번 step target까지 관절을 선형보간해 경로를 검사한다.

각 샘플에서 두 조건을 본다.

1. 모니터 Z가 사용자가 설정한 절대 Z min/max 안인가?
2. 현재 Z와 step target Z 사이를 직선으로 보간한 expected Z에서 실제 기구학 Z가
   `vertical_tolerance_m`보다 많이 벗어나는가?

자동 planner의 `validate_motion()`은 고정 reference Z와 비교하고, 이 수동 UI 함수는
현재 pose와 step target 사이의 기대 Z 변화도 허용한다는 차이가 있다. 따라서 사용자가
고정 Z를 새 값으로 적용하여 높이를 조정할 수 있다.

### 11.10 `motion_parameters()`

Servo speed와 acc UI 값을 읽고 수동 cap 범위인지 검사한다.

### 11.11 `connect()`

포트를 열고 실제각/pose를 읽는다. 적용 중인 고정거리를 이용해 현재 모니터 X에서
사용자 X를 역산한다. 연결만으로 모터를 움직이지 않는다.

### 11.12 `use_current_coordinates()`

```text
user_x = current_monitor_x + applied_distance
```

결과를 게이지 범위로 clamp하여 표시한다. 고정 Z pending 입력은 바꾸지 않는다.

### 11.13 `on_user_x_changed()`

1. 새 X로 IK 미리보기
2. 포트가 연결됐다면 실시간 추종 요청

게이지와 X Spinbox의 화살표/Enter가 이 함수로 연결된다.

### 11.14 `preview_target()`

1. `requested_coordinates()`
2. `kinematics.inverse()`
3. 관절 limit 검사
4. `kinematics.forward()`로 해결된 pose 재확인
5. label과 Canvas 갱신

오류가 나면 target을 `None`으로 만들어 전송 가능 상태가 남지 않게 한다.

### 11.15 `apply_constants_and_move()`

pending 거리/Z를 실제 applied 값으로 바꾸는 유일한 함수다.

1. pending 숫자와 좌표범위 검사
2. pending 값으로 monitor target/IK 계산
3. joint limit 검사
4. applied 값 변경
5. settings JSON에 목표거리와 기본 Z 저장
6. label 갱신
7. 현재 X 목표 실시간 추종 시작

적용 버튼을 누르기 전에는 pending 값이 모터 명령에 들어가지 않는다.

### 11.16 `request_realtime_control()`

안전한 target과 포트 연결을 확인한 뒤 `realtime_active=True`로 만들고 즉시 첫 tick을
`root.after(0, ...)`로 예약한다. 이미 tick이 예약돼 있으면 중복 timer를 만들지 않는다.

### 11.17 `realtime_control_tick()`

실시간 Cartesian 추종의 중심 함수다.

1. 실제 Servo 1·2 각도를 다시 읽는다.
2. 최신 X 게이지로 target을 다시 계산한다.
3. 목표와 실제각의 최대 차이를 구한다.
4. 0.25° 이하면 목표 도달로 보고 반복 종료
5. 차이가 크면 `max_joint_step_deg` 비율만큼 중간 target 생성
6. 관절 한계 및 Z 경로 검사
7. Servo 1·2 SyncWrite
8. `control_interval_ms` 뒤 다음 tick 예약

사용자가 게이지를 크게 이동해도 전체 IK 목표를 한 번에 보내지 않고 최대 관절 step으로
나눈다. 게이지를 놓아도 마지막 목표에 도달할 때까지 반복하고, 도달하면 더 이상 새
position 패킷을 보내지 않는다.

### 11.18 `cancel_realtime_control()`

active flag를 내리고 예약된 Tk timer가 있으면 `after_cancel()`한다. Hold, Torque OFF,
원점 복귀, 설정 저장, 창 닫기 전에 호출된다.

### 11.19 `save_and_apply_ranges()`

사용자 X/Z 범위만 저장한다. pending 고정거리/Z를 몰래 적용하지 않는다. 진행 중인
실시간 추종도 취소하므로 “모터 명령 없음” 상태가 보장된다.

### 11.20 `refresh_current_state()`

350ms마다 실제각, 모니터 X/Z, load를 갱신한다. 조그 tick과 마찬가지로 `after()` 기반
반복이며 별도 thread가 아니다.

### 11.21 `hold()`와 `torque_off()`

실시간 추종을 먼저 취소한다. Hold는 현재 위치를 목표로 다시 쓰고, Torque OFF는 사용자
확인을 받은 뒤 Servo 1·2의 힘을 해제한다.

### 11.22 `move_zero(joint)`

인자에 따라 다음 명령을 만든다.

| 인자 | 동작 |
|---|---|
| `"shoulder_lift"` | ID 1만 command angle 0° |
| `"elbow_flex"` | ID 2만 command angle 0° |
| `None` | ID 1·2 모두 0° SyncWrite |

0°는 raw position 0이 아니라 calibration JSON의 `zero_position`을 뜻한다. 큰 이동이면
확인창을 띄운다.

원점 복귀는 고정 Z를 유지하는 IK가 아니다. 따라서 팔을 지지하고 충돌을 확인해야 한다.

### 11.23 시각화 메서드

- `world_to_canvas()`: metre X/Z → pixel
- `arm_points()`: 현재 파일 안에서 base/shoulder/elbow/monitor 네 점 계산
- `draw_arm()`: 링크와 관절 그림
- `draw_scene()`: 축, 현재/목표팔, 사용자, 고정거리선 전체 갱신

자동 시각화의 `calculate_arm_points()`와 목적은 같다. 수동 UI는 한 파일 안에서 독립적으로
그릴 수 있도록 메서드로 포함하고 있다.

### 11.24 `close()`와 `main()`

창을 닫을 때 실시간 timer를 취소하고 포트를 닫은 뒤 Tk root를 destroy한다.
`main()`은 일반적인 Tkinter 방식으로 root를 만들고 `mainloop()`를 실행한다.

---

## 12. 캘리브레이션 데이터: `servo_calibration_result.json`

이 JSON은 실제 모터를 팀원용 각도 좌표계에 연결하는 기준이다.

### 최상위 필드

- `device`: 시리얼 장치, 현재 `/dev/ttyACM0`
- `baudrate`: 현재 1,000,000
- `servos`: ID 문자열을 key로 갖는 Servo 정보

### Servo별 주요 필드

| 필드 | 의미 |
|---|---|
| `servo_id` | 실제 bus ID |
| `joint` | 코드에서 사용하는 관절 이름 |
| `direction` | URDF 각도 증가와 raw position 변화 관계 |
| `zero_position` | 팀원용/URDF 0°에 대응하는 raw encoder position |
| `safe_position_at_min_angle` | 실측 안전 한쪽 끝 raw position |
| `safe_position_at_max_angle` | 실측 안전 반대쪽 끝 raw position |
| `max_speed` | 실측 안전 최대 speed. 현재 null이면 자동 모드 차단 |

Servo 1 영점 raw position은 1652, Servo 2는 2010이다. 따라서 원점 버튼이 보내는
목표는 raw 0이 아니다.

JSON에는 Servo 3·4 정보도 있지만 현재 모니터 앞뒤 2축 코드가 만드는 command dict에는
그 joint 이름이 들어가지 않는다.

---

## 13. 각도와 raw position 변환: `motor_control`

### 13.1 세 가지 방향 계층

이 부분은 가장 헷갈리기 쉬우므로 분리해서 생각해야 한다.

```text
[팀원용 명령각]
shoulder + = 팔 끝이 위
        ↓ COMMAND_TO_URDF_DIRECTION
[URDF 관절각]
로봇 모델의 수학적 +방향
        ↓ calibration JSON direction
[STS raw position]
0~4095 encoder tick
```

`COMMAND_TO_URDF_DIRECTION=-1`과 JSON의 `direction=1`은 모순이 아니다. 서로 다른 두
단계의 방향을 나타낸다.

Servo 1 예시는 다음과 같다.

```text
팀원 +10°
→ URDF -10°                  # command direction -1
→ raw position 감소          # calibration direction +1
```

### 13.2 `motor_control/config.py`

패키지 공통 상수를 정의한다.

- calibration 기본 경로
- STServo SDK 경로
- 기본 device/baudrate
- raw position 범위 0~4095
- `POSITION_PER_DEGREE = 4096 / 360`
- command→URDF 방향
- 팀원 +명령에서 기대하는 raw 증가/감소 부호
- acc 범위, wait timeout, position tolerance
- Torque Enable 주소와 ON/OFF 값

`EXPECTED_RAW_SIGN_FOR_POSITIVE_COMMAND`는 calibration direction과 command direction을
곱한 최종 raw 변화가 실제 확인한 방향과 같은지 검사하는 기준이다.

### 13.3 `motor_control/calibration.py`

#### `CalibrationError`

캘리브레이션 누락, 방향 불일치, speed 초과, raw hard range 초과를 표현한다.

#### `CalibrationManager.__init__()`

JSON을 읽고 `servos_by_id`, `servos_by_joint` 두 lookup dict를 만든 뒤 방향 설정 전체를
검증한다.

#### `_load()`

- 파일 존재 확인
- JSON 읽기
- device, baudrate 로드
- Servo ID 문자열을 int로 변환
- joint 이름 기준 dict도 구성

같은 Servo 데이터를 ID와 joint 두 방식으로 빠르게 찾기 위한 indexing이다.

#### `_validate_direction_configuration()`

각 joint마다 다음을 계산한다.

```text
actual raw sign
= calibration direction × command-to-URDF direction
```

이 결과가 `EXPECTED_RAW_SIGN_FOR_POSITIVE_COMMAND`와 다르면 시작부터
`CalibrationError`를 낸다. 방향 설정 실수로 반대로 움직이는 위험을 줄인다.

#### `get_joint(joint_name)`

joint 이름으로 Servo dict를 찾는다. 없으면 사용 가능한 이름과 함께 오류를 낸다.

#### `require_position_calibrated(joint_name)`

각도 제어에 필요한 다음 값이 모두 있는지 검사한다.

- `zero_position`
- `safe_position_at_min_angle`
- `safe_position_at_max_angle`

하나라도 `None`이면 raw 제어로 우회하지 않고 차단한다.

#### `validate_speed()`

speed가 양의 정수인지, Servo별 `max_speed`가 설정됐는지, 요청이 그 이하인지 검사한다.

#### `validate_acc()`

acc가 0~254 범위의 정수인지 검사한다.

#### `command_angle_to_position()`

핵심 변환식은 다음과 같다.

```text
urdf_angle = team_angle × command_direction

raw_target
= zero_position
+ calibration_direction × urdf_angle × position_per_degree
```

round 후 `validate_target_position()`을 거쳐 안전한 raw만 반환한다.

#### `position_to_command_angle()`

위 변환의 역연산이다.

```text
urdf_angle
= (raw - zero) × calibration_direction × degree_per_position

team_angle = urdf_angle × command_direction
```

direction 값이 ±1이기 때문에 역변환에서도 곱셈으로 쓸 수 있다.

#### `get_safe_position_range()`

두 calibration 끝점은 방향에 따라 숫자 순서가 뒤집힐 수 있으므로 `min`, `max`로 정렬한다.

#### `get_safe_angle_range()`

두 raw 끝점을 팀원용 각도로 변환한 뒤 다시 정렬한다. UI hard range 표시에 사용한다.

#### `validate_target_position()`

1. STS 전체 범위 0~4095인지
2. Servo별 실측 safe raw range 안인지

두 검사를 모두 통과해야 한다.

### 13.4 `motor_control/servo_driver.py`

Joint나 각도 개념을 모르는 저수준 통신 계층이다. 입력은 Servo ID와 raw position이다.

#### `ServoDriver.__init__()` / `open()` / `close()`

SDK `PortHandler`와 packet handler를 만들고 포트를 연다. `_io_lock`으로 모든 패킷
구간을 보호한다.

#### `ping()`

Servo ID가 응답하는지 확인하고 model number와 통신 상태를 반환한다.

#### `write_position()`

한 Servo에 raw position, speed, acc를 `WritePosEx`로 보낸다.

#### `sync_write_positions()`

여러 Servo command를 SDK buffer에 넣은 뒤 한 번의 packet으로 전송한다. 중간에 실패해도
`finally`에서 buffer를 clear한다.

Servo 1·2를 같은 순간 출발시키는 데 사용한다.

#### `set_torque()`

한 Servo의 Torque Enable register를 쓴다.

#### `disable_torque_all_sync()`

여러 Servo에 Torque OFF를 한 sync packet으로 보낸다. E-Stop용이다. 응답을 개별적으로
기다리지 않으므로 반환 성공은 “송신 성공”이지 실제 토크 상태를 읽어 확인했다는 뜻은
아니다.

#### `read_position()`, `read_speed()`

통신 성공 시 int, 실패 시 `None`을 반환한다.

#### `read_state()`

position, speed, load, voltage, temperature, current raw, moving register를 읽어 dict로
반환한다.

`load_percent`는 STS load register를 백분율로 바꾼 값이며 정밀 토크 센서의 Nm 값이
아니다. `current_raw`도 아직 mA 변환계수를 확정하지 않아 raw 그대로다.

### 13.5 `motor_control/controller.py`

팀원이 joint 이름과 각도로 사용하는 상위 API다.

#### 생성자

- `CalibrationManager`
- `ServoDriver`
- Emergency Stop `threading.Event`
- 명령/E-Stop 경합 방지 `RLock`

#### `emergency_stop()`

1. software emergency latch를 먼저 set
2. command lock 안에서 모든 Servo Torque OFF sync packet
3. 이후 모든 이동 API 차단

Torque OFF는 전원을 물리적으로 끊는 하드웨어 E-Stop과 같지 않으며, 팔이 중력으로
떨어질 수 있다.

#### `move_joint()`

한 joint의 절대 팀원용 각도를 검증하고 raw로 변환해 보낸다. `wait=True`면 목표 도착까지
polling한다.

#### `move_joint_relative()`

현재 raw position을 읽어 팀원용 각도로 변환하고 delta를 더한 뒤 `move_joint()`에
위임한다. 최종 목표는 같은 안전검사를 다시 거친다.

#### `move_joints()`

여러 joint를 모두 먼저 검증한다. 하나라도 실패하면 실제 패킷을 보내지 않는다. 전부
성공하면 SyncWrite한다.

현재 자동 모터 adapter는 이 함수에 Servo 1·2 key만 넘긴다.

#### `move_to_zero()`, `move_all_to_zero()`

0° 목표를 기존 이동 함수에 위임한다. `move_all_to_zero()`는 calibration에 들어 있는
모든 joint를 대상으로 하므로 현재 2축 UI의 개별 원점 기능은 이 함수를 쓰지 않고
ID 1·2만 명시적으로 제어한다.

#### `get_joint_angle()`, `get_joint_state()`, `get_all_states()`

raw 상태를 읽고 팀원용 각도와 사람이 보기 쉬운 dict로 바꾼다. E-Stop 상태에서도
상태 읽기는 허용된다.

#### 작은 상태/보조 API

- `_print_error()`: 오류 메시지에 `[MOTOR ERROR]` prefix를 붙인다.
- `_check_emergency_state()`: latch가 set됐으면 이동 함수가 False로 끝나게 한다.
- `is_emergency_stopped()`: 현재 latch 상태를 bool로 반환한다.
- `is_moving(joint_name)`: `get_joint_state()`의 moving 값을 간단히 꺼낸다.
- `close()`: 시리얼 포트만 닫는다. 자동 Torque OFF나 E-Stop은 수행하지 않는다.

#### `_wait_for_targets()`

목표 raw와 현재 raw의 오차가 tolerance 이하이고 `moving==0`이 될 때까지 polling한다.
timeout 또는 E-Stop이면 False를 반환한다.

#### context manager

`__enter__`, `__exit__`가 있어 다음 패턴도 가능하다.

```python
with MotorController(...) as arm:
    arm.move_joint(...)
# 블록을 나가면 close()
```

### 13.6 `motor_control/__init__.py`

패키지 사용자가 내부 파일 경로를 알 필요 없이 다음처럼 import하게 한다.

```python
from motor_control import MotorController
```

`__all__`은 패키지가 공식적으로 공개하는 이름을 나타낸다.

---

## 14. 테스트 코드: `test_pose_monitor_arm_controller.py`

단위 테스트는 실제 카메라나 모터 없이 순수 계산과 command 범위를 검증한다.

### `FakeManualDriver`

실제 시리얼 대신 write 내용을 list에 저장하는 test double이다. Servo 1·2 영점 raw를
초기 position으로 둔다.

### `ManualJogBusTests`

- Servo 1 개별 조그 packet에 ID 1만 들어가는지
- Servo 2 Hold packet에 ID 2만 들어가는지
- 두 관절 0° 명령이 ID 1·2만 포함하고 raw 영점 1652/2010인지

### `FixedToFUserXSourceTests`

- 센서 원점 X와 range 합이 사용자 X가 되는지
- 허용범위 밖 ToF 사용자 X가 차단되는지
- 고정 사용자 X에서도 가상 모니터가 목표 50cm로 수렴하는지

### `TwoJointKinematicsTests`

- 7cm offset 적용
- 고정 베이스 링크가 길이를 보존한 수직선인지
- 사용자 X와 monitor X 차이가 고정거리인지
- user target → IK → FK round trip
- command → FK → IK round trip
- 시각화 끝점과 FK 결과 일치
- 수직 target 변경 시 X 유지
- 너무 작은 Z tolerance에서 경로 차단
- planner가 기준 높이 허용편차 유지
- ToF 목표 monitor X가 현재보다 크면 +X, 작으면 -X인지

테스트 실행:

```bash
python -m unittest -v test_pose_monitor_arm_controller.py
```

현재 테스트는 수학, 메시지 대상 ID, 가상 수렴을 검증한다. 실제 Servo 방향, 구조물 충돌,
하중, 카메라 진동은 하드웨어 통합시험이 필요하다.

---

## 15. 실행 모드별로 코드 따라 읽기

### 15.1 기본 고정 ToF + Pose + IK 시뮬레이션

```bash
python pose_monitor_arm_controller.py
```

추천 breakpoint/print 순서:

1. `FixedToFUserXSource.read_user_x_m()`의 사용자 X
2. `monitor_target_from_user()`의 target monitor X
3. `MonitorArmPlanner.plan()`의 `monitor_x_error`, `x_step`
4. `TwoJointMonitorArm.inverse()`의 `dx`, `dz`, `radius`
5. `target.shoulder_lift_deg`, `target.elbow_flex_deg`
6. `TwoJointMonitorArm.forward(target)`의 X/Z

### 15.2 다른 ToF 사용자 X 모의시험

```bash
python pose_monitor_arm_controller.py --tof-user-x-m 0.78
```

OpenCV의 값을 비교한다.

```text
tof_user_x  : 베이스 기준 사용자 몸통 X
user-monitor: ToF 사용자 X - 현재 monitor X
```

ToF 사용자 X는 고정되어도 가상팔 monitor X가 움직이므로 user-monitor 거리는 목표
50cm 쪽으로 수렴한다.

### 15.3 실제 모터 자동 모드

```bash
python pose_monitor_arm_controller.py --enable-motor
```

실행 전 필수 조건:

- Servo 1·2 `max_speed` 확정
- soft limit 실측
- 모니터를 제거하거나 확실히 지지한 초기 방향 시험
- 다른 프로그램이 `/dev/ttyACM0`을 사용하지 않음

코드 추적 순서:

1. `MotorControlProcessClient.open()`
2. `motor_process_worker()` 초기화
3. `TwoMotorHardware.open()`
4. 메인의 `planner.plan()`
5. client `move()` → request Queue
6. worker `move` branch
7. `TwoMotorHardware.move()`
8. `MotorController.move_joints()`
9. `CalibrationManager.command_angle_to_position()`
10. `ServoDriver.sync_write_positions()`

### 15.4 수동 조그

```bash
python manual_motor12_limit_ui.py
```

먼저 가장 낮은 jog rate와 speed로 시작하고 모니터를 지지한다. 조그 rate와 Servo speed가
서로 다른 개념임을 기억한다.

### 15.5 수동 Cartesian IK

```bash
python manual_vertical_ik_ui.py
```

- X 게이지: 연결 후 즉시 실시간 목표 갱신
- 고정거리/Z 입력: 적용 버튼 전까지 pending
- 원점: 고정 Z를 유지하지 않는 calibration 0° 이동

---

## 16. 포코에 이식할 때 유지할 경계

### 메인/포즈 계산 측 책임

- 카메라 frame 수신
- Pose landmark 추론
- ToF range 수신과 베이스 기준 사용자 X 계산
- Pose 결과로 향후 나쁜 자세일 때 자동 이동 허용/차단
- 현재 Servo 1·2 각도 telemetry 수신
- 정기구학/역기구학
- 목표거리, reference Z, deadband, direct/stepped 명령 방식
- 목표 `shoulder_lift_deg`, `elbow_flex_deg` 생성

### 하드웨어 프로세스 책임

- 시리얼 포트 단독 소유
- calibration JSON 로드
- 팀원용 각도 → raw position
- calibration hard range와 max speed 검사
- Servo 1·2 SyncWrite
- 현재각/load/temperature telemetry 발행
- 통신 오류와 E-Stop 처리

### 권장 angle command 메시지

```python
{
    "type": "monitor_arm_joint_target",
    "sequence": 123,
    "created_monotonic": 456.789,
    "shoulder_lift_deg": 12.3,
    "elbow_flex_deg": -4.5,
}
```

현재 데모 메시지에는 두 각도와 request ID가 핵심이다. 포코 이식 시에는 오래된 명령을
버리기 위한 sequence/timestamp를 추가하는 것이 좋다.

### 권장 telemetry 메시지

```python
{
    "type": "monitor_arm_joint_state",
    "sequence": 123,
    "shoulder_lift_deg": 12.0,
    "elbow_flex_deg": -4.3,
    "load_percent": {"shoulder_lift": 20.0, "elbow_flex": 18.0},
    "temperature_c": {"shoulder_lift": 35, "elbow_flex": 36},
}
```

메인 planner가 사용하는 current는 “마지막 명령각”보다 “하드웨어가 최근 읽은 실제각”이
더 안전하다.

### 비동기화 시 추가할 안전정책

- 목표 Queue가 밀리면 가장 최신 명령만 남기기
- 일정 시간보다 오래된 명령 폐기
- telemetry timeout이면 새 이동 차단
- 프로세스 종료 시 Hold/Torque 정책 명시
- sequence가 역행하면 명령 무시
- 모터 프로세스 heartbeat 감시

---

## 17. 자주 헷갈리는 점과 현재 한계

### 현재 ToF 값은 실제 측정이 아니라 고정 모의값이다

`tof.fixed_range_m`은 실제 센서가 아니라 시험용 상수다. 실제 통합에서는 ToF read 결과로
교체하고 timeout, 유효범위, 순간 튐 필터를 센서 계층에 추가해야 한다.

### 카메라 Pose는 현재 이동 차단 gate가 아니다

Pose Landmarker를 실행하지만 아직 거북목/정상자세 판정 조건은 구현하지 않았다. 현재는
ToF 사용자 X가 유효하면 제어한다. 포코 병합 시 자세 프로세스의 판정 결과를 planner
호출 전 gate로 연결해야 한다.

### 2축 평면 모델이다

현재 IK에는 yaw/roll, 3D 사용자 위치, wrist 자세가 없다. 모터 3·4는 짐벌 팀의 별도
책임이다.

### 7cm는 하부 링크 방향 offset이다

단순히 월드 X에 7cm를 더하는 것이 아니다. 하부 링크가 기울면 offset도 같은 방향으로
기울어져 X와 Z 모두에 영향을 준다.

### 수직 안전은 토크 제한이 아니다

`vertical_tolerance_m`은 기구학 자세 방어다. 실제 토크를 직접 제한하지 않는다. 실제
보호를 위해서는 load/current/temperature, 속도, acceleration, 기계적 limit도 함께 봐야 한다.

### Hold와 Torque OFF는 반대에 가깝다

- Hold: 현재 위치를 계속 목표로 하여 힘을 유지
- Torque OFF: 힘을 해제하여 사람이 움직일 수 있지만 팔이 떨어질 수 있음

### 원점은 안전한 작업 자세라는 뜻이 아니다

원점은 calibration 기준 0°다. 현재 주변 구조물과 모니터 장착 상태에서 충돌 없는 자세인지는
별도로 확인해야 한다.

### 자동 모드와 수동 UI를 동시에 실행하면 안 된다

둘이 같은 시리얼 포트와 Servo에 서로 다른 목표를 보낼 수 있다. 한 번에 하나만 실행한다.

### 현재 motor-process RPC는 동기식이다

하드웨어 처리는 자식 프로세스지만 메인은 read/move 응답을 기다린다. 포코의 완전한
실시간 구조에서는 비동기 command와 최신 telemetry cache가 더 적합할 수 있다.

---

## 18. 추천 학습 순서와 실습 문제

### 1단계: 순수 수학

읽을 파일:

1. `monitor_arm_settings.json`
2. `monitor_arm_kinematics.py`
3. `TwoJointKinematicsTests`

실습:

- `JointCommand(0, 0)`의 X/Z를 손으로 식에 넣어 보기
- `monitor_offset_m`을 0으로 바꾼 복사 settings에서 끝점 차이 확인
- 임의 command를 FK→IK로 왕복하고 오차 출력
- 도달 불가능한 X/Z를 넣어 `KinematicsError` 확인

### 2단계: ToF 사용자 X와 planner

읽을 클래스:

1. `FixedToFUserXSource`
2. `MonitorArmPlanner`

실습:

- 센서 origin 0/2cm와 range 70cm의 사용자 X 계산
- 사용자 X 70/75/80cm에서 monitor 목표 X 계산
- deadband를 0으로 했을 때 흔들림 예상
- 현재 monitor X가 목표의 양쪽에 있을 때 이동 부호 확인

### 3단계: 시각화

읽을 파일:

1. `monitor_arm_visualizer.py`
2. `manual_vertical_ik_ui.py`의 drawing 메서드

실습:

- 월드 Z에 minus를 빼면 Canvas 그림이 왜 뒤집히는지 확인
- current와 target 색을 바꿔 보기
- elbow point와 monitor point 좌표를 Label로 추가

### 4단계: 이벤트 기반 수동 제어

읽을 파일:

1. `manual_motor12_limit_ui.py`
2. `manual_vertical_ik_ui.py`

실습:

- `after()`를 작은 예제 창에서 10회만 실행
- jog `dt` cap을 제거했을 때 위험 시나리오 생각하기
- pending 값과 applied 값을 print로 비교
- 큰 X 이동이 여러 joint step으로 나뉘는지 로그 출력

### 5단계: calibration과 raw

읽을 파일:

1. `motor_control/config.py`
2. `motor_control/calibration.py`
3. `servo_calibration_result.json`

실습:

- Servo 1의 command 0°가 raw 1652인지 확인
- command +10°의 예상 raw를 식으로 계산
- safe raw 끝점을 command angle로 변환
- direction 중 하나를 복사 JSON에서 바꾸고 방향 검증 오류 확인

실제 calibration 원본 JSON은 실습 중 직접 훼손하지 말고 복사본을 사용한다.

### 6단계: 프로세스와 실제 패킷 경계

읽을 파일:

1. `monitor_arm_motor_process.py`
2. `motor_control/controller.py`
3. `motor_control/servo_driver.py`

실습:

- `_serialize_angles()` 결과 확인
- request ID가 다른 응답을 받았을 때 오류 흐름 따라가기
- `move` message가 최종 SyncWrite command로 변하는 계층을 종이에 그리기
- 동기 RPC를 비동기 latest-command Queue로 바꾸는 설계 작성

---

## 19. 한 문장으로 각 계층 기억하기

```text
pose_monitor_arm_controller.py
    = 보고, 거리를 재고, 다음 두 각도를 결정한다.

monitor_arm_kinematics.py
    = 각도와 X/Z를 서로 바꾸고 그 이동이 안전한지 계산한다.

monitor_arm_motor_process.py
    = 계산된 두 각도를 별도 프로세스의 실제 모터 명령으로 전달한다.

monitor_arm_visualizer.py
    = 계산 상태를 사람이 이해할 수 있는 X-Z 그림으로 바꾼다.

manual_motor12_limit_ui.py
    = 실제 모터를 천천히 하나씩 움직이며 안전범위를 찾는다.

manual_vertical_ik_ui.py
    = 사용자 X와 고정거리/Z를 직접 조작하며 IK 전체 흐름을 시험한다.

CalibrationManager
    = 팀원용 각도와 안전한 raw position 사이의 번역기다.

ServoDriver
    = 번역된 raw 값을 실제 STServo 패킷으로 보내는 가장 아래 계층이다.
```

이 계층 구분이 유지되면 카메라를 Pi Camera로 바꾸거나, IR 센서를 추가하거나, 포코의
Queue 형식으로 바꾸더라도 IK 수학과 calibration 안전 계층을 다시 작성할 필요가 없다.

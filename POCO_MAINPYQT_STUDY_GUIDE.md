# POCO `mainpyQt.py` 전체 구조 학습 가이드

> 대상 코드: `WorkSpace/pyQt/mainpyQt.py`를 진입점으로 하는 통합 실행 경로  
> 코드 기준일: 2026-09-02  
> 기준 브랜치: `Feature_Multiprocessing`  
> 독자 수준: 파이썬 중급자  
> 목적: 코드를 처음부터 따라가며 UI, 비전 AI, 센서, 모터 제어, 안전 로직을 한 번에 이해하기

---

## 문서의 범위와 먼저 알아둘 점

이 문서는 `python3 WorkSpace/pyQt/mainpyQt.py`를 실행했을 때 실제로 사용되는 통합 경로를 설명한다. 별도의 `BH_CODE` 시험 도구나 단독 센서 시험 프로그램은 중심 설명에서 제외하고, 메인 실행 중 호출되는 파일과 서비스에 초점을 맞춘다.

현재 통합 실행 모드는 다음 코드로 결정된다.

```python
# WorkSpace/pyQt/managers/vision_process_manager_profile.py
PROFILE_MODE = "POSE_ONLY"
```

따라서 현재 제품 통합본의 실제 실행 구성은 다음과 같다.

- 자세 판정: 실행함
- 사용자 거리 추종 및 모니터암 제어: 실행함
- 자세 기반 부저 알림: 실행함
- 얼굴 기반 졸음 판정: 코드와 모델 연결 구조는 남아 있으나 현재 실행하지 않음
- 결과 UI의 피로도 표시: 현재 주석 처리됨

이 구분은 중요하다. “졸음 기능이 삭제되었다”가 아니라, 성능이 충분히 확보될 때까지 `POSE_ONLY`로 런타임에서 제외한 상태다. 나중에 `FACE_ONLY` 또는 `BOTH`로 변경하면 Face Process가 추가로 생성된다.

> **코드 기준 주의 사항**  
> 본 문서는 기준일 당시의 실제 소스와 설정값을 바탕으로 작성되었다. 모터 링크 길이, 영점, 안전 각도, 속도 같은 값은 기구물 재조립과 튜닝에 따라 바뀔 수 있으므로, 항상 현재의 `WorkSpace/config/monitor_arm_settings.json`과 함께 확인해야 한다.

---

## 목차

1. 5분 만에 보는 전체 개요
2. 중급 파이썬 개발자를 위한 핵심 용어
3. 프로젝트 디렉터리와 파일 역할
4. 런타임 구성: 프로세스, 스레드, 공유 메모리
5. 프로그램 시작부터 화면 표시까지
6. `MainWindow` 상세 해설
7. `CameraWorker` 상세 해설
8. `VisionProcessManager`와 IPC 설계
9. `VisionResultWorker` 상세 해설
10. Pose Process와 자세 판정
11. Calibration과 사용자 프로필
12. Face Process와 졸음 판정 경로
13. Hardware Process 전체 루프
14. ToF 거리 측정과 사용자 X 좌표
15. 모터 1·2 역기구학 제어
16. IMU와 모터 3·4 짐벌 제어
17. 안전 상태 머신과 예외 처리
18. 측정 준비 화면과 수동조작
19. 자세 알림과 부저
20. 설정, 상태, 파일 저장
21. 사용자 시나리오별 전체 흐름
22. 종료와 자원 정리
23. 성능을 위해 고려된 설계
24. 오류 처리와 현재 주의할 지점
25. 디버깅 가이드
26. 테스트 방법과 권장 읽기 순서
27. 부록: 이벤트, 상태, 함수 사전

<div style="page-break-after: always;"></div>

# 1. 5분 만에 보는 전체 개요

## 1.1 이 프로그램은 무엇을 하는가

POCO 통합 프로그램은 사용자의 자세와 거리를 실시간으로 관찰하고, 정상 자세일 때 모니터암이 사용자를 따라 적절한 거리를 유지하도록 제어한다.

큰 기능은 다섯 묶음으로 나뉜다.

1. **PyQt 사용자 인터페이스**
   - 보정, 프로필 선택, 측정 시작·종료, 설정, 리포트 화면을 제공한다.
2. **카메라와 비전 AI**
   - 카메라 영상을 읽고 MediaPipe로 랜드마크를 찾는다.
   - Pose GRU 모델로 정자세, 비대칭, 거북목, 턱 괴기를 분류한다.
3. **거리와 자세 보정**
   - 사용자의 기준 자세 특징값, ToF 평균, 눈 사이 거리, IMU 기준값을 저장한다.
4. **모니터암 제어**
   - 모터 1·2는 2관절 역기구학으로 모니터의 앞뒤 위치를 제어한다.
   - 모터 3·4는 IMU 오차를 이용해 모니터 기울기를 보정한다.
5. **안전과 알림**
   - 사용자 또는 센서 미검출, 낮은 자세 신뢰도, 비정상 자세에서는 자동 추종을 멈춘다.
   - 나쁜 자세가 일정 시간 지속되면 부저 패턴을 실행한다.

## 1.2 가장 중요한 구조 한 장 요약

```text
┌──────────────────────────── Main Process ────────────────────────────┐
│                                                                      │
│  MainWindow (PyQt GUI thread)                                        │
│      │ 버튼/화면 갱신                                                 │
│      ▼                                                               │
│  CameraWorker (QThread) ── 카메라 읽기 ──┬─ 화면용 QImage            │
│      │                                   └─ 공유 메모리 Frame Ring   │
│      │                                                             │
│  VisionResultWorker (QThread) ◀── 결과/상태/이벤트 Queue             │
└───────────────┬──────────────────────────────┬───────────────────────┘
                │                              │
       ┌────────▼────────┐            ┌────────▼──────────┐
       │  Pose Process   │            │ Hardware Process │
       │ MediaPipe Pose  │── 상태 ───▶│ ToF / IMU        │
       │ Calibration     │            │ Motor 1~4        │
       │ Pose GRU        │◀── 상태 ───│ Safety / Buzzer  │
       └─────────────────┘            └───────────────────┘

현재 POSE_ONLY에서는 Face Process를 만들지 않는다.
BOTH로 바꾸면 Pose Process와 같은 높이에 Face Process가 추가된다.
```

## 1.3 왜 한 프로세스에서 전부 하지 않는가

MediaPipe 추론, 카메라 캡처, PyQt 이벤트 처리, 센서 I/O, 모터 통신을 한 루프에 넣으면 어느 하나가 오래 걸릴 때 전체가 멈춘다. 예를 들어 MediaPipe가 100 ms 걸리면 UI 클릭 반응과 모터 명령도 함께 늦어진다.

이 프로젝트는 역할을 다음과 같이 분리했다.

| 실행 단위 | 핵심 책임 | 오래 걸려도 직접 막지 않는 대상 |
|---|---|---|
| GUI 메인 스레드 | 화면과 사용자 입력 | AI 추론, 센서 통신 |
| CameraWorker QThread | 카메라 프레임 획득 | GUI 이벤트 루프 |
| VisionResultWorker QThread | 여러 IPC 큐 소비 | GUI 이벤트 루프 |
| Pose Process | Pose 랜드마크·GRU | GUI, 하드웨어 루프 |
| Face Process | 얼굴·졸음 GRU, 현재 비활성 | GUI, 하드웨어 루프 |
| Hardware Process | 센서·모터·안전·부저 | GUI, 비전 추론 |

## 1.4 코드를 따라갈 때의 핵심 질문

이 프로젝트를 공부할 때는 각 줄보다 다음 네 질문을 먼저 붙잡는 것이 좋다.

- 이 코드는 **어느 프로세스 또는 스레드에서 실행되는가?**
- 데이터가 **최신 상태(State)** 인가, **유실되면 안 되는 이벤트(Event)** 인가?
- 이 값은 **측정값**, **명령값**, **저장된 기준값** 중 무엇인가?
- 모터 명령 직전에 어떤 **안전 조건**이 통과되어야 하는가?

이 네 가지를 구분하면 복잡해 보이는 코드가 “화면 계층 → 전달 계층 → 계산 계층 → 하드웨어 계층”으로 정리된다.

<div style="page-break-after: always;"></div>

# 2. 중급 파이썬 개발자를 위한 핵심 용어

## 2.1 프로세스와 스레드

**프로세스(Process)** 는 독립된 파이썬 실행 공간이다. 메모리를 기본적으로 공유하지 않으므로 Queue나 Shared Memory 같은 IPC가 필요하다. 대신 Pose 추론이 무거워도 Hardware Process의 제어 루프와 분리된다.

**스레드(Thread)** 는 같은 프로세스의 메모리를 공유한다. `CameraWorker`와 `VisionResultWorker`는 PyQt의 `QThread`이며, 결과를 PyQt Signal로 GUI 스레드에 전달하기 쉽다.

이 코드에서 구분은 다음과 같다.

```text
운영체제 프로세스 경계: multiprocessing.Process
메인 프로세스 내부 작업 스레드: PyQt5.QtCore.QThread
GUI 위젯을 실제로 변경하는 곳: GUI 메인 스레드의 slot
```

PyQt 위젯은 작업 스레드에서 직접 변경하지 않는 것이 원칙이다. 대신 Worker가 Signal을 발생시키고 MainWindow의 Slot이 화면을 바꾼다.

## 2.2 Signal과 Slot

PyQt의 **Signal** 은 “어떤 일이 생겼다”는 알림이고, **Slot** 은 그 알림을 받아 실행되는 함수다.

```python
self.camera_worker.frame_changed.connect(self.update_camera_view)
```

- `frame_changed`: 카메라 워커가 발생시키는 Signal
- `connect(...)`: Signal과 처리 함수를 연결
- `update_camera_view`: GUI 스레드에서 실행되는 Slot

## 2.3 IPC

**IPC(Inter-Process Communication)** 는 프로세스 사이 통신이다. 이 프로젝트는 두 종류를 쓴다.

- `multiprocessing.Queue`: 명령, 결과, 상태, 이벤트 전달
- `multiprocessing.shared_memory`: 큰 영상 프레임 전달

영상 한 장은 320×240×3 바이트로 약 230 KB다. 이를 매 프레임 Queue로 직렬화하면 복사와 pickle 비용이 커진다. 그래서 영상 배열은 공유 메모리에 쓰고, 동기화용 인덱스와 Semaphore만 공유한다.

## 2.4 State와 Event

**State** 는 지금 현재값만 중요하다.

- 현재 ToF 거리
- 현재 모터 각도
- 가장 최근 Pose 랜드마크
- 현재 안전 상태

State Queue는 크기 1이며 새 값이 오면 오래된 값을 버린다. 화면이 잠깐 느렸다고 2초 전 센서값을 차례대로 표시할 이유가 없기 때문이다.

**Event** 는 발생 순서와 유실 방지가 중요하다.

- 보정 시작
- 보정 완료
- 측정 시작 ACK
- 프로필 적용 완료
- 종료 자세 이동 완료
- 오류

이벤트는 오래된 항목을 임의로 버리지 않는다.

## 2.5 ACK와 Handshake

**ACK(Acknowledgement)** 는 상대가 명령을 실제로 처리했음을 알리는 응답이다. 단순히 Queue에 명령을 넣었다고 작업이 성공한 것은 아니다.

예를 들어 측정 시작은 다음 Handshake를 쓴다.

```text
Main → START_MEASUREMENT → Pose Process
Main ← POSE_MEASUREMENT_STARTED(success=True) ← Pose Process
Main → 프레임 공급 재개
```

Pose Process가 baseline, scaler, TFLite 모델을 실제로 로드한 뒤 성공 ACK를 보낸다. 이 확인 전에는 UI에 측정 시작 완료를 표시하지 않는다.

## 2.6 Shared Memory Ring과 Semaphore

**Ring Buffer** 는 정해진 슬롯을 원형으로 재사용하는 버퍼다. 이 프로젝트는 기본 4개 프레임 슬롯을 쓴다.

**Semaphore** 는 사용 가능한 슬롯 수나 읽을 수 있는 프레임 수를 안전하게 세는 동기화 도구다.

버퍼가 가득 차면 카메라를 멈춰 기다리지 않고 현재 프레임을 버린다. 이것은 영상 분석에서 처리량보다 지연시간을 우선한 선택이다. 모든 프레임을 늦게 처리하는 것보다 최신 프레임을 즉시 처리하는 편이 모터 제어에 유리하다.

## 2.7 Lazy Loading

**Lazy Loading** 은 필요한 시점까지 무거운 자원 로딩을 미루는 방식이다. GRU 모델과 scaler는 프로세스 시작이나 보정 때 로드하지 않고, 실제 측정 시작 시 로드한다.

장점은 다음과 같다.

- 초기 UI와 보정 화면이 빨리 준비된다.
- 보정만 하고 종료할 때 불필요한 모델 메모리를 쓰지 않는다.
- 모델 로딩 실패를 측정 시작 ACK로 명확히 처리할 수 있다.

## 2.8 EMA, PID, Deadband, Clamp

**EMA(Exponential Moving Average)** 는 최근 측정값에 더 큰 가중치를 주는 저역통과 필터다.

```text
filtered = alpha × new + (1 - alpha) × previous
```

ToF의 `filter_alpha`는 현재 0.25다. 센서 튐을 줄이지만 alpha가 너무 작으면 반응이 늦어진다.

**PID** 는 오차를 이용해 제어 출력을 계산하는 방식이다. 이 프로젝트의 모터 3·4는 현재 주로 P 동작을 사용하며, 출력은 “목표 각도”가 아니라 “목표 각속도” 성격으로 적분된다.

**Deadband** 는 오차가 아주 작을 때 움직이지 않는 구간이다. 모터 1·2의 거리 deadband는 현재 0.008 m, 즉 8 mm다. 센서 노이즈마다 모터가 흔들리는 것을 막는다.

**Clamp** 는 값이 허용 범위를 넘지 않도록 최솟값과 최댓값 사이로 제한하는 것이다.

## 2.9 FK와 IK

**정기구학(FK, Forward Kinematics)** 은 관절각으로 모니터 위치를 계산한다.

```text
(shoulder angle, elbow angle) → (monitor x, monitor z)
```

**역기구학(IK, Inverse Kinematics)** 은 원하는 모니터 위치로 관절각을 계산한다.

```text
(monitor x, monitor z) → (shoulder angle, elbow angle)
```

사용자가 앞뒤로 움직이면 원하는 모니터 X가 바뀌고, IK가 모터 1·2의 목표 각도를 만든다.

## 2.10 Latch, Timeout, Stable Samples

**Latch** 는 어떤 상태가 한 번 켜지면 명시적인 해제 조건까지 유지하는 방식이다. 예를 들어 사용자가 5초 이상 사라지면 “복귀 동작 요청”이 latch된다.

**Timeout** 은 동작이 끝없이 대기하지 않게 만드는 시간 제한이다.

**Stable Samples** 는 목표 오차 이내인 상태가 여러 번 연속 확인되어야 완료로 인정하는 방식이다. 한 번 우연히 허용 오차 안으로 들어왔다고 도착으로 판정하는 것을 막는다.

## 2.11 Atomic Write

프로필 JSON은 임시 파일에 먼저 완성한 뒤 최종 파일로 교체하는 방식으로 저장한다. 이를 **원자적 쓰기(Atomic Write)** 라고 한다. 저장 도중 전원이 꺼져 반쪽짜리 JSON이 남을 가능성을 줄인다.

<div style="page-break-after: always;"></div>

# 3. 프로젝트 디렉터리와 파일 역할

## 3.1 메인 실행에 직접 관련된 구조

```text
POCO/
└─ WorkSpace/
   ├─ pyQt/
   │  ├─ mainpyQt.py                         # 통합 실행 진입점, 메인 UI
   │  ├─ camera_worker_profile_all.py        # 카메라 QThread와 실행 모드 전환
   │  ├─ result_worker.py                    # IPC 결과를 Qt Signal로 변환
   │  ├─ monitor_arm_preparation_dialog.py   # 측정 준비·수동조작 화면
   │  ├─ user_profile_dialog.py              # 4슬롯 프로필 선택·저장 화면
   │  ├─ managers/
   │  │  └─ vision_process_manager_profile.py
   │  ├─ processes/
   │  │  ├─ pose_process_profile.py
   │  │  ├─ face_process_profile.py
   │  │  └─ hardware_process.py
   │  ├─ ipc/
   │  │  ├─ queue_utils.py
   │  │  └─ shared_frame_ring.py
   │  ├─ services/
   │  │  ├─ calibration_service.py
   │  │  ├─ pose_gru_service.py
   │  │  ├─ face_gru_service.py
   │  │  ├─ tof_service.py
   │  │  ├─ imu_service.py
   │  │  ├─ motor_service.py
   │  │  ├─ motor12_controller.py
   │  │  ├─ motor34_controller.py
   │  │  ├─ monitor_arm_kinematics.py
   │  │  ├─ monitor_arm_planner.py
   │  │  ├─ monitor_arm_safety_supervisor.py
   │  │  ├─ monitor_arm_preparation_controller.py
   │  │  ├─ monitor_arm_calibration_service.py
   │  │  ├─ monitor_arm_user_x.py
   │  │  ├─ posture_alert_service.py
   │  │  ├─ buzzer_service.py
   │  │  ├─ user_profile_service.py
   │  │  ├─ hardware_config_service.py
   │  │  └─ hardware_state_store.py
   │  └─ ui/
   │     └─ pocoApplication_Qss.ui           # Qt Designer 메인 UI
   ├─ modules/
   │  ├─ config.py                           # AI/모델 공통 설정
   │  ├─ features.py                         # Pose/Face 특징값 계산
   │  └─ logger.py                           # 세션 기록
   ├─ config/
   │  └─ monitor_arm_settings.json           # 기구·센서·제어·안전 설정
   ├─ hardware/
   │  └─ motor_control/                      # 실제 STServo 저수준 통신
   ├─ tasks/                                 # MediaPipe task 모델
   ├─ saved_model/                           # TFLite, scaler, baseline
   ├─ data/
   │  ├─ settings/                           # 알림 설정
   │  ├─ user_profiles/                      # 최대 4개 사용자 프로필
   │  └─ session_log/                        # 측정 세션 로그
   └─ streamlit/
      └─ app.py                              # 리포트 UI
```

## 3.2 계층별 책임

### UI 계층

`mainpyQt.py`, 두 Dialog, `.ui` 파일이 해당한다. 사용자의 의도를 명령으로 바꾸고 상태를 화면에 표시한다. 센서나 모터를 직접 열지 않는다.

### Orchestration 계층

`CameraWorker`, `VisionProcessManager`, `VisionResultWorker`가 해당한다. 프로세스를 생성하고, 프레임과 명령을 올바른 통로로 보낸다.

### Process 계층

Pose, Face, Hardware Process의 메인 루프다. 각 프로세스는 자신의 자원과 상태 머신을 소유한다.

### Service 계층

계산 또는 장치 책임을 작은 클래스로 분리한다. 예를 들어 Hardware Process가 모든 IK 식을 직접 갖지 않고 `TwoJointMonitorArm`, `MonitorArmPlanner`, `Motor12Controller`에 위임한다.

### 데이터·설정 계층

JSON, pickle/joblib, TFLite, CSV 등이다. 프로그램을 껐다 켜도 유지되어야 하는 값과 실행 중에만 필요한 상태가 구분된다.

## 3.3 비슷해 보이지만 다른 설정 파일

| 파일 | 저장 내용 | 저장 주체 |
|---|---|---|
| `modules/config.py` | 모델 경로, 프레임 크기, 클래스 라벨, GRU 창 | 소스 코드 |
| `config/monitor_arm_settings.json` | 링크 길이, ToF, 속도, 안전 제한 | HardwareConfig/보정 로직 |
| `data/settings/alarm_settings.json` | 알림 켜기, 지속시간, 반복·강한 알림 | SettingsManager |
| `data/user_profiles/profiles.json` | 4개 슬롯의 이름과 메타데이터 | UserProfileService |
| `saved_model/baseline.pkl` | 현재 활성 Pose 기준 벡터 | Calibration/Profile activation |
| `saved_model/baseline_face.pkl` | 현재 활성 Face 기준 벡터 | Face 활성 모드에서만 필요 |

<div style="page-break-after: always;"></div>

# 4. 런타임 구성: 프로세스, 스레드, 공유 메모리

## 4.1 현재 `POSE_ONLY`의 실행 단위

메인 화면을 처음 띄운 직후에는 아직 Worker가 없다. 사용자가 보정, 프로필, 수동조작, 측정 중 하나를 눌러 `ensure_camera_worker()`가 호출되면 다음 실행 단위가 만들어진다.

```text
Process 1: Main Process
  ├─ Thread 1: Qt GUI event loop
  ├─ Thread 2: CameraWorker
  └─ Thread 3: VisionResultWorker

Process 2: HardwareProcess
Process 3: PoseProcess

FACE_ONLY이면 Pose 대신 Face가 생성된다.
BOTH이면 Hardware + Pose + Face, 총 4개 프로세스가 된다.
```

## 4.2 `spawn` 시작 방식

Manager는 다음처럼 context를 만든다.

```python
self.ctx = multiprocessing.get_context("spawn")
```

`spawn`은 새 파이썬 인터프리터를 띄우고 필요한 객체를 직렬화해 전달한다. `fork`보다 초기 비용은 있지만, 카메라·Qt·네이티브 라이브러리 상태를 그대로 복제해 생기는 충돌을 줄이는 데 유리하다.

이 때문에 프로세스 target은 파일 최상위 함수여야 하고, 전달 인수는 pickle 가능해야 한다. `pose_process_entry`, `face_process_entry`, `hardware_process_entry`가 얇은 진입 함수로 존재하는 이유다.

## 4.3 IPC 채널 지도

```text
Main → Pose
  pose_command_queue           START_CALIBRATION, START_MEASUREMENT, STOP
  pose shared frame ring       카메라 영상

Pose → Main
  pose_result_queue            READY, DONE, ERROR, GRU 결과, 통계
  pose_state_to_main_queue     최신 랜드마크·특징·모드

Main ↔ Hardware
  main_to_hw_state_queue       PREVIEW, MEASURING 같은 최신 main mode
  main_to_hw_event_queue       프로필 적용, 모터 이동, 설정 변경 명령
  hw_to_main_state_queue       전체 최신 하드웨어 상태
  hw_to_main_event_queue       READY, 이동 완료, 설정 ACK, 오류

Pose ↔ Hardware
  pose_to_hw_state_queue       최신 랜드마크 품질·추론 결과
  pose_to_hw_event_queue       Pose 완료/오류 이벤트
  hw_to_pose_state_queue       최신 하드웨어 상태
  hw_to_pose_event_queue       하드웨어 이벤트
```

비활성인 Face도 Queue 인터페이스 자체는 생성하지만, Face Process와 Face Shared Memory는 만들지 않는다. 이 방식은 모드에 따른 코드 분기를 줄이고 인터페이스 형태를 일정하게 유지한다.

## 4.4 큐 크기

| 종류 | 기본 크기 | 이유 |
|---|---:|---|
| State Queue | 1 | 최신값만 필요 |
| Event Queue | 32 | 짧은 이벤트 폭주 흡수 |
| Vision Command Queue | 16 | 순서 있는 제어 명령 |
| Vision Result Queue | 64 | 결과·통계·ACK 여유 |
| Frame Ring | 4 slots | 낮은 지연과 짧은 부하 변동 절충 |

## 4.5 `put_latest()`와 `put_ordered()`

`ipc/queue_utils.py`의 정책은 코드 전체를 이해하는 핵심이다.

### `put_latest(queue, item)`

1. 큐에 새 값을 넣어 본다.
2. 큐가 가득 찼다면 기존 값을 꺼낸다.
3. 새 값을 넣는다.

결과적으로 센서나 랜드마크 State는 과거 데이터가 쌓이지 않는다.

### `put_ordered(queue, item)`

기존 항목을 버리지 않고 timeout 범위에서 순서대로 넣는다. 실패 여부를 `True/False`로 반환해 호출자가 명령 전달 실패를 처리할 수 있다.

## 4.6 프레임 Ring의 지연 우선 정책

CameraWorker는 Pose와 Face 각각의 Writer에 같은 프레임을 쓴다. Reader의 `read_latest()`는 읽을 프레임이 여러 장 밀렸다면 오래된 슬롯을 반환하고 가장 최신 프레임을 처리한다.

```text
카메라 생성: F100 F101 F102 F103
AI가 늦음:         처리 중...
다시 읽을 때: F101, F102를 건너뛰고 F103 처리
```

이때 `overrun_count`와 read/write count가 기록되므로 프로파일 로그에서 병목을 판단할 수 있다.

<div style="page-break-after: always;"></div>

# 5. 프로그램 시작부터 화면 표시까지

## 5.1 파이썬 진입점

`mainpyQt.py` 맨 아래의 실행 순서는 간단하다.

```text
faulthandler 활성화
  → QApplication 생성
  → MainWindow 생성
  → Linux이면 전체화면, 아니면 일반 창
  → app.exec_()로 Qt 이벤트 루프 시작
  → 이벤트 루프 종료 후 exit code 반환
```

`faulthandler`는 Python 예외가 아니라 MediaPipe, OpenCV, Qt 같은 네이티브 C/C++ 계층에서 segmentation fault가 날 때 Python stack 단서를 stderr에 남기기 위한 장치다.

## 5.2 `MainWindow.__init__()` 순서

1. `ui/pocoApplication_Qss.ui`를 `uic.loadUi()`로 로드한다.
2. Worker, 종료 플래그, Streamlit Process 필드를 초기화한다.
3. 알림 설정 Manager를 만든다.
4. 하드웨어 제어 설정과 런타임 상태 저장소를 만든다.
5. Pose·Face·Hardware 최신 상태 필드를 `None`으로 둔다.
6. 사용자 프로필 서비스와 현재 슬롯 상태를 만든다.
7. `.ui`에 있는 버튼 Signal을 Slot에 연결한다.
8. 코드로 추가하는 `프로필`, `수동조작` 버튼을 header layout에 붙인다.
9. 카메라 라벨, 버튼 상태, 실시간 표시, 설정 UI를 초기화한다.

## 5.3 시작 직후 Worker를 만들지 않는 이유

`MainWindow.__init__()`에서는 `CameraWorker`를 만들지 않는다. 따라서 프로그램을 켰다는 이유만으로 카메라, MediaPipe, I2C, 모터 serial이 열리지 않는다.

이것은 다음 이점이 있다.

- 설정만 바꾸려는 경우 카메라와 모터가 불필요하게 켜지지 않는다.
- 프로필 선택 전 이전 baseline이 자동으로 적용되는 것을 막는다.
- 장치 초기화 실패를 사용자가 실제 기능을 요청한 시점에 분명히 표시할 수 있다.

## 5.4 초기 버튼 정책

`initialize_button_state()`는 디스크에 baseline이 있더라도 곧바로 측정 버튼을 열지 않는다. 현재 실행에서 다음 중 하나가 필요하다.

- 새 보정을 완료한다.
- 저장된 프로필을 사용자가 명시적으로 선택한다.

이것은 여러 사용자가 장치를 쓸 때 이전 사용자의 기준값을 실수로 사용하는 것을 막는 정책이다.

# 6. `MainWindow` 상세 해설

## 6.1 `MainWindow`의 책임과 비책임

`MainWindow`의 docstring에 적힌 대로 이 클래스는 UI를 담당한다.

**담당하는 것**

- 버튼 상태와 메시지 박스
- Worker 생성·재사용 요청
- 하드웨어 명령 전송
- 실시간 상태 표시
- 프로필 선택·저장 흐름
- Streamlit 실행
- 앱 종료 시 전체 정리 요청

**담당하지 않는 것**

- 카메라 `read()` 직접 호출
- MediaPipe 실행
- 특징값 계산
- IK 계산
- I2C 또는 serial 직접 접근
- 모터 안전 판정

## 6.2 초기화·경로 함수

### `initialize_camera_label()`

카메라 영역을 검은 배경과 `Camera Off` 텍스트로 만든다.

### `initialize_button_state()`

초기값 준비 버튼만 활성화하고, 현재 세션에서 사용자 선택이 끝나기 전에는 측정 시작을 잠근다. 프로필이 하나라도 있으면 프로필 선택 안내를, 없으면 초기값 설정 안내를 보여준다.

### `initialize_realtime_labels()`

TOP 3, 총 측정시간, 현재 자세, 센서 모니터를 `-` 또는 `--`로 리셋한다. 측정을 새로 시작할 때도 재사용한다.

### `has_baseline()`

`PROFILE_MODE`에 따라 필요한 baseline 파일만 검사한다.

- `POSE_ONLY`: `baseline.pkl`
- `FACE_ONLY`: `baseline_face.pkl`
- `BOTH`: 둘 다

파일 존재만 보는 1차 검사이며, 배열 길이·NaN·모델 호환성은 각 GRU Service가 측정 시작 때 다시 검사한다.

### `resolve_workspace_path()`

절대경로는 그대로 쓰고, 상대경로는 `WorkSpace` 기준 절대경로로 바꾼다. 실행 당시 현재 디렉터리에 따라 모델 파일을 못 찾는 문제를 줄인다.

## 6.3 Worker 생명주기

### `ensure_camera_worker()`

세 경우를 나눈다.

1. Worker가 현재 실행 중이면 아무것도 하지 않는다.
2. Cam Off로 QThread만 멈췄고 Process 자원은 살아 있으면 기존 Worker를 다시 `start()`한다.
3. 전체 shutdown된 Worker이거나 처음이면 새 `CameraWorker`를 만들고 모든 Signal을 연결한다.

CameraWorker의 Signal은 프레임, 상태 문자열, 보정 완료, 측정 시작, 결과, Pose State, Face State, Hardware State, Hardware Event다.

### `stop_camera_worker(full_shutdown=False)`

Cam Off와 앱 종료를 구분한다.

- 일반 Cam Off: 카메라 QThread만 멈추고 Pose/Hardware Process와 Queue/Shared Memory는 IDLE 상태로 유지한다.
- 앱 종료: Worker Signal을 막고 카메라와 모든 자원을 완전히 종료한다.

Cam Off마다 multiprocessing Queue와 Shared Memory를 파괴했다 다시 만들지 않는 이유는 Qt 객체 수명과 native cleanup이 겹칠 때 발생할 수 있는 충돌을 줄이기 위해서다.

### `_camera_shutdown_in_progress`, `_app_closing`

종료 중 이미 큐에 들어와 있던 Signal이 늦게 GUI Slot을 호출할 수 있다. 각 Slot은 이 두 플래그를 확인해 종료 중 위젯 갱신을 무시한다.

## 6.4 최신 상태 수신

### `on_pose_state_changed(state)`

Pose Process의 최신 상태를 `latest_pose_state`에 보관한다. 현재 MainWindow가 랜드마크를 직접 그리거나 계산하지는 않는다.

### `on_face_state_changed(state)`

Face State를 보관한다. `POSE_ONLY`에서는 Signal이 발생하지 않는다.

### `on_hardware_changed(state)`

하드웨어 최신 상태를 세 군데에 반영한다.

1. `latest_hardware_state`
2. 스레드 안전 복사본을 제공하는 `hardware_state_store`
3. UI의 `labelSensorMonitor`

측정 중 안전 사유가 바뀌면 상태 표시줄에도 ToF 경고, 랜드마크 경고 또는 안전 reason을 표시한다.

### `update_sensor_monitor(state)`

다음 값을 두 줄로 압축해 표시한다.

- ToF 필터 거리와 유효 여부
- Pose landmark quality와 minimum visibility
- 제어에 사용하는 사용자 X와 fusion mode
- IMU X/Y 오차와 보정 여부
- Safety state

세 센서 상태를 기준으로 초록, 노랑, 빨강 계열 배경을 선택한다. 여기서 화면의 `fusion mode`가 표시되더라도 현재 제어값은 ToF only이며, legacy vision field는 진단 호환성을 위해 구조에 남아 있다.

## 6.5 Hardware Event 처리

`on_hardware_event_changed()`는 유실되면 안 되는 ACK를 처리한다.

주요 처리 내용은 다음과 같다.

- 준비 Dialog가 열려 있으면 이벤트를 Dialog에도 전달
- 설정 업데이트 이벤트면 Main의 config snapshot 갱신
- `USER_PROFILE_APPLY_STARTED`: 이동 중 상태 표시
- `USER_PROFILE_APPLIED`: 성공 시 프로필 슬롯 활성화, 측정 버튼 개방
- `MEASUREMENT_STOP_AND_REST_ACK`: 종료 자세 완료 후 카메라 종료 예약

프로필 적용 중에는 보정, 프로필, 수동조작 버튼을 잠근다. 모터 이동이 완료되기 전에 다른 명령이 섞이는 것을 막기 위함이다.

## 6.6 하드웨어 Config API

### `get_hardware_config()`

현재 snapshot의 깊은 복사본을 반환한다. 호출자가 중첩 dict를 수정해 원본을 우연히 바꾸지 않게 `copy.deepcopy()`를 쓴다.

### `save_hardware_control_config(control_patch)`

- Hardware Process가 살아 있으면 `UPDATE_CONFIG` 이벤트를 보내 Hardware가 JSON write owner가 된다.
- 아직 Worker가 없으면 Main이 직접 저장한다.

한 파일을 여러 프로세스가 동시에 쓰지 않도록 소유권을 조정한 설계다.

### `request_hardware_config()`, `reload_hardware_config()`

Process가 있으면 이벤트로 요청하고, 없으면 Main에서 디스크를 읽는다.

## 6.7 프로필 선택 흐름

`on_user_profile_clicked()`의 전체 과정은 다음과 같다.

```text
UserProfileDialog 표시
  → 사용자가 슬롯을 한 번 선택
  → 확인 버튼을 한 번 더 누름
  → activate_profile(): 해당 baseline을 전역 활성 경로로 복사
  → ensure_camera_worker()
  → APPLY_USER_PROFILE 이벤트를 Hardware에 전달
  → 모터 1~4 연결/각도 복원/작업 초기위치 이동
  → USER_PROFILE_APPLIED 성공 ACK
  → 측정 시작 버튼 활성화
```

프로필 선택만으로 UI가 성공 처리하지 않는 것이 중요하다. Hardware Process가 저장된 IMU 기준과 모터 상태를 복원하고 실제 도착을 확인한 뒤에만 프로필 적용 완료가 된다.

## 6.8 수동조작 흐름

`on_manual_arm_clicked()`는 Worker를 준비하고, Pose 준비 모드와 Hardware 준비 상태를 시작한 다음 `MonitorArmPreparationDialog(manual_only=True)`를 띄운다.

manual-only에서는 ToF/눈 간격 보정 영역과 준비 완료 버튼을 숨기고 다음 기능만 제공한다.

- 모터 1~4 연결 확인
- 휴식 → 작업 위치
- 작업 → 휴식 위치
- 모터 1·2 사용자 X 기반 수동 IK
- 모터 3·4 조그

Dialog가 끝나면 `on_manual_arm_finished()`가 Worker를 Preview로 돌린다.

## 6.9 새 사용자 보정 흐름

### `on_calibration_clicked()`

“초기값 준비” 단계다. 아직 Pose baseline을 수집하지 않는다.

1. Worker와 카메라/Pose 준비 모드를 시작한다.
2. Hardware에 `START_MONITOR_ARM_PREPARATION`을 보낸다.
3. 일반 준비 Dialog를 연다.
4. 보정 시작과 측정 시작 버튼을 잠근다.

### `on_monitor_arm_preparation_finished(success, message)`

준비 화면에서 모터 조정과 5초 센서 평균 저장이 끝난 결과를 받는다. 성공해야 `초기값 측정시작` 버튼이 열린다.

### `on_calibration_start_clicked()`

다음 두 조건을 다시 검사한다.

- 준비 Dialog가 정상 완료되었는가?
- Hardware State의 monitor-arm calibration에 `session_ready=True`가 있는가?

통과하면 CameraWorker의 Hardware IMU 보정 → Vision baseline 보정 Handshake를 시작한다.

## 6.10 측정 시작과 결과 표시

### `on_camera_on_clicked()`

현재 모드에 필요한 baseline이 있는지 확인한 뒤 Worker에 `start_measurement()`를 요청한다. 이 함수가 반환했다고 측정 성공은 아니다. 실제 성공 여부는 Process ACK 후 `on_measurement_started()`로 들어온다.

### `on_measurement_started(success, message)`

- 성공: 보정·프로필·수동조작을 잠그고 측정 화면을 초기화한다.
- 실패: 경고창을 띄우고 가능한 버튼을 다시 연다.

### `on_result_changed(result)`

통합 결과 dict에서 다음을 꺼내 화면에 표시한다.

- `posture_type`
- `confidence`
- `elapsed_sec`
- `rank_text`

피로도 필드는 구조적으로 들어올 수 있지만 현재 UI 코드는 주석 처리되어 있다.

## 6.11 측정 종료

`on_camera_off_clicked()`는 바로 카메라만 끄지 않는다.

```text
Vision 측정 STOP
  → MEASUREMENT_STOP_AND_REST
  → 모터 1·2 휴식자세 + 모터 3·4 중립각 이동
  → 실제 각도 도착 확인
  → ACK 또는 15초 UI fallback timer
  → _finish_camera_off()
  → Camera QThread 정지
```

ACK 실패면 경고를 띄우지만 UI가 무한히 묶이지 않도록 fallback timer가 있다.

## 6.12 보정 완료와 프로필 저장

`on_calibration_finished()`는 성공 시 사용자에게 프로필 저장 여부를 묻는다. 사용자가 저장을 선택하면 `save_current_profile()`이 슬롯 선택 Dialog를 열고 다음을 묶어 저장한다.

- 활성 모드의 Pose/Face baseline
- 모니터암 거리 보정값
- IMU reference
- 모터 각도와 관련 메타데이터

슬롯은 최대 4개이고 기존 슬롯이면 덮어쓰기 확인을 거친다.

## 6.13 설정과 리포트

`on_save_settings_clicked()`는 UI 값을 JSON에 저장하고, Hardware Process가 실행 중이면 같은 값을 `UPDATE_ALARM_SETTINGS`로 즉시 전달한다. 이 함수에서 `ensure_camera_worker()`를 호출하지 않는다는 주석은 “설정 저장만 했는데 카메라가 켜지는” 부작용을 막기 위한 것이다.

`on_report_clicked()`는 `streamlit/app.py`를 별도 subprocess로 실행한다. Linux에서는 Chromium kiosk 모드, Windows에서는 Chrome/Edge app 모드를 우선한다.

## 6.14 종료와 화면 전환

`closeEvent()`는 Dialog를 닫고 `_app_closing=True`를 설정한 뒤 전체 shutdown을 수행한다. Streamlit server가 살아 있으면 terminate한다.

`keyPressEvent()`는 Esc 키로 전체화면과 일반 창을 전환한다. Raspberry Pi 7인치 화면에서는 기본 전체화면이다.

<div style="page-break-after: always;"></div>

# 7. `CameraWorker` 상세 해설

## 7.1 역할

`CameraWorker`는 `QThread`를 상속한다. 하나의 클래스 안에서 다음 연결부를 담당한다.

- 카메라 열기와 프레임 획득
- 화면 표시용 QImage 생성
- Shared Frame Ring에 프레임 공급
- VisionProcessManager 생성·시작
- VisionResultWorker 생성·Signal 중계
- Preview, Preparing, Calibrating, Measuring 모드 전환
- Hardware 보정 ACK와 Vision 보정 시작 연결

AI 추론은 하지 않는다. 이 차이를 놓치면 카메라 프레임 속도 문제와 AI 속도 문제를 같은 곳에서 찾게 된다.

## 7.2 카메라 추상화

### `OpenCVCameraSource`

일반 USB 카메라나 개발 PC를 위한 구현이다. `cv2.VideoCapture`를 열고 width, height, fps를 설정한다.

### `PiCamera2Source`

Raspberry Pi 카메라를 위한 구현이다. `picamera2`를 사용해 320×240 RGB 계열 프레임을 구성한다.

두 클래스 모두 `open()`, `start()`, `read()`, `release()` 형태를 맞춰 CameraWorker가 장치 종류를 덜 의식하게 한다. 이런 구조를 **공통 인터페이스** 또는 간단한 **Strategy 패턴**으로 볼 수 있다.

### `create_camera_source()`

운영체제와 라이브러리 사용 가능 여부에 따라 적절한 Source를 선택하고, 실패 시 다른 방식으로 fallback한다. 장치 선택 문제는 이 함수부터 추적하면 된다.

## 7.3 `RunMode`

```python
PREVIEW
PREPARING
CALIBRATING
MEASURING
```

- `PREVIEW`: 카메라 화면만 보여주고 추론은 정지
- `PREPARING`: 준비 화면에 필요한 Pose 눈 간격을 계산
- `CALIBRATING`: Pose/Face 특징 baseline 수집
- `MEASURING`: GRU 추론과 모터 제어가 활성화되는 실제 측정

Hardware Process는 별도로 `main_mode` State를 받으므로, CameraWorker 모드 변경 시 Manager의 `send_main_state()`를 함께 호출한다.

## 7.4 생성자 `__init__()`

생성자는 프레임 크기를 기준으로 `VisionProcessManager`를 만들고 Process들을 시작한다. 이어 `VisionResultWorker`를 만들고 결과 Signal을 CameraWorker의 Signal 또는 내부 callback에 연결한다.

중요한 점은 CameraWorker QThread의 `run()`이 시작되기 전에도 하위 Process 초기화가 진행될 수 있다는 것이다. 사용자가 버튼을 빨리 눌렀을 때를 위해 `pending_*` 플래그가 존재한다.

## 7.5 `run()` 카메라 루프

개념적인 순서는 다음과 같다.

```python
camera = create_camera_source()
camera.open()
camera.start()

while running:
    frame = camera.read()
    if frame is invalid:
        continue_or_report()

    if manager.accept_frames:
        manager.write_frame(frame, frame_id, timestamp_ns)

    if GUI 표시 주기가 되었으면:
        frame_changed.emit(convert_frame_to_qimage(frame))

release_resources()
```

카메라는 목표 30 FPS지만 GUI 갱신은 약 15 FPS로 제한한다. 사람이 보는 화면은 15 FPS로 충분하고, QImage 변환과 Qt paint 부담을 절반가량 줄일 수 있기 때문이다. AI에는 가능한 카메라 프레임을 공급한다.

각 프레임에는 `frame_id`와 `timestamp_ns`가 붙는다.

- `frame_id`: 프레임 누락 수 계산
- `timestamp_ns`: Queue 대기시간과 end-to-end latency 계산

## 7.6 Pending command

Worker, Process, MediaPipe 준비가 끝나기 전에 사용자가 보정이나 측정을 누를 수 있다. 이때 요청을 즉시 실패시키지 않고 다음 플래그로 보관한다.

- `pending_preview_start`
- `pending_preparation_start`
- `pending_calibration_start`
- `pending_measurement_start`

`_on_vision_ready()`가 호출되면 `apply_pending_command()`가 보관된 의도를 실행한다. 이것은 비동기 초기화에서 흔히 쓰는 지연 실행 패턴이다.

## 7.7 준비와 보정 Handshake

### `start_monitor_arm_preparation()`

Pose Process를 준비 모드로 전환해 GRU 없이 MediaPipe 눈 랜드마크와 eye gap을 계속 만든다. Hardware 준비 화면이 ToF와 eye gap 실시간 값을 함께 표시할 수 있게 한다.

### `finish_monitor_arm_preparation()`

준비 명령을 끝내고 Preview로 돌아간다.

### `start_calibration()`

바로 Vision baseline을 수집하지 않는다.

```text
Vision analysis STOP
  → Main state CALIBRATION_PRECHECK
  → Hardware PREPARE_CALIBRATION
  → IMU offset 측정
  → HARDWARE_CALIBRATION_READY
  → _begin_vision_calibration()
  → Pose/Face START_CALIBRATION
```

이 순서로 IMU 기준이 준비되지 않았는데 Pose baseline만 생기는 불완전한 세션을 막는다.

### `_begin_vision_calibration()`

ResultWorker의 과거 보정 결과를 리셋하고 활성 Process들에 보정 시작 명령을 보낸다. `POSE_ONLY`이면 Pose 하나, `BOTH`이면 Pose와 Face 양쪽 완료를 기다린다.

## 7.8 측정 시작 검증

`start_measurement()`는 다음을 순서대로 검사한다.

1. CameraWorker가 실행 중인가?
2. 활성 Vision Process가 READY인가?
3. IMU가 available + calibrated이고 보정 중이 아닌가?
4. 모터가 available + enabled + ready인가?
5. Pose 사용 모드에서 현재 세션의 ToF/눈 간격 평균값이 있는가?
6. 활성 모드에 필요한 baseline 파일이 있는가?

통과하면 Process에 `START_MEASUREMENT`를 보내고 ACK를 기다린다.

### `_on_measurement_start_finished()`

활성 Process가 모델, scaler, baseline을 모두 준비했다는 ACK가 모이면 다음을 수행한다.

1. StudyLogger 측정 세션 시작
2. Shared Frame 공급 재개
3. RunMode를 MEASURING으로 변경
4. Hardware에 Main State `MEASURING` 전달
5. MainWindow에 성공 Signal 전송

모델 로딩 중 Frame Ring 공급을 잠시 멈추는 것은 4개 슬롯이 시작 전에 가득 차 overrun 로그가 쌓이는 것을 막는다.

## 7.9 Camera Off와 앱 종료의 차이

### `stop_camera_only()`

- pending 요청 초기화
- 보정 precheck 취소
- Vision analysis STOP
- Main State `CAMERA_OFF`
- 측정 logger 종료
- 카메라 `run()` 루프 종료
- Process/Queue/Shared Memory 유지

### `shutdown()`

`stop_camera_only()` 후 `shutdown_vision_resources()`를 호출한다.

### `shutdown_vision_resources()`

종료 순서가 중요하다.

1. ResultWorker Signal 차단
2. 생산자인 Pose/Face/Hardware Process 정지
3. 소비자인 ResultWorker 정지
4. Queue feeder thread와 file descriptor 정리
5. Shared Memory close/unlink

생산자가 살아 있는데 큐를 먼저 닫으면 종료 중 쓰기 오류나 native 충돌이 날 수 있다.

# 8. `VisionProcessManager`와 IPC 설계

## 8.1 Manager의 역할

Manager는 계산을 하지 않고 실행 인프라를 소유한다.

- `PROFILE_MODE` 검증
- Queue 생성
- Shared Memory Ring 생성
- Process 생성·시작·정지
- Vision 명령 broadcast
- Main/Hardware 이벤트 전송
- 프레임 공급 on/off
- Queue와 Shared Memory 최종 정리

## 8.2 실행 모드

| 모드 | Pose | Face | Hardware |
|---|---:|---:|---:|
| `POSE_ONLY` | O | X | O |
| `FACE_ONLY` | X | O | O |
| `BOTH` | O | O | O |

하드웨어는 어떤 비전 모드에서도 생성된다. `enable_pose`, `enable_face`를 받아 필요한 센서 조건과 상태를 판단한다.

## 8.3 Process entry 함수

`pose_process_entry()` 같은 함수는 실제 구현을 함수 안에서 import한다. `spawn` 자식이 불필요한 네이티브 모듈을 부모 import 시점에 모두 로드하지 않게 하고, Process target을 pickle 가능한 최상위 함수로 유지한다.

## 8.4 lifecycle 함수

### `start()`

Hardware Process를 먼저 시작하고, 활성 모드의 Pose/Face Process를 시작한다.

### `start_monitor_arm_preparation()`

Pose가 활성일 때 준비용 명령을 보내고 프레임 공급을 허용한다.

### `finish_monitor_arm_preparation()`

Vision을 정지하고 준비 상태를 끝낸다.

### `start_calibration()`

활성 Process에 `START_CALIBRATION`을 순서 있는 Command Queue로 전달하고 프레임을 공급한다.

### `start_measurement()`

먼저 프레임 공급을 막고 활성 Process에 측정 시작 명령을 보낸다. 모델 로딩 성공 ACK 후 CameraWorker가 `resume_measurement_frames()`를 호출한다.

### `stop_analysis()`

Pose/Face에 `STOP`을 보내고 `accept_frames=False`로 바꾼다. Hardware Process 자체는 종료하지 않고 Main State에 따라 IDLE 또는 Preview 동작을 한다.

### `send_main_state(state)`

State Queue 정책으로 Hardware에 최신 Main mode를 보낸다.

### `send_hardware_command(message)`

Event Queue 정책으로 모터·보정·설정 명령을 보낸다. 반환값으로 전달 성공 여부를 확인할 수 있다.

### `write_frame()`

Pose와 Face 중 활성 Writer에 프레임을 각각 쓴다. 한 채널이 밀려도 다른 채널의 공유 메모리는 별도다.

## 8.5 종료 함수

### `stop()`

stop event를 켜고 각 Process에 SHUTDOWN을 보낸 뒤 join한다. 정상 시간 안에 끝나지 않는 Process는 제한적으로 terminate하는 방어 로직을 사용한다.

### `close_queues()`

모든 Queue에 `close()`와 `join_thread()`를 적용해 background feeder가 끝나도록 한다.

### `_close_frame_channel()`

Writer close, Shared Memory close, owner의 unlink 순으로 정리한다. `unlink`는 공유 메모리 이름을 시스템에서 제거하는 작업이므로 owner만 수행해야 한다.

<div style="page-break-after: always;"></div>

# 9. `VisionResultWorker` 상세 해설

## 9.1 왜 별도 QThread인가

`multiprocessing.Queue.get()`과 여러 Queue drain을 GUI 이벤트 루프에서 하면 UI가 끊길 수 있다. `VisionResultWorker`는 Queue consumer 전용 QThread이며, 받은 값을 PyQt Signal로 MainWindow에 넘긴다.

## 9.2 run loop

활성 모드에 따라 다음 Queue를 반복해서 비운다.

```text
Pose result → Pose latest state
Face result → Face latest state
Hardware latest state
Hardware ordered events
```

처리한 것이 하나도 없을 때만 5 ms sleep한다. 무조건 sleep하지 않아 결과 응답성을 유지하면서, 빈 루프 CPU 사용률을 줄인다.

## 9.3 READY 집계

`pose_ready`와 `face_ready`는 비활성 Process의 경우 처음부터 `True`다. 활성 Process가 `POSE_READY`, `FACE_READY`를 보내면 `_check_ready()`가 전체 준비 상태를 판단한다.

이 방식 덕분에 `POSE_ONLY`에서도 Face ACK를 영원히 기다리지 않는다.

## 9.4 보정 결과 집계

`reset_calibration()`은 이전 결과를 지운다. 각 Process의 progress를 받아 합친 상태 문자열을 내보내고, 활성 Process가 모두 DONE을 보냈을 때 `_check_calibration_done()`이 최종 성공을 결정한다.

`BOTH`에서 Pose 성공, Face 실패라면 전체 보정은 실패다. 불완전한 프로필을 정상 프로필처럼 저장하지 않기 위한 정책이다.

## 9.5 측정 시작 ACK 집계

`reset_measurement_start()` 후 활성 Process별 `*_MEASUREMENT_STARTED`를 기다린다. 모두 성공해야 `measurement_start_finished(True, ...)`를 발생시킨다.

## 9.6 결과 결합

### `POSE_ONLY`

Pose 결과를 중심으로 UI 결과를 만든다. 데이터 schema 호환을 위해 피로도는 `Normal`, 확률 0.0 같은 기본값을 넣을 수 있지만, 실제 Face 추론 결과가 아니다.

### `FACE_ONLY`

Face 결과를 중심으로 피로도를 갱신한다. 자세 정보는 기본값이 된다.

### `BOTH`

Pose와 Face GRU는 처리 속도와 STRIDE가 달라 정확히 같은 `frame_id`에 결과가 나오지 않을 수 있다. 따라서 가장 최근 결과 둘의 timestamp 차이가 최대 1초 이내면 결합한다.

## 9.7 측정 세션과 TOP 3

`start_measurement_session()`은 시작 시간을 기록하고 `StudyLogger`를 연다. 결과가 올 때 같은 Pose frame을 중복 집계하지 않고 자세 Counter를 증가시킨다.

`_build_rank_text()`는 `Optimal`을 제외한 불안정 자세 빈도를 정렬해 TOP 3 문자열을 만든다. `_get_elapsed_sec()`은 monotonic time 기준 경과시간을 반환한다.

UI Signal은 기본 0.5초 간격으로 제한한다. GRU 결과마다 모든 라벨을 다시 그려 Qt 부하가 늘어나는 것을 막는다.

# 10. Pose Process와 자세 판정

## 10.1 Pose Process 상태

```text
IDLE → PREPARING → IDLE
IDLE → CALIBRATING → WAITING
IDLE/WAITING → MEASURING
모든 활성 상태 → STOP → IDLE
SHUTDOWN → 프로세스 종료
```

- `IDLE`: 프레임을 읽지 않음
- `PREPARING`: 눈 간격과 주요 landmark quality 계산
- `CALIBRATING`: 특징 baseline 수집
- `WAITING`: 보정 완료 또는 측정 시작 실패 후 대기
- `MEASURING`: Pose GRU 추론

## 10.2 MediaPipe 초기화

Pose Landmarker는 320×240, `model_complexity=0` 성격의 경량 설정과 confidence 0.5를 사용한다. 프로세스 시작 시 dummy frame으로 두 번 warm-up한 뒤 `POSE_READY`를 보낸다.

**Warm-up**은 첫 실제 프레임에서 그래프 초기화나 메모리 할당 때문에 갑자기 오래 걸리는 현상을 앞당겨 처리하는 것이다.

## 10.3 `build_pose_features()`

MediaPipe landmark를 `modules.features.calculate_features()`로 보내 10개 자세 특징을 만든다. 크기가 `POSE_FEATURE_SIZE=10`과 다르거나 유효하지 않으면 `None`으로 처리한다.

현재 특징은 다음 의미를 갖는다.

| 번호 | 특징 | 주된 목적 |
|---:|---|---|
| F1 | 귀-어깨 수직 간격/어깨 폭 | 목이 숙여졌는지 |
| F2 | 손-얼굴 최소 거리/어깨 폭 | 턱 괴기 |
| F3 | 좌우 어깨 높이 차/어깨 폭 | 비대칭 |
| F4 | 귀 기준 머리 roll 각도 | 고개 기울기 |
| F5 | 코-어깨 높이/어깨 폭 | 머리 처짐 |
| F6 | 코와 어깨 중심 X 차이 | 좌우 중심 이탈 |
| F7 | 눈-귀 수평 관계 | 얼굴 yaw 보조 |
| F8 | 귀 폭/어깨 폭 | 원근 기반 전방 머리 |
| F9 | 손-눈 최소 거리 | 손 위치 구분 |
| F10 | 손 visibility flag | 손 특징 유효성 |

대부분 어깨 폭으로 정규화하는 이유는 사용자 체격과 카메라 거리 변화의 영향을 줄이기 위해서다.

## 10.4 랜드마크 직렬화와 제어 품질

`serialize_pose_landmarks()`는 MediaPipe 객체를 프로세스 Queue에 직접 넣지 않고 x, y, z, visibility의 일반 list로 바꾼다. 네이티브 객체 pickle 문제와 불필요한 의존성을 줄인다.

`pose_control_landmark_quality()`는 제어용 핵심 점을 별도로 검사한다.

- 왼쪽 눈 index 2
- 오른쪽 눈 index 5
- 왼쪽 어깨 index 11
- 오른쪽 어깨 index 12
- 최소 visibility 0.60

엉덩이 landmark는 책상에 가려지는 환경에서 false missing을 만들 수 있어 제어 존재 판정에서 제외했다. “모델이 landmark를 반환했다”와 “모터 제어에 충분한 품질이다”를 구분한 것이다.

## 10.5 `measure_pose_eye_gap_px()`

양 눈의 정규화 좌표 차이를 영상 width/height로 픽셀 거리로 변환한다. 두 눈 visibility가 0.5 이상이고 눈 간격이 최소 5 px 이상이어야 유효하다.

현재 실제 사용자 X 제어는 ToF only이지만, eye gap 평균은 준비 보정 데이터와 진단 정보로 유지된다.

## 10.6 Pose Process 한 프레임 처리

```text
SharedFrameReader.read_latest()
  → BGR을 RGB로 변환
  → MediaPipe Image 생성
  → PoseLandmarker 처리
  → 10개 feature 계산
  → landmark serialization
  → 제어 landmark quality + eye gap 계산
  → 모드별 Calibration 또는 GRU 처리
  → 최신 POSE_STATE를 Main과 Hardware에 각각 put_latest
  → 결과/진행/ACK를 ordered result queue로 전달
```

Pose State는 Calibration 중에도 landmark, quality, feature를 계속 포함한다. Hardware가 준비 화면과 안전 판정에 같은 최신 상태를 사용할 수 있기 때문이다.

## 10.7 `PoseGruService`

### baseline 로드

저장된 baseline을 float32 1차원 배열로 읽고 다음을 검사한다.

- 파일 존재
- 길이 10
- 모든 값이 finite, 즉 NaN/Inf가 아님

대기 중에는 baseline이 없으면 0 배열을 임시로 쓸 수 있지만, 측정 시작에서는 `required=True`로 실패시킨다.

### 모델 로드

`tflite_runtime.interpreter`를 우선하고 없으면 `tensorflow.lite`를 사용한다. Raspberry Pi에서 전체 TensorFlow보다 가벼운 runtime을 쓸 수 있게 한 fallback이다.

### Window와 STRIDE

```text
각 유효 frame feature - 사용자 baseline
  → 최대 30개 deque
  → 30개가 차면
  → 매 5번째 유효 frame마다 TFLite 추론
```

랜드마크 미검출 frame은 0 feature로 넣지 않는다. “사람이 없음”을 어떤 자세 데이터로 오인해 시퀀스를 오염시키는 것을 막는다.

### 출력

현재 자세 클래스는 다음 네 개다.

| index | label |
|---:|---|
| 0 | Optimal |
| 1 | Asymmetric |
| 2 | Forward Head |
| 3 | Chin Propping |

모델 출력이 scalar binary 형태인지 multi-class vector인지에 따라 `parse_output()`이 threshold 또는 argmax를 사용한다.

## 10.8 프로파일링

Pose Process는 약 2초마다 다음 stage의 평균, P95, 최대 시간을 출력한다.

- Shared ring read
- Queue latency
- BGR→RGB
- MediaPipe
- feature
- GRU
- 전체 latency

평균만 보면 간헐적인 끊김을 놓칠 수 있어 P95와 MAX를 함께 기록한다.

<div style="page-break-after: always;"></div>

# 11. Calibration과 사용자 프로필

## 11.1 보정은 하나가 아니라 세 묶음이다

새 사용자 보정은 실제로 다음 데이터를 만든다.

1. **모니터암 준비 보정**
   - ToF 사용자 X 5초 평균
   - MediaPipe 눈 간격 5초 평균
   - 사용자-모니터 기준 거리와 준비 위치
2. **IMU offset 보정**
   - ADXL345 X/Y reference
3. **Vision baseline 보정**
   - Pose 10차원 평균 벡터
   - Face 활성 모드라면 Face baseline

## 11.2 `CalibrationService`

Pose와 Face가 공통으로 쓰는 일반 특징 baseline 수집기다.

### 첫 유효 샘플부터 5초

사용자가 버튼을 누른 시각이 아니라 첫 정상 feature가 들어온 순간 `start_time`이 설정된다. 카메라 준비 시간 때문에 실제 수집 구간이 짧아지는 것을 막는다.

### 10초 wait timeout

첫 feature가 계속 없으면 기본 10초 후 실패한다. 무한 대기하지 않는다.

### 최소 샘플

기대값은 5초 × 30 FPS = 150개이며 최소 60%, 즉 90개가 필요하다. 유효하지 않은 frame을 제외하고 90개 미만이면 실패한다.

### 실패 시 기존 파일 보존

새 수집이 실패하면 기존 정상 baseline을 덮어쓰지 않는다. 사용자가 다시 시도할 수 있고, 이전 프로필이 손상되지 않는다.

### 성공값

수집 buffer의 axis 0 평균을 `joblib.dump()`로 저장한다. 개인별 평상시 자세의 편향을 모델 입력에서 빼기 위한 기준 벡터다.

## 11.3 `MonitorArmPreparationCalibrationService`

ToF State와 Pose eye gap을 각각 받아 5초 평균을 만든다. 둘의 sample count와 진행 상태를 snapshot으로 제공하므로 Dialog가 실시간으로 표시한다.

여기서 eye gap은 현재 모터 거리 제어에 직접 섞지 않고, 사용자별 보정 정보와 센서 진단에 보존한다.

## 11.4 `UserProfileService`

### 4슬롯 구조

프로필은 `slot_1`부터 `slot_4`까지 최대 네 개다. 각 슬롯에는 이름, 저장 시각, 사용 모드, baseline 파일, 모니터암/IMU/모터 메타데이터가 들어간다.

### `list_profiles()`

UI가 네 슬롯의 점유 여부와 이름을 표시할 수 있는 목록을 반환한다.

### `save_profile()`

현재 활성 baseline과 Hardware State에서 필요한 보정값을 슬롯 디렉터리에 복사하고 메타데이터를 원자적으로 저장한다.

### `load_profile()`

슬롯 metadata와 저장 파일을 읽고 유효한 bundle을 만든다.

### `activate_profile()`

슬롯 전용 baseline을 `saved_model/baseline.pkl` 같은 전역 활성 경로로 원자적으로 복사한다. GRU Service는 항상 활성 경로만 알면 되므로 프로필 슬롯 개념과 분리된다.

### `delete_profile()`

슬롯 데이터와 metadata 점유 정보를 제거한다. UI는 삭제 전 확인을 요구한다.

## 11.5 `UserProfileDialog`

Dialog 크기는 620×360이며 네 슬롯을 표시한다.

- 불러오기 모드: 슬롯 선택 후 확인 버튼을 한 번 더 눌러야 적용
- 저장 모드: 슬롯과 이름 선택, 점유 슬롯이면 덮어쓰기 확인
- 삭제: 별도 확인 후 실행

“선택 즉시 모터 이동”이 아니라 확인을 한 번 더 받는 이유는 7인치 터치 화면에서 오입력을 줄이기 위해서다.

# 12. Face Process와 졸음 판정 경로

## 12.1 현재 상태

Face 기능은 보고서와 코드 구조에는 존재하지만 `PROFILE_MODE="POSE_ONLY"`이므로 현재 제품 통합 실행에서는 Face Process, Face Shared Memory, Face GRU가 생성되지 않는다.

따라서 현재 UI나 로그에서 `Normal`이 보인다고 실제 졸음 모델이 정상 판단했다는 뜻으로 해석하면 안 된다. POSE_ONLY schema 기본값일 수 있다.

## 12.2 Face Process 구조

활성화하면 Pose와 유사하게 동작한다.

```text
Shared Frame
  → MediaPipe FaceLandmarker
  → face blendshape
  → 얼굴 특징 계산
  → Face Calibration 또는 Face GRU
  → FACE_STATE / FACE_RESULT
```

FaceLandmarker는 한 얼굴, detection/presence/tracking confidence 0.5로 구성되고 blendshape 출력을 켠다.

## 12.3 얼굴 특징

현재 Face GRU 입력 크기는 4다. 매 frame의 blendshape에서 눈 깜빡임과 입 벌림 계열 값을 추출하는 경로다. 과거 30초 통계 특징 함수도 `modules/features.py`에 존재하지만, 통합 `face_process_profile.py`는 `calculate_face_features_for_window()`가 반환하는 현재 GRU 입력 형식과 맞춰 동작한다.

Face 경로를 다시 활성화할 때는 모델이 학습된 feature 정의와 런타임 feature 정의가 정확히 같은지 우선 검증해야 한다.

## 12.4 `FaceGruService`

Pose Service와 같은 패턴을 쓴다.

- baseline 길이·finite 검사
- TFLite runtime fallback
- 30 frame deque
- 5 frame STRIDE
- invalid feature frame 제외
- `Normal`, `Drowsy` 출력

## 12.5 BOTH 모드에서 고려할 점

FaceLandmarker와 PoseLandmarker를 동시에 실행하면 CPU와 메모리 부하가 크게 늘어난다. Shared Memory 채널은 분리되어 한 프로세스 지연이 다른 프로세스 메모리를 직접 막지는 않지만, Raspberry Pi 전체 CPU contention과 발열은 함께 증가한다.

따라서 Face 재통합 전에는 다음을 계측해야 한다.

- Pose/Face 각각 FPS, P95 latency
- Frame Ring overrun
- Raspberry Pi 온도와 throttling
- Hardware loop 주기 유지 여부
- 졸음 모델 confusion matrix와 실제 환경 오탐률

<div style="page-break-after: always;"></div>

# 13. Hardware Process 전체 루프

## 13.1 가장 중요한 원칙

`run_hardware_process()`는 통합 하드웨어의 단일 소유자다.

1. 실제 I2C와 serial은 Hardware Process만 연다.
2. serial bus는 하나의 `MotorService`가 소유한다.
3. 한 루프에서 모터 1·2 controller를 먼저, 모터 3·4 controller를 다음에 update한다.
4. 정책은 Controller/Service에 위임하고 Process는 조정한다.
5. 새 실시간 센서값은 Queue를 무작정 늘리지 않고 `HARDWARE_STATE` dict에 추가한다.

특히 serial port를 여러 Process가 동시에 열면 패킷 충돌과 응답 혼선이 생길 수 있다. 단일 소유권은 안전성과 디버깅 가능성을 크게 높인다.

## 13.2 초기 생성 서비스

Hardware Process는 대략 다음 객체를 만든다.

- `HardwareConfigService`
- `ToFSensorService`와 `ToFUserXSource`
- `ADXL345IMUService`
- `MotorService`
- `Motor12Controller`
- `Motor34Controller`
- `MonitorArmPreparationController`
- `MonitorArmSafetySupervisor`
- `PostureAlertService`
- `BuzzerService`

그 뒤 ToF, IMU, serial, 부저를 열고 모터 1~4를 초기화한다. 부저 초기화 실패는 전체 모터·센서 readiness를 실패시키지 않는다. 자세 제어가 가능한데 부저 하나 때문에 제품 전체가 멈추지 않도록 한 degradation 정책이다.

## 13.3 메인 loop의 단계

```text
A. Main/Pose/Face 최신 State 가져오기
B. Main의 ordered Hardware Event 전부 처리
C. ToF 샘플링, 준비 보정 수집, rate-limited IMU 갱신
D. IMU Calibration state machine 갱신
E. Main mode에 맞는 작업 context 결정
F. ToF-only 사용자 X + Safety Supervisor
G. Motor 1·2 update, Motor 3·4 update
H. PostureAlert 판단 + Buzzer non-blocking update
I. 약 20 Hz로 통합 HARDWARE_STATE 발행
```

## 13.4 State 입력 처리

Main, Pose, Face의 State는 `get_latest()`로 가장 최근 하나만 쓴다. 과거 상태를 순서대로 재생하면 현재 사람이 돌아왔는데도 과거 미검출 상태로 모터를 제어할 수 있기 때문이다.

Pose State에서 Hardware는 다음을 주로 사용한다.

- landmark validity와 quality
- minimum visibility
- eye gap
- 최신 posture inference와 confidence
- timestamp

## 13.5 Event 처리

주요 Event는 다음 범주다.

### 준비와 프로필

- `START_MONITOR_ARM_PREPARATION`
- `FINISH_MONITOR_ARM_PREPARATION`
- `CANCEL_MONITOR_ARM_PREPARATION`
- `APPLY_USER_PROFILE`

### 모터 수동 명령

- `MONITOR_ARM_CONNECT_ALL`
- `MONITOR_ARM_MOVE_WORKING_START`
- `MONITOR_ARM_MOVE_REST`
- `MONITOR_ARM_MANUAL_IK_TARGET`
- `MONITOR_ARM_GIMBAL_JOG`
- `MONITOR_ARM_GIMBAL_JOG_STOP`

### 보정과 측정 종료

- `START_MONITOR_ARM_SENSOR_CAPTURE`
- `PREPARE_CALIBRATION`
- `CANCEL_CALIBRATION_PREPARE`
- `IMU_CALIBRATE`
- `MEASUREMENT_STOP_AND_REST`

### 설정과 저수준 제어

- `UPDATE_ALARM_SETTINGS`
- `UPDATE_CONFIG`, `RELOAD_CONFIG`, `GET_CONFIG`, `RESET_CONTROL_CONFIG`
- `MOTOR_ENABLE`, `MOTOR_DISABLE`
- `MOTOR12_REST`, `MOTOR12_RESUME`

각 이벤트 처리 블록은 성공·실패 Event를 Main으로 돌려보내 UI가 단순 전송 성공과 실제 장치 성공을 구분하게 한다.

## 13.6 Main mode와 하드웨어 동작

| Main mode | 모터 1·2 | 모터 3·4 | 비고 |
|---|---|---|---|
| PREVIEW / IDLE / CAMERA_OFF | 자동추종 안 함 | 자동제어 안 함 | 명시적 준비 명령은 별도 |
| MONITOR_ARM_PREPARATION | Dialog 명령 | 조그/준비 명령 | 수동 안전 제어 |
| CALIBRATING | 보통 위치 유지 | IMU 기준/짐벌 제어 | baseline 수집 |
| MEASURING | 안전 허용 시 거리 추종 | 안전 허용 시 IMU 보정 | 실제 측정 |

## 13.7 통합 State 발행

Hardware State는 약 20 Hz로 `hw_to_main_state_queue`에 최신값으로 발행된다. 큰 범주는 다음과 같다.

```text
HARDWARE_STATE
├─ main_mode
├─ tof
├─ imu
├─ motor
├─ motor12 / motor34 진단
├─ monitor_arm_input
├─ pose landmark quality
├─ monitor_arm
│  ├─ calibration
│  ├─ preparation
│  └─ safety
├─ posture_alert
└─ buzzer
```

새 화면은 별도 센서 Queue를 직접 읽기보다 이 snapshot에서 필요한 값을 꺼내는 것이 기본 방향이다.

## 13.8 `finally` 정리

어떤 예외가 나도 `finally`에서 부저, 준비 controller, ToF, 모터 controller, shared MotorService, IMU를 닫고 `HARDWARE_STOPPED`를 보낸다. 실제 장치 close를 owner Process 한 곳에 모은 효과가 여기서도 나타난다.

<div style="page-break-after: always;"></div>

# 14. ToF 거리 측정과 사용자 X 좌표

## 14.1 현재 제어 소스는 ToF only

현재 `monitor_arm_settings.json`에는 다음 값이 있다.

```json
"fusion": {
  "tof_weight": 1.0,
  "vision_weight": 0.0
}
```

코드도 실제 제어용 사용자 X를 ToF에만 의존하도록 구성되어 있다. 눈 간격 기반 거리 추정기와 fusion 필드는 과거 호환, 준비 보정, 진단을 위해 남아 있지만 모터 위치를 결정하지 않는다.

## 14.2 ToF 장치 설정

| 항목 | 현재 값 | 의미 |
|---|---:|---|
| mode | `hardware` | 실제 센서 사용 |
| I2C bus | 3 | `/dev/i2c-3` 계열 |
| address | 41 | 10진수 41, 0x29 |
| sample rate | 20 Hz | 목표 읽기 주기 |
| raw range | 0.03~2.0 m | 센서 레벨 허용 범위 |
| EMA alpha | 0.25 | 거리 필터 계수 |
| control user range | 0.58~0.8307655 m | 제어에 허용할 사용자 X |

안전 존재 판정 범위 0.3~1.5 m와 제어 workspace 0.58~0.8307655 m는 목적이 다르다.

- 존재 판정: 사람이 센서 앞에 있는가?
- 제어 workspace: 현재 기구가 실제로 추종 가능한 사용자 X 범위인가?

## 14.3 `ToFSensorService`

`open()`에서 Raspberry Pi 전용 Adafruit 라이브러리를 lazy import한다. 덕분에 하드웨어가 없는 개발 PC에서도 모듈 import와 일부 순수 로직 테스트가 가능하다.

`update()`는 목표 sample rate에 맞춰 읽고 다음 State를 만든다.

- available
- raw distance
- filtered distance
- valid/error
- last update time

`FixedToFSensorService`는 고정 거리로 동일 인터페이스를 제공해 하드웨어 없이 제어 로직을 시험할 수 있다.

## 14.4 `ToFUserXSource`

센서 거리에서 sensor origin offset을 반영해 base 좌표계의 사용자 X를 만든다. `validate_user_x_m()`은 finite 여부와 설정 범위를 검사하고, `clamp_user_x_m()`은 필요 시 workspace 안으로 제한한다.

여기서 **검사(validate)** 와 **제한(clamp)** 은 다르다.

- validate: 값이 허용 범위 밖이면 “유효하지 않음”이라고 판단
- clamp: 계산을 계속해야 할 때 최솟값 또는 최댓값으로 잘라 사용

안전 판단에서 범위 밖 값을 무조건 경계값으로 바꿔 정상처럼 보이면 안 되므로, 먼저 validity를 보존해야 한다.

## 14.5 좌표 방향

이 시스템에서 +X는 base에서 사용자 방향이다. 목표 모니터 X는 개념적으로 다음과 같다.

```text
monitor_target_x = user_x - desired_user_monitor_distance
```

현재 원하는 사용자-모니터 거리는 0.5 m다.

예시:

```text
사용자 X = 0.75 m
원하는 간격 = 0.50 m
목표 모니터 X = 0.25 m
```

사용자가 센서에서 멀어져 user X가 커지면 monitor target X도 커져 사용자를 따라 나와야 한다. 사용자가 가까워져 user X가 작아지면 target X가 작아져 모니터가 사용자에게서 멀어지는 방향으로 들어가야 한다.

따라서 “앞으로는 가는데 뒤로 안 간다”는 증상은 단순 부호 하나만이 아니라 다음 단계 중 어디서 막혔는지 봐야 한다.

1. ToF filtered user X가 실제로 감소하는가?
2. target monitor X가 감소하는가?
3. deadband 밖인가?
4. IK가 반대 방향 새 목표 각도를 만드는가?
5. `validate_motion()`이 거부하지 않는가?
6. controller hold reason이 비어 있는가?
7. 실제 모터 목표와 현재 각도 오차가 생기는가?

## 14.6 EMA의 장단점

alpha 0.25이면 한 번의 급격한 거리 변화가 filtered 값에 25% 반영된다. 노이즈에는 강하지만 방향 전환 직후에는 이전 이동 방향의 값이 일부 남는다.

```text
이전 filtered 0.80 m, 새 raw 0.60 m
새 filtered = 0.25×0.60 + 0.75×0.80 = 0.75 m
```

반응성을 높이려 alpha를 크게 하면 모터가 센서 노이즈를 따라 흔들릴 수 있다. 그래서 필터, deadband, trajectory step을 함께 보며 조정해야 한다.

# 15. 모터 1·2 역기구학 제어

## 15.1 계층 구조

```text
Hardware Process
  → Motor12Controller          실행 상태, 명령 주기, rest/recovery
    → MonitorArmPlanner        사용자 X에서 안전한 다음 목표 생성
      → TwoJointMonitorArm     FK, IK, path validation
    → MotorService             동기식 두 관절 명령
      → hardware/motor_control 저수준 STServo 패킷
```

## 15.2 기구 파라미터

현재 설정의 주요 값은 다음과 같다.

| 파라미터 | 현재 값 |
|---|---:|
| shoulder origin X | 0.0 m |
| shoulder origin Z | 약 0.1356 m |
| upper link | 약 0.1160 m |
| lower link | 약 0.1350 m |
| monitor offset | 0.070 m |
| 기본 monitor Z | 약 0.2561 m |

`effective_lower_link_m`는 lower link와 monitor offset을 반영해 끝단 위치 계산에 사용한다.

## 15.3 FK 개념

2관절 평면 팔의 단순화된 식은 다음과 같은 형태다.

```text
x = shoulder_x + L1 cos(q1) + L2 cos(q1 + q2)
z = shoulder_z + L1 sin(q1) + L2 sin(q1 + q2)
```

실제 코드에는 모터 명령각과 URDF/기구각 사이 영점 offset과 부호 변환이 포함된다. 따라서 화면에서 본 servo degree를 위 식에 바로 넣으면 안 된다. `command_to_urdf()`와 `urdf_to_command()`가 이 변환을 맡는다.

## 15.4 IK 개념

목표 `(x, z)`와 링크 길이로 먼저 elbow 각을 구한다.

```text
r² = x² + z²
cos(q2) = (r² - L1² - L2²) / (2 L1 L2)
```

`cos(q2)`가 -1~1 밖이면 기구가 닿을 수 없는 점이다. 그 뒤 shoulder 각을 계산한다.

```text
q1 = atan2(z, x) - atan2(L2 sin(q2), L1 + L2 cos(q2))
```

같은 끝점에 접힌 방향이 두 개 있을 수 있지만 현재 구현은 기구와 배선에 맞는 한 branch를 선택한다.

## 15.5 `validate_motion()`

IK 답이 존재한다고 바로 명령하지 않는다.

1. 모터 calibration hard range와 soft joint limit의 교집합 확인
2. 한 번에 변하는 관절각이 `max_joint_step_deg` 이내인지 확인
3. 현재 명령부터 목표까지 기본 30개 점으로 보간
4. 각 중간점의 FK 높이가 기준 Z에서 `vertical_tolerance_m` 이상 벗어나지 않는지 확인

끝점만 안전하고 중간 경로가 위험한 움직임을 막기 위한 검사다.

현재 soft limit은 다음과 같다.

- shoulder: -80°~85°
- elbow: -80°~80°
- vertical tolerance: 0.03 m
- max joint step: 5°

## 15.6 `MonitorArmPlanner`

Planner는 두 가지 모드로 목표를 만든다.

### 정상 추종

1. `user_x - desired_distance`로 모니터 target X 계산
2. 현재 X와 차이가 deadband 8 mm 이하면 HOLD
3. 한 reference update에서 X 변화량을 최대 20 mm로 제한
4. 고정 reference Z와 X로 IK
5. motion validation

### 작업 높이 복귀 Recovery

현재 FK의 Z가 작업 허용 범위 밖이면 정상 X 추종보다 높이 복귀를 우선한다. Recovery latch가 켜지고, 관절당 최대 5° 단계로 working command 방향으로 이동한다.

Recovery 중에는 다음도 확인한다.

- 새 step이 hard/soft limit 바깥으로 더 나가지 않는가?
- 이미 바깥이면 적어도 안전 범위 방향으로 들어오는가?
- timeout 내 도착하는가?

## 15.7 `Motor12Controller`

### 초기화

모터 1·2 metadata와 calibration range를 읽고 ping, 현재 각도를 확인한다. 둘 다 준비되어야 `ready=True`다.

### 20 Hz 명령과 5 Hz 기준 궤적

현재 설정은 다음과 같다.

- motor command: 20 Hz
- trajectory reference: 5 Hz
- reference X step: 최대 0.02 m

20 Hz마다 0.02 m씩 바꾸면 기존 5 Hz 설계보다 네 배 빠른 위치 변화가 된다. 이를 피하기 위해 명령 한 번당 실제 X step을 대략 다음처럼 축소한다.

```text
0.02 m × (5 Hz / 20 Hz) = 0.005 m per command
```

즉 전체 이동 속도 감각은 유지하면서 목표점을 더 촘촘하게 갱신해 뚝뚝 끊기는 느낌을 줄이는 설계다. 비전 추론이 반드시 20 Hz일 필요는 없다. Hardware loop는 가장 최근 ToF와 Pose State를 들고 독립적으로 20 Hz 궤적을 갱신한다.

### Adaptive speed

목표 관절 오차가 크면 빠르게, 작으면 느리게 명령한다. 현재 normal tracking은 대략 min 150, max 800 범위를 사용한다. 목표 근처에서 과도한 속도로 왕복하는 것을 줄인다.

### 동기 명령

`_move_normal_target()`은 모터 1과 2를 개별 순차 명령하지 않고 `MotorService.move_joints()`의 SyncWrite 계열로 보낸다. 두 관절이 같은 시점의 목표로 움직여 끝단 궤적이 덜 왜곡된다.

### 우선순위

`update()`는 대략 다음 우선순위를 갖는다.

```text
rest 요청
  > 입력/안전 invalid HOLD
  > working pose recovery
  > normal distance tracking
```

따라서 target X가 정상이어도 `rest_latched`, `safety_hold`, `recovery` 중 하나면 일반 추종 명령은 나오지 않는다.

### 주요 함수

- `move_manual_user_target()`: 준비 화면의 수동 user X/Z IK
- `move_to_rest()`: 휴식 자세 latch 및 special sync 이동
- `resume_from_rest()`: rest latch를 해제하고 working recovery 시작
- `move_to_working_smooth()`: 준비/프로필 적용 때 작업 시작 목표를 동기 명령
- `_normal_tracking_block_reason()`: normal tracking이 불가능한 정확한 이유 반환
- `_build_state()`: 현재·목표각, hold reason, error, mode를 진단 State로 구성

## 15.8 `MotorService`

MotorService는 정책을 모른다. 다음 저수준 작업만 제공한다.

- serial open/close
- motor ping
- 현재 각도 읽기
- 단일 관절 이동
- 여러 관절 동기 이동
- rest/recovery용 special 이동
- safe range와 metadata 조회

정상 경로와 special 경로를 분리한 이유는 휴식 자세가 normal working soft limit 밖일 수 있기 때문이다. Controller가 명시적으로 검증한 특별 동작만 별도 API를 쓴다.

## 15.9 속도가 아니라 끊김만 줄이는 관점

모터가 끊겨 보일 때 조절할 수 있는 축은 서로 다르다.

| 축 | 크게 하면 | 너무 크면 |
|---|---|---|
| Hardware command Hz | 목표 갱신이 촘촘 | serial/CPU 부하 |
| X step per command | 이동이 빠름 | 각도 점프, 궤적 거침 |
| servo speed | 목표를 빨리 추종 | 충격·overshoot |
| EMA alpha | 센서 반응 빠름 | 노이즈 추종 |
| deadband | 떨림 감소 | 작은 움직임 무시 |

현재 구조는 20 Hz 명령과 비례 축소한 X step으로 “전체 속도는 유지하고 갱신만 촘촘하게” 하는 방향이다.

<div style="page-break-after: always;"></div>

# 16. IMU와 모터 3·4 짐벌 제어

## 16.1 역할 분리

- 모터 1·2: 사용자와 모니터의 앞뒤 거리
- 모터 3: `wrist_flex`, 주로 IMU Y 오차
- 모터 4: `wrist_roll`, 주로 IMU X 오차

디스플레이가 달려 팔이 기울어지거나 베이스 자세가 조금 변해도 IMU 기준과 비교해 화면 방향을 보정한다.

## 16.2 `ADXL345IMUService`

현재 기본 장치 조건은 I2C bus 1, address 0x53, 목표 sample 100 Hz다.

처리 흐름은 다음과 같다.

```text
ADXL345 raw register
  → signed integer 변환
  → g 단위 변환
  → EMA(alpha 약 0.08)
  → 저장된 reference X/Y 빼기
  → imu_x_error_g, imu_y_error_g
```

Pitch와 roll 각도도 계산하지만 현재 PID의 직접 입력은 filtered X/Y g 오차다. 중력벡터 성분을 바로 사용해 작은 기울기 보정에 집중한 것이다.

## 16.3 IMU Calibration

기준 자세에서 약 3초 동안 샘플을 모으고 최소 50개가 필요하다. 평균 X/Y를 reference로 저장한다.

프로필에는 이 reference가 들어가므로 다음 실행에서 프로필 적용 시 복원할 수 있다. 다만 측정 시작 로직은 현재 Hardware State가 실제로 calibrated 상태인지 다시 확인한다.

## 16.4 `DirectIMUPIDController`

개념적인 오차는 `-imu_error`다. deadband는 약 0.01 g이고 출력은 ±24 deg/s 범위로 제한한다. 현재 설정은 주로 `Kp=120`, `Ki=0`, `Kd=0`이므로 실질적으로 P controller에 가깝다.

출력을 각속도로 보고 다음처럼 목표 각도를 적분한다.

```text
target_angle += direction_sign × pid_output_deg_per_sec × dt
```

이 방식은 IMU 오차가 계속 남아 있는 동안 목표각이 조금씩 이동하게 한다.

## 16.5 `Motor34Controller`

### 시작 목표

Controller 활성화 시 목표각을 임의의 0°로 두지 않고 현재 실제각에서 시작한다. 켜지는 순간 갑자기 중립각으로 튀는 것을 막는다.

### 제어 주기

목표 command 주기는 약 100 Hz이며 servo speed 500, acc 12 계열 고정값을 쓴다. loop가 일시 정지해 `dt`가 너무 커져도 적분에 쓰는 dt를 제한해 한 번에 큰 목표 점프가 생기지 않게 한다.

### `move_to_neutral()`

측정 종료 시 모터 3·4를 sensor/profile 기준 중립각으로 이동시킨다. 모터 1·2만 휴식자세로 보내면 화면 기울기가 남을 수 있으므로 네 모터를 종료 자세에 포함한다.

### `update(context)`

IMU available/calibrated, motor enabled, main mode, safety 허용 여부를 확인한 뒤 모터 3과 4를 순서대로 제어한다.

# 17. 안전 상태 머신과 예외 처리

## 17.1 안전 판정의 입력

`MonitorArmSafetySupervisor.update()`는 세 가지를 받는다.

- `tof_valid`
- `landmark_valid`
- 최신 posture inference

센서는 AND 조건이다.

```text
sensors_ok = tof_valid AND landmark_valid
```

즉 ToF와 landmark 중 하나만 미검출이어도 자동 추종이 멈춘다.

## 17.2 상태 목록

| 상태 | 의미 | 모터 자동추종 |
|---|---|---:|
| `AUTO_TRACKING` | 센서 정상 + 최신 Optimal 자세 | 허용 |
| `SENSOR_GRACE_HOLD` | 미검출 5초 미만 또는 재검출 안정화 중 | 정지 |
| `USER_ABSENT_RETURN` | 미검출이 5초 이상 지속 | 초기 작업 위치 복귀 요청 |
| `POSTURE_HOLD` | 비정상 자세 | 정지 |
| `POSTURE_RESULT_HOLD` | 결과 없음·낮은 confidence·오래된 결과 | 정지 |

## 17.3 센서 미검출 흐름

현재 설정값은 다음과 같다.

- presence range: 0.3~1.5 m
- absence timeout: 5초
- reacquire stable: 1초

```text
ToF 또는 landmark NG
  → 즉시 SENSOR_GRACE_HOLD
  → 5초 이내 복구: 조건 재평가
  → 5초 지속: USER_ABSENT_RETURN + request_return 1회
  → 모니터암 working initial pose로 복귀
  → 센서 재검출
  → 1초 연속 안정 확인
  → 자세가 Optimal이면 AUTO_TRACKING 재개
```

`request_return`은 매 loop마다 반복하지 않고 한 번만 발생한다. `return_requested` latch가 중복 명령을 막는다.

## 17.4 ToF 비정상 범위 알림

0.3 m 이하 또는 1.5 m 이상이면 presence valid가 false가 되고, Hardware State의 `tof_alert`가 Main UI에 표시될 수 있다. 센서 자체 raw 범위 0.03~2.0 m와 사용자 존재 범위를 혼동하지 않아야 한다.

## 17.5 landmark confidence

MediaPipe가 landmark 객체를 반환해도 눈·어깨 핵심점의 visibility가 0.60 미만이면 제어에서는 미검출로 본다. “검출 여부”만 확인할 때 생기는 낮은 신뢰도 좌표의 급격한 모터 움직임을 막는다.

## 17.6 자세 판정 조건

현재 기본값은 다음과 같다.

- posture confidence 최소 0.70
- posture result staleness 최대 1초
- 허용 자세 label: 정확히 `Optimal`

다음 경우 모두 모터 자동 추종을 하지 않는다.

- inference dict가 아직 없음
- confidence가 0.70 미만
- 결과 timestamp가 1초보다 오래됨
- `Asymmetric`
- `Forward Head`
- `Chin Propping`

정상 자세로 돌아오고 새 결과의 confidence와 timestamp가 유효해지면 자동 제어가 재개된다.

## 17.7 네 모터에 같은 안전 gate 적용

측정 중 `tracking_allowed=False`이면 모터 1·2의 거리 추종뿐 아니라 모터 3·4의 IMU 자동 보정도 hold하는 방향으로 context가 구성된다. 사용자가 비정상 자세인데 짐벌만 계속 움직이는 예외를 막는다.

## 17.8 기구 안전과 사용자 존재 안전은 별개

Safety Supervisor는 “언제 움직여도 되는가?”를 결정한다. Kinematics/Planner validation은 “그 목표와 경로가 기구적으로 안전한가?”를 결정한다.

```text
사용자/자세 안전 통과
  AND
관절각/경로/높이 안전 통과
  → 실제 모터 명령
```

둘 중 하나라도 실패하면 명령하지 않는다.

<div style="page-break-after: always;"></div>

# 18. 측정 준비 화면과 수동조작

## 18.1 `MonitorArmPreparationDialog`

준비 Dialog는 메인 UI와 같은 Qt 프로세스에서 실행되지만 Hardware를 직접 제어하지 않는다. 생성자에 받은 두 callback만 사용한다.

- `send_command(message)`: Hardware Event 전송
- `get_hardware_state()`: 최신 Hardware snapshot 읽기

100 ms QTimer로 상태를 갱신한다.

## 18.2 탭 구성

### 자세 전환·센서 탭

1. 모터 1~4 연결 확인
2. 휴식 → 작업 시작 위치
3. 작업 → 휴식 자세
4. ToF + MediaPipe 눈 간격 5초 평균

### 모터 수동조작 탭

1. 모터 1·2 user X slider 기반 IK
2. 사용자-모니터 고정거리 지정
3. monitor Z 지정
4. 모터 3·4 press-and-hold jog
5. 현재 팔과 목표 팔을 보여주는 canvas

## 18.3 버튼 활성화 조건

Dialog는 상태에 따라 위험한 버튼을 잠근다.

- 모터 연결 전: 작업 이동, IK, jog 비활성
- 이동 중: 다른 자세 이동 비활성
- 센서 평균 측정 중: 모터 조작 비활성
- 작업 시작 완료 전: manual IK 비활성
- 준비 완료: 연결 + 작업 위치 + sensor capture ready 모두 필요

## 18.4 `ArmPreparationCanvas`

현재 관절각과 목표 관절각을 FK로 점 좌표로 바꿔 팔 형상을 그린다. 이는 장식이 아니라, 수치만 볼 때 놓치기 쉬운 다음 문제를 시각적으로 찾는 도구다.

- IK branch가 반대로 선택됨
- 목표 X/Z가 기구 범위 밖
- 현재와 목표의 큰 차이
- 좌표축 방향 혼동

## 18.5 수동 IK

user slider를 움직이면 100 ms debounce timer가 시작된다. 연속 slider event마다 serial 명령을 보내지 않고 사용자가 잠깐 멈춘 최신값을 보낸다.

고정거리와 높이 spinbox는 값 변경만으로 즉시 적용되지 않는다. 사용자가 “적용 후 현재 X로 이동” 버튼을 눌러야 반영된다. 의도하지 않은 기구 이동을 줄이기 위한 UI 정책이다.

## 18.6 모터 3·4 jog

버튼을 누르는 순간 한 step을 보내고, 계속 누르면 120 ms QTimer로 반복한다. 놓으면 timer를 멈추고 `MONITOR_ARM_GIMBAL_JOG_STOP`을 보낸다.

조그값은 기본 0.5°/tick, 속도 기본 100이며 UI 범위 안에서 조정할 수 있다.

## 18.7 휴식자세 확인

휴식 이동은 충돌 위험이 있으므로 확인 Dialog가 뜬다. 현재 안내 목표는 shoulder +107.75°, elbow -92.55°다. 사용자가 모니터와 팔을 지지하고 주변 충돌물을 확인하도록 요구한다.

## 18.8 `MonitorArmPreparationController`

Hardware Process 안에서 준비 Dialog 명령을 실제 동작으로 바꾼다.

- `begin()`, `end()`: 준비 세션 생명주기
- `connect_all()`: 모터 1~4 ping과 상태 확인
- `refresh_telemetry()`: 실제 각도와 FK 갱신
- `request_working_start()`: 작업 시작 목표 이동
- `request_rest()`: 휴식자세 이동
- `command_manual_ik()`: user X/distance/Z를 IK 명령으로 변환
- `jog_gimbal()`, `stop_gimbal_jog()`: 모터 3·4 조그
- `update()`: 이동 도착, stable samples, timeout 판정
- `snapshot()`: Dialog용 준비 상태 구성

## 18.9 작업 시작 이동 완료 조건

명령 전송 자체가 완료가 아니다. 실제 모터 1·2 각도를 반복 읽어 최대 관절 오차가 arrival tolerance 이내인지 확인하고, 연속 stable samples가 필요하다.

현재 준비/작업 시작의 대표 설정은 다음과 같다.

- arrival tolerance: 3°
- stable samples: 2
- timeout: 25초

디스플레이 하중 때문에 실제각이 목표에 정확히 못 가면 이 조건에서 계속 “이동 중”으로 남을 수 있다. 무작정 tolerance를 늘리기 전에 실제 오차, 어느 관절이 막혔는지, timeout state를 함께 확인해야 한다.

# 19. 자세 알림과 부저

## 19.1 판단과 출력 분리

```text
PostureAlertService: 언제, 어떤 강도로 울릴지 결정
BuzzerService: 실제 GPIO PWM 패턴을 non-blocking으로 실행
```

알림 정책과 GPIO 구현을 분리했기 때문에 하드웨어 없이 정책만 테스트할 수 있고, Raspberry Pi GPIO 방식이 바뀌어도 자세 판단 코드를 덜 건드린다.

## 19.2 `PostureAlertService`

설정값을 받아 다음 상태를 관리한다.

- 알림 사용 여부
- 나쁜 자세 지속시간
- 반복 횟수
- 강한 알림으로 올릴 횟수 기준
- 강한 알림 후 cooldown
- 현재 추적 label과 시작시각

나쁜 자세가 한 frame 검출됐다고 즉시 울리지 않고 지속 조건을 만족해야 한다. 짧은 오분류에 의한 잦은 부저를 막는다.

## 19.3 `BuzzerService`

현재 기본은 GPIO18, PWM 약 2000 Hz, duty 0.5다. `play_command()`는 패턴을 queue에 넣고 즉시 반환한다. `update()`가 현재 phase의 종료시각을 확인해 ON/OFF와 반복 횟수를 진행한다.

즉 다음과 같은 blocking 코드를 쓰지 않는다.

```python
# Hardware loop를 멈추므로 사용하지 않는 방식
buzzer_on()
time.sleep(3)
buzzer_off()
```

대신 매 Hardware loop마다 state machine을 한 단계씩 갱신한다. 부저가 울리는 동안에도 ToF, IMU, 안전, 모터 제어가 계속된다.

## 19.4 Raspberry Pi 5 GPIO 고려

gpiozero가 SOC peripheral base address를 못 찾거나 기본 `lgpio`가 gpiochip을 열지 못하는 환경을 고려해 Raspberry Pi 5의 `/dev/gpiochip*` 탐색과 lgpio 기반 PWM 경로가 포함되어 있다.

그래도 권한, gpiochip 번호, 라이브러리 설치 문제로 부저가 열리지 않을 수 있다. 이때 `BuzzerService.get_state()`의 `available=False`, `last_error`를 확인한다. 앞서 설명한 대로 부저 실패는 전체 Hardware READY를 막지 않는다.

<div style="page-break-after: always;"></div>

# 20. 설정, 상태, 파일 저장

## 20.1 설정과 상태를 분리한 이유

**설정(Config)** 은 재시작 후에도 유지되어야 하는 사용 의도다.

- PID gain
- deadband
- 링크 길이
- 안전 timeout
- 알림 시간

**상태(State)** 는 지금 순간의 사실이다.

- 현재 ToF 72.4 cm
- 모터 1 실제각 12.3°
- safety state가 POSTURE_HOLD
- buzzer가 ON phase

State를 매번 JSON에 쓰면 storage 부담이 크고 오래된 값이 재시작 후 진실처럼 보일 수 있다. 그래서 `HardwareRuntimeStateStore`는 메모리 only다.

## 20.2 `HardwareConfigService`

주요 기능은 다음과 같다.

- 기본 config 생성
- 디스크 JSON load/reload
- 누락 필드에 default deep merge
- 값 범위 normalize/validate
- control patch 업데이트
- IMU calibration record 저장·삭제
- default reset

`_deep_merge()`는 새 버전에서 설정 필드가 추가되어도 예전 JSON을 읽을 수 있게 한다.

## 20.3 현재 모니터암 주요 설정 요약

| 범주 | 항목 | 값 |
|---|---|---:|
| 거리 | desired distance | 0.50 m |
| 거리 | deadband | 0.008 m |
| 거리 | max X step/reference | 0.020 m |
| ToF | sample | 20 Hz |
| ToF | EMA alpha | 0.25 |
| 모터 1·2 | command | 20 Hz |
| 모터 1·2 | reference | 5 Hz |
| 모터 1·2 | pose speed | 최대 800, 최소 150 |
| 안전 | absence | 5.0 s |
| 안전 | reacquire | 1.0 s |
| 안전 | posture confidence | 0.70 |
| 안전 | posture stale | 1.0 s |

## 20.4 `HardwareRuntimeStateStore`

MainWindow가 받은 최신 Hardware State를 lock 아래에서 복사해 저장한다. Dialog나 다른 UI 코드가 `get_imu_state()`, `get_motor_state()` 등으로 안전하게 snapshot을 가져간다.

여기서도 반환값을 복사하는 이유는 UI가 중첩 dict를 바꿔 공유 상태를 오염시키는 것을 막기 위해서다.

## 20.5 알림 설정

`AlarmSettings` dataclass는 UI 값의 자료형과 기본값을 정의한다. `SettingsManager`는 JSON load/save와 clamp를 담당한다.

현재 UI에서 사용하는 대표값은 다음과 같다.

- alarm enabled
- bad posture duration
- posture alert count
- posture strong limit
- strong alert cooldown minutes

Face 피로 관련 설정 widget은 코드에 흔적이 있지만 현재 주석 처리되어 있다.

## 20.6 로그와 리포트

측정 세션이 시작되면 `StudyLogger`가 `data/session_log` 아래에 결과를 기록한다. Streamlit app은 저장된 데이터를 읽어 별도 리포트 화면을 제공한다.

MainWindow는 Streamlit server를 포트 8501로 띄우고 브라우저를 여는 책임만 갖는다. 리포트 데이터 가공은 Streamlit 쪽 책임이다.

## 20.7 실행 스크립트 주의

`WorkSpace/pyQt/start_pyqt.sh`에는 배포 당시의 절대경로가 하드코딩될 수 있다. 현재 workspace 이름과 다르면 스크립트 실행은 실패해도 `mainpyQt.py` 코드 자체 문제는 아닐 수 있다.

직접 실행할 때의 기준 형태는 다음과 같다.

```bash
cd /home/willtek/POCO/WorkSpace/pyQt
python3 mainpyQt.py
```

실제 배포에서는 venv Python, `DISPLAY`, I2C/serial/GPIO 권한, 작업 디렉터리를 함께 확인해야 한다.

<div style="page-break-after: always;"></div>

# 21. 사용자 시나리오별 전체 흐름

## 21.1 시나리오 A: 처음 사용하는 사람의 보정

```text
[사용자] 초기값 준비 클릭
    │
    ▼
[MainWindow] ensure_camera_worker()
    │
    ├─ CameraWorker QThread 시작
    ├─ Hardware Process 시작
    ├─ Pose Process 시작 및 MediaPipe warm-up
    └─ ResultWorker QThread 시작
    │
    ▼
[CameraWorker] PREPARING, Pose 준비 frame 공급
    │
    ▼
[Hardware] START_MONITOR_ARM_PREPARATION
    │
    ▼
[Dialog]
    1) 모터 1~4 연결 확인
    2) 휴식 → 작업 시작 위치
    3) 필요 시 모터 1·2 IK / 모터 3·4 jog
    4) ToF + 눈 간격 5초 평균
    5) 준비 완료
    │
    ▼
[MainWindow] 초기값 측정시작 버튼 활성화
    │
    ▼
[사용자] 초기값 측정시작 클릭
    │
    ▼
[Hardware] IMU 기준값 Calibration
    │ HARDWARE_CALIBRATION_READY
    ▼
[Pose] 정상 feature 첫 검출부터 5초 baseline 수집
    │ POSE_CALIBRATION_DONE
    ▼
[ResultWorker] 활성 Process 결과 집계
    │ calibration_finished
    ▼
[MainWindow] 성공 표시 → 프로필 저장 여부 질문
```

### 실패를 예상해야 하는 지점

- MediaPipe 준비 실패
- 모터 1~4 중 하나 ping 실패
- 작업 시작 위치 timeout
- ToF 또는 eye gap 샘플 부족
- IMU sample 부족
- Pose 첫 feature 10초 동안 없음
- Pose 5초 구간 유효 샘플 90개 미만

각 단계가 ACK와 snapshot으로 분리되어 있으므로 상태 메시지를 보고 실패 계층을 좁힐 수 있다.

## 21.2 시나리오 B: 저장된 프로필로 바로 시작

```text
[사용자] 프로필 버튼
  → 슬롯 선택
  → 확인 버튼
  → baseline을 활성 경로로 복사
  → Hardware Process 시작
  → 저장된 모니터암/IMU 정보 복원
  → 모터 1~4 연결 확인
  → 작업 초기위치 이동
  → 실제 각도 stable arrival
  → USER_PROFILE_APPLIED
  → 자세측정 시작 버튼 활성화
```

프로필 적용은 보정을 다시 하지 않지만, 장치 연결과 실제 모터 위치 확인을 건너뛰지 않는다.

## 21.3 시나리오 C: 측정 시작

```text
[Main] baseline 존재 확인
  → CameraWorker.start_measurement()
  → IMU calibrated 확인
  → Motor ready 확인
  → 현재 세션 monitor calibration 확인
  → Pose START_MEASUREMENT
  → Pose baseline/scaler/TFLite lazy load
  → POSE_MEASUREMENT_STARTED ACK
  → Shared Frame 공급 재개
  → Main State MEASURING
  → 자세 결과, 모터 자동추종, 부저 알림 시작
```

초기 30개 유효 frame이 GRU window에 차기 전까지는 자세 결과가 없다. 이때 Safety는 `POSTURE_RESULT_HOLD`로 모터 자동추종을 기다린다.

## 21.4 시나리오 D: 정상 자세에서 앞뒤 이동

```text
ToF raw 거리
  → EMA filtered 거리
  → user X validate
  → Safety: ToF OK + landmark OK + Optimal
  → target monitor X = user X - 0.5 m
  → deadband / X step
  → IK
  → joint/path safety validation
  → adaptive speed
  → motor 1·2 synchronized command
  → actual angle/FK feedback
```

비전 결과는 거리 자체를 만들지 않으며 “현재 움직여도 되는 정상 자세인가?”를 gate한다.

## 21.5 시나리오 E: 비정상 자세

```text
Pose GRU → Forward Head, confidence 0.86
  → Safety POSTURE_HOLD
  → motor 1·2 거리 추종 HOLD
  → motor 3·4 IMU 자동 보정 HOLD
  → PostureAlertService 지속시간 누적
  → 조건 충족 시 Buzzer command
  → 사용자가 Optimal로 복귀
  → 최신 confidence/timestamp 통과
  → AUTO_TRACKING 재개
```

## 21.6 시나리오 F: 사용자 또는 센서 이탈

```text
ToF NG 또는 landmark NG
  → 즉시 모든 자동추종 HOLD
  → 0~5초: SENSOR_GRACE_HOLD
  → 5초 이상: USER_ABSENT_RETURN
  → 모니터암 작업 초기 위치 복귀 요청
  → 다시 ToF와 landmark 모두 정상
  → 1초 안정화
  → 새 Optimal 결과
  → 현재 사용자 X에 맞춘 추종 재개
```

## 21.7 시나리오 G: 측정 종료

```text
[사용자] Cam Off
  → Vision STOP + logger 종료
  → MEASUREMENT_STOP_AND_REST
  → motor 1·2 rest
  → motor 3·4 neutral
  → actual arrival polling
  → ACK
  → 1.5초 뒤 Camera QThread 정지
  → Process는 IDLE로 유지
```

앱 창 자체를 닫으면 그 후 Process, Queue, Shared Memory까지 완전 종료한다.

# 22. 종료와 자원 정리

## 22.1 종료가 어려운 이유

이 프로그램에는 Qt 객체, QThread, multiprocessing Process, Queue feeder thread, Shared Memory, camera, MediaPipe native object, I2C, serial, GPIO, Streamlit subprocess가 함께 존재한다.

종료 순서가 잘못되면 다음 문제가 생길 수 있다.

- 생산자가 닫힌 Queue에 쓰기
- Shared Memory unlink 후 Reader 접근
- QObject가 파괴된 뒤 queued Signal 실행
- 카메라 native resource double release
- serial port가 비정상 상태로 남음

## 22.2 Cam Off는 완전 종료가 아님

Cam Off는 빠른 재시작과 native 안정성을 위해 Process 인프라를 유지한다. 따라서 `ps`에서 Hardware/Pose Process가 남아 있다고 곧바로 leak으로 판단하면 안 된다. Main state가 `CAMERA_OFF`이고 모터가 자동 제어하지 않는지 함께 봐야 한다.

## 22.3 앱 종료 순서

```text
MainWindow.closeEvent
  → Dialog close
  → Main Signal slot 차단 플래그
  → CameraWorker.shutdown
    → camera loop 정지/카메라 release
    → ResultWorker Signal block
    → Manager.stop: 자식 Process 종료
    → ResultWorker.stop
    → Queue close/join_thread
    → Shared Memory close/unlink
  → Streamlit terminate
  → Qt event loop 종료
```

## 22.4 `monotonic()`과 wall clock

timeout과 경과시간은 가능한 `time.monotonic()`을 쓴다. 시스템 시각이 NTP로 바뀌어도 duration 계산이 뒤틀리지 않는다.

Process 사이 결과 timestamp나 로그 시각에는 `time.time()` 또는 `timestamp_ns`가 쓰인다. 서로 목적이 다르므로 혼용 시 단위를 확인해야 한다.

<div style="page-break-after: always;"></div>

# 23. 성능을 위해 고려된 설계

## 23.1 저지연 우선

이 시스템은 영상의 모든 frame을 반드시 분석하는 batch 시스템이 아니다. 현재 사람에 맞춰 모터를 제어하는 실시간 시스템이므로 “프레임 손실 0”보다 “오래된 프레임을 처리하지 않음”이 중요하다.

이를 위해 다음을 쓴다.

- State Queue 크기 1
- Shared Ring `read_latest()`
- Ring full 시 camera block 대신 frame drop
- GUI 15 FPS 제한
- 결과 UI 0.5초 제한
- 유효 feature만 GRU window에 추가

## 23.2 비전과 모터 주기의 분리

Hardware 20 Hz trajectory가 Pose 결과 20 Hz를 요구하는 것은 아니다.

```text
Pose inference: 최신 정상/비정상 상태를 간헐적으로 갱신
ToF: 약 20 Hz 거리 갱신
Hardware controller: 최신 값을 보유하고 약 20 Hz 목표 보간
Motor 3·4: IMU 최신값으로 더 빠른 내부 갱신
```

이런 **multi-rate system**에서는 각 입력에 staleness 조건이 필요하다. Pose 결과는 1초를 넘으면 자동추종을 막는다.

## 23.3 모델 lazy load와 warm-up

- MediaPipe: Process READY 전에 warm-up
- GRU: 측정할 때만 lazy load
- 측정 시작 ACK 전 frame 공급 중단

초기 응답성, 메모리, Ring overrun을 함께 고려한 조합이다.

## 23.4 프로파일 지표

Camera와 각 Vision Process가 약 2초 단위로 평균, P95, MAX를 출력한다. Raspberry Pi에서 볼 지표는 다음과 같다.

- Camera actual FPS
- GUI FPS
- Pose/Face processing FPS
- Queue latency P95
- end-to-end latency P95
- Ring pending/overrun
- dropped frame sequence
- CPU temperature

### 해석 예시

| 증상 | 가능한 해석 |
|---|---|
| Camera FPS 정상, Pose FPS 낮음 | MediaPipe/GRU CPU 병목 |
| Ring overrun 증가 | Producer가 consumer보다 빠름 |
| Queue latency 높음 | 오래된 frame이 밀리거나 CPU contention |
| Pose FPS 정상, UI만 끊김 | Qt paint 또는 Signal 과다 |
| Vision 정상, 모터 끊김 | Hardware command/trajectory/serial 쪽 |

## 23.5 메모리 복사 절감

프레임은 Shared Memory로 전달하지만 화면 표시용 QImage는 `.copy()`를 한다. QImage가 원본 numpy buffer 수명 이후에도 안전하게 존재해야 하기 때문이다. 모든 복사를 제거하는 것이 항상 안전한 최적화는 아니다.

## 23.6 네이티브 안정성

`spawn`, 명시적 close 순서, `faulthandler`, Cam Off의 Process 재사용은 MediaPipe/OpenCV/Qt/native resource 충돌을 고려한 선택이다.

# 24. 오류 처리와 현재 주의할 지점

## 24.1 오류의 세 층

### 사용자 입력 오류

- baseline 없이 측정
- 준비 보정 없이 측정
- 모터 연결 전 수동 IK
- 센서 수집 중 jog

UI에서 버튼 비활성 또는 경고창으로 막는다.

### 런타임 장치 오류

- 카메라 open 실패
- I2C read 실패
- motor ping 실패
- buzzer GPIO 실패

State의 available/error와 Event로 전달한다. 부저처럼 핵심 기능이 아닌 장치는 degraded mode를 허용한다.

### 계산·모델 오류

- baseline 크기 불일치
- NaN/Inf feature
- TFLite/scaler 파일 없음
- unreachable IK
- unsafe joint/path

측정 시작 실패 ACK 또는 controller hold reason으로 나타난다.

## 24.2 “명령 전송”과 “동작 성공” 구분

`send_hardware_command()`가 `True`라는 뜻은 Queue에 들어갔다는 뜻이다. 실제 motor ping, 이동, 도착 성공은 이후 Event ACK를 봐야 한다.

문제 분석 시 다음 표현을 구분한다.

- command enqueue 성공
- controller가 명령 수락
- serial write 성공
- 실제 각도 변화 확인
- arrival tolerance stable 완료

## 24.3 하중이 늘어난 작업 시작 위치

7인치 디스플레이 장착으로 팔이 아래로 처지면 목표 명령각과 실제각 차이가 커질 수 있다. 그 결과 `working_start`가 timeout까지 계속 이동 중으로 남을 수 있다.

확인 순서는 다음이 안전하다.

1. 실제 shoulder/elbow angle이 읽히는가?
2. 최대 오차가 어느 값에서 멈추는가?
3. 한 관절만 오차가 큰가?
4. 모터 torque/전원/기구 유격 문제인가?
5. tolerance가 실제 반복 정밀도보다 지나치게 작은가?
6. 목표 작업자세 자체를 하중 기준으로 다시 보정해야 하는가?

tolerance 완화는 마지막 단계다. 너무 크게 하면 실제로 충분히 이동하지 않았는데 완료 처리될 수 있다.

## 24.4 반대 방향 추종이 막히는 경우

사용자가 가까워졌는데 모니터가 뒤로 들어가지 않는다면 다음 원인이 독립적으로 가능하다.

- ToF filtered 값이 이전 큰 값에 머묾
- control minimum user X clamp
- 8 mm deadband 안
- target X는 바뀌었으나 IK branch/limit 문제
- 현재 Z가 working 범위 밖이라 recovery 우선
- vertical path validation 실패
- soft/hard joint limit
- posture/sensor safety hold
- rest latch 미해제
- command는 나오지만 servo 하중으로 실제 이동 안 함

따라서 단순히 IK 식만 수정하기 전에 State의 `raw → filtered → user_x → target_pose → target_angles → hold_reason → actual_angles`를 한 줄로 비교해야 한다.

## 24.5 Start script 경로

현재 `start_pyqt.sh`의 절대경로가 `/home/willtek/VisionPoseCoach/...` 형태라면 실제 `/home/willtek/POCO/...` 배치와 다를 수 있다. 자동 시작 문제와 Python 코드 문제를 분리해 판단한다.

## 24.6 Face 재활성화 전 주의

Face 코드가 존재한다는 이유만으로 `BOTH`만 바꾸어 제품 기능이 완성되는 것은 아니다.

- 모델/Scaler/feature definition 버전 일치
- Face baseline 사용자별 저장
- Raspberry Pi 동시 성능
- 졸음 threshold와 시간 집계
- UI label과 리포트 의미
- 오탐/미탐 성능 지표

이 항목을 검증한 뒤 활성화해야 한다.

<div style="page-break-after: always;"></div>

# 25. 디버깅 가이드

## 25.1 가장 먼저 수집할 정보

문제가 생기면 다음 네 묶음을 같은 시간대에 기록한다.

1. Main UI status 문자열
2. Hardware State snapshot
3. Pose Process 통계와 최신 Pose State
4. motor controller 진단과 실제 각도

“안 움직인다”만으로는 안전하게 멈춘 것인지, 명령이 사라진 것인지, 하드웨어가 못 움직인 것인지 구분할 수 없다.

## 25.2 센서 미검출만 계속 표시될 때

확인 순서:

```text
tof.available
tof.filtered_distance_m
safety.tof_presence_valid
pose landmark_valid
pose_landmark_quality
pose_landmark_min_visibility
safety.landmark_presence_valid
safety.state / reason
```

ToF와 IMU가 보정 화면에서 보였다는 사실만으로 측정 중 landmark gate가 정상이라는 뜻은 아니다. Pose Process가 MEASURING frame을 받고 최신 State를 Hardware에 보내는지도 확인해야 한다.

## 25.3 자세측정 시작 실패

메시지별 지점:

| 메시지 성격 | 확인 위치 |
|---|---|
| AI Process 준비 중 | READY ACK, Process 생존 |
| IMU/Motor 준비 안 됨 | `hardware_state.imu`, `motor` |
| ToF/눈 평균 없음 | `monitor_arm.calibration.session_ready` |
| baseline 없음 | `saved_model/baseline*.pkl` |
| baseline 크기 오류 | GRU Service load log |
| 모델/Scaler 없음 | `saved_model` 경로 |

## 25.4 프로필 적용이 끝나지 않을 때

```text
USER_PROFILE_APPLY_STARTED가 왔는가?
  → motor 1~4 ping 결과
  → 저장 IMU reference 복원 결과
  → 목표 motor angle
  → 현재 actual angle
  → max error
  → stable sample count
  → arrival timeout
```

Main 버튼 상태만 보지 말고 Hardware preparation/profile apply State를 본다.

## 25.5 모터 1·2가 아예 움직이지 않을 때

다음 순서로 gate를 확인한다.

1. `main_mode == MEASURING`
2. motor enabled/ready
3. `stop_rest_pending == False`
4. ToF valid
5. landmark valid
6. safety `tracking_allowed=True`
7. Motor12 rest latch false
8. normal tracking block reason 없음
9. target X가 deadband 밖
10. IK/validation 성공
11. command rate 제한 통과
12. serial move success

## 25.6 한 방향만 움직일 때

동일 시각의 값을 표로 적는다.

| 항목 | 멀어질 때 | 가까워질 때 |
|---|---:|---:|
| raw ToF |  |  |
| filtered ToF |  |  |
| user X |  |  |
| target monitor X |  |  |
| current monitor X(FK) |  |  |
| target shoulder/elbow |  |  |
| actual shoulder/elbow |  |  |
| hold reason |  |  |

부호 문제라면 target X부터 반대로 움직이고, validation 문제라면 target은 정상인데 hold reason이 생기며, 하드웨어 문제라면 command target은 정상인데 actual angle이 따라오지 않는다.

## 25.7 모터가 뚝뚝 끊겨 보일 때

다음 지표를 함께 본다.

- Hardware loop 주기
- Motor12 command interval
- reference update interval
- 각 command의 X step
- adaptive speed
- ToF EMA 변동
- serial write 성공률
- 실제 각도 read 간격
- Pose safety state가 TRACKING/HOLD를 반복하는지

안전 상태가 confidence 경계에서 반복되면 궤적 생성기가 부드러워도 명령 자체가 on/off되어 끊겨 보일 수 있다.

## 25.8 모터 3·4가 떨릴 때

- IMU reference가 현재 중립자세와 맞는가?
- filtered error가 deadband 0.01 g 주변에서 부호를 반복하는가?
- PID direction sign이 맞는가?
- 목표각이 calibration range 끝에 붙어 있는가?
- dt가 비정상적으로 큰가?
- 디스플레이 케이블이 기구에 외력을 주는가?

## 25.9 부저가 안 울릴 때

두 층을 나눠 확인한다.

```text
PostureAlertService command가 생성되는가?
    아니오 → 자세 label, hold time, count, cooldown 설정
    예
    ▼
BuzzerService available인가?
    아니오 → gpiochip, 권한, GPIO18, lgpio/gpiozero
    예
    ▼
active phase / PWM output / 실제 배선
```

## 25.10 종료 시 hang 또는 segfault

- 앱 종료 전에 자식 Process가 살아 있는지
- ResultWorker Signal이 차단됐는지
- Queue close 전에 producer가 끝났는지
- Shared Memory unlink owner가 하나인지
- camera release가 두 번 호출되는지
- `-X faulthandler` 로그의 마지막 native 호출

# 26. 테스트 방법과 권장 읽기 순서

## 26.1 하드웨어 없이 가능한 테스트

다음은 실제 모터를 움직이지 않고도 검증할 수 있다.

- 모든 Python 파일 `py_compile`
- settings JSON schema와 범위
- FK→IK 왕복 오차
- unreachable target 거부
- joint/path safety validation
- Safety Supervisor 시간 상태 전이
- UserProfile save/load/activate
- Queue latest/ordered 정책
- Shared Ring 최신 frame 정책
- Calibration minimum sample과 기존 baseline 보존
- GRU baseline shape 검사
- Hardware State schema 생성

프로젝트의 `hardware_logic_selftest.py`, `test_user_profile_and_safety.py`가 일부 순수 로직 검증을 제공한다.

## 26.2 하드웨어 연결 후 단계별 테스트

한 번에 전체 자동제어를 켜기보다 아래 순서가 안전하다.

1. ToF 단독 raw/filtered 거리와 방향
2. IMU 단독 X/Y reference와 오차 부호
3. 모터 1~4 ping·현재각 읽기
4. 낮은 속도의 모터 3·4 jog
5. 모터 1·2 working/rest 전환
6. 수동 user X IK 양방향
7. 5초 준비 보정
8. Pose baseline 보정
9. 정상 자세에서 짧은 자동추종
10. 비정상 자세 hold
11. 센서 하나씩 가리고 5초 return/reacquire
12. 측정 종료 네 모터 자세

## 26.3 코드 읽기 권장 순서

### 1회차: 사용자 흐름

1. `mainpyQt.py`
2. `camera_worker_profile_all.py`
3. `result_worker.py`

목표는 버튼이 어느 Worker 함수로 연결되는지만 보는 것이다.

### 2회차: 프로세스와 IPC

1. `vision_process_manager_profile.py`
2. `ipc/queue_utils.py`
3. `ipc/shared_frame_ring.py`
4. `pose_process_profile.py`

목표는 State/Event/Frame 통로를 구분하는 것이다.

### 3회차: 모터 제어

1. `hardware_process.py`
2. `monitor_arm_safety_supervisor.py`
3. `motor12_controller.py`
4. `monitor_arm_planner.py`
5. `monitor_arm_kinematics.py`
6. `motor_service.py`

목표는 실제 명령 직전 모든 gate를 찾는 것이다.

### 4회차: 준비·프로필·부가 기능

1. `monitor_arm_preparation_dialog.py`
2. `monitor_arm_preparation_controller.py`
3. `user_profile_service.py`
4. `imu_service.py`, `motor34_controller.py`
5. `posture_alert_service.py`, `buzzer_service.py`

## 26.4 함수를 수정할 때 지켜야 할 경계

| 바꾸려는 것 | 우선 수정 위치 |
|---|---|
| 버튼·메시지·화면 | MainWindow/Dialog |
| 카메라 선택·GUI FPS | CameraWorker |
| Process on/off | VisionProcessManager |
| Pose 특징·landmark quality | Pose Process/modules.features |
| 모델 window/stride | GRU Service/config |
| 사용자 거리 필터 | ToF Service/UserX Source |
| IK와 좌표계 | Kinematics |
| 목표 생성·recovery | Planner |
| 명령주기·speed·rest latch | Motor12Controller |
| IMU 기울기 보정 | IMU Service/Motor34Controller |
| 움직임 허용 조건 | Safety Supervisor |
| GPIO 패턴 | BuzzerService |
| 알림 정책 | PostureAlertService |

UI에서 모터 문제를 임시로 고치거나 Hardware Process에 IK 식을 복사하면 책임이 섞인다. 기존 경계를 유지하는 것이 장기적으로 중요하다.

<div style="page-break-after: always;"></div>

# 27. 부록

## 27.1 MainWindow 함수 사전

### 초기화와 공통

| 함수 | 한 줄 설명 |
|---|---|
| `__init__` | UI 로드, 서비스·상태 생성, Signal 연결 |
| `initialize_camera_label` | Camera Off 화면 초기화 |
| `initialize_button_state` | 세션 시작 버튼 정책 적용 |
| `initialize_realtime_labels` | 결과·센서 라벨 리셋 |
| `has_baseline` | 현재 PROFILE_MODE에 필요한 파일 확인 |
| `resolve_workspace_path` | WorkSpace 기준 절대경로 생성 |

### Worker와 상태

| 함수 | 한 줄 설명 |
|---|---|
| `ensure_camera_worker` | 새 Worker 생성 또는 Cam Off Worker 재사용 |
| `stop_camera_worker` | 카메라만 정지하거나 전체 shutdown |
| `on_pose_state_changed` | 최신 Pose State 보관 |
| `on_face_state_changed` | 최신 Face State 보관 |
| `on_hardware_changed` | 최신 Hardware State 저장·센서 UI 갱신 |
| `update_sensor_monitor` | ToF/Pose/IMU/Safety 2줄 진단 표시 |
| `on_hardware_event_changed` | 설정, 프로필, 종료 ACK 처리 |
| `send_hardware_command` | Manager를 통해 ordered Hardware Event 전송 |

### 사용자 동작

| 함수 | 한 줄 설명 |
|---|---|
| `on_user_profile_clicked` | 프로필 선택·활성화·하드웨어 복원 시작 |
| `on_manual_arm_clicked` | manual-only 준비 Dialog 열기 |
| `on_camera_on_clicked` | 저장 baseline 기반 측정 시작 요청 |
| `on_calibration_clicked` | 모니터암 준비 Dialog 시작 |
| `on_monitor_arm_preparation_finished` | 준비 결과에 따라 실제 보정 버튼 개방 |
| `on_calibration_start_clicked` | IMU→Vision 보정 시작 |
| `on_measurement_started` | 측정 ACK에 따라 UI 잠금/복구 |
| `on_camera_off_clicked` | 측정 중지와 종료 자세 요청 |
| `_finish_camera_off` | ACK/fallback 후 Camera QThread 정지 |
| `on_calibration_finished` | 보정 결과 표시와 프로필 저장 질문 |
| `save_current_profile` | 현재 baseline·하드웨어 보정 bundle 저장 |

### 화면·설정·리포트

| 함수 | 한 줄 설명 |
|---|---|
| `on_result_changed` | 자세, confidence, 시간, TOP 3 표시 |
| `update_camera_view` | QImage를 label 크기에 맞춰 표시 |
| `set_status` | 상태 로그와 status label 갱신 |
| `initialize_settings_ui` | 알림 JSON을 UI에 로드 |
| `collect_settings_from_ui` | widget 값을 AlarmSettings로 변환 |
| `on_save_settings_clicked` | JSON 저장 + runtime IPC 반영 |
| `on_report_clicked` | Streamlit server 시작 |
| `open_linux_browser` | Raspberry Pi Chromium kiosk 실행 |
| `closeEvent` | 앱 전체 shutdown |
| `keyPressEvent` | Esc 전체화면 전환 |

## 27.2 CameraWorker 함수 사전

| 함수 | 한 줄 설명 |
|---|---|
| `run` | 카메라 read, shared frame write, GUI frame emit 루프 |
| `create_camera_source` | PiCamera2/OpenCV source 선택 |
| `apply_pending_command` | READY 전에 눌린 요청 지연 실행 |
| `start_preview` | Vision 정지, PREVIEW state |
| `start_monitor_arm_preparation` | 준비용 Pose processing 시작 |
| `finish_monitor_arm_preparation` | 준비 종료, Preview 복귀 |
| `start_calibration` | Hardware IMU precheck 시작 |
| `_begin_vision_calibration` | Hardware ACK 후 Pose/Face baseline 수집 |
| `_on_hardware_event` | IMU 보정 Event를 Vision 흐름과 연결 |
| `start_measurement` | 세션·파일·장치 조건 검사 후 모델 시작 |
| `_on_measurement_start_finished` | ACK 후 frame 공급과 MEASURING 전환 |
| `stop_measurement` | Vision과 logger 정지 |
| `stop_camera_only` | Process 유지, Camera QThread만 정지 |
| `shutdown_vision_resources` | Process→Worker→Queue→Shared Memory 정리 |

## 27.3 VisionProcessManager 함수 사전

| 함수 | 한 줄 설명 |
|---|---|
| `start` | Hardware와 활성 Vision Process 생성 |
| `start_monitor_arm_preparation` | 준비 명령과 frame 공급 |
| `start_calibration` | 활성 Vision Process에 calibration broadcast |
| `start_measurement` | frame pause 후 model start command |
| `resume_measurement_frames` | ACK 후 frame 공급 재개 |
| `stop_analysis` | Vision STOP과 frame 차단 |
| `send_main_state` | 최신 Main mode를 Hardware로 전달 |
| `send_hardware_command` | 유실 방지 Hardware Event 전달 |
| `write_frame` | 활성 shared ring에 프레임 기록 |
| `get_stats` | ring write/read/overrun 통계 |
| `stop` | 자식 Process 정상 종료와 fallback terminate |
| `close_queues` | Queue feeder와 descriptor 정리 |

## 27.4 핵심 Service 함수 사전

### Kinematics와 Planner

| 함수 | 의미 |
|---|---|
| `monitor_target_from_user` | user X와 원하는 간격으로 monitor X 계산 |
| `TwoJointMonitorArm.forward` | 관절각→모니터 X/Z |
| `TwoJointMonitorArm.inverse` | 모니터 X/Z→관절각 |
| `validate_motion` | 각도 범위·step·중간 경로 높이 검사 |
| `MonitorArmPlanner.plan` | deadband/step/IK/validation을 합친 다음 목표 |
| `request_working_pose_recovery` | 작업 높이 복귀 latch 시작 |

### Motor

| 함수 | 의미 |
|---|---|
| `MotorService.move_joints` | 정상 두 관절 동기 명령 |
| `MotorService.move_joints_special` | 검증된 rest/recovery 동기 명령 |
| `Motor12Controller.update` | rest/safety/recovery/tracking 우선순위 실행 |
| `move_to_rest` | 휴식 latch와 목표 이동 |
| `resume_from_rest` | latch 해제와 working recovery |
| `move_to_working_smooth` | 준비·프로필의 작업 위치 동기 이동 |
| `Motor34Controller.update` | IMU 기반 모터 3·4 제어 |
| `move_to_neutral` | 종료 시 모터 3·4 중립 이동 |

### 센서와 안전

| 함수 | 의미 |
|---|---|
| `ToFSensorService.update` | rate limit된 raw/EMA 거리 State |
| `ToFUserXSource.read_user_x_m` | 센서값을 base user X로 변환 |
| `ADXL345IMUService.update` | raw→filtered→reference error State |
| `start_calibration` | IMU 기준 샘플 수집 시작 |
| `MonitorArmSafetySupervisor.update` | sensor/posture gate 상태 전이 |
| `snapshot` | 현재 안전 상태와 return request 제공 |

### Calibration, Profile, Alert

| 함수 | 의미 |
|---|---|
| `CalibrationService.start/update/finish` | 유효 feature 5초 평균과 최소 샘플 검사 |
| `UserProfileService.save_profile` | baseline·보정 bundle 4슬롯 저장 |
| `activate_profile` | 슬롯 baseline을 활성 경로로 복원 |
| `PostureAlertService.update` | 자세 지속시간·반복·cooldown 판단 |
| `BuzzerService.play_command` | non-blocking 패턴 예약 |
| `BuzzerService.update` | PWM phase state machine 진행 |

## 27.5 주요 Command/Event 사전

| 이름 | 방향 | 의미 |
|---|---|---|
| `START_PREPARATION` | Main→Pose | 준비용 landmark 처리 |
| `START_CALIBRATION` | Main→Pose/Face | Vision baseline 수집 |
| `START_MEASUREMENT` | Main→Pose/Face | baseline/model 로드와 추론 시작 |
| `STOP` | Main→Vision | 분석 정지 |
| `POSE_READY` | Pose→Main | MediaPipe 준비 완료 |
| `POSE_CALIBRATION_DONE` | Pose→Main | baseline 수집 결과 |
| `POSE_MEASUREMENT_STARTED` | Pose→Main | 모델·baseline 준비 ACK |
| `APPLY_USER_PROFILE` | Main→Hardware | 프로필 보정·모터 상태 복원 |
| `USER_PROFILE_APPLIED` | Hardware→Main | 실제 적용·도착 결과 |
| `PREPARE_CALIBRATION` | Main→Hardware | IMU 선행 보정 |
| `HARDWARE_CALIBRATION_READY` | Hardware→Main | Vision 보정 시작 허가 |
| `MEASUREMENT_STOP_AND_REST` | Main→Hardware | 네 모터 종료 자세 요청 |
| `MEASUREMENT_STOP_AND_REST_ACK` | Hardware→Main | 실제 도착 결과 |
| `UPDATE_ALARM_SETTINGS` | Main→Hardware | 알림 runtime 설정 반영 |

## 27.6 주요 State를 읽는 법

### Pose State 예시 구조

```text
POSE_STATE
├─ frame_id / timestamp_ns / mode
├─ landmark_valid
├─ landmark_quality / minimum visibility
├─ landmarks
├─ features
├─ eye_gap_px
├─ inference
│  ├─ posture_type
│  ├─ confidence
│  └─ timestamp
└─ calibration
```

### Motor12 진단에서 중요한 값

```text
ready / enabled
mode
current angles
target angles
current FK x/z
target x/z
max error
hold reason
last error
rest/recovery latch
command timestamp/rate
```

### Safety 진단에서 중요한 값

```text
state
reason
tracking_allowed
tof_presence_valid
landmark_presence_valid
missing_elapsed_sec
absence_timeout_sec
return_latched
```

## 27.7 PDF 변환 권장 방법

이 문서는 긴 코드 블록과 표를 포함하므로 한글 폰트를 명시하는 것이 좋다. 예를 들어 Pandoc과 XeLaTeX가 설치되어 있다면 다음 형태를 사용할 수 있다.

```bash
pandoc POCO_MAINPYQT_STUDY_GUIDE.md \
  -o POCO_MAINPYQT_STUDY_GUIDE.pdf \
  --pdf-engine=xelatex \
  -V mainfont="Noto Sans CJK KR" \
  -V monofont="D2Coding" \
  -V geometry:margin=18mm \
  -V fontsize=10pt \
  --toc
```

환경에 해당 폰트가 없다면 설치된 한글 폰트 이름으로 바꾼다. Markdown viewer의 인쇄 기능을 쓸 때는 배율 90~95%, 배경 그래픽 켜기를 권장한다.

## 27.8 마지막 요약

POCO 메인 코드를 이해하는 가장 짧은 문장은 다음과 같다.

> MainWindow가 사용자의 의도를 받고, CameraWorker와 Manager가 비전·하드웨어 프로세스를 조정하며, Pose가 “어떤 자세인가”를 판단하고, ToF가 “사용자가 어디에 있는가”를 측정한다. Hardware Process는 두 정보를 Safety Supervisor로 통과시킨 뒤 IK/PID Controller를 통해 모터 1~4를 움직이고, 모든 최신 상태와 완료 이벤트를 다시 UI로 돌려보낸다.

이 구조에서 기능을 추가할 때는 먼저 데이터가 State인지 Event인지, 어느 Process가 자원의 owner인지, 실제 모터 명령 전에 어떤 안전 gate가 필요한지를 결정하면 된다.

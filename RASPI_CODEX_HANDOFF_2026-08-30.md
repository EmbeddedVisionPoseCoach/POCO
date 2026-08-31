# POCO Raspberry Pi Codex 인수인계 — 2026-08-30

이 문서는 Raspberry Pi에 직접 연결한 새 Codex 세션이 2026-08-30 작업을 이어서 진행할 수 있도록 작성한 인수인계 자료다.

## 1. 가장 먼저 알아야 할 내용

- 현재 기준 코드는 로컬 PC 복사본이 아니라 Raspberry Pi의 다음 작업공간이다.

  ```text
  /home/willtek/POCO
  ```

- 실제 실행 진입점은 다음 파일이다.

  ```text
  /home/willtek/POCO/WorkSpace/pyQt/mainpyQt.py
  ```

- 확인 당시 Git 정보는 다음과 같았다.

  ```text
  branch: Feature_Multiprocessing
  HEAD:   24fda6e Feat : ReadMe 추가
  ```

- 팀원이 먼저 모니터암 코드를 POCO 멀티프로세스 구조에 병합한 상태였고, 오늘은 그 코드 위에 `초기값 준비`용 모니터암 수동 조정 및 ToF/눈 간격 평균 측정 단계를 추가했다.
- 로컬 PC의 `/home/bhc/poco/POCO`는 Raspberry Pi 팀원 병합본보다 오래된 커밋이므로 소스 기준으로 사용하지 말 것.
- 오늘 변경은 Raspberry Pi에 직접 반영했지만 Git 커밋은 하지 않았다.

## 2. 작업 시작 전 반드시 확인할 Git 상태

오늘 최종 확인한 원격 상태는 다음과 같았다.

```text
 M WorkSpace/config/monitor_arm_settings.json
 M WorkSpace/pyQt/camera_worker_profile_all.py
 M WorkSpace/pyQt/mainpyQt.py
 M WorkSpace/pyQt/managers/vision_process_manager_profile.py
 M WorkSpace/pyQt/processes/hardware_process.py
 M WorkSpace/pyQt/processes/pose_process_profile.py
 M WorkSpace/pyQt/services/motor12_controller.py
 M WorkSpace/saved_model/baseline.pkl
?? WorkSpace/pyQt/monitor_arm_preparation_dialog.py
?? WorkSpace/pyQt/services/monitor_arm_calibration_service.py
?? WorkSpace/pyQt/services/monitor_arm_preparation_controller.py
```

주의 사항:

- `WorkSpace/config/monitor_arm_settings.json`과 `WorkSpace/saved_model/baseline.pkl`의 변경은 오늘 작업 전에 이미 존재하던 팀원/사용자 변경이다.
- 위 두 파일을 되돌리거나 덮어쓰지 말 것.
- 나머지 Python 변경 및 신규 파일 3개가 오늘 추가한 초기 준비 기능이다.
- 작업 시작 시 반드시 다시 `git status --short`를 실행하고 다른 팀원의 새로운 변경이 있는지 확인할 것.

## 3. 오늘 구현한 사용자 흐름

메인 UI의 버튼 흐름은 다음과 같다.

```text
초기값 준비
  ↓
별도 PyQt 모니터암 준비 창
  ↓
Motor 1~4 연결 확인
  ↓
휴식자세 → 사용자에게서 가장 먼 작업 시작 위치
  ↓
Motor 1·2 사용자 X/Z 기반 수동 IK 조정
  + Motor 3·4 누르는 동안 ± 조그
  ↓
ToF + MediaPipe Pose 눈 간격 5초 평균 측정/저장
  ↓
준비 완료 후 종료
  ↓
기존 POCO의 초기값 측정시작
  ↓
IMU 보정 + Pose/Face baseline 보정
  ↓
실시간 자세 측정(MEASURING)
```

핵심 정책:

- 초기 준비 창에서 카메라와 MediaPipe Pose를 실행한다.
- Motor 1·2의 일반 ToF/Vision 자동추종은 `MEASURING` 상태에서만 실행한다.
- 초기 준비 중에는 자동추종을 차단하고 명시적인 작업자세 복구 또는 수동 IK 명령만 허용한다.
- Motor 3·4 자동 짐벌 제어도 초기 준비 중에는 실행하지 않는다. 준비 창의 조그 명령만 허용한다.
- 실제 Serial과 I2C는 계속 Hardware Process 하나만 소유한다. PyQt Main/UI에서 직접 하드웨어 포트를 열지 않는다.

## 4. 오늘 추가한 신규 파일

### 4.1 `WorkSpace/pyQt/monitor_arm_preparation_dialog.py`

초기값 준비 단계에서 열리는 별도 PyQt `QDialog`다.

주요 기능:

- Motor 1~4 연결 확인 버튼
- `휴식 → 작업 시작 위치 이동` 버튼
- `작업자세 → 휴식자세 이동` 버튼
- 베이스 기준 사용자 X 게이지
- 사용자–모니터 고정거리와 모니터 Z 입력
- Motor 1·2 현재/목표 IK 표시
- 현재 팔과 목표 팔 X-Z 시각화
- Motor 3·4 누르는 동안 ± 조그
- ToF/눈 간격 실시간 표시
- 5초 평균 측정 진행률 및 결과 표시
- 준비 완료/취소 처리

중요 클래스:

- `ArmPreparationCanvas`: Motor 1·2 현재 자세와 목표 자세를 X-Z 평면에 그린다.
- `MonitorArmPreparationDialog`: UI 구성, Hardware Process 명령 송신, 상태 폴링 및 이벤트 표시를 담당한다.

중요 IPC 명령:

```text
MONITOR_ARM_CONNECT_ALL
MONITOR_ARM_MOVE_WORKING_START
MONITOR_ARM_MOVE_REST
MONITOR_ARM_MANUAL_IK_TARGET
MONITOR_ARM_GIMBAL_JOG
MONITOR_ARM_GIMBAL_JOG_STOP
START_MONITOR_ARM_SENSOR_CAPTURE
FINISH_MONITOR_ARM_PREPARATION
CANCEL_MONITOR_ARM_PREPARATION
```

### 4.2 `WorkSpace/pyQt/services/monitor_arm_preparation_controller.py`

Hardware Process 안에서 초기 준비 단계의 Motor 1~4 명령을 담당한다.

주요 기능:

- 공용 `MotorService`를 사용해 Motor 1~4 연결 상태 확인
- 휴식자세에서 작업 시작 위치로 Motor 1·2 복구
- 사용자 X, 사용자–모니터 거리, 모니터 Z를 이용한 수동 IK
- Motor 3·4 조그 목표 관리
- 현재 Motor 각도와 Forward Kinematics 결과 제공
- UI 시각화용 준비 상태 snapshot 생성

작업 시작 위치 계산:

```text
user_x = manual_cartesian.user_x_min_m
monitor_x = user_x - desired_user_monitor_distance_m
monitor_z = manual_cartesian.default_monitor_z_m
```

확인 당시 목표는 대략 다음과 같았다.

```text
monitor X ≈ 0.1007655 m
monitor Z ≈ 0.2560723 m
shoulder ≈ +70.21°
elbow    ≈ -56.56°
```

Motor 3 + 조그 수정:

- 이전 방식은 매 조그 tick마다 느린 실제각을 다시 읽어 `현재각 + delta`를 계산했다.
- 모터가 목표를 따라오기 전에 다음 tick이 오면 같은 목표를 반복 전송할 수 있었다.
- 이제 `gimbal_jog_targets`에 누적 목표각을 따로 저장한다.
- 버튼을 놓으면 `MONITOR_ARM_GIMBAL_JOG_STOP`으로 실제각을 다시 읽어 다음 조그의 기준으로 사용한다.
- 안전 상한/하한에 도달하면 오류 메시지로 범위를 표시한다.

### 4.3 `WorkSpace/pyQt/services/monitor_arm_calibration_service.py`

초기 준비 단계의 ToF와 눈 간격을 5초 동안 평균 내고 JSON으로 저장한다.

저장 위치:

```text
/home/willtek/POCO/WorkSpace/data/settings/monitor_arm_user_calibration.json
```

주요 클래스:

- `MonitorArmPreparationCalibrationService`
  - 중복 ToF timestamp 제외
  - 중복 Pose frame ID 제외
  - 유효 샘플만 평균
  - ToF 최소 샘플과 눈 간격 최소 샘플 검사
  - 임시 파일 작성 후 `os.replace()`를 사용하는 원자적 JSON 저장
- `CalibratedMonitorArmFusion`
  - 저장된 평균을 이용한 ToF 0.7 + Vision 0.3 융합용 보조 클래스

현재 Hardware Process에서는 평균 측정 완료 시 기존 `EyeGapVisionDistanceEstimator.calibrate()`에도 평균 눈 간격과 기준 사용자–모니터 거리를 적용한다.

## 5. 오늘 수정한 기존 파일

### 5.1 `WorkSpace/pyQt/mainpyQt.py`

- `MonitorArmPreparationDialog` import 추가
- `monitor_arm_preparation_dialog`와 `monitor_arm_preparation_ready` 상태 추가
- `초기값 준비` 버튼이 일반 프리뷰만 시작하지 않고 준비 창을 열도록 수정
- 준비 완료 후에만 `초기값 측정시작` 버튼 활성화
- Hardware 이벤트를 열린 준비 창으로 전달
- 카메라 종료/앱 종료 시 준비 창과 Hardware 준비 상태 정리

### 5.2 `WorkSpace/pyQt/camera_worker_profile_all.py`

- `RunMode.PREPARING` 추가
- `start_monitor_arm_preparation()` 추가
- `finish_monitor_arm_preparation()` 추가
- 카메라/Pose Process 준비 전 요청을 위한 `pending_preparation_start` 추가
- 현재 실행 세션에 ToF/눈 간격 평균값이 없으면 실시간 자세 측정 시작을 차단

### 5.3 `WorkSpace/pyQt/managers/vision_process_manager_profile.py`

- `start_monitor_arm_preparation()` 추가
  - `accept_frames=True`
  - Pose Process에 `START_PREPARATION` 전달
  - Face Process는 준비 단계에서 정지
  - Hardware Process에 Main state `MONITOR_ARM_PREPARATION` 전달
- `finish_monitor_arm_preparation()` 추가
  - Pose 준비 모드를 종료하고 일반 `PREVIEW`로 복귀

### 5.4 `WorkSpace/pyQt/processes/pose_process_profile.py`

- `MODE_PREPARING` 추가
- `START_PREPARATION` 명령 처리 추가
- MediaPipe Pose landmark 2번과 5번으로 양쪽 눈 사이 pixel 거리 계산
- Pose state에 다음 필드 추가

  ```python
  "eye_gap_valid": bool
  "eye_gap_px": float | None
  ```

눈 간격을 못 받았던 기존 원인:

- 기존 `start_preview()`가 `vision_manager.stop_analysis()`를 호출했다.
- 이때 `accept_frames=False`, Pose mode `IDLE`이 되어 준비 화면에서는 MediaPipe Pose가 새 프레임을 처리하지 않았다.
- 따라서 Hardware Process가 같은 오래된 Pose state만 받거나 눈 샘플을 전혀 받지 못했다.
- 준비 전용 Pose 모드와 프레임 공급 경로를 추가해 해결했다.

### 5.5 `WorkSpace/pyQt/processes/hardware_process.py`

- 준비 Controller와 5초 평균 Service 생성
- 위의 준비용 Main→Hardware IPC 명령 처리
- 준비 중 ToF와 Pose state의 고유 샘플 수집
- 평균 완료 시 JSON 저장 및 Vision 기준값 적용
- Hardware state에 다음 구조 추가

  ```python
  state["monitor_arm"] = {
      "calibration": {...},
      "preparation": {...},
      "live_eye_gap_px": ...,
      "live_tof_user_x_m": ...,
      "fusion_weights": {"tof": 0.7, "vision": 0.3},
      "control_prerequisites_ready": ...,
  }
  ```

- 준비 중 Motor 1·2 일반 자동추종 차단
- 일반 자동추종은 Main mode가 `MEASURING`일 때만 활성화
- 준비 중 작업자세 Recovery에는 ToF가 일시적으로 invalid여도 안전한 dummy user X를 넣어 Recovery 자체는 계속 가능하게 함

### 5.6 `WorkSpace/pyQt/services/motor12_controller.py`

- `move_manual_user_target()` 추가
- 사용자 X, 고정거리, Z로 목표 모니터 자세 계산
- IK 및 경로/관절/수직거리 안전 검사
- Motor 1·2 SyncWrite 전송
- 휴식/Recovery는 기존 팀원 구현을 그대로 활용

## 6. 현재 남아 있는 문제 — 가장 먼저 이어서 수정할 것

사용자 확인 결과:

```text
휴식 → 작업 시작 위치 이동은 실제로 잘 이루어진다.
하지만 목표 근처에 도착한 뒤에도 UI가 계속
"작업 시작/IK 목표로 이동 중입니다."라고 표시한다.
```

코드상 가장 가능성 높은 원인은 도착 판정 허용 오차가 너무 엄격한 것이다.

관련 위치:

1. `services/monitor_arm_planner.py`
   - `MonitorArmPlanner._plan_working_pose_recovery()`
   - 현재 완료 기준이 다음처럼 하드코딩되어 있다.

     ```python
     if largest_joint_change <= 0.25:
         self.recovery_active = False
         return None
     ```

2. `services/monitor_arm_preparation_controller.py`
   - `MonitorArmPreparationController.update()`
   - `motor12.planner.recovery_active`가 `False`가 되어야 `working_start_completed=True`가 된다.

3. 같은 Controller의 `snapshot()`
   - `recovery_active` 또는 `target_reason`이 남아 있으면 `movement_active=True`다.

4. `monitor_arm_preparation_dialog.py`
   - `movement_active=True`이면 계속 “이동 중” 상태를 표시한다.

실제 모터에는 하중, 기어 백래시, 정지 오차가 있으므로 목표 근처에서 0.3~1° 정도가 남을 수 있다. 사람 눈에는 도착했지만 0.25° 조건을 만족하지 못해 동일 목표를 계속 전송할 가능성이 높다.

추가 문제:

- Motor12 Recovery가 안전 검사 오류 또는 명령 실패로 중단되더라도 `MonitorArmPreparationController`가 `motor12.last_error`와 `hold_reason`을 UI 상태로 전달하지 않는다.
- 이 경우 실제 팔은 멈췄지만 준비 UI는 계속 “이동 중”이라고만 보일 수 있다.

권장 수정안:

1. 설정 파일에 다음과 같은 값을 추가한다.

   ```json
   "working_start_arrival_tolerance_deg": 1.0,
   "working_start_stable_samples": 3,
   "working_start_timeout_sec": 25.0
   ```

2. 단 한 프레임에서 오차 범위에 들어왔다고 끝내지 말고, Motor 1·2 모두 허용 오차 안에 연속 3회 들어왔을 때 완료한다.
3. 허용 오차 안에 들어오면 `recovery_active=False`, `working_start_completed=True`, `target_reason=None`으로 만들고 반복 Goal 전송을 멈춘다.
4. Motor12의 `hold_reason`, `last_error`, 현재–목표 최대 오차를 preparation snapshot에 추가한다.
5. UI에 다음 상태를 구분해 표시한다.
   - 이동 중
   - 목표 도착 및 안정화 완료
   - Recovery 안전 검사 중단
   - 명령 실패
   - 시간 초과
6. 시간 초과만으로 도착 성공 처리하지 말 것. 목표와 충분히 가깝지 않으면 오류로 정지해야 한다.

수정 전에 먼저 UI의 현재 S/E와 목표 S/E 차이를 확인할 것.

- 최대 차이가 1° 이내이면 거의 확실히 0.25° 도착 판정 문제다.
- 여러 도가 남은 채 멈췄다면 `motor12.last_error` 또는 `hold_reason`을 먼저 확인해야 한다.

## 7. ToF 설정 주의

오늘 최종 확인 당시 Raspberry Pi 설정은 다음과 같았다.

```json
"tof": {
  "mode": "fixed_stub"
}
```

이는 오늘 작업 전에 팀원이 `hardware`에서 `fixed_stub`으로 바꿔둔 미커밋 변경이어서 의도적으로 덮어쓰지 않았다.

따라서 현재 메인 프로그램의 ToF 값은 실제 HW-843 측정값이 아니라 `fixed_range_m` 고정값이다. 실물 ToF로 최종 시험할 때는 사용자의 확인을 받고 다음으로 변경해야 한다.

```json
"mode": "hardware"
```

설정 파일:

```text
/home/willtek/POCO/WorkSpace/config/monitor_arm_settings.json
```

하드웨어 설정:

```text
I2C bus: 3
SDA: GPIO22, physical pin 15
SCL: GPIO23, physical pin 16
Address: 0x29
```

`fixed_stub`은 UI/IPC/IK 흐름 시험에는 쓸 수 있지만 실제 ToF 평균 시험 결과로 간주하면 안 된다.

## 8. 테스트 방법

### 8.1 실행

```bash
cd /home/willtek/POCO
source .venv/bin/activate
cd WorkSpace/pyQt
python3 mainpyQt.py
```

### 8.2 준비 단계 시험

1. 메인 UI에서 `초기값 준비` 클릭
2. 준비 창의 실시간 눈 간격이 `--`가 아닌 px 값인지 확인
3. 실제 ToF 모드라면 ToF 사용자 X가 실시간으로 변하는지 확인
4. `모터 1~4 연결 확인` 클릭
5. Motor 1~4 READY 확인
6. 팔과 모니터를 지지하고 주변 충돌물 제거
7. `휴식 → 작업 시작 위치 이동` 클릭
8. 현재/목표 S/E 및 X/Z 확인
9. 사용자 X 게이지를 천천히 이동하여 Motor 1·2 실시간 IK 확인
10. Motor 3과 Motor 4의 −/+를 짧게 눌러 양방향 확인
11. `작업자세 → 휴식자세 이동` 확인
12. 다시 `휴식 → 작업 시작 위치 이동`
13. 자세 조정 후 `5초 평균 측정 시작`
14. 실제 ToF 20 Hz에서는 약 100개의 ToF 샘플 기대
15. 눈 간격 샘플은 최소 30개 필요
16. 평균 저장 완료 후 `준비 완료 후 종료`
17. 메인 UI에서 `초기값 측정시작` 클릭
18. 기존 IMU 및 Pose/Face baseline 보정 완료
19. 자세 측정 시작 후에만 Motor 1·2 자동추종이 실행되는지 확인

## 9. 오늘 실행해서 통과한 테스트

Raspberry Pi의 `/home/willtek/POCO/.venv`에서 다음을 확인했다.

- `python -m compileall -q WorkSpace/pyQt`: 통과
- PyQt `mainpyQt` 및 준비 Dialog import: 통과
- Offscreen `MainWindow` 생성/종료: 통과
- MediaPipe Pose 눈 landmark 2/5 pixel 거리 계산 Fake 테스트: 통과
- ToF/눈 간격 고유 샘플 평균 및 JSON 저장 Fake 테스트: 통과
- Motor 3이 실제각을 따라오지 않는 상황에서도 + 조그 목표가 `+0.5 → +1.0°`로 누적되는 Fake 테스트: 통과
- 가장 먼 작업 시작 위치 IK가 `monitor X ≈ 0.1007655 m`인지 확인: 통과
- `hardware_logic_selftest.py`: 통과

마지막 self-test 출력:

```text
Hardware logic self-test: PASS (IMU / ToF-Vision / Motor1~4 / Rest-Recovery)
```

아직 하지 않은 것:

- 실제 Motor 1~4 전체 물리 동작 자동 테스트
- 실제 HW-843을 `hardware` 모드로 둔 상태의 5초 평균 저장 시험
- 실제 하중 상태에서 도착 허용 오차 결정

## 10. 안전 관련 주의

- 휴식자세는 일반 Calibration safe range 밖의 확인된 특수 자세다.
- 휴식 이동은 기존 코드의 제한인 Speed 200 이하, Acc 10 이하를 사용한다.
- Motor 1·2 작업자세 복구는 안전범위 안쪽으로 향하는 Recovery만 허용한다.
- Motor 3·4 조그 목표는 `servo_calibration_result.json` 안전각 범위로 clamp한다.
- 물리 시험 전에 모니터를 손으로 지지하고 이동 경로에서 사람과 장애물을 제거한다.
- 오류가 난 상태에서 범위를 넓히거나 special move 검사를 우회하지 말 것.
- Motor 3 +가 계속 안 움직이면 먼저 UI 목표각이 증가하는지와 safe max 도달 메시지를 확인한다. 코드 목표가 증가하는데 실물이 움직이지 않을 때만 배선, 토크, calibration direction을 조사한다.

## 11. 복구 백업

오늘 교체한 기존 Python 파일 6개의 적용 전 백업이 Raspberry Pi에 있다.

```text
/tmp/poco_monitor_arm_preparation_backup_20260830.tar
```

백업 대상:

```text
WorkSpace/pyQt/mainpyQt.py
WorkSpace/pyQt/camera_worker_profile_all.py
WorkSpace/pyQt/managers/vision_process_manager_profile.py
WorkSpace/pyQt/processes/pose_process_profile.py
WorkSpace/pyQt/processes/hardware_process.py
WorkSpace/pyQt/services/motor12_controller.py
```

신규 파일 3개는 백업 tar에 포함되지 않는다. 롤백이 필요하면 기존 파일 복원과 신규 파일 제거를 별도로 검토해야 한다. 사용자의 명시적 요청 없이 롤백하거나 파일을 삭제하지 말 것.

## 12. 다음 Codex 세션에 바로 전달할 프롬프트

아래 내용을 Raspberry Pi의 Codex에 그대로 전달하면 된다.

```text
/home/willtek/POCO 작업공간의 RASPI_CODEX_HANDOFF_2026-08-30.md를 먼저 끝까지 읽어줘.
현재 기준은 Raspberry Pi의 Feature_Multiprocessing 브랜치이며, 사용자/팀원의 미커밋 변경을 보존해야 해.

우선 git status와 관련 파일을 다시 확인한 뒤, 휴식 → 작업 시작 위치 이동이 실제로 거의 완료됐는데도 준비 UI가 계속 “작업 시작/IK 목표로 이동 중입니다”라고 표시하는 문제를 진단해줘.

현재/목표 Motor1·2 각도 오차, MonitorArmPlanner의 0.25° 완료 조건, motor12.last_error, hold_reason, planner.recovery_active를 확인해. 단순히 timeout으로 성공 처리하지 말고, 설정 가능한 도착 허용 오차 + 연속 안정 샘플 + 오류 전파 방식으로 수정안을 제안한 뒤 구현해줘.

실제 모터를 움직이는 테스트는 먼저 나에게 알리고, 초기에는 컴파일/Fake/self-test부터 수행해줘. WorkSpace/config/monitor_arm_settings.json과 WorkSpace/saved_model/baseline.pkl의 기존 변경은 덮어쓰거나 되돌리지 마.
```

## 13. 문서를 Raspberry Pi 작업공간에 둘 위치

이 파일을 Raspberry Pi로 복사할 때 권장 위치는 다음이다.

```text
/home/willtek/POCO/RASPI_CODEX_HANDOFF_2026-08-30.md
```

새 Codex 세션은 이 문서를 읽은 뒤 반드시 실제 현재 파일과 Git 상태를 다시 확인해야 한다. 이 문서는 작업 당시 상태를 기록한 것이며, 이후 다른 팀원이 수정했다면 Raspberry Pi의 최신 코드가 우선이다.

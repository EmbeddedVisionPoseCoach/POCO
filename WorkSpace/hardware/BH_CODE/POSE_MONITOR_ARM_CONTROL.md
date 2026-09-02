# ToF 사용자 X + Pose 기반 Servo 1·2 모니터암 제어

코드 전체 흐름과 클래스/함수를 공부하려면
[`MONITOR_ARM_CODE_STUDY_GUIDE.md`](MONITOR_ARM_CODE_STUDY_GUIDE.md)를 먼저 읽는다.

이 구현은 기존 4축 `monitor_arm_ui.py`와 별개입니다. 자동 IK와 실제 위치 명령에는
다음 두 관절만 존재합니다.

| Servo ID | Joint |
|---|---|
| 1 | `shoulder_lift` |
| 2 | `elbow_flex` |

Servo 3 `wrist_flex`, Servo 4 `wrist_roll`은 계산과 명령 대상에 포함되지 않습니다.

## 파일

- `pose_monitor_arm_controller.py`: 고정 ToF 모의값 → 사용자 X → 2축 IK → 모터, 카메라 Pose 병행
- `monitor_arm_motor_process.py`: 관절각 메시지를 받아 Servo 1·2 시리얼 제어를 전담하는 별도 프로세스
- `monitor_arm_visualizer.py`: 자동 제어의 현재/목표 자세를 보여주는 실시간 Tkinter X-Z 창
- `monitor_arm_kinematics.py`: 2축 정/역기구학과 수직 이동경로 방어
- `manual_motor12_limit_ui.py`: Servo 1·2의 누르는 동안만 움직이는 `−/+` 조그 UI
- `manual_vertical_ik_ui.py`: 사용자 X 게이지·고정거리·고정높이 기반 2축 IK UI
- `monitor_arm_settings.json`: 7 cm 오프셋, 거리, 속도, soft limit, 수직 임계값
- `tasks/pose_landmarker_heavy.task`: VisionPoseCoach에서 복사한 Pose 모델

## ToF 사용자 X와 목표 모니터 X

카메라 눈 간격은 더 이상 거리 계산에 사용하지 않습니다. 베이스 원점에 붙고 +X 방향으로
명치 부근을 향하는 ToF의 측정거리를 사용자 X로 사용합니다. 현재는 실제 센서 대신
`monitor_arm_settings.json`의 고정 모의값을 읽습니다.

```text
user_x = tof_sensor_origin_x + tof_range
target_monitor_x = user_x - desired_user_monitor_distance
```

기본값은 사용자 X 약 73.28cm, 유지거리 50cm이므로 모니터 목표 X는 약 23.28cm입니다.
실제 ToF가 연결되면 `FixedToFUserXSource.read_range_m()`만 하드웨어 읽기로 교체하고
나머지 좌표 변환과 IK는 그대로 사용합니다.

Pose Landmarker는 계속 카메라 프레임을 처리하지만 거리값을 만들지 않습니다. 추후
거북목처럼 얼굴만 앞으로 나오고 몸통은 그대로인 나쁜 자세를 검출하면 ToF 기반 자동
이동을 차단하는 posture gate로 사용합니다. 현재 버전에는 자세 차단 판정 자체는 아직
포함되지 않았습니다.

## 수직 베이스 링크

베이스 원점에서 shoulder 회전축까지의 고정 링크는 +Z 방향으로 수직입니다. 기존 링크
길이 약 13.56cm는 보존했습니다.

```text
base = (0, 0)
shoulder = (0, 0.1356059585m)
```

이 변경으로 같은 관절각의 모니터 X가 약 6.92cm 감소하고 Z는 약 1.90cm 증가합니다.

## 메인 계산과 모터 제어 프로세스 분리

`--enable-motor`로 실행하면 메인 프로세스가 시리얼 포트를 직접 열지 않습니다.
데이터 흐름은 다음과 같습니다.

```text
[메인 프로세스]
ToF range → 사용자 X → 목표 monitor X
카메라 → MediaPipe → 향후 자세 허용/차단 gate
                         ↓ IK/안전검사
                shoulder_lift_deg
                elbow_flex_deg
                         ↓ Queue
[모터 제어 프로세스]
영점/한계 변환 → Servo 1·2 명령 → 현재 각도 응답
```

메인에서 모터 프로세스로 보내는 실질적인 메시지 계약은 다음 두 각도입니다.

```text
shoulder_lift_deg  # Servo 1 팀원용 명령각
elbow_flex_deg     # Servo 2 팀원용 명령각
```

`servo_calibration_result.json` 영점을 raw position으로 변환하는 작업과 실제
`motor_control` 호출은 모두 `monitor_arm_motor_process.py`에만 있습니다. 포코에 이식할
때는 메인의 IK 결과 두 개와 하드웨어 프로세스의 현재 각도 응답을 해당 Queue/
IPC 형식에 맞게 교체하면 됩니다.

가변속도도 이 분리를 유지합니다. 메인 프로세스는 IK로 계산한 Servo 1·2 목표각만
보내고, 모터 프로세스가 실제 현재각을 다시 읽어 두 관절 중 큰 오차를 기준으로 이번
명령의 Speed를 계산합니다. 따라서 카메라/AI/IK 코드는 STServo Speed를 계산하거나
시리얼 포트를 직접 사용하지 않습니다.

## 통합 제어 속도 모드

### 2026-08-28 명령 방식 변경 기록

정상 비전 추종에서 관절 목표를 5°씩 분할하던 기존 방식을 선택 가능한 상태로
보존하고, 기본 방식을 최종 IK 각도 직접 명령으로 변경했습니다. 실행 화면의 모드
표시에서 `DIRECT IK TARGET` 또는 `STEPPED IK (5 DEG)`를 확인할 수 있습니다.

통합 Pose 제어는 `monitor_arm_settings.json`의 다음 전용 설정을 사용합니다.

```text
pose_speed                         # 고정 모드 Speed / 가변 모드 최대 Speed
pose_acc
pose_speed_mode                    # fixed 또는 adaptive
pose_joint_command_mode            # direct(최종 IK 각도) 또는 stepped(기존 5° 분할)
pose_variable_min_speed
pose_variable_full_speed_error_deg
```

`adaptive`에서는 현재각과 받은 목표각의 최대 차이가 클수록 `pose_speed`에 가까워지고,
목표 근처에서는 `pose_variable_min_speed`까지 낮아집니다. 어떤 설정에서도 코드의 절대
상한은 1000입니다.

현재 직접 목표 모드 설정은 `pose_speed=800`, `pose_acc=20`,
`pose_variable_min_speed=150`, `pose_variable_full_speed_error_deg=30`입니다. 예를 들어
관절 오차가 약 10°라면 Speed는 약 367, 15°라면 약 475가 되어 5°만
넘어도 즉시 Speed 1000이 되지 않습니다. 이 값은 실물 하중과 전원 상태에 맞춰 낮은
값부터 조정해야 합니다.

정상 ToF 추종의 기본값인 `direct`는 5°씩 잘라 여러 번 보내지 않고, 이번 주기에
계산된 최종 IK 각도를 모터 프로세스로 한 번에 보냅니다. 모터 프로세스는 현재 실제
각도와 최종 목표의 차이로 adaptive Speed를 정합니다. 명령만 최종 각도로 바뀌는 것이며,
soft/hard 관절 한계와 현재 자세부터 최종 자세까지의 전체 보간 경로 Z 검사는 그대로
수행합니다. ToF 사용자 X로 계산한 최종 monitor X와 현재 X의 차이는 한 제어 주기에
`max_monitor_x_step_m`(기본 2cm)까지만 반영합니다.

결과가 좋지 않으면 아래 한 줄만 바꿔 이전 5° 분할 동작으로 되돌릴 수 있습니다.

```json
"pose_joint_command_mode": "stepped"
```

`stepped`에서는 `safety.max_joint_step_deg`(현재 5°)만큼 나눠 전송합니다. 휴식자세에서
작업자세로 복귀하는 과정은 설정과 무관하게 계속 5°씩 진행합니다. 휴식자세가 보정
안전범위 밖에 있을 수 있어, 이 구간에는 별도의 단계별 안쪽 방향 검사가 필요하기
때문입니다.

## 7 cm 모니터 오프셋

`monitor_arm_settings.json`의 `geometry.monitor_offset_m`은 `0.07`입니다. 코드에서는
두 번째 링크 끝 모터 중심에서 두 번째 링크 진행 방향으로 모니터 중심이 7 cm 더
나간 것으로 계산합니다. 실제 장착 방향이 다르면 이 값과 모델을 실측에 맞게 수정해야
합니다.

## 수직거리 방어

자동 제어 시작 시 현재 모니터 중심의 베이스 기준 Z를 기준 높이로 저장합니다.
목표점뿐 아니라 현재 관절각부터 목표 관절각까지의 보간 경로를 `path_samples`만큼
검사합니다. 어느 지점이든 `vertical_tolerance_m`보다 높이 차이가 커지면 새 명령을
보내지 않고 기존 자세를 유지합니다.

수동 조그 UI에서 다음 값을 측정·저장할 수 있습니다.

- Servo 1·2 soft min/max
- 자동 제어 speed와 acc

단, 시작 자세가 휴식자세처럼 작업 Z 범위 밖이면 그 낮은 높이를 기준 Z로 저장하지
않습니다. `manual_cartesian.default_monitor_z_m`을 작업 기준 높이로 사용하고 먼저
설정된 작업자세(기본 S=0°, E=0°)까지 최대 5°씩 복귀합니다. calibration/soft 범위 밖
구간에서는 매 스텝이 반드시 범위 안쪽으로 향할 때만 예외적으로 전송됩니다.

## 실행 순서

먼저 자동 제어와 다른 시리얼 프로그램을 모두 종료한 상태에서 수동 시험을 합니다.

```bash
python manual_motor12_limit_ui.py
```

각 관절의 `−` 또는 `+` 버튼을 누르고 있는 동안에만 목표 각도가 `조그 속도(°/s)`만큼
연속 갱신됩니다. 버튼을 놓거나 마우스가 버튼 밖으로 나가면 해당 모터의 현재 위치로
Hold 명령을 한 번 보낸 후 새 위치 패킷 전송을 멈춥니다. 별도의 목표값 입력이나 전송
버튼은 없습니다.

관절을 각각 움직이지 않고 사용자 위치를 기준으로 IK 제어하려면 다음 UI를 사용합니다.

```bash
python manual_vertical_ik_ui.py
```

좌표계는 클램프와 로봇팔 베이스의 연결점을 원점 `(0, 0)`으로 사용합니다.

```text
+X = 베이스에서 사용자 방향
+Z = 원점에서 수직 위 방향
```

주 게이지 값은 실제 센서 연결 전의 `ToF 사용자 X 모의값`입니다. 모니터 좌표는 다음
식으로 계산됩니다.

```text
monitor_x = user_x - user_monitor_distance
monitor_z = 입력한 고정 높이
```

따라서 사용자 X를 바꾸더라도 사용자와 모니터 사이 X 거리는 설정값으로 유지되고,
모니터 높이 Z도 입력값으로 유지됩니다. 현재 팔은 회색, 목표 팔은 파랑, 목표 모니터는
주황, 사용자는 초록색으로 표시됩니다. 노란 점선은 유지되는 사용자-모니터 거리입니다.

기본 수동 좌표 한계는 보수적으로 다음과 같이 시작합니다.

```text
사용자 X: 약 60.08~83.08cm
모니터 Z: 20~30cm
사용자-모니터 거리: 50cm
```

UI에서 X/Z 최소·최대값을 바꾸고 `좌표 한계 저장 및 적용`을 누르면
`monitor_arm_settings.json`에 저장됩니다. 이 버튼은 모터 명령을 보내지 않습니다. 직사각형
범위 안이어도 일부 X/Z 조합은 실제 2축 도달범위나 joint soft/hard limit 때문에
차단될 수 있습니다.

포트 연결 후 `사용자 X` 게이지를 움직이면 모터 1·2가 실시간으로 변경된 IK
목표를 추종합니다. 전송 주기는 `control.command_hz`(기본 5 Hz)이고, 한 주기의
최대 관절 변화는 `max_joint_step_deg`로 제한됩니다. 게이지를 놓은 뒤에도 최종
목표에 도달할 때까지만 추종하고, 도달하면 새 위치 패킷 전송을 멈춥니다.

`사용자-모니터 X 거리`와 `모니터 Z`를 키보드나 화살표로 바꿔도 그 즉시 IK나
모터에 반영되지 않습니다. `일정값 적용 및 현재 게이지 위치로 이동`을 눌러야
적용 중인 값이 바뀌고 현재 X 게이지 목표로 제어됩니다.
실시간 X/IK 추종 경로는 `vertical_tolerance_m`과 설정한 Z 범위로 검사됩니다.

`ID 1 원점`, `ID 2 원점`, `ID 1·2 동시 원점` 버튼은 각 관절을
`servo_calibration_result.json` 기준 명령각 0°로 복귀시킵니다. 원점 복귀를 누르면
X 게이지 실시간 추종은 중지됩니다. 원점 복귀는 모니터 고정 Z를 유지하는
IK 명령이 아니므로, 반드시 팔과 모니터를 지지하고 충돌 가능성을 확인해야 합니다.

`servo_calibration_result.json`의 Servo 1·2 `max_speed`는 현재 `null`입니다. 수동
시험에서 결정한 안전 속도를 캘리브레이션 절차로 기록하기 전에는 자동 모터 모드가
의도적으로 시작되지 않습니다.

고정 ToF 모의값, Pose 처리와 IK를 시험합니다. 이 명령은 모터 포트를 열지 않습니다.

```bash
python pose_monitor_arm_controller.py
```

실행하면 OpenCV 카메라 창과 Tkinter `ToF 사용자 X 기반 IK` 창이 같이
열립니다. 시각화 창에서 회색은 현재/명령 전 자세, 파랑은 최신 IK 목표,
청록 점선은 자동 제어 시작 시 저장한 기준 Z입니다. 초록 점은 ToF 사용자 X이고,
노란 점선은 `ToF 사용자 X - 현재 모니터 X` 거리입니다.

센서 모의 사용자 X를 임시로 바꿔 시험할 수 있습니다.

```bash
python pose_monitor_arm_controller.py --tof-user-x-m 0.78
```

IK 시각화가 필요 없는 헤드리스 실행에서는 다음 옵션을 추가합니다.

```bash
python pose_monitor_arm_controller.py --no-ik-visualizer
```

실제 Servo 1·2를 구동합니다.

```bash
python pose_monitor_arm_controller.py --enable-motor
```

현재처럼 Servo 1·2의 calibration `max_speed`가 `null`이면 위 명령은 안전상 시작을
차단합니다. 이미 팔을 지지하고 수동 시험한 상태에서만 다음 명시적 시험 옵션을
추가할 수 있습니다.

```bash
python pose_monitor_arm_controller.py \
  --enable-motor \
  --allow-uncalibrated-speed
```

이 옵션도 Speed 절대 상한 1000, Acc 상한 30, 관절 한계 및 IK 전체 경로 검사를
우회하지 않습니다. 정상 `direct` 추종에는 5° 분할을 적용하지 않고, 휴식자세 복귀에는
5° 안전 스텝을 유지합니다. 최종 운영 시에는 옵션을 제거하고 calibration JSON의
`max_speed`를 모터별 실측 안전값으로 기록해야 합니다.

카메라 화면에서 `q`는 종료입니다. 눈 간격 거리 보정이 없어졌으므로 `r` 기준 재측정
기능도 제거했습니다. Tkinter IK 창을 닫아도 카메라와 모터 제어가 같이 종료됩니다.

Tkinter IK 창의 `휴식자세 요청` 버튼은 확인창을 거쳐 자동 거리 제어를 일시정지하고
S=+107.75°, E=-92.55°로 이동합니다. 휴식 이동 자체는 Speed≤200, Acc≤10으로
제한됩니다. OpenCV 창에서는 `h`를 5초 안에 두 번 눌러 같은 요청을 할 수 있습니다.
휴식 후 `a`를 누르면 작업자세 복귀부터 ToF 사용자 X 자동제어를 재개합니다.

## 테스트

```bash
python -m unittest -v test_pose_monitor_arm_controller.py
```

실제 모터 실행 전에는 `monitor_arm_settings.json`의 soft limit, 수직 편차, speed 및
`servo_calibration_result.json`의 `max_speed`를 실측값으로 확정해야 합니다.

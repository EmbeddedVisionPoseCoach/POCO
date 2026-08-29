# motor_control Stop 기능 정리

이 패키지에는 목적이 다른 두 종류의 정지 기능이 있습니다.

## 1. 기존 `emergency_stop()` — 개발/점검용 Torque OFF

구현 위치:

```text
controller.py → MotorController.emergency_stop() → 194행
servo_driver.py → ServoDriver.disable_torque_all_sync() → 201행
```

```python
arm.emergency_stop()
```

- 기존 팀원 호환성을 위해 이름/동작 유지
- `_emergency_event` ON
- 4축 Torque Enable = 0 SyncWrite
- 이후 모든 이동 함수 차단
- Torque가 없어져 로봇팔이 중력으로 떨어질 수 있음
- 현재 자동 reset/Torque ON API 없음

---

## 2. 새 `user_stop()` — 사용자용 Torque 유지 정지

구현 위치:

```text
controller.py → MotorController.user_stop() → 259행
servo_driver.py → ServoDriver.read_positions() → 260행
servo_driver.py → ServoDriver.sync_write_positions() → 156행
```

```python
arm.user_stop()
```

동작:

```text
_user_stop_event ON
→ 새 이동 요청 즉시 차단
→ _command_lock 획득
→ _user_stop_event 다시 ON 확정 (동시 resume 경합 방어)
→ ID1~ID4 Present Position만 읽기
→ 각 축 현재 Position을 각 축 Goal로 구성
→ 4축 Goal을 한 번의 SyncWrite로 전송
→ Torque는 변경하지 않음
→ 4축 command generation 갱신
```

User Stop 중 차단:

```text
move_joint()
move_joint_relative()
move_joints()
move_to_zero()
move_all_to_zero()
```

상태 읽기는 계속 허용됩니다.

```text
get_joint_angle()
get_joint_state()
get_all_states()
is_moving()
```

---

## 3. `resume_user_stop()` — latch만 해제

구현 위치:

```text
controller.py → MotorController.resume_user_stop() → 356행
```

```python
arm.resume_user_stop()
```

이 함수는 `_user_stop_event`만 clear합니다.

```text
Torque 변경 ❌
Zero 이동 ❌
이전 Goal 복원 ❌
새 Goal 전송 ❌
```

실제 모터는 이후 별도의 이동 명령이 들어왔을 때만 다시 움직입니다.

상태 확인:

```python
arm.is_user_stopped()
```

구현 위치:

```text
controller.py → MotorController.is_user_stopped() → 152행
```

---

## 4. 이동 차단 공통 위치

```text
controller.py → MotorController._check_motion_allowed() → 134행
```

기존 Emergency latch와 새 User Stop latch를 모두 검사합니다.

실제 Goal Write 직전에도 다시 확인하여 검증 중 Stop이 들어온 경우 새 이동 명령을 차단합니다.

---

## 5. wait=True 재호출 처리

```text
controller.py → _bump_command_generations() → 160행
controller.py → _generations_are_current() → 177행
controller.py → _wait_for_targets() → 764행
```

Servo별 command generation을 사용합니다.

이전 `wait=True` 명령이 대기 중일 때 다른 Thread가 같은 Servo에 새로운 Goal을 전송하면 기존 wait는 새 generation을 감지해 `False`로 종료합니다.

---

## 6. 실물 검증 관련 주의

Torque ON 상태에서 현재 Position을 Goal로 덮어쓰는 정지 원리는 `wrist_flex` 단축 실물 테스트에서 성공했습니다.

- test speed = 80
- acc = 10
- Torque Enable 1 유지
- Stop 후 Moving 0
- 자세 유지 확인
- 관측 최대 추가 이동 약 0.44°

현재 패키지는 이를 4축 SyncWrite로 확장했습니다. 최종 사용자 안전 기능 승인 전에는 4축 동시 이동/부하 조건에서 추가 실물 검증이 필요합니다.

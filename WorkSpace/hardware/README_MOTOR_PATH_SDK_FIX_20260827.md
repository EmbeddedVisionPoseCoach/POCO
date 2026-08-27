# Motor3 Path / STServo SDK Fix (2026-08-27)

## 발견된 실제 문제

현재 Workspace 구조는 다음과 같다.

```text
WorkSpace/
├─ hardware/
│  ├─ motor_control/
│  └─ servo_calibration_result.json
└─ pyQt/
```

기존 `pyQt/services/motor_service.py`는 `WorkSpace/servo_calibration_result.json`을 찾고 있었고,
`from motor_control import MotorController`도 `hardware/` 이동 이후 경로와 맞지 않았다.

## 수정

- Calibration: `WorkSpace/hardware/servo_calibration_result.json` 사용
- MotorController: `from hardware.motor_control import MotorController`
- `hardware/__init__.py` 추가
- STServo SDK는 PyPI `ftservo-python-sdk`의 `scservo_sdk`를 우선 사용
- PyPI 패키지가 없으면 기존 `hardware/STServo_Python/stservo-env/scservo_sdk`를 fallback으로 사용

## Raspberry Pi 권장 설치

```bash
cd ~/VisionPoseCoach
source .venv/bin/activate
pip install ftservo-python-sdk
python -c "from scservo_sdk import PortHandler, sms_sts; print('STServo SDK OK')"
```

그 다음:

```bash
cd ~/VisionPoseCoach/WorkSpace
python -c "from hardware.motor_control import MotorController; print('motor_control import OK')"
```

Servo3 캘리브레이션 파일은 `hardware/servo_calibration_result.json`에 존재해야 하며
`servos["3"].joint == "wrist_flex"`, `max_speed > 0` 이어야 한다.

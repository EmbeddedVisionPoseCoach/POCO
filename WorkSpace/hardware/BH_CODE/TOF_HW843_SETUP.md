# HW-843 ToF 모듈 연결 및 코드 사용법

## 1. 센서와 코드가 맡는 일

HW-843은 VL53L0X 기반 ToF(Time of Flight) 모듈이다. 빛이 대상까지 갔다가
돌아오는 시간을 이용해 거리를 측정하고 I2C로 mm 값을 전달한다.

새 파일 `WorkSpace/pyQt/services/tof_service.py`가 다음 작업을 전담한다.

1. `/dev/i2c-3`을 연다.
2. 기본 7-bit 주소 `0x29`의 VL53L0X를 연다.
3. mm 값을 m로 변환한다.
4. EMA 필터를 적용한다.
5. 3 cm~2 m 범위를 벗어난 값이나 I2C 오류를 `valid=False`로 보고한다.
6. `close()`에서 연속 측정과 I2C 객체를 정리한다.

`pose_monitor_arm_controller.py`는 센서 드라이버를 직접 import하지 않고 이
서비스의 `read_distance_m()`만 호출한다. 따라서 나중에 센서가 바뀌어도 서비스
인터페이스만 유지하면 IK 코드는 그대로 둘 수 있다.

## 2. Raspberry Pi 5 배선

| HW-843 | Raspberry Pi 5 | BCM 번호 | 물리 핀 |
|---|---|---:|---:|
| VIN/VCC | 3.3 V | - | 1 |
| GND | GND | - | 6 |
| SDA | GPIO22 | 22 | 15 |
| SCL | GPIO23 | 23 | 16 |

처음 연결할 때는 3.3 V 전원을 권장한다. SDA/SCL 신호선을 5 V로 끌어올리면
라즈베리파이 GPIO를 손상시킬 수 있으므로 사용 중인 HW-843 보드의 풀업 전압도
확인해야 한다. XSHUT와 GPIO1은 센서 한 개만 사용할 때 연결하지 않아도 된다.

## 3. `dtoverlay`의 정확한 의미

팀원이 전달한 `dtoverlay=12c3-pi5 , pins_22_23`에서 앞의 `1`은 숫자 1이
아니라 소문자 `i`여야 하고, 쉼표 앞뒤 공백 없이 다음처럼 쓴다.

```text
dtoverlay=i2c3-pi5,pins_22_23
```

- `dtoverlay`: 부팅 때 Device Tree Overlay를 적용한다.
- `i2c3-pi5`: Raspberry Pi 5의 I2C controller 3을 활성화한다.
- `pins_22_23`: I2C3의 SDA/SCL을 BCM GPIO22/23에 배치한다.
- 결과: Python에서는 `/dev/i2c-3`을 열면 된다.

Raspberry Pi OS Bookworm 기준 `/boot/firmware/config.txt` 마지막에 위 한 줄을
추가하고 재부팅한다. 이 저장소의 코드는 시스템 부팅 설정을 자동 수정하지 않는다.

```bash
sudo nano /boot/firmware/config.txt
sudo reboot
```

재부팅 뒤 확인한다.

```bash
ls -l /dev/i2c-3
sudo apt install i2c-tools
i2cdetect -y 3
```

정상 연결이면 표에 보통 `29`가 보인다. ST 문서의 `0x52/0x53`은 8-bit
read/write 주소 표기이고, Linux/Python 코드에서는 7-bit 주소 `0x29`를 쓴다.

## 4. Python 패키지

```bash
cd /home/bhc/poco/POCO/WorkSpace/hardware/BH_CODE
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements-monitor-arm.txt
```

센서 모듈은 `adafruit_extended_bus.ExtendedI2C(3)`으로 `/dev/i2c-3`을 열고,
`adafruit_vl53l0x.VL53L0X`에서 `.range` 값을 읽는다.

## 5. 설정값

`monitor_arm_settings.json`의 `tof`가 하드웨어 설정이다.

- `mode`: `hardware`이면 실제 센서, `fixed_stub`이면 고정 모의값
- `i2c_bus`: `3`, 즉 `/dev/i2c-3`
- `i2c_address`: `41`, 즉 16진수 `0x29`
- `filter_alpha`: EMA의 새 측정값 비율. 작을수록 부드럽지만 느리다.
- `sensor_origin_x_m`: 베이스 원점에서 센서 발광부까지의 X 오프셋
- `minimum_user_x_m`, `maximum_user_x_m`: IK에 허용하는 사용자 X 안전범위

센서 자체의 최대 측정범위와 팔이 허용하는 사용자 X 범위는 서로 다른 안전조건이다.
예를 들어 센서가 1.2 m를 정상 측정해도 현재 팔 설정의 최대 사용자 X보다 크면
모터 명령은 `SAFE HOLD` 된다.

## 6. ToF 0.7 + Vision 0.3 계산

모든 값을 먼저 같은 `베이스 기준 사용자 X` 좌표로 바꾼 다음 합친다.

```text
tof_user_x = sensor_origin_x + filtered_tof_range

vision_distance
  = first_tof_user_monitor_distance
    * first_eye_gap_px / current_eye_gap_px

vision_user_x = current_monitor_x + vision_distance

fused_user_x = 0.7 * tof_user_x + 0.3 * vision_user_x
target_monitor_x = fused_user_x - desired_user_monitor_distance
```

첫 유효 눈 간격은 그 시점의 ToF 사용자-모니터 거리로 자동 보정한다. `r`을 누르면
비전 기준을 지우고 다음 유효 프레임에서 다시 보정한다.

- ToF와 눈 랜드마크 모두 정상: 0.7 + 0.3 융합
- 눈 랜드마크 유실/비전 이상값: ToF 단독 사용
- ToF 유실/범위 오류: 비전 단독 모터 제어를 금지하고 `SAFE HOLD`

마지막 조건은 얼굴 랜드마크 오검출만으로 팔이 움직이지 않도록 하는 안전장치다.

## 7. 실행 순서

모터 없이 센서·카메라·IK만 확인한다.

```bash
python3 pose_monitor_arm_controller.py
```

라즈베리파이 밖에서 고정 ToF 사용자 X로 시험한다.

```bash
python3 pose_monitor_arm_controller.py --tof-user-x-m 0.73
```

모터 1·2의 수동 한계와 비상 정지 준비를 확인한 뒤에만 실제 모터를 연다.

```bash
python3 pose_monitor_arm_controller.py --enable-motor
```

모터 3·4에는 이 컨트롤러가 명령을 만들거나 보내지 않는다.

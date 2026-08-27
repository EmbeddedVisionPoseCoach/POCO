"""Vision Pose Coach 하드웨어 기본 상수.

원칙
-----
- 실제 배선/센서 기본 동작값은 JSON이 아니라 이 파일의 코드 상수로 정의한다.
- IRSensorService(), ADXL345IMUService() 생성자는 이 값을 기본값으로 사용한다.
- hardware_control.json은 생성자 입력값이 아니라 저장된 런타임 튜닝/override 용도다.
- PyQt에서 설정을 저장하면 Hardware Process가 UPDATE_CONFIG를 받아 apply_*()로 반영한다.
"""

# IR Sensor
IR_PIN = 17
IR_ACTIVE_LOW = True
IR_SAMPLE_HZ = 20.0
IR_STABLE_DETECT_SEC = 0.5
IR_LOST_GRACE_SEC = 0.3
IR_CHECK_TIMEOUT_SEC = 5.0

# ADXL345 IMU
IMU_BUS = 1
IMU_ADDRESS = 0x53
IMU_SAMPLE_HZ = 50.0
IMU_CALIBRATION_SEC = 3.0
IMU_LPF_ALPHA = 0.20
IMU_DEADBAND_DEG = 0.50

# PID default
PITCH_KP = 10.0
PITCH_KI = 0.0
PITCH_KD = 0.0

ROLL_KP = 2.8
ROLL_KI = 0.08
ROLL_KD = 0.18

PID_OUTPUT_LIMIT_DEG_S = 30.0
PID_INTEGRAL_LIMIT_RAD_SEC = 0.35
PID_DERIVATIVE_LPF_ALPHA = 0.15
PID_OUTPUT_LPF_ALPHA = 0.20

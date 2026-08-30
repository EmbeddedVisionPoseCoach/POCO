"""Vision POCO 하드웨어 기본 상수.

원칙
-----
- 실제 배선/통신/샘플링 기본값은 이 파일에서 정의한다.
- hardware_control.json에는 사용자가 조절할 튜닝값만 저장한다.
- Motor1/2와 Motor3/4는 같은 MotorService를 공유한다.
- Motor3/4 짐벌 제어는 IMU X/Y Direct PID V1.9 기준을 사용한다.
"""


# ============================================================
# VL53L0X ToF (HW-843) - Raspberry Pi 5 I2C-3
# ============================================================
# /boot/firmware/config.txt:
#   dtoverlay=i2c3-pi5,pins_22_23
# BCM22/23 are physical pins 15/16.  The Linux device is /dev/i2c-3.
TOF_I2C_BUS = 3
TOF_I2C_ADDRESS = 0x29
TOF_SAMPLE_HZ = 20.0
TOF_MIN_RANGE_M = 0.03
TOF_MAX_RANGE_M = 2.0
TOF_FILTER_ALPHA = 0.25
TOF_IO_TIMEOUT_SEC = 0.20


# ============================================================
# ADXL345 IMU - Direct X/Y Control
# ============================================================
IMU_BUS = 1
IMU_ADDRESS = 0x53

# V1.9에서 잘 동작한 M3/M4 제어 주기.
IMU_SAMPLE_HZ = 100.0
IMU_CALIBRATION_SEC = 3.0
IMU_CALIBRATION_MIN_SAMPLES = 50

# EMA: filtered = alpha * current + (1-alpha) * previous
# 작을수록 더 부드럽고, 클수록 더 빠르게 반응한다.
IMU_LPF_ALPHA = 0.08
IMU_DEADBAND_G = 0.010
IMU_G_PER_LSB = 0.0039


# ============================================================
# Direct IMU PID
# ============================================================
# Motor3 <- IMU Y Error
MOTOR3_IMU_Y_KP = 120.0
MOTOR3_IMU_Y_KI = 0.0
MOTOR3_IMU_Y_KD = 0.0

# Motor4 <- IMU X Error
MOTOR4_IMU_X_KP = 120.0
MOTOR4_IMU_X_KI = 0.0
MOTOR4_IMU_X_KD = 0.0

# PID output unit = target velocity [deg/s]
PID_OUTPUT_LIMIT_DEG_S = 24.0
PID_INTEGRAL_LIMIT_G_SEC = 0.10
PID_DERIVATIVE_LPF_ALPHA = 0.15


# ============================================================
# Motor3 / Motor4 Actuator Tracking
# ============================================================
MOTOR34_COMMAND_HZ = 100.0
MOTOR34_AUTO_SPEED = 500
MOTOR34_AUTO_ACC = 12

# Direct IMU V1.9의 실기 동작과 POCO MotorService의 팀원용 각도 좌표계를 맞춘 값.
#
# 중요:
# - standalone tuner는 Calibration direction만 사용해 angle -> raw 변환한다.
# - POCO MotorController는 그 전에 COMMAND_TO_URDF_DIRECTION=-1을 한 번 더 적용한다.
# - 따라서 tuner의 sign=-1/-1과 같은 실제 raw 방향을 만들려면
#   POCO Motor34Controller의 TEAM-angle 적분 sign은 +1/+1이어야 한다.
#
# PIDController 자체의 error = -imu_error_g 식은 tuner와 동일하게 유지한다.
MOTOR3_IMU_Y_DIRECTION_SIGN = +1.0
MOTOR4_IMU_X_DIRECTION_SIGN = +1.0


# ============================================================
# Legacy compatibility aliases
# ============================================================
# 예전 Pitch/Roll 기반 코드 import가 바로 깨지는 것을 막기 위한 alias.
# 새 Motor3/4 제어에서는 사용하지 않는다.
IMU_DEADBAND_DEG = 0.50
PITCH_KP = MOTOR3_IMU_Y_KP
PITCH_KI = MOTOR3_IMU_Y_KI
PITCH_KD = MOTOR3_IMU_Y_KD
ROLL_KP = MOTOR4_IMU_X_KP
ROLL_KI = MOTOR4_IMU_X_KI
ROLL_KD = MOTOR4_IMU_X_KD
PID_INTEGRAL_LIMIT_RAD_SEC = 0.35
PID_OUTPUT_LPF_ALPHA = 0.20

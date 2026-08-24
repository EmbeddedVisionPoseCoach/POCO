import RPi.GPIO as GPIO
from smbus2 import SMBus
import time

BUS = 1
ADDR = 0x53

bus = SMBus(BUS)

# 측정 모드 활성화
bus.write_byte_data(ADDR, 0x2D, 0x08)

def to_signed(low, high):
    value = (high << 8) | low

    if value & 0x8000:
        value -= 65536

    return value


SENSOR_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)

temp_list = []

avg_x = 0
avg_y = 0
avg_z = 0


# ==========================
# Low Pass Filter 설정
# ==========================
ALPHA = 0.2

filter_x = 0
filter_y = 0
filter_z = 0


# ==========================
# Dead Band 설정
# ==========================
DEADBAND = 5


def calibration(lists):
    sum_x = 0
    sum_y = 0
    sum_z = 0

    for item in lists:
        sum_x += item[0]
        sum_y += item[1]
        sum_z += item[2]

    avg_x = sum_x / len(lists)
    avg_y = sum_y / len(lists)
    avg_z = sum_z / len(lists)

    return avg_x, avg_y, avg_z


def low_pass_filter(current, previous):
    return ALPHA * current + (1 - ALPHA) * previous


def dead_band(value):
    if abs(value) < DEADBAND:
        return 0

    return value


try:
    start_Tick = time.perf_counter()

    calibration_done = False

    while True:

        # ==========================
        # 기존 IR 센서 테스트
        # 그대로 유지
        # ==========================
        if GPIO.input(SENSOR_PIN) == GPIO.LOW:
            print("장애물 감지됨!")
        else:
            print("정상 (안전)")


        # ==========================
        # 가속도 센서
        # ==========================
        data = bus.read_i2c_block_data(ADDR, 0x32, 6)

        end_Tick = time.perf_counter()

        duration = end_Tick - start_Tick

        x = to_signed(data[0], data[1])
        y = to_signed(data[2], data[3])
        z = to_signed(data[4], data[5])


        # ==========================
        # 처음 3초 Calibration
        # ==========================
        if duration < 3.0:

            temp_list.append([x, y, z])

        else:

            # Calibration 한 번만 실행
            if calibration_done == False:

                avg_x, avg_y, avg_z = calibration(temp_list)

                calibration_done = True

                print("Calibration 완료")
                print(
                    f"AVG X: {avg_x:.2f}, "
                    f"Y: {avg_y:.2f}, "
                    f"Z: {avg_z:.2f}"
                )


            # ==========================
            # Offset 제거
            # ==========================
            x -= avg_x
            y -= avg_y
            z -= avg_z


            # ==========================
            # Low Pass Filter
            # ==========================
            filter_x = low_pass_filter(x, filter_x)
            filter_y = low_pass_filter(y, filter_y)
            filter_z = low_pass_filter(z, filter_z)


            # ==========================
            # Dead Band
            # ==========================
            x = dead_band(filter_x)
            y = dead_band(filter_y)
            z = dead_band(filter_z)


        print(
            f"X: {x:6.2f}, "
            f"Y: {y:6.2f}, "
            f"Z: {z:6.2f}"
        )

        time.sleep(0.1)


except KeyboardInterrupt:
    GPIO.cleanup()
    bus.close()
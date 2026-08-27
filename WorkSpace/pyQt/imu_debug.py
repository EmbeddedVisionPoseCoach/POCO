import time

from services.hardware_config_service import HardwareConfigService
from services.imu_service import ADXL345IMUService


def create_imu_from_config(control):
    imu_cfg = control["imu"]
    pid_cfg = control["pid"]
    return ADXL345IMUService(
        bus_number=imu_cfg["bus"],
        address=imu_cfg["address"],
        sample_hz=imu_cfg["sample_hz"],
        calibration_sec=imu_cfg["calibration_sec"],
        imu_alpha=imu_cfg["lpf_alpha"],
        deadband_deg=imu_cfg["deadband_deg"],
        pitch_pid=pid_cfg["pitch"],
        roll_pid=pid_cfg["roll"],
        output_limit_deg_s=pid_cfg["output_limit_deg_s"],
        integral_limit_rad_sec=pid_cfg["integral_limit_rad_sec"],
        derivative_alpha=pid_cfg["derivative_lpf_alpha"],
        output_alpha=pid_cfg["output_lpf_alpha"],
    )


def main():
    config_service = HardwareConfigService()
    config_data = config_service.load()
    imu = create_imu_from_config(config_data["control"])

    if not imu.open():
        print(f"IMU 초기화 실패: {imu.last_error}")
        return

    print("[DEBUG] IR 사전 확인은 생략합니다. 실앱에서는 반드시 IR -> IMU 순서입니다.")
    imu.start_calibration()

    try:
        while True:
            state = imu.update()
            result = imu.consume_calibration_result()
            if result is not None:
                saved = config_service.update_imu_calibration(
                    result["pitch_offset_deg"],
                    result["roll_offset_deg"],
                    result["sample_count"],
                )
                print("[DEBUG] Offset JSON 저장:", saved["calibration"]["imu"])

            if state["calibrating"]:
                print(
                    f"Offset Calibration... {state['calibration_remaining_sec']:.1f}s "
                    f"samples={state['calibration_sample_count']} "
                    f"raw=({state['raw_x']}, {state['raw_y']}, {state['raw_z']})"
                )
            else:
                print(
                    f"Offset(P/R)=({state['pitch_offset_deg']:+6.2f}, "
                    f"{state['roll_offset_deg']:+6.2f}) deg | "
                    f"Pitch={state['pitch_deg']:+6.2f} deg "
                    f"Roll={state['roll_deg']:+6.2f} deg | "
                    f"PitchSpeed={state['correction_pitch_speed_deg_s']:+6.2f} deg/s "
                    f"RollSpeed={state['correction_roll_speed_deg_s']:+6.2f} deg/s"
                )

            time.sleep(imu.sample_interval)

    except KeyboardInterrupt:
        pass
    finally:
        imu.close()
        print("IMU 종료")


if __name__ == "__main__":
    main()

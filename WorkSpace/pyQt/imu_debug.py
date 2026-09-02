"""ADXL345 Direct X/Y PID 단독 디버그 도구."""
import time

from services.hardware_config_service import HardwareConfigService
from services.imu_service import ADXL345IMUService


def main():
    config_service = HardwareConfigService()
    config = config_service.load()
    control = config["control"]

    imu = ADXL345IMUService()
    imu.apply_control_config(control["imu"], control["pid"])

    if not imu.open():
        raise RuntimeError(f"IMU open fail: {imu.last_error}")

    print("[IMU DEBUG] CALIBRATION 시작")
    imu.start_calibration()

    try:
        while True:
            state = imu.update()
            result = imu.consume_calibration_result()

            if result is not None:
                config_service.update_imu_calibration(
                    result["x_reference_g"],
                    result["y_reference_g"],
                    result["sample_count"],
                    x_reference_raw=result.get("x_reference_raw", 0.0),
                    y_reference_raw=result.get("y_reference_raw", 0.0),
                )
                print(
                    "[IMU DEBUG] Calibration Saved "
                    f"Xref={result['x_reference_g']:+.5f}g "
                    f"Yref={result['y_reference_g']:+.5f}g"
                )

            print(
                f"X={state.get('filtered_x_g', 0.0):+.5f}g "
                f"dX={state.get('imu_x_error_g', 0.0):+.5f}g | "
                f"Y={state.get('filtered_y_g', 0.0):+.5f}g "
                f"dY={state.get('imu_y_error_g', 0.0):+.5f}g | "
                f"M3(Y)={state.get('motor3_correction_speed_deg_s', 0.0):+.3f}deg/s "
                f"M4(X)={state.get('motor4_correction_speed_deg_s', 0.0):+.3f}deg/s"
            )

            time.sleep(imu.sample_interval)

    except KeyboardInterrupt:
        pass
    finally:
        imu.close()


if __name__ == "__main__":
    main()

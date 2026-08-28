#!/usr/bin/env python3
"""Servo3/4 무동작 연결 진단. 이동 명령은 보내지 않고 Ping만 확인한다."""
from pathlib import Path
import json
import os
import sys

HARDWARE_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = HARDWARE_DIR.parent
CALIBRATION_FILE = HARDWARE_DIR / "servo_calibration_result.json"
LOCAL_SDK_ROOT = HARDWARE_DIR / "stservo_env" / "scservo_sdk"

if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))
if str(LOCAL_SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(LOCAL_SDK_ROOT))

print("=== Motor3/4 Connection Check ===")
print("hardware_dir      =", HARDWARE_DIR)
print("calibration_file  =", CALIBRATION_FILE)
print("calibration exists=", CALIBRATION_FILE.exists())

if not CALIBRATION_FILE.exists():
    raise SystemExit("[FAIL] servo_calibration_result.json 없음")

with CALIBRATION_FILE.open("r", encoding="utf-8") as f:
    data = json.load(f)

try:
    import port_handler
    print("SDK root           =", LOCAL_SDK_ROOT)
    print("SDK module         =", getattr(port_handler, "__file__", None))
except ModuleNotFoundError as e:
    raise SystemExit(f"[FAIL] local STServo SDK import 실패: {e}")

try:
    from hardware.motor_control import MotorController
except Exception as e:
    raise SystemExit(f"[FAIL] motor_control/SDK import 실패: {e}")

device = str(data.get("device") or "/dev/ttyACM0")
print("device            =", device)
print("baudrate          =", data.get("baudrate"))
print("device exists      =", os.path.exists(device))

if not os.path.exists(device):
    raise SystemExit(f"[FAIL] Serial device 없음: {device}")

arm = None
try:
    arm = MotorController(calibration_file=str(CALIBRATION_FILE))
    print("[OK] Serial port / baudrate open")

    all_ok = True
    for servo_id, expected_joint in ((3, "wrist_flex"), (4, "wrist_roll")):
        servo = data.get("servos", {}).get(str(servo_id), {})
        print(f"Servo{servo_id} joint      =", servo.get("joint"))
        print(f"Servo{servo_id} max_speed  =", servo.get("max_speed"))
        if servo.get("joint") != expected_joint:
            print(f"[FAIL] Servo{servo_id} joint != {expected_joint}")
            all_ok = False
            continue

        result = arm.driver.ping(servo_id)
        print(f"Servo{servo_id} ping result =", result)
        if not result.get("success"):
            all_ok = False

    if not all_ok:
        raise SystemExit("[FAIL] Servo3/4 연결 검사 실패")

    print("[OK] Servo3/4 연결 정상")
finally:
    if arm is not None:
        arm.close()

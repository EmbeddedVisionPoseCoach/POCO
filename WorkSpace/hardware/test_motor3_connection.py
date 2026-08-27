#!/usr/bin/env python3
"""Servo3(wrist_flex) 무동작 연결 진단.

이 스크립트는 모터 이동 명령을 보내지 않는다.
Calibration -> SDK -> Serial Port -> Servo ID 3 Ping만 확인한다.
"""
from pathlib import Path
import json
import os
import sys

HARDWARE_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = HARDWARE_DIR.parent
CALIBRATION_FILE = HARDWARE_DIR / "servo_calibration_result.json"

if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))

print("=== Motor3 Connection Check ===")
print("hardware_dir      =", HARDWARE_DIR)
print("calibration_file  =", CALIBRATION_FILE)
print("calibration exists=", CALIBRATION_FILE.exists())

if not CALIBRATION_FILE.exists():
    raise SystemExit("[FAIL] servo_calibration_result.json 없음")

with CALIBRATION_FILE.open("r", encoding="utf-8") as f:
    data = json.load(f)

servo3 = data.get("servos", {}).get("3")
if not isinstance(servo3, dict):
    raise SystemExit("[FAIL] servos['3'] 없음")

print("device            =", data.get("device"))
print("baudrate          =", data.get("baudrate"))
print("servo3 joint      =", servo3.get("joint"))
print("servo3 max_speed  =", servo3.get("max_speed"))

if servo3.get("joint") != "wrist_flex":
    raise SystemExit("[FAIL] Servo3 joint가 wrist_flex가 아님")
if not servo3.get("max_speed"):
    raise SystemExit("[FAIL] Servo3 max_speed 미설정")

try:
    import scservo_sdk
    print("SDK source         = pip scservo_sdk")
    print("SDK module         =", getattr(scservo_sdk, "__file__", None))
except ModuleNotFoundError:
    local_sdk = HARDWARE_DIR / "STServo_Python" / "stservo-env" / "scservo_sdk"
    print("pip SDK            = NOT INSTALLED")
    print("local SDK fallback =", local_sdk)

try:
    from hardware.motor_control import MotorController
except Exception as e:
    raise SystemExit(f"[FAIL] motor_control/SDK import 실패: {e}")

device = str(data.get("device") or "/dev/ttyACM0")
print("device exists      =", os.path.exists(device))
if not os.path.exists(device):
    raise SystemExit(f"[FAIL] Serial device 없음: {device}")

arm = None
try:
    arm = MotorController(calibration_file=str(CALIBRATION_FILE))
    print("[OK] Serial port / baudrate open")
    result = arm.driver.ping(3)
    print("Servo3 ping result =", result)
    if not result.get("success"):
        raise SystemExit("[FAIL] Servo ID 3 ping 실패")
    print("[OK] Servo ID 3 wrist_flex 연결 정상")
finally:
    if arm is not None:
        arm.close()

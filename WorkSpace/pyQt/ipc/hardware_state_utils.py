def get_ir_state(hardware_state):
    if not isinstance(hardware_state, dict):
        return {}
    ir = hardware_state.get("ir", {})
    return ir if isinstance(ir, dict) else {}


def is_ir_detected(hardware_state, require_stable=False):
    ir = get_ir_state(hardware_state)
    key = "stable_detected" if require_stable else "detected"
    return bool(ir.get("available", False) and ir.get(key, False))


def get_imu_state(hardware_state):
    if not isinstance(hardware_state, dict):
        return {}
    imu = hardware_state.get("imu", {})
    return imu if isinstance(imu, dict) else {}


def get_motor_state(hardware_state):
    if not isinstance(hardware_state, dict):
        return {}
    motor = hardware_state.get("motor", {})
    return motor if isinstance(motor, dict) else {}

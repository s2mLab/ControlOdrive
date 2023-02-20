import json
from odrive.enums import GPIO_MODE_DIGITAL, MOTOR_TYPE_HIGH_CURRENT, EncoderMode


dictionary = {
    "enable_dc_bus_overvoltage_ramp": True,
    "dc_bus_overvoltage_ramp_start": 48.2,
    "dc_bus_overvoltage_ramp_end": 48.35,
    "gpio9_mode": GPIO_MODE_DIGITAL,
    "gpio10_mode": GPIO_MODE_DIGITAL,
    "gpio11_mode": GPIO_MODE_DIGITAL,
    "enable_brake_resistor": True,
    "brake_resistance": 3.3,
    "vel_limit": 5.0,
    "vel_limit_tolerance": 2.0,
    "mode": 1,
    "cpr": 48,
    "calib_scan_distance": 150.0,
    "motor_type": MOTOR_TYPE_HIGH_CURRENT,
    "pole_pairs": 8,
    "torque_constant": 0.21,
    "calibration_current": 10.0,
    "resistance_calib_max_voltage": 20.0,
    "requested_current_range": 25.0,
    "current_control_bandwidth": 100.0,
    "current_lim": 20.0,
    "torque_lim": 10.0,
}

# Serializing json
json_object = json.dumps(dictionary, indent=4)

# Writing to sample.json
with open("hardware_and_security.json", "w") as outfile:
    outfile.write(json_object)

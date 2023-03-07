import json
from odrive.enums import GPIO_MODE_DIGITAL, MOTOR_TYPE_HIGH_CURRENT, ENCODER_MODE_HALL


dictionary = {
    "enable_dc_bus_overvoltage_ramp": True,
    "dc_bus_overvoltage_ramp_start": 48.2,
    "dc_bus_overvoltage_ramp_end": 48.35,
    "gpio9_mode": GPIO_MODE_DIGITAL,  # The pins the encoder is connected to
    "gpio10_mode": GPIO_MODE_DIGITAL,
    "gpio11_mode": GPIO_MODE_DIGITAL,
    "enable_brake_resistor": True,
    "brake_resistance": 3.3,  # Ohms
    "pedals_vel_limit": 120,  # tr/min of the pedals
    "vel_limit_tolerance": 2.0,  # tr/s of the motor
    "mode": ENCODER_MODE_HALL,
    "cpr": 6 * 8,  # Count per revolution
    "calib_scan_distance": 150.0,
    "motor_type": MOTOR_TYPE_HIGH_CURRENT,
    "pole_pairs": 8,
    "torque_constant": 0.08225583064524448,
    "calibration_current": 7.0,  # Not sure of this value
    "resistance_calib_max_voltage": 20.0,  # Not sure of this value
    "requested_current_range": 25.0,  # > current_lim + current_lim_margin nut as little as possible
    "current_control_bandwidth": 100.0,  # Not sure of this value
    "current_lim": 7.0,  # 350W / 48V # Not sure of this value
    "torque_lim": 100.0,  # torque applied by the user TODO: choose a torque to protect the user.
    "watchdog_timeout": 0.1,  # Chosen arbitrarily
    "watchdog_feed_time": 0.01,  # Chosen arbitrarily
    # Some doc specified 93 instead of 91 but the checks on the motor seems to validate 91
    "reduction_ratio": 8/36*10/91,
    "pedals_accel_lim": 1080,  # tr/(min^2) of the pedals
    "maximal_velocity_stop": 15,  # tr/s of the pedals
}

# Serializing json
json_object = json.dumps(dictionary, indent=4)

# Writing to sample.json
with open("hardware_and_security.json", "w") as outfile:
    outfile.write(json_object)
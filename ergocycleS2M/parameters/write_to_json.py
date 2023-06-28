import json
import numpy as np

from odrive.enums import GPIO_MODE_DIGITAL, MOTOR_TYPE_HIGH_CURRENT, ENCODER_MODE_HALL

with open("hardware_and_security.json", "r") as hardware_and_security_file:
    hardware_and_security = json.load(hardware_and_security_file)

current_lim = 500 / 48  # 500W / 48V # Not sure of this value
# Calculated with the `torque_constant_computation.py` script
torque_constant = hardware_and_security["torque_constant"]
# Some doc specified 93 instead of 91 but the checks on the motor seems to validate 91
reduction_ratio = 8 / 36 * 10 / 91
torque_lim = current_lim * torque_constant / reduction_ratio  # torque applied by the user in Nm
# rpm of the pedals, it has been set at this value since above it, we frequently have errors
cadence_lim = 66

dictionary = {
    "enable_dc_bus_overvoltage_ramp": True,
    # See https://discourse.odriverobotics.com/t/power-supply-in-security-mode-when-forcing-on-the-motor/10229/16
    # The doc of the power supply specify that the output voltage adj. range is 48 – 56 VDC
    # At output voltage higher than nominal output voltage max. output current has to be reduced accordingly, in order
    # not to exceed max. output power.
    "dc_bus_overvoltage_ramp_start": 53.0,
    "dc_bus_overvoltage_ramp_end": 55.0,
    "gpio9_mode": GPIO_MODE_DIGITAL,  # The pins the encoder is connected to
    "gpio10_mode": GPIO_MODE_DIGITAL,
    "gpio11_mode": GPIO_MODE_DIGITAL,
    # The bus current allowed to flow back to the power supply before the brake resistor module will start shunting
    # current.
    "max_regen_current": 0.0,
    # This should be greater in magnitude than max_regen_current
    # (And https://discourse.odriverobotics.com/t/power-supply-in-security-mode-when-forcing-on-the-motor/10229/16)
    "dc_max_negative_current": -0.01,
    # Theoretically Imax = 12.5 A, but there are peaks above in normal functioning and I don't think it is problematic
    # to increase this limit
    "dc_max_positive_current": 15,
    "enable_brake_resistor": True,
    # If you set this to a lower value than the true brake resistance then the ODrive will not meet the
    # max_regen_current constraint during braking, that is it will sink more than max_regen_current into the power
    # supply. Some power supplies don’t like this.
    # If you set this to a higher value than the true brake resistance then the ODrive will unnecessarily burn more
    # power than required during braking.
    "brake_resistance": 3.5,  # Ohms
    "pedals_cadence_limit": cadence_lim,
    "vel_limit_tolerance": 2.0,  # tr/s of the motor
    "mode": ENCODER_MODE_HALL,
    "cpr": 6 * 8,  # Count per revolution
    "calib_scan_distance": 150.0,
    "motor_type": MOTOR_TYPE_HIGH_CURRENT,
    "pole_pairs": 8,
    "torque_constant": torque_constant,
    "calibration_current": 7.0,  # Not sure of this value
    "resistance_calib_max_voltage": 20.0,  # Not sure of this value
    "requested_current_range": 25.0,  # > current_lim + current_lim_margin but as little as possible
    "current_control_bandwidth": 100.0,  # Not sure of this value
    "current_lim": current_lim,
    "torque_lim": torque_lim,
    "power_lim": torque_lim * cadence_lim * 2 * np.pi / 60,  # Motor power limit in W
    "watchdog_timeout": 0.3,  # Chosen arbitrarily
    "watchdog_feed_time": 0.01,  # Chosen arbitrarily
    # Some doc specified 93 instead of 91 but the checks on the motor seems to validate 91
    "reduction_ratio": 8 / 36 * 10 / 91,
    "pedals_accel_lim": 31,  # (rpm)/s of the pedals
    "torque_ramp_rate_lim": 5.5,  # Nm/s
    "maximal_cadence_stop": 31,  # tr/s of the pedals
    # The `resisting_current_proportional` and `resisting_current_constant` are calculated with the
    # `resisting_current_calibration.py` script (it runs a calibration on the motor that lasts several minutes)
    "resisting_current_proportional": hardware_and_security["resisting_current_proportional"],
    "resisting_current_constant": hardware_and_security["resisting_current_constant"],
}

# Serializing json
json_object = json.dumps(dictionary, indent=4)

# Writing to sample.json
with open("hardware_and_security.json", "w") as outfile:
    outfile.write(json_object)

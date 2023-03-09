""" Example on how to use the Motor.py code in velocity control"""
import json
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
# motor.calibration()

# Set the control mode
motor.velocity_control(0)

# Variables to plot
t = []
instructions = []
vel_estimate = []
iq_setpoint = []
iq_measured = []
mechanical_power = []
torque = []
vbus = []

t0 = time.time()
t1 = time.time()
t_next = 0

#for instruction, delta_t, mode in zip((15, 30, -15), (15, 30, 45), ("Eccentric", "Eccentric", "Concentric")):
#for i in range(1, 6):
instruction = 120
motor.set_training_mode("Eccentric")
motor.velocity_control(instruction)
print(instruction)

while t1 - t0 < 60:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        instructions.append(instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        iq_setpoint.append(motor.get_iq_setpoint())
        iq_measured.append(motor.get_iq_measured())
        mechanical_power.append(motor.get_mechanical_power())
        vbus.append(motor.get_monitoring_commands()[2])
        torque.append(motor.get_torque_measured())
        print(vel_estimate[-1], torque[-1], motor.get_mechanical_power())
        t_next += 0.05

motor.stop()

dictionary = {
    "vel_estimate": vel_estimate,
    "iq_setpoint": iq_setpoint,
    "iq_measured": iq_measured,
    "mechanical_power": mechanical_power,
    "torque": torque,
}

vel_estimate = np.asarray(vel_estimate)
iq_setpoint = np.asarray(iq_setpoint)
iq_measured = np.asarray(iq_measured)
mechanical_power = np.asarray(mechanical_power)

# Writing to .json

json_object = json.dumps(dictionary, indent=4)
with open(f"XP/{abs(instruction)}_ecc.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(t, vel_estimate, label="Estimated velocity")
plt.plot(t, instructions, label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

plt.figure()
plt.plot(t, mechanical_power)
plt.title("Mechanical power")

plt.show()
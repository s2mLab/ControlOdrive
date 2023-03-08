""" Example on how to use the Motor.py code in velocity control"""
import json
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
motor.zero_position_calibration()

# Variables to plot
t = []
instructions = []
positions = []
iq_setpoint = []
iq_measured = []

t0 = time.time()
t1 = time.time()
t_next = 0

for i in range(8):
    instruction = i * 45
    print(instruction)
    motor.position_control(instruction)
    while t1 - t0 < 12 * (i + 1):
        t1 = time.time()
        if t1 - t0 > t_next:
            t.append(t1 - t0)
            instructions.append(instruction)
            positions.append(motor.get_angle())
            iq_setpoint.append(motor.get_iq_setpoint())
            iq_measured.append(motor.get_iq_measured())
            t_next += 0.05

motor.stop()


dictionary = {
    "time": t,
    "instructions": instructions,
    "positions": positions,
    "iq_setpoint": iq_setpoint,
    "iq_measured": iq_measured,
}

# Writing to .json
json_object = json.dumps(dictionary, indent=4)
with open("torque_constant_right_3kg.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(t, positions, label="Estimated position")
plt.plot(t, instructions, label="Instruction")
plt.title("Position control")
plt.xlabel("Time (s)")
plt.ylabel("Position (deg)")
plt.legend()

plt.figure()
plt.plot(t, iq_measured, label="iq_measured")
plt.plot(t, iq_setpoint, label="iq_setpoint")
plt.title("Current")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.legend()

plt.show()
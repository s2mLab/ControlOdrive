"""
This script was used to find the current corresponding to the resisting torque.
"""
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = MotorController()

# Set the control mode
motor.torque_control(0.0, resisting_torque_current=0.0)

# Variables to plot
t = []
instructions = []
vel_estimate = []
iq_measured = []

t0 = time.time()
t1 = time.time()
t_next = 0

mode = "Concentric"
motor.set_training_mode(mode)

instruction = 2
torque_ramp_rate = 5

motor.torque_control(instruction, torque_ramp_rate/10)

while t1 - t0 < 10:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        instructions.append(instruction)
        vel_estimate.append(motor.get_estimated_velocity())
        iq_measured.append(motor.get_iq_measured())
        t_next += 0.05

motor.stop()

dictionary = {
    "time": t,
    "vel_estimate": vel_estimate,
    "Iq_measured": iq_measured,
}

# Writing to .json
json_object = json.dumps(dictionary, indent=4)
with open(f"find_resisting_torque_{mode}_{instruction}_{torque_ramp_rate}_1.json", "w") as outfile:
    outfile.write(json_object)

plt.title("Velocity in torque control")
plt.plot(t, vel_estimate, label="Estimated velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()

plt.figure()
plt.title("Current")
plt.plot(t, iq_measured, label="Current measured")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.legend()

plt.show()

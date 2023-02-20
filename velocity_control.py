""" Example on how to use the Motor.py code in velocity control"""

import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall(enable_watchdog=False)
motor.calibration()

# Set the control mode
motor.velocity_control(0)

# Variables to plot
t = []
instructions = []
vel_estimate = []
monitoring_commands = []

t0 = time.time()
t1 = time.time()
t_next = 0

for instruction, delta_t in zip((2, 3, -3), (10, 20, 35)):
    motor.velocity_control(instruction)
    print(instruction)

    while t1 - t0 < delta_t:
        t1 = time.time()
        if t1 - t0 > t_next:
            t.append(t1 - t0)
            instructions.append(instruction)
            vel_estimate.append(motor.get_estimated_velocity())
            monitoring_commands.append(motor.get_monitoring_commands())
            t_next += 0.05

motor.stop()

vel_estimate = np.asarray(vel_estimate)
monitoring_commands = np.asarray(monitoring_commands)

plt.plot(t, vel_estimate, label="Estimated velocity")
plt.plot(t, instructions, label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (turn/s)")
plt.legend()

fig2, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(monitoring_commands[:, 4], label="Mechanical power")
axs[1].plot(monitoring_commands[:, 5], label="Electrical power")
axs[2].plot(monitoring_commands[:, 2], label="Iq_setpoint")
axs[2].plot(monitoring_commands[:, 3], label="Iq_mesured")

plt.show()
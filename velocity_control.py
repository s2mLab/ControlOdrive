""" Example on how to use the Motor.py code in velocity control"""

import numpy as np
import matplotlib.pyplot as plt

from Motor import *

# Initialisation
motor = OdriveEncoderHall()
# motor.erase_configuration()
# motor.configuration()
# motor.save_configuration()
# motor.calibration()

# Set the control mode
motor.velocity_control(0)

# Variables to plot
t = []
instructions = []
vel_estimate = []

t0 = time.time()
t1 = time.time()
t_next = 0

for instruction, delta_t in zip((2, 3, -3), (10, 20, 35)):
    motor.set_velocity(instruction)
    print(instruction)

    while t1 - t0 < delta_t:
        t1 = time.time()
        if t1 - t0 > t_next:
            t.append(t1 - t0)
            instructions.append(instruction)
            vel_estimate.append(motor.get_estimated_velocity())
            t_next += 0.05

motor.stop()

vel_estimate = np.asarray(vel_estimate)

plt.plot(t, vel_estimate, label="Estimated velocity")
plt.plot(t, instructions, label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (turn/s)")
plt.legend()

plt.show()
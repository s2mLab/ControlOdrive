""" Example on how to use the Motor.py code in position control"""

import numpy as np
import matplotlib.pyplot as plt

from Motor import *

# Initialisation
motor = OdriveEncoderHall()
#motor.erase_configuration()
#motor.configuration()
#motor.save_configuration()
#motor.calibration()

# Set the control mode
motor.position_control(0)
time.sleep(10)  # In case the motor was not initialized at 0 degree

# Variables to plot
t = []
instructions = []
pos_estimate = []

t0 = time.time()
t1 = time.time()
t_next = 0

for instruction, delta_t in zip((0, 270, -90), (5, 20, 35)):
    motor.set_position(instruction)
    print(instruction)

    while t1 - t0 < delta_t:
        t1 = time.time()
        if t1 - t0 > t_next:
            t.append(t1 - t0)
            instructions.append(instruction)
            pos_estimate.append(motor.get_angle_motor())
            t_next += 0.05

motor.stop()

pos_estimate = np.asarray(pos_estimate)

plt.plot(t, pos_estimate * 360, label="Position")
plt.plot(t, instructions, label="Instruction")
plt.title("Position control")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.legend()
plt.show()
""" Example on how to use the Motor.py code in position control"""

import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
# motor.calibration()

t0 = time.time()
t1 = time.time()
t_next = 0

for instruction, delta_t in zip((0, 10), (6, 50)):
    motor.position_control(instruction)
    print(instruction)

    while t1 - t0 < delta_t:
        t1 = time.time()
        if t1 - t0 > t_next:
            t_next += 0.05

motor.stop()
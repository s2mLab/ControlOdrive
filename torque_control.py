""" Example on how to use the Motor.py code in torque control"""
import json
import random
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()

# Set the control mode
motor.torque_control()

instructions = []

t0 = time.time()
t1 = time.time()
t_next = 0

length = 0.17
m = 4.430
g = 9.81
instruction = -40
print(instruction)
motor.set_training_mode("Concentric")

motor.torque_control(instruction, torque_ramp_rate=8)
print("Go")
with open(f'XP/torque_control_{motor.get_training_mode()}_{abs(instruction)}_{random.randint(0,1000)}.json', 'w') as f:

    while t1 - t0 < 10:
        motor.save_data(instruction)
        t1 = time.time()
        if t1 - t0 > t_next:
            instructions.append(instruction)
            t_next += 0.05
print("Stop")
motor.stop()

angles = np.asarray(motor.data['angle'])

plt.title("Velocity in torque control")
plt.plot(motor.data['time'], motor.data['velocity'], label="Estimated velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

plt.figure()
plt.title("Torque")
plt.plot(motor.data['time'], motor.data['instruction'], label="Instruction")
plt.plot(motor.data['time'], motor.data["user_torque"], label="User torque")
plt.plot(motor.data['time'], motor.data["measured_torque"], label="Measured torque")
plt.plot(motor.data['time'], - length * m * g * np.sin(angles / 180 * np.pi), label="'Actual' torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.show()
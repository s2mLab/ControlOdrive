""" Example on how to use the Motor.py code in power control"""
import json
import random
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()

# Set the control mode
motor.torque_control()

t0 = time.time()
t1 = time.time()
t_next = 0

length = 0.17
m = 4
g = 9.81
instruction = length * m * g

power = 20
torque_lim = 6.67
print(instruction)
motor.set_training_mode("Eccentric")
motor.torque_control(instruction)
vel = 0.0

with open(f'XP/power_control_linear_{motor.get_training_mode()}_{power}_{random.randint(0,1000)}', 'w') as f:

    while t1 - t0 < 40:
        t1 = time.time()
        if t1 - t0 > t_next:
            motor.torque_control(instruction)
            motor.save_data(instruction)
            json.dump(motor.data, f)
            print(f"Vel: {vel}, {motor.data['velocity'][-1]}, "
                  f"Instruction: {motor.data['user_torque'][-1]}, "
                  f"Power: {motor.data['mechanical_power'][-1]}")
            if len(motor.data['velocity']) > 1.0:
                vel = np.mean(motor.data['velocity'][max(0, len(motor.data['velocity'])-20): -1])
            else:
                vel = 0.0

            if abs(vel) < 12:
                instruction = power / (12 * 2 * np.pi / 60)
            else:
                instruction = abs(vel)

            t_next += 0.05

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

plt.figure()
plt.title("Mechanical power")
plt.plot(motor.data['time'], motor.data["mechanical_power"])
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")


plt.show()
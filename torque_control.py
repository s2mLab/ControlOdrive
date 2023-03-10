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
instruction = -15
print(instruction)
motor.set_training_mode("Concentric")

rd = random.randint(0,1000)

motor.torque_control(instruction, torque_ramp_rate=2)
print("Go")

while t1 - t0 < 30:
    motor.save_data(instruction)
    with open(f'XP/torque_control_{motor.get_training_mode()}_{abs(instruction)}_{rd}.json', 'w') as f:
        json.dump(motor.data, f)
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

plt.figure()
plt.title("Mechanical power")
plt.plot(motor.data['time'], motor.data["mechanical_power"], label="Mechanical power")
plt.plot(motor.data['time'], motor.data["user_power"], label="User power")
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")

plt.show()
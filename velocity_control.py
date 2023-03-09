""" Example on how to use the Motor.py code in velocity control"""
import json
import random
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()

# Set the control mode
motor.velocity_control(0)

t0 = time.time()
t1 = time.time()
t_next = 0

instruction = 60
motor.set_training_mode("Eccentric")
motor.velocity_control(instruction)
print(instruction)

with open(f'XP/velocity_control_{motor.get_training_mode()}_{abs(instruction)}_{random.randint(0,1000)}', 'w') as f:

    while t1 - t0 < 30:
        t1 = time.time()
        if t1 - t0 > t_next:
            motor.save_data(instruction)
            json.dump(motor.data, f)
            print(f"Vel: {motor.data['velocity'][-1]}, "
                  f"Torque: {motor.data['user_torque'][-1]}, "
                  f"Power: {motor.data['mechanical_power'][-1]}")
            t_next += 0.05

motor.stop()

plt.plot(motor.data['time'], motor.data['velocity'], label="Estimated velocity")
plt.plot(motor.data['time'], motor.data['instruction'], label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

plt.figure()
plt.plot(motor.data['time'], motor.data['mechanical_power'])
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")
plt.title("Mechanical power")

plt.figure()
plt.plot(motor.data['time'], motor.data['user_torque'])
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("User torque")

plt.show()
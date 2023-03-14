""" Example on how to use the Motor.py code in power control"""
import json
import random
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()

power = 20
print(power)
motor.set_training_mode("Concentric")

with open(f'XP/power_control_{motor.get_training_mode()}_{power}_{random.randint(0,1000)}', 'w') as f:

    motor.power_control(
        power=power,
        power_mode=PowerMode.LINEAR,
        file=f)

motor.stop()

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
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.figure()
plt.title("Mechanical power")
plt.plot(motor.data['time'], motor.data["mechanical_power"])
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")


plt.show()

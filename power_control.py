""" Example on how to use the Motor.py code in torque control"""
import numpy as np
import matplotlib.pyplot as plt

from motor import *

# Initialisation
motor = OdriveEncoderHall()
motor.zero_position_calibration()

# Set the control mode
motor.torque_control()

# Variables to plot
t = []
instructions = []
vel_estimate = []
torque_measured = []
positions = []
mechanical_power = []

t0 = time.time()
t1 = time.time()
t_next = 0

l = 0.17
m = 4
g = 9.81
instruction = l * m * g

power = 1.0
torque_lim = 6.67
print(instruction)
motor.set_training_mode("Eccentric")
motor.torque_control(instruction)

while t1 - t0 < 40:
    t1 = time.time()
    if t1 - t0 > t_next:
        t.append(t1 - t0)
        vel_estimate.append(motor.get_estimated_velocity())
        if len(vel_estimate) > 1.0:
            vel = np.mean(vel_estimate[max(0, len(vel_estimate)-20): -1])
        else:
            vel = 0.0

        if abs(vel) < 12:
            instruction = torque_lim
        else:
            print(abs(power / (vel / 60)))
            instruction = abs(power / (vel / 60))

        motor.torque_control(instruction)
        instructions.append(instruction)
        torque_measured.append(motor.get_torque_measured())
        positions.append(motor.get_angle())
        mechanical_power.append(motor.get_mechanical_power())

        t_next += 0.05

motor.stop()

vel_estimate = np.asarray(vel_estimate)
positions = np.asarray(positions)

plt.title("Velocity in torque control")
plt.plot(t, vel_estimate, label="Estimated velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()

plt.figure()
plt.title("Torque")
plt.plot(t, instructions, label="Instruction")
plt.plot(t, torque_measured, label="Measured torque")
plt.plot(t, - l * m * g * np.sin(positions / 180 * np.pi), label="'Actual' torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.figure()
plt.title("Mechanical power")
plt.plot(t, mechanical_power)
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")

plt.show()
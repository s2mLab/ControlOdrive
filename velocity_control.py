""" Example on how to use the motor.py code in velocity control"""
import matplotlib.pyplot as plt

from motor import *
from save_and_load import load

# Initialisation
motor = OdriveEncoderHall()

# Data saving
t0 = time.time()
t1 = time.time()
f_sample = 20  # Hz
t_next = 0

# Control
instruction = 30
ramp_rate = 6
motor.set_training_mode("Eccentric")
motor.velocity_control(instruction, velocity_ramp_rate=ramp_rate)

while t1 - t0 < 10:
    t1 = time.time()
    if t1 - t0 > t_next:
        motor.save_data_to_file("velocity_control_example", spin_box=instruction, instruction=instruction)
        t_next += 1 / f_sample

motor.stop()

data = load("velocity_control_example.bio")

t = np.asarray(data['time'])

# Plot
plt.plot(t, data['velocity'], label="Velocity")
plt.plot(t, np.min([list(ramp_rate * t), data['instruction']], axis=0), label="Instruction")
plt.title("Ramped velocity control")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (tr/min)")
plt.legend()

plt.show()
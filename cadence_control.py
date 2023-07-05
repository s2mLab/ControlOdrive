"""
Example on how to use the motor.py code in cadence control
"""
import matplotlib.pyplot as plt
import numpy as np
import time

from ergocycleS2M.motor_control.motor_controller import MotorController
from ergocycleS2M.data_processing.load import load, compute_data

# Creation of a motor object that will be used to control the motor
motor = MotorController()

# Variables for data saving
t0 = time.time()
t1 = time.time()
f_sample = 20  # Hz
t_next = 0

# The instruction is the cadence of the pedals in rpm
instruction = 30
# The ramp rate is the rate at which the cadence will increase or decrease in rpm/s
ramp_rate = 6
# The directions available are "Forward" and "Reverse" see the documentation for more details and see what the
# directions refer to on a photo of the ergocycle
motor.set_direction("Reverse")
# The cadence control method will set the cadence of the pedals to the desired value with the desired ramp rate
motor.cadence_control(instruction, cadence_ramp_rate=ramp_rate)

# Loop to save the data (can be done in a separate thread)
while t1 - t0 < 10:
    t1 = time.time()
    if t1 - t0 > t_next:
        # The difference between `spin_box` and `instruction` is not relevant here. It is when used with a graphic user
        # interface, in this case the spin box is the value displayed on the interface and the instruction is the
        # torque or the cadence that is sent to the motor, because no matter what the control model specified by the
        # user is, the control of the motor can only be done with a torque or a cadence control.
        motor.minimal_save_data_to_file(
            "cadence_control_example", spin_box=instruction, instruction=instruction, ramp_instruction=ramp_rate
        )
        t_next += 1 / f_sample

motor.stop()

data = load("cadence_control_example.bio")

t = np.asarray(data["time"])

# Plot
_, cadence, *_ = compute_data(data["vel_estimate"], data["turns"], data["iq_measured"])
plt.plot(t, cadence, label="Pedals cadence")
# The real instruction sent to the motor is not saved in the data, but it can be computed from the ramp_rate and the
# instruction if needed.
plt.plot(t, -np.min([list(ramp_rate * t), data["instruction"]], axis=0), label="Instruction")
plt.title("Ramped cadence control")
plt.xlabel("Time (s)")
plt.ylabel("Cadence (rpm)")
plt.legend()

plt.show()

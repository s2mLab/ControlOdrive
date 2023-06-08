"""
This script is used to sample the current corresponding to the resisting torque of the motor at different velocities.
"""
import matplotlib.pyplot as plt
import random
import time

from ergocycleS2M.data_processing.load import plot_data, read
from ergocycleS2M.motor_control.enums import DirectionMode
from ergocycleS2M.motor_control.motor import MotorController

# Initialisation
motor = MotorController()
motor.set_direction(DirectionMode.REVERSE)

rd = random.randint(0, 100)

file_name = f"data_from_dyn_calibration_{rd}.bio"


def calibration(instruction, nb_turns=5):
    """
    This function is used to sample the current corresponding to the resisting torque of the motor at different
    cadences.

    Parameters
    ----------
    instruction
        The cadence instruction in rpm.
    nb_turns
        The number of turns to sample the data at the desired cadence.
    """
    if instruction > 0:
        motor.set_direction(DirectionMode.FORWARD)
    motor.cadence_control(instruction)

    print(f"Waiting to stabilize at {instruction} rpm...")

    # Wait for to reach the instruction
    while not (instruction - 1 < motor.get_cadence() < instruction + 1):
        pass

    t0 = time.time()
    t1 = time.time()

    # Wait for 1 second to stabilize at the instructed velocity (can't be done with a time.sleep() because of the
    # watchdog thread)
    while t1 - t0 < 1.0:
        motor.minimal_save_data_to_file(file_name, instruction=instruction)
        t1 = time.time()

    print(f"Stabilized at {instruction} rpm.")

    t0 = time.time()
    t1 = time.time()
    t_next = 0

    # Save the data as frequently as possible during 5 turns
    while t1 - t0 < nb_turns / (abs(instruction) / 60):
        t1 = time.time()
        motor.minimal_save_data_to_file(file_name, instruction=instruction)


for ins in range(-60, 61, 5):
    if ins == 0:
        calibration(-1, 1)
        calibration(1, 1)
    else:
        calibration(ins)

motor.stop()

plot_data(read(file_name, 100, 100))

plt.show()

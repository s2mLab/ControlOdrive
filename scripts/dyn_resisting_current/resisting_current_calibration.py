"""
This script is used to sample the current corresponding to the resisting torque of the motor at different velocities.
"""
import random
import matplotlib.pyplot as plt

from ergocycleS2M.motor_control.motor import *

# Initialisation
motor = MotorController()

resisting_current = []
motor_velocity = []

fs = 50

rd = random.randint(0, 1000)

for instruction in range(-65, 66, 5):
    if instruction == 0:
        pass
    else:
        if instruction > 0:
            motor.set_training_mode("Eccentric")
        motor.velocity_control(instruction)

        print(f"Waiting to stabilize at {instruction} rpm...")

        # Wait for to reach the instruction
        while not(instruction - 1 < motor.get_velocity() < instruction + 1):
            pass

        t0 = time.time()
        t1 = time.time()

        # Wait for 1 second to stabilize at the instructed velocity (can't be done with a time.sleep() because of the
        # watchdog thread)
        while t1 - t0 < 1.0:
            motor.save_data_to_file(f"data_from_dyn_calibration_{rd}", instruction=instruction)
            t1 = time.time()

        print(f"Stabilized at {instruction} rpm.")

        iq_measured = []
        vel_estimate = []

        t0 = time.time()
        t1 = time.time()
        t_next = 0

        # Measure the current and velocity for 5 turns
        while t1 - t0 < abs(instruction) / 60 * 5:
            t1 = time.time()
            if t1 - t0 > t_next:
                motor.save_data_to_file(f"data_from_dyn_calibration_{rd}", instruction=instruction)
                iq_measured.append(motor.get_iq_measured())
                vel_estimate.append(motor.odrv0.axis0.encoder.vel_estimate)
                t_next += 1/fs

        resisting_current.append(np.mean(iq_measured))
        motor_velocity.append(np.mean(vel_estimate))

motor.stop()

dictionary = {
    "resisting_current": resisting_current,
    "motor_velocity": motor_velocity,
}

# Writing to .json
json_object = json.dumps(dictionary, indent=4)
with open(f"velocity_and_current_from_dyn_calibration_{rd}.json", "w") as outfile:
    outfile.write(json_object)

plt.plot(motor_velocity, resisting_current)

plt.show()
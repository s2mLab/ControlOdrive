""" Example on how to use the Motor.py code."""

from Motor import *

motor = OdriveEncoderHall()
#motor.erase_configuration()
#motor.configuration()
#motor.save_configuration()
#motor.calibration()

t0 = time.time()
t1 = time.time()
i = 0

# Set a given speed (turn/s) for the motor
motor.set_speed(-2)

while t1 - t0 < 20:
    t1 = time.time()
    if t1 - t0 > i:
        print(i, " : ", motor.get_angle_motor())
        i += 1

motor.set_speed(0)

print("Final position : ", motor.get_angle_motor())
# Set a given torque (Nm)
# motor.set_torque(0.5)

# Set a given torque (Nm) and a target speed (turn/s)
# motor.set_torque_2(-0.1, 30)

# print the angle of the motor, needs a calibration to determine the angle 0°
# print(motor.get_angle_motor())

# print the angle of the motor, needs a calibration to determine the angle 0°
# print(motor.get_angle_crank())
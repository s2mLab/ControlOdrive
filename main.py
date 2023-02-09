""" Example on how to use the Motor.py code."""

from Motor import *


motor = OdriveEncoderHall()
motor.erase_configuration()
motor.configuration()
motor.calibration()

# Set a given speed (turn/s) for the motor
motor.set_speed(10)

# Set a given torque (Nm)
motor.set_torque(0.5)

# Set a given torque (Nm) and a target speed (turn/s)
motor.set_torque_2(-0.1, 30)

# print the angle of the motor, needs a calibration to determine the angle 0°
print(motor.get_angle_motor())

# print the angle of the motor, needs a calibration to determine the angle 0°
print(motor.get_angle_crank())
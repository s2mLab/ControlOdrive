"""
Calibration without load
"""

from ergocycleS2M.motor_control.motor_controller import MotorController

# Initialisation
motor = MotorController(enable_watchdog=False)
motor.erase_configuration()
motor.configuration()
motor.save_configuration()
motor.calibration(mechanical_load=False)

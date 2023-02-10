""" Example on how to use the Motor.py code."""
import numpy as np
import matplotlib.pyplot as plt
from Motor import *

motor = OdriveEncoderHall()
#motor.erase_configuration()
#motor.configuration()
#motor.save_configuration()
#motor.calibration()

t0 = time.time()
t1 = time.time()
i = 0
t = []
x = []
mech_p = []
consigne = []
v = []
monitoring_commands = []

# Set a given speed (turn/s) for the motor
print("Here")
consignet = -2
motor.set_speed(consignet)

while t1 - t0 < 7:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        x.append(xt)
        v.append(vt)
        mech_p.append(mt)
        monitoring_commands.append(motor.get_monitoring_commands())
        i += 0.05

consignet = 0
motor.set_speed(consignet)

while t1 - t0 < 15:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        x.append(xt)
        v.append(vt)
        mech_p.append(mt)
        monitoring_commands.append(motor.get_monitoring_commands())
        i += 0.05

consignet = 2
motor.set_speed(consignet)

while t1 - t0 < 25:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        x.append(xt)
        v.append(vt)
        mech_p.append(mt)
        monitoring_commands.append(motor.get_monitoring_commands())
        i += 0.05

monitoring_commands = np.asarray(monitoring_commands)
motor.stop()

fig, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(t, v, label="Estimated velocity")
axs[0].plot(t, consigne, label="Instruction")
fig.suptitle("Velocity control")

axs[0].set_ylabel("Velocity (turn/s)")
axs[0].legend()

axs[1].plot(t,x)
axs[1].set_ylabel("Angle")
axs[2].plot(t, mech_p)
axs[2].set_ylabel("Mechanical power (W)")
axs[2].set_xlabel("Time (s)")

fig2, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(monitoring_commands[:, 0], label="pos_estimate")
axs[1].plot(monitoring_commands[:, 1], label="vel_estimate")
axs[2].plot(monitoring_commands[:, 2], label="Iq_setpoint")
axs[2].plot(monitoring_commands[:, 2], label="Iq_mesured")

plt.show()

print("Final position : ", motor.get_angle_motor())
# Set a given torque (Nm)
# motor.set_torque(0.5)

# Set a given torque (Nm) and a target speed (turn/s)
# motor.set_torque_2(-0.1, 30)

# print the angle of the motor, needs a calibration to determine the angle 0°
# print(motor.get_angle_motor())

# print the angle of the motor, needs a calibration to determine the angle 0°
# print(motor.get_angle_crank())
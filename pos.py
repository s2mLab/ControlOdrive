import matplotlib.pyplot as plt
from Motor import *
import numpy as np

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
consigne = []
v = []
monitoring_commands = []

consignet = 0
motor.amandine(consignet)
print(consignet)

while t1 - t0 < 15:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        monitoring_commands.append(motor.get_monitoring_commands())
        x.append(xt)
        v.append(vt)
        i += 0.05

consignet = 270
motor.amandine2(consignet)
print(consignet)

while t1 - t0 < 30:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        monitoring_commands.append(motor.get_monitoring_commands())
        x.append(xt)
        v.append(vt)
        i += 0.05

consignet = -90
motor.amandine2(consignet)
print(consignet)

while t1 - t0 < 45:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        monitoring_commands.append(motor.get_monitoring_commands())
        x.append(xt)
        v.append(vt)
        i += 0.05

motor.stop()
monitoring_commands = np.asarray(monitoring_commands)

plt.plot(t, monitoring_commands[:, 0] * 360, label="Position")
plt.plot(t, consigne, label="Instruction")
plt.title("Position control")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.legend()

fig2, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(monitoring_commands[:, 0], label="pos_estimate")
axs[1].plot(monitoring_commands[:, 1], label="vel_estimate")
axs[2].plot(monitoring_commands[:, 2], label="Iq_setpoint")
axs[2].plot(monitoring_commands[:, 2], label="Iq_mesured")

plt.show()
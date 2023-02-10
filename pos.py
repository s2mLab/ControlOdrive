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
consigne = []
v = []


consignet = 0
motor.amandine(consignet)

while t1 - t0 < 15:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        x.append(xt)
        v.append(vt)
        i += 0.05

consignet = 360
motor.amandine(consignet)

while t1 - t0 < 30:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        x.append(xt)
        v.append(vt)
        i += 0.05

consignet = 1080
motor.amandine(consignet)

while t1 - t0 < 45:
    t1 = time.time()
    if t1 - t0 > i:
        xt, vt, mt = motor.get_angle_motor()
        t.append(t1-t0)
        consigne.append(consignet)
        x.append(xt)
        v.append(vt)
        i += 0.05

motor.stop()

plt.plot(t, x, label="Position")
plt.plot(t, consigne, label="Instruction")
plt.title("Position control")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.legend()
plt.show()
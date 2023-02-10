import time
import numpy as np
import matplotlib.pyplot as plt
from Motor import *

motor = OdriveEncoderHall()
#motor.erase_configuration()
#motor.configuration()
#motor.save_configuration()
#motor.calibration()

motor.amandine2(10)
t0 = time.time()
t1 = time.time()
monitoring_commands = []
i = 0.0
while t1 - t0 < 10:
    t1 = time.time()
    if t1 - t0 > i:
        monitoring_commands.append(motor.get_monitoring_commands())
        i += 0.05

motor.stop()
monitoring_commands = np.asarray(monitoring_commands)

fig2, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(monitoring_commands[:, 0], label="pos_estimate")
axs[1].plot(monitoring_commands[:, 1], label="vel_estimate")
axs[2].plot(monitoring_commands[:, 2], label="Iq_setpoint")
axs[2].plot(monitoring_commands[:, 2], label="Iq_mesured")

plt.show()
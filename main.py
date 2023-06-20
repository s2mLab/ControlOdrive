"""
Script to control the ergocycle through a graphical user interface.

python -m PyQt5.uic.pyuic -x ergocycle_gui.ui -o ergocycle_gui.py
"""
import numpy as np
import sys

from PyQt5 import QtWidgets

from ergocycleS2M.gui.application_gui import ErgocycleApplication

# from ergocycleS2M.motor_control.motor_controller import MotorController

from ergocycleS2M.motor_control.mock_controller import MockController


def main():
    """
    Main function to run the application.
    """
    motor = MockController(enable_watchdog=True, external_watchdog=True)
    app = QtWidgets.QApplication(sys.argv)
    gui = ErgocycleApplication(motor)
    gui.show()
    gui.motor.config_watchdog(True, 0.3)
    app.exec()
    app.run = False
    dt = np.asarray(gui.motor_thread.dt)
    import matplotlib.pyplot as plt

    plt.plot(dt)
    plt.show()
    print(np.mean(dt), np.max(dt))


if __name__ == "__main__":
    main()

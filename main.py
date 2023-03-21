from ergocycle_gui import Ui_MainWindow
import sys
import random
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *

from motor import *


class App(QtWidgets.QMainWindow):
    def __init__(self, motor):
        super(App, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.motor = motor
        self.velocity = 30
        data = threading.Thread(target=self._data, name="Data", daemon=True)
        data.start()
        self.motor.config_watchdog(True, 0.5)
        self.ui.Power_pushButton.clicked.connect(self.vel)

    def vel(self):
        self.motor.velocity_control(self.velocity)

    def _data(self):
        """
        To be called by a daemon thread.
        """
        rd = random.randint(0, 1000)

        while True:
            self.motor.odrv0.axis0.watchdog_feed()
            self.motor.save_data()
            #print(
            #    self.motor.odrv0.error,
            #    self.motor.odrv0.axis0.error,
            #    self.motor.odrv0.axis0.controller.error,
            #    self.motor.odrv0.axis0.encoder.error,
            #    self.motor.odrv0.axis0.motor.error,
            #    self.motor.odrv0.axis0.sensorless_estimator.error
            #)
            with open(f'XP/gui_{rd}.json', 'w') as f:
                json.dump(self.motor.data, f)
            self.ui.power_lineEdit.setText(f"{self.motor.data['user_power'][-1]:.0f}")
            self.ui.velocity_lineEdit.setText(f"{self.motor.data['velocity'][-1]:.0f}")
            self.ui.torque_lineEdit.setText(f"{self.motor.data['user_torque'][-1]:.2f}")

            time.sleep(0.05)

    def states_and_errors(self):
        pass


if __name__ == "__main__":
    motor = OdriveEncoderHall(enable_watchdog=False, external_watchdog=True)
    app = QApplication(sys.argv)
    gui = App(motor)
    gui.show()
    app.exec()

# python -m PyQt5.uic.pyuic -x ergocycle_gui.ui -o ergocycle_gui.py
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

        data = threading.Thread(target=self._data, name="Data", daemon=True)
        data.start()
        self.motor.config_watchdog(True, 1.0)

        # Control modes
        self.ui.STOP_pushButton.clicked.connect(self.stop)
        self.ui.power_pushButton.clicked.connect(self.power_mode)
        self.ui.velocity_pushButton.clicked.connect(self.velocity_mode)
        self.ui.torque_pushButton.clicked.connect(self.torque_mode)

        # Instruction
        self.ui.instruction_spinBox.valueChanged.connect(self.change_instruction)

    def stop(self):
        print("ya")
        self.ui.units_label.setText(self.test())
        self.motor.stop()
        self.ui.instruction_spinBox.setValue(0)
        self.ui.units_label.setText("The motor is stopped")

    def test(self):
        print("Test")
        return "Test"

    def power_mode(self):
        self.ui.instruction_spinBox.setValue(0)
        self.ui.units_label.setText("(W)")
        self.motor.power_control(0.0)

    def velocity_mode(self):
        self.ui.instruction_spinBox.setValue(0)
        self.ui.units_label.setText("(tr/min)")
        self.motor.velocity_control(0.0)

    def torque_mode(self):
        self.ui.instruction_spinBox.setValue(0)
        self.ui.units_label.setText("(Nm)")
        self.motor.velocity_control(0.0)

    def change_instruction(self):
        control_mode = self.motor.get_control_mode()
        print(control_mode)
        if control_mode == ControlMode.POWER_CONTROL:
            self.motor.power_control(self.ui.instruction_spinBox.value())
        elif control_mode == ControlMode.VELOCITY_CONTROL:
            print(self.ui.instruction_spinBox.value())
            self.motor.velocity_control(self.ui.instruction_spinBox.value())
        elif control_mode == ControlMode.TORQUE_CONTROL:
            self.motor.torque_control(self.ui.instruction_spinBox.value())

    def _data(self):
        """
        To be called by a daemon thread.
        """
        rd = random.randint(0, 1000)

        while True:
            self.motor.odrv0.axis0.watchdog_feed()
            self.motor.save_data()
            #print(
             #  self.motor.odrv0.error,
              # self.motor.odrv0.axis0.error,
               #self.motor.odrv0.axis0.controller.error,
            #   self.motor.odrv0.axis0.encoder.error,
             #  self.motor.odrv0.axis0.motor.error,
              # self.motor.odrv0.axis0.sensorless_estimator.error
            #)
            with open(f'XP/gui_{rd}.json', 'w') as f:
                json.dump(self.motor.data, f)
            self.ui.power_lineEdit.setText(f"{self.motor.data['user_power'][-1]:.0f}")
            self.ui.velocity_lineEdit.setText(f"{self.motor.data['velocity'][-1]:.0f}")
            self.ui.torque_lineEdit.setText(f"{self.motor.data['user_torque'][-1]:.2f}")

            time.sleep(0.01)


if __name__ == "__main__":
    motor = OdriveEncoderHall(enable_watchdog=False, external_watchdog=True)
    app = QApplication(sys.argv)
    gui = App(motor)
    gui.show()
    app.exec()

# python -m PyQt5.uic.pyuic -x ergocycle_gui.ui -o ergocycle_gui.py
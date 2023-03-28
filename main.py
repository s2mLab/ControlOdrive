from ergocycle_gui import Ui_MainWindow
import sys
import random
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *

from motor import *


with open("parameters/hardware_and_security.json", "r") as hardware_and_security_file:
    hardware_and_security = json.load(hardware_and_security_file)


class App(QtWidgets.QMainWindow):
    def __init__(self, motor):
        super(App, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Style
        self.ui.power_lineEdit.setStyleSheet("background-color: white; color: black;")
        self.ui.velocity_lineEdit.setStyleSheet("background-color: white; color: black;")
        self.ui.torque_lineEdit.setStyleSheet("background-color: white; color: black;")

        self.motor = motor

        self.run = True
        self.instruction = np.inf
        data = threading.Thread(target=self._data, name="Data", daemon=True)
        data.start()
        self.motor.config_watchdog(True, 0.7)

        self.ui.BAU_pushButton.clicked.connect(self.bau)

        # Combo boxes
        _translate = QtCore.QCoreApplication.translate
        self.ui.training_comboBox.setItemText(0, _translate("MainWindow", TrainingMode.CONCENTRIC.value))
        self.ui.training_comboBox.setItemText(1, _translate("MainWindow", TrainingMode.ECCENTRIC.value))
        self.ui.training_comboBox.setCurrentText(self.motor.get_training_mode().value)

        # Training modes
        self.ui.training_comboBox.activated.connect(self.change_training_mode)

        # Control modes
        self.ui.STOP_pushButton.clicked.connect(self.stop)
        self.ui.power_pushButton.clicked.connect(self.power_mode)
        self.ui.velocity_pushButton.clicked.connect(self.velocity_mode)
        self.ui.torque_pushButton.clicked.connect(self.torque_mode)
        self.ui.linear_pushButton.clicked.connect(self.linear_mode)

        # Instruction
        self.ui.instruction_spinBox.valueChanged.connect(self.change_instruction)

        # Saving
        self.ui.save_stop_pushButton.setEnabled(False)
        self.ui.save_start_pushButton.clicked.connect(self.save_start)
        self.ui.save_stop_pushButton.clicked.connect(self.save_stop)
        self._save = False
        self._file_path = ""

        self.power = 0.0
        self.linear_coeff = 0.0
        self.velocities = np.zeros(20)

    def bau(self):
        self.run = False

    def stop(self):
        self.ui.STOP_pushButton.setEnabled(False)
        self.change_mode()
        self.ui.instruction_spinBox.setValue(0)
        self.ui.instruction_spinBox.setEnabled(False)
        self.ui.units_label.setText("")

        self.motor.previous_control_mode = self.motor.get_control_mode()

        self.motor.stopping()
        self.ui.control_label.setText("The motor is stopping")
        self.ui.training_comboBox.setEnabled(False)

        stopped = threading.Thread(target=self._stopped_thread, name="Data", daemon=True)
        stopped.start()

    def _stopped_thread(self):
        self.motor.stopped()
        self.ui.STOP_pushButton.setEnabled(True)
        self.change_mode(stop=True)
        self.ui.control_label.setText("The motor has stopped")

    def change_training_mode(self):
        self.motor.set_training_mode(self.ui.training_comboBox.currentText())

    def change_mode(self, stop=False):
        self.ui.instruction_spinBox.setValue(0)
        if not stop:
            self.ui.control_label.setText(self.motor.get_control_mode().value)
        self.ui.velocity_pushButton.setEnabled(stop)
        self.ui.torque_pushButton.setEnabled(stop)
        self.ui.power_pushButton.setEnabled(stop)
        self.ui.linear_pushButton.setEnabled(stop)
        self.ui.training_comboBox.setEnabled(stop)
        self.ui.instruction_spinBox.setEnabled(not stop)

    def power_mode(self):
        self.ui.instruction_spinBox.setRange(
            0, int(hardware_and_security["pedals_vel_limit"] * hardware_and_security["torque_lim"])
        )
        self.ui.instruction_spinBox.setSingleStep(10)
        self.ui.units_label.setText("(W)")

        power_control_thread = threading.Thread(target=self._power_thread, name="Power control", daemon=True)
        power_control_thread.start()

    def _power_thread(
        self,
        torque_ramp_rate: float = 1.0,
        vel_min: float = 12.0,
    ):
        self.motor.power_init()  # power_mode
        self.velocities = np.zeros(20)

        resisting_torque = self.motor.odrv0.axis0.motor.config.torque_constant * \
            hardware_and_security["resisting_torque_current"] / self.motor._reduction_ratio

        if self.motor.get_control_mode() != ControlMode.POWER_CONTROL:
            if self.power == 0.0:
                self.instruction = 0.0
            else:
                self.instruction = vel_min * 2 * np.pi / 60
            self.motor.torque_control_init(
                self.instruction,
                torque_ramp_rate * self.motor._reduction_ratio,
                ControlMode.POWER_CONTROL
            )

        self.change_mode()

        t0 = time.time()
        f = 10
        i = 0

        while self.motor.get_control_mode() == ControlMode.POWER_CONTROL:
            t1 = time.time()
            if t1 - t0 > i / f:

                if self.power == 0.0:
                    self.motor.odrv0.axis0.controller.input_torque = 0.0
                else:
                    self.motor.odrv0.axis0.controller.input_torque = \
                        - self.motor._sign() * (abs(self.instruction) + resisting_torque) * self.motor._reduction_ratio

                self.velocities[i % 20] = self.motor.get_velocity()

                vel = np.mean(self.velocities[:min(i + 1, 20)])

                if abs(vel) < vel_min:
                    self.instruction = self.power / (vel_min * 2 * np.pi / 60)
                else:
                    self.instruction = self.power / (abs(vel) * 2 * np.pi / 60)

                i += 1
        print("Out of the power thread")

    def velocity_mode(self):
        self.ui.instruction_spinBox.setRange(0, hardware_and_security["pedals_vel_limit"])
        self.ui.instruction_spinBox.setSingleStep(10)
        self.ui.units_label.setText("(tr/min)")
        self.motor.velocity_control(0.0)
        self.change_mode()

    def torque_mode(self):
        self.ui.instruction_spinBox.setRange(0, int(hardware_and_security["torque_lim"]))
        self.ui.instruction_spinBox.setSingleStep(1)
        self.ui.units_label.setText("(Nm)")
        self.motor.torque_control(0.0)
        self.change_mode()

    def linear_mode(self):
        self.ui.instruction_spinBox.setRange(
            0, 100
        )
        self.ui.instruction_spinBox.setSingleStep(1)
        self.ui.units_label.setText("(Nm/(tr/min))")

        linear_control_thread = threading.Thread(target=self._linear_thread, name="Linear control", daemon=True)
        linear_control_thread.start()

    def _linear_thread(
        self,
        torque_ramp_rate: float = 1.0,
        vel_min: float = 12.0,
    ):
        self.linear = 0.0
        self.motor.power_init()  # power_mode
        self.velocities = np.zeros(20)

        resisting_torque = self.motor.odrv0.axis0.motor.config.torque_constant * \
            hardware_and_security["resisting_torque_current"] / self.motor._reduction_ratio

        if self.motor.get_control_mode() != ControlMode.LINEAR_CONTROL:
            if self.power == 0.0:
                torque = 0.0
            else:
                torque = vel_min * 2 * np.pi / 60
            self.motor.torque_control_init(
                torque,
                torque_ramp_rate * self.motor._reduction_ratio,
                ControlMode.LINEAR_CONTROL
            )

        self.change_mode()

        t0 = time.time()
        f = 10
        i = 0

        while self.motor.get_control_mode() == ControlMode.LINEAR_CONTROL:
            t1 = time.time()
            if t1 - t0 > i / f:
                self.instruction = - self.motor._sign() * (abs(torque) + resisting_torque) * self.motor._reduction_ratio
                self.motor.odrv0.axis0.controller.input_torque = self.instruction

                self.velocities[i % 20] = self.motor.get_velocity()

                vel = np.mean(self.velocities[:min(i + 1, 20)])

                if abs(vel) < vel_min:
                    torque = self.linear_coeff * vel_min
                else:
                    torque = self.linear_coeff * abs(vel)

                i += 1
        print("Out of the linear thread")

    def change_instruction(self):
        control_mode = self.motor.get_control_mode()
        if control_mode == ControlMode.POWER_CONTROL:
            self.power = self.ui.instruction_spinBox.value()
        elif control_mode == ControlMode.VELOCITY_CONTROL:
            self.instruction = self.ui.instruction_spinBox.value()
            self.motor.velocity_control(self.instruction)
        elif control_mode == ControlMode.TORQUE_CONTROL:
            self.instruction = self.ui.instruction_spinBox.value()
            self.motor.torque_control(self.instruction)
        elif control_mode == ControlMode.LINEAR_CONTROL:
            self.linear_coeff = self.ui.instruction_spinBox.value()

    def _data(self):
        """
        To be called by a daemon thread.
        """
        rd = random.randint(0, 1000)

        while self.run:
            self.motor.odrv0.axis0.watchdog_feed()
            self.motor.save_data_to_file(f'XP/gui_{rd}',
                                         spin_box=self.ui.instruction_spinBox.value(),
                                         instruction=self.instruction)
            self.motor.odrv0.axis0.watchdog_feed()

            self.ui.power_lineEdit.setText(f"{self.motor.get_user_power():.0f}")
            self.motor.odrv0.axis0.watchdog_feed()
            self.ui.velocity_lineEdit.setText(f"{self.motor.get_velocity():.0f}")
            self.motor.odrv0.axis0.watchdog_feed()
            self.ui.torque_lineEdit.setText(f"{self.motor.get_user_torque():.2f}")
            self.motor.odrv0.axis0.watchdog_feed()
            self.ui.errors_label.setText(
                f"{self.motor.odrv0.error},"
                f"{self.motor.odrv0.axis0.error},"
                f"{self.motor.odrv0.axis0.controller.error},"
                f"{self.motor.odrv0.axis0.encoder.error},"
                f"{self.motor.odrv0.axis0.motor.error},"
                f"{self.motor.odrv0.axis0.sensorless_estimator.error} |"
                f"{self.motor.odrv0.brake_resistor_armed},"
                f"{self.motor.odrv0.brake_resistor_saturated}"
                f"{self.motor.odrv0.brake_resistor_current:.2f}"
            )
            self.motor.odrv0.axis0.watchdog_feed()

            time.sleep(0.01)

    def save_start(self):
        # ToDo: Implement this for real.
        self._file_path = self.ui.save_lineEdit.text()
        self._save = True
        self.ui.save_start_pushButton.setEnabled(False)
        self.ui.save_stop_pushButton.setEnabled(True)
        self.ui.save_lineEdit.setEnabled(False)

    def save_stop(self):
        self._save = False
        self.ui.save_start_pushButton.setEnabled(True)
        self.ui.save_stop_pushButton.setEnabled(False)
        self.ui.save_lineEdit.setEnabled(True)


if __name__ == "__main__":
    motor = OdriveEncoderHall(enable_watchdog=False, external_watchdog=True)
    app = QApplication(sys.argv)
    gui = App(motor)
    gui.show()
    app.exec()
    app.run = False

# python -m PyQt5.uic.pyuic -x ergocycle_gui.ui -o ergocycle_gui.py
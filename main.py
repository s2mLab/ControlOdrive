import numpy as np

from ergocycle_gui import Ui_MainWindow
import sys
import random
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *

from motor import *
from enums import(
    ODriveMotorError,
    ODriveSensorlessEstimatorError,
    ODriveError,
    ODriveAxisError,
    ODriveEncoderError,
    ODriveControllerError,
    ODriveCanError,
)

with open("parameters/hardware_and_security.json", "r") as hardware_and_security_file:
    hardware_and_security = json.load(hardware_and_security_file)


def traduce_error(decim_number, odrive_enum):
    hex_number = "{0:x}".format(decim_number)
    res = ""
    for i, j in enumerate(hex_number[::-1]):
        if j != '0':
            for member in odrive_enum.__members__.values():
                if member.value == "0x" + format(int(j) * 16 ** int(i), f"0{8}X"):
                    res += member.name
                    res += ", "
    return res


class App(QtWidgets.QMainWindow):
    def __init__(self, odrive_motor: OdriveEncoderHall):
        super(App, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Style
        self.ui.power_lineEdit.setStyleSheet("background-color: white; color: black;")
        self.ui.velocity_lineEdit.setStyleSheet("background-color: white; color: black;")
        self.ui.torque_lineEdit.setStyleSheet("background-color: white; color: black;")

        self.motor = odrive_motor

        self.run = True
        self.instruction = np.inf
        self.ramp_instruction = np.inf
        self.spin_box = np.inf
        self.rd = random.randint(0, 1000)
        data = threading.Thread(target=self._data, name="Data", daemon=True)
        data.start()
        self.motor.config_watchdog(True, 1.0)

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
        self.ui.acceleration_spinBox.valueChanged.connect(self.change_instruction)

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
        self.ui.acceleration_spinBox.setValue(0)
        self.instruction = np.inf
        self.spin_box = np.inf
        self.ramp_instruction = np.inf
        self.ui.instruction_spinBox.setEnabled(False)
        self.ui.acceleration_spinBox.setEnabled(False)
        self.ui.training_comboBox.setEnabled(False)
        self.ui.units_label.setText("")
        self.ui.acceleration_units_label.setText("")

        self.motor.previous_control_mode = self.motor.get_control_mode()

        self.motor.stopping()
        self.ui.control_label.setText("The motor is stopping")

        # TODO: Put in the _data_thread
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
        self.spin_box = 0.0
        if not stop:
            self.ui.control_label.setText(self.motor.get_control_mode().value)
        self.ui.velocity_pushButton.setEnabled(stop)
        self.ui.torque_pushButton.setEnabled(stop)
        self.ui.power_pushButton.setEnabled(stop)
        self.ui.linear_pushButton.setEnabled(stop)
        self.ui.training_comboBox.setEnabled(stop)
        self.ui.instruction_spinBox.setEnabled(not stop)
        self.ui.acceleration_spinBox.setEnabled(not stop)

    def power_mode(self):
        self.ui.acceleration_spinBox.setValue(10)
        self.ui.instruction_spinBox.setRange(
            0, int(hardware_and_security["pedals_vel_limit"] * hardware_and_security["torque_lim"])
        )
        self.ui.acceleration_spinBox.setRange(
            0, int(hardware_and_security["pedals_accel_lim"] * hardware_and_security["torque_ramp_rate"])
        )
        self.ui.instruction_spinBox.setSingleStep(10)
        self.ui.acceleration_spinBox.setSingleStep(1)
        self.ui.units_label.setText("(W)")
        self.ui.acceleration_spinBox.setText("(W/s)")

        power_control_thread = threading.Thread(target=self._power_thread, name="Power control", daemon=True)
        power_control_thread.start()

    def _power_thread(
            self,
            torque_ramp_rate: float = 1.0,
            vel_min: float = 12.0,
    ):
        self.velocities = np.zeros(20)
        self.motor.odrv0.axis0.watchdog_feed()

        reduction_ratio = self.motor.get_reduction_ratio()
        self.motor.odrv0.axis0.watchdog_feed()

        resisting_torque = self.motor.odrv0.axis0.motor.config.torque_constant * \
                           hardware_and_security["resisting_torque_current"] / reduction_ratio
        self.motor.odrv0.axis0.watchdog_feed()

        if self.motor.get_control_mode() != ControlMode.POWER_CONTROL:
            self.motor.odrv0.axis0.watchdog_feed()
            if self.power == 0.0:
                self.motor.odrv0.axis0.watchdog_feed()
                self.instruction = 0.0
            else:
                self.motor.odrv0.axis0.watchdog_feed()
                self.instruction = vel_min * 2 * np.pi / 60
            self.motor.odrv0.axis0.watchdog_feed()
            self.motor.torque_control_init(
                self.instruction,
                torque_ramp_rate * reduction_ratio,
                ControlMode.POWER_CONTROL
            )
            self.motor.odrv0.axis0.watchdog_feed()

        self.motor.odrv0.axis0.watchdog_feed()
        self.change_mode()
        self.motor.odrv0.axis0.watchdog_feed()

        t0 = time.time()
        f = 10
        i = 0

        while self.motor.get_control_mode() == ControlMode.POWER_CONTROL:
            t1 = time.time()
            if t1 - t0 > i / f:
                self.motor.save_data_to_file(f'XP/gui_{self.rd}',
                                             spin_box=self.ui.instruction_spinBox.value(),
                                             instruction=self.instruction)
                self.motor.odrv0.axis0.watchdog_feed()

                if self.power == 0.0:
                    self.motor.odrv0.axis0.watchdog_feed()
                    self.motor.odrv0.axis0.controller.input_torque = 0.0
                else:
                    self.motor.odrv0.axis0.watchdog_feed()
                    self.motor.odrv0.axis0.controller.input_torque = \
                        - self.motor.get_sign() * (abs(self.instruction) + resisting_torque) * reduction_ratio

                self.velocities[i % 20] = self.motor.get_velocity()
                self.motor.odrv0.axis0.watchdog_feed()

                vel = np.mean(self.velocities[:min(i + 1, 20)])
                self.motor.odrv0.axis0.watchdog_feed()

                if abs(vel) < vel_min:
                    self.motor.odrv0.axis0.watchdog_feed()
                    self.instruction = self.power / (vel_min * 2 * np.pi / 60)
                else:
                    self.motor.odrv0.axis0.watchdog_feed()
                    self.instruction = self.power / (abs(vel) * 2 * np.pi / 60)

                i += 1
        print("Out of the power thread")

    def velocity_mode(self):
        self.ui.instruction_spinBox.setRange(0, hardware_and_security["pedals_vel_limit"])
        self.ui.acceleration_spinBox.setRange(0, hardware_and_security["pedals_accel_lim"])
        self.ui.instruction_spinBox.setSingleStep(10)
        self.ui.acceleration_spinBox.setSingleStep(5)
        self.ui.acceleration_spinBox.setValue(5)
        self.ui.units_label.setText("(tr/min)")
        self.ui.acceleration_units_label.setText("(tr/min^2)")
        self.motor.velocity_control(0.0)
        self.change_mode()

    def torque_mode(self):
        self.ui.instruction_spinBox.setRange(0, int(hardware_and_security["torque_lim"]))
        self.ui.acceleration_spinBox.setRange(0, int(hardware_and_security["torque_ramp_rate_lim"]))
        self.ui.instruction_spinBox.setSingleStep(1)
        self.ui.acceleration_spinBox.setSingleStep(1)
        self.ui.acceleration_spinBox.setValue(2)
        self.ramp_instruction = 2.0
        self.ui.units_label.setText("(Nm)")
        self.ui.acceleration_units_label.setText("(Nm/s)")
        self.instruction = self.motor.torque_control(0.0)
        self.change_mode()

    def linear_mode(self):
        self.ui.instruction_spinBox.setRange(
            0, 100
        )
        self.ui.acceleration_spinBox.setRange(
            0, 100
        )
        self.ui.instruction_spinBox.setSingleStep(1)
        self.ui.acceleration_spinBox.setSingleStep(1)
        self.ui.units_label.setText("(Nm/(tr/min))")
        self.ui.acceleration_units_label.setText("(Nm/(tr/min)/s)")

        linear_control_thread = threading.Thread(target=self._linear_thread, name="Linear control", daemon=True)
        linear_control_thread.start()

    def _linear_thread(
            self,
            torque_ramp_rate: float = 1.0,
            vel_min: float = 12.0,
    ):
        self.linear = 0.0
        self.velocities = np.zeros(20)
        self.motor.odrv0.axis0.watchdog_feed()
        reduction_ratio = self.motor.get_reduction_ratio()
        self.motor.odrv0.axis0.watchdog_feed()
        resisting_torque = self.motor.odrv0.axis0.motor.config.torque_constant * \
                           hardware_and_security["resisting_torque_current"] / reduction_ratio

        torque = 0.0
        self.motor.odrv0.axis0.watchdog_feed()
        if self.motor.get_control_mode() != ControlMode.LINEAR_CONTROL:
            if self.linear == 0.0:
                self.motor.odrv0.axis0.watchdog_feed()
                torque = 0.0
            else:
                self.motor.odrv0.axis0.watchdog_feed()
                torque = self.linear * self.motor.get_velocity()
            self.motor.odrv0.axis0.watchdog_feed()
            self.motor.torque_control_init(
                torque,
                torque_ramp_rate * reduction_ratio,
                ControlMode.LINEAR_CONTROL
            )
            self.motor.odrv0.axis0.watchdog_feed()

        self.change_mode()
        self.motor.odrv0.axis0.watchdog_feed()
        t0 = time.time()
        f = 10
        i = 0

        while self.motor.get_control_mode() == ControlMode.LINEAR_CONTROL:
            t1 = time.time()
            if t1 - t0 > i / f:
                self.motor.save_data_to_file(f'XP/gui_{self.rd}',
                                             spin_box=self.ui.instruction_spinBox.value(),
                                             instruction=self.instruction)
                self.motor.odrv0.axis0.watchdog_feed()
                self.instruction = - self.motor.get_sign() * (abs(torque) + resisting_torque) * reduction_ratio
                self.motor.odrv0.axis0.controller.input_torque = self.instruction
                self.motor.odrv0.axis0.watchdog_feed()
                self.velocities[i % 20] = self.motor.get_velocity()
                self.motor.odrv0.axis0.watchdog_feed()
                vel = np.mean(self.velocities[:min(i + 1, 20)])
                self.motor.odrv0.axis0.watchdog_feed()
                if abs(vel) < vel_min:
                    torque = self.linear_coeff * vel_min
                else:
                    torque = self.linear_coeff * abs(vel)
                self.motor.odrv0.axis0.watchdog_feed()
                i += 1
        print("Out of the linear thread")

    def change_instruction(self):
        control_mode = self.motor.get_control_mode()
        if control_mode == ControlMode.POWER_CONTROL:
            self.power = self.ui.instruction_spinBox.value()
        elif control_mode == ControlMode.VELOCITY_CONTROL:
            self.spin_box = self.instruction = self.ui.instruction_spinBox.value()
            self.ramp_instruction = self.ui.acceleration_spinBox.value()
            self.motor.velocity_control(self.spin_box, self.ramp_instruction)
        elif control_mode == ControlMode.TORQUE_CONTROL:
            self.spin_box = self.ui.instruction_spinBox.value()
            self.ramp_instruction = self.ui.acceleration_spinBox.value()
            self. instruction = self.motor.torque_control(self.spin_box, self.ramp_instruction)
        elif control_mode == ControlMode.LINEAR_CONTROL:
            self.linear_coeff = self.ui.instruction_spinBox.value()

    def _data(self):
        """
        To be called by a daemon thread.
        """
        while self.run:
            self.motor.odrv0.axis0.watchdog_feed()
            self.motor.save_data_to_file(f'XP/gui_{self.rd}',
                                         spin_box=self.ui.instruction_spinBox.value(),
                                         instruction=self.instruction,
                                         ramp_instruction=self.ramp_instruction,)
            self.motor.odrv0.axis0.watchdog_feed()

            self.ui.power_lineEdit.setText(f"{self.motor.get_user_power():.0f}")
            self.motor.odrv0.axis0.watchdog_feed()
            self.ui.velocity_lineEdit.setText(f"{self.motor.get_velocity():.0f}")
            self.motor.odrv0.axis0.watchdog_feed()
            self.ui.torque_lineEdit.setText(f"{self.motor.get_user_torque():.0f}")
            self.motor.odrv0.axis0.watchdog_feed()
            self.ui.errors_label.setText(
                f"{traduce_error(self.motor.odrv0.error, ODriveError)}"
                f"{traduce_error(self.motor.odrv0.axis0.error, ODriveAxisError)}"
                f"{traduce_error(self.motor.odrv0.axis0.controller.error, ODriveControllerError)}"
                f"{traduce_error(self.motor.odrv0.axis0.encoder.error, ODriveEncoderError)}"
                f"{traduce_error(self.motor.odrv0.axis0.motor.error, ODriveMotorError)}"
                f"{traduce_error(self.motor.odrv0.axis0.sensorless_estimator.error, ODriveSensorlessEstimatorError)}"
                f"{traduce_error(self.motor.odrv0.can.error, ODriveCanError)}"
                f"brake resistor armed: {self.motor.odrv0.brake_resistor_armed}, "
                f"brake resistor saturated: {self.motor.odrv0.brake_resistor_saturated}, "
                f"brake resistor current: {self.motor.odrv0.brake_resistor_current:.2f}"
            )
            self.motor.odrv0.axis0.watchdog_feed()
            if (
                    self.motor.get_control_mode() == ControlMode.VELOCITY_CONTROL
                    and abs(self.motor.get_iq_setpoint()) > 10.0
                    and abs(self.motor.get_velocity()) < 5.0
            ):
                self.motor.odrv0.axis0.watchdog_feed()
                # TODO: Please don't force too much against the motor.
                # vel_ecc_thread = threading.Thread(
                #     target=self._vel_ecc_thread, name="Security for velocity control", daemon=True
                # )
                #
                # self.motor.odrv0.axis0.watchdog_feed()
                #
                # vel_ecc_thread.start()

            # If the motor is in torque control, the torque input needs to be updated in function of the velocity
            # because of the resisting torque.
            # Furthermore, it allows to stop the pedals by reducing the torque if the user has stopped.
            if self.motor.get_control_mode() == ControlMode.TORQUE_CONTROL:
                self.motor.odrv0.axis0.watchdog_feed()
                self.instruction = self.motor.torque_control(self.spin_box, self.ramp_instruction)

            self.motor.odrv0.axis0.watchdog_feed()

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
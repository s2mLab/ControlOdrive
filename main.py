"""
Script to control the ergocycle through a graphical user interface.
"""
import os
import sys

import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import *
import pyqtgraph as pg

from ergocycle_gui import Ui_MainWindow

from motor import *
# from phantom import Phantom

from utils import (
    traduce_error,
    PlotWidget,
    SignalThread,
)

from enums import (
    ControlMode,
    DirectionMode,
    ODriveMotorError,
    ODriveSensorlessEstimatorError,
    ODriveError,
    ODriveAxisError,
    ODriveEncoderError,
    ODriveControllerError,
    ODriveCanError,
)
from gui_enums import (
    TrainingMode,
    GUIControlMode,
    StopwatchStates,
)


class App(QtWidgets.QMainWindow):
    def __init__(self, odrive_motor):
        super(App, self).__init__(parent=None)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.motor = odrive_motor
        self.motor.zero_position_calibration()

        # Plot
        self._gui_control_mode = GUIControlMode.POWER
        self.plot_power = PlotWidget(self, name="Power", y_label="Power (W)", color="g")
        self.ui.power_horizontalLayout.insertWidget(0, self.plot_power)
        self.plot_power.getAxis('bottom').setStyle(tickLength=0)
        self.plot_power.getAxis('bottom').setVisible(False)
        self.plot_cadence = PlotWidget(self, name="cadence", y_label="cadence rpm", color="r")
        self.ui.cadence_horizontalLayout.insertWidget(0, self.plot_cadence)
        self.plot_cadence.getAxis('bottom').setStyle(tickLength=0)
        self.plot_cadence.getAxis('bottom').setVisible(False)
        self.plot_torque = PlotWidget(self, name="Torque", y_label="Torque (N.m)", color="b")
        self.ui.torque_horizontalLayout.insertWidget(0, self.plot_torque)

        # Thread for the watchdog, data saving and control
        # There is only one thread to control the order of execution of the different tasks since there is only one
        # process. Otherwise, the watchdog risks to not be fed in time.
        self.motor_thread = MotorThread(self.ui, odrive_motor)
        self.motor_thread.plot_update_signal.connect(self._plot_update)
        self.motor_thread.start()

        # Colors
        self._color_red = QtGui.QColor(255, 150, 150)
        self._color_green = QtGui.QColor(150, 255, 150)
        self._color_blue = QtGui.QColor(180, 180, 255)
        self._color_default = self.ui.angle_reset_pushButton.palette().color(
            self.ui.angle_reset_pushButton.backgroundRole()
        )

        # Security
        self.ui.emergency_pushButton.clicked.connect(self.emergency_stop)

        # Modes
        # Training mode
        self.ui.training_comboBox.addItems([TrainingMode.CONCENTRIC.value, TrainingMode.ECCENTRIC.value])
        self.ui.training_comboBox.setCurrentIndex(0)
        self.ui.training_comboBox.activated.connect(self._update_instruction_display_on_training_mode_change)
        # Control mode
        self._update_instruction_display_on_training_mode_change()
        self.ui.control_comboBox.activated.connect(self._update_instruction_display_on_control_mode_change)
        # Direction of rotation
        self.ui.direction_comboBox.addItems([DirectionMode.FORWARD.value, DirectionMode.REVERSE.value])
        self.ui.direction_comboBox.setCurrentIndex(0)
        self.ui.direction_comboBox.activated.connect(self._set_instruction_to_0)

        # Command
        self._angle_reset_once = False
        self.ui.angle_reset_pushButton.clicked.connect(self._angle_reset)
        self.ui.start_update_pushButton.clicked.connect(self._control_update)
        self.ui.stop_pushButton.clicked.connect(self._control_stop)
        self.ui.stop_pushButton.setEnabled(False)
        self.ui.angle_reset_pushButton.setStyleSheet(f"background-color: {self._color_blue.name()}")

        # Saving
        self.saving = False
        self.watchdog_frequency = 10  # Hz
        self.watchdog_thread = SignalThread(self.watchdog_frequency)
        self.watchdog_thread.signal.connect(self._save_data)
        self.watchdog_thread.start()
        self.ui.save_lineEdit.textChanged.connect(self._check_text)
        self.ui.save_st_pushButton.clicked.connect(self._save_start_stop)
        self.ui.save_st_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")

        # Comments
        self.comment_to_save = False
        self.comment = ""
        self.ui.comments_save_pushButton.clicked.connect(self._comments_save)
        self.ui.comments_save_pushButton.setEnabled(False)
        self.ui.comments_lineEdit.setEnabled(False)

        # Stopwatch
        self._stopwatch_display()
        self.ui.stopwatch_start_stop_pushButton.clicked.connect(self._stopwatch_start_stop)
        self.ui.stopwatch_lap_reset_pushButton.clicked.connect(self._stopwatch_lap_reset)
        self.ui.stopwatch_lcdNumber.setDigitCount(8)
        self.ui.lap_lcdNumber.setDigitCount(5)

        # Style
        self.ui.start_update_pushButton.setStyleSheet(f"background-color: {self._color_green.name()};")
        self.ui.errors_label.setStyleSheet(f"font-weight: bold; color: red;")
        self.ui.emergency_pushButton.setStyleSheet(f"background-color: red; color: white;")

        self.dt = []

    def emergency_stop(self):
        """
        Emergency stop. Stop the motor and close the GUI.
        """
        self.watchdog_thread.stop()
        self.motor_thread.run = False
        self.close()

    def closeEvent(self, event):
        """
        Stop the motor on close.
        """
        self.watchdog_thread.stop()
        self.motor_thread.run = False
        event.accept()

    def _check_text(self, text):
        """
        Check if the text is correct for a file name.
        """
        new_text = ''
        for char in text:
            if char.isalnum() or char in ('_', '-'):
                new_text += char
        if new_text != text:
            cursor_pos = self.ui.save_lineEdit.cursorPosition()
            self.ui.save_lineEdit.setText(new_text)
            self.ui.save_lineEdit.setCursorPosition(cursor_pos - 1)

    def _update_instruction_display_on_training_mode_change(self):
        """
        Update the display accordingly to the training mode.
        """
        self._set_instruction_to_0()
        training_mode = self.ui.training_comboBox.currentText()
        index = self.ui.control_comboBox.currentIndex()
        # If there was no precedent index, set the GUIControlMode to POWER.
        if index == -1:
            index = 0
        self.ui.control_comboBox.clear()
        if training_mode == TrainingMode.CONCENTRIC.value:
            self.ui.control_comboBox.addItems([
                GUIControlMode.POWER.value,
                GUIControlMode.CADENCE.value,
                GUIControlMode.LINEAR.value,
                GUIControlMode.TORQUE.value])
            self.ui.direction_comboBox.setCurrentText(DirectionMode.FORWARD.value)
        elif training_mode == TrainingMode.ECCENTRIC.value:
            self.ui.control_comboBox.addItems([
                GUIControlMode.POWER.value,
                GUIControlMode.CADENCE.value])
            self.ui.direction_comboBox.setCurrentText(DirectionMode.REVERSE.value)
            # If the GUIControlMode corresponding to the precedent index was LINEAR or TORQUE, set the GUIControlMode to
            # POWER.
            if index >= 2:
                index = 0
        else:
            raise ValueError(f"{training_mode} training has not been implemented yet.")
        self.ui.control_comboBox.setCurrentIndex(index)
        self._update_instruction_display_on_control_mode_change()

    def _update_instruction_display_on_control_mode_change(self):
        """
        Update the display accordingly to the control mode.
        """
        self._set_instruction_to_0()
        power_step = 10
        vel_step = 10
        vel_ramp = 30
        torque_step = 1
        torque_ramp = 5
        linear_step = 0.1

        control_mode = self.ui.control_comboBox.currentText()
        if control_mode == GUIControlMode.POWER.value:
            self.ui.instruction_spinBox.setRange(0, self.motor.hardware_and_security["power_lim"])
            self.ui.instruction_spinBox.setSingleStep(power_step)
            self.ui.units_label.setText("W")

            training_mode = self.ui.training_comboBox.currentText()
            if training_mode == TrainingMode.CONCENTRIC.value:
                self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_ramp_rate_lim"]))
                self.ui.acceleration_spinBox.setSingleStep(torque_step)
                self.ui.acceleration_spinBox.setValue(torque_ramp)
                self.ui.acceleration_units_label.setText("N.m/s")
            elif training_mode == TrainingMode.ECCENTRIC.value:
                self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["pedals_accel_lim"]) - 1)
                self.ui.acceleration_spinBox.setSingleStep(vel_step)
                self.ui.acceleration_spinBox.setValue(vel_ramp)
                self.ui.acceleration_units_label.setText("rpm/s")
            else:
                raise ValueError(f"{training_mode} training has not been implemented yet.")

        elif control_mode == GUIControlMode.LINEAR.value:
            self.ui.instruction_spinBox.setRange(0, 1.0)
            self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_ramp_rate_lim"]))
            self.ui.instruction_spinBox.setSingleStep(linear_step)
            self.ui.acceleration_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setValue(torque_ramp)
            self.ui.units_label.setText("N.m/rpm")
            self.ui.acceleration_units_label.setText("N.m/s")

        elif control_mode == GUIControlMode.CADENCE.value:
            self.ui.instruction_spinBox.setRange(0, self.motor.hardware_and_security["pedals_vel_limit"] - 1)
            self.ui.acceleration_spinBox.setRange(0, self.motor.hardware_and_security["pedals_accel_lim"] - 1)
            self.ui.instruction_spinBox.setSingleStep(vel_step)
            self.ui.acceleration_spinBox.setSingleStep(vel_step)
            self.ui.acceleration_spinBox.setValue(vel_ramp)
            self.ui.units_label.setText("rpm")
            self.ui.acceleration_units_label.setText("rpm/s")

        elif control_mode == GUIControlMode.TORQUE.value:
            self.ui.instruction_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_lim"] - 1))
            self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_ramp_rate_lim"]))
            self.ui.instruction_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setValue(torque_ramp)
            self.ui.units_label.setText("N.m")
            self.ui.acceleration_units_label.setText("N.m/s")

        else:
            raise ValueError(f"{control_mode} control has not been implemented yet.")

    def _set_instruction_to_0(self):
        """
        Set the instruction to 0.
        """
        self.ui.instruction_spinBox.setValue(0)

    def _angle_reset(self):
        """
        Reset the angle to 0.
        """
        self.motor_thread.motor.zero_position_calibration()
        self._angle_reset_once = True
        self.ui.angle_reset_pushButton.setStyleSheet(f"background-color: {self._color_default.name()}")

    def _control_update(self):
        """
        Start or update the control depending on the control mode and the training mode.
        """
        # If the motor is not started: change the display, get the control and training modes and set the direction.
        if not self.motor_thread.motor_started:
            if not self._angle_reset_once:
                self._angle_reset()
            self._control_display()
            self._plot_add_instruction()
            self.motor_thread.plot_start_time = time.time()
            self.motor_thread.time_array = np.linspace(
                - self.motor_thread.size_arrays / self.motor_thread.plot_frequency, 0, self.motor_thread.size_arrays
            )

        # Update instructions and the control depending on the control mode.
        self.motor_thread.ramp_instruction = self.ui.acceleration_spinBox.value()

        if self._gui_control_mode == GUIControlMode.POWER:
            if self._training_mode == TrainingMode.CONCENTRIC.value:
                self.motor_thread.spin_box = self.ui.instruction_spinBox.value()
                self.motor_thread.instruction = self.motor.linear_control(
                    self.motor_thread.spin_box, self.motor_thread.ramp_instruction
                )
            elif self._training_mode == TrainingMode.ECCENTRIC.value:
                # In eccentric mode, the sign is inverted because the user's power is negative
                self.motor_thread.spin_box = - self.ui.instruction_spinBox.value()
                self.motor_thread.instruction = self.motor.eccentric_power_control(
                    self.motor_thread.spin_box, self.motor_thread.ramp_instruction
                )
            else:
                raise ValueError(f"{self._training_mode} training has not been implemented yet.")

        elif self._gui_control_mode == GUIControlMode.LINEAR:
            self.motor_thread.spin_box = self.ui.instruction_spinBox.value()
            self.motor_thread.instruction = self.motor.linear_control(
                self.motor_thread.spin_box, self.motor_thread.ramp_instruction
            )

        elif self._gui_control_mode == GUIControlMode.CADENCE:
            self.motor_thread.instruction = - self.motor.get_sign() * self.ui.instruction_spinBox.value()
            self.motor_thread.spin_box = self.motor_thread.instruction
            self.motor.cadence_control(
                self.motor_thread.spin_box, self.motor_thread.ramp_instruction
            )

        elif self._gui_control_mode == GUIControlMode.TORQUE:
            self.motor_thread.spin_box = - self.motor.get_sign() * self.ui.instruction_spinBox.value()
            self.motor_thread.instruction = self.motor.torque_control(
                self.motor_thread.spin_box, self.motor_thread.ramp_instruction
            )

        else:
            raise ValueError(f"{self._gui_control_mode} control has not been implemented yet.")

    def _control_stop(self):
        """
        Start the stopping procedure and adapt the GUI accordingly.
        """
        ramp_instruction = self.motor_thread.ramp_instruction

        self._control_display()
        self._plot_remove_instruction()
        self.ui.start_update_pushButton.setEnabled(False)
        self.motor_thread.instruction = 0.0
        self.motor_thread.spin_box = 0.0
        self.motor_thread.ramp_instruction = 0.0

        # If the motor is based on cadence control, the ramp is in rpm/s. It can be given to motor.stopping(),
        # else it won't be used. Either way it is protected by the min() function.
        self.motor.stopping(cadence_ramp_rate=ramp_instruction)

    def _control_display(self):
        """
        Change the display when starting or stopping the motor.
        If starting the motor get the control and training modes and set the direction.
        """
        self.motor_thread.motor_started = not self.motor_thread.motor_started
        self.ui.training_comboBox.setEnabled(not self.motor_thread.motor_started)
        self.ui.control_comboBox.setEnabled(not self.motor_thread.motor_started)
        self.ui.direction_comboBox.setEnabled(not self.motor_thread.motor_started)
        self.ui.stop_pushButton.setEnabled(self.motor_thread.motor_started)
        if self.motor_thread.motor_started:
            self._gui_control_mode = self.ui.control_comboBox.currentText()
            self._training_mode = self.ui.training_comboBox.currentText()
            self.motor.set_direction(self.ui.direction_comboBox.currentText())
            self.ui.start_update_pushButton.setText("Update")
            self.ui.start_update_pushButton.setStyleSheet(f"background-color: {self._color_blue.name()};")
            self.ui.stop_pushButton.setStyleSheet(f"background-color: {self._color_red.name()};")
            self.motor_thread.motor_started = True
        else:
            self.ui.start_update_pushButton.setText("Start")
            self.ui.start_update_pushButton.setStyleSheet(f"background-color: {self._color_green.name()};")
            self.ui.stop_pushButton.setStyleSheet(f"background-color: {self._color_default.name()};")
            self.motor_thread.motor_started = False

    def _save_start_stop(self):
        """
        Start or stop saving the data to a file.
        """
        # Choosing the file name at the beginning of the saving.
        self.file_name = f"XP/{self.ui.save_lineEdit.text()}"
        ext = ".bio"
        if os.path.isfile(f"{self.file_name}{ext}"):
            # File already exists, add a suffix to the filename
            i = 1
            while os.path.isfile(f"{self.file_name}({i}){ext}"):
                i += 1
            self.file_name = f"{self.file_name}({i})"

        self.saving = not self.saving
        self.ui.save_lineEdit.setEnabled(not self.saving)
        self.ui.comments_save_pushButton.setEnabled(self.saving)
        self.ui.comments_lineEdit.setEnabled(self.saving)
        if self.saving:
            self.ui.save_st_pushButton.setText("Stop saving")
            self.ui.save_st_pushButton.setStyleSheet(f"background-color: {self._color_red.name()}")
            self.ui.comments_save_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")
        else:
            self.ui.save_st_pushButton.setText("Start saving")
            self.ui.save_st_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")
            self.ui.comments_save_pushButton.setStyleSheet(f"background-color: {self._color_default.name()}")

    def _comments_save(self):
        """
        Indicates that the comment needs to be saved and clear the comment line edit.
        """
        self.comment_to_save = True
        self.comment = self.ui.comments_lineEdit.text()
        self.ui.comments_lineEdit.setText("")

    def _stopwatch_start_stop(self):
        """
        Start, stop or pause the stopwatch.
        """
        if self.motor_thread.stopwatch_state == StopwatchStates.PAUSED:
            self.motor_thread.stopwatch_state = StopwatchStates.RUNNING
            self.motor_thread.stopwatch_start_time += (time.time() - self.motor_thread.stopwatch_pause_time)
            self.motor_thread.stopwatch_lap_time += (time.time() - self.motor_thread.stopwatch_pause_time)

        elif self.motor_thread.stopwatch_state == StopwatchStates.RUNNING:
            self.motor_thread.stopwatch_state = StopwatchStates.PAUSED
            self.motor_thread.stopwatch_pause_time = time.time()

        elif self.motor_thread.stopwatch_state == StopwatchStates.STOPPED:
            self.motor_thread.stopwatch_state = StopwatchStates.RUNNING
            self.motor_thread.stopwatch_start_time = time.time()
            self.motor_thread.stopwatch_lap_time = time.time()

        self._stopwatch_display()

    def _stopwatch_lap_reset(self):
        """
        Reset the stopwatch if it is paused, otherwise, save the lap time.
        """
        if self.motor_thread.stopwatch_state == StopwatchStates.RUNNING:
            self.motor_thread.stopwatch_lap_time = time.time()

        elif self.motor_thread.stopwatch_state == StopwatchStates.PAUSED:
            self.motor_thread.stopwatch_state = StopwatchStates.STOPPED

        self._stopwatch_display()

    def _stopwatch_display(self):
        """
        Update the stopwatch display accordingly to the stopwatch state.
        """
        if self.motor_thread.stopwatch_state == StopwatchStates.PAUSED:
            self.ui.stopwatch_lap_reset_pushButton.setText("Reset")
            self.ui.stopwatch_start_stop_pushButton.setText("Start")
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f'background-color: {self._color_green.name()}')

        elif self.motor_thread.stopwatch_state == StopwatchStates.RUNNING:
            self.ui.stopwatch_lap_reset_pushButton.setEnabled(True)
            self.ui.stopwatch_lap_reset_pushButton.setText("Lap")
            self.ui.stopwatch_start_stop_pushButton.setText("Stop")
            self.ui.stopwatch_lap_reset_pushButton.setStyleSheet(f'background-color: {self._color_blue.name()}')
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f'background-color: {self._color_red.name()}')

        elif self.motor_thread.stopwatch_state == StopwatchStates.STOPPED:
            self.ui.stopwatch_lap_reset_pushButton.setEnabled(False)
            self.ui.stopwatch_lap_reset_pushButton.setText("Lap")
            self.ui.stopwatch_start_stop_pushButton.setText("Start")
            self.ui.stopwatch_lap_reset_pushButton.setStyleSheet(f'background-color: {self._color_default.name()}')
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f'background-color: {self._color_green.name()}')

    def _plot_add_instruction(self):
        """
        Add an instruction to the plot.
        """
        if self._gui_control_mode == GUIControlMode.POWER:
            self.plot_power.add_instruction()
        elif self._gui_control_mode == GUIControlMode.CADENCE:
            self.plot_cadence.add_instruction()
        elif self._gui_control_mode == GUIControlMode.TORQUE:
            self.plot_torque.add_instruction()

    def _plot_remove_instruction(self):
        """
        Remove the instruction from the plot.
        """
        if self._gui_control_mode == GUIControlMode.POWER:
            self.plot_power.remove_instruction()
        elif self._gui_control_mode == GUIControlMode.CADENCE:
            self.plot_cadence.remove_instruction()
        elif self._gui_control_mode == GUIControlMode.TORQUE:
            self.plot_torque.remove_instruction()

    def _plot_update(self):
        """
        Update the plot with the new data.
        """
        power_spin_box_array = None
        cadence_spin_box_array = None
        torque_spin_box_array = None
        if self._gui_control_mode == GUIControlMode.POWER:
            power_spin_box_array = self.motor_thread.spin_box_array
        elif self._gui_control_mode == GUIControlMode.CADENCE:
            cadence_spin_box_array = self.motor_thread.spin_box_array
        elif self._gui_control_mode == GUIControlMode.TORQUE:
            torque_spin_box_array = self.motor_thread.spin_box_array

        self.plot_power.update_plot(
            self.motor_thread.time_array,
            self.motor_thread.power_array,
            power_spin_box_array,
        )
        self.plot_cadence.update_plot(
            self.motor_thread.time_array,
            self.motor_thread.cadence_array,
            cadence_spin_box_array,
        )
        self.plot_torque.update_plot(
            self.motor_thread.time_array,
            self.motor_thread.torque_array,
            torque_spin_box_array,
        )

    def _save_data(self):
        self.motor.odrv0.axis0.watchdog_feed()

        if self.saving:
            t0 = time.time()
            # Save data
            if self.comment_to_save:
                comment = self.comment
                self.comment_to_save = False
            else:
                comment = ""
            self.motor.save_data_to_file(
                self.file_name,
                spin_box=self.motor_thread.spin_box,
                instruction=self.motor_thread.instruction,
                ramp_instruction=self.motor_thread.ramp_instruction,
                comment=comment,
                stopwatch=self.motor_thread.stopwatch,
                lap=self.motor_thread.lap,
            )
            self.dt.append(time.time() - t0)
            self.motor.odrv0.axis0.watchdog_feed()


class MotorThread(QtCore.QThread):
    """
    Thread that controls the motor. It is separated from the GUI thread to avoid freezing the GUI when the motor is
    running.
    """
    plot_update_signal = QtCore.pyqtSignal(name="plot_update_signal")

    def __init__(self, ui: Ui_MainWindow, odrive_motor):
        super(MotorThread, self).__init__(parent=None)
        self.ui = ui
        self.motor = odrive_motor

        # Security
        self.run = True

        # Control
        self.motor_started = False

        # Saving
        self.instruction = 0.0
        self.ramp_instruction = 0.0
        self.spin_box = 0.0

        # Comments
        self.comment_to_save = False
        self.comment = ""

        # Stopwatch
        self.stopwatch_start_time = 0.0
        self.stopwatch_pause_time = 0.0
        self.stopwatch_lap_time = 0.0
        self.stopwatch_state = StopwatchStates.STOPPED
        self.stopwatch = 0.0
        self.lap = 0.0

        # Data
        self._display_frequency = 4  # Hz
        self._save_frequency = 5  # Hz

        # Plot
        self.plot_start_time = 0.0
        self.plot_frequency = 2  # Hz
        self.size_arrays = 10 * self.plot_frequency  # 10 seconds
        self.time_array = np.linspace(- self.size_arrays / self.plot_frequency, 0, self.size_arrays)
        self.cadence_array = np.zeros(self.size_arrays)
        self.torque_array = np.zeros(self.size_arrays)
        self.power_array = np.zeros(self.size_arrays)
        self.spin_box_array = None

    def run(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """
        t_plot_precedent = t_display_precedent = time.time()
        self.plot_start_time = time.time()
        date_time = QtCore.QDateTime()

        while self.run:
            self.motor.odrv0.axis0.watchdog_feed()

            # Date
            current_time = date_time.currentDateTime().toString('yyyy-MM-dd hh:mm:ss')
            self.ui.date_label.setText(current_time)

            # Stopwatch
            if self.stopwatch_state == StopwatchStates.RUNNING:
                self.stopwatch = time.time() - self.stopwatch_start_time
                self.lap = time.time() - self.stopwatch_lap_time
            elif self.stopwatch_state == StopwatchStates.PAUSED:
                self.stopwatch = self.stopwatch_pause_time - self.stopwatch_start_time
                self.lap = self.stopwatch_pause_time - self.stopwatch_lap_time
            elif self.stopwatch_state == StopwatchStates.STOPPED:
                self.stopwatch = 0.0
                self.lap = 0.0

            minutes = int(self.stopwatch // 60)  # get the integer part of the quotient
            seconds = int(self.stopwatch % 60)  # get the integer part of the remainder
            milliseconds = int(
                (self.stopwatch - int(self.stopwatch)) * 100)  # get the milliseconds component
            self.ui.stopwatch_lcdNumber.display(f"{minutes:02d}:{seconds:02d}.{milliseconds:02d}")
            minutes = int(self.lap // 60)  # get the integer part of the quotient
            seconds = int(self.lap % 60)  # get the integer part of the remainder
            self.ui.lap_lcdNumber.display(f"{minutes:02d}:{seconds:02d}")

            # Display data
            t_since_precedent_display = time.time() - t_display_precedent
            if t_since_precedent_display > 1 / self._display_frequency:
                self.ui.power_display.setText(f"{self.motor.get_user_power():.0f} W")
                self.ui.cadence_display.setText(f"{self.motor.get_cadence():.0f} rpm")
                self.ui.torque_display.setText(f"{self.motor.get_user_torque():.0f} N.m")
                self.ui.turns_display.setText(f"{self.motor.get_turns():.0f} tr")
                self.ui.angle_display.setText(f"{self.motor.get_angle():.0f} °")
                t_display_precedent = time.time()

            self.ui.errors_label.setText(
                f"{traduce_error(self.motor.odrv0.error, ODriveError)}"
                f"{traduce_error(self.motor.odrv0.axis0.error, ODriveAxisError)}"
                f"{traduce_error(self.motor.odrv0.axis0.controller.error, ODriveControllerError)}"
                f"{traduce_error(self.motor.odrv0.axis0.encoder.error, ODriveEncoderError)}"
                f"{traduce_error(self.motor.odrv0.axis0.motor.error, ODriveMotorError)}"
                f"{traduce_error(self.motor.odrv0.axis0.sensorless_estimator.error, ODriveSensorlessEstimatorError)}"
                f"{traduce_error(self.motor.odrv0.can.error, ODriveCanError)}"
            )

            self.motor.odrv0.axis0.watchdog_feed()

            # Adapt the control of the motor accordingly to the current cadence and torque
            control_mode = self.motor.get_control_mode()
            # If the motor is in torque control, the torque input needs to be updated in function of the cadence
            # because of the resisting torque.
            # Furthermore, it allows to stop the pedals by reducing the torque if the user has stopped.
            if control_mode == ControlMode.TORQUE_CONTROL:
                self.instruction = self.motor.torque_control(self.spin_box, self.ramp_instruction)

            # The concentric power control mode is based on the torque control mode, but the torque input is calculated
            # from the current cadence (torque_input = f(power / cadence, resiting torque)).
            elif control_mode == ControlMode.CONCENTRIC_POWER_CONTROL:
                self.instruction = self.motor.concentric_power_control(self.spin_box, self.ramp_instruction)

            # The linear control mode is based on the torque control mode, but the torque input is calculated from the
            # current cadence (torque_input = linear_coeff * cadence and resiting torque).
            elif control_mode == ControlMode.LINEAR_CONTROL:
                self.instruction = self.motor.linear_control(self.spin_box, self.ramp_instruction)

            # The concentric power control mode is based on the cadence control mode, but the cadence input is
            # calculated from the current torque (cadence_input = f(power / torque, resiting torque)).
            elif control_mode == ControlMode.ECCENTRIC_POWER_CONTROL:
                self.instruction = self.motor.eccentric_power_control(self.spin_box,
                                                                      self.ramp_instruction)

            elif self.motor.get_control_mode() == ControlMode.STOPPING:
                if abs(motor.get_cadence()) < 10.0:
                    self.motor.stopped()
                    self.ui.start_update_pushButton.setEnabled(True)

            self.motor.odrv0.axis0.watchdog_feed()

            # Plot data
            t_since_precedent_plot = time.time() - t_plot_precedent
            if t_since_precedent_plot > 1 / self.plot_frequency:
                self.time_array = np.roll(self.time_array, -1)
                self.time_array[-1] = time.time() - self.plot_start_time
                self.cadence_array = np.roll(self.cadence_array, -1)
                self.cadence_array[-1] = self.motor.get_cadence()
                self.torque_array = np.roll(self.torque_array, -1)
                self.torque_array[-1] = self.motor.get_user_torque()
                self.power_array = np.roll(self.power_array, -1)
                self.power_array[-1] = self.motor.get_user_power()
                if self.motor_started:
                    if self.spin_box_array is None:
                        self.spin_box_array = np.zeros(self.size_arrays)
                    self.spin_box_array = np.roll(self.spin_box_array, -1)
                    self.spin_box_array[-1] = self.spin_box
                else:
                    self.spin_box_array = None
                self.plot_update_signal.emit()
                t_plot_precedent = time.time()


if __name__ == "__main__":
    motor = OdriveEncoderHall(enable_watchdog=True, external_watchdog=True)
    app = QApplication(sys.argv)
    gui = App(motor)
    gui.show()
    gui.motor.config_watchdog(True, 0.3)
    app.exec()
    app.run = False
    dt = np.asarray(gui.dt)
    print(np.sum(dt) / len(dt))

# python -m PyQt5.uic.pyuic -x ergocycle_gui.ui -o ergocycle_gui.py
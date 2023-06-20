import multiprocessing as mp
import numpy as np
import os
import sys
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from ctypes import c_bool, c_double

from ergocycleS2M.gui.ergocycle_gui import Ui_MainWindow
from ergocycleS2M.motor_control.enums import (
    DirectionMode,
)
from ergocycleS2M.gui.gui_enums import (
    GUIControlMode,
    StopwatchStates,
    TrainingMode,
)
from ergocycleS2M.gui.gui_utils import (
    PlotWidget,
)
from ergocycleS2M.motor_control.enums import ControlMode
from ergocycleS2M.motor_control.motor_computations import MotorComputations


class ErgocycleGUI(QtWidgets.QMainWindow):
    def __init__(
        self,
        run: mp.Manager().Value,
        instruction: mp.Manager().Value,
        ramp_instruction: mp.Manager().Value,
        spin_box: mp.Manager().Value,
        stopping: mp.Manager().Value,
        saving: mp.Manager().Value,
        queue_comment: mp.Pipe,
        i_measured: mp.Manager().Value,
        turns: mp.Manager().Value,
        vel_estimate: mp.Manager().Value,
        queue_instructions: mp.Queue,
        zero_position: mp.Manager().Value,
    ):
        super(ErgocycleGUI, self).__init__(parent=None)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.motor_computations = MotorComputations()

        # Shared memory
        self.run = run
        self.instruction = instruction
        self.ramp_instruction = ramp_instruction
        self.spin_box = spin_box
        self.stopping = stopping
        self.saving = saving
        self.queue_comment = queue_comment
        self.i_measured = i_measured
        self.turns = turns
        self.vel_estimate = vel_estimate
        self.queue_instruction = queue_instructions
        self.zero_position = zero_position

        self.motor_started = False

        # Plot
        self._gui_control_mode = GUIControlMode.POWER
        self.plot_power = PlotWidget(self, name="Power", y_label="Power (W)", color="g", show_x_axis=False)
        self.ui.power_horizontalLayout.insertWidget(0, self.plot_power)
        self.plot_power.getAxis("bottom").setStyle(tickLength=0)
        self.plot_cadence = PlotWidget(self, name="Cadence", y_label="Cadence (rpm)", color="r", show_x_axis=False)
        self.ui.cadence_horizontalLayout.insertWidget(0, self.plot_cadence)
        self.plot_cadence.getAxis("bottom").setStyle(tickLength=0)
        self.plot_torque = PlotWidget(self, name="Torque", y_label="Torque (N.m)", color="b", y_axis_range=5)
        self.ui.torque_horizontalLayout.insertWidget(0, self.plot_torque)
        self.plot_start_time = 0.0
        self.plot_frequency = 2  # Hz
        self.size_arrays = 20 * self.plot_frequency  # 10 seconds
        self.time_array = np.linspace(-self.size_arrays / self.plot_frequency, 0, self.size_arrays)
        self.cadence_array = np.zeros(self.size_arrays)
        self.torque_array = np.zeros(self.size_arrays)
        self.power_array = np.zeros(self.size_arrays)
        self.spin_box_array = None

        # Colors
        self._color_red = QtGui.QColor(255, 150, 150)
        self._color_green = QtGui.QColor(150, 255, 150)
        self._color_blue = QtGui.QColor(180, 180, 255)
        self._color_default = self.ui.angle_reset_pushButton.palette().color(
            self.ui.angle_reset_pushButton.backgroundRole()
        )
        self._color_grey = QtGui.QColor(220, 220, 220)
        self.ui.power_label.setStyleSheet(f"background-color: {self._color_grey.name()}")
        self.ui.cadence_label.setStyleSheet(f"background-color: {self._color_grey.name()}")
        self.ui.torque_label.setStyleSheet(f"background-color: {self._color_grey.name()}")

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
        self.ui.save_lineEdit.textChanged.connect(self._check_text)
        self.ui.save_st_pushButton.clicked.connect(self._save_start_stop)
        self.ui.save_st_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")
        self.training_mode = self.ui.training_comboBox.currentText()

        # Comments
        self.ui.comments_save_pushButton.clicked.connect(self._comments_save)
        self.ui.comments_save_pushButton.setEnabled(False)
        self.ui.comments_lineEdit.setEnabled(False)

        # Stopwatch
        self.stopwatch_start_time = 0.0
        self.stopwatch_pause_time = 0.0
        self.stopwatch_lap_time = 0.0
        self.stopwatch_state = StopwatchStates.STOPPED
        self.stopwatch = 0.0
        self.lap = 0.0
        self._stopwatch_display()
        self.ui.stopwatch_start_stop_pushButton.clicked.connect(self._stopwatch_start_stop)
        self.ui.stopwatch_lap_reset_pushButton.clicked.connect(self._stopwatch_lap_reset)
        self.ui.stopwatch_lcdNumber.setDigitCount(8)
        self.ui.lap_lcdNumber.setDigitCount(5)

        # Style
        self.ui.start_update_pushButton.setStyleSheet(f"background-color: {self._color_green.name()};")
        self.ui.errors_label.setStyleSheet(f"font-weight: bold; color: red;")
        self.ui.emergency_pushButton.setStyleSheet(f"background-color: red; color: white;")

        # Data
        self._display_frequency = 4  # Hz
        self._save_frequency = 5  # Hz

        self.plot_start_time = QtCore.QDateTime.currentMSecsSinceEpoch() / 1000


        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.loop)  # Connect the timeout signal to update_label method
        self.timer.start(100)  # Update the label every 1000 milliseconds (1 second)

    def emergency_stop(self):
        """
        Emergency stop. Stop the motor and close the GUI.
        """
        self.run.value = False
        self.close()

    def closeEvent(self, event):
        """
        Stop the motor on close.
        """
        self.run.value = False
        event.accept()

    def _check_text(self, text):
        """
        Check if the text is correct for a file name.
        """
        new_text = ""
        for char in text:
            if char.isalnum() or char in ("_", "-"):
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
            self.ui.control_comboBox.addItems(
                [
                    GUIControlMode.POWER.value,
                    GUIControlMode.CADENCE.value,
                    GUIControlMode.LINEAR.value,
                    GUIControlMode.TORQUE.value,
                ]
            )
            self.ui.direction_comboBox.setCurrentText(DirectionMode.FORWARD.value)
        elif training_mode == TrainingMode.ECCENTRIC.value:
            self.ui.control_comboBox.addItems([GUIControlMode.POWER.value, GUIControlMode.CADENCE.value])
            self.ui.direction_comboBox.setCurrentText(DirectionMode.REVERSE.value)
            # If the GUIControlMode corresponding to the precedent index was LINEAR or TORQUE, set the GUIControlMode to
            # POWER.
            if index >= 2:
                index = 0
        else:
            raise NotImplementedError(f"{training_mode} training has not been implemented yet.")
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
            self.ui.instruction_spinBox.setRange(0, self.motor_computations.hardware_and_security["power_lim"])
            self.ui.instruction_spinBox.setSingleStep(power_step)
            self.ui.instruction_label.setText("Power instruction")
            self.ui.units_label.setText("W")

            training_mode = self.ui.training_comboBox.currentText()
            if training_mode == TrainingMode.CONCENTRIC.value:
                self.ui.acceleration_spinBox.setRange(
                    0, int(self.motor_computations.hardware_and_security["torque_ramp_rate_lim"])
                )
                self.ui.acceleration_spinBox.setSingleStep(torque_step)
                self.ui.acceleration_spinBox.setValue(torque_ramp)
                self.ui.ramp_label.setText("Torque ramp")
                self.ui.acceleration_units_label.setText("N.m/s")
            elif training_mode == TrainingMode.ECCENTRIC.value:
                self.ui.acceleration_spinBox.setRange(
                    0, int(self.motor_computations.hardware_and_security["pedals_accel_lim"]) - 1
                )
                self.ui.acceleration_spinBox.setSingleStep(vel_step)
                self.ui.acceleration_spinBox.setValue(vel_ramp)
                self.ui.ramp_label.setText("Cadence ramp")
                self.ui.acceleration_units_label.setText("rpm/s")
            else:
                raise NotImplementedError(f"{training_mode} training has not been implemented yet.")

        elif control_mode == GUIControlMode.LINEAR.value:
            self.ui.instruction_spinBox.setRange(0, 1.0)
            self.ui.acceleration_spinBox.setRange(
                0, int(self.motor_computations.hardware_and_security["torque_ramp_rate_lim"])
            )
            self.ui.instruction_spinBox.setSingleStep(linear_step)
            self.ui.acceleration_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setValue(torque_ramp)
            self.ui.instruction_label.setText("Linear coefficient")
            self.ui.units_label.setText("N.m/rpm")
            self.ui.ramp_label.setText("Torque ramp")
            self.ui.acceleration_units_label.setText("N.m/s")

        elif control_mode == GUIControlMode.CADENCE.value:
            self.ui.instruction_spinBox.setRange(
                0, self.motor_computations.hardware_and_security["pedals_vel_limit"] - 1
            )
            self.ui.acceleration_spinBox.setRange(
                0, self.motor_computations.hardware_and_security["pedals_accel_lim"] - 1
            )
            self.ui.instruction_spinBox.setSingleStep(vel_step)
            self.ui.acceleration_spinBox.setSingleStep(vel_step)
            self.ui.acceleration_spinBox.setValue(vel_ramp)
            self.ui.instruction_label.setText("Cadence instruction")
            self.ui.units_label.setText("rpm")
            self.ui.ramp_label.setText("Cadence ramp")
            self.ui.acceleration_units_label.setText("rpm/s")

        elif control_mode == GUIControlMode.TORQUE.value:
            self.ui.instruction_spinBox.setRange(
                0, int(self.motor_computations.hardware_and_security["torque_lim"] - 1)
            )
            self.ui.acceleration_spinBox.setRange(
                0, int(self.motor_computations.hardware_and_security["torque_ramp_rate_lim"])
            )
            self.ui.instruction_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setValue(torque_ramp)
            self.ui.instruction_label.setText("Torque instruction")
            self.ui.units_label.setText("N.m")
            self.ui.ramp_label.setText("Torque ramp")
            self.ui.acceleration_units_label.setText("N.m/s")

        else:
            raise NotImplementedError(f"{control_mode} control has not been implemented yet.")

    def _set_instruction_to_0(self):
        """
        Set the instruction to 0.
        """
        self.ui.instruction_spinBox.setValue(0)

    def _angle_reset(self):
        """
        Reset the angle to 0.
        """
        self.zero_position.value = True
        self._angle_reset_once = True
        self.ui.angle_reset_pushButton.setStyleSheet(f"background-color: {self._color_default.name()}")

    @staticmethod
    def _get_sign(direction) -> int:
        """
        Set the sign of the rotation depending on the rotation direction (reverse or forward).
        """
        if direction == DirectionMode.FORWARD.value:
            return 1
        else:
            return -1

    def _control_update(self):
        """
        Start or update the control depending on the control mode and the training mode.
        """
        # If the motor is not started: change the display, get the control and training modes and set the direction.
        if not self.motor_started:
            if not self._angle_reset_once:
                self._angle_reset()
            self._control_display()
            self._plot_add_instruction()
            self.training_mode = self.ui.training_comboBox.currentText()
            self.plot_start_time = time.time()
            self.time_array = np.linspace(-self.size_arrays / self.plot_frequency, 0, self.size_arrays)

        # Update instructions and the control depending on the control mode.
        self.ramp_instruction.value = self.ui.acceleration_spinBox.value()

        if self._gui_control_mode == GUIControlMode.POWER:
            if self._training_mode == TrainingMode.CONCENTRIC.value:
                self.spin_box.value = self.ui.instruction_spinBox.value()
                self.queue_instruction.put_nowait(
                    (ControlMode.CONCENTRIC_POWER_CONTROL, self.ui.direction_comboBox.currentText())
                )
            elif self._training_mode == TrainingMode.ECCENTRIC.value:
                # In eccentric mode, the sign is inverted because the user's power is negative
                self.spin_box.value = -self.ui.instruction_spinBox.value()
                self.queue_instruction.put_nowait(
                    (ControlMode.ECCENTRIC_POWER_CONTROL, self.ui.direction_comboBox.currentText())
                )
            else:
                raise NotImplementedError(f"{self._training_mode} training has not been implemented yet.")

        elif self._gui_control_mode == GUIControlMode.LINEAR:
            self.spin_box.value = self.ui.instruction_spinBox.value()
            self.queue_instruction.put_nowait((ControlMode.LINEAR_CONTROL, self.ui.direction_comboBox.currentText()))

        elif self._gui_control_mode == GUIControlMode.CADENCE:
            self.instruction.value = (
                self._get_sign(self.ui.direction_comboBox.currentText()) * self.ui.instruction_spinBox.value()
            )
            self.spin_box.value = self.instruction.value
            self.queue_instruction.put_nowait((ControlMode.CADENCE_CONTROL, self.ui.direction_comboBox.currentText()))

        elif self._gui_control_mode == GUIControlMode.TORQUE:
            self.spin_box.value = (
                self._get_sign(self.ui.direction_comboBox.currentText()) * self.ui.instruction_spinBox.value()
            )
            self.queue_instruction.put_nowait((ControlMode.TORQUE_CONTROL, self.ui.direction_comboBox.currentText()))

        else:
            raise NotImplementedError(f"{self._gui_control_mode} control has not been implemented yet.")

    def _control_stop(self):
        """
        Start the stopping procedure and adapt the GUI accordingly.
        """
        ramp_instruction = self.ramp_instruction.value

        self._control_display()
        self._plot_remove_instruction()
        self.ui.start_update_pushButton.setEnabled(False)
        self.instruction.value = 0.0
        self.spin_box.value = 0.0
        self.ramp_instruction.value = 0.0

        # If the motor is based on cadence control, the ramp is in rpm/s. It can be given to motor.stopping(),
        # else it won't be used. Either way it is protected by the min() function.
        self.stopping.value = True

    def _control_display(self):
        """
        Change the display when starting or stopping the motor.
        If starting the motor get the control and training modes and set the direction.
        """
        self.motor_started = not self.motor_started
        self.ui.training_comboBox.setEnabled(not self.motor_started)
        self.ui.control_comboBox.setEnabled(not self.motor_started)
        self.ui.direction_comboBox.setEnabled(not self.motor_started)
        self.ui.stop_pushButton.setEnabled(self.motor_started)
        if self.motor_started:
            self._gui_control_mode = self.ui.control_comboBox.currentText()
            self._training_mode = self.ui.training_comboBox.currentText()
            self.ui.start_update_pushButton.setText("Update")
            self.ui.start_update_pushButton.setStyleSheet(f"background-color: {self._color_blue.name()};")
            self.ui.stop_pushButton.setStyleSheet(f"background-color: {self._color_red.name()};")
            self.motor_started = True
        else:
            self.ui.start_update_pushButton.setText("Start")
            self.ui.start_update_pushButton.setStyleSheet(f"background-color: {self._color_green.name()};")
            self.ui.stop_pushButton.setStyleSheet(f"background-color: {self._color_default.name()};")
            self.motor_started = False

    def _save_start_stop(self):
        """
        Start or stop saving the data to a file.
        """
        self.saving.value = not self.saving.value
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
        self.queue_comment.put_nowait(self.ui.comments_lineEdit.text())
        self.ui.comments_lineEdit.setText("")

    def _stopwatch_start_stop(self):
        """
        Start, stop or pause the stopwatch.
        """
        if self.stopwatch_state == StopwatchStates.PAUSED:
            self.stopwatch_state = StopwatchStates.RUNNING
            self.stopwatch_start_time += time.time() - self.stopwatch_pause_time
            self.stopwatch_lap_time += time.time() - self.stopwatch_pause_time

        elif self.stopwatch_state == StopwatchStates.RUNNING:
            self.stopwatch_state = StopwatchStates.PAUSED
            self.stopwatch_pause_time = time.time()

        elif self.stopwatch_state == StopwatchStates.STOPPED:
            self.stopwatch_state = StopwatchStates.RUNNING
            self.stopwatch_start_time = time.time()
            self.stopwatch_lap_time = time.time()

        self._stopwatch_display()

    def _stopwatch_lap_reset(self):
        """
        Reset the stopwatch if it is paused, otherwise, save the lap time.
        """
        if self.stopwatch_state == StopwatchStates.RUNNING:
            self.stopwatch_lap_time = time.time()

        elif self.stopwatch_state == StopwatchStates.PAUSED:
            self.stopwatch_state = StopwatchStates.STOPPED

        self._stopwatch_display()

    def _stopwatch_display(self):
        """
        Update the stopwatch display accordingly to the stopwatch state.
        """
        if self.stopwatch_state == StopwatchStates.PAUSED:
            self.ui.stopwatch_lap_reset_pushButton.setText("Reset")
            self.ui.stopwatch_start_stop_pushButton.setText("Start")
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")

        elif self.stopwatch_state == StopwatchStates.RUNNING:
            self.ui.stopwatch_lap_reset_pushButton.setEnabled(True)
            self.ui.stopwatch_lap_reset_pushButton.setText("Lap")
            self.ui.stopwatch_start_stop_pushButton.setText("Stop")
            self.ui.stopwatch_lap_reset_pushButton.setStyleSheet(f"background-color: {self._color_blue.name()}")
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f"background-color: {self._color_red.name()}")

        elif self.stopwatch_state == StopwatchStates.STOPPED:
            self.ui.stopwatch_lap_reset_pushButton.setEnabled(False)
            self.ui.stopwatch_lap_reset_pushButton.setText("Lap")
            self.ui.stopwatch_start_stop_pushButton.setText("Start")
            self.ui.stopwatch_lap_reset_pushButton.setStyleSheet(f"background-color: {self._color_default.name()}")
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")

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
            power_spin_box_array = self.spin_box_array
        elif self._gui_control_mode == GUIControlMode.CADENCE:
            cadence_spin_box_array = self.spin_box_array
        elif self._gui_control_mode == GUIControlMode.TORQUE:
            torque_spin_box_array = self.spin_box_array

        self.plot_power.update_plot(
            self.time_array,
            self.power_array,
            power_spin_box_array,
        )
        self.plot_cadence.update_plot(
            self.time_array,
            self.cadence_array,
            cadence_spin_box_array,
        )
        self.plot_torque.update_plot(
            self.time_array,
            self.torque_array,
            torque_spin_box_array,
        )

    def loop(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """
        date_time = QtCore.QDateTime()

        if self.run.value:
            if not self.stopping.value:
                self.ui.start_update_pushButton.setEnabled(True)
            # Date
            current_time = date_time.currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
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
            milliseconds = int((self.stopwatch - int(self.stopwatch)) * 100)  # get the milliseconds component
            self.ui.stopwatch_lcdNumber.display(f"{minutes:02d}:{seconds:02d}.{milliseconds:02d}")
            minutes = int(self.lap // 60)  # get the integer part of the quotient
            seconds = int(self.lap % 60)  # get the integer part of the remainder
            self.ui.lap_lcdNumber.display(f"{minutes:02d}:{seconds:02d}")

            # Data
            i_measured = self.i_measured.value
            turns = self.turns.value
            vel_estimate = self.vel_estimate.value

            user_torque = self.motor_computations.compute_user_torque(i_measured, vel_estimate)
            cadence = self.motor_computations.compute_cadence(vel_estimate)
            user_power = self.motor_computations.compute_user_power(user_torque, cadence)
            angle = self.motor_computations.compute_angle(turns)

            # Display data
            self.ui.current_power_display.setText(f"{user_power:.0f} W")
            self.ui.average_power_display.setText(f"{np.mean(self.power_array):.0f} W")
            self.ui.current_cadence_display.setText(f"{cadence:.0f} rpm")
            self.ui.average_cadence_display.setText(f"{np.mean(self.cadence_array):.0f} rpm")
            self.ui.current_torque_display.setText(f"{user_torque:.0f} N.m")
            self.ui.average_torque_display.setText(f"{np.mean(self.torque_array):.0f} N.m")
            self.ui.turns_display.setText(f"{turns:.0f} tr")
            self.ui.angle_display.setText(f"{angle:.0f} °")

            # TODO: print errors + save errors

            # Plot data
            self.time_array = np.roll(self.time_array, -1)
            self.time_array[-1] = QtCore.QDateTime.currentMSecsSinceEpoch() / 1000 - self.plot_start_time
            self.cadence_array = np.roll(self.cadence_array, -1)
            self.cadence_array[-1] = cadence
            self.torque_array = np.roll(self.torque_array, -1)
            self.torque_array[-1] = user_torque
            self.power_array = np.roll(self.power_array, -1)
            self.power_array[-1] = user_power
            if self.motor_started:
                if self.spin_box_array is None:
                    self.spin_box_array = np.zeros(self.size_arrays)
                self.spin_box_array = np.roll(self.spin_box_array, -1)
                self.spin_box_array[-1] = self.spin_box.value
            else:
                self.spin_box_array = None
            self._plot_update()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    # Shared memory
    # Instructions
    instruction = mp.Manager().Value(c_double, 0.0)
    ramp_instruction = mp.Manager().Value(c_double, 0.0)
    spin_box = mp.Manager().Value(c_double, 0.0)
    run = mp.Manager().Value(c_bool, True)
    stopping = mp.Manager().Value(c_bool, False)
    queue_instructions = mp.Manager().Queue()
    zero_position = mp.Manager().Value(c_bool, 0.0)
    i_measured = mp.Manager().Value(c_double, 0.0)
    turns = mp.Manager().Value(c_double, 0.0)
    vel_estimate = mp.Manager().Value(c_double, 0.0)
    saving = mp.Manager().Value(c_bool, False)
    queue_comment = mp.Queue()

    gui = ErgocycleGUI(
        run,
        instruction,
        ramp_instruction,
        spin_box,
        stopping,
        saving,
        queue_comment,
        i_measured,
        turns,
        vel_estimate,
        queue_instructions,
        zero_position,
    )
    gui.show()
    app.exec()

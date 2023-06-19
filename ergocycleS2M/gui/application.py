import numpy as np
import os
import sys
import time
from PyQt5 import QtWidgets, QtCore, QtGui

from ergocycleS2M.gui.ergocycle_gui import Ui_MainWindow
from ergocycleS2M.motor_control.enums import (
    ControlMode,
    DirectionMode,
)

# from ergocycleS2M.motor_control.motor import MotorController

from ergocycleS2M.motor_control.mock_controller import MockController
from ergocycleS2M.gui.gui_enums import (
    GUIControlMode,
    StopwatchStates,
    TrainingMode,
)
from ergocycleS2M.gui.gui_utils import (
    PlotWidget,
)

from ergocycleS2M.gui.motor_update_and_display_thread import MotorUpdateAndDisplayThread


class ErgocycleApplication(QtWidgets.QMainWindow):
    def __init__(self, odrive_motor):
        super(ErgocycleApplication, self).__init__(parent=None)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.motor = odrive_motor
        self.motor.zero_position_calibration()

        # Plot
        self._gui_control_mode = GUIControlMode.POWER
        self.plot_power = PlotWidget(self, name="Power", y_label="Power (W)", color="g", show_x_axis=False)
        self.ui.power_horizontalLayout.insertWidget(0, self.plot_power)
        self.plot_power.getAxis("bottom").setStyle(tickLength=0)
        self.plot_cadence = PlotWidget(self, name="Cadence", y_label="Cadence (rpm)", color="r", show_x_axis=False)
        self.ui.cadence_horizontalLayout.insertWidget(0, self.plot_cadence)
        self.plot_cadence.getAxis("bottom").setStyle(tickLength=0)
        self.plot_torque = PlotWidget(self, name="Torque", y_label="Torque (N.m)", color="b")
        self.ui.torque_horizontalLayout.insertWidget(0, self.plot_torque)

        # Thread for the watchdog, data saving and control
        # There is only one thread to control the order of execution of the different tasks since there is only one
        # process. Otherwise, the watchdog risks to not be fed in time.
        self.motor_thread = MotorUpdateAndDisplayThread(self.ui, odrive_motor)
        self.motor_thread.plot_update_signal.connect(self._plot_update)
        self.motor_thread.start()

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

        # Comments
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

    def emergency_stop(self):
        """
        Emergency stop. Stop the motor and close the GUI.
        """
        self.motor_thread.run = False
        self.close()

    def closeEvent(self, event):
        """
        Stop the motor on close.
        """
        self.motor_thread.run = False
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
            self.ui.instruction_spinBox.setRange(0, self.motor.hardware_and_security["power_lim"])
            self.ui.instruction_spinBox.setSingleStep(power_step)
            self.ui.instruction_label.setText("Power instruction")
            self.ui.units_label.setText("W")

            training_mode = self.ui.training_comboBox.currentText()
            if training_mode == TrainingMode.CONCENTRIC.value:
                self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_ramp_rate_lim"]))
                self.ui.acceleration_spinBox.setSingleStep(torque_step)
                self.ui.acceleration_spinBox.setValue(torque_ramp)
                self.ui.ramp_label.setText("Torque ramp")
                self.ui.acceleration_units_label.setText("N.m/s")
            elif training_mode == TrainingMode.ECCENTRIC.value:
                self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["pedals_accel_lim"]) - 1)
                self.ui.acceleration_spinBox.setSingleStep(vel_step)
                self.ui.acceleration_spinBox.setValue(vel_ramp)
                self.ui.ramp_label.setText("Cadence ramp")
                self.ui.acceleration_units_label.setText("rpm/s")
            else:
                raise NotImplementedError(f"{training_mode} training has not been implemented yet.")

        elif control_mode == GUIControlMode.LINEAR.value:
            self.ui.instruction_spinBox.setRange(0, 1.0)
            self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_ramp_rate_lim"]))
            self.ui.instruction_spinBox.setSingleStep(linear_step)
            self.ui.acceleration_spinBox.setSingleStep(torque_step)
            self.ui.acceleration_spinBox.setValue(torque_ramp)
            self.ui.instruction_label.setText("Linear coefficient")
            self.ui.units_label.setText("N.m/rpm")
            self.ui.ramp_label.setText("Torque ramp")
            self.ui.acceleration_units_label.setText("N.m/s")

        elif control_mode == GUIControlMode.CADENCE.value:
            self.ui.instruction_spinBox.setRange(0, self.motor.hardware_and_security["pedals_vel_limit"] - 1)
            self.ui.acceleration_spinBox.setRange(0, self.motor.hardware_and_security["pedals_accel_lim"] - 1)
            self.ui.instruction_spinBox.setSingleStep(vel_step)
            self.ui.acceleration_spinBox.setSingleStep(vel_step)
            self.ui.acceleration_spinBox.setValue(vel_ramp)
            self.ui.instruction_label.setText("Cadence instruction")
            self.ui.units_label.setText("rpm")
            self.ui.ramp_label.setText("Cadence ramp")
            self.ui.acceleration_units_label.setText("rpm/s")

        elif control_mode == GUIControlMode.TORQUE.value:
            self.ui.instruction_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_lim"] - 1))
            self.ui.acceleration_spinBox.setRange(0, int(self.motor.hardware_and_security["torque_ramp_rate_lim"]))
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
            self.motor_thread.training_mode = self.ui.training_comboBox.currentText()
            self.motor_thread.plot_start_time = time.time()
            self.motor_thread.time_array = np.linspace(
                -self.motor_thread.size_arrays / self.motor_thread.plot_frequency, 0, self.motor_thread.size_arrays
            )

        # Update instructions and the control depending on the control mode.
        self.motor_thread.ramp_instruction = self.ui.acceleration_spinBox.value()

        if self._gui_control_mode == GUIControlMode.POWER:
            if self._training_mode == TrainingMode.CONCENTRIC.value:
                self.motor_thread.spin_box = self.ui.instruction_spinBox.value()
                self.motor_thread.instruction = self.motor.concentric_power_control(
                    self.motor_thread.spin_box, self.motor_thread.ramp_instruction
                )
            elif self._training_mode == TrainingMode.ECCENTRIC.value:
                # In eccentric mode, the sign is inverted because the user's power is negative
                self.motor_thread.spin_box = -self.ui.instruction_spinBox.value()
                self.motor_thread.instruction = self.motor.eccentric_power_control(
                    self.motor_thread.spin_box, self.motor_thread.ramp_instruction
                )
            else:
                raise NotImplementedError(f"{self._training_mode} training has not been implemented yet.")

        elif self._gui_control_mode == GUIControlMode.LINEAR:
            self.motor_thread.spin_box = self.ui.instruction_spinBox.value()
            self.motor_thread.instruction = self.motor.linear_control(
                self.motor_thread.spin_box, self.motor_thread.ramp_instruction
            )

        elif self._gui_control_mode == GUIControlMode.CADENCE:
            self.motor_thread.instruction = self.motor.get_sign() * self.ui.instruction_spinBox.value()
            self.motor_thread.spin_box = self.motor_thread.instruction
            self.motor.cadence_control(self.motor_thread.spin_box, self.motor_thread.ramp_instruction)

        elif self._gui_control_mode == GUIControlMode.TORQUE:
            self.motor_thread.spin_box = self.motor.get_sign() * self.ui.instruction_spinBox.value()
            self.motor_thread.instruction = self.motor.torque_control(
                self.motor_thread.spin_box, self.motor_thread.ramp_instruction
            )

        else:
            raise NotImplementedError(f"{self._gui_control_mode} control has not been implemented yet.")

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
        self.motor_thread.stopping = True
        self.motor_thread.stopping_ramp_instruction = ramp_instruction

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
        self.motor_thread.file_name = self.ui.save_lineEdit.text()
        ext = ".bio"
        if os.path.isfile(f"{self.motor_thread.file_name}{ext}"):
            # File already exists, add a suffix to the filename
            i = 1
            while os.path.isfile(f"{self.motor_thread.file_name}_{i}{ext}"):
                i += 1
            self.motor_thread.file_name = f"{self.motor_thread.file_name}_{i}"

        self.motor_thread.saving = not self.motor_thread.saving
        self.ui.save_lineEdit.setEnabled(not self.motor_thread.saving)
        self.ui.comments_save_pushButton.setEnabled(self.motor_thread.saving)
        self.ui.comments_lineEdit.setEnabled(self.motor_thread.saving)
        if self.motor_thread.saving:
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
        self.motor_thread.comment_to_save = True
        self.motor_thread.comment = self.ui.comments_lineEdit.text()
        self.ui.comments_lineEdit.setText("")

    def _stopwatch_start_stop(self):
        """
        Start, stop or pause the stopwatch.
        """
        if self.motor_thread.stopwatch_state == StopwatchStates.PAUSED:
            self.motor_thread.stopwatch_state = StopwatchStates.RUNNING
            self.motor_thread.stopwatch_start_time += time.time() - self.motor_thread.stopwatch_pause_time
            self.motor_thread.stopwatch_lap_time += time.time() - self.motor_thread.stopwatch_pause_time

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
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f"background-color: {self._color_green.name()}")

        elif self.motor_thread.stopwatch_state == StopwatchStates.RUNNING:
            self.ui.stopwatch_lap_reset_pushButton.setEnabled(True)
            self.ui.stopwatch_lap_reset_pushButton.setText("Lap")
            self.ui.stopwatch_start_stop_pushButton.setText("Stop")
            self.ui.stopwatch_lap_reset_pushButton.setStyleSheet(f"background-color: {self._color_blue.name()}")
            self.ui.stopwatch_start_stop_pushButton.setStyleSheet(f"background-color: {self._color_red.name()}")

        elif self.motor_thread.stopwatch_state == StopwatchStates.STOPPED:
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

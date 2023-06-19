import numpy as np
import time

from PyQt5 import QtCore

from ergocycleS2M.gui.ergocycle_gui import Ui_MainWindow
from ergocycleS2M.motor_control.enums import ControlMode
from ergocycleS2M.gui.gui_enums import StopwatchStates


class MotorUpdateAndDisplayThread(QtCore.QThread):
    """
    Thread that controls the motor. It is separated from the GUI thread to avoid freezing the GUI when the motor is
    running.
    """

    plot_update_signal = QtCore.pyqtSignal(name="plot_update_signal")

    def __init__(self, ui: Ui_MainWindow, odrive_motor):
        super(MotorUpdateAndDisplayThread, self).__init__(parent=None)
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
        self.training_mode = self.ui.training_comboBox.currentText()

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
        self.time_array = np.linspace(-self.size_arrays / self.plot_frequency, 0, self.size_arrays)
        self.cadence_array = np.zeros(self.size_arrays)
        self.torque_array = np.zeros(self.size_arrays)
        self.power_array = np.zeros(self.size_arrays)
        self.spin_box_array = None

        # Watchdog
        self.watchdog_prev = time.time()
        self.dt = []

        # Saving
        self.saving = False
        self.file_name = ""

        # Stopping
        self.stopping = False
        self.stopping_ramp_instruction = 30.0

    def watchdog_feed(self):
        """
        Feed the watchdog of the motor.
        """
        t = time.time()
        self.dt.append(t - self.watchdog_prev)
        self.watchdog_prev = t
        self.motor.watchdog_feed()

    def run(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """
        t_plot_precedent = t_display_precedent = time.time()
        self.plot_start_time = time.time()
        date_time = QtCore.QDateTime()

        while self.run:
            self.watchdog_feed()

            # TODO time.sleep to have a constant frequency

            if self.saving:
                # Save data
                if self.comment_to_save:
                    comment = self.comment
                    self.comment_to_save = False
                else:
                    comment = ""
                self.motor.minimal_save_data_to_file(
                    self.file_name,
                    spin_box=self.spin_box,
                    instruction=self.instruction,
                    ramp_instruction=self.ramp_instruction,
                    comment=comment,
                    stopwatch=self.stopwatch,
                    lap=self.lap,
                    training_mode=self.training_mode,
                )
                self.watchdog_feed()

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

            self.watchdog_feed()

            # Display data
            t_since_precedent_display = time.time() - t_display_precedent
            if t_since_precedent_display > 1 / self._display_frequency:
                self.ui.current_power_display.setText(f"{self.motor.get_user_power():.0f} W")
                self.ui.average_power_display.setText(f"{np.mean(self.power_array):.0f} W")
                self.ui.current_cadence_display.setText(f"{self.motor.get_cadence():.0f} rpm")
                self.ui.average_cadence_display.setText(f"{np.mean(self.cadence_array):.0f} rpm")
                self.ui.current_torque_display.setText(f"{self.motor.get_user_torque():.0f} N.m")
                self.ui.average_torque_display.setText(f"{np.mean(self.torque_array):.0f} N.m")
                self.ui.turns_display.setText(f"{self.motor.get_turns():.0f} tr")
                self.ui.angle_display.setText(f"{self.motor.get_angle():.0f} °")
                t_display_precedent = time.time()

                self.ui.errors_label.setText(self.motor.get_errors())

                self.watchdog_feed()

            if not self.stopping:
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
                    self.instruction = self.motor.eccentric_power_control(self.spin_box, self.ramp_instruction)

            else:
                self.motor.stopping(cadence_ramp_rate=self.stopping_ramp_instruction)
                if abs(self.motor.get_cadence()) < 10.0:
                    self.stopping = not self.motor.stopped()
                    self.ui.start_update_pushButton.setEnabled(True)

            self.watchdog_feed()

            # Plot data
            t_since_precedent_plot = time.time() - t_plot_precedent
            if t_since_precedent_plot > 1 / self.plot_frequency:
                self.time_array = np.roll(self.time_array, -1)
                self.time_array[-1] = time.time() - self.plot_start_time
                self.cadence_array = np.roll(self.cadence_array, -1)
                self.cadence_array[-1] = cadence = self.motor.get_cadence()
                self.torque_array = np.roll(self.torque_array, -1)
                self.torque_array[-1] = torque = self.motor.get_user_torque()
                self.power_array = np.roll(self.power_array, -1)
                self.power_array[-1] = self.motor.compute_user_power(torque, cadence)
                if self.motor_started:
                    if self.spin_box_array is None:
                        self.spin_box_array = np.zeros(self.size_arrays)
                    self.spin_box_array = np.roll(self.spin_box_array, -1)
                    self.spin_box_array[-1] = self.spin_box
                else:
                    self.spin_box_array = None
                self.plot_update_signal.emit()
                t_plot_precedent = time.time()

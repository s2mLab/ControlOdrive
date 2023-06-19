import multiprocessing as mp
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from ergocycleS2M.gui.ergocycle_gui import Ui_MainWindow
from ergocycleS2M.gui.gui_enums import StopwatchStates
from ergocycleS2M.motor_control.mock_controller import MockController

queue = mp.Manager().Queue()

class MotorUpdateAndDisplayThread(QtCore.QThread):
    """
    Thread that controls the motor. It is separated from the GUI thread to avoid freezing the GUI when the motor is
    running.
    """

    plot_update_signal = QtCore.pyqtSignal(name="plot_update_signal")

    def __init__(self, ui: Ui_MainWindow):
        super(MotorUpdateAndDisplayThread, self).__init__(parent=None)
        self.ui = ui

        # Security
        self.run = True

        # Stopwatch
        self.stopwatch_start_time = 0.0
        self.stopwatch_pause_time = 0.0
        self.stopwatch_lap_time = 0.0
        self.stopwatch_state = StopwatchStates.STOPPED
        self.stopwatch = 0.0
        self.lap = 0.0

    def run(self):
        """
        Main loop of the thread. It is called when the thread is started. It is stopped when the thread is stopped.
        It updates the command and the display, saves the data and feeds the watchdog.
        """
        while self.run:
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

            try:
                self.ui.errors_label.setText(f"torque {queue.get_nowait()}")
            except:
                pass


class Application(QtWidgets.QMainWindow):
    def __init__(self):
        super(Application, self).__init__(parent=None)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.motor_thread = MotorUpdateAndDisplayThread(self.ui)
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

        # Stopwatch
        self._stopwatch_display()
        self.ui.stopwatch_start_stop_pushButton.clicked.connect(self._stopwatch_start_stop)
        self.ui.stopwatch_lap_reset_pushButton.clicked.connect(self._stopwatch_lap_reset)
        self.ui.stopwatch_lcdNumber.setDigitCount(8)
        self.ui.lap_lcdNumber.setDigitCount(5)

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


def run_appli():
    app = QtWidgets.QApplication(sys.argv)
    gui = Application()
    gui.show()
    app.exec()


def motor_process_function():
    motor = MockController(enable_watchdog=True, external_watchdog=True)
    while True:
        time.sleep(1.0)
        queue.put_nowait(motor.get_motor_torque())


motor_process = mp.Process(name="gui", target=motor_process_function, daemon=True)
motor_process.start()
# motor_process.join()

gui_process = mp.Process(name="gui", target=run_appli, daemon=True)
gui_process.start()
gui_process.join()
print("Hello")
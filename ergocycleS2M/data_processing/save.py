"""
All the functions needed to save data.
"""
import multiprocessing as mp
import pickle
import time

from pathlib import Path
from PyQt5 import QtCore, QtWidgets


def save(data_dict, data_path):
    """
    This function adds data to a pickle file. It appends the data to the end, so it's fast.
    Copied and adapted from
    https://github.com/pyomeca/biosiglive/blob/a9ac18d288e6cadd1bd3d38f9fc4a1584a789065/biosiglive/file_io/save_and_load.py

    Parameters
    ----------
    data_dict : dict
        The data to be added to the file.
    data_path : str
        The path to the file. The file must exist.
    """
    if Path(data_path).suffix != ".bio":
        if Path(data_path).suffix == "":
            data_path += ".bio"
        else:
            raise ValueError("The file must be a .bio file.")
    with open(data_path, "ab") as out_file:
        pickle.dump(data_dict, out_file, pickle.HIGHEST_PROTOCOL)


def save_data_to_file(
    file_path: str,
    time: float = None,
    spin_box: float = None,
    instruction: float = None,
    ramp_instruction: float = None,
    comment: str = None,
    stopwatch: float = None,
    lap: float = None,
    control_mode: str = None,
    direction: str = None,
    state: str = None,
    training_mode: str = None,
    vel_estimate: float = None,
    turns: float = None,
    iq_measured: float = None,
    error: int = None,
    axis_error: int = None,
    controller_error: int = None,
    encoder_error: int = None,
    motor_error: int = None,
    sensorless_estimator_error: int = None,
    can_error: int = None,
):
    """
    Saves data. Only the data needed to reconstruct all the data is saved.

    Parameters
    ----------
    file_path: str
        The path of the file where the data will be saved.
    time: float
        The time at the instant of the saving.
    spin_box: float
        The value of the spin box at the instant of the saving.
    instruction: float
        The value of the instruction at the instant of the saving.
    ramp_instruction: float
        The value of the ramp instruction at the instant of the saving.
    comment: str
        A comment to add to the data at the instant of the saving.
    stopwatch: float
        The value of the stopwatch at the instant of the saving.
    lap: float
        The value of the stopwatch lap at the instant of the saving.
    control_mode: str
        The control mode at the instant of the saving (see ControlMode in the enums.py file).
    direction: str
        The direction at the instant of the saving (forward or reverse).
    state: str
        The state at the instant of the saving (see odrv0.axis0.current_state in the ODrive documentation).
    training_mode: str
        The training mode at the instant of the saving (concentric or eccentric, in the case we are in cadence
        control).
    vel_estimate: float
        The velocity estimate at the instant of the saving (tr/s of the motor).
    turns: float
        The number of turns at the instant of the saving.
    iq_measured: float
        The current measured at the instant of the saving.
    error: int
        The error at the instant of the saving (see odrv0.error in the ODrive documentation).
    axis_error: int
        The axis error at the instant of the saving (see odrv0.axis0.error in the ODrive documentation).
    controller_error: int
        The controller error at the instant of the saving (see odrv0.axis0.controller.error in the ODrive
        documentation).
    encoder_error: int
        The encoder error at the instant of the saving (see odrv0.axis0.encoder.error in the ODrive documentation).
    motor_error: int
        The motor error at the instant of the saving (see odrv0.axis0.motor.error in the ODrive documentation).
    sensorless_estimator_error: int
        The sensorless estimator error at the instant of the saving (see odrv0.axis0.sensorless_estimator.error in the
        ODrive documentation).
    can_error: int
        The CAN error at the instant of the saving (see odrv0.can.error in the ODrive documentation).
    """
    data = {
        "time": time,
        "spin_box": spin_box,
        "instruction": instruction,
        "ramp_instruction": ramp_instruction,
        "comments": comment,
        "stopwatch": stopwatch,
        "lap": lap,
        "state": state,
        "control_mode": control_mode,
        "direction": direction,
        "training_mode": training_mode,
        "vel_estimate": vel_estimate,
        "turns": turns,
        "iq_measured": iq_measured,
        "error": error,
        "axis_error": axis_error,
        "controller_error": controller_error,
        "encoder_error": encoder_error,
        "motor_error": motor_error,
        "sensorless_estimator_error": sensorless_estimator_error,
        "can_error": can_error,
    }

    save(data, file_path)


class DataSaver(QtWidgets.QApplication):
    def __init__(
            self,
            file_path: str,
            run: mp.Value,
            saving: mp.Value,
            spin_box: mp.Value,
            instruction: mp.Value,
            ramp_instruction: mp.Value,
            queue_comment: mp.Queue,
            stopwatch: mp.Value,
            lap: mp.Value,
            control_mode: mp.Value,
            direction: mp.Value,
            state: mp.Value,
            training_mode: mp.Value,
            vel_estimate: mp.Value,
            turns: mp.Value,
            iq_measured: mp.Value,
            error: mp.Value,
            axis_error: mp.Value,
            controller_error: mp.Value,
            encoder_error: mp.Value,
            motor_error: mp.Value,
            sensorless_estimator_error: mp.Value,
            can_error: mp.Value,
            motor_time: mp.Value,
            save_period: float = 0.1,
    ):
        super().__init__(parent=None)
        self.save_period = save_period

        self.run = run
        self.saving = saving

        # Data
        self.file_path = file_path
        self.spin_box = spin_box
        self.instruction = instruction
        self.ramp_instruction = ramp_instruction
        self.comment = ""
        self.queue_comment = queue_comment
        self.stopwatch = stopwatch
        self.lap = lap
        self.state = state
        self.training_mode = training_mode
        self.control_mode = control_mode
        self.direction = direction
        self.vel_estimate = vel_estimate
        self.turns = turns
        self.iq_measured = iq_measured
        self.error = error
        self.axis_error = axis_error
        self.controller_error = controller_error
        self.encoder_error = encoder_error
        self.motor_error = motor_error
        self.sensorless_estimator_error = sensorless_estimator_error
        self.can_error = can_error

        self.motor_time = motor_time

        # Update the display according to `update_period`
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.load_data_and_save)
        self.start_time = time.time()
        self.timer.start(int(self.save_period * 1000))  # milliseconds

    def load_data_and_save(self):
        """
        Loads data from the shared memory and saves it to a file.
        """
        if self.run.value and self.saving.value:
            self.set_comment()
            self.save_data_to_file()
        else:
            self.timer.stop()
            self.close()

    def set_comment(self):
        """
        Sets the comment to the last comment in the queue.
        """
        try:
            self.comment = self.queue_comment.get_nowait()
        except Exception:
            self.comment = ""

    def save_data_to_file(self):
        """
        Saves data. Only the data needed to reconstruct all the data is saved.
        """
        data = {
            "time": time.time() - self.start_time,
            "motor_time": self.motor_time.value,
            "spin_box": self.spin_box.value,
            "instruction": self.instruction.value,
            "ramp_instruction": self.ramp_instruction.value,
            "comments": self.comment,
            "stopwatch": self.stopwatch.value,
            "lap": self.lap.value,
            "state": self.state.value,
            "control_mode": self.control_mode.value,
            "direction": self.direction.value,
            "training_mode": self.training_mode.value,
            "vel_estimate": self.vel_estimate.value,
            "turns": self.turns.value,
            "iq_measured": self.iq_measured.value,
            "error": self.error.value,
            "axis_error": self.axis_error.value,
            "controller_error": self.controller_error.value,
            "encoder_error": self.encoder_error.value,
            "motor_error": self.motor_error.value,
            "sensorless_estimator_error": self.sensorless_estimator_error.value,
            "can_error": self.can_error.value,
        }

        save(data, self.file_path)

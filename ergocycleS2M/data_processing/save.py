"""
All the functions needed to save data.
"""
import pickle

from pathlib import Path


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
        training_mode: str
            The training mode at the instant of the saving (concentric or eccentric, in the case we are in cadence
            control).
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

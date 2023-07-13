"""
All the functions needed to load data, process it and plot it.
"""
import argparse
import copy
import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
from pathlib import Path
import sys
from typing import Tuple

from ergocycleS2M.motor_control.enums import (
    ControlMode,
)
from ergocycleS2M.motor_control.motor_computations import MotorComputations
from ergocycleS2M.utils import (
    ODriveError,
    ODriveAxisError,
    ODriveEncoderError,
    ODriveControllerError,
    ODriveSensorlessEstimatorError,
    ODriveMotorError,
    ODriveCanError,
    traduce_error,
)


def load(filename: str) -> dict:
    """
    This function reads data from a pickle file to concatenate them into one dictionary. Copied and adapted from
    https://github.com/pyomeca/biosiglive/blob/a9ac18d288e6cadd1bd3d38f9fc4a1584a789065/biosiglive/file_io/save_and_load.py

    Parameters
    ----------
    filename : str
        The path to the file.

    Returns
    -------
    data : dict
        The data read from the file.
    """
    if Path(filename).suffix != ".bio":
        raise ValueError("The file must be a .bio file.")
    data = {}
    with open(filename, "rb") as file:
        while True:
            try:
                data_tmp = pickle.load(file)
                for key in data_tmp.keys():
                    # If the key is already in the dictionary, we concatenate the data
                    if key in data.keys() and data[key] is not None:
                        if isinstance(data[key], list) is True:
                            data[key].append(data_tmp[key])
                        else:
                            data[key] = np.append(data[key], data_tmp[key], axis=len(data[key].shape) - 1)
                    # If the key is not in the dictionary (first appearance), we add it
                    else:
                        # If the first appearance is not the first entry of the file, we must create the entry and fill
                        # it with Nones
                        if "time" in data.keys() and len(data["time"]) > 1:
                            data[key] = [None] * (len(data["time"]) - 1) + [data_tmp[key]]
                        # Else, we can create the entry and fill it with the data
                        else:
                            data[key] = [data_tmp[key]]
            except EOFError:
                break
    return data


def compute_data(
    vel_estimate: np.ndarray,
    turns: np.ndarray,
    iq_measured: np.ndarray,
    gear: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Computes the data of interest from raw data saved.

    Parameters
    ----------
    vel_estimate: np.ndarray
        The cadence of the motor in tr/s.
    turns: np.ndarray
        The turns done by the motor since the last reset in tr.
    iq_measured: np.ndarray
        The current measured in the motor in A.
    gear: np.ndarray
        The current gear (0 if the chain is not on the motor, 1 for the easiest gear 10 for the hardest gear).

    Returns
    -------
    The data of interest (user_torque, cadence, angle, user_power, resisting_torque, motor_torque).
    """
    motor_object = MotorComputations()

    nb_points = len(iq_measured)

    user_torque = np.zeros(nb_points)
    cadence = np.zeros(nb_points)
    angle = np.zeros(nb_points)
    user_power = np.zeros(nb_points)
    resisting_torque = np.zeros(nb_points)
    motor_torque = np.zeros(nb_points)
    for i in range(nb_points):
        cadence[i] = motor_object.compute_cadence(vel_estimate[i])
        angle[i] = motor_object.compute_angle(turns[i])
        user_torque[i] = motor_object.compute_user_torque(iq_measured[i], vel_estimate[i], gear[i])
        user_power[i] = motor_object.compute_user_power(user_torque[i], cadence[i])
        resisting_torque[i] = motor_object.compute_resisting_torque(
            iq_measured[i], vel_estimate[i], gear[i]
        )
        motor_torque[i] = motor_object.compute_motor_torque(iq_measured[i])
    return user_torque, cadence, angle, user_power, resisting_torque, motor_torque


def interpolate_data(data: dict, frequency: float = 10) -> dict:
    """
    This function interpolates the data to have a constant frequency.

    Parameters
    ----------
    data : dict
        The data read from the file.
    frequency : float
        The desired frequency of the data.

    Returns
    -------
    data_interpolated : dict
        The data interpolated at the desired frequency.
    """
    data_interpolated = {}
    time_for_interpolated = data_interpolated["time"] = np.interp(
        np.linspace(0, data["time"][-1], num=int(data["time"][-1] * frequency)), data["time"], data["time"]
    )

    # For the following keys, we interpolate the data with np.interp which is a linear interpolation.
    linear_interpolation_keys = ["vel_estimate", "turns", "iq_measured", "stopwatch", "lap"]
    for key in data.keys():
        if key in linear_interpolation_keys:
            data_interpolated[key] = np.interp(
                np.linspace(0, data["time"][-1], num=int(data["time"][-1] * frequency)), data["time"], data[key]
            )

    # For the stopwatch and lap, we set the values to 0 when the stopwatch has stopped (if not the interpolation may
    # have created decreasing ramps).
    for i in range(len(data["time"]) - 1):
        if data["stopwatch"][i] != 0 and data["stopwatch"][i + 1] == 0:
            i_interpolated_0 = np.where(time_for_interpolated > data["time"][i])[0][0]
            i_interpolated_1 = np.where(time_for_interpolated > data["time"][i + 1])[0][0]
            for j in range(i_interpolated_0, i_interpolated_1):
                data_interpolated["stopwatch"][j] = 0.0
                data_interpolated["lap"][j] = 0.0

    # For the other keys, which are strings, instructions or errors, we take the nearest value.
    if "gear" not in data.keys():
        data["gear"] = np.zeros(len(data["time"]))
    nearest_keys = [
        "gear",
        "state",
        "control_mode",
        "direction",
        "error",
        "axis_error",
        "controller_error",
        "encoder_error",
        "motor_error",
        "sensorless_estimator_error",
        "can_error",
    ]
    if data["spin_box"] is not None:
        nearest_keys.append("spin_box")
    if data["instruction"] is not None:
        nearest_keys.append("instruction")
    if data["ramp_instruction"] is not None:
        nearest_keys.append("ramp_instruction")

    # Create the arrays.
    for key in nearest_keys:
        data_interpolated[key] = np.zeros(len(time_for_interpolated), dtype=type(np.asarray(data[key]).dtype))

    i_prev = 0
    i_next = 1
    # Select the nearest value at each instant.
    for i, t in enumerate(time_for_interpolated):
        # Select the two iterations of the original data around the time t.
        while data["time"][i_next] < t:
            i_prev += 1
            i_next += 1
        # Choose the nearest value to the time t and register it in the interpolated data.
        for key in nearest_keys:
            if data["time"][i_next] - t < t - data["time"][i_prev]:
                data_interpolated[key][i] = data[key][i_next]
            else:
                data_interpolated[key][i] = data[key][i_prev]

    # For the comments, the comment is registered at the nearest time to the comment time in the new time array.
    # Create the arrays.
    data_interpolated["comments"] = np.zeros(len(time_for_interpolated), dtype=np.asarray(data["comments"]).dtype)

    i_prev = 0
    i_next = 1
    for i, t in enumerate(data["time"]):
        if data["comments"][i] != "":
            # Select the time just before and just after the time comment.
            while time_for_interpolated[i_next] < t:
                i_prev += 1
                i_next += 1
            # Choose the nearest time and register the comment at that time in the interpolated data.
            if time_for_interpolated[i_next] - t < t - time_for_interpolated[i_prev]:
                data_interpolated["comments"][i_next] = data["comments"][i]
            else:
                data_interpolated["comments"][i_prev] = data["comments"][i]

    return data_interpolated


def smooth_data(data: dict, window_length: int) -> dict:
    """
    This function smooths the data with a moving average filter. To be used after the interpolation.

    Parameters
    ----------
    data : dict
        The data read from the file and already interpolated.
    window_length : int
        The length of the window for the moving average filter.

    Returns
    -------
    smoothed_data : dict
        The data smoothed with a moving average filter or the data itself if window_length is 0.
    """
    if window_length == 0:
        smoothed_data = copy.deepcopy(data)
        smoothed_data["time_for_smoothed"] = data["time"]
    else:
        kernel = np.ones(window_length) / window_length
        smoothed_data = copy.deepcopy(data)
        for key in ["turns", "iq_measured", "user_torque", "cadence", "user_power", "resisting_torque", "motor_torque"]:
            smoothed_data[f"smoothed_{key}"] = np.convolve(data[key], kernel, mode="valid")

        smoothed_data["time_for_smoothed"] = data["time"][window_length // 2 : -window_length // 2 + 1]

    return smoothed_data


def read(data_path: str, sample_frequency: float, window_length: int) -> dict:
    """
    This function reads the data from the file and interpolates them and then smooth it.

    Parameters
    ----------
    data_path: str
         The path to the data file, has to be a `.bio` file.
    sample_frequency: float
        The frequency at which the data will be interpolated from the original data.
    window_length:
        The length of the window for the moving average filter.

    Returns
    -------
    data: dict
        The data interpolated at the desired frequency and smoothed with a moving average filter of the desired window
        length.
    """
    data = load(data_path)
    data_interpolated = interpolate_data(data, frequency=sample_frequency)
    (
        data_interpolated["user_torque"],
        data_interpolated["cadence"],
        data_interpolated["angle"],
        data_interpolated["user_power"],
        data_interpolated["resisting_torque"],
        data_interpolated["motor_torque"],
    ) = compute_data(
        data_interpolated["vel_estimate"],
        data_interpolated["turns"],
        data_interpolated["iq_measured"],
        data_interpolated["gear"],
    )
    smoothed_data = smooth_data(data_interpolated, window_length)
    return smoothed_data


def plot_data(data: dict, plot_errors: bool = False):
    """
    Plot the data acquired during an experiment.

    Parameters
    ----------
    data: dict
        The data to plot, the data of interest must have been computed by `compute_data`.
    plot_errors: bool
        If the user wants to see the errors according to time or not.
    """

    # Comments
    for time, comment in zip(data["time"], data["comments"]):
        if comment != "":
            print(f"{time:.0f}: {comment}")

    # Cadence
    instruction_for_cadence = np.zeros(len(data["instruction"]))
    instruction_for_cadence[:] = np.nan
    instruction_for_cadence[data["control_mode"] == ControlMode.CADENCE_CONTROL.value] = data["instruction"][
        data["control_mode"] == ControlMode.CADENCE_CONTROL.value
    ]
    instruction_for_cadence[data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value] = data["instruction"][
        data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value
    ]

    plt.figure()
    plt.title("Cadence")
    plt.ylabel("Cadence (rpm)")
    plt.xlabel("Time (s)")

    plt.plot(data["time"], data["cadence"], label="Cadence")
    plt.plot(data["time_for_smoothed"], data["smoothed_cadence"], label="Smoothed cadence")
    plt.plot(data["time"], instruction_for_cadence, label="Instruction")

    plt.legend()

    # Torques
    instruction_for_torque = np.zeros(len(data["instruction"]))
    instruction_for_torque[:] = np.nan
    instruction_for_torque[data["control_mode"] == ControlMode.TORQUE_CONTROL.value] = data["instruction"][
        data["control_mode"] == ControlMode.TORQUE_CONTROL.value
    ]
    instruction_for_torque[data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value] = data["instruction"][
        data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value
    ]
    instruction_for_torque[data["control_mode"] == ControlMode.LINEAR_CONTROL.value] = data["instruction"][
        data["control_mode"] == ControlMode.LINEAR_CONTROL.value
    ]

    if "spin_box" in data.keys():
        spinbox_for_torque = np.zeros(len(data["spin_box"]))
        spinbox_for_torque[:] = np.nan
        spinbox_for_torque[data["control_mode"] == ControlMode.TORQUE_CONTROL.value] = data["spin_box"][
            data["control_mode"] == ControlMode.TORQUE_CONTROL.value
        ]

    fig, ax1 = plt.subplots()

    ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Torque (Nm)")
    ax2.set_ylabel("Cadence (rpm)")

    plt.title("Torques")

    ax2.plot(data["time_for_smoothed"], data["smoothed_cadence"], label="Smoothed cadence", color="k")
    ax1.plot(data["time_for_smoothed"], data["smoothed_motor_torque"], label="Smoothed motor torque")
    ax1.plot(data["time_for_smoothed"], data["smoothed_user_torque"], label="Smoothed user torque")
    ax1.plot(data["time_for_smoothed"], data["smoothed_resisting_torque"], label="Smoothed resisting torque")
    if "spin_box" in data.keys():
        ax1.plot(data["time"], spinbox_for_torque, label="Spin box")
    ax1.plot(
        data["time"],
        instruction_for_torque,
        label="Instruction",
    )

    fig.legend(loc="upper right")

    # Powers
    if "spin_box" in data.keys():
        spinbox_for_power = np.zeros(len(data["spin_box"]))
        spinbox_for_power[:] = np.nan
        spinbox_for_power[data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value] = data["spin_box"][
            data["control_mode"] == ControlMode.CONCENTRIC_POWER_CONTROL.value
        ]
        spinbox_for_power[data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value] = data["spin_box"][
            data["control_mode"] == ControlMode.ECCENTRIC_POWER_CONTROL.value
        ]

    fig, ax1 = plt.subplots()
    plt.title("Powers")

    ax2 = ax1.twinx()  # Create a second y-axis that shares the same x-axis
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Powers(W)")
    ax2.set_ylabel("Cadence (rpm)")

    ax1.plot(data["time_for_smoothed"], data["smoothed_user_power"], label="User power")
    if "spin_box" in data.keys():
        ax1.plot(data["time"], spinbox_for_power, label="Spin_box")
    ax2.plot(data["time_for_smoothed"], data["smoothed_cadence"], label="Smoothed cadence", color="k")

    fig.legend(loc="upper right")

    if plot_errors:
        # Errors
        plt.figure()

        plt.title("Errors")

        plt.plot(data["error"], label="Error")
        plt.plot(data["axis_error"], label="Axis error")
        plt.plot(data["controller_error"], label="Controller error")
        plt.plot(data["encoder_error"], label="Encoder error")
        plt.plot(data["motor_error"], label="Motor error")
        plt.plot(data["sensorless_estimator_error"], label="Sensorless estimator error")
        plt.plot(data["can_error"], label="Can error")

    # noinspection PyTypeChecker
    print(
        f"{traduce_error(data['error'][-1], ODriveError)}"
        f"{traduce_error(data['axis_error'][-1], ODriveAxisError)}"
        f"{traduce_error(data['controller_error'][-1], ODriveControllerError)}"
        f"{traduce_error(data['encoder_error'][-1], ODriveEncoderError)}"
        f"{traduce_error(data['motor_error'][-1], ODriveMotorError)}"
        f"{traduce_error(data['sensorless_estimator_error'][-1], ODriveSensorlessEstimatorError)}"
        f"{traduce_error(data['can_error'][-1], ODriveCanError)}"
    )

    plt.show()


def read_from_terminal():
    """
    Plot the data of interest from a pickle file. The name of the file is given as an argument.
    """
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(description="Name of the file to be read")

    # Add arguments
    parser.add_argument("name", type=str, help="Name of the file to be read")

    # Parse the arguments
    args = parser.parse_args()

    # Access the arguments
    file_name = args.name

    # Read the data
    data = read(file_name, 100, 100)

    # Plot the data
    plot_data(data)


if __name__ == "__main__":
    script_path = sys.argv[0]
    script_directory = os.path.dirname(os.path.abspath(script_path))
    control_odrive_directory = os.path.dirname(os.path.dirname(os.path.dirname(script_directory)))

    plot_data(read(control_odrive_directory + "/XP/Sprint_Amandine_FES_End.bio", 100, 100), plot_errors=True)

import copy
import pickle
import numpy as np
from pathlib import Path
from motor_computations import MotorComputations

def save(data_dict, data_path):
    """This function adds data to a pickle file. It not open the file, but appends the data to the end, so it's fast.
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
    with open(data_path, "ab") as outf:
        pickle.dump(data_dict, outf, pickle.HIGHEST_PROTOCOL)


def load(filename, number_of_line=None):
    """This function reads data from a pickle file to concatenate them into one dictionary.
    Parameters
    ----------
    filename : str
        The path to the file.
    number_of_line : int
        The number of lines to read. If None, all lines are read.
    Returns
    -------
    data : dict
        The data read from the file.
    """
    if Path(filename).suffix != ".bio":
        raise ValueError("The file must be a .bio file.")
    data = {}
    limit = 2 if not number_of_line else number_of_line
    with open(filename, "rb") as file:
        count = 0
        while count < limit:
            try:
                data_tmp = pickle.load(file)
                for key in data_tmp.keys():
                    if key in data.keys() and data[key] is not None:
                        if isinstance(data[key], list) is True:
                            data[key].append(data_tmp[key])
                        else:
                            data[key] = np.append(data[key], data_tmp[key], axis=len(data[key].shape) - 1)
                    else:
                        if isinstance(data_tmp[key], (int, float, str, dict)) is True:
                            data[key] = [data_tmp[key]]
                        elif isinstance(data_tmp[key], list) is True:
                            data[key] = [data_tmp[key]]
                        else:
                            data[key] = data_tmp[key]
                if number_of_line:
                    count += 1
                else:
                    count = 1
            except EOFError:
                break
    return data


def compute_data(vel_estimate, turns, iq_measured):
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
        user_torque[i] = motor_object.compute_user_torque(iq_measured[i], vel_estimate[i])
        user_power[i] = motor_object.compute_user_power(user_torque[i], cadence[i])
        resisting_torque[i] = motor_object.compute_resisting_torque(iq_measured[i], vel_estimate[i])
        motor_torque[i] = motor_object.compute_motor_torque(iq_measured[i])
    return user_torque, cadence, angle, user_power, resisting_torque, motor_torque


def interpolate_data(data, frequency=10):
    """
    This function interpolates the data to have a constant frequency.

    Parameters
    ----------
    frequency : int
        The frequency of the data.
    data : dict
        The data read from the file.
    """
    data_interpolated = {}
    time = data_interpolated["time"] = np.interp(
        np.linspace(0, data["time"][-1], num=int(data["time"][-1] * frequency)),
        data["time"],
        data["time"]
    )

    # For the following keys, we interpolate the data with np.interp which is a linear interpolation.
    for key in data.keys():
        if key in ["vel_estimate", "turns", "iq_measured", "stopwatch", "lap"]:
            data_interpolated[key] = np.interp(np.linspace(0, data["time"][-1], num=int(data["time"][-1] * frequency)),
                                               data["time"], data[key])

    for i in range(len(data["time"])):
        if data["stopwatch"][i] != 0 and data["stopwatch"][i + 1] == 0:
            i_interpolated_0 = np.where(time > data["time"][i])[0][0]
            i_interpolated_1 = np.where(time > data["time"][i + 1])[0][0]
            for j in range(i_interpolated_0, i_interpolated_1):
                data_interpolated["stopwatch"][j] = 0.0
                data_interpolated["lap"][j] = 0.0

    # For the other keys, we take the nearest value
    keys_nearest = ["state", "control_mode", "direction", "comments", "spin_box", "instruction", "ramp_instruction",
                    "error", "axis_error", "controller_error", "encoder_error", "motor_error",
                    "sensorless_estimator_error", "can_error"]
    for key in keys_nearest:
        data_interpolated[key] = np.zeros(len(time), dtype=type(np.asarray(data[key]).dtype))
    i_prec = 0
    i_next = 1
    for i, t in enumerate(time):
        while data["time"][i_next] < t:
            i_prec += 1
            i_next += 1
        for key in keys_nearest:
            if data["time"][i_next] - t < t - data["time"][i_prec]:
                data_interpolated[key][i] = data[key][i_next]
            else:
                data_interpolated[key][i] = data[key][i_prec]
    return data_interpolated


def smooth_data(data, window_length):
    """
    This function smooths the data with a moving average filter.

    Parameters
    ----------
    data : dict
        The data read from the file and already interpolated.
    window_length : int
        The length of the window for the moving average filter.
    """
    kernel = np.ones(window_length) / window_length
    smoothed_data = copy.deepcopy(data)
    for key in ['iq_measured', 'user_torque', 'cadence', 'user_power', 'resisting_torque', 'motor_torque']:
        smoothed_data[f"smoothed_{key}"] = np.convolve(data[key], kernel, mode='valid')

    smoothed_data["time_for_smoothed"] = data["time"][window_length // 2:-window_length // 2 + 1]

    return smoothed_data


def read(data_path, sample_frequency, window_length,):
    """
    This function reads the data from the file and interpolates them.
    :param data_path:
    :param sample_frequency:
    :param window_length:
    :return:
    """
    data = load(data_path)
    data_interpolated = interpolate_data(data, frequency=sample_frequency)
    (data_interpolated["user_torque"], data_interpolated["cadence"], data_interpolated["angle"],
     data_interpolated["user_power"], data_interpolated["resisting_torque"], data_interpolated["motor_torque"]) \
        = compute_data(data_interpolated["vel_estimate"], data_interpolated["turns"], data_interpolated["iq_measured"])
    smoothed_data = smooth_data(data_interpolated, window_length)
    return smoothed_data

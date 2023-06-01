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

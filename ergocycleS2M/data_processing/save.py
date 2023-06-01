"""
All the functions needed to save data.
"""
import pickle

from pathlib import Path


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
    with open(data_path, "ab") as out_file:
        pickle.dump(data_dict, out_file, pickle.HIGHEST_PROTOCOL)

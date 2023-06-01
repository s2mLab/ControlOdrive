"""
This module contains the GUI utilities.
"""
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtGui


class PlotWidget(pg.PlotWidget):
    """
    Custom plot widget.
    """

    def __init__(self, parent=None, name=None, y_label=None, color="b"):
        super().__init__(parent=parent)

        # Set background color to white
        self.setBackground("w")

        # Set x and y axis labels
        self.setLabel("bottom", "Time since last control start (s)", color="k")
        self.setLabel("left", y_label, color="k")

        # Create a legend
        self.legend = self.addLegend(offset=(1, 1))
        self.legend.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
        self._instruction_curve = self.plot([], [], pen=pg.mkPen("k", width=2), name="Instruction")
        self._data_curve = self.plot([], [], pen=pg.mkPen(color, width=2), name=name)
        self.removeItem(self._instruction_curve)

        # Set color of legend text to black
        self.legend.labelTextColor = pg.mkColor("k")

        # Set axes color
        axis = self.getAxis("bottom")
        axis.setPen(pg.mkPen("k"))
        axis.setTextPen("k")

        axis = self.getAxis("left")
        axis.setPen(pg.mkPen("k"))
        axis.setTextPen("k")

        # Set the grid
        self.showGrid(x=False, y=True, alpha=0.5)

    def update_plot(
        self,
        time_array: np.ndarray,
        data_array: np.ndarray,
        spin_box_array: np.ndarray = None,
    ):
        """
        Update the plot with the new data.

        Parameters
        ----------
        time_array: np.ndarray
            Time values to plot.
        data_array: np.ndarray
            Data to plot.
        spin_box_array: np.ndarray
            Spin box values to plot.
        """
        self._data_curve.setData(time_array, data_array)
        min_array = np.min(data_array)
        max_array = np.max(data_array)

        if spin_box_array is not None:
            min_array = min(min_array, np.min(spin_box_array))
            max_array = max(max_array, np.max(spin_box_array))
            self._instruction_curve.setData(time_array, spin_box_array)

        self.setXRange(time_array[0], time_array[-1])
        if min_array > -10 and max_array < 10:
            self.setYRange(-10, 10)
        else:
            self.setYRange(min(min_array, 0), max(max_array, 0))

    def add_instruction(self):
        """
        Add the instruction to the plot.
        """
        self.addItem(self._instruction_curve)

    def remove_instruction(self):
        """
        Remove the instruction from the plot.
        """
        self.removeItem(self._instruction_curve)

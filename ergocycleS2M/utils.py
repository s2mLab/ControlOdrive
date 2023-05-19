import time
import numpy as np
from enum import Enum

from PyQt5 import QtGui, QtCore
import pyqtgraph as pg


class StrEnum(str, Enum):
    """
    Enum where members are also (and must be) strings. Existing class in Python 3.11 but the code is running in Python
    3.9 for now.
    """

    def __new__(cls, *values):
        """
        Values must already be of type `str`.
        """
        if len(values) > 3:
            raise TypeError('too many arguments for str(): %r' % (values,))
        if len(values) == 1:
            # it must be a string
            if not isinstance(values[0], str):
                raise TypeError('%r is not a string' % (values[0],))
        if len(values) >= 2:
            # check that encoding argument is a string
            if not isinstance(values[1], str):
                raise TypeError('encoding must be a string, not %r' % (values[1],))
        if len(values) == 3:
            # check that errors argument is a string
            if not isinstance(values[2], str):
                raise TypeError('errors must be a string, not %r' % (values[2]))
        value = str(*values)
        member = str.__new__(cls, value)
        member._value_ = value
        return member

    def _generate_next_value_(self, start, count, last_values):
        """
        Return the lower-cased version of the member name.
        """
        return self.lower()


def traduce_error(decimal_number, odrive_enum):
    """
    Traduce an error code into a string.
    """
    hex_number = "{0:x}".format(decimal_number)
    res = ""
    for i, j in enumerate(hex_number[::-1]):
        if j != '0':
            for member in odrive_enum.__members__.values():
                if member.value == "0x" + format(int(j) * 16 ** int(i), f"0{8}X"):
                    res += member.name
                    res += ", "
    return res


class PlotWidget(pg.PlotWidget):
    def __init__(self, parent=None, name=None, y_label=None, color="b"):
        super().__init__(parent=parent)

        # Set background color to white
        self.setBackground('w')

        # Set x and y axis labels
        self.setLabel('bottom', 'Time since last control start (s)', color='k')
        self.setLabel('left', y_label, color='k')

        # Create a legend
        self.legend = self.addLegend(offset=(1, 1))
        self.legend.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
        self._instruction_curve = self.plot([], [], pen=pg.mkPen('k', width=2), name='Instruction')
        self._data_curve = self.plot([], [], pen=pg.mkPen(color, width=2), name=name)
        self.removeItem(self._instruction_curve)

        # Set color of legend text to black
        self.legend.labelTextColor = pg.mkColor('k')

        # Set axes color
        axis = self.getAxis('bottom')
        axis.setPen(pg.mkPen('k'))
        axis.setTextPen('k')

        axis = self.getAxis('left')
        axis.setPen(pg.mkPen('k'))
        axis.setTextPen('k')

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
        if min_array > - 10 and max_array < 10:
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


class SignalThread(QtCore.QThread):
    signal = QtCore.pyqtSignal(name="signal")

    def __init__(self, plot_frequency: float):
        super(SignalThread, self).__init__(parent=None)

        self.period = int(1 / plot_frequency * 1000)
        self.t0 = time.time()
        self.stop_flag = False

    def run(self):
        timer = QtCore.QTimer(parent=None)
        timer.timeout.connect(self.signal.emit)
        timer.start(self.period)

        while not self.stop_flag:
            QtCore.QCoreApplication.processEvents()

        self.quit()

    def stop(self):
        self.stop_flag = True

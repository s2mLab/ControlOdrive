import sys
import random
import numpy as np

from untitled import Ui_MainWindow

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
import matplotlib.pyplot as plt
import numpy as np

class App(QtWidgets.QMainWindow):
    def __init__(self):
        super(App, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Add the sinus plot to the graphics view
        fig = plt.figure()
        ax = fig.add_subplot(111)
        x = range(100)
        y = [np.sin(i*np.pi/50) for i in x]
        ax.plot(x, y)
        ax.set_title('Sinus Function')

        canvas = FigureCanvas(fig)
        proxy = QtWidgets.QGraphicsProxyWidget()
        proxy.setWidget(canvas)

        self.ui.graphicsView.setScene(QtWidgets.QGraphicsScene(self.ui.graphicsView))
        self.ui.graphicsView.scene().addItem(proxy)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = App()
    gui.show()
    app.exec()
    app.run = False

# python -m PyQt5.uic.pyuic -x untitled.ui -o untitled.py
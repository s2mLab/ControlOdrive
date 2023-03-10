from main import Ui_MainWindow
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5 import QtCore

from motor import *


class App(QtWidgets.QMainWindow):
    def __init__(self):
        super(App, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.Power_pushButton.clicked.connect(self.action)
        self.ui.Power_pushButton.clicked.connect(self.power_output)
        self.motor = OdriveEncoderHall()
        self.ui.Power_lineEdit.setText(f"{self.motor.get_velocity()}")

    def action(self):
        print("hello")

    def power_output(self):
        pass


if __name__ == "__main__":

    app = QApplication(sys.argv)
    gui = App()
    gui.show()
    app.exec()

# python -m PyQt5.uic.pyuic -x [FILENAME].ui -o [FILENAME].py


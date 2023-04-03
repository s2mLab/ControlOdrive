# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ergocycle_gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1022, 398)
        MainWindow.setMinimumSize(QtCore.QSize(0, 0))
        MainWindow.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.power_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.power_pushButton.setObjectName("power_pushButton")
        self.gridLayout.addWidget(self.power_pushButton, 2, 0, 1, 1)
        self.instructions_verticalLayout = QtWidgets.QVBoxLayout()
        self.instructions_verticalLayout.setObjectName("instructions_verticalLayout")
        self.control_label = QtWidgets.QLabel(self.centralwidget)
        self.control_label.setScaledContents(False)
        self.control_label.setObjectName("control_label")
        self.instructions_verticalLayout.addWidget(self.control_label)
        self.training_comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.training_comboBox.setEnabled(True)
        self.training_comboBox.setObjectName("training_comboBox")
        self.training_comboBox.addItem("")
        self.training_comboBox.addItem("")
        self.instructions_verticalLayout.addWidget(self.training_comboBox)
        self.instructions_horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.instructions_horizontalLayout_2.setObjectName("instructions_horizontalLayout_2")
        self.instruction_spinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.instruction_spinBox.setObjectName("instruction_spinBox")
        self.instructions_horizontalLayout_2.addWidget(self.instruction_spinBox)
        self.units_label = QtWidgets.QLabel(self.centralwidget)
        self.units_label.setText("")
        self.units_label.setObjectName("units_label")
        self.instructions_horizontalLayout_2.addWidget(self.units_label)
        self.instructions_verticalLayout.addLayout(self.instructions_horizontalLayout_2)
        self.graph_horizontalLayout = QtWidgets.QHBoxLayout()
        self.graph_horizontalLayout.setObjectName("graph_horizontalLayout")
        self.errors_label = QtWidgets.QLabel(self.centralwidget)
        self.errors_label.setObjectName("errors_label")
        self.graph_horizontalLayout.addWidget(self.errors_label)
        self.instructions_verticalLayout.addLayout(self.graph_horizontalLayout)
        self.gridLayout.addLayout(self.instructions_verticalLayout, 0, 2, 1, 1)
        self.power_horizontalLayout = QtWidgets.QHBoxLayout()
        self.power_horizontalLayout.setObjectName("power_horizontalLayout")
        self.power_label = QtWidgets.QLabel(self.centralwidget)
        self.power_label.setObjectName("power_label")
        self.power_horizontalLayout.addWidget(self.power_label)
        self.power_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.power_lineEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.power_lineEdit.sizePolicy().hasHeightForWidth())
        self.power_lineEdit.setSizePolicy(sizePolicy)
        self.power_lineEdit.setText("")
        self.power_lineEdit.setObjectName("power_lineEdit")
        self.power_horizontalLayout.addWidget(self.power_lineEdit)
        self.gridLayout.addLayout(self.power_horizontalLayout, 2, 3, 1, 1)
        self.BAU_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.BAU_pushButton.setObjectName("BAU_pushButton")
        self.gridLayout.addWidget(self.BAU_pushButton, 0, 3, 1, 1)
        self.velocity_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.velocity_pushButton.setObjectName("velocity_pushButton")
        self.gridLayout.addWidget(self.velocity_pushButton, 3, 0, 1, 1)
        self.torque_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.torque_pushButton.setObjectName("torque_pushButton")
        self.gridLayout.addWidget(self.torque_pushButton, 5, 0, 1, 1)
        self.torque_horizontalLayout = QtWidgets.QHBoxLayout()
        self.torque_horizontalLayout.setObjectName("torque_horizontalLayout")
        self.torque_label = QtWidgets.QLabel(self.centralwidget)
        self.torque_label.setObjectName("torque_label")
        self.torque_horizontalLayout.addWidget(self.torque_label)
        self.torque_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.torque_lineEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.torque_lineEdit.sizePolicy().hasHeightForWidth())
        self.torque_lineEdit.setSizePolicy(sizePolicy)
        self.torque_lineEdit.setText("")
        self.torque_lineEdit.setObjectName("torque_lineEdit")
        self.torque_horizontalLayout.addWidget(self.torque_lineEdit)
        self.gridLayout.addLayout(self.torque_horizontalLayout, 4, 3, 1, 1)
        self.linear_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.linear_pushButton.setObjectName("linear_pushButton")
        self.gridLayout.addWidget(self.linear_pushButton, 4, 0, 1, 1)
        self.STOP_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.STOP_pushButton.setObjectName("STOP_pushButton")
        self.gridLayout.addWidget(self.STOP_pushButton, 0, 0, 1, 1)
        self.velocity_horizontalLayout = QtWidgets.QHBoxLayout()
        self.velocity_horizontalLayout.setObjectName("velocity_horizontalLayout")
        self.velocity_label = QtWidgets.QLabel(self.centralwidget)
        self.velocity_label.setObjectName("velocity_label")
        self.velocity_horizontalLayout.addWidget(self.velocity_label)
        self.velocity_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.velocity_lineEdit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.velocity_lineEdit.sizePolicy().hasHeightForWidth())
        self.velocity_lineEdit.setSizePolicy(sizePolicy)
        self.velocity_lineEdit.setText("")
        self.velocity_lineEdit.setObjectName("velocity_lineEdit")
        self.velocity_horizontalLayout.addWidget(self.velocity_lineEdit)
        self.gridLayout.addLayout(self.velocity_horizontalLayout, 3, 3, 1, 1)
        self.save_verticalLayout = QtWidgets.QVBoxLayout()
        self.save_verticalLayout.setObjectName("save_verticalLayout")
        self.save_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.save_lineEdit.setObjectName("save_lineEdit")
        self.save_verticalLayout.addWidget(self.save_lineEdit)
        self.save_horizontalLayout = QtWidgets.QHBoxLayout()
        self.save_horizontalLayout.setObjectName("save_horizontalLayout")
        self.save_start_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.save_start_pushButton.setObjectName("save_start_pushButton")
        self.save_horizontalLayout.addWidget(self.save_start_pushButton)
        self.save_stop_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.save_stop_pushButton.setObjectName("save_stop_pushButton")
        self.save_horizontalLayout.addWidget(self.save_stop_pushButton)
        self.save_verticalLayout.addLayout(self.save_horizontalLayout)
        self.gridLayout.addLayout(self.save_verticalLayout, 1, 3, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem, 1, 2, 1, 1)
        self.test_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.test_pushButton.setObjectName("test_pushButton")
        self.gridLayout.addWidget(self.test_pushButton, 1, 0, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1022, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.power_pushButton.setText(_translate("MainWindow", "Power control"))
        self.control_label.setText(_translate("MainWindow", "<html><head/><body><p>Choose a control mode (power, velocity, torque),</p><p>a training mode and an instruction.</p></body></html>"))
        self.training_comboBox.setItemText(0, _translate("MainWindow", "Eccentric"))
        self.training_comboBox.setItemText(1, _translate("MainWindow", "Concentric"))
        self.errors_label.setText(_translate("MainWindow", "TextLabel"))
        self.power_label.setText(_translate("MainWindow", "Power (W)"))
        self.BAU_pushButton.setText(_translate("MainWindow", "BAU"))
        self.velocity_pushButton.setText(_translate("MainWindow", "Velocity control"))
        self.torque_pushButton.setText(_translate("MainWindow", "Torque control"))
        self.torque_label.setText(_translate("MainWindow", "Torque (Nm)"))
        self.linear_pushButton.setText(_translate("MainWindow", "Linear control"))
        self.STOP_pushButton.setText(_translate("MainWindow", "STOP"))
        self.velocity_label.setText(_translate("MainWindow", "Velocity (tr/min)"))
        self.save_lineEdit.setText(_translate("MainWindow", "XP_participantX"))
        self.save_start_pushButton.setText(_translate("MainWindow", "Start"))
        self.save_stop_pushButton.setText(_translate("MainWindow", "Stop"))
        self.test_pushButton.setText(_translate("MainWindow", "Test"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

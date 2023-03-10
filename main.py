# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(667, 600)
        MainWindow.setMinimumSize(QtCore.QSize(0, 0))
        MainWindow.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.Power_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.Power_pushButton.setObjectName("Power_pushButton")
        self.verticalLayout.addWidget(self.Power_pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout.addWidget(self.pushButton_2)
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.spinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox.setObjectName("spinBox")
        self.horizontalLayout_2.addWidget(self.spinBox)
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setEditable(False)
        self.comboBox.setObjectName("comboBox")
        self.horizontalLayout_2.addWidget(self.comboBox)
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setObjectName("pushButton_4")
        self.horizontalLayout_2.addWidget(self.pushButton_4)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(20, 300, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.gridLayout.addLayout(self.verticalLayout_2, 0, 1, 1, 1)
        self.data_verticalLayout = QtWidgets.QVBoxLayout()
        self.data_verticalLayout.setObjectName("data_verticalLayout")
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
        self.data_verticalLayout.addLayout(self.power_horizontalLayout)
        self.velocity_horizontalLayout = QtWidgets.QHBoxLayout()
        self.velocity_horizontalLayout.setObjectName("velocity_horizontalLayout")
        self.velocity_label = QtWidgets.QLabel(self.centralwidget)
        self.velocity_label.setObjectName("velocity_label")
        self.velocity_horizontalLayout.addWidget(self.velocity_label)
        self.velocity_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.velocity_lineEdit.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.velocity_lineEdit.sizePolicy().hasHeightForWidth())
        self.velocity_lineEdit.setSizePolicy(sizePolicy)
        self.velocity_lineEdit.setText("")
        self.velocity_lineEdit.setObjectName("velocity_lineEdit")
        self.velocity_horizontalLayout.addWidget(self.velocity_lineEdit)
        self.data_verticalLayout.addLayout(self.velocity_horizontalLayout)
        self.torque_horizontalLayout = QtWidgets.QHBoxLayout()
        self.torque_horizontalLayout.setObjectName("torque_horizontalLayout")
        self.torque_label = QtWidgets.QLabel(self.centralwidget)
        self.torque_label.setObjectName("torque_label")
        self.torque_horizontalLayout.addWidget(self.torque_label)
        self.torque_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.torque_lineEdit.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.torque_lineEdit.sizePolicy().hasHeightForWidth())
        self.torque_lineEdit.setSizePolicy(sizePolicy)
        self.torque_lineEdit.setText("")
        self.torque_lineEdit.setObjectName("torque_lineEdit")
        self.torque_horizontalLayout.addWidget(self.torque_lineEdit)
        self.data_verticalLayout.addLayout(self.torque_horizontalLayout)
        self.gridLayout.addLayout(self.data_verticalLayout, 0, 2, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 667, 22))
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
        self.Power_pushButton.setText(_translate("MainWindow", "Power"))
        self.pushButton_2.setText(_translate("MainWindow", "Vitesse"))
        self.pushButton.setText(_translate("MainWindow", "Torque"))
        self.pushButton_4.setText(_translate("MainWindow", "PushButton"))
        self.power_label.setText(_translate("MainWindow", "Power (W)"))
        self.velocity_label.setText(_translate("MainWindow", "Velocity (tr/min)"))
        self.torque_label.setText(_translate("MainWindow", "Torque (Nm)"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

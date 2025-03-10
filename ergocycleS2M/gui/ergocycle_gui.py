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
        MainWindow.resize(1060, 780)
        MainWindow.setMinimumSize(QtCore.QSize(0, 0))
        MainWindow.setMaximumSize(QtCore.QSize(1060, 16777215))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.cadence_verticalLayout = QtWidgets.QVBoxLayout()
        self.cadence_verticalLayout.setObjectName("cadence_verticalLayout")
        self.cadence_label = QtWidgets.QLabel(self.centralwidget)
        self.cadence_label.setMinimumSize(QtCore.QSize(0, 0))
        self.cadence_label.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.cadence_label.setFont(font)
        self.cadence_label.setAlignment(QtCore.Qt.AlignCenter)
        self.cadence_label.setObjectName("cadence_label")
        self.cadence_verticalLayout.addWidget(self.cadence_label)
        self.cadence_horizontalLayout = QtWidgets.QHBoxLayout()
        self.cadence_horizontalLayout.setObjectName("cadence_horizontalLayout")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.current_cadence_label = QtWidgets.QLabel(self.centralwidget)
        self.current_cadence_label.setMaximumSize(QtCore.QSize(140, 16777215))
        self.current_cadence_label.setObjectName("current_cadence_label")
        self.verticalLayout_3.addWidget(self.current_cadence_label)
        self.current_cadence_display = QtWidgets.QLabel(self.centralwidget)
        self.current_cadence_display.setMinimumSize(QtCore.QSize(140, 0))
        self.current_cadence_display.setMaximumSize(QtCore.QSize(140, 70))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.current_cadence_display.setFont(font)
        self.current_cadence_display.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter
        )
        self.current_cadence_display.setObjectName("current_cadence_display")
        self.verticalLayout_3.addWidget(self.current_cadence_display)
        self.average_cadence_label = QtWidgets.QLabel(self.centralwidget)
        self.average_cadence_label.setMaximumSize(QtCore.QSize(140, 16777215))
        self.average_cadence_label.setObjectName("average_cadence_label")
        self.verticalLayout_3.addWidget(self.average_cadence_label)
        self.average_cadence_display = QtWidgets.QLabel(self.centralwidget)
        self.average_cadence_display.setMaximumSize(QtCore.QSize(140, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.average_cadence_display.setFont(font)
        self.average_cadence_display.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter
        )
        self.average_cadence_display.setObjectName("average_cadence_display")
        self.verticalLayout_3.addWidget(self.average_cadence_display)
        self.cadence_horizontalLayout.addLayout(self.verticalLayout_3)
        self.cadence_verticalLayout.addLayout(self.cadence_horizontalLayout)
        self.gridLayout_2.addLayout(self.cadence_verticalLayout, 4, 0, 1, 1)
        self.power_verticalLayout = QtWidgets.QVBoxLayout()
        self.power_verticalLayout.setObjectName("power_verticalLayout")
        self.power_label = QtWidgets.QLabel(self.centralwidget)
        self.power_label.setMinimumSize(QtCore.QSize(0, 0))
        self.power_label.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.power_label.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.power_label.setFont(font)
        self.power_label.setScaledContents(False)
        self.power_label.setAlignment(QtCore.Qt.AlignCenter)
        self.power_label.setWordWrap(False)
        self.power_label.setObjectName("power_label")
        self.power_verticalLayout.addWidget(self.power_label)
        self.power_horizontalLayout = QtWidgets.QHBoxLayout()
        self.power_horizontalLayout.setObjectName("power_horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.current_power_label = QtWidgets.QLabel(self.centralwidget)
        self.current_power_label.setMaximumSize(QtCore.QSize(140, 16777215))
        self.current_power_label.setObjectName("current_power_label")
        self.verticalLayout_2.addWidget(self.current_power_label)
        self.current_power_display = QtWidgets.QLabel(self.centralwidget)
        self.current_power_display.setMinimumSize(QtCore.QSize(140, 0))
        self.current_power_display.setMaximumSize(QtCore.QSize(140, 70))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.current_power_display.setFont(font)
        self.current_power_display.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.current_power_display.setObjectName("current_power_display")
        self.verticalLayout_2.addWidget(self.current_power_display)
        self.average_power_label = QtWidgets.QLabel(self.centralwidget)
        self.average_power_label.setMaximumSize(QtCore.QSize(140, 16777215))
        self.average_power_label.setObjectName("average_power_label")
        self.verticalLayout_2.addWidget(self.average_power_label)
        self.average_power_display = QtWidgets.QLabel(self.centralwidget)
        self.average_power_display.setMaximumSize(QtCore.QSize(140, 70))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.average_power_display.setFont(font)
        self.average_power_display.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.average_power_display.setObjectName("average_power_display")
        self.verticalLayout_2.addWidget(self.average_power_display)
        self.power_horizontalLayout.addLayout(self.verticalLayout_2)
        self.power_verticalLayout.addLayout(self.power_horizontalLayout)
        self.gridLayout_2.addLayout(self.power_verticalLayout, 3, 0, 1, 1)
        self.command_verticalLayout = QtWidgets.QVBoxLayout()
        self.command_verticalLayout.setObjectName("command_verticalLayout")
        self.header_horizontalLayout = QtWidgets.QHBoxLayout()
        self.header_horizontalLayout.setObjectName("header_horizontalLayout")
        self.date_label = QtWidgets.QLabel(self.centralwidget)
        self.date_label.setText("")
        self.date_label.setObjectName("date_label")
        self.header_horizontalLayout.addWidget(self.date_label)
        self.errors_label = QtWidgets.QLabel(self.centralwidget)
        self.errors_label.setMinimumSize(QtCore.QSize(284, 0))
        self.errors_label.setMaximumSize(QtCore.QSize(16777215, 50))
        self.errors_label.setText("")
        self.errors_label.setWordWrap(True)
        self.errors_label.setObjectName("errors_label")
        self.header_horizontalLayout.addWidget(self.errors_label)
        self.emergency_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.emergency_pushButton.setMinimumSize(QtCore.QSize(0, 50))
        self.emergency_pushButton.setMaximumSize(QtCore.QSize(360, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.emergency_pushButton.setFont(font)
        self.emergency_pushButton.setAutoFillBackground(False)
        self.emergency_pushButton.setObjectName("emergency_pushButton")
        self.header_horizontalLayout.addWidget(self.emergency_pushButton)
        self.command_verticalLayout.addLayout(self.header_horizontalLayout)
        self.line_4 = QtWidgets.QFrame(self.centralwidget)
        self.line_4.setLineWidth(0)
        self.line_4.setMidLineWidth(20)
        self.line_4.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.command_verticalLayout.addWidget(self.line_4)
        self.command_horizontalLayout = QtWidgets.QHBoxLayout()
        self.command_horizontalLayout.setObjectName("command_horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.gear_label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(11)
        self.gear_label.setFont(font)
        self.gear_label.setObjectName("gear_label")
        self.verticalLayout.addWidget(self.gear_label)
        self.gear_spinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.gear_spinBox.setMaximumSize(QtCore.QSize(50, 16777215))
        self.gear_spinBox.setObjectName("gear_spinBox")
        self.verticalLayout.addWidget(self.gear_spinBox)
        self.command_horizontalLayout.addLayout(self.verticalLayout)
        self.line_5 = QtWidgets.QFrame(self.centralwidget)
        self.line_5.setLineWidth(0)
        self.line_5.setMidLineWidth(20)
        self.line_5.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.command_horizontalLayout.addWidget(self.line_5)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.verticalLayout_8.addWidget(self.label)
        self.angle_reset_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.angle_reset_pushButton.setMinimumSize(QtCore.QSize(80, 0))
        self.angle_reset_pushButton.setMaximumSize(QtCore.QSize(120, 16777215))
        self.angle_reset_pushButton.setAutoDefault(False)
        self.angle_reset_pushButton.setObjectName("angle_reset_pushButton")
        self.verticalLayout_8.addWidget(self.angle_reset_pushButton)
        self.command_horizontalLayout.addLayout(self.verticalLayout_8)
        self.modes_verticalLayout = QtWidgets.QVBoxLayout()
        self.modes_verticalLayout.setObjectName("modes_verticalLayout")
        self.training_comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.training_comboBox.setEnabled(True)
        self.training_comboBox.setMinimumSize(QtCore.QSize(120, 0))
        self.training_comboBox.setMaximumSize(QtCore.QSize(120, 16777215))
        self.training_comboBox.setObjectName("training_comboBox")
        self.modes_verticalLayout.addWidget(self.training_comboBox)
        self.control_comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.control_comboBox.setMinimumSize(QtCore.QSize(120, 0))
        self.control_comboBox.setMaximumSize(QtCore.QSize(120, 16777215))
        self.control_comboBox.setObjectName("control_comboBox")
        self.modes_verticalLayout.addWidget(self.control_comboBox)
        self.direction_comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.direction_comboBox.setMinimumSize(QtCore.QSize(120, 0))
        self.direction_comboBox.setMaximumSize(QtCore.QSize(120, 16777215))
        self.direction_comboBox.setObjectName("direction_comboBox")
        self.modes_verticalLayout.addWidget(self.direction_comboBox)
        self.command_horizontalLayout.addLayout(self.modes_verticalLayout)
        self.instructions_gridLayout = QtWidgets.QGridLayout()
        self.instructions_gridLayout.setObjectName("instructions_gridLayout")
        self.acceleration_horizontalLayout = QtWidgets.QHBoxLayout()
        self.acceleration_horizontalLayout.setObjectName("acceleration_horizontalLayout")
        self.acceleration_spinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.acceleration_spinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.acceleration_spinBox.setMaximumSize(QtCore.QSize(80, 16777215))
        self.acceleration_spinBox.setObjectName("acceleration_spinBox")
        self.acceleration_horizontalLayout.addWidget(self.acceleration_spinBox)
        self.acceleration_units_label = QtWidgets.QLabel(self.centralwidget)
        self.acceleration_units_label.setMinimumSize(QtCore.QSize(100, 0))
        self.acceleration_units_label.setMaximumSize(QtCore.QSize(100, 16777215))
        self.acceleration_units_label.setText("")
        self.acceleration_units_label.setObjectName("acceleration_units_label")
        self.acceleration_horizontalLayout.addWidget(self.acceleration_units_label)
        self.instructions_gridLayout.addLayout(self.acceleration_horizontalLayout, 1, 2, 1, 1)
        self.instruction_horizontalLayout = QtWidgets.QHBoxLayout()
        self.instruction_horizontalLayout.setObjectName("instruction_horizontalLayout")
        self.instruction_spinBox = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.instruction_spinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.instruction_spinBox.setMaximumSize(QtCore.QSize(80, 16777215))
        self.instruction_spinBox.setObjectName("instruction_spinBox")
        self.instruction_horizontalLayout.addWidget(self.instruction_spinBox)
        self.units_label = QtWidgets.QLabel(self.centralwidget)
        self.units_label.setMinimumSize(QtCore.QSize(100, 0))
        self.units_label.setMaximumSize(QtCore.QSize(100, 16777215))
        self.units_label.setText("")
        self.units_label.setObjectName("units_label")
        self.instruction_horizontalLayout.addWidget(self.units_label)
        self.instructions_gridLayout.addLayout(self.instruction_horizontalLayout, 1, 0, 1, 1)
        self.stop_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.stop_pushButton.setMinimumSize(QtCore.QSize(180, 0))
        self.stop_pushButton.setMaximumSize(QtCore.QSize(180, 16777215))
        self.stop_pushButton.setObjectName("stop_pushButton")
        self.instructions_gridLayout.addWidget(self.stop_pushButton, 2, 2, 1, 1)
        self.start_update_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.start_update_pushButton.setMinimumSize(QtCore.QSize(180, 0))
        self.start_update_pushButton.setMaximumSize(QtCore.QSize(180, 16777215))
        self.start_update_pushButton.setObjectName("start_update_pushButton")
        self.instructions_gridLayout.addWidget(self.start_update_pushButton, 2, 0, 1, 1)
        self.instruction_label = QtWidgets.QLabel(self.centralwidget)
        self.instruction_label.setObjectName("instruction_label")
        self.instructions_gridLayout.addWidget(self.instruction_label, 0, 0, 1, 1)
        self.ramp_label = QtWidgets.QLabel(self.centralwidget)
        self.ramp_label.setObjectName("ramp_label")
        self.instructions_gridLayout.addWidget(self.ramp_label, 0, 2, 1, 1)
        self.command_horizontalLayout.addLayout(self.instructions_gridLayout)
        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setLineWidth(0)
        self.line_2.setMidLineWidth(20)
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.command_horizontalLayout.addWidget(self.line_2)
        self.save_verticalLayout = QtWidgets.QVBoxLayout()
        self.save_verticalLayout.setObjectName("save_verticalLayout")
        self.save_horizontalLayout = QtWidgets.QHBoxLayout()
        self.save_horizontalLayout.setObjectName("save_horizontalLayout")
        self.save_label = QtWidgets.QLabel(self.centralwidget)
        self.save_label.setMinimumSize(QtCore.QSize(100, 0))
        self.save_label.setMaximumSize(QtCore.QSize(100, 16777215))
        self.save_label.setObjectName("save_label")
        self.save_horizontalLayout.addWidget(self.save_label)
        self.save_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.save_lineEdit.setMinimumSize(QtCore.QSize(100, 0))
        self.save_lineEdit.setMaximumSize(QtCore.QSize(200, 16777215))
        self.save_lineEdit.setObjectName("save_lineEdit")
        self.save_horizontalLayout.addWidget(self.save_lineEdit)
        self.save_st_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.save_st_pushButton.setMinimumSize(QtCore.QSize(110, 0))
        self.save_st_pushButton.setMaximumSize(QtCore.QSize(110, 16777215))
        self.save_st_pushButton.setObjectName("save_st_pushButton")
        self.save_horizontalLayout.addWidget(self.save_st_pushButton)
        self.save_verticalLayout.addLayout(self.save_horizontalLayout)
        self.comments_horizontalLayout = QtWidgets.QHBoxLayout()
        self.comments_horizontalLayout.setObjectName("comments_horizontalLayout")
        self.comments_label = QtWidgets.QLabel(self.centralwidget)
        self.comments_label.setMinimumSize(QtCore.QSize(100, 0))
        self.comments_label.setMaximumSize(QtCore.QSize(100, 16777215))
        self.comments_label.setObjectName("comments_label")
        self.comments_horizontalLayout.addWidget(self.comments_label)
        self.comments_lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.comments_lineEdit.setMinimumSize(QtCore.QSize(100, 0))
        self.comments_lineEdit.setMaximumSize(QtCore.QSize(200, 16777215))
        self.comments_lineEdit.setObjectName("comments_lineEdit")
        self.comments_horizontalLayout.addWidget(self.comments_lineEdit)
        self.comments_save_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.comments_save_pushButton.setMinimumSize(QtCore.QSize(110, 0))
        self.comments_save_pushButton.setMaximumSize(QtCore.QSize(110, 16777215))
        self.comments_save_pushButton.setObjectName("comments_save_pushButton")
        self.comments_horizontalLayout.addWidget(self.comments_save_pushButton)
        self.save_verticalLayout.addLayout(self.comments_horizontalLayout)
        self.command_horizontalLayout.addLayout(self.save_verticalLayout)
        self.line_3 = QtWidgets.QFrame(self.centralwidget)
        self.line_3.setLineWidth(0)
        self.line_3.setMidLineWidth(20)
        self.line_3.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.command_horizontalLayout.addWidget(self.line_3)
        self.stopwatch_verticalLayout = QtWidgets.QVBoxLayout()
        self.stopwatch_verticalLayout.setObjectName("stopwatch_verticalLayout")
        self.stopwatch_lcdNumber = QtWidgets.QLCDNumber(self.centralwidget)
        self.stopwatch_lcdNumber.setMinimumSize(QtCore.QSize(168, 0))
        self.stopwatch_lcdNumber.setMaximumSize(QtCore.QSize(168, 16777215))
        self.stopwatch_lcdNumber.setObjectName("stopwatch_lcdNumber")
        self.stopwatch_verticalLayout.addWidget(self.stopwatch_lcdNumber)
        self.lap_horizontalLayout = QtWidgets.QHBoxLayout()
        self.lap_horizontalLayout.setObjectName("lap_horizontalLayout")
        self.lap_label = QtWidgets.QLabel(self.centralwidget)
        self.lap_label.setMinimumSize(QtCore.QSize(80, 0))
        self.lap_label.setObjectName("lap_label")
        self.lap_horizontalLayout.addWidget(self.lap_label)
        self.lap_lcdNumber = QtWidgets.QLCDNumber(self.centralwidget)
        self.lap_lcdNumber.setMinimumSize(QtCore.QSize(80, 0))
        self.lap_lcdNumber.setObjectName("lap_lcdNumber")
        self.lap_horizontalLayout.addWidget(self.lap_lcdNumber)
        self.stopwatch_verticalLayout.addLayout(self.lap_horizontalLayout)
        self.stopwatch_horizontalLayout = QtWidgets.QHBoxLayout()
        self.stopwatch_horizontalLayout.setObjectName("stopwatch_horizontalLayout")
        self.stopwatch_lap_reset_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopwatch_lap_reset_pushButton.setMinimumSize(QtCore.QSize(80, 0))
        self.stopwatch_lap_reset_pushButton.setMaximumSize(QtCore.QSize(80, 16777215))
        self.stopwatch_lap_reset_pushButton.setObjectName("stopwatch_lap_reset_pushButton")
        self.stopwatch_horizontalLayout.addWidget(self.stopwatch_lap_reset_pushButton)
        self.stopwatch_start_stop_pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopwatch_start_stop_pushButton.setMinimumSize(QtCore.QSize(80, 0))
        self.stopwatch_start_stop_pushButton.setMaximumSize(QtCore.QSize(80, 16777215))
        self.stopwatch_start_stop_pushButton.setObjectName("stopwatch_start_stop_pushButton")
        self.stopwatch_horizontalLayout.addWidget(self.stopwatch_start_stop_pushButton)
        self.stopwatch_verticalLayout.addLayout(self.stopwatch_horizontalLayout)
        self.command_horizontalLayout.addLayout(self.stopwatch_verticalLayout)
        self.command_verticalLayout.addLayout(self.command_horizontalLayout)
        self.gridLayout_2.addLayout(self.command_verticalLayout, 0, 0, 1, 1)
        self.torque_verticalLayout = QtWidgets.QVBoxLayout()
        self.torque_verticalLayout.setObjectName("torque_verticalLayout")
        self.torque_label = QtWidgets.QLabel(self.centralwidget)
        self.torque_label.setMinimumSize(QtCore.QSize(0, 0))
        self.torque_label.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.torque_label.setFont(font)
        self.torque_label.setAlignment(QtCore.Qt.AlignCenter)
        self.torque_label.setObjectName("torque_label")
        self.torque_verticalLayout.addWidget(self.torque_label)
        self.torque_horizontalLayout = QtWidgets.QHBoxLayout()
        self.torque_horizontalLayout.setObjectName("torque_horizontalLayout")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.current_torque_label = QtWidgets.QLabel(self.centralwidget)
        self.current_torque_label.setMaximumSize(QtCore.QSize(140, 16777215))
        self.current_torque_label.setObjectName("current_torque_label")
        self.verticalLayout_4.addWidget(self.current_torque_label)
        self.current_torque_display = QtWidgets.QLabel(self.centralwidget)
        self.current_torque_display.setMinimumSize(QtCore.QSize(140, 0))
        self.current_torque_display.setMaximumSize(QtCore.QSize(140, 70))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.current_torque_display.setFont(font)
        self.current_torque_display.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter
        )
        self.current_torque_display.setObjectName("current_torque_display")
        self.verticalLayout_4.addWidget(self.current_torque_display)
        self.average_torque_label = QtWidgets.QLabel(self.centralwidget)
        self.average_torque_label.setMaximumSize(QtCore.QSize(140, 16777215))
        self.average_torque_label.setObjectName("average_torque_label")
        self.verticalLayout_4.addWidget(self.average_torque_label)
        self.average_torque_display = QtWidgets.QLabel(self.centralwidget)
        self.average_torque_display.setMaximumSize(QtCore.QSize(140, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.average_torque_display.setFont(font)
        self.average_torque_display.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter
        )
        self.average_torque_display.setObjectName("average_torque_display")
        self.verticalLayout_4.addWidget(self.average_torque_display)
        self.torque_horizontalLayout.addLayout(self.verticalLayout_4)
        self.torque_verticalLayout.addLayout(self.torque_horizontalLayout)
        self.gridLayout_2.addLayout(self.torque_verticalLayout, 6, 0, 1, 1)
        self.turns_horizontalLayout = QtWidgets.QHBoxLayout()
        self.turns_horizontalLayout.setObjectName("turns_horizontalLayout")
        self.turns_label = QtWidgets.QLabel(self.centralwidget)
        self.turns_label.setEnabled(True)
        self.turns_label.setMinimumSize(QtCore.QSize(140, 25))
        self.turns_label.setMaximumSize(QtCore.QSize(400, 25))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.turns_label.setFont(font)
        self.turns_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.turns_label.setObjectName("turns_label")
        self.turns_horizontalLayout.addWidget(self.turns_label)
        self.turns_display = QtWidgets.QLabel(self.centralwidget)
        self.turns_display.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.turns_display.setFont(font)
        self.turns_display.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.turns_display.setObjectName("turns_display")
        self.turns_horizontalLayout.addWidget(self.turns_display)
        self.angle_label = QtWidgets.QLabel(self.centralwidget)
        self.angle_label.setMinimumSize(QtCore.QSize(140, 25))
        self.angle_label.setMaximumSize(QtCore.QSize(450, 25))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.angle_label.setFont(font)
        self.angle_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.angle_label.setObjectName("angle_label")
        self.turns_horizontalLayout.addWidget(self.angle_label)
        self.angle_display = QtWidgets.QLabel(self.centralwidget)
        self.angle_display.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.angle_display.setFont(font)
        self.angle_display.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.angle_display.setObjectName("angle_display")
        self.turns_horizontalLayout.addWidget(self.angle_display)
        self.gridLayout_2.addLayout(self.turns_horizontalLayout, 2, 0, 1, 1)
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setLineWidth(0)
        self.line.setMidLineWidth(20)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setObjectName("line")
        self.gridLayout_2.addWidget(self.line, 1, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1060, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Ergocycle S2M"))
        self.cadence_label.setText(_translate("MainWindow", "Cadence"))
        self.current_cadence_label.setText(_translate("MainWindow", "Current cadence"))
        self.current_cadence_display.setText(_translate("MainWindow", "TextLabel"))
        self.average_cadence_label.setText(_translate("MainWindow", "Average cadence"))
        self.average_cadence_display.setText(_translate("MainWindow", "TextLabel"))
        self.power_label.setText(_translate("MainWindow", "Power"))
        self.current_power_label.setText(_translate("MainWindow", "Current power"))
        self.current_power_display.setText(_translate("MainWindow", "TextLabel"))
        self.average_power_label.setText(_translate("MainWindow", "Average power"))
        self.average_power_display.setText(_translate("MainWindow", "TextLabel"))
        self.emergency_pushButton.setText(_translate("MainWindow", "EMERGENCY STOP"))
        self.gear_label.setText(_translate("MainWindow", "Gear"))
        self.label.setText(_translate("MainWindow", "Reset the angle and turns"))
        self.angle_reset_pushButton.setText(_translate("MainWindow", "0 ° and 0 tr"))
        self.stop_pushButton.setText(_translate("MainWindow", "Stop"))
        self.start_update_pushButton.setText(_translate("MainWindow", "Start"))
        self.instruction_label.setText(_translate("MainWindow", "TextLabel"))
        self.ramp_label.setText(_translate("MainWindow", "TextLabel"))
        self.save_label.setText(_translate("MainWindow", "File name:"))
        self.save_lineEdit.setText(_translate("MainWindow", "XP_participantX"))
        self.save_st_pushButton.setText(_translate("MainWindow", "Start saving"))
        self.comments_label.setText(_translate("MainWindow", "Comments:"))
        self.comments_save_pushButton.setText(_translate("MainWindow", "Save event"))
        self.lap_label.setText(_translate("MainWindow", "Current lap"))
        self.stopwatch_lap_reset_pushButton.setText(_translate("MainWindow", "Lap"))
        self.stopwatch_start_stop_pushButton.setText(_translate("MainWindow", "Start"))
        self.torque_label.setText(_translate("MainWindow", "Torque"))
        self.current_torque_label.setText(_translate("MainWindow", "Current torque"))
        self.current_torque_display.setText(_translate("MainWindow", "TextLabel"))
        self.average_torque_label.setText(_translate("MainWindow", "Average torque"))
        self.average_torque_display.setText(_translate("MainWindow", "TextLabel"))
        self.turns_label.setText(_translate("MainWindow", "Turns since first start or last reset  "))
        self.turns_display.setText(_translate("MainWindow", "TextLabel"))
        self.angle_label.setText(_translate("MainWindow", "Current angle  "))
        self.angle_display.setText(_translate("MainWindow", "TextLabel"))


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

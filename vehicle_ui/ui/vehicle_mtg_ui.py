# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'vehicle_mtg.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(899, 726)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.groupBox_operation_mode = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_operation_mode.setObjectName("groupBox_operation_mode")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox_operation_mode)
        self.gridLayout.setObjectName("gridLayout")
        self.label_state_operation_mode = QtWidgets.QLabel(self.groupBox_operation_mode)
        self.label_state_operation_mode.setMinimumSize(QtCore.QSize(0, 100))
        self.label_state_operation_mode.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.label_state_operation_mode.setStyleSheet("background-color: rgb(255, 120, 0);")
        self.label_state_operation_mode.setObjectName("label_state_operation_mode")
        self.gridLayout.addWidget(self.label_state_operation_mode, 0, 0, 2, 1)
        self.pushButton_auto = QtWidgets.QPushButton(self.groupBox_operation_mode)
        self.pushButton_auto.setEnabled(False)
        self.pushButton_auto.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_auto.setObjectName("pushButton_auto")
        self.gridLayout.addWidget(self.pushButton_auto, 0, 1, 1, 1)
        self.pushButton_stop = QtWidgets.QPushButton(self.groupBox_operation_mode)
        self.pushButton_stop.setEnabled(False)
        self.pushButton_stop.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.gridLayout.addWidget(self.pushButton_stop, 0, 2, 1, 1)
        self.pushButton_local = QtWidgets.QPushButton(self.groupBox_operation_mode)
        self.pushButton_local.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_local.setObjectName("pushButton_local")
        self.gridLayout.addWidget(self.pushButton_local, 1, 1, 1, 1)
        self.pushButton_remote = QtWidgets.QPushButton(self.groupBox_operation_mode)
        self.pushButton_remote.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_remote.setObjectName("pushButton_remote")
        self.gridLayout.addWidget(self.pushButton_remote, 1, 2, 1, 1)
        self.verticalLayout_2.addWidget(self.groupBox_operation_mode)
        self.groupBox_autoware_control = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_autoware_control.setObjectName("groupBox_autoware_control")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox_autoware_control)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_state_autoware_control = QtWidgets.QLabel(self.groupBox_autoware_control)
        self.label_state_autoware_control.setMinimumSize(QtCore.QSize(0, 50))
        self.label_state_autoware_control.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.label_state_autoware_control.setStyleSheet("background-color: rgb(51, 209, 122);")
        self.label_state_autoware_control.setObjectName("label_state_autoware_control")
        self.gridLayout_2.addWidget(self.label_state_autoware_control, 0, 0, 1, 1)
        self.pushButton_enable = QtWidgets.QPushButton(self.groupBox_autoware_control)
        self.pushButton_enable.setEnabled(False)
        self.pushButton_enable.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_enable.setObjectName("pushButton_enable")
        self.gridLayout_2.addWidget(self.pushButton_enable, 0, 1, 1, 1)
        self.pushButton_disable = QtWidgets.QPushButton(self.groupBox_autoware_control)
        self.pushButton_disable.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_disable.setObjectName("pushButton_disable")
        self.gridLayout_2.addWidget(self.pushButton_disable, 0, 2, 1, 1)
        self.verticalLayout_2.addWidget(self.groupBox_autoware_control)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.groupBox_routing = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_routing.setObjectName("groupBox_routing")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox_routing)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_state_routing = QtWidgets.QLabel(self.groupBox_routing)
        self.label_state_routing.setMinimumSize(QtCore.QSize(0, 50))
        self.label_state_routing.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_state_routing.setStyleSheet("background-color: rgb(249, 240, 107);")
        self.label_state_routing.setObjectName("label_state_routing")
        self.gridLayout_3.addWidget(self.label_state_routing, 0, 0, 1, 1)
        self.pushButton_clear_router = QtWidgets.QPushButton(self.groupBox_routing)
        self.pushButton_clear_router.setEnabled(True)
        self.pushButton_clear_router.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_clear_router.setObjectName("pushButton_clear_router")
        self.gridLayout_3.addWidget(self.pushButton_clear_router, 1, 0, 1, 1)
        self.horizontalLayout.addWidget(self.groupBox_routing)
        self.groupBox_Localization = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_Localization.setObjectName("groupBox_Localization")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.groupBox_Localization)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.label_state_localization = QtWidgets.QLabel(self.groupBox_Localization)
        self.label_state_localization.setMinimumSize(QtCore.QSize(0, 50))
        self.label_state_localization.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_state_localization.setStyleSheet("background-color: rgb(249, 240, 107);")
        self.label_state_localization.setObjectName("label_state_localization")
        self.gridLayout_4.addWidget(self.label_state_localization, 0, 0, 1, 1)
        self.pushButton_init_gnss = QtWidgets.QPushButton(self.groupBox_Localization)
        self.pushButton_init_gnss.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_init_gnss.setObjectName("pushButton_init_gnss")
        self.gridLayout_4.addWidget(self.pushButton_init_gnss, 1, 0, 1, 1)
        self.horizontalLayout.addWidget(self.groupBox_Localization)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.label_state_motion = QtWidgets.QLabel(self.groupBox)
        self.label_state_motion.setMinimumSize(QtCore.QSize(0, 50))
        self.label_state_motion.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_state_motion.setStyleSheet("background-color: rgb(255, 120, 0);")
        self.label_state_motion.setObjectName("label_state_motion")
        self.gridLayout_5.addWidget(self.label_state_motion, 0, 0, 1, 1)
        self.pushButton_accept_start = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_accept_start.setEnabled(False)
        self.pushButton_accept_start.setMinimumSize(QtCore.QSize(0, 50))
        self.pushButton_accept_start.setObjectName("pushButton_accept_start")
        self.gridLayout_5.addWidget(self.pushButton_accept_start, 1, 0, 1, 1)
        self.horizontalLayout.addWidget(self.groupBox)
        self.groupBox_failsafe = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_failsafe.setObjectName("groupBox_failsafe")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox_failsafe)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_failsafe1 = QtWidgets.QLabel(self.groupBox_failsafe)
        self.label_failsafe1.setMinimumSize(QtCore.QSize(0, 50))
        self.label_failsafe1.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_failsafe1.setStyleSheet("background-color: rgb(51, 209, 122);")
        self.label_failsafe1.setObjectName("label_failsafe1")
        self.gridLayout_6.addWidget(self.label_failsafe1, 0, 0, 1, 1)
        self.label_failsafe2 = QtWidgets.QLabel(self.groupBox_failsafe)
        self.label_failsafe2.setMinimumSize(QtCore.QSize(0, 50))
        self.label_failsafe2.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_failsafe2.setStyleSheet("background-color: rgb(51, 209, 122);")
        self.label_failsafe2.setObjectName("label_failsafe2")
        self.gridLayout_6.addWidget(self.label_failsafe2, 1, 0, 1, 1)
        self.horizontalLayout.addWidget(self.groupBox_failsafe)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.pushButton_station_1 = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_station_1.setMinimumSize(QtCore.QSize(0, 100))
        self.pushButton_station_1.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.pushButton_station_1.setFont(font)
        self.pushButton_station_1.setObjectName("pushButton_station_1")
        self.gridLayout_7.addWidget(self.pushButton_station_1, 0, 0, 1, 1)
        self.pushButton_station_2 = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_station_2.setMinimumSize(QtCore.QSize(0, 100))
        self.pushButton_station_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.pushButton_station_2.setFont(font)
        self.pushButton_station_2.setObjectName("pushButton_station_2")
        self.gridLayout_7.addWidget(self.pushButton_station_2, 0, 1, 1, 1)
        self.pushButton_station_3 = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_station_3.setMinimumSize(QtCore.QSize(0, 100))
        self.pushButton_station_3.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.pushButton_station_3.setFont(font)
        self.pushButton_station_3.setObjectName("pushButton_station_3")
        self.gridLayout_7.addWidget(self.pushButton_station_3, 0, 2, 1, 1)
        self.pushButton_station_4 = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_station_4.setEnabled(True)
        self.pushButton_station_4.setMinimumSize(QtCore.QSize(0, 100))
        self.pushButton_station_4.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.pushButton_station_4.setFont(font)
        self.pushButton_station_4.setInputMethodHints(QtCore.Qt.ImhNone)
        self.pushButton_station_4.setAutoDefault(False)
        self.pushButton_station_4.setDefault(False)
        self.pushButton_station_4.setFlat(False)
        self.pushButton_station_4.setObjectName("pushButton_station_4")
        self.gridLayout_7.addWidget(self.pushButton_station_4, 0, 3, 1, 1)
        self.verticalLayout_2.addWidget(self.groupBox_2)
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.groupBox_3)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_message_station = QtWidgets.QLabel(self.groupBox_3)
        self.label_message_station.setMinimumSize(QtCore.QSize(0, 50))
        self.label_message_station.setObjectName("label_message_station")
        self.verticalLayout.addWidget(self.label_message_station)
        self.verticalLayout_2.addWidget(self.groupBox_3)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 899, 28))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Autoware"))
        self.groupBox_operation_mode.setTitle(_translate("MainWindow", "OperationMode"))
        self.label_state_operation_mode.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">STOP</p></body></html>"))
        self.pushButton_auto.setText(_translate("MainWindow", "Auto"))
        self.pushButton_stop.setText(_translate("MainWindow", "Stop"))
        self.pushButton_local.setText(_translate("MainWindow", "Local"))
        self.pushButton_remote.setText(_translate("MainWindow", "Remote"))
        self.groupBox_autoware_control.setTitle(_translate("MainWindow", "AutowareControl"))
        self.label_state_autoware_control.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Enable</p></body></html>"))
        self.pushButton_enable.setText(_translate("MainWindow", "Enable"))
        self.pushButton_disable.setText(_translate("MainWindow", "Disable"))
        self.groupBox_routing.setTitle(_translate("MainWindow", "Routing"))
        self.label_state_routing.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">UNSET</p></body></html>"))
        self.pushButton_clear_router.setText(_translate("MainWindow", "Clear Router"))
        self.groupBox_Localization.setTitle(_translate("MainWindow", "Localization"))
        self.label_state_localization.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">UNINITIALIZED</p></body></html>"))
        self.pushButton_init_gnss.setText(_translate("MainWindow", "Init by GNSS"))
        self.groupBox.setTitle(_translate("MainWindow", "Motion"))
        self.label_state_motion.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">STOPPED</p></body></html>"))
        self.pushButton_accept_start.setText(_translate("MainWindow", "Accept Start"))
        self.groupBox_failsafe.setTitle(_translate("MainWindow", "FailSafe"))
        self.label_failsafe1.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">NONE</p></body></html>"))
        self.label_failsafe2.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">NONE</p></body></html>"))
        self.groupBox_2.setTitle(_translate("MainWindow", "站点"))
        self.pushButton_station_1.setText(_translate("MainWindow", "1"))
        self.pushButton_station_2.setText(_translate("MainWindow", "2"))
        self.pushButton_station_3.setText(_translate("MainWindow", "3"))
        self.pushButton_station_4.setText(_translate("MainWindow", "4"))
        self.groupBox_3.setTitle(_translate("MainWindow", "日志"))
        self.label_message_station.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><br/></p></body></html>"))

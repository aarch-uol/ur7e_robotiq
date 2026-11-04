from threading import Thread
import sys
from tkinter import N
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2ServoJoints
from pymoveit2 import MoveIt2Servo
from pymoveit2.robots import ur3

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):

    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(868, 393)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/Window/robotic-arm.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setStyleSheet("color: rgb(255, 140, 0);")
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.tab)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox.setFont(font)
        self.doubleSpinBox.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox.setSuffix(" m")
        self.doubleSpinBox.setDecimals(3)
        self.doubleSpinBox.setMinimum(-3.0)
        self.doubleSpinBox.setMaximum(3.0)
        self.doubleSpinBox.setSingleStep(0.005)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.gridLayout_2.addWidget(self.doubleSpinBox, 0, 0, 1, 1)
        self.doubleSpinBox_4 = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox_4.setFont(font)
        self.doubleSpinBox_4.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox_4.setSuffix("")
        self.doubleSpinBox_4.setDecimals(3)
        self.doubleSpinBox_4.setMinimum(-3.0)
        self.doubleSpinBox_4.setMaximum(3.0)
        self.doubleSpinBox_4.setSingleStep(0.005)
        self.doubleSpinBox_4.setObjectName("doubleSpinBox_4")
        self.gridLayout_2.addWidget(self.doubleSpinBox_4, 0, 1, 1, 1)
        self.doubleSpinBox_2 = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox_2.setFont(font)
        self.doubleSpinBox_2.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox_2.setSuffix(" m")
        self.doubleSpinBox_2.setDecimals(3)
        self.doubleSpinBox_2.setMinimum(-3.0)
        self.doubleSpinBox_2.setMaximum(3.0)
        self.doubleSpinBox_2.setSingleStep(0.005)
        self.doubleSpinBox_2.setObjectName("doubleSpinBox_2")
        self.gridLayout_2.addWidget(self.doubleSpinBox_2, 1, 0, 1, 1)
        self.doubleSpinBox_5 = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox_5.setFont(font)
        self.doubleSpinBox_5.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox_5.setSuffix("")
        self.doubleSpinBox_5.setDecimals(3)
        self.doubleSpinBox_5.setMinimum(-3.0)
        self.doubleSpinBox_5.setMaximum(3.0)
        self.doubleSpinBox_5.setSingleStep(0.005)
        self.doubleSpinBox_5.setObjectName("doubleSpinBox_5")
        self.gridLayout_2.addWidget(self.doubleSpinBox_5, 1, 1, 1, 1)
        self.doubleSpinBox_3 = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox_3.setFont(font)
        self.doubleSpinBox_3.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox_3.setSuffix(" m")
        self.doubleSpinBox_3.setDecimals(3)
        self.doubleSpinBox_3.setMinimum(-3.0)
        self.doubleSpinBox_3.setMaximum(3.0)
        self.doubleSpinBox_3.setSingleStep(0.005)
        self.doubleSpinBox_3.setObjectName("doubleSpinBox_3")
        self.gridLayout_2.addWidget(self.doubleSpinBox_3, 2, 0, 1, 1)
        self.doubleSpinBox_6 = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox_6.setFont(font)
        self.doubleSpinBox_6.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox_6.setSuffix("")
        self.doubleSpinBox_6.setDecimals(3)
        self.doubleSpinBox_6.setMinimum(-3.0)
        self.doubleSpinBox_6.setMaximum(3.0)
        self.doubleSpinBox_6.setSingleStep(0.005)
        self.doubleSpinBox_6.setObjectName("doubleSpinBox_6")
        self.gridLayout_2.addWidget(self.doubleSpinBox_6, 2, 1, 1, 1)
        self.doubleSpinBox_7 = QtWidgets.QDoubleSpinBox(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        font.setItalic(False)
        self.doubleSpinBox_7.setFont(font)
        self.doubleSpinBox_7.setStyleSheet("color: rgb(255, 140, 0);")
        self.doubleSpinBox_7.setSuffix("")
        self.doubleSpinBox_7.setDecimals(3)
        self.doubleSpinBox_7.setMinimum(-3.0)
        self.doubleSpinBox_7.setMaximum(3.0)
        self.doubleSpinBox_7.setSingleStep(0.005)
        self.doubleSpinBox_7.setObjectName("doubleSpinBox_7")
        self.gridLayout_2.addWidget(self.doubleSpinBox_7, 3, 1, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.tab)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout_2.addWidget(self.pushButton_3, 3, 0, 1, 1)
        self.gridLayout_3.addLayout(self.gridLayout_2, 0, 0, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.tab, clicked= lambda: self.pose()) 
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        self.pushButton.setFont(font)
        self.pushButton.setStyleSheet("color: rgb(255, 140, 0);")
        self.pushButton.setObjectName("pushButton")
        self.gridLayout_3.addWidget(self.pushButton, 1, 0, 1, 1)
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.tab_2)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.gridLayout_5 = QtWidgets.QGridLayout()
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.spinBox_4 = QtWidgets.QSpinBox(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        self.spinBox_4.setFont(font)
        self.spinBox_4.setMinimum(-180)
        self.spinBox_4.setMaximum(180)
        self.spinBox_4.setSingleStep(1)
        self.spinBox_4.setObjectName("spinBox_4")
        self.gridLayout_5.addWidget(self.spinBox_4, 0, 0, 1, 1)
        self.horizontalSlider_6 = QtWidgets.QSlider(self.tab_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_6.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_6.setSizePolicy(sizePolicy)
        self.horizontalSlider_6.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_6.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_6.setMinimum(-180)
        self.horizontalSlider_6.setMaximum(180)
        self.horizontalSlider_6.setPageStep(0)
        self.horizontalSlider_6.setTracking(False)
        self.horizontalSlider_6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_6.setInvertedAppearance(False)
        self.horizontalSlider_6.setInvertedControls(False)
        self.horizontalSlider_6.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_6.setTickInterval(0)
        self.horizontalSlider_6.setObjectName("horizontalSlider_6")
        self.gridLayout_5.addWidget(self.horizontalSlider_6, 2, 1, 1, 1)
        self.horizontalSlider_4 = QtWidgets.QSlider(self.tab_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_4.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_4.setSizePolicy(sizePolicy)
        self.horizontalSlider_4.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_4.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_4.setMinimum(-180)
        self.horizontalSlider_4.setMaximum(180)
        self.horizontalSlider_4.setPageStep(0)
        self.horizontalSlider_4.setTracking(False)
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setInvertedAppearance(False)
        self.horizontalSlider_4.setInvertedControls(False)
        self.horizontalSlider_4.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_4.setTickInterval(0)
        self.horizontalSlider_4.setObjectName("horizontalSlider_4")
        self.gridLayout_5.addWidget(self.horizontalSlider_4, 0, 1, 1, 1)
        self.spinBox_6 = QtWidgets.QSpinBox(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        self.spinBox_6.setFont(font)
        self.spinBox_6.setMinimum(-180)
        self.spinBox_6.setMaximum(180)
        self.spinBox_6.setSingleStep(1)
        self.spinBox_6.setObjectName("spinBox_6")
        self.gridLayout_5.addWidget(self.spinBox_6, 2, 0, 1, 1)
        self.spinBox_5 = QtWidgets.QSpinBox(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        self.spinBox_5.setFont(font)
        self.spinBox_5.setMinimum(-180)
        self.spinBox_5.setMaximum(180)
        self.spinBox_5.setSingleStep(1)
        self.spinBox_5.setProperty("value", -45)
        self.spinBox_5.setObjectName("spinBox_5")
        self.gridLayout_5.addWidget(self.spinBox_5, 1, 0, 1, 1)
        self.horizontalSlider_5 = QtWidgets.QSlider(self.tab_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_5.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_5.setSizePolicy(sizePolicy)
        self.horizontalSlider_5.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_5.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_5.setMinimum(-180)
        self.horizontalSlider_5.setMaximum(180)
        self.horizontalSlider_5.setPageStep(0)
        self.horizontalSlider_5.setProperty("value", -45)
        self.horizontalSlider_5.setTracking(False)
        self.horizontalSlider_5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_5.setInvertedAppearance(False)
        self.horizontalSlider_5.setInvertedControls(False)
        self.horizontalSlider_5.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_5.setTickInterval(0)
        self.horizontalSlider_5.setObjectName("horizontalSlider_5")
        self.gridLayout_5.addWidget(self.horizontalSlider_5, 1, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.gridLayout_5.addWidget(self.label_7, 2, 2, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.gridLayout_5.addWidget(self.label_6, 1, 2, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.gridLayout_5.addWidget(self.label_5, 0, 2, 1, 1)
        self.gridLayout_6.addLayout(self.gridLayout_5, 0, 1, 1, 1)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.spinBox_3 = QtWidgets.QSpinBox(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        self.spinBox_3.setFont(font)
        self.spinBox_3.setMinimum(-180)
        self.spinBox_3.setMaximum(180)
        self.spinBox_3.setSingleStep(1)
        self.spinBox_3.setObjectName("spinBox_3")
        self.gridLayout_4.addWidget(self.spinBox_3, 2, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.gridLayout_4.addWidget(self.label_2, 1, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.gridLayout_4.addWidget(self.label_3, 2, 0, 1, 1)
        self.horizontalSlider_3 = QtWidgets.QSlider(self.tab_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_3.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_3.setSizePolicy(sizePolicy)
        self.horizontalSlider_3.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_3.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_3.setMinimum(-180)
        self.horizontalSlider_3.setMaximum(180)
        self.horizontalSlider_3.setPageStep(0)
        self.horizontalSlider_3.setTracking(False)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setInvertedAppearance(False)
        self.horizontalSlider_3.setInvertedControls(False)
        self.horizontalSlider_3.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_3.setTickInterval(0)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.gridLayout_4.addWidget(self.horizontalSlider_3, 2, 1, 1, 1)
        self.spinBox = QtWidgets.QSpinBox(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        self.spinBox.setFont(font)
        self.spinBox.setPrefix("")
        self.spinBox.setMinimum(-180)
        self.spinBox.setMaximum(180)
        self.spinBox.setSingleStep(1)
        self.spinBox.setObjectName("spinBox")
        self.gridLayout_4.addWidget(self.spinBox, 0, 2, 1, 1)
        self.horizontalSlider = QtWidgets.QSlider(self.tab_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider.sizePolicy().hasHeightForWidth())
        self.horizontalSlider.setSizePolicy(sizePolicy)
        self.horizontalSlider.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider.setMinimum(-180)
        self.horizontalSlider.setMaximum(180)
        self.horizontalSlider.setPageStep(0)
        self.horizontalSlider.setTracking(False)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setInvertedAppearance(False)
        self.horizontalSlider.setInvertedControls(False)
        self.horizontalSlider.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider.setTickInterval(0)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.gridLayout_4.addWidget(self.horizontalSlider, 0, 1, 1, 1)
        self.spinBox_2 = QtWidgets.QSpinBox(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        self.spinBox_2.setFont(font)
        self.spinBox_2.setMinimum(-180)
        self.spinBox_2.setMaximum(180)
        self.spinBox_2.setSingleStep(1)
        self.spinBox_2.setProperty("value", -45)
        self.spinBox_2.setObjectName("spinBox_2")
        self.gridLayout_4.addWidget(self.spinBox_2, 1, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.gridLayout_4.addWidget(self.label, 0, 0, 1, 1)
        self.horizontalSlider_2 = QtWidgets.QSlider(self.tab_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_2.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_2.setSizePolicy(sizePolicy)
        self.horizontalSlider_2.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_2.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_2.setMinimum(-180)
        self.horizontalSlider_2.setMaximum(180)
        self.horizontalSlider_2.setPageStep(0)
        self.horizontalSlider_2.setProperty("value", -45)
        self.horizontalSlider_2.setTracking(False)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setInvertedAppearance(False)
        self.horizontalSlider_2.setInvertedControls(False)
        self.horizontalSlider_2.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_2.setTickInterval(0)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.gridLayout_4.addWidget(self.horizontalSlider_2, 1, 1, 1, 1)
        self.gridLayout_6.addLayout(self.gridLayout_4, 0, 0, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.tab_2, clicked= lambda: self.joint())
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout_6.addWidget(self.pushButton_2, 1, 1, 1, 1)
        self.pushButton_4 = QtWidgets.QPushButton(self.tab_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout_6.addWidget(self.pushButton_4, 1, 0, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_6, 1, 1, 1, 1)
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.tab_3)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.checkBox = QtWidgets.QCheckBox(self.tab_3)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        font.setItalic(False)
        self.checkBox.setFont(font)
        self.checkBox.setChecked(False)
        self.checkBox.setTristate(False)
        self.checkBox.setObjectName("checkBox")
        self.gridLayout_9.addWidget(self.checkBox, 0, 2, 1, 1)
        self.radioButton_2 = QtWidgets.QRadioButton(self.tab_3)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.radioButton_2.setFont(font)
        self.radioButton_2.setStyleSheet("selection-background-color: rgb(255, 179, 0);\n"
"color: rgb(255, 140, 0);")
        self.radioButton_2.setText("Joint")
        self.radioButton_2.setIconSize(QtCore.QSize(100, 100))
        self.radioButton_2.setCheckable(True)
        self.radioButton_2.setObjectName("radioButton_2")
        self.gridLayout_9.addWidget(self.radioButton_2, 0, 1, 1, 1)
        self.radioButton = QtWidgets.QRadioButton(self.tab_3)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.radioButton.setFont(font)
        self.radioButton.setStyleSheet("border-color: rgb(255, 140, 0);\n"
"border-size: 2px;\n"
"color: rgb(255, 140, 0);\n"
"hover:rgb(255, 204, 116);")
        self.radioButton.setText("Linear")
        self.radioButton.setIconSize(QtCore.QSize(100, 100))
        self.radioButton.setCheckable(True)
        self.radioButton.setChecked(True)
        self.radioButton.setObjectName("radioButton")
        self.gridLayout_9.addWidget(self.radioButton, 0, 0, 1, 1)
        self.comboBox = QtWidgets.QComboBox(self.tab_3)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(12)
        self.comboBox.setFont(font)
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.gridLayout_9.addWidget(self.comboBox, 0, 3, 1, 1)
        self.frame = QtWidgets.QFrame(self.tab_3)
        self.frame.setMaximumSize(QtCore.QSize(16777215, 360))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.formLayout_2 = QtWidgets.QFormLayout(self.frame)
        self.formLayout_2.setObjectName("formLayout_2")
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout.setObjectName("gridLayout")
        self.spinBox_10 = QtWidgets.QSpinBox(self.frame_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.spinBox_10.setFont(font)
        self.spinBox_10.setKeyboardTracking(False)
        self.spinBox_10.setSuffix("")
        self.spinBox_10.setMinimum(-100)
        self.spinBox_10.setMaximum(100)
        self.spinBox_10.setObjectName("spinBox_10")
        self.gridLayout.addWidget(self.spinBox_10, 3, 0, 1, 1)
        self.spinBox_7 = QtWidgets.QSpinBox(self.frame_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.spinBox_7.setFont(font)
        self.spinBox_7.setKeyboardTracking(False)
        self.spinBox_7.setSuffix("")
        self.spinBox_7.setMinimum(-100)
        self.spinBox_7.setMaximum(100)
        self.spinBox_7.setObjectName("spinBox_7")
        self.gridLayout.addWidget(self.spinBox_7, 0, 0, 1, 1)
        self.spinBox_8 = QtWidgets.QSpinBox(self.frame_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.spinBox_8.setFont(font)
        self.spinBox_8.setKeyboardTracking(False)
        self.spinBox_8.setSuffix("")
        self.spinBox_8.setMinimum(-100)
        self.spinBox_8.setMaximum(100)
        self.spinBox_8.setObjectName("spinBox_8")
        self.gridLayout.addWidget(self.spinBox_8, 1, 0, 1, 1)
        self.spinBox_9 = QtWidgets.QSpinBox(self.frame_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.spinBox_9.setFont(font)
        self.spinBox_9.setKeyboardTracking(False)
        self.spinBox_9.setSuffix("")
        self.spinBox_9.setMinimum(-100)
        self.spinBox_9.setMaximum(100)
        self.spinBox_9.setObjectName("spinBox_9")
        self.gridLayout.addWidget(self.spinBox_9, 2, 0, 1, 1)
        self.spinBox_11 = QtWidgets.QSpinBox(self.frame_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.spinBox_11.setFont(font)
        self.spinBox_11.setKeyboardTracking(False)
        self.spinBox_11.setSuffix("")
        self.spinBox_11.setMinimum(-100)
        self.spinBox_11.setMaximum(100)
        self.spinBox_11.setObjectName("spinBox_11")
        self.gridLayout.addWidget(self.spinBox_11, 4, 0, 1, 1)
        self.spinBox_12 = QtWidgets.QSpinBox(self.frame_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(14)
        self.spinBox_12.setFont(font)
        self.spinBox_12.setKeyboardTracking(False)
        self.spinBox_12.setSuffix("")
        self.spinBox_12.setMinimum(-100)
        self.spinBox_12.setMaximum(100)
        self.spinBox_12.setObjectName("spinBox_12")
        self.gridLayout.addWidget(self.spinBox_12, 5, 0, 1, 1)
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.frame_2)
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")
        self.label_4 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(16)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_4)
        self.horizontalSlider_7 = QtWidgets.QSlider(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_7.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_7.setSizePolicy(sizePolicy)
        self.horizontalSlider_7.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_7.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_7.setMinimum(-100)
        self.horizontalSlider_7.setMaximum(100)
        self.horizontalSlider_7.setPageStep(0)
        self.horizontalSlider_7.setTracking(False)
        self.horizontalSlider_7.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_7.setInvertedAppearance(False)
        self.horizontalSlider_7.setInvertedControls(False)
        self.horizontalSlider_7.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_7.setTickInterval(0)
        self.horizontalSlider_7.setObjectName("horizontalSlider_7")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_7)
        self.label_10 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(16)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_10)
        self.horizontalSlider_8 = QtWidgets.QSlider(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_8.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_8.setSizePolicy(sizePolicy)
        self.horizontalSlider_8.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_8.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_8.setMinimum(-100)
        self.horizontalSlider_8.setMaximum(100)
        self.horizontalSlider_8.setPageStep(0)
        self.horizontalSlider_8.setTracking(False)
        self.horizontalSlider_8.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_8.setInvertedAppearance(False)
        self.horizontalSlider_8.setInvertedControls(False)
        self.horizontalSlider_8.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_8.setTickInterval(0)
        self.horizontalSlider_8.setObjectName("horizontalSlider_8")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_8)
        self.label_8 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(16)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_8)
        self.horizontalSlider_9 = QtWidgets.QSlider(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_9.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_9.setSizePolicy(sizePolicy)
        self.horizontalSlider_9.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_9.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_9.setMinimum(-100)
        self.horizontalSlider_9.setMaximum(100)
        self.horizontalSlider_9.setPageStep(0)
        self.horizontalSlider_9.setTracking(False)
        self.horizontalSlider_9.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_9.setInvertedAppearance(False)
        self.horizontalSlider_9.setInvertedControls(False)
        self.horizontalSlider_9.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_9.setTickInterval(0)
        self.horizontalSlider_9.setObjectName("horizontalSlider_9")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_9)
        self.label_9 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(16)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_9)
        self.horizontalSlider_10 = QtWidgets.QSlider(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_10.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_10.setSizePolicy(sizePolicy)
        self.horizontalSlider_10.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_10.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_10.setMinimum(-100)
        self.horizontalSlider_10.setMaximum(100)
        self.horizontalSlider_10.setPageStep(0)
        self.horizontalSlider_10.setTracking(False)
        self.horizontalSlider_10.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_10.setInvertedAppearance(False)
        self.horizontalSlider_10.setInvertedControls(False)
        self.horizontalSlider_10.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_10.setTickInterval(0)
        self.horizontalSlider_10.setObjectName("horizontalSlider_10")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_10)
        self.label_11 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(16)
        self.label_11.setFont(font)
        self.label_11.setObjectName("label_11")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.label_11)
        self.horizontalSlider_11 = QtWidgets.QSlider(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_11.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_11.setSizePolicy(sizePolicy)
        self.horizontalSlider_11.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_11.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_11.setMinimum(-100)
        self.horizontalSlider_11.setMaximum(100)
        self.horizontalSlider_11.setPageStep(0)
        self.horizontalSlider_11.setTracking(False)
        self.horizontalSlider_11.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_11.setInvertedAppearance(False)
        self.horizontalSlider_11.setInvertedControls(False)
        self.horizontalSlider_11.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_11.setTickInterval(0)
        self.horizontalSlider_11.setObjectName("horizontalSlider_11")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_11)
        self.label_12 = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setFamily("Roboto Cn")
        font.setPointSize(16)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.label_12)
        self.horizontalSlider_12 = QtWidgets.QSlider(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.horizontalSlider_12.sizePolicy().hasHeightForWidth())
        self.horizontalSlider_12.setSizePolicy(sizePolicy)
        self.horizontalSlider_12.setStyleSheet("QSlider::groove:horizontal { \n"
"    background-color: rgb(255, 140, 0);\n"
"    border: 0px solid #424242; \n"
"    height: 6px; \n"
"    border-radius: 3px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal { \n"
"    background-color: white; \n"
"    width: 16px; \n"
"    height: 20px; \n"
"    line-height: 20px; \n"
"    margin-top: -5px; \n"
"    margin-bottom: -5px; \n"
"    border-radius: 8px; \n"
"}\n"
"QSlider::handle:horizontal:hover { \n"
"    border-radius: 5px;\n"
"    background-color: rgb(255, 204, 116)\n"
"}")
        self.horizontalSlider_12.setInputMethodHints(QtCore.Qt.ImhNone)
        self.horizontalSlider_12.setMinimum(-100)
        self.horizontalSlider_12.setMaximum(100)
        self.horizontalSlider_12.setPageStep(0)
        self.horizontalSlider_12.setTracking(False)
        self.horizontalSlider_12.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_12.setInvertedAppearance(False)
        self.horizontalSlider_12.setInvertedControls(False)
        self.horizontalSlider_12.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.horizontalSlider_12.setTickInterval(0)
        self.horizontalSlider_12.setObjectName("horizontalSlider_12")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.horizontalSlider_12)
        self.formLayout_2.setLayout(0, QtWidgets.QFormLayout.FieldRole, self.formLayout)
        self.gridLayout_9.addWidget(self.frame, 1, 0, 1, 4)
        self.tabWidget.addTab(self.tab_3, "")
        self.gridLayout_8.addWidget(self.tabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        self.horizontalSlider.sliderMoved['int'].connect(self.spinBox.setValue) # type: ignore
        self.spinBox.valueChanged['int'].connect(self.horizontalSlider.setValue) # type: ignore
        self.horizontalSlider_2.sliderMoved['int'].connect(self.spinBox_2.setValue) # type: ignore
        self.spinBox_2.valueChanged['int'].connect(self.horizontalSlider_2.setValue) # type: ignore
        self.horizontalSlider_3.sliderMoved['int'].connect(self.spinBox_3.setValue) # type: ignore
        self.spinBox_3.valueChanged['int'].connect(self.horizontalSlider_3.setValue) # type: ignore
        self.horizontalSlider_4.sliderMoved['int'].connect(self.spinBox_4.setValue) # type: ignore
        self.horizontalSlider_5.sliderMoved['int'].connect(self.spinBox_5.setValue) # type: ignore
        self.horizontalSlider_6.sliderMoved['int'].connect(self.spinBox_6.setValue) # type: ignore
        self.spinBox_6.valueChanged['int'].connect(self.horizontalSlider_6.setValue) # type: ignore
        self.spinBox_5.valueChanged['int'].connect(self.horizontalSlider_5.setValue) # type: ignore
        self.spinBox_4.valueChanged['int'].connect(self.horizontalSlider_4.setValue) # type: ignore
        self.radioButton.clicked.connect(self.label_4.setFocus) # type: ignore
        self.radioButton.clicked.connect(self.label_10.setFocus) # type: ignore
        self.radioButton.clicked.connect(self.label_8.setFocus) # type: ignore
        self.radioButton.clicked.connect(self.label_9.setFocus) # type: ignore
        self.radioButton.clicked.connect(self.label_11.setFocus) # type: ignore
        self.radioButton.clicked.connect(self.label_12.setFocus) # type: ignore
        self.radioButton_2.clicked.connect(self.label_4.setFocus) # type: ignore
        self.radioButton_2.clicked.connect(self.label_10.setFocus) # type: ignore
        self.radioButton_2.clicked.connect(self.label_9.setFocus) # type: ignore
        self.radioButton_2.clicked['bool'].connect(self.label_11.setFocus) # type: ignore
        self.radioButton_2.clicked.connect(self.label_12.setFocus) # type: ignore
        self.horizontalSlider_7.sliderMoved['int'].connect(self.spinBox_7.setValue) # type: ignore
        self.horizontalSlider_8.sliderMoved['int'].connect(self.spinBox_8.setValue) # type: ignore
        self.horizontalSlider_9.sliderMoved['int'].connect(self.spinBox_9.setValue) # type: ignore
        self.horizontalSlider_10.sliderMoved['int'].connect(self.spinBox_10.setValue) # type: ignore
        self.horizontalSlider_11.sliderMoved['int'].connect(self.spinBox_11.setValue) # type: ignore
        self.horizontalSlider_12.sliderMoved['int'].connect(self.spinBox_12.setValue) # type: ignore
        self.pushButton_4.clicked['bool'].connect(lambda: self.spinBox_3.setValue(0)) # type: ignore
        self.pushButton_4.clicked['bool'].connect(lambda: self.spinBox_2.setValue(0)) # type: ignore
        self.pushButton_4.clicked['bool'].connect(lambda: self.spinBox.setValue(0)) # type: ignore
        self.pushButton_4.clicked['bool'].connect(lambda: self.spinBox_4.setValue(0)) # type: ignore
        self.pushButton_4.clicked['bool'].connect(lambda: self.spinBox_5.setValue(0)) # type: ignore
        self.pushButton_4.clicked['bool'].connect(lambda: self.spinBox_6.setValue(0)) # type: ignore
        self.pushButton_3.clicked['bool'].connect(lambda: self.doubleSpinBox_3.setValue(0)) # type: ignore
        self.pushButton_3.clicked['bool'].connect(lambda: self.doubleSpinBox_7.setValue(0)) # type: ignore
        self.pushButton_3.clicked['bool'].connect(lambda: self.doubleSpinBox_2.setValue(0)) # type: ignore
        self.pushButton_3.clicked['bool'].connect(lambda: self.doubleSpinBox.setValue(0)) # type: ignore
        self.pushButton_3.clicked['bool'].connect(lambda: self.doubleSpinBox_4.setValue(0)) # type: ignore
        self.pushButton_3.clicked['bool'].connect(lambda: self.doubleSpinBox_5.setValue(0)) # type: ignore
        self.pushButton_3.clicked.connect(lambda: self.doubleSpinBox_6.setValue(0)) # type: ignore
        self.radioButton_2.toggled['bool'].connect(lambda: self.label_4.setText("L1")) # type: ignore
        self.radioButton_2.toggled['bool'].connect(lambda: self.label_10.setText("L2")) # type: ignore
        self.radioButton_2.toggled['bool'].connect(lambda: self.label_8.setText("L3")) # type: ignore
        self.radioButton_2.toggled['bool'].connect(lambda: self.label_9.setText("L4")) # type: ignore
        self.radioButton_2.toggled['bool'].connect(lambda: self.label_11.setText("L5")) # type: ignore
        self.radioButton_2.toggled['bool'].connect(lambda: self.label_12.setText("L6"))# type: ignore
        self.radioButton.toggled['bool'].connect(lambda: self.label_4.setText("X")) # type: ignore
        self.radioButton.toggled['bool'].connect(lambda: self.label_10.setText("Y")) # type: ignore
        self.radioButton.toggled['bool'].connect(lambda: self.label_8.setText("Z")) # type: ignore
        self.radioButton.toggled['bool'].connect(lambda: self.label_9.setText("Rx")) # type: ignore
        self.radioButton.toggled['bool'].connect(lambda: self.label_11.setText("Ry")) # type: ignore
        self.radioButton.toggled['bool'].connect(lambda: self.label_12.setText("Rz"))# type: ignore
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Control Panel"))
        self.doubleSpinBox.setPrefix(_translate("MainWindow", "x = "))
        self.doubleSpinBox_4.setPrefix(_translate("MainWindow", "x = "))
        self.doubleSpinBox_2.setPrefix(_translate("MainWindow", "y = "))
        self.doubleSpinBox_5.setPrefix(_translate("MainWindow", "y = "))
        self.doubleSpinBox_3.setPrefix(_translate("MainWindow", "z = "))
        self.doubleSpinBox_6.setPrefix(_translate("MainWindow", "z = "))
        self.doubleSpinBox_7.setPrefix(_translate("MainWindow", "w = "))
        self.pushButton_3.setText(_translate("MainWindow", "Reset"))
        self.pushButton.setText(_translate("MainWindow", "Go"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Pose"))
        self.spinBox_4.setSuffix(_translate("MainWindow", "°"))
        self.spinBox_6.setSuffix(_translate("MainWindow", "°"))
        self.spinBox_5.setSuffix(_translate("MainWindow", "°"))
        self.label_7.setText(_translate("MainWindow", "L6"))
        self.label_6.setText(_translate("MainWindow", "L5"))
        self.label_5.setText(_translate("MainWindow", "L4"))
        self.spinBox_3.setSuffix(_translate("MainWindow", "°"))
        self.label_2.setText(_translate("MainWindow", "L2"))
        self.label_3.setText(_translate("MainWindow", "L3"))
        self.spinBox.setSuffix(_translate("MainWindow", "°"))
        self.spinBox_2.setSuffix(_translate("MainWindow", "°"))
        self.label.setText(_translate("MainWindow", "L1"))
        self.pushButton_2.setText(_translate("MainWindow", "Go"))
        self.pushButton_4.setText(_translate("MainWindow", "Reset"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Joint"))
        self.checkBox.setText(_translate("MainWindow", "Servo Mode"))
        self.comboBox.setItemText(0, _translate("MainWindow", "Base Link"))
        self.comboBox.setItemText(1, _translate("MainWindow", "End Effector"))
        self.spinBox_10.setPrefix(_translate("MainWindow", "%"))
        self.spinBox_7.setPrefix(_translate("MainWindow", "%"))
        self.spinBox_8.setPrefix(_translate("MainWindow", "%"))
        self.spinBox_9.setPrefix(_translate("MainWindow", "%"))
        self.spinBox_11.setPrefix(_translate("MainWindow", "%"))
        self.spinBox_12.setPrefix(_translate("MainWindow", "%"))
        self.label_4.setText(_translate("MainWindow", "X"))
        self.label_10.setText(_translate("MainWindow", "Y"))
        self.label_8.setText(_translate("MainWindow", "Z"))
        self.label_9.setText(_translate("MainWindow", "Rx"))
        self.label_11.setText(_translate("MainWindow", "Ry"))
        self.label_12.setText(_translate("MainWindow", "Rz"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Servo"))

        # Set Servo Slider to zero
        self.horizontalSlider_7.sliderReleased.connect(self.slider_zero) # type: ignore
        self.horizontalSlider_8.sliderReleased.connect(self.slider_zero_2) # type: ignore
        self.horizontalSlider_9.sliderReleased.connect(self.slider_zero_3) # type: ignore
        self.horizontalSlider_10.sliderReleased.connect(self.slider_zero_4) # type: ignore
        self.horizontalSlider_11.sliderReleased.connect(self.slider_zero_5) # type: ignore
        self.horizontalSlider_12.sliderReleased.connect(self.slider_zero_6) # type: ignore

        

        self.radioButton.toggled.connect(lambda: self.unlock_joint_frame())
        self.radioButton.toggled.connect(lambda: self.radio_button_change_1())
        self.radioButton_2.toggled.connect(lambda: self.lock_joint_frame())
        self.comboBox.currentTextChanged.connect(self.destroy)
        self.comboBox.currentTextChanged.connect(self.servo_init)
        self.radioButton_2.toggled.connect(lambda: self.radio_button_change_2())
        self.checkBox.stateChanged.connect(lambda: self.servo_on_off()) #connect(lambda: self.servo_init)

    def servo_on_off(self):
        if self.checkBox.isChecked():
            if self.radioButton.isChecked():
                self.servo_init()
            else:
                try:
                    self.destroy()
                    self.servoj_init()
                except:
                   self.servoj_init() 
        else:
            self.destroy()
    # Locks ComboBox when ServoJ enabled
    def lock_joint_frame(self):
        self.comboBox.setCurrentIndex(0)
        self.comboBox.setDisabled(1)
    # Unlocks ComboBox when ServoJ disabled
    def unlock_joint_frame(self):
        self.comboBox.setEnabled(1)
    # Changes publisher to Servo
    def radio_button_change_1(self):
        if self.checkBox.isChecked():
            self.destroy()
            self.servo_init()
        else:
            self.servo_init()
    # Changes publisher to ServoJ
    def radio_button_change_2(self):
        if self.checkBox.isChecked():
            self.destroy()
            self.servoj_init() 

    def go_pose(self):
        try:
            self.destroy()
            self.pose()
        except:
            self.pose()  
    # Sets slider back to zero
    def slider_zero(self):
        #if self.horizontalSlider_7.value() != 0:
        self.horizontalSlider_7.setValue(0)
        self.spinBox_7.setValue(0)
    def slider_zero_2(self):
        self.horizontalSlider_8.setValue(0)
        self.spinBox_8.setValue(0)
    def slider_zero_3(self):
        self.horizontalSlider_9.setValue(0)
        self.spinBox_9.setValue(0)
    def slider_zero_4(self):
        self.horizontalSlider_10.setValue(0)
        self.spinBox_10.setValue(0)
    def slider_zero_5(self):
        self.horizontalSlider_11.setValue(0)
        self.spinBox_11.setValue(0)
    def slider_zero_6(self):
        self.horizontalSlider_12.setValue(0)
        self.spinBox_12.setValue(0)

    # Kills node
    def destroy(self):
        try:
            self.node.destroy_node()
        except:
            print("Failed")

    def pose(self):
        try:
            node = Node("ex_pose_goal")
            callback_group = ReentrantCallbackGroup()
            moveit2 = MoveIt2(
                node=node,
                joint_names=ur3.joint_names(),
                base_link_name=ur3.base_link_name(),
                end_effector_name=ur3.end_effector_name(),
                group_name=ur3.MOVE_GROUP_ARM,
                callback_group=callback_group,
            )

            self.executor.add_node(node)

            position = [self.doubleSpinBox.value(), self.doubleSpinBox_2.value(), self.doubleSpinBox_3.value()] 
            quat_xyzw = [self.doubleSpinBox_4.value(), self.doubleSpinBox_5.value(), self.doubleSpinBox_6.value(), self.doubleSpinBox_7.value()]

            node.get_logger().info(
                "Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            )
            moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw)
            moveit2.wait_until_executed()
            node.destroy_node()
        except:
            print("Failed to go Pose")    

    def joint(self):

        node = Node("ex_joint_goal")
        node.declare_parameter(
            "joint_positions",
            [
            self.spinBox.value()*22/1260,
            self.spinBox_2.value()*22/1260,
            self.spinBox_3.value()*22/1260,
            self.spinBox_4.value()*22/1260,
            self.spinBox_5.value()*22/1260,
            self.spinBox_6.value()*22/1260,
            ],
        )

        callback_group = ReentrantCallbackGroup()

        moveit2 = MoveIt2(
            node=node,
            joint_names=ur3.joint_names(),
            base_link_name=ur3.base_link_name(),
            end_effector_name=ur3.end_effector_name(),
            group_name=ur3.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        self.executor.add_node(node)

        joint_positions = (
            node.get_parameter("joint_positions").get_parameter_value().double_array_value
        )
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")

        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()
        node.destroy_node()
        #self.destroy()
    
    def servoj_init(self):
        self.node = Node("ex_servoj")
        callback_group = ReentrantCallbackGroup()

        self.moveit2_servoj = MoveIt2ServoJoints(
            node=self.node,
            frame_id= ur3.base_link_name(),
            joint_names=ur3.joint_names(),  #["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"],
            callback_group=callback_group,
        )
         
        self.executor.add_node(self.node)
        self.node.create_timer(0.01,self.servoj)
        
    def servoj(self):

        self.moveit2_servoj.servoJ(
            velocities=(self.spinBox_7.value()/100, self.spinBox_8.value()/100, self.spinBox_9.value()/100,
            self.spinBox_10.value()/100, self.spinBox_11.value()/100, self.spinBox_12.value()/100)) 
        print(float(self.spinBox_7.value()))
        
    def servo_init(self):
        if self.checkBox.isChecked(): 
            self.node = Node("ex_servo")
            callback_group = ReentrantCallbackGroup()

            if self.comboBox.currentIndex() == 0:
                self.moveit2_servo = MoveIt2Servo(
                  node=self.node,
                  frame_id=ur3.base_link_name(),
                  callback_group=callback_group,
                  linear_speed=0.01,
                  angular_speed=0.01,
                )
            elif self.comboBox.currentIndex() == 1:
                self.moveit2_servo = MoveIt2Servo(
                  node=self.node,
                  frame_id=ur3.end_effector_name(),
                  callback_group=callback_group,
                  linear_speed=0.01,
                  angular_speed=0.01,
                )         
            self.executor.add_node(self.node)
            self.node.create_timer(0.01,self.servo)
        else:
            print("ERROR")
        
    def servo(self):

        self.moveit2_servo.servo(
            linear=(self.spinBox_7.value(), self.spinBox_8.value(), self.spinBox_9.value()), 
            angular=(self.spinBox_10.value(), self.spinBox_11.value(), self.spinBox_12.value()))
        print(float(self.spinBox_7.value()))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    #ui.servo_init()
    #ui.servoj_init()
    sys.exit(app.exec_())

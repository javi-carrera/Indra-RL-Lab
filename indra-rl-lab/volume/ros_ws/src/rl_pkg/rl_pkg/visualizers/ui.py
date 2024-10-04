from PySide6 import QtCore, QtWidgets
from PySide6.QtWidgets import QMainWindow
import pyqtgraph as pg


class Ui_MainWindow(object):
    def setupUi(self, MainWindow: QMainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)  # Set an initial size for the window

        # Create a central widget and set it for the MainWindow
        self.central_widget = QtWidgets.QWidget(MainWindow)
        MainWindow.setCentralWidget(self.central_widget)

        # Create a vertical layout for the central widget
        self.verticalLayout = QtWidgets.QVBoxLayout(self.central_widget)

        # Create a horizontal layout for the buttons
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(-1, -1, -1, 4)
        self.horizontalLayout.setSpacing(4)

        # Add the Start button
        self.startButton = QtWidgets.QPushButton("Start!")
        self.startButton.setStyleSheet('font: 700 11pt "Segoe UI";')
        self.horizontalLayout.addWidget(self.startButton)

        # Add the Stop button
        self.stopButton = QtWidgets.QPushButton("Stop")
        self.stopButton.setEnabled(False)
        self.stopButton.setStyleSheet('font: 700 11pt "Segoe UI";')
        self.horizontalLayout.addWidget(self.stopButton)

        # Add the horizontal layout (buttons) to the main vertical layout
        self.verticalLayout.addLayout(self.horizontalLayout)

        # Create the pyqtgraph GraphicsLayoutWidget for plotting
        self.win = pg.GraphicsLayoutWidget(parent=self.central_widget)

        # Add the pyqtgraph widget to the vertical layout (it will expand to fill the remaining space)
        self.verticalLayout.addWidget(self.win)

        self.retranslateUi(MainWindow)

    def retranslateUi(self, MainWindow: QMainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
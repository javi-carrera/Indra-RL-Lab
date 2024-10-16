import PySide6
import pyqtgraph as pg
import time
import threading
import queue

from PySide6 import QtCore
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer, QThread

from rl_pkg.visualizers.ui import Ui_MainWindow
from rl_pkg.visualizers.controller import PlotController

import numpy as np

class FrameCounter(QtCore.QObject):
    sigFpsUpdate = QtCore.Signal(object)

    def __init__(self, interval=1000):
        super().__init__()
        self.count = 0
        self.last_update = 0
        self.interval = interval

    def update(self):
        self.count += 1

        if self.last_update == 0:
            self.last_update = time.perf_counter()
            self.startTimer(self.interval)

    def timerEvent(self, evt):
        now = time.perf_counter()
        elapsed = now - self.last_update
        fps = self.count / elapsed
        self.last_update = now
        self.count = 0
        self.sigFpsUpdate.emit(fps)

class MainWindow(QMainWindow):
    def __init__(
        self,
        data_queue: queue.Queue,
        *args,
        **kwargs,
    ):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.resize(720, 480)

        self.plotController = PlotController(data_queue)
        for plt in self.plotController.plots:
            self.ui.win.addItem(plt)
            # plt.getViewBox().setXRange(0, 100)
            # plt.getViewBox().enableAutoRange(x=False, y=True)

        self.framecnt = FrameCounter()
        self.framecnt.sigFpsUpdate.connect(lambda fps: self.statusBar().showMessage(f"FrameRate: {fps:.1f} fps"))

        self.ui.startButton.clicked.connect(self.startFakeData)
        self.ui.stopButton.clicked.connect(self.stopFakeData)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.setInterval(30)

        self.initPlots()

    def startFakeData(self):
        self.clear_and_init_plots()
        self.ui.startButton.setEnabled(False)
        self.ui.stopButton.setEnabled(True)
        self.timer.start()

    def stopFakeData(self):
        self.ui.startButton.setEnabled(True)
        self.ui.stopButton.setEnabled(False)
        self.timer.stop()

    def update_plots(self):
        self.plotController.update()
        self.framecnt.update()

    def initPlots(self):
        self.plotController.init()

    def clearPlots(self):
        self.plotController.clear()

    def clear_and_init_plots(self):
        self.clearPlots()
        self.initPlots()

    def closeEvent(self, event):
        self.timer.stop()
        return super().closeEvent(event)

def plot_thread(data_queue):
    app = QApplication([])
    window = MainWindow(data_queue)
    window.setWindowTitle("Real-Time Env Plot")
    window.show()
    app.exec()

class MainApp:
    def __init__(self):
        # Set up the multiprocessing Queue to send data to the plot process
        self.data_queue = queue.Queue()

        # Set up the plot process
        self.plot_thread  = threading.Thread(target=plot_thread, args=(self.data_queue,))

    def start_plotting(self):
        """Start the plot process."""
        self.plot_thread.start()

    def stop_plotting(self):
        """Stop the plotting process."""
        self.plot_thread.join()

    def send_data_to_plot(self, observation, reward, dones):
        """Send data to the plot process through the Queue."""
        self.data_queue.put((observation, reward, dones))

if __name__ == "__main__":
    app = MainApp()
    app.start_plotting()

    try:
        while True:
            # Simulate training data (e.g., observations and rewards)
            observation = [
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1),
            ]
            reward = np.random.uniform(-1, 1)

            # Send the data to the plotting process via the Queue
            app.send_data_to_plot(observation, reward)
            time.sleep(0.02)
    except KeyboardInterrupt:
        app.stop_plotting()
        print("Plotting stopped.")
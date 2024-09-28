# Project: Playground
# File: vector.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from typing import List

import matplotlib.pyplot as plt
import numpy as np

from geometry_msgs.msg import Vector3


class VectorVisualizer:

    def __init__(self):

        # Initialize matplotlib
        plt.style.use('seaborn-darkgrid')       # Use print(plt.style.available)) to see available styles
        plt.ion()

        # Initialize the plot (line plot with 3 lines)
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1)

        self.xs = np.array([[]])
        self.ys = np.array([[]])
        self.zs = np.array([[]])


    def visualize(self, vectors: List[Vector3]):

        # Clear the plot
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        # Extract the data from the vectors
        x = [vector.x for vector in vectors]
        y = [vector.y for vector in vectors]
        z = [vector.z for vector in vectors]

        # Append the data to the arrays
        self.xs = np.append(self.xs, np.array([x]), axis=1)
        self.ys = np.append(self.ys, np.array([y]), axis=1)
        self.zs = np.append(self.zs, np.array([z]), axis=1)


        # Create a line plot
        self.ax1.plot(self.xs[0], label='x')
        self.ax2.plot(self.ys[0], label='y')
        self.ax3.plot(self.zs[0], label='z')
        
        # # Set the y-axis ticks and labels
        # self.ax.set_yticks(range(len(trigger_sensors)))
        # self.ax.set_yticklabels([f"Sensor {i+1}" for i in range(len(trigger_sensors))])

        # # Set the x-axis limits and labels
        # self.ax.set_xlim(0, 100)
        # self.ax.set_xlabel("Progress (%)")

        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Display the plot
        # plt.show()
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan

class LidarSensorVisualizer:

    def __init__(self):

        # Initialize matplotlib
        plt.style.use('seaborn-darkgrid')       # Use print(plt.style.available)) to see available styles
        plt.ion()

        # Initialize the plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.max_lidar_line, = self.ax.plot([], [], linestyle=':', marker='.')
        self.min_lidar_line, = self.ax.plot([], [], linestyle=':', marker='.')
        
        # Add text display for LIDAR data
        self.text = self.fig.text(
            0.05,
            0.2,
            '',
            transform=self.fig.transFigure,
            ha='left',
            va='top',
            color='white',
            fontsize=10,
            fontdict={
                'family': 'monospace',
            },
            bbox={
                'facecolor': 'grey',
                'edgecolor': 'none',
                'alpha': 0.5,
            }
        )

    def visualize(self, lidar_scan: LaserScan):

        # Extract LIDAR data from the message
        angle_min = lidar_scan.angle_min
        angle_max = lidar_scan.angle_max
        angle_increment = lidar_scan.angle_increment
        range_min = lidar_scan.range_min
        range_max = lidar_scan.range_max
        ranges = np.array(lidar_scan.ranges)

        # Update plot limits based on new range_max
        self.ax.set_ylim(0.0, range_max * 1.1)
        
        # Calculate angles for the ranges
        num_ranges = len(ranges)
        angles = np.linspace(angle_min, angle_max, num_ranges)

        # Convert angles to radians for plotting
        angles = np.deg2rad(angles)

        # Update the data for the max line plot
        self.max_lidar_line.set_xdata(angles)
        self.max_lidar_line.set_ydata(ranges)

        # Update the data for the min line plot
        self.min_lidar_line.set_xdata(angles)
        self.min_lidar_line.set_ydata(range_min * np.ones(num_ranges))

        # Update text display with current LIDAR data
        lidar_data_text = \
            f'Min angle [deg]: {angle_min:.2f}\n'\
            f'Max angle [deg]: {angle_max:.2f}\n'\
            f'Increment [deg]: {angle_increment:.2f}\n'\
            f'Min range [m]  : {range_min:.2f}\n'\
            f'Max range [m]  : {range_max:.2f}'
        
        self.text.set_text(lidar_data_text)

        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

# Project: Playground
# File: smart_lidar_sensor_visualizer.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from interfaces_pkg.msg import SmartLidarSensor


tag_colors = {
    -1: '#000000',  # Unknown
    0: '#FF0000',   # Untagged
    1: '#00FF00',   # Green
    2: '#0000FF',   # Blue
}

class SmartLidarSensorVisualizer:

    def __init__(self):

        # Initialize matplotlib
        plt.style.use('seaborn-darkgrid')       # Use print(plt.style.available)) to see available styles
        plt.ion()

        # Initialize the plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.max_lidar_line, = self.ax.plot([], [], linestyle=':', marker='', color='grey')
        self.min_lidar_line, = self.ax.plot([], [], linestyle=':', marker='', color='grey')

        scatter_labels = sorted(tag_colors.keys())
        scatter_colors = [tag_colors[tag] for tag in scatter_labels]
        scatter_norm = matplotlib.colors.Normalize(vmin=scatter_labels[0], vmax=scatter_labels[-1])
        scatter_color_map = matplotlib.colors.ListedColormap(scatter_colors)
        self.tag_scatter = self.ax.scatter([], [], marker='o', s=10, c=[], cmap=scatter_color_map, norm=scatter_norm)
        
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

    def visualize(self, smart_lidar_scan: SmartLidarSensor):

        # Extract LIDAR data from the message
        angle_min = smart_lidar_scan.angle_min
        angle_max = smart_lidar_scan.angle_max
        angle_increment = smart_lidar_scan.angle_increment
        range_min = smart_lidar_scan.range_min
        range_max = smart_lidar_scan.range_max
        ranges = np.array(smart_lidar_scan.ranges)
        tags = np.array(smart_lidar_scan.tags)

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

        # Update the data for the tag scatter plot
        self.tag_scatter.set_offsets(np.column_stack([angles, ranges]))
        self.tag_scatter.set_array(tags)
        


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

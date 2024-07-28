from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from interfaces_pkg.srv import LidarSensorTestEnvironmentStep

import numpy as np
import matplotlib.pyplot as plt


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

    def visualize(
            self,
            ranges: np.ndarray,
            angle_min: float,
            angle_max: float,
            angle_increment: float,
            range_min: float,
            range_max: float
        ):

        # Update plot limits based on new range_max
        self.ax.set_ylim(0.0, range_max)
        
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


class LidarSensorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='lidar_sensor_test_environment',
            service_msg_type=LidarSensorTestEnvironmentStep,
        )

        # Initialize the visualizer
        self.lidar_visualizer = LidarSensorVisualizer()

        # Initialize the response
        #self._response = LidarSensorTestEnvironmentStep.Response()


    def convert_action_to_request(self, action: np.ndarray) -> LidarSensorTestEnvironmentStep.Request:
        
        #request = self._service_msg_type.Request()

        # Convert the action to ROS request format
        self._request.agent_action.lidar_sensor_request = True

        return self._request
    

    def convert_response_to_state(self, response) -> np.ndarray:

        # Store the response
        self._response = response

        # Convert the response to numpy array
        state = np.array(
            response.agent_state.laser_scan.ranges
        )

        return state
    

    def render(self):
        
        # Visualize the lidar data
        self.lidar_visualizer.visualize(
            self._response.agent_state.laser_scan.ranges,
            self._response.agent_state.laser_scan.angle_min,
            self._response.agent_state.laser_scan.angle_max,
            self._response.agent_state.laser_scan.angle_increment,
            self._response.agent_state.laser_scan.range_min,
            self._response.agent_state.laser_scan.range_max
        )


    def obersvation(self, state: np.ndarray) -> np.ndarray:
        return state
    
    def reward(self, state: np.ndarray, action: np.ndarray = None) -> float:
        return 0.0
    
    def terminated(self, state: np.ndarray) -> bool:
        return False
    
    def truncated(self, state: np.ndarray) -> bool:
        return False

    def info(self, state: np.ndarray) -> dict:
        return {}


def main():

    rclpy.init()

    env = LidarSensorTestEnvironment()
    action = np.array([])

    while True:
        state, reward, terminated, truncated, info = env.step(action)
        env.render()

    env.close()


if __name__ == '__main__':
    main()

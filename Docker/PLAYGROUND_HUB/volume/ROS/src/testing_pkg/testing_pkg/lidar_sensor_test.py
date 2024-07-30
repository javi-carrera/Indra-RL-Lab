from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.lidar_sensor_visualizer import LidarSensorVisualizer
from interfaces_pkg.srv import LidarSensorTestEnvironmentStep

import numpy as np
import matplotlib.pyplot as plt


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


    def convert_action_to_request(self, action: np.ndarray) -> LidarSensorTestEnvironmentStep.Request:

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
        self.lidar_visualizer.visualize(self._response.agent_state.laser_scan)


    def observation(self, state: np.ndarray) -> np.ndarray:
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

from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.lidar_sensor_visualizer import LidarSensorVisualizer
from interfaces_pkg.srv import LidarSensorTestEnvironmentAction, LidarSensorTestEnvironmentState, LidarSensorTestEnvironmentReset

import numpy as np
import matplotlib.pyplot as plt


class LidarSensorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='lidar_sensor_test_environment',
            action_service_msg_type=LidarSensorTestEnvironmentAction,
            state_service_msg_type=LidarSensorTestEnvironmentState,
            reset_service_msg_type=LidarSensorTestEnvironmentReset,
            sample_time=0.0
        )

        # Initialize the visualizer
        self.lidar_visualizer = LidarSensorVisualizer()


    def convert_action_to_request(self, action: np.ndarray) -> LidarSensorTestEnvironmentAction.Request:
        return self.action_request
    

    def convert_response_to_state(self, response) -> dict:

        # Convert the response to dict
        state = {
            'laser_scan': np.array(response.state.laser_scan)
        }

        return state
    
    def convert_reset_to_request(self) -> Type:
        return self.reset_request
    

    def render(self):
        
        # Visualize the lidar data
        self.lidar_visualizer.visualize(self.state_response.state.laser_scan)


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

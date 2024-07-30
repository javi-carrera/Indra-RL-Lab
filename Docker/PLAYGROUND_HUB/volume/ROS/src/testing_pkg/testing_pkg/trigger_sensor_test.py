from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.trigger_sensor_visualizer import TriggerSensorVisualizer
from interfaces_pkg.srv import TriggerSensorTestEnvironmentStep

import numpy as np
import matplotlib.pyplot as plt


class TriggerSensorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='trigger_sensor_test_environment',
            service_msg_type=TriggerSensorTestEnvironmentStep,
        )

        # Initialize the visualizer
        self._visualizer = TriggerSensorVisualizer()


    def convert_action_to_request(self, action: np.ndarray) -> TriggerSensorTestEnvironmentStep.Request:

        # Convert the action to ROS request format
        self._request.agent_action.trigger_sensor_request = True

        return self._request
    

    def convert_response_to_state(self, response) -> np.ndarray:

        # Store the response
        self._response = response

        # Convert the response to numpy array
        state = np.array([
            float(response.agent_state.trigger_sensor_01_data.has_triggered),
            float(response.agent_state.trigger_sensor_02_data.has_triggered),
            float(response.agent_state.trigger_sensor_03_data.has_triggered),
            float(response.agent_state.trigger_sensor_04_data.has_triggered),

            float(response.agent_state.trigger_sensor_01_data.has_timer_finished),
            float(response.agent_state.trigger_sensor_02_data.has_timer_finished),
            float(response.agent_state.trigger_sensor_03_data.has_timer_finished),
            float(response.agent_state.trigger_sensor_04_data.has_timer_finished),

            response.agent_state.trigger_sensor_01_data.timer_count,
            response.agent_state.trigger_sensor_02_data.timer_count,
            response.agent_state.trigger_sensor_03_data.timer_count,
            response.agent_state.trigger_sensor_04_data.timer_count,

            response.agent_state.trigger_sensor_01_data.max_timer_count,
            response.agent_state.trigger_sensor_02_data.max_timer_count,
            response.agent_state.trigger_sensor_03_data.max_timer_count,
            response.agent_state.trigger_sensor_04_data.max_timer_count,
        ])

        return state
    

    def render(self):
            
        # Visualize the trigger sensor data
        self._visualizer.visualize([
            self._response.agent_state.trigger_sensor_01_data,
            self._response.agent_state.trigger_sensor_02_data,
            self._response.agent_state.trigger_sensor_03_data,
            self._response.agent_state.trigger_sensor_04_data,
        ])


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

    env = TriggerSensorTestEnvironment()
    action = np.array([])

    while True:
        state, reward, terminated, truncated, info = env.step(action)
        env.render()

    env.close()


if __name__ == '__main__':
    main()

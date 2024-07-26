from typing import Tuple, Type

import gymnasium as gym
import numpy as np

from .single_agent_environment_node import SingleAgentEnvironmentNode
from interfaces_pkg.srv import EnvironmentStep

class Environment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='environment',
            service_msg_type=EnvironmentStep,
        )


    def convert_action_to_request(self, action: np.ndarray) -> EnvironmentStep.Request:
        
        request = self._service_msg_type.Request()

        # Convert the action to ROS request format
        request.agent_action.target_position.x = action[0]
        request.agent_action.target_position.y = action[1]
        request.agent_action.target_position.z = action[2]

        return request
    

    def convert_response_to_state(self, response) -> np.ndarray:

        # Convert the response to numpy array
        state = np.array([
            response.agent_state.position.x,
            response.agent_state.position.y,
            response.agent_state.position.z,
            response.agent_state.velocity.x,
            response.agent_state.velocity.y,
            response.agent_state.velocity.z
        ])

        return state


    def obersvation(self, state: np.ndarray) -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    

    def reward(self, state: np.ndarray, action: np.ndarray = None) -> float:
        return 0.0
    

    def terminated(self, state: np.ndarray) -> bool:
        # Return True with a probability of 0.1
        return np.random.rand() < 0.1
    
    
    def truncated(self, state: np.ndarray) -> bool:
        return False
    

    def info(self, state: np.ndarray) -> dict:
        return {}
    


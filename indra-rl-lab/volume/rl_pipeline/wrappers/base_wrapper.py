# Project: Indra-RL-Lab
# File: base_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple, Union
import numpy as np
from rl_pkg.environment_node import EnvironmentNode

class BaseWrapper(Wrapper):

    def __init__(self, env: Union[EnvironmentNode, 'BaseWrapper']):
        Wrapper.__init__(self, env)
        self.env: Union[EnvironmentNode, BaseWrapper]

    def send_reset_request(self) -> Type:

        self.unwrapped.reset_request.reset = True
        self.unwrapped.reset_response = self.unwrapped._send_service_request('reset')
        state = self.convert_response_to_state(self.unwrapped.reset_response)

        return state
    
    def send_step_request(self, action: np.ndarray) -> Type:

        self.unwrapped.step_request = self.convert_action_to_request(action)
        self.unwrapped.step_response = self.unwrapped._send_service_request('step')
        state = self.convert_response_to_state(self.unwrapped.step_response)

        return state

    def reset(self, **kwargs) -> Tuple[np.ndarray, dict]:

        state = self.unwrapped.send_reset_request()

        self.reset_environment_variables()
        observation = self.observation(state)
        info = self.info(state)

        self.unwrapped.n_step = 0

        return observation, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:

        state = self.unwrapped.send_step_request(action)

        observation = self.observation(state)
        reward = self.reward(state)
        terminated = self.terminated(state)
        truncated = self.truncated(state)
        info = self.info(state)

        self.unwrapped.n_step += 1

        return observation, reward, terminated, truncated, info
    
    def reset_environment_variables(self):
        return self.env.reset_environment_variables()
    
    def convert_action_to_request(self, action: np.ndarray) -> Type:
        raise self.env.convert_action_to_request(action)
    
    def convert_response_to_state(self, response: Type) -> Tuple[Type, Type]:
        raise self.env.convert_response_to_state(response)
    
    def observation(self, state: Type) -> np.ndarray:
        return self.env.observation(state)
    
    def reward(self, state: Type) -> np.ndarray:
        return self.env.reward(state)
    
    def terminated(self, state: Type) -> bool:
        return self.env.terminated(state)
    
    def truncated(self, state: Type) -> bool:
        return self.env.truncated(state)
    
    def info(self, state: Type) -> dict:
        return self.env.info(state)
    
    @property
    def unwrapped(self) -> EnvironmentNode:
        return self.env.unwrapped
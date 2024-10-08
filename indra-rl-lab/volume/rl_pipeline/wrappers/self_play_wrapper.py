# Project: Indra-RL-Lab
# File: self_play_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple, Union
import numpy as np
from rl_pkg.environment_node import EnvironmentNode
from rl_pipeline.wrappers import BaseWrapper

class SelfPlayWrapper(Wrapper):

    def __init__(self, env: Union[EnvironmentNode, BaseWrapper]):
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

        state = self.send_reset_request()

        self.reset_environment_variables()
        observation = self.observation(state)
        info = self.env.info(state)

        self.unwrapped.n_step = 0

        return observation, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:

        state = self.send_step_request(action)

        observation = self.observation(state)
        reward = self.env.reward(state)
        terminated = self.env.terminated(state)
        truncated = self.env.truncated(state)
        info = self.env.info(state)

        self.unwrapped.n_step += 1

        return observation, reward, terminated, truncated, info
    
    def reset_environment_variables(self):
        return self.env.reset_environment_variables()
    
    def convert_action_to_request(self, action: np.ndarray) -> Type:
        raise NotImplementedError
    
    def convert_response_to_state(self, response: Type) -> Tuple[Type, Type]:
        raise NotImplementedError
    
    def observation(self, state: Type) -> np.ndarray:
        raise NotImplementedError
    
    @property
    def unwrapped(self) -> EnvironmentNode:
        return self.env.unwrapped
import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple, Union
import numpy as np
from rl_pkg.environment_node import EnvironmentNode

class BaseWrapper(Wrapper):

    def __init__(self, env: gym.Env):
        Wrapper.__init__(self, env)
        self.env: Union[EnvironmentNode, BaseWrapper]

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
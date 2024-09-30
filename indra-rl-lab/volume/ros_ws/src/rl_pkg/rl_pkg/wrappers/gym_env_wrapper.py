# Project: Playground
# File: gym_env_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from typing import Tuple

import gymnasium as gym
import numpy as np


class GymEnvWrapper(gym.Env):

    def __init__(
            self,
            env,
            observation_space: gym.Space,
            action_space: gym.Space,
            reward_range: Tuple[float, float]
        ):

        self.env = env
        self.observation_space = observation_space
        self.action_space = action_space
        self.reward_range = reward_range

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:

        observation, reward, terminated, truncated, info = self.env.step(action)
        observation = observation.astype(np.float32)
        reward = float(reward)

        return observation, reward, terminated, truncated, info

    def reset(self, **kwargs) -> Tuple[np.ndarray, dict]:

        observation, info = self.env.reset()
        observation = observation.astype(np.float32)

        return observation, info
    
    def render(self):
        return self.env.render()

    def close(self):
        return self.env.close()
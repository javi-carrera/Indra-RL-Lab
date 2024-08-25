import gymnasium as gym
import numpy as np
from gymnasium.spaces import Space
from typing import Tuple
from stable_baselines3.common.env_checker import check_env

class GymEnvWrapper(gym.Env):

    def __init__(
            self,
            env,
            observation_space: Space,
            action_space: Space,
            reward_range: Tuple[float, float]
        ):

        # Environment initialization
        self.env = env

        self.observation_space = observation_space
        self.action_space = action_space
        self.reward_range = reward_range
    

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:

        # Take a step in the environment
        observation, reward, terminated, truncated, info = self.env.step(action)

        # Cast the observation from float64 to float32
        observation = observation.astype(np.float32)

        # Cast the reward from float64 to float32
        reward = float(reward)

        return observation, reward, terminated, truncated, info
    

    def reset(self, **kwargs) -> Tuple[np.ndarray, dict]:

        # Reset the environment
        observation, info = self.env.reset()

        # Cast the observation from float64 to float32
        observation = observation.astype(np.float32)

        return observation, info
    

    def render(self):
        return self.env.render()
    

    def close(self):
        return self.env.close()
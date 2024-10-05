import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple
import numpy as np
from use_cases.uc1 import UC1Environment
from rl_pipeline.wrappers import BaseWrapper
from interfaces_pkg.msg import UC1AgentState

class UC2RewardWrapper(BaseWrapper):

    def __init__(self, env: gym.Env):
        BaseWrapper.__init__(self, env)
    
    @property
    def unwrapped(self) -> UC1Environment:
        return self.env.unwrapped
    
    def reward(self, state: UC1AgentState) -> float:

        reward = 0.0

        # Health reward
        if self.unwrapped.previous_health_normalized is not None:
            health_reward =  10.0 * (self.unwrapped.current_health_normalized - self.unwrapped.previous_health_normalized)
        else:
            health_reward = 0.0

        self.unwrapped.previous_health_normalized = self.unwrapped.current_health_normalized

        # Distance reward
        if self.unwrapped.previous_target_distance is not None:
            distance_reward = 10.0 * (self.unwrapped.previous_target_distance - self.unwrapped.current_target_distance)
        else:
            distance_reward = 0.0

        self.unwrapped.previous_target_distance = self.unwrapped.current_target_distance

        # Total reward
        reward = health_reward + distance_reward
        
        return reward
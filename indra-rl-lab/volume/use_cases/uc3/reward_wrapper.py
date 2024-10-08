import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple
import numpy as np
from use_cases.uc3 import UC3Environment
from rl_pipeline.wrappers import BaseWrapper
from interfaces_pkg.msg import UC3AgentState

class UC3RewardWrapper(BaseWrapper):

    def __init__(self, env: gym.Env):
        BaseWrapper.__init__(self, env)

        self.unwrapped.reward_range = (-5.0, 5.0)

        self.REWARD_DISTANCE_THRESHOLD = 10.0
    
    @property
    def unwrapped(self) -> UC3Environment:
        return self.env.unwrapped
    
        
    
    def reward(self, state: UC3AgentState) -> float:

        # Health reward
        if self.unwrapped.previous_health_normalized is not None:
            health_reward =  10.0 * (self.unwrapped.current_health_normalized - self.unwrapped.previous_health_normalized)
        else:
            health_reward = 0.0

        self.unwrapped.previous_health_normalized = self.unwrapped.current_health_normalized

        # Target health reward
        if self.unwrapped.previous_target_health_normalized is not None:
            target_health_reward = -10.0 * (self.unwrapped.current_target_health_normalized - self.unwrapped.previous_target_health_normalized)
        else:
            target_health_reward = 0.0

        self.unwrapped.previous_target_health_normalized = self.unwrapped.current_target_health_normalized

        # Distance reward
        if self.unwrapped.previous_target_distance is not None:
            if self.unwrapped.current_target_distance > self.REWARD_DISTANCE_THRESHOLD:
                distance_reward = 1.0 * (self.unwrapped.previous_target_distance - self.unwrapped.current_target_distance)
            else:
                distance_reward = 0.0
        else:
            distance_reward = 0.0

        self.unwrapped.previous_target_distance = self.unwrapped.current_target_distance

        # Has fired reward
        has_fired_reward = -0.1 if state.tank.turret_sensor.has_fired else 0.0

        # Has died reward and has target died reward
        has_died_reward = -5.0 if state.tank.health_info.health <= 0.0 else 0.0
        has_target_died_reward = 5.0 if state.target_tank.health_info.health <= 0.0 else 0.0

        return health_reward + target_health_reward + distance_reward + has_fired_reward + has_died_reward + has_target_died_reward
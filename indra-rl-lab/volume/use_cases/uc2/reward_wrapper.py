# Project: Indra-RL-Lab
# File: reward_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple
import numpy as np
from use_cases.uc2 import UC2Environment
from rl_pipeline.wrappers import BaseWrapper
from interfaces_pkg.msg import UC2AgentState

class UC2RewardWrapper(BaseWrapper):

    def __init__(self, env: gym.Env):
        BaseWrapper.__init__(self, env)

        self.unwrapped.reward_range = (-5.0, 5.0)

        self.MAX_REWARD_DISTANCE = 35.0
        self.MIN_REWARD_DISTANCE = 25.0
    
    @property
    def unwrapped(self) -> UC2Environment:
        return self.env.unwrapped

    def sigmoid_distance_reward(
        self,
        x: float,
        k: float,
        d1: float,
        d2: float,
        p1: float,
        p2: float,
    ) -> float:

        """ Sigmoid distance reward function.

        Args:
            x (float): Current distance
            k (float): Scaling factor
            d1 (float): Distance 1
            d2 (float): Distance 2
            p1 (float): Percentage 1 of k at distance d1
            p2 (float): Percentage 2 of k at distance d2

        Returns:
            float: Distance reward.
        """

        a = np.log((p1 * (1 - p2)) / (p2 * (1 - p1))) / (d1 - d2)
        b = np.log((1 - p1) / p1) / a + d1

        return k / (1 + np.exp(-a * (x - b)))
    
    def reward(self, state: UC2AgentState) -> float:

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
        distance_reward = self.sigmoid_distance_reward(
            x=self.unwrapped.current_target_distance,
            k=-0.1,
            d1=self.MAX_REWARD_DISTANCE,
            d2=self.MIN_REWARD_DISTANCE,
            p1=0.9,
            p2=0.1
        )

        # Has fired reward
        has_fired_reward = -0.1 if state.tank.turret_sensor.has_fired else 0.0

        # Has died reward and has target died reward
        has_died_reward = -5.0 if state.tank.health_info.health <= 0.0 else 0.0
        has_target_died_reward = 5.0 if state.target_tank.health_info.health <= 0.0 else 0.0

        # Alive penalty
        alive_penalty = -0.05

        # return health_reward + target_health_reward + distance_reward + has_fired_reward + has_died_reward + has_target_died_reward
        return health_reward + target_health_reward + has_fired_reward + has_died_reward + has_target_died_reward + alive_penalty
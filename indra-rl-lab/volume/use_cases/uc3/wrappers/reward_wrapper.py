# Project: Indra-RL-Lab
# File: reward_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

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

        self.unwrapped.reward_range = (-1.0, 1.0)

        self.DISTANCE_REWARD_INNER_THRESHOLD = 20.0
        self.DISTANCE_REWARD_OUTER_THRESHOLD = 40.0
        self.DISTANCE_REWARD_INNER_PERCENTAGE = 0.001
        self.DISTANCE_REWARD_OUTER_PERCENTAGE = 0.999
        self.DISTANCE_REWARD_MULTIPLIER = -0.05

        self._DISTANCE_REWARD_ALPHA, self._DISTANCE_REWARD_BETA = self._get_sigmoid_distance_parameters(
            self.DISTANCE_REWARD_INNER_THRESHOLD,
            self.DISTANCE_REWARD_OUTER_THRESHOLD,
            self.DISTANCE_REWARD_INNER_PERCENTAGE,
            self.DISTANCE_REWARD_OUTER_PERCENTAGE,
        )

        self.previous_health_normalized = None
        self.previous_target_health_normalized = None

    
    @property
    def unwrapped(self) -> UC3Environment:
        return self.env.unwrapped
    
    def reset_environment_variables(self):
        self.previous_health_normalized = None
        self.previous_target_health_normalized = None
        self.env.reset_environment_variables()

    def reward(self, state: UC3AgentState) -> float:

        # Health reward
        current_health_normalized = state.tank.health_info.health / state.tank.health_info.max_health
        if self.previous_health_normalized is not None:
            health_reward =  1.0 * (current_health_normalized - self.previous_health_normalized)
        else:
            health_reward = 0.0

        self.previous_health_normalized = current_health_normalized

        # Target health reward
        current_target_health_normalized = state.target_tank.health_info.health / state.target_tank.health_info.max_health
        if self.previous_target_health_normalized is not None:
            target_health_reward = -0.5 * (current_target_health_normalized - self.previous_target_health_normalized)
        else:
            target_health_reward = 0.0

        self.previous_target_health_normalized = current_target_health_normalized

        # Distance reward
        distance_reward = self.sigmoid_distance_reward(self.unwrapped.current_target_distance)

        # Has fired reward
        has_fired_reward = -0.05 if state.tank.turret_sensor.has_fired else 0.0

        # Total reward
        if state.tank.health_info.health <= 0.0:
            total_reward = -1.0
        elif state.target_tank.health_info.health <= 0.0:
            total_reward = 1.0
        else:
            total_reward = health_reward + target_health_reward + distance_reward + has_fired_reward

        return total_reward
    
    def _get_sigmoid_distance_parameters(
        self,
        d1: float,
        d2: float,
        p1: float,
        p2: float,
    ) -> Tuple[float, float]:
        
        """ Get sigmoid distance parameters.

        Args:
            d1 (float): Inner distance
            d2 (float): Outer distance
            p1 (float): Percentage 1 of k at inner distance d1
            p2 (float): Percentage 2 of k at outer distance d2

        Returns:
            Tuple[float, float]: Alpha and beta parameters
        """
        
        alpha = np.log((p2 * (1 - p1)) / (p1 * (1 - p2))) / (d2 - d1)
        beta = np.log((1 - p2) / p2) / alpha + d2

        return alpha, beta

    def sigmoid_distance_reward(
        self,
        current_distance: float,
    ) -> float:

        """ Sigmoid distance reward function.

        Args:
            current_distance (float): Current distance

        Returns:
            float: Distance reward.
        """

        # a = np.log((p1 * (1 - p2)) / (p2 * (1 - p1))) / (d1 - d2)
        # b = np.log((1 - p1) / p1) / a + d1
        # return k / (1 + np.exp(-a * (x - b)))

        return self.DISTANCE_REWARD_MULTIPLIER / (1 + np.exp(-self._DISTANCE_REWARD_ALPHA * (current_distance - self._DISTANCE_REWARD_BETA)))
import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple
import numpy as np
from use_cases.uc1 import UC1Environment
from rl_pipeline.wrappers import BaseWrapper
from interfaces_pkg.msg import UC1AgentState

class UC1FitnessWrapper(BaseWrapper):

    def __init__(self, env: gym.Env):
        BaseWrapper.__init__(self, env)

        self.max_speed = self.get_max_possible_speed()  # TODO Define this
        self.w_P = 0.45
        self.w_S = 0.45
        self.w_H = 0.1
        self.time_elapsed = 0
        self.total_distance_towards_goal = 0.0
        self.initial_target_distance = None
        self._current_target_distance = None
    
    @property
    def unwrapped(self) -> UC1Environment:
        return self.env.unwrapped
    
    def info(self, state: Type) -> dict:
        return super().info(state)

    def compute_time_score(self):
        # Normalize time taken
        time_ratio = self.time_elapsed / self.max_time
        # Normalize initial distance
        distance_ratio = self.initial_target_distance / self.max_initial_distance
        # Adjust time score
        time_score = 1 - (time_ratio * distance_ratio)
        # Ensure time score is between 0 and 1
        time_score = max(0, min(time_score, 1))
        return time_score

    def compute_fitness(self, state):
        """
        Fitness function for UC1, KPIs:
        1. Progress Score: proportion of distance to the goal covered ((init_dist - final_dist) / init_dist)
        2. Speed Score: how fast the goal is reached (Average_speed / Maximum_estimate_speed)
        3. Health Score: agent's remaining health

        We use defined weights w_P, w_S, w_H
        """
        # Progress Score
        P = (self.initial_target_distance - self._current_target_distance) / self.initial_target_distance
        P = max(0, min(P, 1))

        # Average Speed Towards Goal
        if steps_taken > 0 and self.total_distance_towards_goal > 0:
            V_avg = self.total_distance_towards_goal / steps_taken
            S = V_avg / self.max_speed
            S = max(0, min(S, 1))
        else:
            S = 0.0

        H = self._current_health_normalized # has to be normalized between 0 and 1

        # Combined Fitness Score
        fitness = 100 * (self.w_P * P + self.w_S * S + self.w_H * H)
        fitness = max(0, min(fitness, 100))

        self._infos['fitness'] = fitness

        return fitness

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:

        state = self.unwrapped.send_step_request(action)

        previous_distance = self._current_target_distance

        observation, reward, terminated, truncated, info = super().step(action)

        self._current_target_distance = self.get_distance_to_goal()

        distance_covered = previous_distance - self._current_target_distance

        done = terminated or truncated
        if done > 0:
            self.total_distance_towards_goal += distance_covered

        done = self.time_elapsed >= self.max_time

        # Compute fitness at the end of the episode
        if done:
            fitness = self.compute_fitness()
            self._infos['fitness'] = fitness
        
        return observation, reward, terminated, truncated, info
    
    def reset(self, **kwargs) -> Tuple[np.ndarray, dict]:
        self.time_elapsed = 0
        self.total_distance_towards_goal = 0.0
        self._current_target_distance = self.get_distance_to_goal()
        self.initial_target_distance = self._current_target_distance

        return super().reset(**kwargs)

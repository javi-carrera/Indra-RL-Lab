# Project: Indra-RL-Lab
# File: observation_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import gymnasium as gym
import numpy as np
from use_cases.uc1 import UC1Environment
from rl_pipeline.wrappers import BaseWrapper

from scipy.spatial.transform import Rotation
from interfaces_pkg.msg import UC1AgentState

class UC1ObservationWrapper(BaseWrapper):

    def __init__(self, env: gym.Env):
        BaseWrapper.__init__(self, env)

        self.unwrapped.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(25,),
            dtype=np.float32
        )

        self.DISTANCE_THRESHOLD = 10.0

        
    @property
    def unwrapped(self) -> UC1Environment:
        return self.env.unwrapped
    
    def observation(self, state: UC1AgentState) -> np.ndarray:

        # Target relative position normalized
        target_relative_position = np.array([state.target_pose.x - state.tank.pose.x, state.target_pose.y - state.tank.pose.y, 0.0])
        yaw = state.tank.pose.theta
        rotation = Rotation.from_euler("z", yaw, degrees=True)
        target_relative_position = rotation.inv().apply(target_relative_position)
        target_relative_position = target_relative_position[:2]

        self.unwrapped.current_target_distance = np.linalg.norm(target_relative_position)
        
        target_relative_position_normalized = (target_relative_position / self.DISTANCE_THRESHOLD if self.unwrapped.current_target_distance < self.DISTANCE_THRESHOLD
                                               else target_relative_position / self.unwrapped.current_target_distance)

        # Linear and angular velocities normalized
        linear_velocity_normalized = (state.tank.twist.y - self.unwrapped.MIN_LINEAR_VELOCITY) / (self.unwrapped.MAX_LINEAR_VELOCITY - self.unwrapped.MIN_LINEAR_VELOCITY) * 2 - 1
        angular_velocity_normalized = state.tank.twist.theta / self.unwrapped.MAX_YAW_RATE

        # Lidar ranges normalized
        ranges = np.array(state.tank.smart_laser_scan.ranges)
        lidar_ranges_normalized = (ranges - state.tank.smart_laser_scan.range_min) / (state.tank.smart_laser_scan.range_max - state.tank.smart_laser_scan.range_min)

        # Health normalized
        self.unwrapped.current_health_normalized = state.tank.health_info.health / state.tank.health_info.max_health

        # Observation
        observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [angular_velocity_normalized],
            lidar_ranges_normalized,
            [self.unwrapped.current_health_normalized],
        ])

        return observation

    

import gymnasium as gym
from gymnasium.core import Wrapper
from typing import Type, Tuple
import numpy as np
from use_cases.uc2 import UC2Environment
from rl_pipeline.wrappers import BaseWrapper

from scipy.spatial.transform import Rotation
from interfaces_pkg.msg import UC2AgentState

class UC2ObservationWrapper(BaseWrapper):

    def __init__(self, env: gym.Env):
        BaseWrapper.__init__(self, env)

    @property
    def unwrapped(self) -> UC2Environment:
        return self.env.unwrapped
    
    def observation(self, state: UC2AgentState) -> np.ndarray:
        
        # Target relative position normalized
        target_relative_position = np.array([state.target_pose.x - state.tank.pose.x, state.target_pose.y - state.tank.pose.y, 0.0])
        yaw = state.tank.pose.theta
        rotation = Rotation.from_euler("z", yaw, degrees=True)
        target_relative_position = rotation.inv().apply(target_relative_position)
        target_relative_position = target_relative_position[:2]

        self.unwrapped.current_target_distance = np.linalg.norm(target_relative_position)
        
        target_relative_position_normalized = (target_relative_position / self.unwrapped.DISTANCE_THRESHOLD if self.unwrapped.current_target_distance < self.unwrapped.DISTANCE_THRESHOLD 
                                               else target_relative_position / self.unwrapped.current_target_distance)

        # Linear and angular velocities normalized
        linear_velocity_normalized = (state.tank.twist.y - self.unwrapped.MIN_LINEAR_VELOCITY) / (self.unwrapped.MAX_LINEAR_VELOCITY - self.unwrapped.MIN_LINEAR_VELOCITY) * 2 - 1
        angular_velocity_normalized = state.tank.twist.theta / self.unwrapped.MAX_YAW_RATE

        # Lidar ranges normalized
        ranges = np.array(state.tank.smart_laser_scan.ranges)
        lidar_ranges_normalized = (ranges - state.tank.smart_laser_scan.range_min) / (state.tank.smart_laser_scan.range_max - state.tank.smart_laser_scan.range_min)

        # Health and target health normalized
        self.unwrapped.current_health_normalized = state.tank.health_info.health / state.tank.health_info.max_health
        self.unwrapped.current_target_health_normalized = state.target_health_info.health / state.target_health_info.max_health

        # Turret
        turret_angle_sin = np.sin(np.deg2rad(state.tank.turret_sensor.current_angle))
        turret_angle_cos = np.cos(np.deg2rad(state.tank.turret_sensor.current_angle))
        turret_cooldown_normalized = state.tank.turret_sensor.cooldown * state.tank.turret_sensor.fire_rate
        turret_has_fired = 1.0 if state.tank.turret_sensor.has_fired else 0.0

        # Current observation
        current_observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [angular_velocity_normalized],
            lidar_ranges_normalized,
            [self.unwrapped.current_health_normalized],
            [self.unwrapped.current_target_health_normalized],
            [turret_angle_sin, turret_angle_cos],
            [turret_cooldown_normalized],
            [turret_has_fired]
        ])

        self.unwrapped.observation_buffer.insert(0, current_observation)
        if len(self.unwrapped.observation_buffer) > self.unwrapped.max_past_index + 1:
            self.unwrapped.observation_buffer.pop()

        # Assemble past observations
        assembled_observations = []
        for idx in self.unwrapped.past_observations:
            if idx < len(self.unwrapped.observation_buffer):
                obs = self.unwrapped.observation_buffer[idx]
            else:
                obs = np.zeros_like(current_observation)
            assembled_observations.append(obs)

        combined_observation = np.concatenate(assembled_observations)

        return combined_observation

    

# Project: Indra-RL-Lab
# File: self_play_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import gymnasium as gym
from typing import Type, Tuple
import numpy as np
from use_cases.uc3 import UC3Environment
from rl_pipeline.wrappers import BaseWrapper
from pathlib import Path
import yaml
from rl_pipeline.algorithm_registry import ALGORITHMS

from scipy.spatial.transform import Rotation
from interfaces_pkg.msg import UC3AgentState
from interfaces_pkg.srv import UC3EnvironmentStep, UC3EnvironmentReset

class UC3SelfPlayWrapper(BaseWrapper):

    def __init__(self, env: UC3Environment):
        BaseWrapper.__init__(self, env)

        self.env: UC3Environment

        self.past_observations = [0, 2, 4, 6]
        self.max_past_index = max(self.past_observations)
        obs_dim = 31 * len(self.past_observations)

        self.unwrapped.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(obs_dim,),
            dtype=np.float32
        )

        self.unwrapped.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0, shape=(4,),
            dtype=np.float32
        )

        self.DISTANCE_THRESHOLD = 25.0

        self.observation_buffer = []
        self.pretrained_model_observation_buffer = []

        # Pretrained model
        self.pretrained_model = None
        # self.update_pretrained_model('ppo_2024-10-10_19-57-08', 'best_model')


    def update_pretrained_model(self, log_dir: str, checkpoint: str):

        log_dir = Path(log_dir)
        pretrained_model_path = log_dir / 'checkpoints' / checkpoint
        experiment_training_config = yaml.safe_load(open(log_dir / 'config' / 'training_config.yml', 'r'))

        self.pretrained_model = ALGORITHMS[experiment_training_config['algorithm']].load(pretrained_model_path)

    def reset_environment_variables(self):
        self.observation_buffer = []
        self.pretrained_model_observation_buffer = []
        self.env.reset_environment_variables()
    
    def convert_action_to_request(self, action: np.ndarray) -> Type:

        # Movement
        linear_velocity = (action[0] + 1.0) * (self.unwrapped.MAX_LINEAR_VELOCITY - self.unwrapped.MIN_LINEAR_VELOCITY) / 2.0 + self.unwrapped.MIN_LINEAR_VELOCITY
        yaw_rate = action[1] * self.unwrapped.MAX_YAW_RATE

        # Turret
        turret_rotation_speed = self.unwrapped.MAX_TURRET_ROTATION_SPEED * action[2]
        fire = bool(action[3] > 0.0)

        self.unwrapped.step_request.action.tank.target_twist.y = linear_velocity
        self.unwrapped.step_request.action.tank.target_twist.theta = yaw_rate
        self.unwrapped.step_request.action.tank.turret_actuator.rotation_speed = turret_rotation_speed
        self.unwrapped.step_request.action.tank.turret_actuator.fire = fire

        # Pretrained model
        if self.pretrained_model is not None:
            try:
                
                pretrained_model_observation = self.observation(self.unwrapped.step_response.target_state, is_pretrained=True)
                pretrained_model_action, _ = self.pretrained_model.predict(pretrained_model_observation, deterministic=True)

                pretrained_model_linear_velocity = (pretrained_model_action[0] + 1.0) * (self.unwrapped.MAX_LINEAR_VELOCITY - self.unwrapped.MIN_LINEAR_VELOCITY) / 2.0 + self.unwrapped.MIN_LINEAR_VELOCITY
                pretrained_model_yaw_rate = pretrained_model_action[1] * self.unwrapped.MAX_YAW_RATE
                pretrained_model_turret_rotation_speed = self.unwrapped.MAX_TURRET_ROTATION_SPEED * pretrained_model_action[2]
                pretrained_model_fire = bool(pretrained_model_action[3] > 0.0)

                self.unwrapped.step_request.target_action.tank.target_twist.y = pretrained_model_linear_velocity
                self.unwrapped.step_request.target_action.tank.target_twist.theta = pretrained_model_yaw_rate
                self.unwrapped.step_request.target_action.tank.turret_actuator.rotation_speed = pretrained_model_turret_rotation_speed
                self.unwrapped.step_request.target_action.tank.turret_actuator.fire = pretrained_model_fire

                # self.unwrapped.step_request.target_action.tank.target_twist.y = 0.0
                # self.unwrapped.step_request.target_action.tank.target_twist.theta = 0.0
                # self.unwrapped.step_request.target_action.tank.turret_actuator.rotation_speed = 0.0
                # self.unwrapped.step_request.target_action.tank.turret_actuator.fire = False

            except ZeroDivisionError:

                self.unwrapped.step_request.target_action.tank.target_twist.y = 0.0
                self.unwrapped.step_request.target_action.tank.target_twist.theta = 0.0
                self.unwrapped.step_request.target_action.tank.turret_actuator.rotation_speed = 0.0
                self.unwrapped.step_request.target_action.tank.turret_actuator.fire = False

                self.unwrapped.logger.error("Zero division error in pretrained model")

        else:
            self.unwrapped.step_request.target_action.tank.target_twist.y = 0.0
            self.unwrapped.step_request.target_action.tank.target_twist.theta = 0.0
            self.unwrapped.step_request.target_action.tank.turret_actuator.rotation_speed = 0.0
            self.unwrapped.step_request.target_action.tank.turret_actuator.fire = False


        return self.unwrapped.step_request
    
    def convert_response_to_state(self, response: UC3EnvironmentStep.Response) -> Tuple[UC3AgentState, UC3AgentState]:
        return response.state
    
    def observation(self, state: UC3AgentState, is_pretrained: bool = False) -> np.ndarray:
        
        # Target relative position normalized
        target_relative_position = np.array([state.target_tank.pose.x - state.tank.pose.x, state.target_tank.pose.y - state.tank.pose.y, 0.0])
        yaw = state.tank.pose.theta
        rotation = Rotation.from_euler("z", yaw, degrees=True)
        target_relative_position = rotation.inv().apply(target_relative_position)
        target_relative_position = target_relative_position[:2]

        current_target_distance = np.linalg.norm(target_relative_position)
        
        target_relative_position_normalized = (target_relative_position / self.DISTANCE_THRESHOLD if current_target_distance < self.DISTANCE_THRESHOLD 
                                               else target_relative_position / current_target_distance)

        # Linear and angular velocities normalized
        linear_velocity_normalized = (state.tank.twist.y - self.unwrapped.MIN_LINEAR_VELOCITY) / (self.unwrapped.MAX_LINEAR_VELOCITY - self.unwrapped.MIN_LINEAR_VELOCITY) * 2 - 1
        angular_velocity_normalized = state.tank.twist.theta / self.unwrapped.MAX_YAW_RATE

        # Lidar ranges normalized
        ranges = np.array(state.tank.smart_laser_scan.ranges)
        lidar_ranges_normalized = ((ranges - state.tank.smart_laser_scan.range_min) / (state.tank.smart_laser_scan.range_max - state.tank.smart_laser_scan.range_min)) * 2 - 1

        # Health and target health normalized
        current_health_normalized = (state.tank.health_info.health / state.tank.health_info.max_health) * 2 - 1
        current_target_health_normalized = (state.target_tank.health_info.health / state.target_tank.health_info.max_health) * 2 - 1

        # Turret
        turret_angle_sin = np.sin(np.deg2rad(state.tank.turret_sensor.current_angle))
        turret_angle_cos = np.cos(np.deg2rad(state.tank.turret_sensor.current_angle))
        laser_range_normalized = (state.tank.turret_sensor.laser_range / state.tank.turret_sensor.max_laser_range) * 2 - 1
        turret_cooldown_normalized = state.tank.turret_sensor.cooldown * state.tank.turret_sensor.fire_rate
        turret_has_fired = 1.0 if state.tank.turret_sensor.has_fired else -1.0

        # Current observation
        current_observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [angular_velocity_normalized],
            lidar_ranges_normalized,
            [current_health_normalized],
            [current_target_health_normalized],
            [turret_angle_sin, turret_angle_cos],
            [laser_range_normalized],
            [turret_cooldown_normalized],
            [turret_has_fired]
        ])

        if is_pretrained:
            return self.combine_observations(current_observation, self.pretrained_model_observation_buffer)

        else:
            self.unwrapped.current_target_distance = current_target_distance
            return self.combine_observations(current_observation, self.observation_buffer)
        

    def combine_observations(
        self,
        current_observation: np.ndarray,
        buffer: list,
    ) -> np.ndarray:
        
        buffer.insert(0, current_observation)
        if len(buffer) > self.max_past_index + 1:
            buffer.pop()

        assembled_observations = []
        for idx in self.past_observations:
            if idx < len(buffer):
                obs = buffer[idx]
            else:
                obs = np.zeros_like(current_observation)
            assembled_observations.append(obs)

        return np.concatenate(assembled_observations)

    @property
    def unwrapped(self) -> UC3Environment:
        return self.env.unwrapped
    

# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import cv2

import gymnasium as gym
import numpy as np
import yaml

from typing import List, Tuple
from pathlib import Path

from scipy.spatial.transform import Rotation

from interfaces_pkg.msg import UC3AgentState
from interfaces_pkg.srv import UC3EnvironmentStep, UC3EnvironmentReset
from rl_pkg import EnvironmentNode
from rl_pipeline.algorithm_registry import ALGORITHMS, get_algorithm_kwargs


class UC3Environment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # EnvironmentNode (ROS)
        EnvironmentNode.__init__(
            self,
            environment_name="uc3_environment",
            environment_id=environment_id,
            step_service_msg_type=UC3EnvironmentStep,
            reset_service_msg_type=UC3EnvironmentReset,
        )
        
        # Gymasium
        self.observation_space = None
        self.reward_range = None
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0, shape=(4,),
            dtype=np.float32
        )

        # Environment parameters
        self.MIN_LINEAR_VELOCITY = -5.0
        self.MAX_LINEAR_VELOCITY = 5.0
        self.MAX_YAW_RATE = 5.0
        self.MAX_TURRET_ROTATION_SPEED = 5.5
        self.DISTANCE_THRESHOLD = 20.0
        self.REWARD_DISTANCE_THRESHOLD = 10.0
        self.MAX_EPISODE_STEPS = 1024

        self.current_target_distance = None
        self.previous_target_distance = None
        self.current_health_normalized = None
        self.previous_health_normalized = None
        self.current_target_health_normalized = None
        self.previous_target_health_normalized = None
        self.observation_buffer = []

        self.pretrained_model_past_observations = [0, 1, 2, 3, 10, 20]
        self.pretrained_model_max_past_index = max(self.pretrained_model_past_observations)

        # Pretrained model
        experiment_name = 'ppo_2024-10-05_09-24-04'
        checkpoint = 'best_model'

        experiments_path = Path('experiments')
        log_dir = experiments_path / f"uc3/{experiment_name}"
        pretrained_model_path = log_dir / 'checkpoints' / checkpoint
        experiment_training_config = yaml.safe_load(open(log_dir / 'config' / 'training_config.yml', 'r'))

        self.pretrained_model = ALGORITHMS[experiment_training_config['algorithm']].load(pretrained_model_path)


    def reset_environment_variables(self):
        
        self.previous_target_distance = None
        self.previous_health_normalized = None
        self.previous_target_health_normalized = None
        self.observation_buffer = []
        self.pretrained_model_observation_buffer = []

    def convert_action_to_request(self, action: np.ndarray = None):

        self.step_request: UC3EnvironmentStep.Request
        self.step_response: UC3EnvironmentStep.Response

        # Movement
        linear_velocity = (action[0] + 1.0) * (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) / 2.0 + self.MIN_LINEAR_VELOCITY
        yaw_rate = action[1] * self.MAX_YAW_RATE

        # Turret
        turret_rotation_speed = self.MAX_TURRET_ROTATION_SPEED * action[2]
        fire = bool(action[3] > 0.5)

        self.step_request.action.tank.target_twist.y = linear_velocity
        self.step_request.action.tank.target_twist.theta = yaw_rate
        self.step_request.action.tank.turret_actuator.rotation_speed = turret_rotation_speed
        self.step_request.action.tank.turret_actuator.fire = fire

        # Pretrained model
        try:
            
            pretrained_model_observation = self.pretrained_model_observation(self.step_response.target_state)
            pretrained_model_action, _ = self.pretrained_model.predict(pretrained_model_observation, deterministic=True)

            pretrained_model_linear_velocity = (pretrained_model_action[0] + 1.0) * (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) / 2.0 + self.MIN_LINEAR_VELOCITY
            pretrained_model_yaw_rate = pretrained_model_action[1] * self.MAX_YAW_RATE
            pretrained_model_turret_rotation_speed = self.MAX_TURRET_ROTATION_SPEED * pretrained_model_action[2]
            pretrained_model_fire = bool(pretrained_model_action[3] > 0.5)

            self.step_request.target_action.tank.target_twist.y = pretrained_model_linear_velocity
            self.step_request.target_action.tank.target_twist.theta = pretrained_model_yaw_rate
            self.step_request.target_action.tank.turret_actuator.rotation_speed = pretrained_model_turret_rotation_speed
            self.step_request.target_action.tank.turret_actuator.fire = pretrained_model_fire

        except ZeroDivisionError:

            self.step_request.target_action.tank.target_twist.y = 0.0
            self.step_request.target_action.tank.target_twist.theta = 0.0
            self.step_request.target_action.tank.turret_actuator.rotation_speed = 0.0
            self.step_request.target_action.tank.turret_actuator.fire = False

            self.logger.info("Zero division error in pretrained model observation")

        return self.step_request

    def convert_response_to_state(self, response: UC3EnvironmentStep.Response) -> UC3AgentState:
        return response.state

    def terminated(self, state: UC3AgentState) -> bool:

        has_died = state.tank.health_info.health <= 0.0
        has_target_died = state.target_tank.health_info.health <= 0.0

        return has_died or has_target_died

    def truncated(self, state: UC3AgentState) -> bool:
        return self.n_step > self.MAX_EPISODE_STEPS

    def info(self, state: UC3AgentState) -> dict:
        return {}
    
    def pretrained_model_observation(self, state: UC3AgentState) -> np.ndarray:
            
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
        linear_velocity_normalized = (state.tank.twist.y - self.MIN_LINEAR_VELOCITY) / (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) * 2 - 1
        angular_velocity_normalized = state.tank.twist.theta / self.MAX_YAW_RATE

        # Lidar ranges normalized
        ranges = np.array(state.tank.smart_laser_scan.ranges)
        lidar_ranges_normalized = (ranges - state.tank.smart_laser_scan.range_min) / (state.tank.smart_laser_scan.range_max - state.tank.smart_laser_scan.range_min)

        # Health and target health normalized
        current_health_normalized = state.tank.health_info.health / state.tank.health_info.max_health
        current_target_health_normalized = state.target_tank.health_info.health / state.target_tank.health_info.max_health

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
            [current_health_normalized],
            [current_target_health_normalized],
            [turret_angle_sin, turret_angle_cos],
            [turret_cooldown_normalized],
            [turret_has_fired]
        ])

        # Assembled past observations
        self.pretrained_model_observation_buffer.insert(0, current_observation)
        if len(self.pretrained_model_observation_buffer) > self.pretrained_model_max_past_index + 1:
            self.pretrained_model_observation_buffer.pop()

        assembled_observations = []
        for idx in self.pretrained_model_past_observations:
            if idx < len(self.pretrained_model_observation_buffer):
                obs = self.pretrained_model_observation_buffer[idx]
            else:
                obs = np.zeros_like(current_observation)
            assembled_observations.append(obs)

        combined_observation = np.concatenate(assembled_observations)

        return combined_observation

# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import cv2

import gymnasium as gym
import numpy as np

from scipy.spatial.transform import Rotation

from interfaces_pkg.msg import UC3AgentState
from interfaces_pkg.srv import UC3EnvironmentStep, UC3EnvironmentReset
from rl_pkg.environment_node import EnvironmentNode


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
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(32,),
            dtype=np.float32
        )

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0, shape=(3,),
            dtype=np.float32
        )

        self.reward_range = (-1.0, 1.0)

        # Environment parameters
        self.MIN_LINEAR_VELOCITY = -5.0
        self.MAX_LINEAR_VELOCITY = 5.0
        self.MAX_YAW_RATE = 5.0
        self.MAX_TURRET_ROTATION_SPEED = 100.0
        self.MAX_EPISODE_STEPS = 1024

        self._current_target_distance = None
        self._previous_target_distance = None
        self._current_health_normalized = None
        self._previous_health_normalized = None
        self._current_target_health_normalized = None
        self._previous_target_health_normalized = None


    def convert_action_to_request(self, action: np.ndarray = None):

        self.step_request: UC3EnvironmentStep.Request

        # Movement
        linear_velocity = (action[0] + 1.0) * (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) / 2.0 + self.MIN_LINEAR_VELOCITY
        yaw_rate = action[1] * self.MAX_YAW_RATE

        # Turret
        turret_rotation_speed = self.MAX_TURRET_ROTATION_SPEED * action[2]
        # fire = bool(action[3] > 0.5)
        fire = True

        self.step_request.action.tank.target_twist.y = linear_velocity
        self.step_request.action.tank.target_twist.theta = yaw_rate
        self.step_request.action.tank.turret_actuator.target_angle = turret_rotation_speed
        self.step_request.action.tank.turret_actuator.fire = fire

        return self.step_request

    def convert_response_to_state(self, response: UC3EnvironmentStep.Response) -> UC3AgentState:
        return response.state

    def reset(self):
        self._previous_target_distance = None
        self._previous_health_normalized = None
        self._previous_target_health_normalized = None

        return super().reset()

    def observation(self, state: UC3AgentState) -> np.ndarray:

        # Target relative position normalized
        target_relative_position = np.array([state.enemy_tank.pose.x - state.tank.pose.x, state.enemy_tank.pose.y - state.tank.pose.y, 0.0])
        yaw = state.tank.pose.theta
        rotation = Rotation.from_euler("z", yaw, degrees=True)
        target_relative_position = rotation.inv().apply(target_relative_position)
        target_relative_position = target_relative_position[:2]

        self._current_target_distance = np.linalg.norm(target_relative_position)
        distance_threshold = 10.0
        target_relative_position_normalized = (target_relative_position / distance_threshold if self._current_target_distance < distance_threshold else target_relative_position / self._current_target_distance)

        # Linear and angular velocities normalized
        linear_velocity_normalized = (state.tank.twist.y - self.MIN_LINEAR_VELOCITY) / (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) * 2 - 1
        angular_velocity_normalized = state.tank.twist.theta / self.MAX_YAW_RATE

        # Target linear velocity normalized
        target_linear_velocity_normalized_x = (state.enemy_tank.twist.x - self.MIN_LINEAR_VELOCITY) / (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) * 2 - 1
        target_linear_velocity_normalized_y = (state.enemy_tank.twist.y - self.MIN_LINEAR_VELOCITY) / (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) * 2 - 1

        # Lidar ranges normalized
        ranges = np.array(state.tank.smart_laser_scan.ranges)
        lidar_ranges_normalized = (ranges - state.tank.smart_laser_scan.range_min) / (state.tank.smart_laser_scan.range_max - state.tank.smart_laser_scan.range_min)

        # Health and target health normalized
        self._current_health_normalized = state.tank.health_info.health / state.tank.health_info.max_health
        self._current_target_health_normalized = state.enemy_tank.health_info.health / state.enemy_tank.health_info.max_health

        # Turret
        # turret_angle_normalized = (state.tank.turret_sensor.current_angle / 360.0) * 2.0 - 1.0
        turret_angle_sin = np.sin(np.deg2rad(state.tank.turret_sensor.current_angle))
        turret_angle_cos = np.cos(np.deg2rad(state.tank.turret_sensor.current_angle))
        turret_cooldown_normalized = state.tank.turret_sensor.cooldown * state.tank.turret_sensor.fire_rate
        turret_has_fired = 1.0 if state.tank.turret_sensor.has_fired else 0.0

        # Observation
        observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [target_linear_velocity_normalized_x, target_linear_velocity_normalized_y],
            [angular_velocity_normalized],
            lidar_ranges_normalized,
            [self._current_health_normalized],
            [self._current_target_health_normalized],
            [turret_angle_sin, turret_angle_cos],
            [turret_cooldown_normalized],
            [turret_has_fired]
        ])

        return observation

    def reward(self, state: UC3AgentState, action: np.ndarray = None) -> float:

        reward = 0.0

        # Health reward
        if self._previous_health_normalized is not None:
            health_reward =  10.0 * (self._current_health_normalized - self._previous_health_normalized)
        else:
            health_reward = 0.0

        self._previous_health_normalized = self._current_health_normalized

        # Target health reward
        if self._previous_target_health_normalized is not None:
            target_health_reward = -10.0 * (self._current_target_health_normalized - self._previous_target_health_normalized)
        else:
            target_health_reward = 0.0

        self._previous_target_health_normalized = self._current_target_health_normalized

        # Distance reward
        distance_threshold = 10.0
        if self._previous_target_distance is not None:
            if self._current_target_distance > distance_threshold:
                distance_reward = 1.0 * (self._previous_target_distance - self._current_target_distance)

            else:
                distance_reward = -1.0 * (self._previous_target_distance - self._current_target_distance)
        else:
            distance_reward = 0.0

        self._previous_target_distance = self._current_target_distance

        # # Has fired reward
        turret_has_fired = state.tank.turret_sensor.has_fired
        has_fired_reward = -0.1 if turret_has_fired else 0.0

        # Total reward
        reward = health_reward + target_health_reward + distance_reward + has_fired_reward
        # reward = health_reward + target_health_reward + distance_reward

        return reward

    def terminated(self, state: UC3AgentState) -> bool:

        has_died = state.tank.health_info.health <= 0.0
        has_target_died = state.enemy_tank.health_info.health <= 0.0
        terminated = has_died or has_target_died

        return terminated

    def truncated(self, state: UC3AgentState) -> bool:

        truncated = self.n_step > self.MAX_EPISODE_STEPS

        return truncated

    def info(self, state: UC3AgentState) -> dict:
        return {}

    def render(self):
        pass
        
        # # Check if the render mode is valid
        # valid_render_modes = ['human', 'rgb_array']

        # if render_mode not in valid_render_modes:
        #     raise ValueError(f"Invalid render mode: {render_mode}. Valid render modes are {valid_render_modes}")

        # state = self.step_response.state
        
        # # Decompress the image
        # np_arr = np.frombuffer(state.compressed_image.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # if render_mode == 'human':
        #     cv2.imshow("ShootingExampleEnvironment", image)
        #     cv2.waitKey(1)

        # elif render_mode == 'rgb_array':
        #     return image
        
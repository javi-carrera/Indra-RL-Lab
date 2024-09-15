# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time

import gymnasium as gym
import numpy as np

from scipy.spatial.transform import Rotation

from interfaces_pkg.srv import UC1EnvironmentStep, UC1EnvironmentReset
from rl_pkg.environment_node import EnvironmentNode


class UC1Environment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # ROS initialization
        EnvironmentNode.__init__(
            self,
            environment_name='uc1_environment',
            environment_id=environment_id,
            step_service_msg_type=UC1EnvironmentStep,
            reset_service_msg_type=UC1EnvironmentReset,
        )

        # Gym environment initialization
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(25,),
            dtype=np.float32
        )

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        self.reward_range = (-1.0, 1.0)

        # Environment parameters
        self._min_linear_velocity = -5.0
        self._max_linear_velocity = 5.0
        self._max_yaw_rate = 5.0
        self._max_episode_time_seconds = 60.0 # 3 minutes
        self._episode_start_time_seconds = None

        self._current_target_distance = None
        self._previous_target_distance = None
        # self._current_target_distance_normalized = None


    def convert_action_to_request(self, action: np.ndarray = None):
        
        # action = np.array([linear_velocity, yaw_rate])
        # AutonomousNavigationExampleEnvironmentAction.Request:
        # geometry_msgs/Twist twist

        # Scale the action to the range [self._min_linear_velocity, self._max_linear_velocity] when action[0] is in the range [-1.0, 1.0]
        linear_velocity = (action[0] + 1.0) * (self._max_linear_velocity - self._min_linear_velocity) / 2.0 + self._min_linear_velocity
        yaw_rate = action[1] * self._max_yaw_rate

        # Set the agent linear velocity and yaw rate
        self.step_request.action.tank_action.target_twist2d.y = linear_velocity
        self.step_request.action.tank_action.target_twist2d.theta = yaw_rate

        return self.step_request


    def convert_response_to_state(self, response):

        return response.state
    

    def reset(self):

        self._episode_start_time_seconds = time.time()
        self._previous_target_distance = None

        return super().reset()
    

    def observation(self, state) -> np.ndarray:

        # Get the target relative position in the global coordinate system
        target_relative_position = np.array([
            state.target_pose2d.x - state.tank_state.pose2d.x,
            state.target_pose2d.y - state.tank_state.pose2d.y,
            0.0
        ])

        # Get the euler angles
        yaw = state.tank_state.pose2d.theta

        
        # Rotate the target relative position
        r = Rotation.from_euler('z', yaw)
        target_relative_position = r.apply(target_relative_position)

        # Remove the z component
        target_relative_position = target_relative_position[:2]

        # Normalize the target relative position
        self._current_target_distance = np.linalg.norm(target_relative_position)
        target_relative_position_normalized = target_relative_position if self._current_target_distance < 1.0 else target_relative_position / self._current_target_distance

        # target_relative_position_normalized = target_relative_position / self._max_target_distance
        # self._current_target_distance_normalized = np.linalg.norm(target_relative_position_normalized)

        # Get the linear and angular velocities
        linear_velocity_normalized = (state.tank_state.twist2d.y - self._min_linear_velocity) / (self._max_linear_velocity - self._min_linear_velocity) * 2 - 1
        angular_velocity_normalized = state.tank_state.twist2d.theta / self._max_yaw_rate

        # Get and min-max normalize the lidar data
        ranges = np.array(state.tank_state.smart_laser_scan2d.ranges)
        lidar_ranges_normalized = (ranges - state.tank_state.smart_laser_scan2d.range_min) / (state.tank_state.smart_laser_scan2d.range_max - state.tank_state.smart_laser_scan2d.range_min)

        # Get and normalize the agent's health
        self._current_health_normalized = state.tank_state.health_info.health / state.tank_state.health_info.max_health

        # Get the combined observation
        observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [angular_velocity_normalized],
            lidar_ranges_normalized,
            [self._current_health_normalized]
        ])

        return observation
    

    def reward(self, state, action: np.ndarray = None) -> float:

        # reward =  + self._current_health_normalized - self._current_target_distance_normalized
        reward = self._current_health_normalized

        if self._previous_target_distance is not None:
            reward += 20.0 * (self._previous_target_distance - self._current_target_distance)

        self._previous_target_distance = self._current_target_distance

        return reward
    

    def terminated(self, state) -> bool:
        
        has_reached_target = state.target_trigger_sensor.timer_count > state.target_trigger_sensor.max_timer_count
        has_died = state.tank_state.health_info.health <= 0.0

        terminated = has_reached_target or has_died

        return terminated
    

    def truncated(self, state) -> bool:
        
        episode_time_seconds = time.time() - self._episode_start_time_seconds

        truncated = episode_time_seconds > self._max_episode_time_seconds

        return truncated


    def info(self, state) -> dict:
        return {}
    

    def render(self):
        pass


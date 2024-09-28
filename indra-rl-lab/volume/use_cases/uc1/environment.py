# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import cv2

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
            environment_name="uc1_environment",
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
        self._max_episode_time_seconds = 60.0
        self._episode_start_time_seconds = None

        self._current_target_distance = None
        self._previous_target_distance = None

        self._current_health_normalized = None
        self._previous_health_normalized = None

    def convert_action_to_request(self, action: np.ndarray = None):

        # action = np.array([linear_velocity, yaw_rate])
        # AutonomousNavigationExampleEnvironmentAction.Request:
        # geometry_msgs/Twist twist

        self.step_request: UC1EnvironmentStep.Request

        # Scale the action to the range [self._min_linear_velocity, self._max_linear_velocity] when action[0] is in the range [-1.0, 1.0]
        linear_velocity = (action[0] + 1.0) * (self._max_linear_velocity - self._min_linear_velocity) / 2.0 + self._min_linear_velocity
        yaw_rate = action[1] * self._max_yaw_rate

        # Set the agent linear velocity and yaw rate
        self.step_request.action.tank.target_twist.y = linear_velocity
        self.step_request.action.tank.target_twist.theta = yaw_rate

        return self.step_request

    def convert_response_to_state(self, response):

        return response.state

    def reset(self):

        self._episode_start_time_seconds = time.time()
        self._previous_health_normalized = None
        self._previous_target_distance = None

        return super().reset()

    def observation(self, state) -> np.ndarray:

        # Get the target relative position in the global coordinate system
        target_relative_position = np.array([state.target_pose.x - state.tank.pose.x, state.target_pose.y - state.tank.pose.y, 0.0])

        # Get the tank's yaw
        yaw = state.tank.pose.theta

        # Rotate the target relative position
        r = Rotation.from_euler("z", yaw, degrees=True).inv()
        target_relative_position = r.apply(target_relative_position)

        # Remove the z component
        target_relative_position = target_relative_position[:2]

        # Normalize the target relative position
        self._current_target_distance = np.linalg.norm(target_relative_position)
        threshold = 10.0
        target_relative_position_normalized = (target_relative_position / threshold if self._current_target_distance < threshold else target_relative_position / self._current_target_distance)

        # Get the linear and angular velocities
        linear_velocity_normalized = (state.tank.twist.y - self._min_linear_velocity) / (self._max_linear_velocity - self._min_linear_velocity) * 2 - 1
        angular_velocity_normalized = state.tank.twist.theta / self._max_yaw_rate

        # Get and min-max normalize the lidar data
        ranges = np.array(state.tank.smart_laser_scan.ranges)
        lidar_ranges_normalized = (ranges - state.tank.smart_laser_scan.range_min) / (state.tank.smart_laser_scan.range_max - state.tank.smart_laser_scan.range_min)

        # Get and normalize the agent's health
        self._current_health_normalized = state.tank.health_info.health / state.tank.health_info.max_health

        # Get the combined observation
        observation = np.concatenate([
                target_relative_position_normalized,
                [linear_velocity_normalized],
                [angular_velocity_normalized],
                lidar_ranges_normalized,
                [self._current_health_normalized],
        ])

        # self.logger.info(f"Distance: {target_relative_position_normalized}")

        return observation

    def reward(self, state, action: np.ndarray = None) -> float:
        
        reward = 0.0

        # Calculate the health reward
        if self._previous_health_normalized is not None:
            health_reward =  + self._current_health_normalized - self._previous_health_normalized
        else:
            health_reward = 0.0

        # Calculate the distance reward
        if self._previous_target_distance is not None:
            distance_reward = 10.0 * (self._previous_target_distance - self._current_target_distance)
        else:
            distance_reward = 0.0

        # Calculate the total reward
        reward = health_reward + distance_reward
        
        # Update the previous health and target distance
        self._previous_health_normalized = self._current_health_normalized
        self._previous_target_distance = self._current_target_distance

        return reward

    def terminated(self, state) -> bool:

        has_reached_target = state.target_trigger_sensor.timer_count >= state.target_trigger_sensor.max_timer_count
        has_died = state.tank.health_info.health <= 0.0

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

        # # Check if the render mode is valid
        # valid_render_modes = ["human", "rgb_array"]

        # if render_mode not in valid_render_modes:
        #     raise ValueError(f"Invalid render mode: {render_mode}. Valid render modes are {valid_render_modes}")

        # # Decompress the image
        # np_arr = np.frombuffer(self.step_response.compressed_image.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # if render_mode == "human":
        #     cv2.imshow("ShootingExampleEnvironment", image)
        #     cv2.waitKey(1)

        # elif render_mode == "rgb_array":
        #     return image

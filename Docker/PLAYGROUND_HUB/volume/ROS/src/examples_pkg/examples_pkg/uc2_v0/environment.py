# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import yaml

import gymnasium as gym
import numpy as np

from scipy.spatial.transform import Rotation

from interfaces_pkg.srv import ShootingExampleEnvironmentReset, ShootingExampleEnvironmentStep
from playground_pkg.gym_env_wrapper import GymEnvWrapper
from playground_pkg.environment_node import EnvironmentNode
from playground_pkg.utils.communication_monitor import CommunicationMonitor
from playground_pkg.visualizers.smart_lidar_sensor_visualizer import SmartLidarSensorVisualizer
from playground_pkg.visualizers.trigger_sensor_visualizer import TriggerSensorVisualizer


class ShootingExampleEnvironment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # ROS initialization
        EnvironmentNode.__init__(
            self,
            environment_name='shooting_example_environment',
            environment_id=environment_id,
            step_service_msg_type=ShootingExampleEnvironmentStep,
            reset_service_msg_type=ShootingExampleEnvironmentReset,
        )

        # Gym environment initialization
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(26,),
            dtype=np.float32
        )

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32
        )

        self.reward_range = (-1.5, 1.5)

        # Environment parameters
        self._min_linear_velocity = -5.0
        self._max_linear_velocity = 5.0
        self._max_yaw_rate = 5.0
        self._max_episode_time_seconds = 60.0 # 3 minutes
        self._episode_start_time_seconds = None

        self._current_target_distance = None
        self._previous_target_distance = None
        self._current_health_normalized = None
        self._current_target_health_normalized = None

        # Visualizers initialization
        self._smart_lidar_sensor_visualizer = SmartLidarSensorVisualizer()
        self._trigger_sensor_visualizer = TriggerSensorVisualizer()


    def convert_action_to_request(self, action: np.ndarray = None):
        
        # action = np.array([linear_velocity, yaw_rate, turret_target_angle, fire])

        # Scale the action to the range [self._min_linear_velocity, self._max_linear_velocity] when action[0] is in the range [-1.0, 1.0]
        linear_velocity = (action[0] + 1.0) * (self._max_linear_velocity - self._min_linear_velocity) / 2.0 + self._min_linear_velocity
        yaw_rate = action[1] * self._max_yaw_rate

        # Scale the action to the range [0.0, 360.0] when action[2] is in the range [-1.0, 1.0]
        turret_target_angle = (action[2] + 1.0) * 360.0 / 2.0
        fire = bool(action[3] > 0.5)

        # Fill the step request
        self.step_request.action.twist_actuator.linear.y = linear_velocity
        self.step_request.action.twist_actuator.angular.z = yaw_rate
        self.step_request.action.turret_actuator.target_angle = turret_target_angle
        self.step_request.action.turret_actuator.fire = fire

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
            state.target_pose_sensor.position.x - state.pose_sensor.position.x,
            state.target_pose_sensor.position.y - state.pose_sensor.position.y,
            state.target_pose_sensor.position.z - state.pose_sensor.position.z
        ])

        # Get the euler angles
        orientation = state.pose_sensor.orientation
        rotation = Rotation.from_quat(np.array([orientation.x, orientation.y, orientation.z, orientation.w]))

        #yaw, pitch, roll = rotation.as_euler('zyx', degrees=True)
        
        # Rotate the target relative position
        target_relative_position = rotation.inv().apply(target_relative_position)

        # Remove the z component
        target_relative_position = np.array([
            target_relative_position[0],
            target_relative_position[1],
        ])

        # Normalize the target relative position
        self._current_target_distance = np.linalg.norm(target_relative_position)
        target_relative_position_normalized = target_relative_position if self._current_target_distance < 1.0 else target_relative_position / self._current_target_distance

        # Get the linear and angular velocities
        linear_velocity_normalized = (state.twist_sensor.linear.y - self._min_linear_velocity) / (self._max_linear_velocity - self._min_linear_velocity) * 2 - 1
        angular_velocity_normalized = state.twist_sensor.angular.z / self._max_yaw_rate

        # Get and min-max normalize the lidar data
        ranges = np.array(state.smart_lidar_sensor.ranges)
        lidar_ranges_normalized = (ranges - state.smart_lidar_sensor.range_min) / (state.smart_lidar_sensor.range_max - state.smart_lidar_sensor.range_min)

        # Get and normalize the agent's health
        self._current_health_normalized = state.health_sensor.health / state.health_sensor.max_health

        # Get and normalize the target's health
        self._current_target_health_normalized = state.target_health_sensor.health / state.target_health_sensor.max_health

        # Get the combined observation
        observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [angular_velocity_normalized],
            lidar_ranges_normalized,
            [self._current_health_normalized],
            [self._current_target_health_normalized]
        ])

        return observation
    

    def reward(self, state, action: np.ndarray = None) -> float:

        reward = self._current_health_normalized - self._current_target_health_normalized

        if self._previous_target_distance is not None and self._current_target_distance > 4.0:
            reward += 20.0 * (self._previous_target_distance - self._current_target_distance)


        return reward
    

    def terminated(self, state) -> bool:
        
        has_died = state.health_sensor.health <= 0.0
        has_target_died = state.target_health_sensor.health <= 0.0

        terminated = has_died or has_target_died

        return terminated
    

    def truncated(self, state) -> bool:
        
        episode_time_seconds = time.time() - self._episode_start_time_seconds

        truncated = episode_time_seconds > self._max_episode_time_seconds

        return truncated



    def info(self, state) -> dict:
        return {}
    

    def render(self):
        pass


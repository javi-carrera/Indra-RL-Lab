# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time

import gymnasium as gym
import numpy as np

from scipy.spatial.transform import Rotation

from interfaces_pkg.srv import AutonomousNavigationExampleEnvironmentReset, AutonomousNavigationExampleEnvironmentStep
from playground_pkg.environment_node import EnvironmentNode
from playground_pkg.utils.pose_converter import PoseConverter
from playground_pkg.visualizers.lidar_sensor_visualizer import LidarSensorVisualizer
from playground_pkg.visualizers.trigger_sensor_visualizer import TriggerSensorVisualizer


class AutonomousNavigationExampleEnvironment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # ROS initialization
        EnvironmentNode.__init__(
            self,
            environment_name='autonomous_navigation_example_environment',
            environment_id=environment_id,
            step_service_msg_type=AutonomousNavigationExampleEnvironmentStep,
            reset_service_msg_type=AutonomousNavigationExampleEnvironmentReset,
        )

        # Gym environment initialization
        self.observation_space = gym.spaces.Box(        # TODO change
            low=-1.0,
            high=1.0,
            shape=(24,),
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
        self._max_target_distance = 25.0 * np.sqrt(2.0)
        self._current_target_distance_normalized = self._max_target_distance
        self._min_linear_velocity = 0.0
        self._max_linear_velocity = 3.0
        self._max_yaw_rate = 3.0
        self._max_episode_time_seconds = 30.0
        self._episode_start_time_seconds = None

        # Visualizers initialization
        self._lidar_sensor_visualizer = LidarSensorVisualizer()
        self._trigger_sensor_visualizer = TriggerSensorVisualizer()


    def convert_action_to_request(self, action: np.ndarray = None) -> AutonomousNavigationExampleEnvironmentStep.Request:
        
        # action = np.array([linear_velocity, yaw_rate])
        # AutonomousNavigationExampleEnvironmentAction.Request:
        # geometry_msgs/Twist twist

        self.step_request: AutonomousNavigationExampleEnvironmentStep.Request

        # Scale the action to the range [self._min_linear_velocity, self._max_linear_velocity] when action[0] is in the range [-1.0, 1.0]
        linear_velocity = (action[0] + 1.0) * (self._max_linear_velocity - self._min_linear_velocity) / 2.0 + self._min_linear_velocity
        yaw_rate = action[1] * self._max_yaw_rate

        # Set the agent linear velocity and yaw rate
        self.step_request.action.twist.linear.x = linear_velocity
        self.step_request.action.twist.angular.z = yaw_rate

        return self.step_request


    def convert_response_to_state(self, response: AutonomousNavigationExampleEnvironmentStep.Response):

        return response.state
    

    def convert_reset_to_request(self) -> AutonomousNavigationExampleEnvironmentReset.Request:

        self.reset_request: AutonomousNavigationExampleEnvironmentReset.Request
        # geometry_msgs/Pose agent_target_pose
        # geometry_msgs/Pose target_target_pose        

        # Reset the agent pose
        self.reset_request.reset_action.agent_target_pose.position.x = np.random.choice([-12.5, -7.5, 7.5, 12.5])
        self.reset_request.reset_action.agent_target_pose.position.y = np.random.choice([-12.5, -7.5, 7.5, 12.5])
        self.reset_request.reset_action.agent_target_pose.position.z = 1.0

        # Reset the target pose
        self.reset_request.reset_action.target_target_pose.position.x = np.random.choice([-12.5, -7.5, 7.5, 12.5])
        self.reset_request.reset_action.target_target_pose.position.y = np.random.choice([-12.5, -7.5, 7.5, 12.5])
        self.reset_request.reset_action.target_target_pose.position.z = 1.0

        return self.reset_request
    

    def reset(self) -> AutonomousNavigationExampleEnvironmentStep.Response:

        self._episode_start_time_seconds = time.time()
        self._current_target_distance_normalized = self._max_target_distance

        return super().reset()
    

    def observation(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> np.ndarray:

        # Get the target relative position in the global coordinate system
        target_relative_position = np.array([
            state.target_pose.position.x - state.pose.position.x,
            state.target_pose.position.y - state.pose.position.y,
            state.target_pose.position.z - state.pose.position.z
        ])

        # Get the euler angles
        orientation = state.pose.orientation
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
        target_relative_position_normalized = target_relative_position / self._max_target_distance
        self._current_target_distance_normalized = np.linalg.norm(target_relative_position_normalized)

        # Get the linear and angular velocities
        linear_velocity_normalized = state.twist.linear.x / self._max_linear_velocity
        angular_velocity_normalized = state.twist.angular.z / self._max_yaw_rate

        # Get and min-max normalize the lidar data
        ranges = np.array(state.laser_scan.ranges)
        lidar_ranges_normalized = (ranges - state.laser_scan.range_min) / (state.laser_scan.range_max - state.laser_scan.range_min)

        # Get the combined observation
        observation = np.concatenate([
            target_relative_position_normalized,
            [linear_velocity_normalized],
            [angular_velocity_normalized],
            lidar_ranges_normalized
        ])

        return observation
    

    def reward(self, state: AutonomousNavigationExampleEnvironmentStep.Response, action: np.ndarray = None) -> float:

        has_collided = state.collision_trigger_sensor.has_triggered

        reward = 1.0 - self._current_target_distance_normalized

        if has_collided:
            reward = -1.0

        return reward
    

    def terminated(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> bool:
        
        has_collided = state.collision_trigger_sensor.has_triggered
        has_reached_target = state.target_trigger_sensor.has_timer_finished

        terminated = has_collided or has_reached_target

        return terminated
    

    def truncated(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> bool:
        
        episode_time_seconds = time.time() - self._episode_start_time_seconds

        truncated = episode_time_seconds > self._max_episode_time_seconds

        return truncated


    def info(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> dict:
        return {}
    

    def render(self):

        self.state_response: AutonomousNavigationExampleEnvironmentStep.Response
        
        self._lidar_sensor_visualizer.visualize(self.step_response.state.laser_scan)
        self._trigger_sensor_visualizer.visualize([
            self.step_response.state.collision_trigger_sensor, 
            self.step_response.state.target_trigger_sensor
        ])


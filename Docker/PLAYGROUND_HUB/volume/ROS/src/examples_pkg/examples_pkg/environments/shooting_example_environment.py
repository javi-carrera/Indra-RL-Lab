from typing import Tuple, Type
import time

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.pose_converter import PoseConverter
from interfaces_pkg.srv import ShootingExampleEnvironmentStep, ShootingExampleEnvironmentReset
from playground_pkg.utils.communication_monitor import CommunicationMonitor
from playground_pkg.visualizers.smart_lidar_sensor_visualizer import SmartLidarSensorVisualizer
from playground_pkg.visualizers.trigger_sensor_visualizer import TriggerSensorVisualizer
from playground_pkg.gym_env_wrapper import GymEnvWrapper

import numpy as np
import matplotlib.pyplot as plt
import stable_baselines3 as sb3
from scipy.spatial.transform import Rotation as R

from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO


class AutonomousNavigationExampleEnvironment(SingleAgentEnvironmentNode):

    def __init__(self, environment_id: int):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='shooting_example_environment',
            environment_id=environment_id,
            step_service_msg_type=ShootingExampleEnvironmentStep,
            reset_service_msg_type=ShootingExampleEnvironmentReset,
        )

        # Environment parameters
        self.max_relative_target_distance = 5.0
        self._previous_target_relative_distance = None
        self._current_target_relative_distance = None
        self._min_linear_velocity = -1.0
        self._max_linear_velocity = 3.0
        self._max_yaw_rate = 3.0

        # Visualizers initialization
        self._smart_lidar_sensor_visualizer = SmartLidarSensorVisualizer()
        self._trigger_sensor_visualizer = TriggerSensorVisualizer()


    def convert_action_to_request(self, action: np.ndarray = None) -> ShootingExampleEnvironmentStep.Request:
        
        # action = np.array([linear_velocity, yaw_rate])
        # AutonomousNavigationExampleEnvironmentAction.Request:
        # geometry_msgs/Twist twist

        self.step_request: ShootingExampleEnvironmentStep.Request

        # Scale the action to the range [self._min_linear_velocity, self._max_linear_velocity] when action[0] is in the range [-1.0, 1.0]
        linear_velocity = (action[0] + 1.0) * (self._max_linear_velocity - self._min_linear_velocity) / 2.0 + self._min_linear_velocity
        yaw_rate = action[1] * self._max_yaw_rate

        # Set the agent linear velocity and yaw rate
        self.step_request.action.twist.linear.x = linear_velocity
        self.step_request.action.twist.angular.y = yaw_rate

        return self.step_request


    def convert_response_to_state(self, response: ShootingExampleEnvironmentStep.Response):

        return response
    

    def convert_reset_to_request(self) -> ShootingExampleEnvironmentReset.Request:

        self.reset_request: ShootingExampleEnvironmentReset.Request
        # geometry_msgs/Pose agent_target_pose
        # geometry_msgs/Pose target_target_pose        

        # Reset the agent pose
        self.reset_request.reset_action.agent_target_pose.position.x = np.random.choice([-12.5, -7.5, 7.5, 12.5])
        self.reset_request.reset_action.agent_target_pose.position.y = 1.0
        self.reset_request.reset_action.agent_target_pose.position.z = np.random.choice([-12.5, -7.5, 7.5, 12.5])

        # Reset the target pose
        self.reset_request.reset_action.target_target_pose.position.x = np.random.choice([-10.0, -5.0, 5.0, 10.0])
        self.reset_request.reset_action.target_target_pose.position.y = 1.0
        self.reset_request.reset_action.target_target_pose.position.z = np.random.choice([-10.0, -5.0, 5.0, 10.0])

        return self.reset_request  
    

    def observation(self, state: ShootingExampleEnvironmentStep.Response) -> np.ndarray:

        observation = np.array([])

        return observation
    

    def reward(self, state: ShootingExampleEnvironmentStep.Response, action: np.ndarray = None) -> float:

        reward = 0.0

        return reward
    

    def terminated(self, state: ShootingExampleEnvironmentStep.Response) -> bool:
        
        terminated = False

        return terminated
    

    def truncated(self, state: ShootingExampleEnvironmentStep.Response) -> bool:
        return False


    def info(self, state: ShootingExampleEnvironmentStep.Response) -> dict:
        return {}
    

    def render(self):

        self.state_response: ShootingExampleEnvironmentStep.Response
        
        self._smart_lidar_sensor_visualizer.visualize(self.step_response.state.smart_lidar_sensor)
        self._trigger_sensor_visualizer.visualize([
            self.step_response.state.collision_trigger_sensor, 
        ])


def create_environment(environment_id: int) -> GymEnvWrapper:

    observation_space = gym.spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(5,),
        dtype=np.float32
    )

    action_space = gym.spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(2,),
        dtype=np.float32
    )

    reward_range = (-np.inf, np.inf)

    return GymEnvWrapper(
        env=AutonomousNavigationExampleEnvironment(environment_id),
        observation_space=observation_space,
        action_space=action_space,
        reward_range=reward_range
    )


def main():

    simulated_inference_time = 0.0

    env = create_environment(environment_id=0)
    communication_monitor = CommunicationMonitor(env)

    env.reset()
    action = np.array([0.0, 0.0])

    while True:
        
        observation, reward, terminated, truncated, info = env.step(action)
        action = np.random.uniform(-1.0, 1.0, 2)

        if terminated or truncated:
            env.reset()

        communication_monitor.display()
        env.render()

        time.sleep(simulated_inference_time)

    env.close()


if __name__ == '__main__':
    main()



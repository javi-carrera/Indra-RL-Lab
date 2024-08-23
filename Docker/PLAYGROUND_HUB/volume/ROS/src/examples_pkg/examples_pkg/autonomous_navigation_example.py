from typing import Tuple, Type
import time

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.pose_converter import PoseConverter
from interfaces_pkg.srv import AutonomousNavigationExampleEnvironmentStep, AutonomousNavigationExampleEnvironmentReset
from playground_pkg.utils.communication_monitor import CommunicationMonitor
from playground_pkg.utils.lidar_sensor_visualizer import LidarSensorVisualizer
from playground_pkg.utils.smart_lidar_sensor_visualizer import SmartLidarSensorVisualizer
from playground_pkg.utils.trigger_sensor_visualizer import TriggerSensorVisualizer
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
            environment_name='autonomous_navigation_example_environment',
            environment_id=environment_id,
            step_service_msg_type=AutonomousNavigationExampleEnvironmentStep,
            reset_service_msg_type=AutonomousNavigationExampleEnvironmentReset,
        )

        # Environment parameters
        self.max_relative_target_distance = 5.0
        self._previous_target_relative_distance = None
        self._current_target_relative_distance = None
        self._min_linear_velocity = -1.0
        self._max_linear_velocity = 3.0
        self._max_yaw_rate = 3.0

        # Visualizers initialization
        # self._lidar_sensor_visualizer = LidarSensorVisualizer()
        self._smart_lidar_sensor_visualizer = SmartLidarSensorVisualizer()
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
        self.step_request.action.twist.angular.y = yaw_rate

        return self.step_request


    def convert_response_to_state(self, response: AutonomousNavigationExampleEnvironmentStep.Response):

        return response
    

    def convert_reset_to_request(self) -> AutonomousNavigationExampleEnvironmentReset.Request:

        self.reset_request: AutonomousNavigationExampleEnvironmentReset.Request
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
    

    def observation(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> np.ndarray:

        # Get the target relative position in the global coordinate system
        target_relative_position = np.array([
            state.state.target_pose.position.x - state.state.pose.position.x,
            state.state.target_pose.position.y - state.state.pose.position.y,
            state.state.target_pose.position.z - state.state.pose.position.z
        ])

        # Get the euler angles
        euler_angles, rotation = PoseConverter.unity_quaternion_to_ros_euler(np.array([
            state.state.pose.orientation.x,
            state.state.pose.orientation.y,
            state.state.pose.orientation.z,
            state.state.pose.orientation.w
        ]))
        
        yaw = euler_angles[2]

        # Rotate the target relative position
        target_relative_position = rotation.apply(target_relative_position)

        # Remove the z component
        target_relative_position = np.array([
            target_relative_position[0],
            target_relative_position[1],
        ])

        # Normalize the target relative position
        target_relative_position = target_relative_position / self.max_relative_target_distance
        self._current_target_relative_distance = np.linalg.norm(target_relative_position)

        # Limit the target relative distance to 1.0
        if self._current_target_relative_distance > 1.0:
            target_relative_position = target_relative_position / self._current_target_relative_distance

        # Normalize the yaw to [-1.0, 1.0]
        yaw = yaw / np.pi

        # Get and min-max normalize the lidar data
        # lidar_ranges = (state['laser_scan']['ranges'] - state['laser_scan']['range_min']) / (state['laser_scan']['range_max'] - state['laser_scan']['range_min'])

        # Get the combined observation
        observation = np.concatenate([
            target_relative_position,
            [yaw],
            # lidar_ranges
        ])

        return observation
    

    def reward(self, state: AutonomousNavigationExampleEnvironmentStep.Response, action: np.ndarray = None) -> float:

        # Check if the agent has moved closer to the target
        if self._previous_target_relative_distance is not None:
            reward = 0.1 * np.sign(self._previous_target_relative_distance - self._current_target_relative_distance)
        else:
            reward = 0.0

        if state.state.target_trigger_sensor.has_triggered:
            reward = 10.0

        # Update the previous target relative distance
        self._previous_target_relative_distance = self._current_target_relative_distance

        return reward
    

    def terminated(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> bool:
        
        has_collided = state.state.collision_trigger_sensor.has_triggered
        has_reached_target = state.state.target_trigger_sensor.has_triggered

        terminated = has_collided or has_reached_target

        return terminated
    

    def truncated(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> bool:
        return False


    def info(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> dict:
        return {}
    

    def render(self):

        self.state_response: AutonomousNavigationExampleEnvironmentStep.Response
        
        self._smart_lidar_sensor_visualizer.visualize(self.step_response.state.smart_lidar_sensor)
        self._trigger_sensor_visualizer.visualize([
            self.step_response.state.collision_trigger_sensor, 
            self.step_response.state.target_trigger_sensor
        ])




def main():

    rclpy.init()

    simulated_inference_time = 0.1

    base_env = AutonomousNavigationExampleEnvironment(environment_id=0)

    print("Environment initialized...")


    observation_space = gym.spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(3,),
        dtype=np.float32
    )

    action_space = gym.spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(2,),
        dtype=np.float32
    )
    reward_range = (-np.inf, np.inf)

    env = GymEnvWrapper(
        base_env,
        observation_space=observation_space,
        action_space=action_space,
        reward_range=reward_range
    )

    print("Environment wrapped...")

    communication_monitor = CommunicationMonitor(base_env)

    print("Communication monitor initialized...")

    # # Check the environment
    # check_env(env)

    normalize = True
    n_timesteps = 5e6
    policy = 'MlpPolicy'
    n_steps = 2048
    batch_size = 64
    gae_lambda = 0.95
    gamma = 0.999
    n_epochs = 10
    ent_coef = 0.0
    learning_rate = 3e-4
    clip_range = 0.18

    # # Create the agent
    # model = PPO(
    #     policy,
    #     env,
    #     verbose=1,
    #     learning_rate=learning_rate,
    #     n_steps=n_steps,
    #     batch_size=batch_size,
    #     n_epochs=n_epochs,
    #     gamma=gamma,
    #     gae_lambda=gae_lambda,
    #     clip_range=clip_range,
    #     ent_coef=ent_coef,
    #     normalize_advantage=normalize,
    # )

    # Train the agent
    # model.learn(total_timesteps=int(n_timesteps))


    env.reset()
    action = np.array([0.0, 0.0])

    
    print("Starting the environment...")
    while True:
        
        state, reward, terminated, truncated, info = env.step(action)

        # communication_monitor.display()

        action = np.random.uniform(-1.0, 1.0, 2)

        if terminated or truncated:
            env.reset()

        env.render()

        time.sleep(simulated_inference_time)




    env.close()


if __name__ == '__main__':
    main()

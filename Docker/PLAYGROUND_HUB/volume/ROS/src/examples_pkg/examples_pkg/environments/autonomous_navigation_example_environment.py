# Project: Playground
# File: autonomous_navigation_example_environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import yaml

import gymnasium as gym
import numpy as np

from interfaces_pkg.srv import AutonomousNavigationExampleEnvironmentReset, AutonomousNavigationExampleEnvironmentStep
from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.communication_monitor import CommunicationMonitor
from playground_pkg.utils.pose_converter import PoseConverter
from playground_pkg.visualizers.lidar_sensor_visualizer import LidarSensorVisualizer
from playground_pkg.visualizers.trigger_sensor_visualizer import TriggerSensorVisualizer


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

        # Gym environment initialization
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(5,),
            dtype=np.float32
        )

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        self.reward_range = (-np.inf, np.inf)

        # Environment parameters
        self.max_relative_target_distance = 5.0
        self._previous_target_relative_distance = None
        self._current_target_relative_distance = None
        self._min_linear_velocity = -1.0
        self._max_linear_velocity = 3.0
        self._max_yaw_rate = 3.0

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

        # # Limit the target relative distance to 1.0
        # if self._current_target_relative_distance > 1.0:
        #     target_relative_position = target_relative_position / self._current_target_relative_distance

        # Normalize the yaw to [-1.0, 1.0]
        yaw = yaw / np.pi

        # Get and min-max normalize the lidar data
        # lidar_ranges = (state['laser_scan']['ranges'] - state['laser_scan']['range_min']) / (state['laser_scan']['range_max'] - state['laser_scan']['range_min'])

        # Get the linear and angular velocities
        linear_velocity = state.state.twist.linear.x
        angular_velocity = state.state.twist.angular.y

        # Get the combined observation
        observation = np.concatenate([
            target_relative_position,
            [linear_velocity],
            [angular_velocity],
            [yaw],
            # lidar_ranges
        ])

        return observation
    

    def reward(self, state: AutonomousNavigationExampleEnvironmentStep.Response, action: np.ndarray = None) -> float:

        reward = 0.0

        # Check if the agent has moved closer to the target
        if self._previous_target_relative_distance is not None:
            reward += 10.0 * (self._previous_target_relative_distance - self._current_target_relative_distance)

        if state.state.target_trigger_sensor.has_timer_finished:
            reward = 10.0

        # Update the previous target relative distance
        self._previous_target_relative_distance = self._current_target_relative_distance

        return reward
    

    def terminated(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> bool:
        
        has_collided = state.state.collision_trigger_sensor.has_triggered
        has_reached_target = state.state.target_trigger_sensor.has_timer_finished

        terminated = has_collided or has_reached_target

        return terminated
    

    def truncated(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> bool:
        return False


    def info(self, state: AutonomousNavigationExampleEnvironmentStep.Response) -> dict:
        return {}
    

    def render(self):

        self.state_response: AutonomousNavigationExampleEnvironmentStep.Response
        
        self._lidar_sensor_visualizer.visualize(self.step_response.state.laser_scan)
        self._trigger_sensor_visualizer.visualize([
            self.step_response.state.collision_trigger_sensor, 
            self.step_response.state.target_trigger_sensor
        ])



def test_gym_environment():

    simulated_inference_time = 0.0

    env = AutonomousNavigationExampleEnvironment.create_gym_environment(environment_id=0)
    communication_monitor = CommunicationMonitor(env)
    
    env.reset()
    action = np.array([0.0, 0.0])

    while True:
        
        observation, reward, terminated, truncated, info = env.step(action)
        action = np.random.uniform(-1.0, 1.0, 2)

        communication_monitor.display()
        env.render()

        if terminated or truncated:
            env.reset()

        time.sleep(simulated_inference_time)

    env.close()


def test_vectorized_environment():

    # Load the configuration file
    config_file_path = "config.yml"
    with open(config_file_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    n_environments = config["n_environments"]
    simulated_inference_time = 1.0
    
    vec_env = AutonomousNavigationExampleEnvironment.create_vectorized_environment(n_environments=n_environments, return_type='gym')

    vec_env.reset()
    actions = [[0.0, 0.0] for _ in range(vec_env.num_envs)]

    while True:

        start_time = time.perf_counter()

        # Step the environment
        observations, rewards, terminateds, truncateds, infos = vec_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vec_env.num_envs)]

        # Simulate inference time
        time.sleep(simulated_inference_time)

        print(f"Time taken: {time.perf_counter() - start_time}")

    vec_env.close()


def main():

    #test_gym_environment()
    test_vectorized_environment()



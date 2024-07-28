from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from testing_pkg.lidar_sensor_test import LidarSensorVisualizer
from interfaces_pkg.srv import EscoPilotExampleEnvironmentStep

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from stable_baselines3 import PPO # pip install stable-baselines3
# import everything needed to wrap env in a gymnasium environment
from stable_baselines3.common.env_checker import check_env
from gymnasium import Env

class GymWrapper(Env):
    def __init__(self, env):
        self.env = env
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float64)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(23,), dtype=np.float64)
    
    def reset(self, **kwargs):
        return self.env.reset()
    
    def step(self, action):
        return self.env.step(action)
    
    def render(self):
        self.env.render()

class EscoPilotExampleEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='escopilot_example_environment',
            service_msg_type=EscoPilotExampleEnvironmentStep,
        )

        # Initialize the visualizer
        self.lidar_visualizer = LidarSensorVisualizer()
        self._last_distance_to_target = np.inf


    def convert_action_to_request(self, action: np.ndarray) -> EscoPilotExampleEnvironmentStep.Request:
        
        # action = np.array([vx, vy, wz]) in meters/second and radians/second
        # print(action)

        # Convert the action to ROS request format (target_velocity is a Twist)
        self._request.agent_action.target_velocity.linear.x = action[0]
        self._request.agent_action.target_velocity.linear.y = 0.0
        self._request.agent_action.target_velocity.linear.z = 0.0

        self._request.agent_action.target_velocity.angular.x = 0.0
        self._request.agent_action.target_velocity.angular.y = action[1]
        self._request.agent_action.target_velocity.angular.z = 0.0

        return self._request
    

    def convert_response_to_state(self, response) -> np.ndarray:

        # sensor_msgs/LaserScan laser_scan
        # geometry_msgs/Pose pose
        # geometry_msgs/Pose target_pose
        # TriggerSensor target_trigger_sensor

        # Store the response
        self._response = response

        # Convert the response to dict
        state = {
            'laser_scan': {
                'ranges': response.agent_state.laser_scan.ranges,
                'range_min': response.agent_state.laser_scan.range_min,
            },
            'pose': {
                'position': np.array([
                    response.agent_state.pose.position.x,
                    response.agent_state.pose.position.y,
                ]),
                'orientation': np.array([
                    response.agent_state.pose.orientation.x,
                    response.agent_state.pose.orientation.y,
                    response.agent_state.pose.orientation.z,
                    response.agent_state.pose.orientation.w,
                ]),
            },
            'target_pose': {
                'position': np.array([
                    response.agent_state.target_pose.position.x,
                    response.agent_state.target_pose.position.y,
                ]),
                'orientation': np.array([
                    response.agent_state.target_pose.orientation.x,
                    response.agent_state.target_pose.orientation.y,
                    response.agent_state.target_pose.orientation.z,
                    response.agent_state.target_pose.orientation.w,
                ]),
            },
            'target_trigger_sensor': {
                'has_triggered': response.agent_state.target_trigger_sensor.has_triggered,
                'has_timer_finished': response.agent_state.target_trigger_sensor.has_timer_finished,
            }
        }

        return state
    

    def render(self):
        # Visualize the lidar data
        self.lidar_visualizer.visualize(
            self._response.agent_state.laser_scan.ranges,
            self._response.agent_state.laser_scan.angle_min,
            self._response.agent_state.laser_scan.angle_max,
            self._response.agent_state.laser_scan.angle_increment,
            self._response.agent_state.laser_scan.range_min,
            self._response.agent_state.laser_scan.range_max
        )


    def observation(self, state: np.ndarray) -> np.ndarray:
        # normalize rel_pos if magnitude is greater than 1
        rel_pos = (state['target_pose']['position'] - state['pose']['position'])
        if np.linalg.norm(rel_pos) > 1:
            rel_pos /= np.linalg.norm(rel_pos)

        lidar_obs = np.interp(state['laser_scan']['ranges'], 
                      (self._response.agent_state.laser_scan.range_min, self._response.agent_state.laser_scan.range_max), 
                      (0, 1))

        state_np = np.array([
            *lidar_obs,
            *rel_pos,
            (R.from_quat(state['target_pose']['orientation']).as_euler('xyz') - R.from_quat(state['pose']['orientation']).as_euler('xyz'))[1],
        ])
        return state_np
    
    def reward(self, state: np.ndarray, action: np.ndarray = None) -> float:
        distance_to_target = np.linalg.norm(state['target_pose']['position'] - state['pose']['position'])   

        if state["target_trigger_sensor"]["has_timer_finished"]:
            return 100.0
        # colision if the minimum distance is less than (self._response.agent_state.laser_scan.range_min * 1.05) meters
        if min(state["laser_scan"]["ranges"]) < (state["laser_scan"]["range_min"] * 1.05):
            return -10.0
        # if the distance to the target is less than the last distance to the target, give a reward
        reward = np.clip((self._last_distance_to_target - distance_to_target)*100, -0.5, 0.5)
        # print(reward)

        self._last_distance_to_target = distance_to_target
        return reward
    
    def terminated(self, state: np.ndarray) -> bool:
        return state["target_trigger_sensor"]["has_timer_finished"]
    
    def truncated(self, state: np.ndarray) -> bool:
        return min(state["laser_scan"]["ranges"]) < (state["laser_scan"]["range_min"] * 1.05)

    def info(self, state: np.ndarray) -> dict:
        return {}


def main():

    rclpy.init()

    env = EscoPilotExampleEnvironment()
    env = GymWrapper(env)
    check_env(env)


    normalize = True
    n_timesteps = 5e6
    policy = 'MlpPolicy'
    n_steps = 2048
    batch_size = 64
    gae_lambda = 0.95
    gamma = 0.999
    n_epochs = 50
    ent_coef = 0.0
    learning_rate = 3e-2
    clip_range = 0.18

    # Create the agent
    model = PPO(
        policy,
        env,
        verbose=1,
        learning_rate=learning_rate,
        n_steps=n_steps,
        batch_size=batch_size,
        n_epochs=n_epochs,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        ent_coef=ent_coef,
        normalize_advantage=normalize,
    )

    # # Train the agent
    # for epoch in range(int(n_timesteps // n_steps)):
    #     model.learn(n_steps)
    #     env.render()


    # Train the agent
    model.learn(total_timesteps=int(n_timesteps), progress_bar=True)
    # action = np.array([0.0, 0.0, 0.0])

    # while True:
    #     observation, reward, terminated, truncated, info = env.step(action)
    #     if terminated or truncated:
    #         env.reset()
    #     env.render(observation)
    #     print(reward)

    env.close()


if __name__ == '__main__':
    main()

import gymnasium as gym
import rclpy

import numpy as np
from gymnasium.vector import VectorEnv
from gymnasium.spaces import Space

from examples_pkg.autonomous_navigation_example import AutonomousNavigationExampleEnvironment

class VectorizedEnvironment(VectorEnv):

    def __init__(
            self,
            env,
            num_envs: int,
            sample_time: float,
        ):

        # Initialize the environment
        self.envs = []
        for i in range(num_envs):

            env = env(
                environment_id=i,
                sample_time=sample_time
            )

            self.envs.append(env)
            self.num_envs = num_envs

        self.action_space = self.envs[0].action_space
        self.observation_space = self.envs[0].observation_space

    
    def reset(self):
        return [env.reset() for env in self.envs]

    def step(self, actions):
        return zip(*[env.step(action) for env, action in zip(self.envs, actions)])


def main():

    rclpy.init()

    # Create the vectorized environment
    vectorized_env = VectorizedEnvironment(
        env=AutonomousNavigationExampleEnvironment,
        num_envs=4,
        sample_time=0.1
    )

    # Reset the environment
    obs = vectorized_env.reset()
    print(obs)

    # Take a step in the environment
    actions = [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
    ]

    while True:
        obs, rewards, dones, infos = vectorized_env.step(actions)
        print(obs, rewards, dones, infos)





    



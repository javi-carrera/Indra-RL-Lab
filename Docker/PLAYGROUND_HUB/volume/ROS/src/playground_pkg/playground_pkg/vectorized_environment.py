import gymnasium as gym
import rclpy
from rclpy.executors import SingleThreadedExecutor

import numpy as np
from gymnasium.vector import VectorEnv
from gymnasium.spaces import Space

from examples_pkg.autonomous_navigation_example import AutonomousNavigationExampleEnvironment

from stable_baselines3.common.env_checker import check_env
from typing import Tuple, Type
import time


# class VectorizedEnvironment(VectorEnv):

#     def __init__(
#             self,
#             env_cls,
#             num_envs: int,
#             sample_time: float,
#         ):

#         # Initialize the environment
#         self.envs = []
#         for i in range(num_envs):

#             env = env_cls(
#                 environment_id=i,
#                 sample_time=sample_time
#             )

#             self.envs.append(env)

#         self.num_envs = num_envs
#         # self.action_space = self.envs[0].action_space
#         # self.observation_space = self.envs[0].observation_space

    
#     def reset(self):
#         return zip(*[env.reset() for env in self.envs])

#     def step(self, actions):
#         return zip(*[env.step(action) for env, action in zip(self.envs, actions)])


# def create_environment(environment_id: int, sample_time: float) -> AutonomousNavigationExampleEnvironment:
#     return AutonomousNavigationExampleEnvironment(
#         environment_id=environment_id,
#         sample_time=sample_time
#     )


class GymEnvWrapper(gym.Env):

    def __init__(self, env: Type[AutonomousNavigationExampleEnvironment]):

        # Environment initialization
        self.env = env

        # Environment parameters
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3,),
            dtype=np.float32
        )

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        self.reward_range = (-np.inf, np.inf)
    

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        observation, reward, terminated, truncated, info = self.env.step(action)

        # Cast the observation from float64 to float32
        observation = observation.astype(np.float32)

        return observation, reward, terminated, truncated, info
    

    def reset(self, **kwargs) -> Tuple[np.ndarray, dict]:

        observation, info = self.env.reset()

        # Cast the observation from float64 to float32
        observation = observation.astype(np.float32)

        return observation, info
    

    def render(self):
        return self.env.render()
    

    def close(self):
        return self.env.close()
    

def create_environment(environment_id: int, sample_time: float) -> GymEnvWrapper:

    

    return GymEnvWrapper(
        env=AutonomousNavigationExampleEnvironment(
            environment_id=environment_id,
            sample_time=sample_time
        )
    )


def main():

    
    rclpy.init()

    # Define the number of environments
    num_envs = 16

    # Create the vectorized environment
    vectorized_env = gym.vector.AsyncVectorEnv([
        lambda i=i: create_environment(environment_id=i, sample_time=0.0) for i in range(num_envs)
    ])
    

    # Reset the environment
    vectorized_env.reset()

    # Take a step in the environment
    actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]

    while True:
        start_time = time.time()
        observations, rewards, terminated, truncated, infos = vectorized_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]
        print(f"Time taken: {time.time() - start_time}")





    



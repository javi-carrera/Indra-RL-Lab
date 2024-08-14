import gymnasium as gym
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import numpy as np
from gymnasium.vector import VectorEnv, AsyncVectorEnv, SyncVectorEnv
from gymnasium.spaces import Space

from playground_pkg.gym_env_wrapper import GymEnvWrapper
from examples_pkg.autonomous_navigation_example import AutonomousNavigationExampleEnvironment

from stable_baselines3.common.env_checker import check_env
from typing import Tuple, Type, Callable, List
import time
import asyncio
from concurrent.futures import ThreadPoolExecutor



    

def create_environment(environment_id: int, sample_time: float) -> GymEnvWrapper:

    rclpy.init()

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

    return GymEnvWrapper(
        env=AutonomousNavigationExampleEnvironment(
            environment_id=environment_id,
            sample_time=sample_time
        ),
        observation_space=observation_space,
        action_space=action_space,
        reward_range=reward_range
    )




def main():

    

    # Define the number of environments
    num_envs = 1
    sample_time = 0.0

    # Create the vectorized environment
    vectorized_env = AsyncVectorEnv(
        [lambda env_id=i: create_environment(env_id, sample_time) for i in range(num_envs)],
        context='spawn'
    )
    

    # Reset the environment
    vectorized_env.reset()

    # Take a step in the environment
    actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]

    while True:
        start_time = time.time()
        observations, rewards, terminated, truncated, infos = vectorized_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]
        print(f"Time taken: {time.time() - start_time}")





    



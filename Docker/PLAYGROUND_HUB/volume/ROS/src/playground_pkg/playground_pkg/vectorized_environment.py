import gymnasium as gym
import rclpy

import numpy as np
from gymnasium.vector import AsyncVectorEnv

from playground_pkg.gym_env_wrapper import GymEnvWrapper
from examples_pkg.autonomous_navigation_example import AutonomousNavigationExampleEnvironment

import time
import json
import os


def create_environment(environment_id: int) -> GymEnvWrapper:

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
        ),
        observation_space=observation_space,
        action_space=action_space,
        reward_range=reward_range
    )




def main():

    config_file_path = "ros_config.json"

    # Load the configuration file
    with open(config_file_path, "r") as f:
        config = json.load(f)

    # Define the number of environments
    n_environments = config["n_environments"]

    # Create the vectorized environment
    vectorized_env = AsyncVectorEnv(
        [lambda env_id=i: create_environment(env_id) for i in range(n_environments)],
        context='spawn'
    )
    

    # Reset the environment
    vectorized_env.reset()

    # Take a step in the environment
    actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]

    while True:
        start_time = time.perf_counter()
        observations, rewards, terminated, truncated, infos = vectorized_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]

        print(f"Time taken: {time.perf_counter() - start_time}")




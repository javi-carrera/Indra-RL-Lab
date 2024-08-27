import numpy as np
from gymnasium.vector import AsyncVectorEnv

from examples_pkg.environments.autonomous_navigation_example_environment import create_environment

import time
import yaml


def main():

    simulated_inference_time = 1.0

    # Load the configuration file
    config_file_path = "config.yml"
    with open(config_file_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    n_environments = config["n_environments"]


    # Create the vectorized environment
    vectorized_env = AsyncVectorEnv(
        [lambda env_id=i: create_environment(env_id) for i in range(n_environments)],
        context='spawn'
    )
    
    vectorized_env.reset()
    actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]

    while True:

        start_time = time.perf_counter()

        # Step the environment
        observations, rewards, terminated, truncated, infos = vectorized_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vectorized_env.num_envs)]

        print(
            f"Observations: {observations}\n"
            f"Rewards: {rewards}\n"
            f"Terminated: {terminated}\n"
            f"Truncated: {truncated}\n"
            f"Infos: {infos}\n"
        )

        # Simulate inference time
        time.sleep(simulated_inference_time)

        print(f"Time taken: {time.perf_counter() - start_time}")




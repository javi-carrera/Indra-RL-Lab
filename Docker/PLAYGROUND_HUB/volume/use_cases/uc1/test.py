# Project: Playground
# File: test.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import yaml

import numpy as np

from use_cases.uc1.environment import UC1Environment
from rl_pkg.utils.communication_monitor import CommunicationMonitor


def test_uc1():

    test_gym_environment()
    # test_vectorized_environment()


def test_gym_environment():

    simulated_inference_time = 0.0

    env = UC1Environment.create_gym_environment(environment_id=0)
    communication_monitor = CommunicationMonitor(env)
    
    env.reset()
    action = np.array([0.0, 0.0])

    while True:
        
        observation, reward, terminated, truncated, info = env.step(action)
        # action = np.random.uniform(-1.0, 1.0, 2)
        action = np.array([1.0, 1.0])

        communication_monitor.display()
        env.render()

        if terminated or truncated:
            # env.reset()
            pass
            

        time.sleep(simulated_inference_time)

    env.close()


def test_vectorized_environment():

    # Load the configuration file
    config_file_path = "config.yml"
    with open(config_file_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    n_environments = config["n_environments"]
    simulated_inference_time = 1.0
    

    # Create the vectorized environment
    vec_env = UC1Environment.create_vectorized_environment(n_environments=n_environments, return_type='gym')

    # vec_env.reset()
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



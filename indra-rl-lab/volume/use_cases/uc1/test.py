# Project: Playground
# File: test.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import yaml

import numpy as np

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor

from use_cases.uc1.environment import UC1Environment
from rl_pkg.utils.communication_monitor import CommunicationMonitor


def test_uc1():

    test_gym_environment()
    #test_vectorized_environment()


def test_gym_environment():

    env = UC1Environment.create_gym_environment(environment_id=0)
    communication_monitor = CommunicationMonitor(env, window_size=1000)
    
    env.reset()
    action = np.array([0.0, 0.0])

    while True:
        
        observation, reward, terminated, truncated, info = env.step(action)
        # action = np.random.uniform(-1.0, 1.0, size=2)
        action = np.array([1.0, 1.0])

        # communication_monitor.display()
        env.render()

        if terminated or truncated:
            env.reset()

    env.close()


def test_vectorized_environment():

    # Load the configuration file
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path))

    n_environments = config['environment']['n_environments']

    # Create the vectorized environment
    vec_env = UC1Environment.create_vectorized_environment(n_environments=n_environments, return_type='gym')

    vec_env.reset()
    actions = [[0.0, 0.0] for _ in range(vec_env.num_envs)]

    while True:

        # Step the environment
        observations, rewards, terminateds, truncateds, infos = vec_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=2) for _ in range(vec_env.num_envs)]


    vec_env.close()

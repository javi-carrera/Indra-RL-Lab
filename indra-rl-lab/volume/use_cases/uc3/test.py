# Project: Indra-RL-Lab
# File: test.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

import numpy as np

from stable_baselines3.common.env_checker import check_env

from rl_pkg.utils.communication_monitor import CommunicationMonitor
from use_cases.uc3 import UC3Environment, UC3RewardWrapper, UC3SelfPlayWrapper


def test_uc3():

    test_gym_environment()
    # test_vectorized_environment()

def test_gym_environment():

    # Environment
    env = UC3Environment.wrap_environment(
        environment_id=0,
        monitor=True,
        wrappers=[
            UC3RewardWrapper,
            UC3SelfPlayWrapper,
        ]
    )
    communication_monitor = CommunicationMonitor(env)
    
    # Test
    env.reset()
    action = np.array([0.0, 0.0, 0.0, 0.0])
    while True:
        observation, reward, terminated, truncated, info = env.step(action)
        # action = np.random.uniform(-1.0, 1.0, size=3)
        action = np.array([0.0, 0.0, 1.0, 0.0])

        # communication_monitor.display()
        # env.render()

        if terminated or truncated:
            env.reset()

    env.close()

def test_vectorized_environment():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path))

    # Create the vectorized environment
    vec_env = UC3Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type='gym'
    )

    # Test
    vec_env.reset()
    actions = [[0.0, 0.0, 0.0, 0.0] for _ in range(vec_env.num_envs)]
    while True:
        observations, rewards, terminateds, truncateds, infos = vec_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vec_env.num_envs)]

    vec_env.close()

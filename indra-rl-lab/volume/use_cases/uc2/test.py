# Project: Indra-RL-Lab
# File: test.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

import numpy as np

from stable_baselines3.common.env_checker import check_env

from rl_pkg.utils.communication_monitor import CommunicationMonitor
from use_cases.uc2 import UC2Environment, UC2ObservationWrapper, UC2RewardWrapper

from rl_pkg.visualizers.esco_visualizer import MainApp

def test_uc2():

    # test_gym_environment()
    test_vectorized_environment()

def test_gym_environment():
    app = MainApp()
    app.start_plotting()

    # Environment
    env = UC2Environment.wrap_environment(
        environment_id=0,
        monitor=True,
        wrappers=[
            UC2ObservationWrapper,
            UC2RewardWrapper,
        ]
    )
    
    # Test
    env.reset()
    action = np.array([0.0, 0.0, 0.0, 0.0])

    try:
        while True:
                observation, reward, terminated, truncated, info = env.step(action)
                app.send_data_to_plot(observation, reward)
                # action = np.random.uniform(-1.0, 1.0, size=3)
                linear_velocity = np.random.normal(0, 2.0)
                angular_velocity = np.random.normal(0, 2.0)
                fire = np.random.choice([0, 1])
                action = np.array([linear_velocity, angular_velocity, fire, 0.0])

                if terminated or truncated:
                    env.reset()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        app.stop_plotting()
        env.close()

def test_vectorized_environment():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path))

    # Create the vectorized environment
    vec_env = UC2Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type="gym",
        monitor=True,
        wrappers=[
            UC2ObservationWrapper,
            UC2RewardWrapper
        ]
    )

    # Test
    vec_env.reset()
    actions = [[0.0, 0.0, 0.0, 0.0] for _ in range(vec_env.num_envs)]
    while True:
        observations, rewards, terminateds, truncateds, infos = vec_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=4) for _ in range(vec_env.num_envs)]

    vec_env.close()

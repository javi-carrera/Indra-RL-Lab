# Project: Playground
# File: test.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import yaml

import numpy as np

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor

from rl_pkg.utils.communication_monitor import CommunicationMonitor
from use_cases.uc3 import UC3Environment

from rl_pkg.visualizers.esco_visualizer import MainApp



def test_uc3():

    test_gym_environment()
    # test_vectorized_environment()


def test_gym_environment():
    app = MainApp()
    app.start_plotting()

    env = UC3Environment.create_gym_environment(environment_id=0)
    communication_monitor = CommunicationMonitor(env)
    
    env.reset()
    action = np.array([0.0, 0.0, 0.0])

    try:
        while True:
            
            observation, reward, terminated, truncated, info = env.step(action)
            app.send_data_to_plot(observation, reward)
            # action = np.random.uniform(-1.0, 1.0, size=3)
            linear_velocity = np.random.normal(0, 2.0)
            angular_velocity = np.random.normal(0, 2.0)
            fire = np.random.choice([0, 1])
            action = np.array([linear_velocity, angular_velocity, fire])

            # env.render()

            if terminated or truncated:
                env.reset()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        app.stop_plotting()
        env.close()


def test_vectorized_environment():

    # Load the configuration file
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path))

    n_environments = config['environment']['n_environments']

    # Create the vectorized environment
    vec_env = UC3Environment.create_vectorized_environment(n_environments=n_environments, return_type='gym')

    vec_env.reset()
    actions = [[0.0, 0.0, 0.0, 0.0] for _ in range(vec_env.num_envs)]

    while True:

        # Step the environment
        observations, rewards, terminateds, truncateds, infos = vec_env.step(actions)
        actions = [np.random.uniform(-1, 1, size=3) for _ in range(vec_env.num_envs)]


    vec_env.close()

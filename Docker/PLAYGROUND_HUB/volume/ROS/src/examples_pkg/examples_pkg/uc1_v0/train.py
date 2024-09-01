# Project: Playground
# File: train.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from stable_baselines3 import PPO, DDPG
import yaml

from examples_pkg.uc1_v0.environment import AutonomousNavigationExampleEnvironment


def train():

    # Load the configuration file
    config_file_path = "config.yml"
    with open(config_file_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    n_environments = config["n_environments"]


    # Create the vectorized environment
    vec_env = AutonomousNavigationExampleEnvironment.create_vectorized_environment(n_environments=n_environments, return_type='stable-baselines')
    vec_env.reset()

    n_timesteps = 5e6

    model = DDPG(
        policy="MlpPolicy",
        env=vec_env,
        verbose=1,
        # tensorboard_log="./tensorboard_logs/",
    )

    # Train the agent
    model.learn(total_timesteps=int(n_timesteps))


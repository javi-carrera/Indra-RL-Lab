# Project: Playground
# File: train.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import torch
from stable_baselines3 import PPO, DDPG
import yaml

from .environment import UC1Environment


def train_uc1():

    # Load the configuration file
    config_file_path = "config.yml"
    with open(config_file_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    n_environments = config["n_environments"]


    # Create the vectorized environment
    vec_env = UC1Environment.create_vectorized_environment(n_environments=n_environments, return_type='stable-baselines')
    vec_env.reset()

    # Create the agent
    normalize = True
    policy = 'MlpPolicy'
    n_steps = 2048
    batch_size = 1024
    gae_lambda = 0.95
    gamma = 0.999
    n_epochs = 20
    ent_coef = 0.0
    learning_rate = 3e-4
    clip_range = 0.18
    policy_kwargs = {
        'net_arch': {
            'pi': [128, 128],
            'vf': [128, 128]
        },
        'activation_fn': torch.nn.LeakyReLU
    }


    # Create the agent
    model = PPO(
        policy=policy,
        env=vec_env,
        verbose=2,
        learning_rate=learning_rate,
        n_steps=n_steps,
        batch_size=batch_size,
        n_epochs=n_epochs,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        ent_coef=ent_coef,
        normalize_advantage=normalize,
        policy_kwargs=policy_kwargs,
    )

    # Print the network architecture
    print(model.policy)

    # Train the agent
    n_timesteps = 5e6

    model.learn(total_timesteps=int(n_timesteps))


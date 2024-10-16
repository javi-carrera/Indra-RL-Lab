# Project: Indra-RL-Lab
# File: train.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

from rl_pipeline import RLTrainer
from use_cases.uc3 import UC3Environment, UC3SelfPlayWrapper, UC3RewardWrapper


def train_uc3():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    # Environment
    vec_env = UC3Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type="stable-baselines",
        monitor=True,
        wrappers=[
            UC3RewardWrapper,
            UC3SelfPlayWrapper
        ]
    )

    # Trainer
    trainer = RLTrainer(
        env=vec_env,
        environment_config=config['environment'],
        training_config=config['training'],
    )
    trainer.run()

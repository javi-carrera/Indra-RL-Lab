# Project: Indra-RL-Lab
# File: train.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

from rl_pipeline.rl_trainer import RLTrainer
from use_cases.uc1 import UC1Environment, UC1ObservationWrapper, UC1RewardWrapper


def train_uc1():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    # Environment
    vec_env = UC1Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type="stable-baselines",
        monitor=True,
        wrappers=[
            UC1ObservationWrapper,
            UC1RewardWrapper
        ]
    )

    # Trainer
    trainer = RLTrainer(env=vec_env, config=config)
    trainer.run()

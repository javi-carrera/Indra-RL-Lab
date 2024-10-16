# Project: Indra-RL-Lab
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import logging
import yaml

from pathlib import Path
from typing import Dict

from rl_pipeline.algorithm_registry import ALGORITHMS, get_algorithm_kwargs


class RLDeployer:

    def __init__(
        self,
        env,
        environment_config: Dict,
        deployment_config: Dict,
    ):

        # Configuration
        self.deployment_config = deployment_config

        environment_id = environment_config['id']
        experiment_name = deployment_config['experiment_name']
        checkpoint = deployment_config['checkpoint']

        experiments_path = Path('experiments')
        log_dir = experiments_path / f"{environment_id}/{experiment_name}"
        config_dir = log_dir / 'config'
        checkpoint_dir = log_dir / 'checkpoints'

        experiment_training_config = yaml.safe_load(open(config_dir / 'training_config.yml', 'r'))

        # Algorithm
        self.env = env
        algorithm = experiment_training_config['algorithm']
        algorithm_kwargs = get_algorithm_kwargs(
            env=env,
            algorithm=algorithm
        )

        pretrained_model_path = checkpoint_dir / checkpoint
        self.model = ALGORITHMS[algorithm].load(pretrained_model_path, **algorithm_kwargs)

        # Logger
        self.logger = logging.getLogger(f"{experiment_name}_rl_deployer")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)


    def deploy(self):

        observations = self.env.reset()

        while True:
            action, _ = self.model.predict(observations, deterministic=True)
            observations, rewards, dones, info = self.env.step(action)

        

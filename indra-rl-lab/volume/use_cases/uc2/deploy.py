# Project: Indra-RL-Lab
# File: deploy.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

from rl_pipeline import RLDeployer
from use_cases.uc2 import UC2Environment, UC2ObservationWrapper, UC2RewardWrapper


def deploy_uc2():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    # Environment
    vec_env = UC2Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type="stable-baselines",
        monitor=True,
        wrappers=[
            UC2ObservationWrapper,
            UC2RewardWrapper
        ]
    )
    
    # Deployer
    deployer = RLDeployer(
        env=vec_env,
        environment_config=config['environment'],
        deployment_config=config['deployment'],
    )
    deployer.deploy()
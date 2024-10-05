import yaml

from rl_pipeline import RLDeployer
from use_cases.uc1 import UC1Environment, UC1ObservationWrapper, UC1RewardWrapper


def deploy_uc1():

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
    
    # Deployer
    deployer = RLDeployer(
        env=vec_env,
        environment_config=config['environment'],
        deployment_config=config['deployment'],
    )
    deployer.deploy()
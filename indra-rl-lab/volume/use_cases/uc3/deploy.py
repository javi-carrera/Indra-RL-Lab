import yaml

from rl_pipeline import RLDeployer
from use_cases.uc3 import UC3Environment, UC3RewardWrapper, UC3SelfPlayWrapper


def deploy_uc3():

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
            UC3SelfPlayWrapper,
        ]
    )
    
    # Deployer
    deployer = RLDeployer(
        env=vec_env,
        environment_config=config['environment'],
        deployment_config=config['deployment'],
    )
    deployer.deploy()
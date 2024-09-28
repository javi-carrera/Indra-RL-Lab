from pathlib import Path
import yaml
import gymnasium as gym
from torch.backends.cudnn import deterministic
import datetime
from use_cases.uc1 import UC1Environment

from rl_pipeline.utils.algorithm_registry import ALGORITHMS, get_algorithm_kwargs


def deploy_uc1():

    # Load the configuration file
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    # Define the experiment name and log directory
    experiment_name = config['deployment']['experiment_name']
    checkpoint = config['deployment']['checkpoint']
    pretrained_model_path = Path('experiments') / config['environment']['id'] / config['deployment']['algorithm'] / experiment_name / checkpoint

    # Create the vectorized environment
    vec_env = UC1Environment.create_vectorized_environment(
        # n_environments=config['environment']['n_environments'],
        n_environments=1,
        return_type="stable-baselines",
        monitor=True
    )
    
    
    algorithm_kwargs = get_algorithm_kwargs(
        env=vec_env,
        algorithm=config['training']['algorithm']
    )

    algorithm = ALGORITHMS[config['training']['algorithm']].load(pretrained_model_path, **algorithm_kwargs)

    observations = vec_env.reset()
    reward_sum = 0

    while True:
        action, _ = algorithm.predict(observations, deterministic=True)
        observations, rewards, dones, info = vec_env.step(action)
        # reward_sum += reward
        # env.render()

    # print(f"****** Total reward: {reward_sum}")



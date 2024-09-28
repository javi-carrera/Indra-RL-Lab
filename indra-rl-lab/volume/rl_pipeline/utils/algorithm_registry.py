from pathlib import Path
import yaml

import stable_baselines3 as sb3
import torch

from rl_pipeline.models.feature_extractors.resnet_extractor import ResnetMLP


ALGORITHMS = {
    'PPO': sb3.PPO,
    'SAC': sb3.SAC,
    'DQN': sb3.DQN,
    'DDPG': sb3.DDPG,
    'TD3': sb3.TD3,
}

ALGORITHM_CONFIG_PATHS = {
    'PPO': 'rl_pipeline/configs/ppo_config.yml',
}

ACTIVATION_FUNCTIONS = {
    'ReLU': torch.nn.ReLU,
    'LeakyReLU': torch.nn.LeakyReLU,
    'ELU': torch.nn.ELU,
    'Tanh': torch.nn.Tanh,
    'Sigmoid': torch.nn.Sigmoid,
    'Softmax': torch.nn.Softmax,
}

AVAILABLE_MODELS = {
    'ResnetMLP': ResnetMLP,
}


def get_algorithm_kwargs(env, algorithm: str, log_dir: Path=None):
    
    # Check if the algorithm is available
    if algorithm not in ALGORITHMS:
        raise ValueError(f'Algorithm {algorithm} not available. Available algorithms: {list(ALGORITHMS.keys())}')

    # Load the algorithm config
    algorithm_kwargs = yaml.safe_load(open(ALGORITHM_CONFIG_PATHS[algorithm]))

    # Get the activation function callable
    activation_fn = ACTIVATION_FUNCTIONS[algorithm_kwargs['policy_kwargs']['activation_fn']]

    # Get the feature extractor callable
    feature_extractor = AVAILABLE_MODELS[algorithm_kwargs['policy_kwargs']['features_extractor_class']]

    # Update the config dict
    algorithm_kwargs['env'] = env
    algorithm_kwargs['tensorboard_log'] = str(log_dir / 'tensorboard') if log_dir else None

    algorithm_kwargs['policy_kwargs']['activation_fn'] = activation_fn
    algorithm_kwargs['policy_kwargs']['features_extractor_class'] = feature_extractor


    return algorithm_kwargs

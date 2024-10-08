# Project: Indra-RL-Lab
# File: algorithm_registry.py
# Authors: Nicolás Rozado, Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

from pathlib import Path
from typing import Dict

import stable_baselines3 as sb3
import torch

from rl_pipeline.models.feature_extractors import ResnetMLP
from rl_pipeline.schedulers import exponential_schedule, cosine_schedule


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

MODELS = {
    'ResnetMLP': ResnetMLP,
}

ACTIVATION_FUNCTIONS = {
    'ReLU': torch.nn.ReLU,
    'LeakyReLU': torch.nn.LeakyReLU,
    'ELU': torch.nn.ELU,
    'Tanh': torch.nn.Tanh,
    'Sigmoid': torch.nn.Sigmoid,
    'Softmax': torch.nn.Softmax,
}

SCHEDULERS = {
    'exponential': exponential_schedule,
    'cosine': cosine_schedule,
}


def get_algorithm_config(algorithm: str) -> Dict:
    return yaml.safe_load(open(ALGORITHM_CONFIG_PATHS[algorithm]))


def get_algorithm_kwargs(env, algorithm: str, log_dir: Path=None) -> Dict:
    
    if algorithm not in ALGORITHMS:
        raise ValueError(f'Algorithm {algorithm} not available. Available algorithms: {list(ALGORITHMS.keys())}')

    algorithm_kwargs = get_algorithm_config(algorithm)

    activation_fn = ACTIVATION_FUNCTIONS[algorithm_kwargs['policy_kwargs']['activation_fn']]
    feature_extractor = MODELS.get(algorithm_kwargs['policy_kwargs']['features_extractor_class'])

    algorithm_kwargs['env'] = env
    algorithm_kwargs['tensorboard_log'] = str(log_dir / 'tensorboard') if log_dir else None
    algorithm_kwargs['policy_kwargs']['activation_fn'] = activation_fn
    algorithm_kwargs['policy_kwargs']['features_extractor_class'] = feature_extractor
    algorithm_kwargs['learning_rate'] = exponential_schedule(
        algorithm_kwargs['learning_rate'],
        1e-5,
    )

    return algorithm_kwargs


from typing import Dict
from stable_baselines3 import PPO, DQN, SAC, DDPG, TD3
import torch
from pathlib import Path

from rl_pipeline.utils.models_registry import AVAILABLE_MODELS


def get_ppo_params(env, config: Dict, log_dir: Path):
    policy_kwargs = None
    if config.get('architecture', None) is not None:
        feature_extractor = AVAILABLE_MODELS.get(config['architecture'].get('features_extractor_class', None), None)

        policy_kwargs = {
            'activation_fn': getattr(torch.nn, config['architecture'].get('activation_fn', 'ReLU')),
            'net_arch': config['architecture'].get('net_arch', None),
            'share_features_extractor': config['architecture'].get('share_features_extractor', False)
        }

        if feature_extractor is not None:
            fe_dict = {
                'features_extractor_class': feature_extractor,
                'features_extractor_kwargs': config['architecture'].get('features_extractor_kwargs', None),
            }
            policy_kwargs.update(fe_dict)

    ppo_params = {
        'policy': config['algorithm_parameters'].get('policy', 'MlpPolicy'),
        'env': env,
        'policy_kwargs': policy_kwargs,
        'learning_rate': config['algorithm_parameters'].get('learning_rate', 3e-4),
        'n_steps': config['algorithm_parameters'].get('n_steps', 2048),
        'batch_size': config['algorithm_parameters'].get('batch_size', 64),
        'n_epochs': config['algorithm_parameters'].get('n_epochs', 10),
        'gamma': config['algorithm_parameters'].get('gamma', 0.99),
        'gae_lambda': config['algorithm_parameters'].get('gae_lambda', 0.95),
        'clip_range': config['algorithm_parameters'].get('clip_range', 0.2),
        'ent_coef': config['algorithm_parameters'].get('ent_coef', 0.0),
        'vf_coef': config['algorithm_parameters'].get('vf_coef', 0.5),
        'max_grad_norm': config['algorithm_parameters'].get('max_grad_norm', 0.5),
        'seed': config['training'].get('seed', None),
        'device': config['training'].get('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
        'tensorboard_log': log_dir
    }

    return ppo_params


def get_sac_params(env, config: Dict, log_dir: Path):
    policy_kwargs = None
    if config.get('architecture', None) is not None:
        feature_extractor = AVAILABLE_MODELS.get(config['architecture'].get('features_extractor_class', None), None)

        policy_kwargs = {
            'activation_fn': getattr(torch.nn, config['architecture'].get('activation_fn', 'ReLU')),
            'net_arch': config['architecture'].get('net_arch', None),
            'share_features_extractor': config['architecture'].get('share_features_extractor', False)
        }

        if feature_extractor is not None:
            fe_dict = {
                'features_extractor_class': feature_extractor,
                'features_extractor_kwargs': config['architecture'].get('features_extractor_kwargs', None),
            }
            policy_kwargs.update(fe_dict)

    sac_params = {
        'policy': config['algorithm_parameters'].get('policy', 'MlpPolicy'),
        'policy_kwargs': policy_kwargs,
        'env': env,
        'learning_rate': config['algorithm_parameters'].get('learning_rate', 3e-4),
        'buffer_size': config['algorithm_parameters'].get('buffer_size', 1_000_000),
        'learning_starts': config['algorithm_parameters'].get('learning_starts', 100),
        'batch_size': config['algorithm_parameters'].get('batch_size', 256),
        'ent_coef': config['algorithm_parameters'].get('ent_coef', 'auto'),
        'gamma': config['algorithm_parameters'].get('gamma', 0.99),
        'tau': config['algorithm_parameters'].get('tau', 0.005),
        'train_freq': config['algorithm_parameters'].get('train_freq', 1),
        'gradient_steps': config['algorithm_parameters'].get('gradient_steps', 1),
        'target_update_interval': config['algorithm_parameters'].get('target_update_interval', 1),
        'target_entropy': config['algorithm_parameters'].get('target_entropy', 'auto'),
        'seed': config['training'].get('seed', None),
        'device': config['training'].get('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
        'tensorboard_log': log_dir
    }

    return sac_params


def get_dqn_params(env, config: Dict, log_dir: Path):
    policy_kwargs = None
    if config.get('architecture', None) is not None:
        feature_extractor = AVAILABLE_MODELS.get(config['architecture'].get('features_extractor_class', None), None)

        policy_kwargs = {
            'activation_fn': getattr(torch.nn, config['architecture'].get('activation_fn', 'ReLU')),
            'net_arch': config['architecture'].get('net_arch', None),
            'share_features_extractor': config['architecture'].get('share_features_extractor', False)
        }

        if feature_extractor is not None:
            fe_dict = {
                'features_extractor_class': feature_extractor,
                'features_extractor_kwargs': config['architecture'].get('features_extractor_kwargs', None),
            }
            policy_kwargs.update(fe_dict)

    dqn_params = {
        'policy': config['algorithm_parameters'].get('policy', 'MlpPolicy'),
        'env': env,
        'policy_kwargs': policy_kwargs,
        'learning_rate': config['algorithm_parameters'].get('learning_rate', 1e-4),
        'buffer_size': config['algorithm_parameters'].get('buffer_size', 1_000_000),
        'learning_starts': config['algorithm_parameters'].get('learning_starts', 100),
        'batch_size': config['algorithm_parameters'].get('batch_size', 32),
        'gamma': config['algorithm_parameters'].get('gamma', 0.99),
        'tau': config['algorithm_parameters'].get('tau', 1.0),
        'train_freq': config['algorithm_parameters'].get('train_freq', 4),
        'gradient_steps': config['algorithm_parameters'].get('gradient_steps', 1),
        'exploration_fraction': config['algorithm_parameters'].get('exploration_fraction', 0.1),
        'exploration_initial_eps': config['algorithm_parameters'].get('exploration_initial_eps', 1.0),
        'exploration_final_eps': config['algorithm_parameters'].get('exploration_final_eps', 0.05),
        'max_grad_norm': config['algorithm_parameters'].get('max_grad_norm', 10),
        'target_update_interval': config['algorithm_parameters'].get('target_update_interval', 10000),
        'seed': config['training'].get('seed', None),
        'device': config['training'].get('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
        'tensorboard_log': log_dir
    }

    return dqn_params


def get_ddpg_params(env, config: Dict, log_dir: Path):
    policy_kwargs = None
    if config.get('architecture', None) is not None:
        feature_extractor = AVAILABLE_MODELS.get(config['architecture'].get('features_extractor_class', None), None)

        policy_kwargs = {
            'activation_fn': getattr(torch.nn, config['architecture'].get('activation_fn', 'ReLU')),
            'net_arch': config['architecture'].get('net_arch', None),
            'share_features_extractor': config['architecture'].get('share_features_extractor', False)
        }

        if feature_extractor is not None:
            fe_dict = {
                'features_extractor_class': feature_extractor,
                'features_extractor_kwargs': config['architecture'].get('features_extractor_kwargs', None),
            }
            policy_kwargs.update(fe_dict)

    ddpg_params = {
        'policy': config['algorithm_parameters'].get('policy', 'MlpPolicy'),
        'env': env,
        'policy_kwargs': policy_kwargs,
        'learning_rate': config['algorithm_parameters'].get('learning_rate', 1e-4),
        'buffer_size': config['algorithm_parameters'].get('buffer_size', 1_000_000),
        'learning_starts': config['algorithm_parameters'].get('learning_starts', 100),
        'batch_size': config['algorithm_parameters'].get('batch_size', 256),
        'gamma': config['algorithm_parameters'].get('gamma', 0.99),
        'tau': config['algorithm_parameters'].get('tau', 0.005),
        'train_freq': config['algorithm_parameters'].get('train_freq', 1),
        'gradient_steps': config['algorithm_parameters'].get('gradient_steps', 1),
        'seed': config['training'].get('seed', None),
        'device': config['training'].get('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
        'tensorboard_log': log_dir
    }

    return ddpg_params


def get_td3_params(env, config: Dict, log_dir: Path):
    policy_kwargs = None
    if config.get('architecture', None) is not None:
        feature_extractor = AVAILABLE_MODELS.get(config['architecture'].get('features_extractor_class', None), None)

        policy_kwargs = {
            'activation_fn': getattr(torch.nn, config['architecture'].get('activation_fn', 'ReLU')),
            'net_arch': config['architecture'].get('net_arch', None),
            'share_features_extractor': config['architecture'].get('share_features_extractor', False)
        }

        if feature_extractor is not None:
            fe_dict = {
                'features_extractor_class': feature_extractor,
                'features_extractor_kwargs': config['architecture'].get('features_extractor_kwargs', None),
            }
            policy_kwargs.update(fe_dict)

    td3_params = {
        'policy': config['algorithm_parameters'].get('policy', 'MlpPolicy'),
        'env': env,
        'policy_kwargs': policy_kwargs,
        'learning_rate': config['algorithm_parameters'].get('learning_rate', 1e-3),
        'buffer_size': config['algorithm_parameters'].get('buffer_size', 1_000_000),
        'learning_starts': config['algorithm_parameters'].get('learning_starts', 100),
        'batch_size': config['algorithm_parameters'].get('batch_size', 256),
        'gamma': config['algorithm_parameters'].get('gamma', 0.99),
        'tau': config['algorithm_parameters'].get('tau', 0.005),
        'train_freq': config['algorithm_parameters'].get('train_freq', 1),
        'gradient_steps': config['algorithm_parameters'].get('gradient_steps', 1),
        'policy_delay': config['algorithm_parameters'].get('policy_delay', 2),
        'target_policy_noise': config['algorithm_parameters'].get('target_policy_noise', 0.2),
        'target_noise_clip': config['algorithm_parameters'].get('target_noise_clip', 0.5),
        'seed': config['training'].get('seed', None),
        'device': config['training'].get('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
        'tensorboard_log': log_dir
    }

    return td3_params

AVAILABLE_ALGORITHMS = {
    'ppo': (PPO, get_ppo_params),
    'sac': (SAC, get_sac_params),
    'dqn': (DQN, get_dqn_params),
    'ddpg': (DDPG, get_ddpg_params),
    'td3': (TD3, get_td3_params),
    }

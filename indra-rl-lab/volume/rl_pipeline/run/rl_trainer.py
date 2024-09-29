import datetime
import logging
from pathlib import Path
from typing import Dict

import wandb

from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
from wandb.integration.sb3 import WandbCallback

from rl_pipeline.utils.algorithm_registry import ALGORITHMS, get_algorithm_config, get_algorithm_kwargs
import yaml


class RLTrainer:

    def __init__(
            self,
            env,
            config: Dict,
        ):

        # Configuration
        self.environment_config = config['environment']
        self.training_config = config['training']
        self.logging_config = self.training_config['logging']

        environment_id = self.environment_config['id']
        experiment_name = f"{self.training_config['experiment_name']}"
        algorithm = self.training_config['algorithm']
        date = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        experiments_path = Path('experiments')
        log_dir = experiments_path / f"{environment_id}/{algorithm}/{experiment_name}_{date}"
        log_dir.mkdir(parents=True, exist_ok=True)
        checkpoint_dir = log_dir / 'checkpoints'

        # Algorithm
        algorithm_kwargs = get_algorithm_kwargs(
            env=env,
            algorithm=algorithm,
            log_dir=log_dir
        )

        pretrained_model_config = self.training_config['pretrained_model']
        if pretrained_model_config['use_pretrained_model']:
            pretrained_experiment_name = pretrained_model_config['experiment_name']
            checkpoint = pretrained_model_config['checkpoint']
            pretrained_model_path = experiments_path / f"{environment_id}/{algorithm}/{pretrained_experiment_name}/{checkpoint}"
            self.algorithm = ALGORITHMS[algorithm].load(pretrained_model_path, **algorithm_kwargs)

        else:
            self.algorithm = ALGORITHMS[algorithm](**algorithm_kwargs, verbose=self.logging_config['verbose'])


        # Saving
        # yaml.safe_dump(config, open(log_dir / 'config.yml', 'w'))
        yaml.safe_dump(self.environment_config, open(log_dir / 'environment_config.yml', 'w'), sort_keys=False)
        yaml.safe_dump(self.training_config, open(log_dir / 'training_config.yml', 'w'), sort_keys=False)

        algorithm_config = get_algorithm_config(algorithm)
        yaml.safe_dump(algorithm_config, open(log_dir / 'algorithm_config.yml', 'w'), sort_keys=False)

        architecture = str(self.algorithm.policy)
        with open(log_dir / 'architecture.txt', 'w') as f:
            f.write(architecture)           
        
        # Callbacks
        self.callback_list = []
        log_freq = self.logging_config['log_freq']

        checkpoint_callback = CheckpointCallback(
            save_freq=log_freq,
            save_path=str(checkpoint_dir),
            name_prefix='checkpoint',
            verbose=self.logging_config['verbose']
        )

        self.callback_list.append(checkpoint_callback)

        # Logging
        if self.logging_config['use_wandb']:

            self.wandb_run = wandb.init(
                # dir=log_dir,
                config=config,
                project=environment_id,
                entity=self.logging_config['wandb_entity'],
                group=algorithm,
                name=f"{experiment_name}_{date}",
                mode='online',
                sync_tensorboard=True,
                monitor_gym=True,
                save_code=False,
            )

            wandb.save(log_dir / 'architecture.txt', base_path=log_dir)
            wandb.save(log_dir / 'training_config.yml', base_path=log_dir)
            wandb.save(log_dir / 'algorithm_config.yml', base_path=log_dir)

            wandb_callback = WandbCallback(
                verbose=self.logging_config['verbose'],
                model_save_path=str(checkpoint_dir),
                model_save_freq=log_freq,
                gradient_save_freq=log_freq,
                log='all'
            )

            self.callback_list.append(wandb_callback)

        # Logger
        self.logger = logging.getLogger(f"{experiment_name}_rl_trainer")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)


    def run(self):

        self.algorithm.learn(
            total_timesteps=self.training_config['total_timesteps'],
            callback=CallbackList(self.callback_list),
            progress_bar=self.logging_config['progress_bar']
        )

        if self.logging_config['use_wandb']:
            self.wandb_run.finish()

        self.logger.info(f"Training done for {self.training_config['total_timesteps']} timesteps.")

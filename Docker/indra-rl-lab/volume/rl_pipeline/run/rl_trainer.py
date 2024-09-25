import logging
import wandb
from wandb.integration.sb3 import WandbCallback
from rl_pipeline.utils.algorithm_registry import AVAILABLE_ALGORITHMS, ALGORITH_KWARGS
from rl_pipeline.utils.callbacks import SaveDataCallback
from typing import Dict
from pathlib import Path

import json

from stable_baselines3 import PPO

from pprint import pprint


class RLTrainer:

    def __init__(
            self,
            env,
            config: Dict,
            experiment_name: str,
            log_dir: Path,
        ):

        # Save the configuration and log directory
        self._config = config
        self._log_dir = log_dir

        # Check if the algorithm is available
        if self._config['training']['algorithm'] not in AVAILABLE_ALGORITHMS.keys():
            raise ValueError(f"Algorithm {self._config['training']['algorithm']} not available. Choose from {AVAILABLE_ALGORITHMS.keys()}")        

        # Create the algorithm or load the pretrained model
        algorithm_kwargs = ALGORITH_KWARGS[self._config['training']['algorithm']]
        algorithm_kwargs.update({'env': env})
        algorithm_kwargs.update({'tensorboard_log': str(self._log_dir / 'tensorboard')})

        pprint(algorithm_kwargs)

        pretrained_model_path = self._config['training']['pretrained_model_path']

        if pretrained_model_path:
            self.algorithm = AVAILABLE_ALGORITHMS[self._config['training']['algorithm']].load(pretrained_model_path, **algorithm_kwargs)
        else:
            self.algorithm = AVAILABLE_ALGORITHMS[self._config['training']['algorithm']](**algorithm_kwargs, verbose=self._config['training']['verbose'])

        # Logging initialization
        self.logger = logging.getLogger(f"{self._config['training']['experiment_name']}_rl_trainer")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

        # WandB initialization
        if self._config['training']['logging']['use_wandb']:

            self._wandb_run = wandb.init(
                project=self._config['training']['logging']['wandb_project'],
                entity=self._config['training']['logging']['wandb_entity'],
                name=experiment_name,
                group=self._config['environment']['id'],
                config=self._config['training'],
                sync_tensorboard=True,
                monitor_gym=True,
                save_code=False,
                mode='online'
            )

    def run(self, eval_env):

        # Calculate the log frequency
        log_freq = max(1, int(self._config['training']['total_timesteps'] // self._config['training']['num_envs'] / self._config['training']['logging']['log_points']))

        # Create the callback
        if self._config['training']['logging']['use_wandb']:

            # Use the WandbCallback
            info_saver_callback = WandbCallback(
                # verbose=self._config['training']['verbose'],
                model_save_path=str(self._log_dir),
                model_save_freq=log_freq,
                gradient_save_freq=log_freq,
                log='all'
            )

        else:

            # Use the custom SaveDataCallback
            info_saver_callback = SaveDataCallback(
                save_freq=log_freq,
                save_path=str(self._log_dir)
            )

        # Save the configuration
        architecture = str(self.algorithm.policy)
        with open(self._log_dir / 'architecture.txt', 'w') as f:
            f.write(architecture)

        # Train the model
        self.algorithm.learn(total_timesteps=self._config['training']['total_timesteps'], callback=info_saver_callback)

        # Save the model locally
        self.algorithm.save(path=self._log_dir / 'last_model.zip')

        # Save the model to wandb and finish the run
        if self._config['training']['logging']['use_wandb']:
            wandb.save(self._log_dir / 'architecture.txt', base_path=self._log_dir)
            self._wandb_run.finish()


        self.logger.info(f"Training done for {self._log_dir} experiment!")

import logging
import wandb
from wandb.integration.sb3 import WandbCallback
from rl_pipeline.utils.algorithm_registry import ALGORITHMS, get_algorithm_kwargs
from rl_pipeline.utils.callbacks import SaveDataCallback
from typing import Dict
from pathlib import Path


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

        # Create the algorithm or load the pretrained model
        algorithm_kwargs = get_algorithm_kwargs(
            env=env,
            algorithm=self._config['training']['algorithm'],
            log_dir=self._log_dir
        )


        if self._config['training']['pretrained_model']['use_pretrained_model']:
            experiment_name = self._config['training']['pretrained_model']['experiment_name']
            checkpoint = self._config['training']['pretrained_model']['checkpoint']
            pretrained_model_path = Path('experiments') / self._config['environment']['id'] / self._config['training']['algorithm'] / experiment_name / checkpoint
            self.algorithm = ALGORITHMS[self._config['training']['algorithm']].load(pretrained_model_path, **algorithm_kwargs)
        else:
            self.algorithm = ALGORITHMS[self._config['training']['algorithm']](**algorithm_kwargs, verbose=self._config['training']['verbose'])

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
        log_freq = self._config['training']['logging']['log_freq']

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

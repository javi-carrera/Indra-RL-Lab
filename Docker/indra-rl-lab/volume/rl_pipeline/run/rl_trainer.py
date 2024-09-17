import wandb
from wandb.integration.sb3 import WandbCallback
from rl_pipeline.utils.algorithm_registry import AVAILABLE_ALGORITHMS
from rl_pipeline.utils.callbacks import SaveDataCallback
from typing import Dict
from pathlib import Path

import torch


class RLTrainer:
    def __init__(self, env, config: Dict, log_dir: Path, exp_name:str, wandb_group:str, pretrained_model=None):
        self._config = config
        self._log_dir = log_dir

        if self._config.get('algorithm') not in AVAILABLE_ALGORITHMS.keys():
            raise ValueError(f"Algorithm {self._config.get('algorithm')} not available. "
                             f"Choose from {AVAILABLE_ALGORITHMS.keys()}")

        self._params = AVAILABLE_ALGORITHMS[self._config.get('algorithm')][1](env, self._config, self._log_dir)

        self._pretrained_model = pretrained_model

        if self._config['use_wandb']:
            self._wandb_run = wandb.init(
                project="sb3",
                name=exp_name,
                group=wandb_group,
                config=config,
                sync_tensorboard=True,
                monitor_gym=True,
                save_code=False,
            )

    def run(self, eval_env, logger=None):

        log_freq = max(1, int(self._config['training'].get('total_timesteps') // self._config['training'].get('num_envs') /
            self._config['training'].get('log_points')))

        if self._config['use_wandb']:
            info_saver_callback = WandbCallback(
                model_save_path=str(self._log_dir),
                model_save_freq=log_freq,
                gradient_save_freq=log_freq,
                log='all')
        else:
            info_saver_callback = SaveDataCallback(save_freq=log_freq, save_path=str(self._log_dir))

        if self._pretrained_model is None:
            model = AVAILABLE_ALGORITHMS[
                self._config.get('algorithm')][0](**self._params, verbose=self._config['training'].get('verbose'))
        else:
            model = AVAILABLE_ALGORITHMS[self._config.get('algorithm')][0].load(path=self._pretrained_model, **self._params)

        model.learn(total_timesteps=self._config['training'].get('total_timesteps'), callback=info_saver_callback)

        model.save(path=self._log_dir / 'last_model.zip')

        if self._config['use_wandb']:
            self._wandb_run.finish()

        if logger:
            logger.info(f"Training done for {self._log_dir} experiment!")

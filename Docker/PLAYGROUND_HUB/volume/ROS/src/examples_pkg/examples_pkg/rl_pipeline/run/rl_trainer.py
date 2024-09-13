import wandb
from wandb.integration.sb3 import WandbCallback
from examples_pkg.rl_pipeline.utils.algorithm_registry import AVAILABLE_ALGORITHMS
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

        info_saver_callback = WandbCallback(model_save_path=str(self._log_dir),
                                            model_save_freq=log_freq,
                                            gradient_save_freq=log_freq,
                                            log='all')

        if self._pretrained_model is None:
            model = AVAILABLE_ALGORITHMS[
                self._config.get('algorithm')][0](**self._params, verbose=self._config['training'].get('verbose'))
        else:
            model = AVAILABLE_ALGORITHMS[self._config.get('algorithm')][0].load(path=self._pretrained_model, **self._params)

        if self._config.get('training').get('torch_compile', False):
            torch.set_float32_matmul_precision('high')
            model.policy = torch.compile(model.policy)

        model.learn(total_timesteps=self._config['training'].get('total_timesteps'),
                    callback=info_saver_callback)

        self._wandb_run.finish()

        if logger:
            logger.info(f"Training done for {self._log_dir} experiment!")

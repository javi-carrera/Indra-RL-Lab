import datetime
import logging
import shutil
import yaml

from pathlib import Path
from typing import Dict

import wandb

from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, EvalCallback
from wandb.integration.sb3 import WandbCallback

from rl_pipeline.algorithm_registry import ALGORITHMS, get_algorithm_config, get_algorithm_kwargs


class RLTrainer:

    def __init__(
        self,
        env,
        environment_config: Dict,
        training_config: Dict,
    ):

        # Configuration
        self.training_config = training_config
        self.logging_config = training_config['logging']
        self.evaluation_config = training_config['evaluation']

        environment_id = environment_config['id']
        experiment_name = f"{training_config['experiment_name']}"
        date = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        experiments_path = Path('experiments')
        use_case_path = Path('use_cases') / environment_id
        log_dir = experiments_path / f"{environment_id}/{experiment_name}_{date}"
        config_dir = log_dir / 'config'
        checkpoint_dir = log_dir / 'checkpoints'

        log_dir.mkdir(parents=True, exist_ok=True)
        config_dir.mkdir(parents=True, exist_ok=True)

        # Algorithm
        algorithm = training_config['algorithm']
        algorithm_config = get_algorithm_config(algorithm)
        algorithm_kwargs = get_algorithm_kwargs(
            env=env,
            algorithm=algorithm,
            log_dir=log_dir
        )

        pretrained_model_config = training_config['pretrained_model']
        if pretrained_model_config['use_pretrained_model']:
            pretrained_experiment_name = pretrained_model_config['experiment_name']
            checkpoint = pretrained_model_config['checkpoint']
            pretrained_model_path = experiments_path / f"{environment_id}/{pretrained_experiment_name}/checkpoints/{checkpoint}"
            self.algorithm = ALGORITHMS[algorithm].load(pretrained_model_path, **algorithm_kwargs)

        else:
            self.algorithm = ALGORITHMS[algorithm](**algorithm_kwargs, verbose=self.logging_config['verbose'])


        # Saving
        yaml.safe_dump(environment_config, open(config_dir / 'environment_config.yml', 'w'), sort_keys=False)
        yaml.safe_dump(training_config, open(config_dir / 'training_config.yml', 'w'), sort_keys=False)
        yaml.safe_dump(algorithm_config, open(config_dir / 'algorithm_config.yml', 'w'), sort_keys=False)
        shutil.copy(use_case_path / 'environment.py', log_dir / 'environment.py')
        # shutil.copy(use_case_path / 'observation_wrapper.py', log_dir / 'observation_wrapper.py')
        shutil.copy(use_case_path / 'reward_wrapper.py', log_dir / 'reward_wrapper.py')
        with open(log_dir / 'architecture.txt', 'w') as f:
            f.write(str(self.algorithm.policy))
        
        # Callbacks
        self.callback_list = []
        checkpoint_freq = training_config['total_timesteps'] / self.logging_config['n_checkpoints'] // environment_config['n_environments']

        checkpoint_callback = CheckpointCallback(
            save_freq=checkpoint_freq,
            save_path=str(checkpoint_dir),
            name_prefix='checkpoint',
            verbose=self.logging_config['verbose']
        )

        self.callback_list.append(checkpoint_callback)

        evaluation_callback = EvalCallback(
            eval_env=env,
            n_eval_episodes=self.evaluation_config['eval_episodes'],
            eval_freq=self.evaluation_config['eval_freq'],
            best_model_save_path=str(checkpoint_dir),
            log_path=log_dir,
            verbose=self.logging_config['verbose']
        )

        self.callback_list.append(evaluation_callback)

        # Logging
        if self.logging_config['use_wandb']:

            self.wandb_run = wandb.init(
                # dir=log_dir,
                config={**environment_config, **training_config},
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
            wandb.save(config_dir / 'environment_config.yml', base_path=log_dir)
            wandb.save(config_dir / 'training_config.yml', base_path=log_dir)
            wandb.save(config_dir / 'algorithm_config.yml', base_path=log_dir)
            wandb.save(log_dir / 'environment.py', base_path=log_dir)
            wandb.save(log_dir / 'observation_wrapper.py', base_path=log_dir)
            wandb.save(log_dir / 'reward_wrapper.py', base_path=log_dir)

            wandb_callback = WandbCallback(
                verbose=self.logging_config['verbose'],
                model_save_path=str(checkpoint_dir),
                model_save_freq=checkpoint_freq,
                gradient_save_freq=checkpoint_freq,
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

        total_timesteps = self.training_config['total_timesteps']

        self.algorithm.learn(
            total_timesteps=total_timesteps,
            callback=CallbackList(self.callback_list),
        )

        if self.logging_config['use_wandb']:
            self.wandb_run.finish()

        self.logger.info(f"Training done for {total_timesteps} timesteps.")

        

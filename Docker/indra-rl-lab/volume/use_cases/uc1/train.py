# Project: Playground
# File: train.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml, datetime
from pathlib import Path
from stable_baselines3.common.vec_env import VecVideoRecorder

from rl_pipeline.run.rl_trainer import RLTrainer
from use_cases.uc1 import UC1Environment


def train_uc1():

    # Load the configuration file
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    # Define the experiment name and log directory
    experiment_name = f"{config['training']['experiment_name']}_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    log_dir = Path('experiments/') / config['environment']['id'] / config['training']['algorithm'] / experiment_name

    # Create the vectorized environment
    vec_env = UC1Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type="stable-baselines",
        monitor=True
    )
    
    # Add video wrapper
    if config['environment']['render']['use_video_wrapper']:

        vec_env = VecVideoRecorder(
            vec_env,
            video_folder=f"{str(log_dir / 'videos')}",
            record_video_trigger=lambda x: x % config['environment']['render']['video_trigger'] == 0,
            video_length=config['environment']['render']['video_length']
        )

    # Create the RL trainer
    trainer = RLTrainer(
        env=vec_env,
        config=config,
        experiment_name=experiment_name,
        log_dir=log_dir
    )
    
    # vec_env.reset()
    trainer.run(eval_env=None)

# Project: Playground
# File: train.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml, datetime
from pathlib import Path
from stable_baselines3.common.vec_env import VecVideoRecorder

from rl_pipeline.run.rl_trainer import RLTrainer
from use_cases.uc2 import UseCase2Environment




def train_uc2():

    # Load the configuration file
    config_file_path = "config.yml"
    train_config_path = 'rl_pipeline/configs/base_ppo_config.yaml'

    config = yaml.safe_load(open(config_file_path, 'r'))
    train_config = yaml.safe_load(open(train_config_path, 'r'))

    exp_name = f"{train_config['experiment']['name']}_{str(datetime.date.today())}"
    log_dir = (Path('experiments/') / train_config['environment']['id'] / train_config['training']['algorithm'] /
               exp_name)

    n_environments = config["n_environments"]

    # Create the vectorized environment
    vec_env = UseCase2Environment.create_vectorized_environment(n_environments=n_environments, return_type="stable-baselines", monitor=train_config['environment']['monitor'])
    
    if train_config['environment'].get('video_wrapper'):
        vec_env = VecVideoRecorder(
            vec_env,
            video_folder=f"{str(log_dir / 'videos')}",
            record_video_trigger=lambda x: x % train_config.get('environment').get('video_trigger') == 0,
            video_length=train_config.get('environment').get('video_length')
        )    
    
    vec_env.reset()

    pm_path = train_config['training']['pretrained_model']
    pretrained_model = None if pm_path == 'None' else Path(pm_path)

    trainer = RLTrainer(env=vec_env, config=train_config['training'], log_dir=log_dir, pretrained_model=pretrained_model,
                        exp_name=exp_name, wandb_group=train_config['environment']['id'])
    trainer.run(eval_env=None, logger=None)

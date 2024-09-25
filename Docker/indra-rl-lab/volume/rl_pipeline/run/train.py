from pathlib import Path
import yaml
import datetime

from rl_pipeline.run.rl_trainer import RLTrainer
from rl_pipeline.utils.make_env import make_env


def train(config_path: str):
    config = yaml.safe_load(open(config_path, 'r'))
    exp_name = f"{config['experiment']['name']}_{str(datetime.date.today())}"

    log_dir = (Path('experiments/') / config['environment']['id'] / config['training']['algorithm'] /
               exp_name)

    env = make_env(config=config, log_dir=log_dir)

    pm_path = config['training']['pretrained_model']
    pretrained_model = None if pm_path == 'None' else Path(pm_path)

    trainer = RLTrainer(env=env, training_config=config['training'], log_dir=log_dir, pretrained_model=pretrained_model,
                        exp_name=exp_name, wandb_group=config['environment']['id'])
    trainer.run(eval_env=None, logger=None)

from pathlib import Path
import yaml
import gymnasium as gym
from torch.backends.cudnn import deterministic

from rl_pipeline.utils.algorithm_registry import AVAILABLE_ALGORITHMS


def play_pretrained(c_file):
    env = gym.make(id=c_file['environment']['id'], render_mode='human')
    obs, _ = env.reset()
    env.render()

    log_dir = (Path('experiments/') / c_file['environment']['id'] / c_file['training']['algorithm'] /
               c_file['play']['experiment'])

    model_dir = f"{log_dir}/{c_file['play']['pretrained_model']}"
    parameters = AVAILABLE_ALGORITHMS[c_file['training'].get('algorithm')][1](env, c_file['training'], log_dir)
    model = AVAILABLE_ALGORITHMS[c_file['training'].get('algorithm')][0].load(path=model_dir, **parameters)

    terminated = False
    reward_sum = 0
    while not terminated:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        reward_sum += reward
        env.render()

    print(f"****** Total reward: {reward_sum}")


def play_random(c_file):
    env = gym.make(id=c_file['environment']['id'], render_mode='human')
    env.reset()
    env.render()

    terminated = False
    reward_sum = 0
    while not terminated:
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        reward_sum += reward
        env.render()

    print(f"****** Total reward: {reward_sum}")

def play(config_path: str, random_agent: bool):
    config = yaml.safe_load(open(config_path, 'r'))

    if random_agent:
        play_random(config)
    else:
        if config['play'].get('pretrained_model') in (None, 'None'):
            print('Pretrained model path not provided, aborting run')
        else:
            play_pretrained(config)

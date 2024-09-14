import gymnasium as gym
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder


def make_env(config, log_dir, **kwargs):
    def _init():
        env = gym.make(config['environment']['id'], render_mode=config['environment']['render_mode'], **kwargs)

        if config['environment']['monitor']:
            env = Monitor(env)

        return env

    venv = DummyVecEnv([_init])

    if config['environment'].get('video_wrapper'):
        venv = VecVideoRecorder(
            venv,
            video_folder=f"{str(log_dir / 'videos')}",
            record_video_trigger=lambda x: x % config.get('environment').get('video_trigger') == 0,
            video_length=config.get('environment').get('video_length')
        )

    return venv

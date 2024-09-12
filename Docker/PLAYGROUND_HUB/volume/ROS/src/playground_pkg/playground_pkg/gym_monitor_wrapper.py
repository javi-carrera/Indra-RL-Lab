# Project: Playground
# File: gym_monitor_wrapper.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

from typing import Tuple

import gymnasium as gym
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder


class GymMonitorWrapper(gym.Env):
    pass
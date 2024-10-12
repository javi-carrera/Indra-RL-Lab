# Project: Indra-RL-Lab
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import cv2

import gymnasium as gym
import numpy as np
import yaml

from typing import List, Tuple
from pathlib import Path

from scipy.spatial.transform import Rotation

from interfaces_pkg.msg import UC3AgentState
from interfaces_pkg.srv import UC3EnvironmentStep, UC3EnvironmentReset
from rl_pkg import EnvironmentNode
from rl_pipeline.algorithm_registry import ALGORITHMS, get_algorithm_kwargs


class UC3Environment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # EnvironmentNode (ROS)
        EnvironmentNode.__init__(
            self,
            environment_name="uc3_environment",
            environment_id=environment_id,
            step_service_msg_type=UC3EnvironmentStep,
            reset_service_msg_type=UC3EnvironmentReset,
        )
        
        # Gymasium
        self.observation_space = None
        self.reward_range = None
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0, shape=(4,),
            dtype=np.float32
        )

        # Environment parameters
        self.MIN_LINEAR_VELOCITY = -5.0
        self.MAX_LINEAR_VELOCITY = 5.0
        self.MAX_YAW_RATE = 5.0
        self.MAX_TURRET_ROTATION_SPEED = 5.5
        self.MAX_EPISODE_STEPS = 1024

        self.current_target_distance = None
        self.previous_target_distance = None
        self.current_health_normalized = None
        self.previous_health_normalized = None
        self.current_target_health_normalized = None
        self.previous_target_health_normalized = None
        self.observation_buffer = []

        self.step_request: UC3EnvironmentStep.Request
        self.step_response: UC3EnvironmentStep.Response
        self.reset_request: UC3EnvironmentReset.Request
        self.reset_response: UC3EnvironmentReset.Response



    def reset_environment_variables(self):
        
        self.previous_target_distance = None
        self.previous_health_normalized = None
        self.previous_target_health_normalized = None
        self.observation_buffer = []
        

    def convert_action_to_request(self, action: np.ndarray = None):
        raise NotImplementedError

    def convert_response_to_state(self, response: UC3EnvironmentStep.Response) -> UC3AgentState:
        raise NotImplementedError

    def terminated(self, state: UC3AgentState) -> bool:

        has_died = state.tank.health_info.health <= 0.0
        has_target_died = state.target_tank.health_info.health <= 0.0

        return has_died or has_target_died

    def truncated(self, state: UC3AgentState) -> bool:
        return self.n_step > self.MAX_EPISODE_STEPS

    def info(self, state: UC3AgentState) -> dict:
        return {}
    


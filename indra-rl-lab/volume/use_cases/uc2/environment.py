# Project: Indra-RL-Lab
# File: environment.py
# Authors: Javier Carrera, Guillermo Escolano
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import cv2

import gymnasium as gym
import numpy as np
from typing import List, Tuple

from scipy.spatial.transform import Rotation

from interfaces_pkg.msg import UC2AgentState
from interfaces_pkg.srv import UC2EnvironmentStep, UC2EnvironmentReset
from rl_pkg import EnvironmentNode

class UC2Environment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # EnvironmentNode (ROS)

        EnvironmentNode.__init__(
            self,
            environment_name="uc2_environment",
            environment_id=environment_id,
            step_service_msg_type = UC2EnvironmentStep,
            reset_service_msg_type= UC2EnvironmentReset,
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
        self.MAX_YAW_RATE = 4.0
        self.MAX_TURRET_ROTATION_SPEED = 5.5
        self.DISTANCE_THRESHOLD = 20.0
        self.REWARD_DISTANCE_THRESHOLD = 10.0
        self.MAX_EPISODE_STEPS = 1024

        self.current_target_distance = None
        self.current_health_normalized = None
        self.previous_health_normalized = None
        self.current_target_health_normalized = None
        self.previous_target_health_normalized = None
        self.observation_buffer = []

        self.n_succes = 0
        self.tot_succes = 9
        self.row_succes = 4
        self.env_id = environment_id

    @property
    def step_request(self) -> UC2EnvironmentStep.Request:
        return self._step_request

    @property
    def reset_request(self) -> UC2EnvironmentReset.Request:
        return self._reset_request
    
    def reset_environment_variables(self) -> UC2AgentState:
        self.previous_target_distance = None
        self.previous_health_normalized = None
        self.previous_target_health_normalized = None
        self.observation_buffer = []

        self.n_succes = 0
        self.tot_succes = 9

        print("RESET ENVIRONMENT VARIABLES","Tot Succeses",self.tot_succes, "Number of successes:", self.n_succes, "for environment", self.env_id, flush=True)
        if self.n_succes == self.row_succes:
            self.tot_succes += 1
            self.n_succes = 0

        if self.n_succes == -self.row_succes:
            self.tot_succes -= 1
            self.tot_succes = max(0,self.tot_succes)
            self.n_succes = 0
        
        self.reset_request.bot_params.speed = 0.0
        self.reset_request.bot_params.fire_rate = 0.3
        self.reset_request.bot_params.follow_waypoints = np.random.random() > 0.5
        self.reset_request.bot_params.can_shoot = False
        self.reset_request.bot_params.turret_rotation_speed = self.MAX_TURRET_ROTATION_SPEED
        self.reset_request.bot_params.angle_error = 90.0
        self.reset_request.bot_params.range = 5.0

        if self.tot_succes == 1:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 10.0
            return

        if self.tot_succes == 2:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 5.0
            return

        if self.tot_succes == 3:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 90.0
            return

        if self.tot_succes == 4:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 45.0
            self.reset_request.bot_params.fire_rate = 0.5
            return
        
        if self.tot_succes == 5:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 45.0
            self.reset_request.bot_params.fire_rate = 0.5
            self.reset_request.bot_params.range = 10.0
            return
        
        if self.tot_succes == 6:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 20.0
            self.reset_request.bot_params.fire_rate = 0.7
            self.reset_request.bot_params.range = 10.0
            return
        
        if self.tot_succes == 7:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 20.0
            self.reset_request.bot_params.fire_rate = 1.0
            self.reset_request.bot_params.range = 15.0
            return
        
        if self.tot_succes == 8:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 10.0
            self.reset_request.bot_params.fire_rate = 2.0
            self.reset_request.bot_params.range = 15.0
            return
        
        # if self.tot_succes == 9:
        #     self.tot_succes = np.random.randint(0,9)
        #     self.reset_environment_variables()

        if self.tot_succes == 9:
            self.reset_request.bot_params.speed = self.MAX_LINEAR_VELOCITY / 2.0
            self.reset_request.bot_params.can_shoot = True
            self.reset_request.bot_params.angle_error = 15.0
            self.reset_request.bot_params.fire_rate = 2.0
            self.reset_request.bot_params.range = 20.0
        

        
    def convert_action_to_request(self, action: np.ndarray = None) -> UC2EnvironmentStep.Request:
        # Movement
        linear_velocity = (action[0] + 1.0) * (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) / 2.0 + self.MIN_LINEAR_VELOCITY
        yaw_rate = action[1] * self.MAX_YAW_RATE

        # Turret
        turret_rotation_speed = self.MAX_TURRET_ROTATION_SPEED * action[2]
        fire = bool(action[3] > 0.0)

        self.step_request.action.tank.target_twist.y = linear_velocity
        self.step_request.action.tank.target_twist.theta = yaw_rate
        self.step_request.action.tank.turret_actuator.rotation_speed = turret_rotation_speed
        self.step_request.action.tank.turret_actuator.fire = fire

        return self.step_request

    def convert_response_to_state(self, response: UC2EnvironmentStep.Response) -> UC2AgentState:
        return response.state
    
    def terminated(self, state: UC2AgentState) -> bool:

        has_died = state.tank.health_info.health <= 0.0
        has_target_died = state.target_tank.health_info.health <= 0.0
        if has_target_died:
            self.n_succes += 1
        elif has_died:
            self.n_succes -= 1

        return has_died or has_target_died
    
    def truncated(self, state: UC2AgentState) -> bool:
        if self.n_step > self.MAX_EPISODE_STEPS:
            self.n_succes -= 1
            return True
        return False
    
    def info(self, state: UC2AgentState) -> dict:
        return {}


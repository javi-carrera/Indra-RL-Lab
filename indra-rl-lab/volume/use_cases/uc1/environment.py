# Project: Playground
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import time
import cv2

import gymnasium as gym
import numpy as np

from scipy.spatial.transform import Rotation

from interfaces_pkg.msg import UC1AgentState
from interfaces_pkg.srv import UC1EnvironmentStep, UC1EnvironmentReset
from rl_pkg import EnvironmentNode


class UC1Environment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # EnvironmentNode (ROS)
        EnvironmentNode.__init__(
            self,
            environment_name="uc1_environment",
            environment_id=environment_id,
            step_service_msg_type=UC1EnvironmentStep,
            reset_service_msg_type=UC1EnvironmentReset,
        )

        # Gymasium
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )
        self.observation_space = None
        self.reward_range = None

        # Environment parameters
        self.MIN_LINEAR_VELOCITY = -5.0
        self.MAX_LINEAR_VELOCITY = 5.0
        self.MAX_YAW_RATE = 5.0
        self.MAX_EPISODE_STEPS = 1024

        self.current_target_distance = None
        self.previous_target_distance = None
        self.current_health_normalized = None
        self.previous_health_normalized = None

    def convert_action_to_request(self, action: np.ndarray = None):

        self.step_request: UC1EnvironmentStep.Request

        # Movement
        # Scale the action to the range [self._min_linear_velocity, self._max_linear_velocity] when action[0] is in the range [-1.0, 1.0]
        linear_velocity = (action[0] + 1.0) * (self.MAX_LINEAR_VELOCITY - self.MIN_LINEAR_VELOCITY) / 2.0 + self.MIN_LINEAR_VELOCITY
        yaw_rate = action[1] * self.MAX_YAW_RATE

        self.step_request.action.tank.target_twist.y = linear_velocity
        self.step_request.action.tank.target_twist.theta = yaw_rate

        return self.step_request

    def convert_response_to_state(self, response: UC1EnvironmentStep.Response) -> UC1AgentState:
        return response.state

    def reset(self):
        self.previous_health_normalized = None
        self.previous_target_distance = None

        return super().reset()

    def observation(self, state: UC1AgentState) -> np.ndarray:
        raise NotImplementedError

    def reward(self, state: UC1AgentState, action: np.ndarray = None) -> float:
        raise NotImplementedError

    def terminated(self, state: UC1AgentState) -> bool:

        has_reached_target = state.target_trigger_sensor.timer_count >= state.target_trigger_sensor.max_timer_count
        has_died = state.tank.health_info.health <= 0.0
        terminated = has_reached_target or has_died

        return terminated

    def truncated(self, state: UC1AgentState) -> bool:

        truncated = self.n_step > self.MAX_EPISODE_STEPS

        return truncated

    def info(self, state: UC1AgentState) -> dict:
        return {}

    def render(self):
        pass

        # # Check if the render mode is valid
        # valid_render_modes = ["human", "rgb_array"]

        # if render_mode not in valid_render_modes:
        #     raise ValueError(f"Invalid render mode: {render_mode}. Valid render modes are {valid_render_modes}")

        # # Decompress the image
        # np_arr = np.frombuffer(self.step_response.compressed_image.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # if render_mode == "human":
        #     cv2.imshow("ShootingExampleEnvironment", image)
        #     cv2.waitKey(1)

        # elif render_mode == "rgb_array":
        #     return image

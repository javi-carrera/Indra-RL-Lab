# Project: Playground
# File: single_agent_environment_node.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import logging
import time
from typing import Callable, Tuple, Type, Union, List, Optional

import numpy as np

from gymnasium.vector import AsyncVectorEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node

from rl_pkg.wrappers.gym_env_wrapper import GymEnvWrapper


class EnvironmentNode(Node):

    def __init__(
            self,
            environment_name: str,
            environment_id: int,
            step_service_msg_type: Callable,
            reset_service_msg_type: Callable,
    ):
        
        environment_name = f'{environment_name}_{environment_id}'
        
        # Logging
        self.logger = logging.getLogger(f'{environment_name}')
        handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # ROS
        rclpy.init()
        self.logger.info(f'Initializing...')
        super().__init__(environment_name)

        self._step_service_name = f'/{environment_name}/step'
        self._reset_service_name = f'/{environment_name}/reset'

        self._step_service_msg_type = step_service_msg_type
        self._reset_service_msg_type = reset_service_msg_type

        self._step_client = self.create_client(step_service_msg_type, self._step_service_name)
        self._reset_client = self.create_client(reset_service_msg_type, self._reset_service_name)

        self.step_request, self.step_response = step_service_msg_type.Request(), step_service_msg_type.Response()
        self.reset_request, self.reset_response = reset_service_msg_type.Request(), reset_service_msg_type.Response()

        while not (self._step_client.wait_for_service(timeout_sec=1.0) and self._reset_client.wait_for_service(timeout_sec=1.0)):
            self.logger.info(f'Services not available, waiting...')

        self.logger.info(f'Services available')

        # Gymnasium
        self.observation_space = None
        self.action_space = None
        self.reward_range = None

        self.n_step = 0
    

    @staticmethod
    def _get_current_timestamp() -> Time:

        current_time = time.time()
        seconds = int(current_time)
        nanoseconds = int((current_time - seconds) * 1e9)

        timestamp = Time()
        timestamp.sec = seconds
        timestamp.nanosec = nanoseconds

        return timestamp


    def _send_service_request(self, service_name: str) -> Type:

        available_services = {
            'step': (self._step_client, self.step_request),
            'reset': (self._reset_client, self.reset_request)
        }

        client, request = available_services.get(service_name, (None, None))

        if client is None or request is None:
            raise ValueError(f"Invalid service name: {service_name}")
        
        request.request_sent_timestamp = self._get_current_timestamp()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        response.response_received_timestamp = self._get_current_timestamp()
        
        return response


    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:

        self.step_request = self.convert_action_to_request(action)
        self.step_response = self._send_service_request('step')
        state = self.convert_response_to_state(self.step_response)

        observation = self.observation(state)
        reward = self.reward(state, action)
        terminated = self.terminated(state)
        truncated = self.truncated(state)
        info = self.info(state)

        self.n_step += 1

        return observation, reward, terminated, truncated, info
    

    def reset(self) -> Tuple[np.ndarray, dict]:
        
        self.reset_request.reset = True
        self.reset_response = self._send_service_request('reset')
        state = self.convert_response_to_state(self.reset_response)

        observation = self.observation(state)
        info = self.info(state)

        self.n_step = 0

        return observation, info
    

    def close(self):

        self.logger.info(f'Closing environment...')
        self.destroy_node()
        # rclpy.shutdown()


    @classmethod
    def create_gym_environment(
        cls,
        environment_id: int = 0,
        monitor: bool = False,
        wrappers: Optional[List[Callable]] = None
    ) -> GymEnvWrapper:
        
        env = GymEnvWrapper(cls(environment_id))

        wrappers = wrappers if wrappers is not None else []
        wrappers.append(Monitor) if monitor else None
        for wrapper in wrappers:
            env = wrapper(env)
        
        return env


    @classmethod
    def create_vectorized_environment(
        cls,
        n_environments: int = 1,
        return_type: str = 'gym',
        monitor: bool = False,
        wrappers: Optional[List[Callable]] = None
    ) -> Union[AsyncVectorEnv, SubprocVecEnv]:


        return_types = {
            'gym': AsyncVectorEnv,
            'stable-baselines': SubprocVecEnv
        }

        return_kwargs = {
            'gym': {'context': 'spawn'},
            'stable-baselines': {'start_method': 'spawn'}
        }

        if return_type not in return_types.keys():
            raise ValueError(f"Invalid return type: {return_type}. Valid return types are {return_types.keys()}")

        environment_generators = [
            lambda env_id=i: cls.create_gym_environment(
                env_id,
                monitor=monitor,
                wrappers=wrappers
            ) for i in range(n_environments)
        ]

        return return_types[return_type](
            environment_generators,
            **return_kwargs[return_type]
        )

    def convert_action_to_request(self, action: np.ndarray = None) -> Type:
        """
        Convert the action to ROS request format
        """
        raise NotImplementedError
    
    def convert_response_to_state(self, response: Type) -> Type:
        """
        Convert the response ro numpy array
        """
        raise NotImplementedError    

    def observation(self, state) -> np.ndarray:
        raise NotImplementedError
    
    def reward(self, state, action) -> float:
        raise NotImplementedError
    
    def terminated(self, state) -> bool:
        raise NotImplementedError
    
    def truncated(self, state) -> bool:
        raise NotImplementedError
    
    def info(self, state) -> dict:
        raise NotImplementedError
    
    def render(self):
        raise NotImplementedError
    
# Project: Playground
# File: single_agent_environment_node.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import logging
import time
from typing import Callable, Tuple, Type, Union, List, Optional

import numpy as np
import gymnasium as gym

from gymnasium.vector import AsyncVectorEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node

class EnvironmentNode(gym.Env):

    """
    Base class for ROS2 environment nodes that interact with the Gymnasium API

    Attributes
    - :attr:`observation_space` - Observation space of the environment
    - :attr:`action_space` - Action space of the environment
    - :attr:`reward_range` - Reward range of the environment
    - :attr:`n_step` - Number of steps taken in the environment during the current episode
    - :attr:`logger` - Logger for the environment node

    Methods
    - :meth:`reset` - Reset the environment
    - :meth:`step` - Take a step in the environment
    - :meth:`close` - Close the environment
    - :meth:`render` - Render the environment
    - :meth:`wrap_environment` - Returns a wrapped environment with the specified wrappers
    - :meth:`create_vectorized_environment` - Create a vectorized environment with the specified number of environments and return type
    """

    def __init__(
        self,
        environment_name: str,
        environment_id: int,
        step_service_msg_type: Callable,
        reset_service_msg_type: Callable,
    ):
        
        """
        Args:
            environment_name (str): Name of the environment. This name will be used to create the ROS2 node and used as a prefix for the ROS2 services
            environment_id (int): ID of the environment
            step_service_msg_type (Callable): ROS service message type for the step service
            reset_service_msg_type (Callable): ROS service message type for the reset service
        """
        
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
        self._node = Node(environment_name)

        self._step_service_name = f'/{environment_name}/step'
        self._reset_service_name = f'/{environment_name}/reset'

        self._step_service_msg_type = step_service_msg_type
        self._reset_service_msg_type = reset_service_msg_type

        self._step_client = self._node.create_client(step_service_msg_type, self._step_service_name)
        self._reset_client = self._node.create_client(reset_service_msg_type, self._reset_service_name)

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

        if service_name not in available_services.keys():
            raise ValueError(f"Invalid service name: {service_name}. Valid service names are {available_services.keys()}")
        
        client, request = available_services.get(service_name, (None, None))

        request.request_sent_timestamp = self._get_current_timestamp()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        response = future.result()
        response.response_received_timestamp = self._get_current_timestamp()
        
        return response
    
    def send_reset_request(self) -> Type:

        """Send a reset request to the environment
        
        Returns:
            state (Type): State of the environment after the reset
        """

        self.reset_request.reset = True
        self.reset_response = self._send_service_request('reset')
        state = self.convert_response_to_state(self.reset_response)

        return state
    
    def send_step_request(self, action: np.ndarray) -> Type:

        """Send a step request to the environment

        Args:
            action (np.ndarray): Action to take in the environment

        Returns:
            state (Type): State of the environment after taking the step
        """
            
        self.step_request = self.convert_action_to_request(action)
        self.step_response = self._send_service_request('step')
        state = self.convert_response_to_state(self.step_response)
    
        return state

    
    def reset(self, **kwargs) -> Tuple[np.ndarray, dict]:

        """Reset the environment"""

        state = self.send_reset_request()

        self.reset_environment_variables()
        observation = self.observation(state)
        info = self.info(state)

        self.n_step = 0

        return observation, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:

        """Take a step in the environment

        Args:
            action (np.ndarray): Action to take in the environment

        Returns:
            observation (np.ndarray): Observation after taking the step\n
            reward (float): Reward after taking the step\n
            terminated (bool): Whether the episode has terminated\n
            info (dict): Additional information
        """

        state = self.send_step_request(action)

        observation = self.observation(state)
        reward = self.reward(state)
        terminated = self.terminated(state)
        truncated = self.truncated(state)
        info = self.info(state)

        self.n_step += 1

        return observation, reward, terminated, truncated, info
    
    def close(self):

        """Close the environment"""

        self.logger.info(f'Closing environment...')
        self._node.destroy_node()
        # rclpy.shutdown()

    @classmethod
    def wrap_environment(
        cls,
        environment_id: int = 0,
        monitor: bool = False,
        wrappers: Optional[List[Callable]] = None
    ) -> Type:
        
        """Create a wrapped environment with the specified wrappers

        Args:
            environment_id (int): ID of the environment
            monitor (bool): Whether to monitor the environment
            wrappers (Optional[List[Callable]]): List of wrappers to apply to the environment

        Returns:
            env (Type): Wrapped environment
        """
        
        env = cls(environment_id)

        wrappers = wrappers if wrappers is not None else []
        for wrapper in wrappers:
            env = wrapper(env)

        if monitor:
            env = Monitor(env)
        
        return env

    @classmethod
    def create_vectorized_environment(
        cls,
        n_environments: int = 1,
        return_type: str = 'gym',
        monitor: bool = False,
        wrappers: Optional[List[Callable]] = None
    ) -> Union[AsyncVectorEnv, SubprocVecEnv]:
        
        """Create a vectorized environment with the specified number of environments and return type

        Args:
            n_environments (int): Number of environments
            return_type (str): Type of vectorized environment to return
            monitor (bool): Whether to monitor the environment
            wrappers (Optional[List[Callable]]): List of wrappers to apply to the environment

        Returns:
            env (Union[AsyncVectorEnv, SubprocVecEnv]): Vectorized environment
        """

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
            lambda env_id=i: cls.wrap_environment(
                env_id,
                monitor=monitor,
                wrappers=wrappers
            ) for i in range(n_environments)
        ]

        return return_types[return_type](
            environment_generators,
            **return_kwargs[return_type]
        )
    
    def reset_environment_variables(self):
        """Reset the environment variables"""
        self.logger.error("'reset_environment_variables' method is not implemented")
        raise NotImplementedError

    def convert_action_to_request(self, action: np.ndarray) -> Type:
        """Convert the action to ROS request format

        Args:
            action (np.ndarray): Action to convert

        Returns:
            request (Type): ROS request message
        """
        self.logger.error("'convert_action_to_request' method is not implemented")
        raise NotImplementedError
    
    def convert_response_to_state(self, response: Type) -> Type:
        """Convert the response to numpy array

        Args:
            response (Type): ROS response message

        Returns:
            state (Type): State of the environment in ROS response format
        """
        self.logger.error("'convert_response_to_state' method is not implemented")
        raise NotImplementedError

    def observation(self, state: Type) -> np.ndarray:
        """Convert the state to observation

        Args:
            state (Type): State of the environment in ROS response format

        Returns:
            observation (np.ndarray): Observation of the environment
        """
        self.logger.error("'observation' method is not implemented")
        raise NotImplementedError
    
    def reward(self, state: Type) -> float:
        """Calculate the reward

        Args:
            state (Type): State of the environment in ROS response format

        Returns:
            reward (float): Reward of the environment
        """
        self.logger.error("'reward' method is not implemented")
        raise NotImplementedError
    
    def terminated(self, state: Type) -> bool:
        """Check if the episode has terminated

        Args:
            state (Type): State of the environment in ROS response format

        Returns:
            terminated (bool): Whether the episode has terminated
        """
        self.logger.error("'terminated' method is not implemented")
        raise NotImplementedError
    
    def truncated(self, state: Type) -> bool:
        """Check if the episode has been truncated

        Args:
            state (Type): State of the environment in ROS response format

        Returns:
            truncated (bool): Whether the episode has been truncated
        """
        self.logger.error("'truncated' method is not implemented")
        raise NotImplementedError
    
    def info(self, state: Type) -> dict:
        """Get additional information about the environment

        Args:
            state (Type): State of the environment in ROS response format

        Returns:
            info (dict): Additional information about the environment
        """
        self.logger.error("'info' method is not implemented")
        raise NotImplementedError

    def render(self):
        """Render the environment"""
        self.logger.error("'render' method is not implemented")
        raise NotImplementedError


    
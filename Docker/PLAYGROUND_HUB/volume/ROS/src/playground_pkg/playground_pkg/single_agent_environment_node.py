from typing import Tuple, Type, Callable

import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
import time
import logging


class SingleAgentEnvironmentNode(Node):

    def __init__(
            self,
            environment_name: str,
            environment_id: int,
            step_service_msg_type: Callable,
            reset_service_msg_type: Callable,
    ):
        
        environment_name = f'{environment_name}_{environment_id}'
        
        # Logging initialization
        self.logger = logging.getLogger(f'{environment_name}')
        handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # ROS initialization
        try:
            rclpy.init()
        except RuntimeError:
            pass

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


        # Wait for the service to be available
        while not (self._step_client.wait_for_service(timeout_sec=1.0) and self._reset_client.wait_for_service(timeout_sec=1.0)):
            self.logger.info(f'Services not available, waiting...')

        self.logger.info(f'Services available')
    

    @staticmethod
    def _get_current_timestamp() -> Time:

        # Get the current time
        current_time = time.time()

        seconds = int(current_time)
        nanoseconds = int((current_time - seconds) * 1e9)

        # Convert the current time to ROS time
        timestamp = Time()
        timestamp.sec = seconds
        timestamp.nanosec = nanoseconds

        return timestamp


    def _send_service_request(self, service_name: str) -> Type:

        # Determine the client based on the service name
        match service_name:
            case 'step':
                client = self._step_client
                request = self.step_request
            case 'reset':
                client = self._reset_client
                request = self.reset_request
            case _:
                raise ValueError(f"Invalid service name: {service_name}")
        
        # Set the timestamp of the request
        request.request_sent_timestamp = self._get_current_timestamp()

        # Call the service and wait for the response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # Set the timestamp of the response
        response.response_received_timestamp = self._get_current_timestamp()
        
        return response


    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:

        # Format the request, send it to the 'step' service and format the response
        self.step_request = self.convert_action_to_request(action)
        self.step_response = self._send_service_request('step')
        state = self.convert_response_to_state(self.step_response)

        # Get the observation, reward, terminated, truncated and info from the state
        observation = self.observation(state)
        reward = self.reward(state, action)
        terminated = self.terminated(state)
        truncated = self.truncated(state)
        info = self.info(state)

        return observation, reward, terminated, truncated, info
    

    def reset(self) -> Tuple[np.ndarray, dict]:
        
        # Format the request, send it to the 'reset' service and format the response
        self.reset_request = self.convert_reset_to_request()
        self.reset_response = self._send_service_request('reset')
        state = self.convert_response_to_state(self.reset_response)

        # Get the observation
        observation = self.observation(state)

        # Get the info
        info = self.info(state)

        return observation, info
    

    def close(self):

        self.logger.info(f'Closing {self.get_name()} environment node')

        # Destroy the node and shutdown ROS
        self.destroy_node()
        # rclpy.shutdown()
        
    

    def convert_action_to_request(self, action: np.ndarray = None) -> Type:
        """
        Convert the action to ROS request format
        """
        raise NotImplementedError
    
    def convert_response_to_state(self, response) -> Type:
        """
        Convert the response ro numpy array
        """
        raise NotImplementedError
    
    def convert_reset_to_request(self) -> Type:
        """
        Convert the reset to ROS request format
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
    


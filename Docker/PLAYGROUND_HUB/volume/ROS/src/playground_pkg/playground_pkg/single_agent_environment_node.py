from typing import Tuple, Type

import numpy as np

import rclpy
from rclpy.node import Node


class SingleAgentEnvironmentNode(Node):

    def __init__(
            self,
            environment_name: str,
            service_msg_type,
    ):
        
        # ROS initialization
        super().__init__(environment_name)

        self._service_name = f'/{environment_name}'
        self._service_msg_type = service_msg_type
        self._client = self.create_client(service_msg_type, self._service_name)

        self._request = service_msg_type.Request()
        self._response = service_msg_type.Response()


        # Wait for the service to be available
        while not self._client.wait_for_service(timeout_sec=1.0):
            print(f'Service {self._service_name} not available, waiting...')

        print(f'Service {self._service_name} is available\n')


    def _send_service_request(self, request):

        # Call the service and wait for the response
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:

        # Call the service
        request = self.convert_action_to_request(action)
        response = self._send_service_request(request)
        state = self.convert_response_to_state(response)

        # Get the observation, reward, terminated, truncated and info
        observation = self.observation(state)
        reward = self.reward(state, action)
        terminated = self.terminated(state)
        truncated = self.truncated(state)
        info = self.info(state)

        return observation, reward, terminated, truncated, info
    

    def reset(self) -> Tuple[np.ndarray, dict]:
        
        # Call the service
        request = self._service_msg_type.Request()
        request.reset = True
        response = self._send_service_request(request)
        state = self.convert_response_to_state(response)

        # Get the observation
        observation = self.observation(state)

        return observation, {}
    

    def close(self):

        # Destroy the node and shutdown ROS
        self.destroy_node()
        rclpy.shutdown()
    

    def convert_action_to_request(self, action: np.ndarray, reset: bool = False):
        """
        Convert the action to ROS request format
        """
        raise NotImplementedError
    
    def convert_response_to_state(self, response) -> np.ndarray:
        """
        Convert the response ro numpy array
        """
        raise NotImplementedError
    

    def observation(self, state: np.ndarray) -> np.ndarray:
        raise NotImplementedError
    
    def reward(self, state: np.ndarray, action: np.ndarray = None) -> float:
        raise NotImplementedError
    
    def terminated(self, state: np.ndarray) -> bool:
        raise NotImplementedError
    
    def truncated(self, state: np.ndarray) -> bool:
        raise NotImplementedError
    
    def info(self, state: np.ndarray) -> dict:
        raise NotImplementedError
    
    def render(self):
        raise NotImplementedError
    


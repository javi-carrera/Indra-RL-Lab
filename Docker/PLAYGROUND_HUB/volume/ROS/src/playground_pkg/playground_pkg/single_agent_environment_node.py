from typing import Tuple, Type

import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
import time


class SingleAgentEnvironmentNode(Node):

    def __init__(
            self,
            environment_name: str,
            action_service_msg_type: Type,
            state_service_msg_type: Type,
            reset_service_msg_type: Type,
            sample_time: float,
    ):
        
        # ROS initialization
        super().__init__(environment_name)

        self._action_service_name = f'/{environment_name}/action'
        self._state_service_name = f'/{environment_name}/state'
        self._reset_service_name = f'/{environment_name}/reset'

        self._action_service_msg_type = action_service_msg_type
        self._state_service_msg_type = state_service_msg_type
        self._reset_service_msg_type = reset_service_msg_type

        self._action_client = self.create_client(action_service_msg_type, self._action_service_name)
        self._state_client = self.create_client(state_service_msg_type, self._state_service_name)
        self._reset_client = self.create_client(reset_service_msg_type, self._reset_service_name)

        self.action_request, self.action_response = action_service_msg_type.Request(), action_service_msg_type.Response()
        self.state_request, self.state_response = state_service_msg_type.Request(), state_service_msg_type.Response()
        self.reset_request, self.reset_response = reset_service_msg_type.Request(), reset_service_msg_type.Response()

        self._sample_time = sample_time


        # TODO: Wait for ALL the services to be available
        # Wait for the service to be available
        while not self._action_client.wait_for_service(timeout_sec=1.0):
            print(f'Service {self._action_service_name} not available, waiting...')

        print(f'Service {self._action_service_name} is available\n')
    

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
            case 'action':
                client = self._action_client
                request = self.action_request
            case 'state':
                client = self._state_client
                request = self.state_request
            case 'reset':
                client = self._reset_client
                request = self.reset_request
            case _:
                raise ValueError(f"Invalid service name: {service_name}")
        
        # Set the timestamp of the request
        request.timestamp = self._get_current_timestamp()

        # Call the service and wait for the response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:

        # Format the request and send it to the 'action' service
        self.action_request = self.convert_action_to_request(action)
        self.action_response = self._send_service_request('action')

        # Wait for 'sample_time' seconds 
        time.sleep(self._sample_time)

        # Send a request to the 'state' service and format the response
        self.state_response = self._send_service_request('state')
        state = self.convert_response_to_state(self.state_response)

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

        # Destroy the node and shutdown ROS
        self.destroy_node()
        rclpy.shutdown()
    

    def convert_action_to_request(self, action: np.ndarray = None) -> Type:
        """
        Convert the action to ROS request format
        """
        raise NotImplementedError
    
    def convert_response_to_state(self, response) -> dict:
        """
        Convert the response ro numpy array
        """
        raise NotImplementedError
    
    def convert_reset_to_request(self) -> Type:
        """
        Convert the reset to ROS request format
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
    


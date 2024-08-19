from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from builtin_interfaces.msg import Time
import time


class CommunicationMonitor:


    def __init__(self, environment: SingleAgentEnvironmentNode):

        self._environment = environment

        self._previous_state_response_time = None

        self._step_request_time = None
        self._step_response_time = None

        self._reset_request_time = None
        self._reset_response_time = None


    @staticmethod
    def get_elapsed_time(start_time: Time, end_time: Time) -> float:

        if start_time is None or end_time is None:
            return None
        
        # Get the elapsed time in seconds
        elapsed_time = end_time.sec - start_time.sec + (end_time.nanosec - start_time.nanosec) / 1e9

        return elapsed_time
    

    def update(self):
        
        # Get the timestamps for each service
        self._step_request_time = self._environment.step_request.timestamp
        self._step_response_time = self._environment.step_response.timestamp

        self._reset_request_time = self._environment.reset_request.timestamp
        self._reset_response_time = self._environment.reset_response.timestamp
        

    def visualize(self):

        # Get the elapsed time for each service
        step_elapsed_time = CommunicationMonitor.get_elapsed_time(self._step_request_time, self._step_response_time)
        reset_elapsed_time = CommunicationMonitor.get_elapsed_time(self._reset_request_time, self._reset_response_time)

        # Get the 'inference_time'
        inference_time = CommunicationMonitor.get_elapsed_time(self._previous_state_response_time, self._step_request_time)

        # Update the previous state response time
        self._previous_state_response_time = self._step_response_time

        # Display the results
        print(
            f'Step elapsed time   : {step_elapsed_time} [sec]\n'
            f'Reset elapsed time  : {reset_elapsed_time} [sec]\n'
            f'Inference time      : {inference_time} [sec]\n'
        )
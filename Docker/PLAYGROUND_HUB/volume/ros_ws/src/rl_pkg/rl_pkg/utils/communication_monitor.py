# Project: Playground
# File: communication_monitor.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from builtin_interfaces.msg import Time

from rl_pkg.wrappers.gym_env_wrapper import GymEnvWrapper
from rl_pkg.environment_node import EnvironmentNode


class CommunicationMonitor:


    def __init__(self, environment: EnvironmentNode | GymEnvWrapper):

        self._environment = environment

        # Check if the environment is a 'GymEnvWrapper' instance
        if isinstance(self._environment, EnvironmentNode):
            self._environment = self._environment
        elif isinstance(self._environment, GymEnvWrapper):
            self._environment = self._environment.env
        else:
            raise ValueError("The 'environment' must be an instance of 'EnvironmentNode' or 'GymEnvWrapper'")

        self._previous_step_response_received_timestamp = None

        self._step_request_sent_timestamp = None
        self._step_request_received_timestamp = None
        self._step_response_sent_timestamp = None
        self._step_response_received_timestamp = None

        self._reset_request_sent_timestamp = None
        self._reset_request_received_timestamp = None
        self._reset_response_sent_timestamp = None
        self._reset_response_received_timestamp = None


    @staticmethod
    def get_elapsed_time(start_time: Time | None, end_time: Time | None) -> float:

        if start_time is None or end_time is None:
            return None
        
        # Get the elapsed time in seconds
        elapsed_time = end_time.sec - start_time.sec + (end_time.nanosec - start_time.nanosec) / 1e9

        return elapsed_time
    

    def _update(self):
        
        # Get the timestamps for each service
        self._step_request_sent_timestamp = self._environment.step_request.request_sent_timestamp
        self._step_request_received_timestamp = self._environment.step_response.request_received_timestamp
        self._step_response_sent_timestamp = self._environment.step_response.response_sent_timestamp
        self._step_response_received_timestamp = self._environment.step_response.response_received_timestamp

        self._reset_request_sent_timestamp = self._environment.reset_request.request_sent_timestamp
        self._reset_request_received_timestamp = self._environment.reset_response.request_received_timestamp
        self._reset_response_sent_timestamp = self._environment.reset_response.response_sent_timestamp
        self._reset_response_received_timestamp = self._environment.reset_response.response_received_timestamp
        

    def display(self):

        # Update the timestamps
        self._update()

        # Get the elapsed times for each service
        step_request_latency_time = self.get_elapsed_time(self._step_request_sent_timestamp, self._step_request_received_timestamp)
        step_request_process_time = self.get_elapsed_time(self._step_request_received_timestamp, self._step_response_sent_timestamp)
        step_response_latency_time = self.get_elapsed_time(self._step_response_sent_timestamp, self._step_response_received_timestamp)
        step_total_elapsed_time = step_request_latency_time + step_request_process_time + step_response_latency_time

        reset_request_latency_time = self.get_elapsed_time(self._reset_request_sent_timestamp, self._reset_request_received_timestamp)
        reset_request_process_time = self.get_elapsed_time(self._reset_request_received_timestamp, self._reset_response_sent_timestamp)
        reset_response_latency_time = self.get_elapsed_time(self._reset_response_sent_timestamp, self._reset_response_received_timestamp)
        reset_total_elapsed_time = reset_request_latency_time + reset_request_process_time + reset_response_latency_time

        # Get the 'inference_time'
        inference_time = self.get_elapsed_time(self._previous_step_response_received_timestamp, self._step_request_sent_timestamp)

        # Update the previous state response time
        self._previous_step_response_received_timestamp = self._step_response_received_timestamp

        # Display the results
        print(
            f"\n------------------------------------------\n"
            f"Step request latency time   [s] : {step_request_latency_time}\n" \
            f"Step request process time   [s] : {step_request_process_time}\n" \
            f"Step response latency time  [s] : {step_response_latency_time}\n" \
            f"Total step elapsed time     [s] : {step_total_elapsed_time}\n" \
            f"\n"
            f"Reset request latency time  [s] : {reset_request_latency_time}\n" \
            f"Reset request process time  [s] : {reset_request_process_time}\n" \
            f"Reset response latency time [s] : {reset_response_latency_time}\n" \
            f"Total reset elapsed time    [s] : {reset_total_elapsed_time}\n" \
            f"\n"
            f"Inference time              [s] : {inference_time}\n"
            f"------------------------------------------\n"
        )

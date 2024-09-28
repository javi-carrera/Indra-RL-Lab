# Project: Playground
# File: communication_monitor.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import numpy as np
from collections import deque
from builtin_interfaces.msg import Time

from rl_pkg.wrappers.gym_env_wrapper import GymEnvWrapper
from rl_pkg.environment_node import EnvironmentNode

class CommunicationMonitor:

    def __init__(self, environment: EnvironmentNode | GymEnvWrapper, window_size: int = 100):

        # Check if the environment is a 'GymEnvWrapper' instance
        if isinstance(environment, EnvironmentNode):
            self._environment = environment
        elif isinstance(environment, GymEnvWrapper):
            self._environment = environment.env
        else:
            raise ValueError("The 'environment' must be an instance of 'EnvironmentNode' or 'GymEnvWrapper'")

        # Initialize the previous step response received timestamp
        self._previous_step_response_received_timestamp = None

        # Initialize the timestamps for the step service
        self._step_request_sent_timestamp = None
        self._step_request_received_timestamp = None
        self._step_response_sent_timestamp = None
        self._step_response_received_timestamp = None

        # Initialize the timestamps for the reset service
        self._reset_request_sent_timestamp = None
        self._reset_request_received_timestamp = None
        self._reset_response_sent_timestamp = None
        self._reset_response_received_timestamp = None

        # Initialize the window size for rolling window statistics
        self.window_size = window_size

        # Initialize deques to store times for rolling window statistics
        self.step_request_latency_times = deque(maxlen=self.window_size)
        self.step_request_process_times = deque(maxlen=self.window_size)
        self.step_response_latency_times = deque(maxlen=self.window_size)
        self.step_total_elapsed_times = deque(maxlen=self.window_size)
        self.inference_times = deque(maxlen=self.window_size)

    @staticmethod
    def get_elapsed_time(start_time: Time | None, end_time: Time | None) -> float | None:

        # Check if the timestamps are not None
        if start_time is None or end_time is None:
            return None

        # Calculate the elapsed time in seconds, handling possible nanosecond underflow
        sec_diff = end_time.sec - start_time.sec
        nanosec_diff = end_time.nanosec - start_time.nanosec

        # Handle nanosecond underflow
        if nanosec_diff < 0:
            sec_diff -= 1
            nanosec_diff += 1e9

        # Calculate the elapsed time in seconds
        elapsed_time = sec_diff + nanosec_diff / 1e9

        return elapsed_time
    
    @staticmethod
    def calculate_statistics(values: deque) -> tuple[float, float, float, float]:

        if len(values) == 0:
            return (None, None, None, None)
        
        mean = np.mean(values)
        std = np.std(values)
        max_value = np.max(values)
        min_value = np.min(values)

        return (mean, std, max_value, min_value)
    
    @staticmethod
    def format_row(metric_name, current_value, stats):

        if current_value is None:
            current_value_str = 'N/A'
        else:
            current_value_str = f"{current_value:.6f}"

        mean, std, max_value, min_value = stats
        mean_str = f"{mean:.6f}" if mean is not None else 'N/A'
        std_str = f"{std:.6f}" if std is not None else 'N/A'
        max_str = f"{max_value:.6f}" if max_value is not None else 'N/A'
        min_str = f"{min_value:.6f}" if min_value is not None else 'N/A'

        return f"{metric_name:<35}{current_value_str:>15}{mean_str:>15}{std_str:>15}{max_str:>15}{min_str:>15}"

    def _update(self):

        # Get the timestamps for the step service
        self._step_request_sent_timestamp = self._environment.step_request.request_sent_timestamp
        self._step_request_received_timestamp = self._environment.step_response.request_received_timestamp
        self._step_response_sent_timestamp = self._environment.step_response.response_sent_timestamp
        self._step_response_received_timestamp = self._environment.step_response.response_received_timestamp

        # Get the timestamps for the reset service
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

        # Calculate the total step elapsed time
        if None not in (step_request_latency_time, step_request_process_time, step_response_latency_time):
            step_total_elapsed_time = step_request_latency_time + step_request_process_time + step_response_latency_time
        else:
            step_total_elapsed_time = None

        # Get the 'inference_time'
        inference_time = self.get_elapsed_time(self._previous_step_response_received_timestamp, self._step_request_sent_timestamp)

        # Update the previous state response time
        self._previous_step_response_received_timestamp = self._step_response_received_timestamp

        # Append times to rolling window deques if they are not None
        if step_request_latency_time is not None:
            self.step_request_latency_times.append(step_request_latency_time)
        if step_request_process_time is not None:
            self.step_request_process_times.append(step_request_process_time)
        if step_response_latency_time is not None:
            self.step_response_latency_times.append(step_response_latency_time)
        if step_total_elapsed_time is not None:
            self.step_total_elapsed_times.append(step_total_elapsed_time)
        if inference_time is not None:
            self.inference_times.append(inference_time)

        # Calculate statistics for step times
        step_request_latency_stats = self.calculate_statistics(self.step_request_latency_times)
        step_request_process_stats = self.calculate_statistics(self.step_request_process_times)
        step_response_latency_stats = self.calculate_statistics(self.step_response_latency_times)
        step_total_elapsed_stats = self.calculate_statistics(self.step_total_elapsed_times)
        inference_stats = self.calculate_statistics(self.inference_times)


        # Format the table
        header = f"{'Metric':<35}{'Current Value':>15}{'Mean':>15}{'Std':>15}{'Max':>15}{'Min':>15}"
        separator = '-' * len(header)

        table = f"""
            {header}
            {separator}
            {self.format_row('Step request latency time [s]', step_request_latency_time, step_request_latency_stats)}
            {self.format_row('Step request process time [s]', step_request_process_time, step_request_process_stats)}
            {self.format_row('Step response latency time [s]', step_response_latency_time, step_response_latency_stats)}
            {self.format_row('Total step elapsed time [s]', step_total_elapsed_time, step_total_elapsed_stats)}
            {self.format_row('Inference time [s]', inference_time, inference_stats)}
            """

        # Display the results
        self._environment.logger.info(table)

# Project: Playground
# File: communication_monitor.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import numpy as np
from collections import deque
from builtin_interfaces.msg import Time

from rl_pkg.environment_node import EnvironmentNode

class CommunicationMonitor:

    def __init__(
        self,
        environment: EnvironmentNode,
        window_size: int = 100
    ):

        self._environment = environment
        
        self._previous_step_response_received_timestamp = None
        self._step_request_sent_timestamp = None
        self._step_request_received_timestamp = None
        self._step_response_sent_timestamp = None
        self._step_response_received_timestamp = None
        self._reset_request_sent_timestamp = None
        self._reset_request_received_timestamp = None
        self._reset_response_sent_timestamp = None
        self._reset_response_received_timestamp = None

        self.window_size = window_size
        self.step_request_latency_times = deque(maxlen=self.window_size)
        self.step_request_process_times = deque(maxlen=self.window_size)
        self.step_response_latency_times = deque(maxlen=self.window_size)
        self.step_total_elapsed_times = deque(maxlen=self.window_size)
        self.inference_times = deque(maxlen=self.window_size)

    @staticmethod
    def get_elapsed_time(start_time: Time | None, end_time: Time | None) -> float | None:

        if start_time is None or end_time is None:
            return None

        sec_diff = end_time.sec - start_time.sec
        nanosec_diff = end_time.nanosec - start_time.nanosec

        if nanosec_diff < 0:
            sec_diff -= 1
            nanosec_diff += 1e9

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

        self._step_request_sent_timestamp = self._environment.step_request.request_sent_timestamp
        self._step_request_received_timestamp = self._environment.step_response.request_received_timestamp
        self._step_response_sent_timestamp = self._environment.step_response.response_sent_timestamp
        self._step_response_received_timestamp = self._environment.step_response.response_received_timestamp

        self._reset_request_sent_timestamp = self._environment.reset_request.request_sent_timestamp
        self._reset_request_received_timestamp = self._environment.reset_response.request_received_timestamp
        self._reset_response_sent_timestamp = self._environment.reset_response.response_sent_timestamp
        self._reset_response_received_timestamp = self._environment.reset_response.response_received_timestamp

    def display(self):

        self._update()

        step_request_latency_time = self.get_elapsed_time(self._step_request_sent_timestamp, self._step_request_received_timestamp)
        step_request_process_time = self.get_elapsed_time(self._step_request_received_timestamp, self._step_response_sent_timestamp)
        step_response_latency_time = self.get_elapsed_time(self._step_response_sent_timestamp, self._step_response_received_timestamp)

        if None not in (step_request_latency_time, step_request_process_time, step_response_latency_time):
            step_total_elapsed_time = step_request_latency_time + step_request_process_time + step_response_latency_time
        else:
            step_total_elapsed_time = None

        inference_time = self.get_elapsed_time(self._previous_step_response_received_timestamp, self._step_request_sent_timestamp)

        self._previous_step_response_received_timestamp = self._step_response_received_timestamp

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

        step_request_latency_stats = self.calculate_statistics(self.step_request_latency_times)
        step_request_process_stats = self.calculate_statistics(self.step_request_process_times)
        step_response_latency_stats = self.calculate_statistics(self.step_response_latency_times)
        step_total_elapsed_stats = self.calculate_statistics(self.step_total_elapsed_times)
        inference_stats = self.calculate_statistics(self.inference_times)

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
        
        self._environment.logger.info(table)

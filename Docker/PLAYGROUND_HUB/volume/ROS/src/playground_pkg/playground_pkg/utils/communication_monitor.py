from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from builtin_interfaces.msg import Time
import time


class CommunicationMonitor:


    def __init__(self, environment: SingleAgentEnvironmentNode):

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
            f"------------------------------------------\n"
            f"Step request latency time   : {step_request_latency_time} [s]\n" \
            f"Step request process time   : {step_request_process_time} [s]\n" \
            f"Step response latency time  : {step_response_latency_time} [s]\n" \
            f"Total step elapsed time     : {step_total_elapsed_time} [s]\n" \
            f"\n"
            f"Reset request latency time  : {reset_request_latency_time} [s]\n" \
            f"Reset request process time  : {reset_request_process_time} [s]\n" \
            f"Reset response latency time : {reset_response_latency_time} [s]\n" \
            f"Total reset elapsed time    : {reset_total_elapsed_time} [s]\n" \
            f"\n"
            f"Inference time              : {inference_time} [s]\n"
            f"------------------------------------------\n"
        )

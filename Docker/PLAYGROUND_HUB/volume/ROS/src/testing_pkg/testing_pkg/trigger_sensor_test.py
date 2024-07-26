from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from interfaces_pkg.srv import TriggerSensorTestEnvironmentStep

import numpy as np
import matplotlib.pyplot as plt


class TriggerSensorVisualizer:

    def __init__(self):

        # Initialize matplotlib
        plt.style.use('seaborn-darkgrid')       # Use print(plt.style.available)) to see available styles
        plt.ion()

        # Initialize the plot (bar plot)
        self.fig, self.ax = plt.subplots()


    def visualize(self, trigger_sensors_data: list):

        # Clear the plot
        self.ax.clear()

        # Extract the data from the trigger sensors
        has_triggered = [data.has_triggered for data in trigger_sensors_data]
        has_timer_finished = [data.has_timer_finished for data in trigger_sensors_data]
        timer_count = [data.timer_count for data in trigger_sensors_data]
        max_timer_count = [data.max_timer_count for data in trigger_sensors_data]

        # Calculate the progress as a percentage
        progress = [
            timer_count / max_timer_count * 100 
            if max_timer_count != 0 else 100 if triggered else 0
            for timer_count, max_timer_count, triggered in zip(timer_count, max_timer_count, has_triggered
        )]

        # Display the timer_count and max_timer_count
        for i, (timer_count, max_timer_count) in enumerate(zip(timer_count, max_timer_count)):
            self.ax.text(
                100,
                i,
                f"{timer_count:.2f}/{max_timer_count:.2f}",
                ha='right',
                va='center',
                color='black',
                fontsize=10,
                fontdict={
                    'family': 'monospace',
                },
                bbox={
                    'facecolor': 'white',
                    'edgecolor': 'none',
                    'alpha': 0.5,
                }
            )

        # Create a horizontal bar plot
        self.ax.barh(
            range(len(trigger_sensors_data)),
            progress,
            height=0.5,
            color=['green' if finished else 'red' for finished in has_timer_finished],
        )

        # Set the y-axis ticks and labels
        self.ax.set_yticks(range(len(trigger_sensors_data)))
        self.ax.set_yticklabels([f"Sensor {i+1}" for i in range(len(trigger_sensors_data))])

        # Set the x-axis limits and labels
        self.ax.set_xlim(0, 100)
        self.ax.set_xlabel("Progress (%)")

        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


class TriggerSensorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='trigger_sensor_test_environment',
            service_msg_type=TriggerSensorTestEnvironmentStep,
        )

        # Initialize the visualizer
        self._visualizer = TriggerSensorVisualizer()

        # Initialize the response
        self._response = TriggerSensorTestEnvironmentStep.Response()


    def convert_action_to_request(self, action: np.ndarray) -> TriggerSensorTestEnvironmentStep.Request:
        
        request = self._service_msg_type.Request()

        # Convert the action to ROS request format
        request.agent_action.trigger_sensor_request = True

        return request
    

    def convert_response_to_state(self, response) -> np.ndarray:

        # Store the response
        self._response = response

        # Convert the response to numpy array
        state = np.array([
            float(response.agent_state.trigger_sensor_01_data.has_triggered),
            float(response.agent_state.trigger_sensor_02_data.has_triggered),
            float(response.agent_state.trigger_sensor_03_data.has_triggered),
            float(response.agent_state.trigger_sensor_04_data.has_triggered),

            float(response.agent_state.trigger_sensor_01_data.has_timer_finished),
            float(response.agent_state.trigger_sensor_02_data.has_timer_finished),
            float(response.agent_state.trigger_sensor_03_data.has_timer_finished),
            float(response.agent_state.trigger_sensor_04_data.has_timer_finished),

            response.agent_state.trigger_sensor_01_data.timer_count,
            response.agent_state.trigger_sensor_02_data.timer_count,
            response.agent_state.trigger_sensor_03_data.timer_count,
            response.agent_state.trigger_sensor_04_data.timer_count,

            response.agent_state.trigger_sensor_01_data.max_timer_count,
            response.agent_state.trigger_sensor_02_data.max_timer_count,
            response.agent_state.trigger_sensor_03_data.max_timer_count,
            response.agent_state.trigger_sensor_04_data.max_timer_count,
        ])

        return state
    

    def render(self):
            
        # Visualize the trigger sensor data
        self._visualizer.visualize([
            self._response.agent_state.trigger_sensor_01_data,
            self._response.agent_state.trigger_sensor_02_data,
            self._response.agent_state.trigger_sensor_03_data,
            self._response.agent_state.trigger_sensor_04_data,
        ])


    def obersvation(self, state: np.ndarray) -> np.ndarray:
        return state
    
    def reward(self, state: np.ndarray, action: np.ndarray = None) -> float:
        return 0.0
    
    def terminated(self, state: np.ndarray) -> bool:
        return False
    
    def truncated(self, state: np.ndarray) -> bool:
        return False

    def info(self, state: np.ndarray) -> dict:
        return {}


def main():

    rclpy.init()

    env = TriggerSensorTestEnvironment()
    action = np.array([])

    while True:
        state, reward, terminated, truncated, info = env.step(action)
        env.render()

    env.close()


if __name__ == '__main__':
    main()

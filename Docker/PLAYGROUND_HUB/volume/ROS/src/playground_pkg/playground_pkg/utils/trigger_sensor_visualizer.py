

import numpy as np
from typing import List

import matplotlib.pyplot as plt
from interfaces_pkg.msg import TriggerSensor

class TriggerSensorVisualizer:

    def __init__(self):

        # Initialize matplotlib
        plt.style.use('seaborn-darkgrid')       # Use print(plt.style.available)) to see available styles
        plt.ion()

        # Initialize the plot (bar plot)
        self.fig, self.ax = plt.subplots()


    def visualize(self, trigger_sensors: List[TriggerSensor]):

        # Clear the plot
        self.ax.clear()

        # Extract the data from the trigger sensors
        has_triggered = [trigger_sensor.has_triggered for trigger_sensor in trigger_sensors]
        has_timer_finished = [trigger_sensor.has_timer_finished for trigger_sensor in trigger_sensors]
        timer_count = [trigger_sensor.timer_count for trigger_sensor in trigger_sensors]
        max_timer_count = [trigger_sensor.max_timer_count for trigger_sensor in trigger_sensors]

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
            range(len(trigger_sensors)),
            progress,
            height=0.5,
            color=['green' if finished else 'red' for finished in has_timer_finished],
        )

        # Set the y-axis ticks and labels
        self.ax.set_yticks(range(len(trigger_sensors)))
        self.ax.set_yticklabels([f"Sensor {i+1}" for i in range(len(trigger_sensors))])

        # Set the x-axis limits and labels
        self.ax.set_xlim(0, 100)
        self.ax.set_xlabel("Progress (%)")

        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
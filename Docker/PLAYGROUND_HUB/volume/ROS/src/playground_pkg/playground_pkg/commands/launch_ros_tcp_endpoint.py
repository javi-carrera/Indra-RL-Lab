# Project: Playground
# File: launch_ros_tcp_endpoint.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import os
import subprocess
import time
import yaml


def launch_ros_tcp_endpoint(n_environments: int = 1):

    username = os.environ.get('USERNAME')
    
    # Start environments
    print(f"Starting {n_environments} ROS TCP endpoint instances...")

    command = f"""
    bash -c "source /opt/ros/humble/setup.bash &&
             source /home/{username}/ROS/install/setup.bash &&
             ros2 launch ROS/launch/launch_ros_tcp_endpoint.py n_environments:={n_environments}"
    """

    p = subprocess.Popen(command, shell=True)


    print()

    try:

        # Keep the script running until all subprocesses are done
        while True:
            # Reduce CPU usage by limiting the check rate
            time.sleep(0.2)  

    except KeyboardInterrupt:

        print("Stopping all ROS environments...")

        # Sends SIGTERM on Unix, terminates process on Windows
        p.terminate()

        # Wait for processes to exit after termination signal
        p.wait()

        print("All ROS environments stopped.")


if __name__ == "__main__":

    config_file_path = "config.yml"

    with open(config_file_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)


    launch_ros_tcp_endpoint(n_environments=config['n_environments'])

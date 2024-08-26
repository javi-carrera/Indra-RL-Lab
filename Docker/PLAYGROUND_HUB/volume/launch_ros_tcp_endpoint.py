
import json
import subprocess
import time

def launch_ros_tcp_endpoint(n_environments: int = 1):
    
    # Start environments
    processes = []
    print(f"Starting {n_environments} ROS TCP endpoints...\n")

    for i in range(n_environments):

        port = 10000 + i

        command = f"""
        bash -c "source /opt/ros/humble/setup.bash &&
                source ros/install/setup.bash &&
                ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:={port}"
        """

        print(f"Starting ROS instance {i} on TCP port {port}")

        proc = subprocess.Popen(command, shell=True)
        processes.append(proc)

    try:

        # Keep the script running until all subprocesses are done
        while True:
            # Reduce CPU usage by limiting the check rate
            time.sleep(0.2)  

    except KeyboardInterrupt:

        print("Stopping all ROS environments...")

        # Sends SIGTERM on Unix, terminates process on Windows
        for p in processes:
            p.terminate()
            
        # Wait for processes to exit after termination signal
        for p in processes:
            p.wait()

        # Kill the processes if they are still running
        for p in processes:
            p.kill()

        # Wait for processes to exit after termination signal
        for p in processes:
            p.wait()

        print("All ROS environments stopped.")


if __name__ == "__main__":

    config_file_path = "ros_config.json"

    with open(config_file_path) as f:
        config = json.load(f)

    launch_ros_tcp_endpoint(n_environments=config['n_environments'])

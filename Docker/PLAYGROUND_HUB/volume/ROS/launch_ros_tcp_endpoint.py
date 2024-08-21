import subprocess
import time
import sys
import json

def main():

    # Load the configuration file
    with open("config.json", "r") as f:
        config = json.load(f)

    n_environments = config["n_environments"]

    processes = []
    
    # Start environments
    print(f"Starting {n_environments} ROS environments...")

    for i in range(n_environments):

        port = 10000 + i

        command = f"""
        bash -c "source /opt/ros/humble/setup.bash &&
                 source install/setup.bash &&
                 ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:={port}"
        """

        print(f"Starting ROS instance {i} on TCP port {port}")

        proc = subprocess.Popen(command, shell=True)
        processes.append(proc)

    try:

        # Keep the script running until all subprocesses are done
        while True:

            all_exited = all(p.poll() is not None for p in processes)
            if all_exited:
                break
            
            # Reduce CPU usage by limiting the check rate
            time.sleep(0.5)  

    except KeyboardInterrupt:

        print("Stopping all ROS environments...")

        # Sends SIGTERM on Unix, terminates process on Windows
        for p in processes:
            p.terminate()
        
        # Wait for processes to exit after termination signal
        for p in processes:
            p.wait()

        print("All ROS environments stopped.")

if __name__ == "__main__":
    main()

import subprocess
import time
import json

def launch_node(package_name: str, node_name: str):
    
    # Start environments
    print(f"Starting {package_name} {node_name}...\n")

    command = f"""
    bash -c "source /opt/ros/humble/setup.bash &&
                source ros/install/setup.bash &&
                ros2 run {package_name} {node_name}"
    """

    p = subprocess.Popen(command, shell=True)

    try:

        # Keep the script running until all subprocesses are done
        while True:
            # Reduce CPU usage by limiting the check rate
            time.sleep(0.2)  

    except KeyboardInterrupt:

        print(f"Stopping {package_name} {node_name}...")

        # Sends SIGTERM on Unix, terminates process on Windows
        p.terminate()
        
        # Wait for processes to exit after termination signal
        p.wait()

        print(f"{package_name} {node_name} stopped.")



if __name__ == "__main__":

    config_file_path = "ros_config.json"

    with open(config_file_path) as f:
        config = json.load(f)

    launch_node(
        package_name=config['package_name'],
        node_name=config['node_name']
    )

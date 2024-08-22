import subprocess
import time
import sys
import json


class ROSLauncher:

    @staticmethod
    def launch_node(package_name: str, node_name: str):
        
        # Start environments
        print(f"Starting {package_name} {node_name}...")

        command = f"""
        bash -c "source /opt/ros/humble/setup.bash &&
                    source ros/install/setup.bash &&
                    ros2 run {package_name} {node_name}"
        """

        p = subprocess.Popen(command, shell=True)

        try:

            # Keep the script running until all subprocesses are done
            while True:

                # exited = p.poll() is not None
                # if exited:
                #     break
                
                # Reduce CPU usage by limiting the check rate
                time.sleep(0.2)  

        except KeyboardInterrupt:

            print(f"Stopping {package_name} {node_name}...")

            # Sends SIGTERM on Unix, terminates process on Windows
            p.terminate()
            
            # Wait for processes to exit after termination signal
            p.wait()

            print(f"{package_name} {node_name} stopped.")


    @staticmethod
    def launch_ros_tcp_endpoint(n_environments: int = 1):
        
        # Start environments
        processes = []
        print(f"Starting {n_environments} ROS environments...")

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

                # all_exited = all(p.poll() is not None for p in processes)
                # if all_exited:
                #     break
                
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

            print("All ROS environments stopped.")



import subprocess
import time
import sys
import json
from ros_launcher import ROSLauncher

if __name__ == "__main__":

    config_file_path = "ros_config.json"

    with open(config_file_path) as f:
        config = json.load(f)

    ros_launcher = ROSLauncher()

    ros_launcher.launch_node(
        package_name=config['package_name'],
        node_name=config['node_name']
    )

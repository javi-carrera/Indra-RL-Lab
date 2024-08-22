from ros_launcher import ROSLauncher
import json

if __name__ == "__main__":

    config_file_path = "ros_config.json"

    with open(config_file_path) as f:
        config = json.load(f)

    ros_launcher = ROSLauncher()

    ros_launcher.launch_ros_tcp_endpoint(n_environments=config['n_environments'])

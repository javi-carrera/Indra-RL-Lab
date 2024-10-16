# Project: Indra-RL-Lab
# File: lauch_ros_tcp_endpoint.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path))

    n_environments = config['environment']['n_environments']
    nodes = []

    for i in range(n_environments):

        port = 10000 + i

        node = Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': port}],
        )

        nodes.append(node)

    return LaunchDescription(nodes)

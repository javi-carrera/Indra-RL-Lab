# Project: Playground
# File: lauch_ros_tcp_endpoint.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    username = os.environ.get('USERNAME')

    config_file_path = f"/home/{username}/config.yml"

    with open(config_file_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    n_environments = config['n_environments']
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

# Project: Playground
# File: lauch_ros_tcp_endpoint.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rl_pkg',
            executable='deploy',
            output='screen',
        )
    ])

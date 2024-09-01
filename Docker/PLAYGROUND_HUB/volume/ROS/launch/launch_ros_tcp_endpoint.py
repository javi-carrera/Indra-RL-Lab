# Project: Playground
# File: autonomous_navigation_example_environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_nodes(context, *args, **kwargs):

    n_environments = int(context.launch_configurations['n_environments'])
    nodes = []

    for i in range(n_environments):

        port = 10000 + i

        node = Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name=f'ros_tcp_endpoint_{i}',
            parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': port}]
        )

        nodes.append(node)

    return nodes


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'n_environments',
            default_value='1',
            description='Number of environments to start'
        ),
        OpaqueFunction(function=launch_nodes)
    ])

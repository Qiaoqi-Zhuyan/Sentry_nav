import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nav_client'), 'config', 'socket_param.yaml')

    test_socket_node = Node(
        package='nav_client',
        executable='test_nav_socket_node',
        namespace='',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([test_socket_node])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav_bringup')
    rm_vision_launch_dir = os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch')
    vision_client_dir = os.path.join(get_package_share_directory('nav_client'), 'launch')

    start_rm_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rm_vision_launch_dir, 'vision_bringup.launch.py'))
    )

    start_vision_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_client_dir, 'vision_client_bringup.launch.py'))
    )



    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(start_rm_vision)
    # ld.add_action(start_vision_client)

    return ld

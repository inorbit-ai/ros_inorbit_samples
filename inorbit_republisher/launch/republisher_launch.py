# Read online that is more recomended to execute a python-based file for ROS2
#This file attends to launch republisher.py file, with example.yaml file as a confir parameter.
# inorbit_republisher/launch/republisher_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    package_name = 'inorbit_republisher'
    package_share = get_package_share_directory(package_name)

    # Path to the configuration file
    config_file = os.path.join(package_share, 'config', 'example.yaml')

    # Define the node
    republisher_node = Node(
        package=package_name,
        executable='/resource/republisher',         
        name='inorbit_republisher',
        output='screen',
        parameters=[config_file]           # Load parameters from the YAML file
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        republisher_node
    ])


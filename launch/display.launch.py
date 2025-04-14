#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of your package
    pkg_share = get_package_share_directory('my_package')

    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'quad.urdf.xacro')

    # Command to convert xacro to URDF
    robot_description_command = ['xacro ', xacro_file]

    return LaunchDescription([
        # Load the robot_description parameter by converting the xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': 
                # Use command-line substitution to load URDF from xacro
                os.popen('xacro '+xacro_file).read()
            }]
        ),

        # Launch RViz2 with a preset configuration file (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'launch', 'quad_config.rviz')]
        )
    ])

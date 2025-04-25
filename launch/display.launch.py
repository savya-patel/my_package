#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package share and xacro path
    pkg_share = get_package_share_directory('my_package')
    xacro_file = os.path.join(pkg_share, 'urdf', 'quad.urdf.xacro')

    # Prepare robot_description by converting xacro
    robot_description = {
        'robot_description': os.popen('xacro ' + xacro_file).read()
    }

    # Declare use_sim_time argument (enabled by default)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Enable sim time for all nodes
        use_sim_time_arg,

        # Publish robot_description and TF for the model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Static transform from world to base_link so RViz has a root frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_base',
            output='screen',
            arguments=[
                '0', '0', '0',  # x y z
                '0', '0', '0',  # roll pitch yaw
                'world',       # parent frame
                'base_link'    # child frame
            ]
        ),

        # Launch RViz2 with view_quad.rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', os.path.join(pkg_share, 'config', 'view_quad.rviz')]
        )
    ])

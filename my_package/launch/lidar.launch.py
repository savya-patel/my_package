#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_package')

    # Paths to your world and RViz config
    world_file = os.path.join(pkg, 'worlds', 'lidar_demo.world')
    rvizcfg    = os.path.join(pkg, 'launch', 'psuedo_lidar.rviz')

    return LaunchDescription([
        Node(
            package='my_package',
            executable='world_maker_publisher.py',
            name='world_marker_pub',
            output='screen'
        ),

        # 2) Fake LiDAR publisher
        Node(
            package='my_package',
            executable='psuedo_rplidar_node.py',
            name='fake_lidar',
            output='screen'
        ),

        # 3) Static TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf',
            arguments=[ '0.05', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame' ]
        ),

        # 4) RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'view_lidar.rviz')]
            # Launch RViz2 with view_quad.rviz
        ),
    ])

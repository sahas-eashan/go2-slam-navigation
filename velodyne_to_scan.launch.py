#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/velodyne_points'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                'min_height': -0.3,       # Filter ground points
                'max_height': 0.3,        # Filter ceiling/high points
                'angle_min': -3.14159,    # -180 degrees
                'angle_max': 3.14159,     # +180 degrees
                'angle_increment': 0.0087, # ~0.5 degrees (360°/720 points)
                'scan_time': 0.1,         # Expected scan time
                'range_min': 0.9,         # Minimum range (meters)
                'range_max': 100.0,       # Maximum range (meters)
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
    ])

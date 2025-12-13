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
                # Use the sensor frame to avoid scan skew from base motion
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                # Keep a modest vertical slice to reject floor/ceiling
                'min_height': 0.05,
                'max_height': 0.30,
                'angle_min': -3.14159,       # -180 degrees
                'angle_max': 3.14159,        # +180 degrees
                'angle_increment': 0.0087,   # ~0.5 degree resolution
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 5.0,
                # Publish inf for missing returns so SLAM treats them as free space
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
    ])

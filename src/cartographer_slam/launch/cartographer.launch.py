#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='go2_2d.lua')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value='go2_2d.lua',
            description='Cartographer configuration file'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', slam_config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[('scan', '/scan')]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', '0.05',
                '-publish_period_sec', '0.5'
            ]
        ),
    ])

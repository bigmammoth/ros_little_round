#!/usr/bin/env python3
"""
ROS 2 launch file for the little_chassis node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('little_chassis'),
                'params',
                'chassis.yaml',
            ]),
            description='Full path to the little_chassis parameter file.',
        ),
        Node(
            package='little_chassis',
            executable='little_chassis',
            name='little_chassis',
            output='screen',
            parameters=[params_file],
        ),
    ])

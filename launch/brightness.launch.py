#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    brightness_beta_arg = DeclareLaunchArgument(
        'brightness_beta',
        default_value='80',
        description='Brightness adjustment value (0-255)'
    )

    # Node
    brightness_node = Node(
        package='cv_usb_cam',
        executable='brightness.py',
        name='brightness_adjuster',
        output='screen',
        parameters=[{
            'brightness_beta': LaunchConfiguration('brightness_beta'),
        }]
    )

    return LaunchDescription([
        brightness_beta_arg,
        brightness_node,
    ])

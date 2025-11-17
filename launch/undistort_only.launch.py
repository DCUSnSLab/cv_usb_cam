#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Raw image topic'
    )

    undistorted_topic_arg = DeclareLaunchArgument(
        'undistorted_topic',
        default_value='/camera/image_undistorted',
        description='Undistorted image topic'
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value=[
            'file://',
            PathJoinSubstitution([
                FindPackageShare('cv_usb_cam'),
                'camera_info',
                'cam.yaml'
            ])
        ],
        description='Camera info URL'
    )

    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera',
        description='Camera frame ID'
    )

    # Node
    image_undistorter_node = Node(
        package='cv_usb_cam',
        executable='image_undistorter',
        name='image_undistorter',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'undistorted_topic': LaunchConfiguration('undistorted_topic'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        undistorted_topic_arg,
        camera_info_url_arg,
        camera_frame_id_arg,
        image_undistorter_node,
    ])

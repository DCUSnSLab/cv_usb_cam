#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='Video device path'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1920',
        description='Image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='1080',
        description='Image height'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='20',
        description='Camera framerate'
    )

    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='ardu_cam_link',
        description='Camera frame ID'
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

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/ardu_cam_link/image_raw',
        description='Raw image topic'
    )

    undistorted_topic_arg = DeclareLaunchArgument(
        'undistorted_topic',
        default_value='/ardu_cam_link/image_undistorted',
        description='Undistorted image topic'
    )

    # Nodes
    opencv_usb_cam_node = Node(
        package='cv_usb_cam',
        executable='opencv_usb_cam',
        name='cv_usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }]
    )

    image_undistorter_node = Node(
        package='cv_usb_cam',
        executable='image_undistorter',
        name='image_undistorter',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'undistorted_topic': LaunchConfiguration('undistorted_topic'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }]
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        camera_frame_id_arg,
        camera_info_url_arg,
        image_topic_arg,
        undistorted_topic_arg,
        opencv_usb_cam_node,
        image_undistorter_node,
    ])

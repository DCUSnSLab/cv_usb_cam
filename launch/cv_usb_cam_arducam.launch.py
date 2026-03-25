#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='960',
        description='Image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='540',
        description='Image height'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='15',
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

    enable_undistorter_arg = DeclareLaunchArgument(
        'enable_undistorter',
        default_value='false',
        description='Whether to launch the image undistorter node'
    )

    enable_hsv_filter_arg = DeclareLaunchArgument(
        'enable_hsv_filter',
        default_value='false',
        description='Enable HSV color range filtering on image_raw output'
    )

    publish_hsv_debug_topics_arg = DeclareLaunchArgument(
        'publish_hsv_debug_topics',
        default_value='true',
        description='Publish /image_hsv_filtered and /image_hsv_mask when HSV filter is enabled'
    )

    h_min_arg = DeclareLaunchArgument('h_min', default_value='0', description='HSV H min (0-179)')
    h_max_arg = DeclareLaunchArgument('h_max', default_value='179', description='HSV H max (0-179)')
    s_min_arg = DeclareLaunchArgument('s_min', default_value='0', description='HSV S min (0-255)')
    s_max_arg = DeclareLaunchArgument('s_max', default_value='255', description='HSV S max (0-255)')
    v_min_arg = DeclareLaunchArgument('v_min', default_value='0', description='HSV V min (0-255)')
    v_max_arg = DeclareLaunchArgument('v_max', default_value='255', description='HSV V max (0-255)')

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
            'enable_hsv_filter': LaunchConfiguration('enable_hsv_filter'),
            'publish_hsv_debug_topics': LaunchConfiguration('publish_hsv_debug_topics'),
            'h_min': LaunchConfiguration('h_min'),
            'h_max': LaunchConfiguration('h_max'),
            's_min': LaunchConfiguration('s_min'),
            's_max': LaunchConfiguration('s_max'),
            'v_min': LaunchConfiguration('v_min'),
            'v_max': LaunchConfiguration('v_max'),
        }]
    )

    image_undistorter_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_undistorter')),
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
        enable_undistorter_arg,
        enable_hsv_filter_arg,
        publish_hsv_debug_topics_arg,
        h_min_arg,
        h_max_arg,
        s_min_arg,
        s_max_arg,
        v_min_arg,
        v_max_arg,
        opencv_usb_cam_node,
        image_undistorter_node,
    ])

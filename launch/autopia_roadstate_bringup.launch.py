#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to an ARSD rosbag directory or .mcap file'
    )

    rviz_config = Path(get_package_share_directory('autopia_roadstate')) / 'rviz' / 'autopia_roadstate.rviz'

    static_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('autopia_roadstate'), 'launch'),
            '/static_transform.launch.py'])
    )

    ros2bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        ros2bag_play,
        static_transform_launch,
        Node(namespace='rviz2', package='rviz2', executable='rviz2',
             arguments=[str(rviz_config)]),
    ])

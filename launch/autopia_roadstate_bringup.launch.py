#!/usr/bin/env python
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import ExecuteProcess


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    rviz_config = Path(get_package_share_directory('autopia_roadstate')) / 'rviz' / 'autopia_roadstate.rviz'

    static_transform_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('autopia_roadstate'), 'launch'),
         '/static_transform.launch.py'])
      )

    ros2bag_launch =  LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    "<path/to/dataset>",
                ],
                output="screen",
            )
        ]
    )


    return LaunchDescription([
        ros2bag_launch,
        static_transform_launch,
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=str(rviz_config))
    ])

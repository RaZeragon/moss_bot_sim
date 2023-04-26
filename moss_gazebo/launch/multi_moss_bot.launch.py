#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_moss_gazebo = get_package_share_directory('moss_gazebo')
    pkg_moss_description = get_package_share_directory('moss_description')
    pkg_moss_control = get_package_share_directory('moss_control')

    # Start World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moss_gazebo, 'launch', 'start_world.launch.py')
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moss_description, 'launch', 'multi_spawn_moss_bot.launch.py')
        )
    )

    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moss_control, 'launch', 'controller.launch.py')
        )
    )     

    return LaunchDescription([
        start_world,
        spawn_robot_world,
        spawn_robot_control
    ])
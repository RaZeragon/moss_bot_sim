#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import random

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def gen_robot_list(number_of_robots):
    robots = []
    positions = random.sample(range(-5, 5), 8)

    for i in range(number_of_robots):
        robot_name = "moss_bot"+str(i)
        x_pos = float(positions[i])
        y_pos = float(positions[i + 1])
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.01})

    return robots 

def generate_launch_description():
    default_swarm_size = 4

    pkg_moss_description = get_package_share_directory('moss_description')
    pkg_moss_gazebo = get_package_share_directory('moss_gazebo')
    pkg_moss_control = get_package_share_directory('moss_control')

    urdf = os.path.join(pkg_moss_description, 'urdf/', 'box_bot.urdf')
    assert os.path.exists(urdf), "The moss_bot.urdf doesnt exist in "+str(urdf)

    for arg in sys.argv:
        if arg.startswith("swarm_amount:="):
            swarm_amount = int(arg.split(":=")[1])
        else:
            swarm_amount = default_swarm_size

    # Names and poses of the robots
    robots = gen_robot_list(swarm_amount)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []

    print(str(robots))
    for robot in robots:
        print("#############"+str(robot))
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_moss_description, 'launch', '_spawn_moss_bot.launch.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld
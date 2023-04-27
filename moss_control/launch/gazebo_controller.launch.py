import os
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# Generates a list of robot names to use for namespaces
def gen_name_list(number_of_robots):
    name = []

    for i in range(number_of_robots):
        robot_name = "moss_bot"+str(i)
        name.append({'name': robot_name})

    return name

def generate_launch_description():
    # Setting default values
    default_swarm_size = 4
    default_square_size = 5

    pkg_moss_description = get_package_share_directory('moss_description')
    pkg_moss_gazebo = get_package_share_directory('moss_gazebo')
    pkg_moss_control = get_package_share_directory('moss_control')

    # Parsing arguments ran with the ros2 launch command
    # swarm_amount:=#
    # square_size:=#
    for arg in sys.argv:
        if arg.startswith("swarm_amount:="):
            swarm_amount = int(arg.split(":=")[1])
        elif arg.startswith("square_size:="):
            square_size = int(arg.split(":=")[1])
        else:
            swarm_amount = default_swarm_size
            square_size = default_square_size

    names = gen_name_list(swarm_amount)

    spawn_estimator_cmds = []
    spawn_square_cmds = []
    spawn_corner_cmds = []
    spawn_voter_cmds = []

    # For each robot in the swarm, launch the following nodes with 
    # specific arguments like namespace, swarm size, and square size
    print(str(names))
    for name in names:
        print("#############"+str((name)))
        spawn_estimator_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_moss_control, 'launch', '_gazebo_estimator.launch.py')),
                launch_arguments={
                                  'robot_name': name['name'],
                                  'robot_namespace': name['name']
                                  }.items()))
        spawn_square_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_moss_control, 'launch', '_gazebo_square.launch.py')),
                launch_arguments={
                                  'robot_name': name['name'],
                                  'robot_namespace': name['name'],
                                  'swarm_amount': str(swarm_amount),
                                  'square_size': str(square_size)
                                  }.items()))
        spawn_corner_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_moss_control, 'launch', '_gazebo_corner.launch.py')),
                launch_arguments={
                                  'robot_name': name['name'],
                                  'robot_namespace': name['name'],
                                  'swarm_amount': str(swarm_amount)
                                  }.items()))
        # spawn_voter_cmds.append(
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(os.path.join(pkg_box_bot_control, 'launch', '_gazebo_voter.launch.py')),
        #         launch_arguments={
        #                           'robot_name': name['name'],
        #                           'robot_namespace': name['name'],
        #                           'swarm_amount': str(swarm_amount)
        #                           }.items()))

    ld = LaunchDescription()
    
    for spawn_estimator_cmd in spawn_estimator_cmds:
        ld.add_action(spawn_estimator_cmd)
    for spawn_square_cmd in spawn_square_cmds:
        ld.add_action(spawn_square_cmd)
    for spawn_corner_cmd in spawn_corner_cmds:
        ld.add_action(spawn_corner_cmd)
    # for spawn_voter_cmd in spawn_voter_cmds:
    #     ld.add_action(spawn_voter_cmd)

    return ld
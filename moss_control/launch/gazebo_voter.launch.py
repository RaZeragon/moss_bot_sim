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

    pkg_moss_description = get_package_share_directory('moss_description')
    pkg_moss_gazebo = get_package_share_directory('moss_gazebo')
    pkg_moss_control = get_package_share_directory('moss_control')

    # Parsing arguments ran with the ros2 launch command
    # swarm_amount:=#
    for arg in sys.argv:
        if arg.startswith("swarm_amount:="):
            swarm_amount = int(arg.split(":=")[1])
        else:
            swarm_amount = default_swarm_size

    names = gen_name_list(swarm_amount)

    spawn_voter_cmds = []
    spawn_movement_cmds = []

    # For each robot in the swarm, launch the following nodes with 
    # specific arguments like namespace, swarm size, and square size
    print(str(names))
    for name in names:
        print("#############"+str((name)))
        spawn_voter_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_moss_control, 'launch', '_gazebo_voter.launch.py')),
                launch_arguments={
                                  'robot_name': name['name'],
                                  'robot_namespace': name['name'],
                                  'swarm_amount': str(swarm_amount)
                                  }.items()))
        spawn_movement_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_moss_control, 'launch', '_gazebo_movement.launch.py')),
                launch_arguments={
                                  'robot_name': name['name'],
                                  'robot_namespace': name['name'],
                                  'swarm_amount': str(swarm_amount)
                                  }.items()))

    ld = LaunchDescription()
    
    for spawn_voter_cmd in spawn_voter_cmds:
        ld.add_action(spawn_voter_cmd)
    for spawn_movement_cmd in spawn_movement_cmds:
        ld.add_action(spawn_movement_cmd)

    return ld
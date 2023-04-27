from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='moss_control',
            executable='gazebo_vote',
            output='screen',
            arguments=[
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '--swarm_amount', launch.substitutions.LaunchConfiguration('swarm_amount')]),
    ])
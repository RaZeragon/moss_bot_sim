# gazebo_simulation

## 1. Introduction

&ensp;&ensp;&ensp;&ensp;This repository contains simulation models for the Modular Open-Source Swarm Project. The development and test environment is as follows:
- Ubuntu 20.04 + ROS Foxy

&ensp;&ensp;&ensp;&ensp;For the repository that controls real world movement, please go to the control_system repository.

## 2. Update History
- (2023-05-09) Updated README.md and added package descriptions

## 3. Installation
- ### 3.1 Install [ROS2 Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html) 

- ### 3.2 Install [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)  

- ### 3.3 Create a workspace
    ```bash
    # Skip this step if you already have a target workspace
    $ cd ~
    $ mkdir -p dev_ws/src
    ```

- ### 3.4 Obtain source code of "gazebo_simulation" repository
    ```bash
    # Remember to source ros2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/src
    $ git clone https://github.com/modularopensourceadaptiveswarmrobotics/gazebo_simulation.git
    ```

- ### 3.5 Update "gazebo_simulation" repository 
    ```bash
    $ cd ~/dev_ws/src/gazebo_simulation
    $ git pull
    ```

- ### 3.6 Install dependencies
    ```bash
    # Remember to source ros2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- ### 3.7 Build gazebo_simulation
    ```bash
    # Remember to source ros2 and moveit2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/
    # build all packages
    $ colcon build
    
    # build selected packages
    $ colcon build --packages-select moss_gazebo
    ```

## 4. Package Introduction

__Reminder 1: If there are multiple people using ros2 in the current LAN, in order to avoid mutual interference, please set ROS_DOMAIN_ID__
  - [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Concepts/About-Domain-ID.html)

__Reminder 2： Remember to source the environment setup script before running any applications in gazebo_simulation__

```bash
$ cd ~/dev_ws/
$ source install/setup.bash
```

- ### 4.1 moss_description
    This package contains robot description files and 3D models of the MOSS robot and other robots used in the simulation. Models can be displayed in RViz by the following launch file:
    ```bash
    $ cd ~/dev_ws/
    $ ros2 launch moss_control moss_bot_rviz.launch.py
    ```


- ### 4.2 moss_msgs  
    This package contains all interface definitions for gazebo_simulation and is currently under development.

- ### 4.3 moss_gazebo
    This package is for supporting the MOSS robot simulation with Gazebo. The simulation can be started with the following launch file:
    ```bash
    $ cd ~/dev_ws/
    $ ros2 launch moss_gazebo multi_moss_bot.launch.py
    ```

- ### 4.4 moss_driver
    This package is for driving the lower level hardware of the MOSS robot. This package is currently under development and may or may not be moved to the control_system repository.

- ### 4.5 moss_control
    This package is for controlling the MOSS robots and performing formation control. After launching the moss_gazebo simulation in one terminal, launch another terminal and use the following command to make the robots form a square.

    ```bash
    # Remember to rebuild, source ROS2, and source the package again.multi_moss_bot
    $ cd ~/dev_ws/
    # set 'square_size' equal to a positive integer to determine the side length of the resultant square (default = 5)
    $ ros2 launch moss_control gazebo_voter.launch.py [square_size:=5]
    ```

- ## 5. References
  - [Spawning multiple robots in Gazebo with ROS2 by The Construct](https://www.theconstructsim.com/spawning-multiple-robots-in-gazebo-with-ros2/)
  - [How to Simulate a Robot Using Gazebo and ROS 2 by Automatic Addison](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/#Test_Your_ROS_2_and_Gazebo_Integration)
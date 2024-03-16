# Autonomous RC Car - MSR Winter Project

This repository contains a collection of C++ ROS 2  packages for controlling an autonomous RC Ackermann robot to drag race and follow paths autonomously, as well as drive with tele-op control. These packages are designed for use with Isaac ROS Humble on the Nvidia Jetson Orin Nano.

These packages were developed for a custom autonomous version of the <a href="https://www.reddit.com/r/EngineeringNS/comments/zvellk/tarmo5/" target="_blank"><u>Tarmo5</u></a> RC car project. The car was modified to include an Intel RealSense d435i camera and IMU, a RPLidar A1 360 scanning lidar, and an AS5048a encoder for feedback. Though these packages were designed with this specific robot in mind, they could easily be modified for other Ackermann robots with similar sensor suites. 

## Package List
- rviz_description: The URDF description of the robot and simulation files for Isaac Sim.
- car_control: Nodes for controlling the speed and direction of the robot, teleop control, and launching Slam Toolbox
- racing: Nodes for drag racing, mapping, and point to point racing

## Setup and Building 
This repository should be cloned into the source folder of a ROS 2 workspace on the robot itself. Optionally, it can also be cloned on a client PC for visualization purposes. However this is not required; all launch files and nodes can be launched from a client PC using SSH, and once launched, all critical functionality can be controlled using an Xbox Controller connected directly to the robot. 

Once the repo is cloned, build the packages in the workspace by simply using `colcon build`. Building takes ~1 minutes on the Jetson. Cross compilation can be used to accelerate this process if desired. 

## Xbox Controller
Many of the nodes in this repo can be controlled with an Xbox Controller by using the `controller_interface` node. This node, and the `joy` node which is also required to be running, can be launched with the command `ros2 launch car_control controller.launch.xml`. Note that this launches the controller only, not the car.

Once the controller is launched, use the following buttons and sticks to control the robot:
- Right Trigger - forward throttle
- Left Trigger - reverse throttle (is overridden by any forward throttle command)
- Left Stick - steering control
- Right Bumper - enable publishing throttle and steering commands from the `controller_interface` node
- Left Bumper - disable publishing throttle and steering commands from the `controller_interface` node
- A - calls the `enable_drive` service to enable the drive motor
- B - calls the `enable_drive` service to disable the drive motor
- X - calls the `start_race` service to start a race
- Y - calls the `reset_odom` service to reset all odometry to the initial state
- Start - calls the `slam_toolbox/serialize_map` service to save the current map when mapping with Slam Toolbox
- Select - calls the `save_path` service to save the current path when creating a path for a point-to-point race


## Launching Tele-Op Control


## Launching Drag Racing Control


## Mapping and Racing a Path
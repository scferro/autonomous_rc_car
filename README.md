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

## Visualize in RViz
To visualize the URDF in RViz, the command `ros2 launch rviz_description car_rviz.launch.py` can be used. If you would like to manually adjust the joints in the visualization, use `ros2 launch rviz_description car_rviz.launch.py use_jsp:=jsp_gui`.

## Xbox Controller
Many of the nodes in this repo can be controlled with an Xbox Controller by using the `controller_interface` node. This node, and the `joy` node which is also required to be running, can be launched with the command `ros2 launch car_control controller.launch.xml`. Note that this launches the controller only, not the car.

Once the controller is launched, use the following buttons and sticks to control the robot:
- Right Trigger - forward throttle
- Left Trigger - reverse throttle (is overridden by any forward throttle command)
- Left Stick - steering control
- Right Bumper - enable publishing throttle and steering commands from the `controller_interface` node
- Left Bumper - disable publishing throttle and steering commands from the `controller_interface` node
- A - calls the `enable_drive` service to enable the drive motor
- B - calls the `enable_drive` service to disable the drive motor (I use this to be the kill button when testing)
- X - calls the `start_race` service to start a race
- Y - calls the `reset_odom` service to reset all odometry to the initial state
- Start - calls the `slam_toolbox/serialize_map` service to save the current map when mapping with Slam Toolbox
- Select - calls the `save_path` service to save the current path when creating a path for a point-to-point race

## Launching Tele-Op Control
To launch tele-op control, use the command  `ros2 launch car_control drive_car.launch.xml`. This will launch the controller launch file and all the required nodes for controlling the car. Note that by default, the robot will launch with the controller in `disabled` mode, meaning it will not publish commands. Simply press the right bumper to start publishing.

## Launching Drag Racing Control
There are two types of drag racing possible with this package, timed drag racing and distance racing. Timed races run for a set time and can be launched with the command `ros2 launch racing drag_race_time.launch.xml race_time:=5.0` where `race_time:=5.0` can be replaced with the desired race time in seconds. 

Distance races use Slam Toolbox to localize the robot, and run until the robot has covered a specified distance. They can be launched with the command `ros2 launch racing drag_race_distance.launch.xml race_distance:=5.0` where `race_distance:=5.0` can be replaced with the desired race distance in meters.

Once the race launch file is running, press the X button on the controller to start the race. Overall, the timed drag races are far more reliable, as Slam Toolbox has issues with poor odometry due to wheel spin from launching the car.

## Mapping and Racing a Path
Mapping and racing a path requires three steps. First, you need to map the environment using Slam Toolbox. To do this, use the command `ros2 launch car_control drive_with_slam.launch.xml` to begin mapping. This will launch the robot in slow_mode, which limits throttle commands and ramping speed to ensure smooth operation when mapping. It is recommended to make several passes around the environment to ensure you achieve loop closure. This will ensure you have the most accurate map possible. Once your environment is mapped, press the Start button on the controller to save the map.

Next, you need to plan a path for the robot to follow. To do this, run the command `ros2 launch racing plan_path.launch.xml`. This will launch slam toolbox in localization mode (faster, but no new mapping) and the robot will again be launched in slow mode. To create a path, simply drive the robot along the path you want it to follow. The robot will track it's path by adding poses to a ROS2 Path message. Once you have completed the path, press the select button on the controller to save it as a CSV file.

Finally, run `ros2 launch racing race_path.launch.xml` to race the path. The robot should first manually be reset to it's initial position. To begin the race, simply press the X button on the controller. The robot should now follow the path to the end position.
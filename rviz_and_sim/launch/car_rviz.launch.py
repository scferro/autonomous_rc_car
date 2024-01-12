"""
Starts all the nodes to visualize a robot in rviz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="use_jsp", default_value="gui",
                              description="gui (default): use jsp_gui, jsp: use joint_state_publisher, none: no joint states published"),

        DeclareLaunchArgument(name="use_rviz", default_value="true",
                              description="true (default): start rviz, otherwise don't start rviz"),

        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui",
             condition= LaunchConfigurationEquals("use_jsp", "gui")
             ),
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             condition= LaunchConfigurationEquals("use_jsp", "jsp")
             ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                 Command([TextSubstitution(text="xacro "),
                          PathJoinSubstitution(
                              [FindPackageShare("rviz_and_sim"), "urdf", "car.urdf.xacro"])])}
            ]
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d",
                       PathJoinSubstitution(
                           [FindPackageShare("rviz_and_sim"), "config", "car_urdf.rviz"])],
            condition=LaunchConfigurationEquals("use_rviz", "true")
            )
        ])

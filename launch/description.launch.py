#!/usr/bin/env python3
import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    simulation = LaunchConfiguration("simulation", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default=simulation)
    urdf_file_name = "custom_iris.xacro"

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory("uav_inspections_ros2"),"urdf",
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true"),

        DeclareLaunchArgument(
            "simulation",
            default_value="false",
            description="It won't start the joint state publisher and will use sim time if true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "publish_frequency": 20.0, "ignore_timestamp": False}]),
        
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("simulation")))
        ]
    )


#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory("my_drone_sim")
    world_file = os.path.join(pkg_share, "worlds", "drone_scenario.sdf")
    model_file = os.path.join(pkg_share, "models", "x500_gimbal", "model.sdf")
    bridge_config_file = os.path.join(pkg_share, "config", "bridge_topics.yaml")

    # Gazebo Sim command
    gazebo_cmd = ExecuteProcess(cmd=["gz", "sim", "-r", world_file], output="screen")

    # Spawn drone after a short delay
    spawn_drone_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", model_file, "-x", "0", "-y", "0", "-z", "0.1"],
        output="screen",
    )

    # ROS-GZ Bridge command with parameter file
    bridge_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "--ros-args",
            "-p",
            f"config_file:={bridge_config_file}",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_cmd,
            TimerAction(
                period=1.0, actions=[spawn_drone_cmd]
            ),  # Delay to ensure Gazebo loads first
            TimerAction(
                period=2.0, actions=[bridge_cmd]
            ),  # Slight delay to start the bridge after spawn
        ]
    )

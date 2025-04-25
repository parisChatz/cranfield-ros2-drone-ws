#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for spawning a drone in a Gazebo simulation environment using ROS 2 and ROS-GZ.

    This launch file allows users to spawn a specified drone model into a given Gazebo world,
    with optional headless mode and custom spawn coordinates. It supports launching Gazebo
    with or without GUI, spawning the drone via `ros_gz_sim`, and optionally running a
    ROS-GZ bridge for topic communication.

    Launch Arguments:
        headless_gazebo (bool):
            Whether to launch Gazebo in headless (no GUI) mode. Default is "false".

        world (str):
            Absolute path to the Gazebo world file. Default is the drone_scenario2.sdf
            file within the `my_drone_sim` package.

        model (str):
            Absolute path to the drone model SDF file. Default is the x500_gimbal model
            from the `my_drone_sim` package.

        x (float):
            X coordinate for the drone's initial spawn position. Default is "0".

        y (float):
            Y coordinate for the drone's initial spawn position. Default is "0".

        z (float):
            Z coordinate for the drone's initial spawn position. Default is "0.1".

    Usage:
        ros2 launch my_drone_sim my_world.launch.py [arguments]

    Examples:
        ros2 launch my_drone_sim my_world.launch.py headless_gazebo:=true x:=2.0 y:=1.0 z:=1.5

    Note:
        The ROS-GZ bridge section is included in the file but commented out cause it slows down
        the simulation. Uncomment it to enable parameter bridging with the specified YAML config.
    """

    # Package paths
    pkg_share = get_package_share_directory("my_drone_sim")
    world_file = os.path.join(pkg_share, "worlds", "drone_scenario2.sdf")
    model_file = os.path.join(pkg_share, "models", "x500_gimbal", "model.sdf")
    bridge_config_file = os.path.join(pkg_share, "config", "bridge_topics.yaml")

    # Gazebo Sim command
    gazebo_cmd = ExecuteProcess(cmd=["gz", "sim", world_file], output="screen")

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

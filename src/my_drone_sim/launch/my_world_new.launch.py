import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ----------------------------------------------------------------
    # Declare launch arguments
    # ----------------------------------------------------------------
    model_name = "x500"
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="If true, run gz sim in headless rendering mode",
    )
    # Default model file in your package; override by passing model:=/full/path/to/model.sdf
    default_model = os.path.join(
        get_package_share_directory("my_drone_sim"), "models", model_name, "model.sdf"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Full path to the drone model SDF to spawn",
    )
    x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial X position for the spawned drone"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="Initial Y position for the spawned drone"
    )
    z_arg = DeclareLaunchArgument(
        "z",
        default_value="0.01",
        description="Initial Z position for the spawned drone",
    )

    # ----------------------------------------------------------------
    # LaunchConfiguration shortcuts
    # ----------------------------------------------------------------
    headless = LaunchConfiguration("headless")
    model = LaunchConfiguration("model")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    # ----------------------------------------------------------------
    # Gazebo launch
    # ----------------------------------------------------------------
    # Only unset DISPLAY when headless
    unset_display = SetEnvironmentVariable(
        name="DISPLAY", value="", condition=IfCondition(headless)
    )

    world_file = os.path.join(
        get_package_share_directory("my_drone_sim"),
        "worlds",
        "simple_map.sdf",
    )

    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-v", "2", "-s", "-r", "--headless-rendering", world_file],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "2", "-r", world_file],
        output="screen",
        condition=UnlessCondition(headless),
    )

    # ----------------------------------------------------------------
    # Spawn drone after 1s
    # ----------------------------------------------------------------
    spawn_drone = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    model,
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                ],
                output="screen",
            )
        ],
    )

    # ----------------------------------------------------------------
    # Assemble LaunchDescription
    # ----------------------------------------------------------------
    return LaunchDescription(
        [
            # args
            headless_arg,
            model_arg,
            x_arg,
            y_arg,
            z_arg,
            # launch actions
            unset_display,
            gz_headless,
            gz_gui,
            # spawn_drone,
        ]
    )

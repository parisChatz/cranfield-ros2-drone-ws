import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg_share = get_package_share_directory("x500_simulator")

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="If true, run gz sim in headless rendering mode",
    )
    headless = LaunchConfiguration("headless")

    world_file = os.path.join(pkg_share, "worlds", "forest.sdf")

    # ----------------------------------------------------------------
    # Environment
    # ----------------------------------------------------------------
    # Prepend this package's models dir so Gazebo resolves model:// URIs.
    # This complements the ament env hook and ensures it works even when
    # launched from a subprocess that hasn't sourced the workspace.
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_share, "models")
        + ":"
        + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    # Only unset DISPLAY when running headless
    unset_display = SetEnvironmentVariable(
        name="DISPLAY", value="", condition=IfCondition(headless)
    )

    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-v", "2", "-s", "-r", "--headless-rendering", world_file],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "1", "-r", world_file],
        output="screen",
        condition=UnlessCondition(headless),
    )

    # ----------------------------------------------------------------
    # LaunchDescription
    # ----------------------------------------------------------------
    return LaunchDescription(
        [
            headless_arg,
            gz_resource_path,
            unset_display,
            gz_headless,
            gz_gui,
        ]
    )

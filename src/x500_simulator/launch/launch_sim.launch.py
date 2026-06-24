import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value="false",
        description="If true, launch ros_gz_bridge and rviz2 for visualization",
    )
    visualize = LaunchConfiguration("visualize")

    world_file = os.path.join(pkg_share, "worlds", "warehouse_world.sdf")

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
    # Robot description (for rviz RobotModel display)
    # ----------------------------------------------------------------
    urdf_path = os.path.join(pkg_share, "config", "x500.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read().replace(
            "package://x500_simulator", "file://" + pkg_share
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(visualize),
    )

    # ----------------------------------------------------------------
    # ROS-Gazebo bridge
    # ----------------------------------------------------------------
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("x500_simulator"), "config", "bridge_topics.yaml"]
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
        condition=IfCondition(visualize),
    )

    # ----------------------------------------------------------------
    # RViz2
    # ----------------------------------------------------------------
    rviz_config = os.path.join(pkg_share, "config", "rviz_config.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(visualize),
    )

    # ----------------------------------------------------------------
    # LaunchDescription
    # ----------------------------------------------------------------
    return LaunchDescription(
        [
            headless_arg,
            visualize_arg,
            gz_resource_path,
            gz_headless,
            gz_gui,
            ros_gz_bridge,
            robot_state_publisher,
            rviz2,
        ]
    )

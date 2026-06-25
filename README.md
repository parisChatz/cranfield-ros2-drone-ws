# PX4 drone simulation for RL training using ROS2 and Gazebo Sim 8

ROS2 Jazzy + Gazebo Harmonic workspace for X500 quadcopter simulation. The `x500_simulator` ROS2 package provides all simulation assets (models, worlds, launch files, bridge config).

> **Note:** The `gym-drones` Gymnasium environment package has been removed from this repo and is being refactored as a separate package.

## Installation

### Prerequisites

- ROS2 Jazzy
- Gazebo Harmonic (Sim 8)

### Project installation

```bash
colcon build
source install/setup.bash
```

`GZ_SIM_RESOURCE_PATH` is set automatically by the package's ament environment hook — no manual export needed.

## Usage

Launch the simulation:

```bash
# Headless (default)
ros2 launch x500_simulator launch_sim.launch.py

# With GUI and RViz
ros2 launch x500_simulator launch_sim.launch.py visualize:=true
```

Send a twist command via ROS2:

```bash
ros2 topic pub /x500/command/twist geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

Or publish directly on the Gazebo topic:

```bash
gz topic -t "/x500/command/twist" -m gz.msgs.Twist -p "linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.5 }"
```

Kill the drone motors:

```bash
ros2 topic pub /x500/enabled_motors std_msgs/msg/Bool "{data: false}"
```

Run the ROS2–Gazebo bridge manually (if not using the launch file):

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/x500_simulator/config/bridge_topics.yaml
```

## Errors and Warnings

1. `[ros2-3] [WARN] [...] [ros_gz_bridge]: Failed to create a bridge for topic [/x500/scan/2d]`

   This occurs when the active drone model does not publish that topic (e.g. no 2D lidar). Safe to ignore if you are not using that sensor.

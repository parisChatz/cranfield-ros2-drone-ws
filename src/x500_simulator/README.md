# x500_simulator Package

A ROS2 package that provides everything needed to simulate an X500 quadcopter drone in Gazebo Harmonic. It contains the drone's 3D models, sensor definitions, simulation worlds, and all the configuration files to visualise the drone in RViz2.

This package **does not contain any ROS2 nodes** for controlling the drone. It is purely a simulation assets package. Control is handled externally (e.g. by the `gym-drones` Python package for reinforcement learning).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Building the Package](#building-the-package)
- [Running the Simulation](#running-the-simulation)
- [Launch Arguments](#launch-arguments)
- [Package Structure](#package-structure)
- [How the Launch File Works](#how-the-launch-file-works)
- [The Drone Model](#the-drone-model)
- [Simulation Worlds](#simulation-worlds)
- [The ROS-Gazebo Bridge](#the-ros-gazebo-bridge)
- [Useful Commands](#useful-commands)

---

## Prerequisites

Before using this package, make sure you have the following installed:

- **ROS2 Jazzy** - the ROS2 distribution this workspace targets
- **Gazebo Harmonic** (also called Gazebo Sim 8) - the physics simulator
- **ros_gz_bridge** - translates messages between Gazebo and ROS2
- **rviz2** - the ROS2 visualisation tool (included with most ROS2 desktop installs)

## Building the Package

From the root of the workspace:

```bash
# Build the package
colcon build --symlink-install

# Source the workspace so ROS2 can find the package
source install/setup.bash
```

You need to re-run `source install/setup.bash` every time you open a new terminal, or add it to your `~/.bashrc`.

## Running the Simulation

The main entry point is the launch file. This single command starts the Gazebo simulator and (by default) the ROS2 bridge and RViz2 for visualisation:

```bash
ros2 launch x500_simulator launch_sim.launch.py
```

This will:
1. Open the Gazebo GUI with the drone in a warehouse world
2. Start the ROS-Gazebo bridge (so ROS2 tools can see the drone's sensor data)
3. Start RViz2 (a 3D viewer for ROS2) with a pre-made configuration

### Running Headless (No GUI)

If you don't need to see the Gazebo window (e.g. for training an AI agent), run in headless mode:

```bash
ros2 launch x500_simulator launch_sim.launch.py headless:=true visualize:=false
```

This runs the physics simulation without opening any windows, which is faster and uses less resources.

## Launch Arguments

The launch file accepts two arguments that control its behaviour:

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | When `true`, runs Gazebo without a GUI window (server-only mode with headless rendering). Useful for automated training. |
| `visualize` | `true` | When `true`, starts the ROS-Gazebo bridge, robot state publisher, and RViz2. Set to `false` if you only need the simulation running. |

### Example Combinations

```bash
# Full visual mode (default) - Gazebo GUI + RViz2
ros2 launch x500_simulator launch_sim.launch.py

# Gazebo GUI only, no ROS2 visualisation
ros2 launch x500_simulator launch_sim.launch.py visualize:=false

# Headless + no visualisation (fastest, for training)
ros2 launch x500_simulator launch_sim.launch.py headless:=true visualize:=false

# Headless but with ROS2 bridge and RViz2 (useful for debugging training)
ros2 launch x500_simulator launch_sim.launch.py headless:=true visualize:=true
```

## Package Structure

```
x500_simulator/
├── launch/
│   └── launch_sim.launch.py      # Main launch file - starts everything
├── models/                        # Gazebo SDF models
│   ├── x500_gimbal/               # The main drone (composed from sub-models)
│   ├── x500_mono_cam/             # Drone body + front camera
│   ├── x500/                      # Drone body + control plugins
│   ├── x500_base/                 # Physical drone frame (links, joints, meshes)
│   ├── mono_cam/                  # Front-facing RGB camera sensor
│   ├── gimbal/                    # Camera gimbal with segmentation camera
│   ├── lidar_2d_v2/               # 2D lidar sensor
│   ├── goal/                      # Visual marker for RL target position
│   ├── flight_arena/              # Indoor flight arena mesh
│   ├── maze_model_rs/             # Maze environment mesh
│   ├── red_box/                   # Simple coloured obstacle
│   └── green_box/                 # Simple coloured obstacle
├── worlds/                        # Gazebo world files (.sdf)
│   ├── warehouse_world.sdf        # Warehouse with walls and obstacles (currently active)
│   ├── flight_arena.sdf           # Open flight arena
│   ├── forest.sdf                 # Outdoor forest environment
│   └── simple_map.sdf             # Minimal world
├── config/
│   ├── bridge_topics.yaml         # Defines which Gazebo topics get bridged to ROS2
│   ├── x500.urdf                  # Robot description for RViz's RobotModel display
│   └── rviz_config.rviz           # Pre-configured RViz2 layout
├── hooks/
│   └── setup.dsv.in               # Tells Gazebo where to find our models/worlds
├── CMakeLists.txt                 # Build configuration
└── package.xml                    # ROS2 package manifest
```

## How the Launch File Works

The launch file (`launch/launch_sim.launch.py`) is a Python script that tells ROS2 what to start. Here is what each section does:

### 1. Environment Variables

Gazebo needs to know where to find our custom models (the drone, obstacles, etc.). This is handled **automatically** by the environment hook (`hooks/setup.dsv.in`), which adds the package's `models/` and `worlds/` directories to `GZ_SIM_RESOURCE_PATH` whenever you `source install/setup.bash`. So as long as you've sourced the workspace, Gazebo will find all models when a world file references e.g. `model://x500_gimbal`.

### 2. Start Gazebo

Depending on the `headless` argument, it starts Gazebo in one of two modes:

- **GUI mode** (`headless:=false`): Runs `gz sim -r <world_file>` which opens the full Gazebo window where you can see and interact with the simulation.
- **Headless mode** (`headless:=true`): Runs `gz sim -s -r --headless-rendering <world_file>` which runs only the physics engine with no visible window. The `-s` flag means server-only, and `--headless-rendering` enables off-screen rendering so cameras still produce images even without a display.

The `-r` flag tells Gazebo to start running the simulation immediately (instead of starting paused).

### 3. Start the ROS-Gazebo Bridge (when `visualize:=true`)

The `ros_gz_bridge` node translates Gazebo messages into ROS2 messages. This is needed so that ROS2 tools (like RViz2 or `ros2 topic echo`) can access the drone's sensor data.

Which topics get translated is defined in `config/bridge_topics.yaml` (see [The ROS-Gazebo Bridge](#the-ros-gazebo-bridge) section below).

### 4. Start the Robot State Publisher (when `visualize:=true`)

This ROS2 node reads the drone's URDF file (`config/x500.urdf`) and publishes the robot model description. RViz2 uses this to display the drone's 3D model.

### 5. Start RViz2 (when `visualize:=true`)

Opens RViz2 with a pre-configured layout (`config/rviz_config.rviz`) that already has the drone's camera feed, lidar scan, TF tree, and robot model set up for viewing.

## The Drone Model

The main drone used in simulation is called **`x500_gimbal`**. Rather than being defined in a single huge file, it is built by combining several smaller models together, like building blocks. Each sub-model adds a specific piece of hardware:

```
x500_gimbal  (the complete drone)
  |
  ├── x500_mono_cam  (drone body + front camera)
  │     ├── x500            (drone with control/odometry plugins)
  │     │     └── x500_base (physical frame, propellers, meshes)
  │     └── mono_cam        (front-facing RGB camera)
  |
  ├── gimbal                (camera gimbal + segmentation camera)
  |
  └── lidar_2d_v2           (2D lidar scanner)
```

This modular design means you can easily swap or remove sensors. For example, `x500_mono_cam` is a drone with just a front camera and no gimbal or lidar.

### Sensors on the Drone

| Sensor | Gazebo Topic | What It Provides |
|--------|-------------|------------------|
| Front camera | `/x500/camera` | 160x160 RGB image |
| Gimbal camera | `/x500/gimbal_camera` | Downward-facing RGB image |
| Segmentation camera | `/x500/gimbal_segmentation_camera/colored_map` | Colour-coded object labels |
| 2D Lidar | `/x500/scan` | Laser distance scan around the drone |
| Odometry | `/x500/odometry` | Position, orientation, and velocity |

### Controlling the Drone

The drone accepts velocity commands on the Gazebo topic `/x500/cmd_vel`. You send a **Twist** message containing linear velocity (forward/sideways/up-down) and angular velocity (yaw rate), and the drone moves accordingly. This is the same message type used by many ROS2 robots.

## Simulation Worlds

The package includes several world files in the `worlds/` directory. Each defines a different environment for the drone to fly in:

| World | Description |
|-------|-------------|
| `warehouse_world.sdf` | An enclosed warehouse with outer walls, internal dividers, obstacles, and coloured shelf markers. **Currently active** in the launch file. |
| `flight_arena.sdf` | A simpler open indoor arena |
| `forest.sdf` | An outdoor forest environment |
| `simple_map.sdf` | A minimal, mostly empty world |

To change which world is loaded, edit the launch file (`launch/launch_sim.launch.py`) and change line 32 to point to a different `.sdf` file. For example:

```python
# Change from:
world_file = os.path.join(pkg_share, "worlds", "warehouse_world.sdf")
# To:
world_file = os.path.join(pkg_share, "worlds", "forest.sdf")
```

After editing, rebuild the package with `colcon build` and re-source the workspace.

## The ROS-Gazebo Bridge

Gazebo and ROS2 use different messaging systems. The **bridge** connects them by subscribing to topics on one side and re-publishing them on the other.

The bridged topics are defined in `config/bridge_topics.yaml`:

| Gazebo Topic | ROS2 Topic | ROS2 Message Type | Direction |
|-------------|------------|-------------------|-----------|
| `/x500/camera` | `/x500/camera` | `sensor_msgs/msg/Image` | Gazebo -> ROS2 |
| `/x500/cmd_vel` | `/x500/cmd_vel` | `geometry_msgs/msg/Twist` | Both ways |
| `/x500/gimbal_camera` | `/x500/gimbal_camera` | `sensor_msgs/msg/Image` | Gazebo -> ROS2 |
| `/x500/gimbal_segmentation_camera/colored_map` | (same) | `sensor_msgs/msg/Image` | Gazebo -> ROS2 |
| `/x500/odometry` | `/x500/odometry` | `nav_msgs/msg/Odometry` | Gazebo -> ROS2 |
| `/x500/scan` | `/x500/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS2 |
| `/model/x500/pose` (GZ) | `/tf` (ROS2) | `tf2_msgs/msg/TFMessage` | Gazebo -> ROS2 |

Most topics flow **one way** (from Gazebo to ROS2) because they are sensor outputs. The `/x500/cmd_vel` topic is **bidirectional** so you can send movement commands from ROS2 into the simulation.

## Useful Commands

Once the simulation is running, here are some handy commands to try in a **new terminal** (remember to `source install/setup.bash` first):

### List available ROS2 topics

```bash
ros2 topic list
```

### View the drone's position

```bash
ros2 topic echo /x500/odometry
```

### Send a velocity command to the drone

```bash
# Move the drone forward at 1 m/s
ros2 topic pub /x500/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### View the TF frame tree

```bash
ros2 run tf2_tools view_frames
```

### List Gazebo topics directly (bypassing ROS2)

```bash
gz topic -l
```

### Echo a Gazebo topic directly

```bash
gz topic -e -t /x500/odometry
```

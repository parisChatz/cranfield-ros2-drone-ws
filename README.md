# PX4 drone simulation for RL training using ROS2 and Gazebo Sim 8
This is a repo for working on RL in GazeboSim and ROS2 with PX4 drones. The models have been taken from the official PX4 repo.

## Installation
Clone and do ```colcon build``` and ```source install.setup.bash```.

## Usage
To begin an empty world drone simulation run ```ros2 launch my_drone_sim my_world.launch.py```.

A simple command from ros2 is ```ros2 topic pub /x500/command/twist geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once```.

If you want you can publish straight on the gazebo topic by doing ```gz topic -t "/x500/command/twist" -m gz.msgs.Twist -p "linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.5 }"```.

The bridge for the bidirectional relationship between ros2 and gzsim topics is run in the launch file, but if you comment it out you can run the bridge from the terminal by doing ```ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/my_drone_sim/config/bridge_topics.yaml```.

To kill the drones motors do ```ros2 topic pub /x500/enabled_motors std_msgs/msg/Bool "{data: false}"```.


## Errors and Warnings
1. ```[ros2-3] [WARN] [1739878250.383380792] [ros_gz_bridge]: Failed to create a bridge for topic [/x500/scan/2d] with ROS2 type [sensor_msgs/msg/LaserScan] to topic [/x500/scan/2d] with Gazebo Transport type [gz.msgs.LaserScan]```
This happens because the model you are running doesn't have a gz topic with that name. In this case the drone I am running doesn't have a lidar sensor with that topic, thus no bridge has been made for that topic.
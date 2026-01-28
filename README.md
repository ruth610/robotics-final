# Autonomous Crop Monitoring Robot (ROS2 Humble)

This project simulates a low-cost autonomous crop monitoring robot for smallholder farms. It uses ROS2 Humble, Gazebo, Nav2 (navigation), SLAM Toolbox, and a camera-based perception node (OpenCV) to detect basic crop stress (discoloration, dryness, texture changes).

## Prerequisites

Tested on Ubuntu 22.04 with ROS2 Humble.

Install core dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-xacro \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  python3-opencv \
  python3-colcon-common-extensions
```

## Build

```bash
# From workspace root
cd /home/rutha/Projects/class_projects/robotics_final
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

## Run (full system)

```bash
# Start Gazebo, spawn robot, Nav2 with SLAM, and perception
ros2 launch farm_robot_bringup bringup.launch.py
```

Optional tools:

```bash
# Visualize topics
ros2 topic list
ros2 topic echo /crop_monitor/status

# RViz2 manual (optional)
rviz2
```

## Packages Overview

- `farm_robot_description`: Robot URDF/Xacro, Gazebo world (crop rows), spawn/bringup.
- `farm_robot_navigation`: Nav2 configuration and launch (includes SLAM Toolbox).
- `farm_robot_perception`: OpenCV-based node subscribing to camera images and publishing stress metrics.
- `farm_robot_bringup`: Orchestrates all components with a single launch.

## What it does

- Navigates rows using Nav2 (SLAM builds a map on the fly).
- Captures camera frames while moving.
- Computes indices: chlorophyll (green content), dryness (brown/yellow content + texture roughness), and status.
- Publishes:
  - `/crop_monitor/chlorophyll_index` (Float32)
  - `/crop_monitor/dryness_index` (Float32)
  - `/crop_monitor/texture_index` (Float32)
  - `/crop_monitor/status` (String)

## Notes

- If Gazebo fails to launch, verify `gazebo_ros` is installed and sourced by checking `ros2 pkg list | grep gazebo`.
- Tuning Nav2 parameters and SLAM settings may be required for different world scales.
- For real hardware, replace Gazebo plugins with `ros2_control` hardware interfaces and a physical camera driver.
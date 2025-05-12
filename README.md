
# Autonomous Drone-based SLAM System for Subsurface Exploration of Simulated Caves

This project simulates the use of an autonomous drone to perform SLAM (Simultaneous Localization and Mapping) inside a realistic cave environment, using tools such as ROS 2, PX4, Gazebo Ignition, Blender, and intended RTAB-Map for mapping.

Inspired by lunar lava tubes and Mars exploration missions, the system is designed for subsurface, GPS-denied autonomous navigation and mapping.

---

## Project Overview

- **Team**: Atharv Kulkarni, Shalom Richard Pakhare, Harsh Padmalwar  
- **Course**: RAS 598 - Space Robotics and AI  
- **Objective**: Map unknown 3D cave environments with a drone using onboard LiDAR and SLAM.

---

## System Architecture

- **Simulator**: Gazebo ignition (with ROS 2 integration)
- **Autopilot**: PX4
- **Middleware**: ROS 2 Humble
- **Modeling**: Blender (for cave creation)
- **SLAM Package**: RTAB-Map (planned)
- **Sensors**: 2D LiDAR (front-mounted), IMU
- **World**: Custom cave environment based on Sketchfab model

---

## Implementation Steps

### 1. Cave Environment Creation

- Downloaded modular cave mesh in FBX format from Sketchfab.
- Imported FBX into Blender.
- Edited cave mesh (remove unneeded geometry, scale, origin alignment).
- Exported mesh as `.dae` (COLLADA) or `.stl` for use in Gazebo.
- Created a corresponding `model.config` and `model.sdf` for the cave.

### 2. Drone and Sensor Setup

- Used PX4 x500 drone model as base.
- Modified drone’s SDF/Xacro to include:
  - `ray` sensor plugin for 2D LiDAR with horizontal FOV.
  - Proper orientation and frame alignment for LiDAR link.
- Spawned drone using PX4’s `make px4_sitl gazebo` with custom world.

### 3. World Integration

- Created `cave_world.sdf` to include:
  - Static cave model reference.
  - Custom light settings.
  - Gravity settings and ground plane toggled as needed.
- World was launched through PX4’s `PX4_GZ_WORLD=cave_world`.

### 4. SLAM Attempt (RTAB-Map)

- Planned use of `rtabmap_ros` with ROS 2 launch file.
- Configured `rtabmap.launch.py` and `rtabmap.yaml`.
- Encountered roadblock:
  - LiDAR `/scan` topic was not publishing correctly, so SLAM node received no data.
  - Visual SLAM fallback (e.g. stereo or RGBD) was considered, but not implemented.

---

## Media

| Preview 1 | Preview 2 |
|-----------|-----------|
| ![Screenshot from 2025-05-08 14-03-38](https://github.com/user-attachments/assets/e716e324-c4a8-40fe-b46f-77ac31643dd6) |
| ![Screenshot from 2025-05-08 14-51-00](https://github.com/user-attachments/assets/e214a1d3-99f0-4041-8d02-4188a7c59e18) |

---

## Current Limitations

- Could not bridge `/scan` topic correctly to ROS 2 for SLAM input.
- As a result, no point cloud was generated, and SLAM was not demonstrated live.

---

## Future Work

- Fix LiDAR to `/scan` bridging using `ros_ign_bridge` or custom plugin.
- Integrate RTAB-Map or Cartographer for full SLAM loop.
- Add DFS path planning to explore tunnels using occupancy grid.
- Add a depth camera for RGBD SLAM and 3D map building.
- Extend system to multi-agent setups (drone + rover cooperation).
- Explore use cases for disaster recovery, volcanology, and planetary missions.


---

## Command History / Setup Steps

### Install PX4 and Dependencies

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gazebo
```

### Install ROS 2 Humble

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

### Source ROS 2 and Build Workspace

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Launch Drone in Cave World

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=cave_world make px4_sitl gazebo
```

### (Planned) Run SLAM Node

```bash
ros2 launch rtabmap_ros rtabmap.launch.py use_sim_time:=true
```

---



### Bridge LiDAR Scan Topic from Ignition to ROS 2

```bash
# Install ros_ign_bridge
sudo apt install ros-humble-ros-ign-bridge

# Source ROS 2
source /opt/ros/humble/setup.bash

# Bridge the /scan topic (replace with actual Ignition topic if different)
ros2 run ros_ign_bridge parameter_bridge /world/default/model/x500_lidar_2d/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
```

> Note: Use `ign topic -l` to find the actual topic name if it's different.

### Visualize in RViz2

```bash
# Launch RViz2 and manually add a "LaserScan" display
rviz2
```

Or with a pre-configured display:

```bash
rviz2 -d ~/ros2_ws/src/my_package/rviz/lidar_view.rviz
```


## License

This project is open-source under the MIT License.

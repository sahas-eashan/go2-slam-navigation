# Autonomous Navigation with 2D SLAM - Unitree Go2 EDU

Complete ROS2 Humble workspace for autonomous navigation using 2D SLAM on the Unitree Go2 EDU quadruped robot.

## Overview

This repository provides a ready-to-use implementation of SLAM-based autonomous navigation for the Unitree Go2 EDU robot in Gazebo simulation. The system uses a Velodyne VLP-16 3D LiDAR converted to 2D laser scans for mapping and localization.

**Key Features:**
- Gazebo simulation with realistic Go2 physics
- Velodyne VLP-16 3D LiDAR with 2D conversion
- SLAM Toolbox for mapping
- Nav2 for autonomous navigation
- CHAMP quadruped controller
- Pre-configured for ROS2 Humble

## System Requirements

- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **RAM**: 8GB minimum (16GB recommended)
- **GPU**: Recommended for Gazebo

## Quick Start

### 1. Install Dependencies

```bash
# ROS2 packages
sudo apt install -y \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-velodyne \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan
```

### 2. Build the Workspace

```bash
cd ~/unitree_ros2/go2_slam_navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Gazebo Simulation

**Terminal 1**: Start Gazebo with Go2 robot
```bash
cd ~/unitree_ros2/go2_slam_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

### 4. Launch Velodyne to 2D Scan Converter

**Terminal 2**: Convert 3D point cloud to 2D laser scan
```bash
cd ~/unitree_ros2/go2_slam_navigation
source /opt/ros/humble/setup.bash
ros2 launch velodyne_to_scan.launch.py
```

### 5. Verify Sensors

**Terminal 3**: Check topics
```bash
source /opt/ros/humble/setup.bash

# Should see these topics:
ros2 topic list | grep -E "scan|velodyne|imu|odom"
# /scan                 ← 2D laser scan (for SLAM)
# /velodyne_points      ← 3D LiDAR data
# /imu/data            ← IMU sensor
# /odom                ← Odometry

# Verify /scan is publishing
ros2 topic info /scan
# Should show: Publisher count: 1
```

## Package Structure

```
go2_slam_navigation/
├── src/
│   ├── go2_description/       # Robot URDF with Velodyne sensor
│   ├── go2_config/            # Launch files and configurations
│   └── champ/                 # Quadruped controller framework
├── velodyne_to_scan.launch.py # Point cloud to laser scan converter
└── README.md
```

## Sensors

| Sensor | Topic | Message Type | Rate |
|--------|-------|--------------|------|
| **Velodyne VLP-16** | `/velodyne_points` | PointCloud2 | 10 Hz |
| **2D Laser Scan** | `/scan` | LaserScan | 10 Hz |
| **IMU** | `/imu/data` | Imu | 100 Hz |
| **Odometry** | `/odom` | Odometry | 30 Hz |
| **Joint States** | `/joint_states` | JointState | 30 Hz |

## Next Steps for SLAM

After verifying sensors work, proceed with SLAM mapping:

### Step 1: Launch SLAM Toolbox

```bash
# Terminal 4
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_config slam.launch.py
```

### Step 2: Teleoperate Robot

```bash
# Terminal 5
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
- `i` - Forward
- `k` - Stop
- `j` - Turn left
- `l` - Turn right
- `u/o` - Forward + turn
- `m/.` - Backward + turn

### Step 3: Save Map

After exploring the environment:
```bash
mkdir -p ~/unitree_ros2/go2_slam_navigation/maps
ros2 run nav2_map_server map_saver_cli -f ~/unitree_ros2/go2_slam_navigation/maps/my_map
```

### Step 4: Launch Navigation

```bash
# Stop SLAM (Ctrl+C in Terminal 4)
# Launch Nav2 with saved map
ros2 launch go2_config navigate.launch.py \
    map:=$HOME/unitree_ros2/go2_slam_navigation/maps/my_map.yaml
```

### Step 5: Set Navigation Goals

In RViz:
1. Click "2D Pose Estimate" - set initial robot position
2. Click "Nav2 Goal" - set destination
3. Robot navigates autonomously!

## Configuration Files

### Velodyne to Scan Converter Parameters

Located in: `velodyne_to_scan.launch.py`

Key parameters:
- `min_height`: -0.3m (filter ground)
- `max_height`: 0.3m (filter ceiling)
- `angle_min/max`: ±180° (full circle)
- `range_min`: 0.9m
- `range_max`: 100.0m

### SLAM Toolbox Configuration

Located in: `src/go2_config/config/autonomy/slam.yaml`

- Mode: Online async mapping
- Solver: Ceres with SPARSE_NORMAL_CHOLESKY
- Loop closure: Enabled
- Map resolution: 0.05m

### Nav2 Configuration

Located in: `src/go2_config/config/autonomy/navigation.yaml`

- Planner: Navfn
- Controller: DWB
- Recovery behaviors: Spin, backup, wait

## Troubleshooting

### Issue: /scan topic not publishing

**Check:**
```bash
ros2 topic info /scan
```

**Solution:**
- Ensure velodyne_to_scan converter is running
- Check /velodyne_points is publishing first

### Issue: SLAM not building map

**Check:**
```bash
# Verify TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check SLAM node
ros2 node list | grep slam
```

**Solution:**
- Ensure robot is moving (static robot = no map)
- Verify /scan topic has valid data (not all .inf)

### Issue: Robot not moving in Gazebo

**Check:**
```bash
ros2 topic echo /cmd_vel
```

**Solution:**
- Verify teleop node is publishing
- Check CHAMP controller is running

## Repository Information

**Created**: December 2024
**Platform**: ROS2 Humble on Ubuntu 22.04
**Robot**: Unitree Go2 EDU
**Simulation**: Gazebo Classic

## Credits

This project builds upon:
- [CHAMP Quadruped Controller](https://github.com/chvmp/champ)
- [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros)
- [anujjain-dev/unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2)

## License

See individual package licenses.

---

**For detailed step-by-step documentation, see the research repository.**

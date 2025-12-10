# Verification Checklist - Go2 SLAM Navigation

Use this checklist to verify your setup is working correctly.

## ✅ Pre-Build Verification

- [ ] Ubuntu 22.04 installed
- [ ] ROS2 Humble installed and sourced
- [ ] All dependencies installed (run `./setup.sh`)
- [ ] Workspace built successfully (`colcon build`)

## ✅ Sensor Verification

### Step 1: Launch Simulation

```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

**Verify:**
- [ ] Gazebo window opens
- [ ] Go2 robot visible in Gazebo
- [ ] RViz window opens
- [ ] Robot model visible in RViz

### Step 2: Check Topics

```bash
ros2 topic list
```

**Must see:**
- [ ] `/velodyne_points` - 3D LiDAR
- [ ] `/imu/data` - IMU sensor
- [ ] `/odom` - Odometry
- [ ] `/joint_states` - Robot joints
- [ ] `/cmd_vel` - Command velocity

### Step 3: Launch Converter

```bash
ros2 launch velodyne_to_scan.launch.py
```

**Verify:**
- [ ] No errors in terminal
- [ ] `/scan` topic appears

### Step 4: Verify Sensors Publishing

```bash
# Check Velodyne
ros2 topic info /velodyne_points
# Should show: Publisher count: 1

# Check 2D scan
ros2 topic info /scan
# Should show: Publisher count: 1

# Check IMU
ros2 topic info /imu/data
# Should show: Publisher count: 1

# Check odometry
ros2 topic info /odom
# Should show: Publisher count: 1
```

**Verify:**
- [ ] `/velodyne_points` has 1 publisher
- [ ] `/scan` has 1 publisher
- [ ] `/imu/data` has 1 publisher
- [ ] `/odom` has 1 publisher

### Step 5: Check Sensor Data

```bash
# Check scan data
ros2 topic echo /scan --once
```

**Verify:**
- [ ] `angle_min: -3.14159`
- [ ] `angle_max: 3.14159`
- [ ] `ranges:` array present
- [ ] `frame_id: velodyne`

### Step 6: Verify TF Tree

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Verify:**
- [ ] PDF opens showing TF tree
- [ ] `odom → base_link` connection exists
- [ ] `base_link → velodyne` connection exists
- [ ] `base_link → imu_link` connection exists
- [ ] All leg frames connected

## ✅ Control Verification

### Step 7: Test Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Verify:**
- [ ] Press `i` - robot moves forward in Gazebo
- [ ] Press `k` - robot stops
- [ ] Press `j` - robot turns left
- [ ] Press `l` - robot turns right

### Step 8: Visualize in RViz

**In RViz:**

1. Add LaserScan:
   - [ ] Click Add → LaserScan
   - [ ] Set Topic: `/scan`
   - [ ] Red dots visible showing laser data

2. Add PointCloud2:
   - [ ] Click Add → PointCloud2
   - [ ] Set Topic: `/velodyne_points`
   - [ ] Point cloud visible

3. Add TF:
   - [ ] Click Add → TF
   - [ ] All frames visible (base_link, velodyne, etc.)

## ✅ SLAM Verification

### Step 9: Launch SLAM

```bash
ros2 launch go2_config slam.launch.py
```

**Verify:**
- [ ] SLAM node starts without errors
- [ ] `/map` topic appears
- [ ] Map displayed in RViz

### Step 10: Build Map

**Teleoperate robot around environment**

**Verify:**
- [ ] Map builds in RViz (white/green = free, black = obstacles)
- [ ] Map updates as robot moves
- [ ] No major distortions in map

### Step 11: Save Map

```bash
mkdir -p ~/unitree_ros2/go2_slam_navigation/maps
ros2 run nav2_map_server map_saver_cli -f ~/unitree_ros2/go2_slam_navigation/maps/test_map
```

**Verify:**
- [ ] `test_map.pgm` file created
- [ ] `test_map.yaml` file created
- [ ] Files are not empty

## ✅ Navigation Verification

### Step 12: Launch Navigation

```bash
# Stop SLAM first (Ctrl+C)
ros2 launch go2_config navigate.launch.py \
    map:=$HOME/unitree_ros2/go2_slam_navigation/maps/test_map.yaml
```

**Verify:**
- [ ] Nav2 launches without errors
- [ ] Map loads in RViz
- [ ] Costmaps visible (local and global)

### Step 13: Set Initial Pose

**In RViz:**
- [ ] Click "2D Pose Estimate"
- [ ] Click on map where robot actually is
- [ ] Drag to set orientation
- [ ] AMCL particles converge (green arrows cluster around robot)

### Step 14: Navigate to Goal

**In RViz:**
- [ ] Click "Nav2 Goal" (or "2D Goal Pose")
- [ ] Click destination on map
- [ ] Drag to set goal orientation
- [ ] Path appears (green = global, red = local)
- [ ] Robot navigates to goal autonomously
- [ ] Robot avoids obstacles

## ✅ Final Checks

- [ ] All 14 steps completed successfully
- [ ] No persistent errors in any terminal
- [ ] Robot can autonomously navigate in simulation
- [ ] Maps save and load correctly

---

## Troubleshooting

If any step fails, see:
- README.md Troubleshooting section
- Check terminal outputs for error messages
- Verify all dependencies installed: `./setup.sh`

## Next Steps

Once all checks pass:
1. ✅ Try different Gazebo worlds
2. ✅ Tune SLAM parameters
3. ✅ Tune navigation parameters
4. ✅ Prepare for real robot deployment

---

**Date Completed:** _______________
**Verified By:** _______________

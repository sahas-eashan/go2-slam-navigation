#!/bin/bash

# Setup script for Autonomous Navigation with 2D SLAM - Unitree Go2 EDU
# This script installs all required dependencies and builds the workspace

set -e

echo "================================================"
echo "  Unitree Go2 SLAM Navigation Setup"
echo "================================================"
echo ""

# Check Ubuntu version
if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "Warning: This script is designed for Ubuntu 22.04 (Jammy)"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check ROS2 Humble is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS2 Humble is not installed!"
    echo "Please install ROS2 Humble first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✓ ROS2 Humble detected"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

echo "Installing dependencies..."
echo ""

# Update package list
sudo apt update

# Install ROS2 dependencies
echo "Installing ROS2 packages..."
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

echo ""
echo "✓ All dependencies installed"
echo ""

# Build workspace
echo "Building workspace..."
cd "$(dirname "$0")"
colcon build --symlink-install

echo ""
echo "✓ Workspace built successfully"
echo ""

# Create maps directory
mkdir -p maps

echo "================================================"
echo "  Setup Complete!"
echo "================================================"
echo ""
echo "To use this workspace:"
echo "  1. Source the workspace:"
echo "     source ~/unitree_ros2/go2_slam_navigation/install/setup.bash"
echo ""
echo "  2. Launch Gazebo simulation:"
echo "     ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true"
echo ""
echo "  3. Launch Velodyne converter (in new terminal):"
echo "     ros2 launch velodyne_to_scan.launch.py"
echo ""
echo "See README.md for complete usage instructions."
echo ""

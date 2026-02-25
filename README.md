# Autonomous Navigation with 2D SLAM - Unitree Go2 EDU

## Project Objective

Implementing autonomous navigation for the Unitree Go2 EDU quadruped robot, utilizing Simultaneous Localization and Mapping (SLAM) to travel from a specified starting location to any designated endpoint.

## Software Stack

- **Operating System**: Ubuntu 22.04
- **ROS 2 Distribution**: Humble
- **Simulator**: Gazebo/Ignition for physics-based quadruped simulation
- **SLAM**: SLAM Toolbox for mapping and localization
- **Navigation**: Nav2 for global planning, local planning, and behavior execution
- **Robot Interface**: Unitree Go2 SDK2 or community ROS 2 driver

## Implementation Phases

### Simulation

#### 1. URDF Integration and Verification
Obtain and verify the Unitree Go2 URDF model with proper joint configurations and sensor frames (map, odom, base_link, imu_link, lidar_link). Test the model in RViz and validate the TF tree structure.

#### 2. Gazebo Integration
Import the Go2 URDF into Gazebo with required physics and sensor plugins (LiDAR, IMU, optional depth camera). Create a test world with obstacles for navigation testing.

#### 3. TF and Odometry Setup
Establish correct transform relationships: odom → base_link from robot odometry, base_link → lidar_link as fixed transform, and map → odom from SLAM. Ensure stable frame structure for navigation.

#### 4. SLAM Configuration
Configure SLAM Toolbox with appropriate robot frames and sensor inputs. Enable online SLAM mode and verify data reception and TF stability.

#### 5. Mapping
Drive the simulated robot to generate a complete 2D map of the environment. Save map files and pose graphs for navigation use.

#### 6. Nav2 Setup
Configure Nav2 with the saved map, localization method, global and local costmaps, robot footprint, planners, and local controller. Set up Behavior Tree Navigator for autonomous navigation tasks.

#### 7. Control Integration
Connect Nav2 motion commands to the Go2 control interface through the ROS 2 driver, ensuring proper translation of navigation commands to robot locomotion (gait and velocity control).

#### 8. Full Simulation Testing
Launch complete navigation pipeline in Gazebo: set initial pose, send navigation goals, and verify global path planning, dynamic obstacle avoidance, and smooth local control.

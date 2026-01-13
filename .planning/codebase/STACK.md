# Technology Stack

**Analysis Date:** 2026-01-12

## Languages

**Primary:**
- Python 3 - All ROS2 nodes, drivers, and control logic (`colcon_ws/src/transbot_bringup/scripts/*.py`)

**Secondary:**
- C++ - Odometry estimation and test services (`colcon_ws/src/transbot_bringup/src/base_node.cpp`, `move_server.cpp`)
- XML - ROS2 interface definitions and launch configurations
- YAML - Configuration files (`colcon_ws/src/transbot_bringup/config/*.yaml`)

## Runtime

**Environment:**
- ROS2 Humble (2023 distribution) - `.devcontainer/Dockerfile`
- Python 3.x with `#!/usr/bin/env python3` shebang
- Docker base: `ghcr.io/sloretz/ros:humble-desktop`

**Package Manager:**
- Colcon - ROS2 build tool for workspace
- Ament CMake - Build system (`buildtool_depend` in `package.xml`)
- pip3 - Python packages (`ultralytics`, `ros2_numpy`, `pyserial`)

## Frameworks

**Core:**
- ROS2 rclpy/rclcpp - Python and C++ client libraries
- Navigation2 (Nav2) - Autonomous navigation stack (`colcon_ws/src/transbot_bringup/launch/nav2_slam.launch.py`)
- SLAM Toolbox - Simultaneous localization and mapping (`config/mapper_params_online_async.yaml`)
- Robot Localization - Extended Kalman Filter sensor fusion (`config/ekf_localization.yaml`)

**Computer Vision:**
- Ultralytics YOLOv11 - Human detection (`pip3 install ultralytics`)
- cv_bridge - OpenCV to ROS2 image conversion
- OpenCV - Image processing (via cv_bridge)

**Testing:**
- ament_lint_auto, ament_lint_common - Code linting (declared but not configured)
- No active test suite implemented

**Build/Dev:**
- CMake with ament_cmake extensions
- ament_cmake_python - Python package building
- xacro - URDF macro preprocessor

## Key Dependencies

**Critical:**
- `nav2_simple_commander` - Navigation interface for action server
- `ultralytics` - YOLOv11 model loading and inference (`colcon_ws/src/transbot_bringup/scripts/human_detection_node.py`)
- `pyserial` - Hardware UART communication (`colcon_ws/src/transbot_bringup/transbot_bringup/Transbot_Lib.py`)
- `tf2_ros`, `tf2_geometry_msgs` - Coordinate frame transforms

**Infrastructure:**
- `sllidar_ros2` - RPlidar A1 driver (cloned from GitHub)
- `ros2_astra_camera` - Astra Pro Plus RGB-D camera driver (OrbbecSDK)
- `rf2o_laser_odometry` - Laser-based odometry estimation
- `imu_filter_madgwick` - IMU sensor fusion
- `image_transport`, `point_cloud_transport` - Data compression

**ROS2 Message Types:**
- `geometry_msgs` - PoseStamped, Twist, TransformStamped
- `sensor_msgs` - Image, LaserScan, PointCloud2, Imu
- `vision_msgs` - Detection2D, Detection2DArray
- `transbot_msgs` - Custom messages (`colcon_ws/src/transbot_msgs/msg/*.msg`)

## Configuration

**Environment:**
- ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0 (`.devcontainer/Dockerfile`)
- RMW_IMPLEMENTATION=rmw_zenoh_cpp (Zenoh DDS middleware)
- Time sync via Chrony to server 192.168.1.246

**Build:**
- `colcon_ws/src/*/CMakeLists.txt` - Package build configuration
- `colcon_ws/src/*/package.xml` - Package metadata and dependencies
- `colcon_ws/src/transbot_description/urdf/transbot_astra.urdf.xacro` - Robot model

**Parameters:**
- `config/nav2_params.yaml` - Navigation2 costmap and planner settings
- `config/mapper_params_online_async.yaml` - SLAM Toolbox configuration
- `config/ekf_localization.yaml` - EKF sensor fusion parameters
- `config/yolo11n.pt` - YOLOv11 nano model weights

## Platform Requirements

**Development:**
- Docker with privileged mode and host networking
- Remote access: Port 6080 (noVNC web desktop), 5901 (VNC)
- USB access for sensors (udev rules for rplidar, orbbec camera)

**Production:**
- Transbot hardware platform (extension board via `/dev/ttyAMA0`)
- RPlidar A1 LiDAR sensor
- Astra Pro Plus RGB-D camera
- 9-DOF IMU on extension board

---

*Stack analysis: 2026-01-12*
*Update after major dependency changes*

# External Integrations

**Analysis Date:** 2026-01-12

## APIs & External Services

**AI/ML:**
- Ultralytics YOLOv11 - Human detection inference
  - Model: `colcon_ws/src/transbot_bringup/config/yolo11n.pt` (nano variant)
  - Framework: PyTorch via Ultralytics package
  - Detection class: 0 (person), configurable confidence threshold (default 0.74)
  - Integration: `colcon_ws/src/transbot_bringup/scripts/human_detection_node.py`

**External APIs:**
- None detected (offline operation)

## Data Storage

**Databases:**
- None (stateless operation)

**File Storage:**
- YOLO model weights: `config/yolo11n.pt`
- SLAM maps: Saved via SLAM Toolbox (configured in `mapper_params_online_async.yaml`)
- RViz configs: `config/*.rviz`

**Caching:**
- No persistent caching (in-memory state only)

## Hardware Integrations

**Transbot Extension Board:**
- Connection: UART serial via `/dev/ttyAMA0` at 115200 baud
- Interface: `colcon_ws/src/transbot_bringup/transbot_bringup/Transbot_Lib.py`
- Capabilities:
  - Motor control (tank-style differential drive)
  - Servo angle control (camera pan, 3-DOF arm)
  - IMU data reading (accelerometer, gyroscope)
  - Battery voltage monitoring
  - LED and buzzer control

**RPlidar A1 LiDAR:**
- Package: `sllidar_ros2` (cloned from GitHub)
- Launch: `sllidar_a1_launch.py` (included in `bringup.launch.py`)
- Topic: `/scan` (sensor_msgs/LaserScan)
- Udev rules: `rplidar.rules`
- Range: Up to 8 meters

**Astra Pro Plus RGB-D Camera (OrbbecSDK):**
- Package: `ros2_astra_camera` (cloned from GitHub)
- Launch: `astro_pro_plus.launch.xml`
- Topics:
  - `/camera/color/image_raw` - RGB images
  - `/camera/depth/image_rect` - Depth images
  - `/camera/depth/points` - Point cloud
- Configuration: `depth_registration: true`
- Udev rules: `56-orbbec-usb.rules`

**IMU (9-DOF on extension board):**
- Access: Via Transbot_Lib UART protocol
- Data: Accelerometer (3-axis), Gyroscope (3-axis)
- Topic: `/imu/data_raw` (sensor_msgs/Imu)
- Filtering: IMU Filter Madgwick (`imu_filter_madgwick` node)

**Servo Motors:**
- Camera pan servo: PWM control via `/pwm_servo` topic
- Arm servos (3-DOF): Controlled via `/target_angle` topic (Arm message)
- Servo IDs: 7, 8, 9 for arm joints

## ROS2 Communication Infrastructure

**DDS Middleware:**
- RMW implementation: `rmw_zenoh_cpp` (Zenoh)
- ROS Domain ID: 0
- Localhost only: Disabled (network communication enabled)

**Custom Message Types (`transbot_msgs`):**
- `Arm.msg` - Array of Joint messages for arm control
- `Joint.msg` - Servo ID, angle, runtime
- `PWMServo.msg` - Camera pan servo control
- `Battery.msg` - Voltage reading
- `Adjust.msg` - IMU assist flag
- `Position.msg` - 3D point
- `PointArray.msg` - Array of positions
- `JoyState.msg` - Joystick input

**Custom Service Types:**
- `Move.srv` - Velocity, time, direction for motion
- `SetSearchPose.srv` - Add/remove search waypoints
- `GetSearchPoses.srv` - Retrieve mission waypoints
- `RobotArm.srv` - Arm control
- `Patrol.srv`, `Buzzer.srv`, `Headlight.srv`, `RGBLight.srv` - Peripherals

**Custom Action Types:**
- `SearchAndRescue.action` - Multi-stage rescue mission
  - Goal: `search_poses` (PoseStamped[])
  - Feedback: `current_task` (string)
  - Result: `success` (bool)

## Data Transport

**On-board Compression:**
- RGB images: `image_transport` raw → compressed
- Point clouds: `point_cloud_transport` raw → draco
- Launch: `colcon_ws/src/transbot_bringup/launch/transport_onboard.launch.py`

**Off-board Decompression:**
- Compressed topics decompressed on remote machine
- Launch: `colcon_ws/src/transbot_bringup/launch/transport_offboard.launch.py`
- Includes human_detection_node and human_poses_node

## Navigation Integration

**Nav2 Stack:**
- Package: `nav2_bringup`
- Config: `colcon_ws/src/transbot_bringup/config/nav2_params.yaml`
- Interface: `nav2_simple_commander.robot_navigator.BasicNavigator`
- Methods used: `goToPose()`, `goThroughPoses()`, `cancelTask()`, `isTaskComplete()`

**SLAM Toolbox:**
- Mode: Online async mapping
- Config: `colcon_ws/src/transbot_bringup/config/mapper_params_online_async.yaml`
- Solver: Ceres Solver with SPARSE_NORMAL_CHOLESKY
- Loop closure: Enabled

**Robot Localization (EKF):**
- Config: `colcon_ws/src/transbot_bringup/config/ekf_localization.yaml`
- Inputs:
  - `/odom_raw` - Wheel odometry
  - `/odom_rf2o` - Laser odometry
  - `/imu/data` - Filtered IMU data
- Output: `/odom` (filtered odometry)
- Frequency: 30 Hz

## Environment Configuration

**Development:**
- Docker container with ROS2 Humble desktop
- Privileged mode for hardware access
- Host networking for ROS2 communication
- Required USB devices: RPlidar, Astra camera

**Time Synchronization:**
- Chrony NTP client
- Server: 192.168.1.246 (hardcoded in Dockerfile)

**Remote Access:**
- Port 6080: noVNC web desktop
- Port 5901: VNC server

## Static Transforms

**Sensor Mounting:**
- `base_link` → `imu_link`: translation (0, 0, 0.02)
- `base_link` → `laser`: translation (-0.03, 0, 0.13), rotation (0, 0, π)
- `astra_link` → `camera_link`: translation (0, 0, 0.02)

**Configured in:**
- `colcon_ws/src/transbot_bringup/launch/bringup.launch.py`

## Third-Party Repositories

**Cloned in Dockerfile:**
1. `https://github.com/libuvc/libuvc.git` - USB video device library
2. `https://github.com/Slamtec/sllidar_ros2.git` - RPlidar driver
3. `https://github.com/orbbec/ros2_astra_camera.git` - Astra camera driver
4. `https://github.com/Adlink-ROS/rf2o_laser_odometry.git` - Laser odometry
5. `https://github.com/introlab/find-object.git` - Object tracking (unused?)

---

*Integration audit: 2026-01-12*
*Update when adding/removing external services*

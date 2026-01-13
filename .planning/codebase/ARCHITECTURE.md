# Architecture

**Analysis Date:** 2026-01-12

## Pattern Overview

**Overall:** Distributed ROS2 Node Architecture with Hardware Integration

**Key Characteristics:**
- Publish-subscribe messaging for loose coupling
- Hardware abstraction layer via UART serial protocol
- Multi-sensor fusion for state estimation
- Action server pattern for mission control
- Hybrid Python (high-level) and C++ (performance-critical) implementation

## Layers

**Hardware Abstraction Layer:**
- Purpose: Encapsulate serial protocol communication with Transbot extension board
- Contains: Motor control, servo actuation, IMU/battery reading, LED/buzzer control
- Location: `colcon_ws/src/transbot_bringup/transbot_bringup/Transbot_Lib.py`
- Depends on: pyserial, UART at 115200 baud
- Used by: TransbotDriver node

**Device Driver Layer:**
- Purpose: Bridge hardware abstraction to ROS2 topics/services
- Contains: `TransbotDriver` (Python), `base_node` (C++)
- Location: `colcon_ws/src/transbot_bringup/scripts/transbot_driver.py`, `src/base_node.cpp`
- Depends on: Hardware abstraction layer, ROS2 rclpy/rclcpp
- Used by: Navigation stack, teleop, mission controller

**Sensor Processing Layer:**
- Purpose: Process raw sensor data into actionable information
- Contains: Human detection (YOLOv11), pose estimation, IMU filtering, laser odometry
- Location: `colcon_ws/src/transbot_bringup/scripts/human_detection_node.py`, `pose_estimator.py`
- Depends on: Camera driver, tf2 transforms
- Used by: Mission controller, state estimation

**State Estimation Layer:**
- Purpose: Fuse multiple sensors for accurate robot pose
- Contains: EKF node (robot_localization), IMU filter (Madgwick), RF2O laser odometry
- Location: External packages, configured via `config/ekf_localization.yaml`
- Depends on: IMU, wheel odometry, laser odometry
- Used by: Navigation stack, mission controller

**Navigation Layer:**
- Purpose: Autonomous path planning and obstacle avoidance
- Contains: Nav2 stack, SLAM Toolbox
- Location: `colcon_ws/src/transbot_bringup/launch/nav2_slam.launch.py`, `config/nav2_params.yaml`
- Depends on: State estimation, LiDAR, costmaps
- Used by: Mission controller

**Mission Control Layer:**
- Purpose: High-level task execution for search and rescue
- Contains: SearchAndRescueActionServer, state machine
- Location: `colcon_ws/src/transbot_bringup/scripts/search_and_rescue_action_server_old.py`
- Depends on: Navigation, human detection, arm control
- Used by: External action client

## Data Flow

**Search and Rescue Mission Flow:**

1. User sends `SearchAndRescue` action goal with list of search poses
2. Action server enters SEARCHING state, navigates to waypoints via Nav2
3. Human detection node processes camera images with YOLOv11
4. On detection, transforms pose from camera frame to map frame
5. Action server switches to APPROACHING_VICTIM, calculates approach pose (0.5m away)
6. Robot navigates to approach pose, enters RESCUING state
7. Arm control aligns with victim (TODO: grasp not implemented)
8. Robot enters RETURNING_TO_BASE, navigates to home pose
9. Action returns success result

**Sensor Data Flow:**

```
[Hardware] → [TransbotDriver] → [cmd_vel/Arm/PWMServo subscriptions]
                ↓
         [Transbot_Lib UART]
                ↓
[IMU data] → [imu_filter_madgwick] → [/imu/data]
                                          ↓
[Wheel odom] → [base_node] → [/odom_raw] → [EKF] → [/odom]
                                          ↑
[LiDAR] → [rf2o_laser_odometry] → [/odom_rf2o]

[Camera RGB] → [compression] → [human_detection_node] → [/human_detections]
                                        ↓
                              [pose_estimator] → [/human_detection/pose]
```

**State Management:**
- Mission state enum: `MissionTask` (NONE → SEARCHING → APPROACHING_VICTIM → RESCUING → RETURNING_TO_BASE)
- Human poses tracked in set data structure
- Navigation state via Nav2 `isTaskComplete()` polling

## Key Abstractions

**Transbot Class:**
- Purpose: Hardware protocol abstraction
- Location: `colcon_ws/src/transbot_bringup/transbot_bringup/Transbot_Lib.py`
- Pattern: Thread-based async receive handler with state machine parser
- Methods: `set_car_motion()`, `set_uart_servo_angle()`, `get_accelerometer_data()`

**ROS2 Node Classes:**
- Purpose: Encapsulate node lifecycle and callbacks
- Examples: `TransbotDriver(Node)`, `HumanDetectionNode(Node)`, `SearchAndRescueActionServer(Node)`
- Pattern: MultiThreadedExecutor with ReentrantCallbackGroup

**MissionTask Enum:**
- Purpose: State machine for search and rescue mission
- Location: `colcon_ws/src/transbot_bringup/transbot_bringup/tasks.py`
- States: NONE, SEARCHING, APPROACHING_VICTIM, RESCUING, RETURNING_TO_BASE

**BasicNavigator:**
- Purpose: High-level navigation interface
- Location: External `nav2_simple_commander.robot_navigator`
- Methods: `goToPose()`, `goThroughPoses()`, `cancelTask()`, `isTaskComplete()`

## Entry Points

**Main Bringup:**
- Location: `colcon_ws/src/transbot_bringup/launch/bringup.launch.py`
- Triggers: `ros2 launch transbot_bringup bringup.launch.py`
- Responsibilities: Launch transbot_driver, sensors, EKF, transforms, robot description

**Navigation + SLAM:**
- Location: `colcon_ws/src/transbot_bringup/launch/nav2_slam.launch.py`
- Triggers: Included from other launches or run standalone
- Responsibilities: SLAM Toolbox (online_async), Nav2 navigation stack

**Data Transport:**
- On-board: `colcon_ws/src/transbot_bringup/launch/transport_onboard.launch.py` - Compress RGB/point clouds
- Off-board: `colcon_ws/src/transbot_bringup/launch/transport_offboard.launch.py` - Decompress and run perception

## Error Handling

**Strategy:** Minimal error handling; exceptions often silently caught

**Patterns:**
- TransbotDriver has signal handler for SIGINT to stop robot
- Transbot_Lib uses bare `except: pass` throughout (anti-pattern)
- Navigation failures not explicitly handled in action server
- Transform exceptions logged but may leave state inconsistent

## Cross-Cutting Concerns

**Logging:**
- ROS2 logger via `self.get_logger().info/warn/error()`
- Legacy print statements in Transbot_Lib (not captured by ROS logging)

**Transforms:**
- Static transforms for sensor mounting (imu_link, laser, camera_link)
- Dynamic transforms from SLAM (map → odom) and odometry (odom → base_link)
- TF2 buffer with lookupTransform for pose transformations

**Threading:**
- MultiThreadedExecutor for concurrent callback processing
- ReentrantCallbackGroup for thread-safe callbacks
- Separate receive thread in Transbot_Lib for async serial data

---

*Architecture analysis: 2026-01-12*
*Update when major patterns change*

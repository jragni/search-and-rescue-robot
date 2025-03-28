# 🤖 Autonomous Search and Rescue Robot

An autonomous navigation and rescue system designed to locate and retrieve lost astronauts on Martian terrain using ROS2, SLAM, and computer vision. The platform is built on a transbot robot from Yahboom.

![Search and Rescue Robot](https://via.placeholder.com/800x400)

## 📋 Overview

This project develops an autonomous robot capable of navigating unknown Martian terrain to locate and rescue lost astronauts (or "Martians"). The robot utilizes LiDAR, IMU, and depth camera sensors for mapping, object detection, and path planning, all integrated within a ROS2 framework.

### Why It Matters

Organizations like NASA, SpaceX, and research institutions are actively developing autonomous systems for harsh environments, object identification, and rescue missions. This project aligns with these efforts, contributing to the advancement of robotics for space exploration and search-and-rescue operations.

## 🔧 System Architecture

### Hardware Components

- Custom-built mobile robot chassis (3D printed or off-the-shelf kit)
- Raspberry Pi 4 / NVIDIA Jetson Nano / ROS2-compatible SBC
- LiDAR (e.g., RPLiDAR A1/A2 or Intel RealSense)
- IMU (e.g., MPU6050)
- Depth Camera (e.g., Intel RealSense D435i)
- Motor controllers (e.g., RoboClaw, ODrive)
- High-torque DC motors / Servo motors
- Battery system and power management

### Software & Frameworks

- ROS2 (Humble)
- rclpy / rclcpp
- SLAM Toolbox / RTAB-Map
- Gazebo Ignition / RViz2
- OpenCV & YOLO
- Docker

### Communication Architecture

- ROS2 Publisher-Subscriber model
- ROS2 Services & Actions
- SLAM-based autonomous navigation
- AI-based target detection using YOLO/OpenCV

## 🚀 Getting Started

### Prerequisites

```bash
# Install ROS2
sudo apt install ros-humble-desktop-full

# Install dependencies
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### Installation

```bash
# Clone the repository
git clone --recurse-submodules https://github.com/jragni/search-and-rescue-robot.git
cd search-and-rescue-robot

# Build the ROS2 workspace
colcon build
source install/setup.bash
```

### Running the Simulation

```bash
# Launch the Gazebo Martian simulation environment
# TODO: Add the launch file for the simulation

# In a new terminal, launch the robot navigation stack
# TODO: Add the launch file for the navigation stack

# In another terminal, start the rescue mission
# TODO: Add the launch file for the rescue mission
```

## 📊 Project Structure

```
search-and-rescue-robot/
├── src/
│   ├── transbot_msgs/
│   │   ├── msg/
│   │   ├── srv/
│   │   ├── action/
│   ├── transbot_bringup/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── src/
│   │   │   ├── navigation/
│   │   │   ├── detection/
│   │   │   ├── rescue/
│   │   ├── worlds/
│   │   ├── models/
│   │   ├── meshes/
├── docker/
├── docs/
```

## 📅 Development Timeline

| Week | Focus | Description |
|------|-------|-------------|
| 1 | Mobile Robot Chassis | Building or assembling the physical robot platform |
| 2 | ROS2 Motor Control | Implementing basic movement control using ROS2 |
| 3 | LiDAR + IMU for Localization | Setting up sensors for robot localization |
| 4 | Path Planning & Obstacle Avoidance | Implementing navigation algorithms |
| 5 | ROS2 Rescue Service | Creating services for rescue operations |
| 6 | Object Detection with AI | Implementing astronaut detection with YOLO |
| 7 | ROS2 Action Server for Retrieval | Creating action servers for rescue missions |
| 8 | Gazebo Martian Environment Simulation | Developing realistic Martian terrain in Gazebo |
| 9 | Full System Integration & Debugging | Final testing and optimization |

## 📷 Screenshots & Demo

# TODO: Add screenshots and demo
![SLAM Mapping](https://via.placeholder.com/400x300)
![Object Detection](https://via.placeholder.com/400x300)
![Path Planning](https://via.placeholder.com/400x300)

## 🧪 Testing

```bash
# Run unit tests
# TODO: Add the unit test launch file

# Run integration tests
# TODO: Add the integration test launch file
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📚 Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [YOLOv11 Documentation](https://docs.ultralytics.com/)

## 📧 Contact

Jhensen Ray Agni - [jhensenrayagni@gmail.com](mailto:jhensenrayagni@gmail.com)

Project Link: [https://github.com/jragni/search-and-rescue-robot](https://github.com/jhensenrayagni/search-and-rescue-robot)

# TODO: Add the Project Link
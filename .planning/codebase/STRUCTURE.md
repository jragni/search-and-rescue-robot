# Codebase Structure

**Analysis Date:** 2026-01-12

## Directory Layout

```
search-and-rescue-robot/
├── colcon_ws/                    # ROS2 workspace
│   └── src/                      # Source packages
│       ├── transbot_bringup/     # Main bringup package
│       ├── transbot_description/ # Robot URDF and meshes
│       └── transbot_msgs/        # Custom message definitions
├── .devcontainer/                # Docker development environment
├── .planning/                    # Project planning docs (this folder)
└── README.md                     # Project documentation
```

## Directory Purposes

**colcon_ws/src/transbot_bringup/**
- Purpose: Main robot bringup, drivers, and control nodes
- Contains: Python scripts, C++ sources, launch files, configuration
- Key files:
  - `scripts/transbot_driver.py` - Hardware driver node
  - `scripts/human_detection_node.py` - YOLOv11 detection
  - `scripts/search_and_rescue_action_server_old.py` - Mission controller
  - `transbot_bringup/Transbot_Lib.py` - Serial protocol handler
- Subdirectories:
  - `scripts/` - Executable Python nodes
  - `src/` - C++ source files
  - `launch/` - Launch files
  - `config/` - YAML parameters and model weights
  - `transbot_bringup/` - Python package modules

**colcon_ws/src/transbot_description/**
- Purpose: Robot URDF model and visualization
- Contains: Xacro files, STL meshes, launch file
- Key files:
  - `urdf/transbot_astra.urdf.xacro` - Robot model with 3-DOF arm
  - `launch/transbot_description.launch.py` - Robot state publisher
- Subdirectories:
  - `urdf/` - URDF/Xacro files
  - `meshes/` - STL collision and visual meshes
  - `launch/` - Launch files

**colcon_ws/src/transbot_msgs/**
- Purpose: Custom ROS2 message, service, and action definitions
- Contains: Interface definition files
- Key files: See msg/, srv/, action/ directories
- Subdirectories:
  - `msg/` - Custom messages (Arm, Joint, Battery, PWMServo, etc.)
  - `srv/` - Custom services (Move, SetSearchPose, GetSearchPoses, etc.)
  - `action/` - Custom actions (SearchAndRescue)

**.devcontainer/**
- Purpose: Docker development environment configuration
- Contains: Dockerfile, devcontainer.json
- Key files:
  - `Dockerfile` - Container build instructions
  - `devcontainer.json` - VS Code remote container settings

## Key File Locations

**Entry Points:**
- `colcon_ws/src/transbot_bringup/launch/bringup.launch.py` - Full system launch
- `colcon_ws/src/transbot_bringup/launch/nav2_slam.launch.py` - Navigation + SLAM
- `colcon_ws/src/transbot_bringup/scripts/transbot_driver.py` - Main hardware driver

**Configuration:**
- `colcon_ws/src/transbot_bringup/config/nav2_params.yaml` - Navigation parameters
- `colcon_ws/src/transbot_bringup/config/mapper_params_online_async.yaml` - SLAM settings
- `colcon_ws/src/transbot_bringup/config/ekf_localization.yaml` - Sensor fusion
- `colcon_ws/src/transbot_bringup/config/yolo11n.pt` - YOLOv11 model weights
- `colcon_ws/src/transbot_bringup/config/bringup.rviz` - RViz visualization

**Core Logic:**
- `colcon_ws/src/transbot_bringup/transbot_bringup/Transbot_Lib.py` - Hardware abstraction
- `colcon_ws/src/transbot_bringup/transbot_bringup/tasks.py` - Mission state enum
- `colcon_ws/src/transbot_bringup/transbot_bringup/helpers.py` - Utility functions
- `colcon_ws/src/transbot_bringup/scripts/human_detection_node.py` - YOLOv11 detection
- `colcon_ws/src/transbot_bringup/scripts/search_and_rescue_action_server_old.py` - Action server

**Robot Model:**
- `colcon_ws/src/transbot_description/urdf/transbot_astra.urdf.xacro` - Main robot URDF
- `colcon_ws/src/transbot_description/meshes/` - STL files for visualization

**Testing:**
- No test files currently present (gap identified)

**Documentation:**
- `README.md` - Project overview

## Naming Conventions

**Files:**
- `snake_case.py` - Python modules and scripts
- `snake_case.launch.py` - Launch files
- `snake_case.yaml` - Configuration files
- `PascalCase_Lib.py` - Library classes (inconsistent, should be snake_case)
- `*.msg`, `*.srv`, `*.action` - ROS2 interface definitions

**Directories:**
- `snake_case` - All directories follow snake_case
- Plural for collections: `scripts/`, `meshes/`, `msg/`, `srv/`

**Special Patterns:**
- `*_node.py` - ROS2 node implementations
- `*_server.py` - Service/action server implementations
- `*_old.py` - Deprecated versions (should be removed)
- `*.launch.py` - Python-based launch files

## Where to Add New Code

**New ROS2 Node:**
- Python implementation: `colcon_ws/src/transbot_bringup/scripts/{name}_node.py`
- C++ implementation: `colcon_ws/src/transbot_bringup/src/{name}_node.cpp`
- Add to CMakeLists.txt install section
- Launch integration: `colcon_ws/src/transbot_bringup/launch/`

**New Message/Service/Action:**
- Definition: `colcon_ws/src/transbot_msgs/{msg,srv,action}/{Name}.{msg,srv,action}`
- Add to CMakeLists.txt rosidl_generate_interfaces section

**New Configuration:**
- YAML parameters: `colcon_ws/src/transbot_bringup/config/{name}.yaml`
- Reference in launch file

**New Utility Function:**
- Location: `colcon_ws/src/transbot_bringup/transbot_bringup/helpers.py`
- Or create new module in same directory

**New Launch File:**
- Location: `colcon_ws/src/transbot_bringup/launch/{name}.launch.py`
- Add to package data in setup.py if needed

## Special Directories

**build/, install/, log/**
- Purpose: Colcon build artifacts
- Source: Generated by `colcon build`
- Committed: No (.gitignore)

**.devcontainer/**
- Purpose: Development container configuration
- Source: Manual configuration
- Committed: Yes

**.planning/**
- Purpose: Project planning documentation
- Source: GSD workflow outputs
- Committed: Yes

**colcon_ws/src/transbot_bringup/config/**
- Purpose: Runtime configuration and model weights
- Source: Manual configuration, downloaded models
- Committed: Yes (including yolo11n.pt)

---

*Structure analysis: 2026-01-12*
*Update when directory structure changes*

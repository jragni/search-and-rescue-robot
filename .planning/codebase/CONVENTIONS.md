# Coding Conventions

**Analysis Date:** 2026-01-12

## Naming Patterns

**Files:**
- `snake_case.py` for all Python files (e.g., `human_detection_node.py`, `transbot_driver.py`)
- Exception: `Transbot_Lib.py` uses PascalCase (inconsistent, should be `transbot_lib.py`)
- `*.launch.py` for Python-based launch files
- `*.msg`, `*.srv`, `*.action` for ROS2 interfaces (PascalCase names inside)

**Functions:**
- `snake_case` for all functions (e.g., `set_car_motion()`, `get_accelerometer_data()`)
- `*_callback()` suffix for ROS2 callbacks (e.g., `cmd_vel_callback()`, `image_sub_callback()`)
- Setter/getter pattern: `set_*()`, `get_*()` for hardware methods

**Variables:**
- `snake_case` for local variables (e.g., `linear_velocity`, `angular_velocity`)
- `UPPER_SNAKE_CASE` for constants (e.g., `FUNC_SET_PID = 0x01`, `FUNC_MOTION = 0x02`)
- Double underscore `__` prefix for private class attributes (e.g., `__uart_state`, `__delay_time`)
- ROS2 member pattern: `*_sub_`, `*_pub_` suffix (e.g., `cmd_vel_sub_`, `imu_pub_`)

**Classes:**
- `PascalCase` for class names (e.g., `Transbot`, `TransbotDriver`, `HumanDetectionNode`)
- `Node` suffix for ROS2 nodes (e.g., `HumanDetectionNode(Node)`)

**Types:**
- Modern type hints used in recent code: `list[float | int]`, `tuple`, function return types
- Location: `colcon_ws/src/transbot_bringup/transbot_bringup/helpers.py`

## Code Style

**Formatting:**
- Indentation: 4 spaces (PEP 8 compliant)
- Line length: Generally follows PEP 8 (~100 characters)
- String quotes: Mix of single and double quotes (no strict convention)
- Semicolons: Absent (correct Python style)
- No formatter configured (black, autopep8, etc.)

**Linting:**
- `ament_lint_auto`, `ament_lint_common` declared in package.xml
- No linting configuration files (`.flake8`, `.pylintrc`) present
- Linting skipped in CMakeLists.txt: `ament_cmake_cpplint_FOUND TRUE`

## Import Organization

**Order (observed pattern):**
1. Standard library imports (e.g., `import math`, `from enum import Enum`)
2. ROS2 imports (e.g., `import rclpy`, `from rclpy.node import Node`)
3. Message imports (e.g., `from geometry_msgs.msg import PoseStamped`)
4. Local imports (e.g., `from transbot_bringup.Transbot_Lib import Transbot`)

**Grouping:**
- No blank lines between groups (inconsistent)
- No automatic sorting (isort not configured)

**Path Aliases:**
- None used; direct imports only

## Error Handling

**Patterns:**
- Bare `except: pass` prevalent in `Transbot_Lib.py` (20+ instances) - anti-pattern
- TransformException caught and logged in detection nodes
- No structured error types; standard Python exceptions used

**Error Types:**
- Hardware errors silently ignored (serial communication failures)
- Transform exceptions logged but may leave state inconsistent
- No custom exception classes defined

**Logging:**
- ROS2 logger: `self.get_logger().info/warn/error()` in nodes
- Legacy `print()` statements in `Transbot_Lib.py` (not ROS2 integrated)

## Logging

**Framework:**
- ROS2 logging via `self.get_logger()` in node classes
- Print statements for debug output in hardware library

**Patterns:**
- Info level for state transitions and mission updates
- Warn level for recoverable issues
- Error level for failures (underutilized)

**Locations:**
- `transbot_driver.py` - Uses ROS2 logger
- `Transbot_Lib.py` - Uses print() (should migrate to logging)

## Comments

**When to Comment:**
- Bilingual comments (Chinese + English) in protocol documentation
- TODO comments for incomplete implementations
- Inline comments for non-obvious logic

**Docstring Style:**
- Modern docstrings with detailed descriptions in `helpers.py`
- Minimal or absent docstrings in older code

**Examples from codebase:**

```python
# Modern style (helpers.py)
def is_arm_within_limits(
    angles: list[float | int],
    x_min: float | int,
    y_min: float | int,
) -> bool:
    """Checks if the arm is within limits..."""

# Legacy style (Transbot_Lib.py)
# 根据数据帧的类型来做出对应的解析
# According to the type of data frame to make the corresponding parsing
```

**TODO Comments:**
- Format: `# TODO description` (no username)
- Present in: `search_and_rescue_action_server_old.py` (lines 151, 166), `transbot_driver.py` (line 187)

## Function Design

**Size:**
- Functions vary widely; some exceed 50 lines
- `execute_callback` in action server is >100 lines (should be split)

**Parameters:**
- Type hints used in newer code
- No strict limit on parameters; some functions have 4+

**Return Values:**
- Explicit returns for functions with return values
- `None` implicit for void functions

## Module Design

**Exports:**
- No barrel files (index.py)
- Direct imports from specific modules

**Package Structure:**
- `transbot_bringup` is both package name and Python module directory
- `__init__.py` present but empty

## ROS2 Naming Conventions

**Topics:**
- snake_case: `/cmd_vel`, `/imu/data_raw`, `/human_detections`
- Hierarchical with `/` separator: `/camera/color/image_raw`, `/transbot/get_vel`

**Services:**
- snake_case: `set_search_pose`, `get_search_poses`, `move`

**Actions:**
- snake_case: `search_and_rescue_server`

**Frames:**
- Standard ROS convention: `base_link`, `base_footprint`, `odom`, `map`
- Sensor frames: `imu_link`, `laser`, `camera_link`, `astra_link`

**Parameters:**
- snake_case: `min_threshold_score`, `use_sim_time`, `publish_tf`

---

*Convention analysis: 2026-01-12*
*Update when patterns change*

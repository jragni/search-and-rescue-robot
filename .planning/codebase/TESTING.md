# Testing Patterns

**Analysis Date:** 2026-01-12

## Test Framework

**Current Status: NO ACTIVE TEST SUITE**

Test dependencies declared but not implemented.

**Runner:**
- None configured

**Assertion Library:**
- None configured

**Run Commands:**
```bash
colcon test                        # Would run tests if they existed
colcon test --packages-select transbot_bringup  # Package-specific
colcon test-result                 # Show results
```

## Test File Organization

**Location:**
- No test files found in `colcon_ws/src/*/test/`
- No `test_*.py` or `*_test.py` files present

**Naming (recommended):**
- `test_{module_name}.py` for unit tests
- `test_{feature}_integration.py` for integration tests
- `test_{launch_file}_launch.py` for launch tests

**Structure (recommended):**
```
colcon_ws/src/transbot_bringup/
├── test/
│   ├── test_helpers.py
│   ├── test_tasks.py
│   ├── test_transbot_driver.py
│   └── test_human_detection_node.py
```

## Test Structure

**Test Dependencies Declared:**

In `package.xml`:
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

In `CMakeLists.txt`:
```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
```

**Note:** `copyright_FOUND` and `cpplint_FOUND` are set to TRUE, effectively skipping those checks.

## Mocking

**Framework:**
- None configured
- Recommended: `unittest.mock` or `pytest-mock`

**What Should Be Mocked:**
- Serial communication (`Transbot_Lib.py`)
- Hardware sensors (camera, lidar, IMU)
- ROS2 subscribers and publishers
- Navigation service calls

**What NOT to Mock:**
- Pure functions in `helpers.py`
- Message construction
- State machine transitions

## Fixtures and Factories

**Test Data (recommended):**

```python
# Factory for test poses
def create_test_pose(x=0.0, y=0.0, z=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

# Factory for arm angles
def create_test_arm_angles(angles=[90, 90, 90]):
    return angles
```

## Coverage

**Requirements:**
- No coverage target defined
- No coverage tool configured

**Recommended Setup:**
- Tool: pytest-cov or colcon-lcov-result
- Target: >80% for critical modules (`helpers.py`, `tasks.py`)

**View Coverage (once configured):**
```bash
colcon test --pytest-args --cov=transbot_bringup
coverage html
```

## Test Types

**Unit Tests (needed):**
- `helpers.py` - Math functions (`is_arm_within_limits()`, `get_approach_pose()`, `is_within_tolerance()`)
- `tasks.py` - MissionTask enum transitions
- `Transbot_Lib.py` - Protocol encoding/decoding (with mocked serial)

**Integration Tests (needed):**
- `transbot_driver.py` - Hardware interface with mocked Transbot_Lib
- `human_detection_node.py` - Detection pipeline with sample images

**Launch Tests (needed):**
- `bringup.launch.py` - Verify all nodes start correctly
- `nav2_slam.launch.py` - SLAM configuration validation

**ROS2-Specific Testing:**
- Framework: `launch_testing` for integration tests
- Topic assertion: Verify message publishing
- Service testing: Test request/response cycles

## Common Patterns

**Recommended Test Structure:**
```python
import pytest
from transbot_bringup.helpers import is_arm_within_limits

class TestArmLimits:
    def test_arm_within_limits_valid(self):
        # arrange
        angles = [90, 90, 90]
        x_min, y_min = -0.1, -0.04

        # act
        result = is_arm_within_limits(angles, x_min, y_min)

        # assert
        assert result is True

    def test_arm_outside_limits(self):
        angles = [180, 180, 180]  # Extreme angles
        result = is_arm_within_limits(angles, -0.1, -0.04)
        assert result is False
```

**Async Testing (for ROS2 nodes):**
```python
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_publishes_message(ros_context):
    # Test implementation
    pass
```

**Error Testing:**
```python
def test_division_by_zero_handling():
    # Robot at same position as victim
    with pytest.raises(ZeroDivisionError):
        get_approach_pose(robot_at_victim_position)
```

## Test Gaps (Critical)

**Priority 1 - Must Test:**
1. `helpers.py` - All math functions (division by zero risk identified)
2. `tasks.py` - State machine transitions
3. `Transbot_Lib.py` - Protocol correctness

**Priority 2 - Should Test:**
1. `transbot_driver.py` - Callback handling
2. `human_detection_node.py` - Detection accuracy
3. Action server state machine

**Priority 3 - Nice to Have:**
1. Launch file validation
2. End-to-end mission tests
3. Hardware-in-loop tests

## Recommended Tools to Add

```
# Testing
pytest >= 7.0.0
pytest-cov >= 3.0.0
pytest-ros >= 0.1.0
launch_testing >= 1.0.0

# Linting
flake8 >= 4.0.0
pylint >= 2.0.0
black >= 22.0.0
isort >= 5.0.0
mypy >= 0.900
```

---

*Testing analysis: 2026-01-12*
*Update when test patterns change*

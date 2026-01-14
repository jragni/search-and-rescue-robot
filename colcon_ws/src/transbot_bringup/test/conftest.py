"""Common test fixtures for transbot_bringup tests."""

import pytest
import sys
from pathlib import Path

# Add package to path for imports
package_path = Path(__file__).parent.parent / "transbot_bringup"
sys.path.insert(0, str(package_path.parent))

# Try to import ROS2 messages, provide mocks if not available
try:
    from geometry_msgs.msg import PoseStamped, TransformStamped
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

    # Create minimal mock classes for testing without ROS2
    class MockPose:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class MockPosition(MockPose):
        pass

    class MockOrientation(MockPose):
        pass

    class MockPoseData:
        def __init__(self):
            self.position = MockPosition()
            self.orientation = MockOrientation()

    class PoseStamped:
        def __init__(self):
            self.pose = MockPoseData()
            self.header = type('Header', (), {'frame_id': '', 'stamp': None})()

    class MockTranslation:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class MockTransform:
        def __init__(self):
            self.translation = MockTranslation()

    class TransformStamped:
        def __init__(self):
            self.transform = MockTransform()


@pytest.fixture
def sample_pose_stamped():
    """Create a sample PoseStamped for testing."""
    pose = PoseStamped()
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


@pytest.fixture
def sample_pose_tuple():
    """Sample 7-tuple representing a pose."""
    return (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)


@pytest.fixture
def sample_transform():
    """Create a sample TransformStamped for robot position."""
    transform = TransformStamped()
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    return transform


@pytest.fixture
def ros2_available():
    """Indicates if real ROS2 messages are available."""
    return ROS2_AVAILABLE

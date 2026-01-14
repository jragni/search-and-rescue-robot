"""Baseline tests for helpers module.

These tests document the current behavior and serve as a safety net
for refactoring. If any of these tests fail after refactoring,
the behavior has changed unexpectedly.
"""

import pytest
import sys
import math
from pathlib import Path
from unittest.mock import MagicMock

# Mock ROS2 geometry_msgs before importing helpers
class MockPosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockOrientation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0

class MockPose:
    def __init__(self):
        self.position = MockPosition()
        self.orientation = MockOrientation()

class MockPoseStamped:
    def __init__(self):
        self.pose = MockPose()
        self.header = MagicMock()

# Create mock geometry_msgs module
mock_geometry_msgs = MagicMock()
mock_geometry_msgs.msg.PoseStamped = MockPoseStamped
sys.modules['geometry_msgs'] = mock_geometry_msgs
sys.modules['geometry_msgs.msg'] = mock_geometry_msgs.msg

# Now add package to path and import helpers
sys.path.insert(0, str(Path(__file__).parent.parent))

from transbot_bringup.helpers import (
    pose_to_tuple,
    tuple_to_pose,
    is_within_tolerance,
    get_approach_pose,
    correct_gyro,
)


# Fixtures
@pytest.fixture
def sample_pose_stamped():
    """Create a sample PoseStamped for testing."""
    pose = MockPoseStamped()
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
    transform = MagicMock()
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    return transform


class TestPoseToTuple:
    """Tests for pose_to_tuple function."""

    def test_converts_pose_to_7_tuple(self, sample_pose_stamped):
        """PoseStamped converts to 7-element tuple."""
        result = pose_to_tuple(sample_pose_stamped)
        assert len(result) == 7

    def test_tuple_ordering_is_xyz_quaternion(self, sample_pose_stamped):
        """Tuple order is (x, y, z, qx, qy, qz, qw)."""
        result = pose_to_tuple(sample_pose_stamped)
        # Expected from fixture: pos(1,2,0) orient(0,0,0,1)
        assert result == (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def test_extracts_position_correctly(self, sample_pose_stamped):
        """Position values are in first 3 elements."""
        result = pose_to_tuple(sample_pose_stamped)
        x, y, z = result[0], result[1], result[2]
        assert x == sample_pose_stamped.pose.position.x
        assert y == sample_pose_stamped.pose.position.y
        assert z == sample_pose_stamped.pose.position.z

    def test_extracts_orientation_correctly(self, sample_pose_stamped):
        """Orientation values are in last 4 elements."""
        result = pose_to_tuple(sample_pose_stamped)
        qx, qy, qz, qw = result[3], result[4], result[5], result[6]
        assert qx == sample_pose_stamped.pose.orientation.x
        assert qy == sample_pose_stamped.pose.orientation.y
        assert qz == sample_pose_stamped.pose.orientation.z
        assert qw == sample_pose_stamped.pose.orientation.w


class TestTupleToPose:
    """Tests for tuple_to_pose function."""

    def test_converts_tuple_to_pose_stamped(self, sample_pose_tuple):
        """7-tuple converts to PoseStamped."""
        result = tuple_to_pose(sample_pose_tuple)
        assert hasattr(result, 'pose')
        assert hasattr(result.pose, 'position')
        assert hasattr(result.pose, 'orientation')

    def test_sets_position_correctly(self, sample_pose_tuple):
        """Position values are set correctly."""
        result = tuple_to_pose(sample_pose_tuple)
        assert result.pose.position.x == sample_pose_tuple[0]
        assert result.pose.position.y == sample_pose_tuple[1]
        assert result.pose.position.z == sample_pose_tuple[2]

    def test_sets_orientation_correctly(self, sample_pose_tuple):
        """Orientation values are set correctly."""
        result = tuple_to_pose(sample_pose_tuple)
        assert result.pose.orientation.x == sample_pose_tuple[3]
        assert result.pose.orientation.y == sample_pose_tuple[4]
        assert result.pose.orientation.z == sample_pose_tuple[5]
        assert result.pose.orientation.w == sample_pose_tuple[6]

    def test_round_trip_preserves_values(self, sample_pose_tuple):
        """pose_to_tuple(tuple_to_pose(t)) == t."""
        pose = tuple_to_pose(sample_pose_tuple)
        result = pose_to_tuple(pose)
        assert result == sample_pose_tuple


class TestIsWithinTolerance:
    """Tests for is_within_tolerance function."""

    def test_returns_true_when_within_tolerance(self):
        """Returns True when pose is within tolerance of a pose in set."""
        human_pose = (1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        pose_set = {(1.05, 1.05, 0.0, 0.0, 0.0, 0.0, 1.0)}
        tolerance = 0.15
        assert is_within_tolerance(human_pose, pose_set, tolerance) is True

    def test_returns_false_when_outside_tolerance(self):
        """Returns False when pose is outside tolerance of all poses."""
        human_pose = (1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        pose_set = {(5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0)}
        tolerance = 0.15
        assert is_within_tolerance(human_pose, pose_set, tolerance) is False

    def test_returns_false_for_empty_set(self):
        """Returns False when pose set is empty."""
        human_pose = (1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        pose_set = set()
        tolerance = 0.15
        assert is_within_tolerance(human_pose, pose_set, tolerance) is False

    def test_returns_true_if_any_pose_matches(self):
        """Returns True if any pose in set is within tolerance."""
        human_pose = (1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        pose_set = {
            (10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0),  # Far away
            (1.05, 1.05, 0.0, 0.0, 0.0, 0.0, 1.0),  # Within tolerance
            (20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0),  # Far away
        }
        tolerance = 0.15
        assert is_within_tolerance(human_pose, pose_set, tolerance) is True

    def test_uses_euclidean_distance(self):
        """Uses 3D Euclidean distance for comparison."""
        human_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        # Distance should be sqrt(0.1^2 + 0.1^2 + 0.1^2) ≈ 0.173
        pose_set = {(0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 1.0)}

        # Tolerance of 0.2 should include it
        assert is_within_tolerance(human_pose, pose_set, 0.2) is True
        # Tolerance of 0.1 should exclude it
        assert is_within_tolerance(human_pose, pose_set, 0.1) is False

    def test_exact_match_within_tolerance(self):
        """Exact same position is within tolerance."""
        human_pose = (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        pose_set = {(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)}
        tolerance = 0.15
        assert is_within_tolerance(human_pose, pose_set, tolerance) is True


class TestGetApproachPose:
    """Tests for get_approach_pose function."""

    def test_returns_pose_stamped(self, sample_transform):
        """Returns a PoseStamped object."""
        victim_pose = (2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        result = get_approach_pose(victim_pose, sample_transform)
        assert hasattr(result, 'pose')

    def test_approach_is_half_meter_from_victim(self, sample_transform):
        """Approach position is 0.5m from victim along robot-to-victim vector."""
        # Robot at origin, victim at (2, 0)
        victim_pose = (2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        result = get_approach_pose(victim_pose, sample_transform)

        # Approach should be at (1.5, 0) - 0.5m before victim
        assert abs(result.pose.position.x - 1.5) < 0.001
        assert abs(result.pose.position.y - 0.0) < 0.001

    def test_approach_faces_victim(self, sample_transform):
        """Approach orientation faces toward victim."""
        # Robot at origin, victim at (2, 0) - should face +X direction (yaw=0)
        victim_pose = (2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        result = get_approach_pose(victim_pose, sample_transform)

        # For yaw=0: qz=0, qw=1
        assert abs(result.pose.orientation.z) < 0.001
        assert abs(result.pose.orientation.w - 1.0) < 0.001

    def test_approach_at_angle(self, sample_transform):
        """Approach works correctly for victim at an angle."""
        # Robot at origin, victim at (1, 1) - 45 degrees
        victim_pose = (1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        result = get_approach_pose(victim_pose, sample_transform)

        # Distance from victim should be 0.5m
        dx = victim_pose[0] - result.pose.position.x
        dy = victim_pose[1] - result.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        assert abs(distance - 0.5) < 0.001

    def test_z_position_is_zero(self, sample_transform):
        """Approach z position is always 0."""
        victim_pose = (2.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0)
        result = get_approach_pose(victim_pose, sample_transform)
        assert result.pose.position.z == 0.0


class TestCorrectGyro:
    """Tests for correct_gyro function."""

    def test_returns_three_values(self):
        """Returns tuple of 3 corrected values."""
        result = correct_gyro(1.0, 0.0, 0.0)
        assert len(result) == 3

    def test_90_degree_z_rotation_x_to_negative_y(self):
        """X-axis input becomes negative Y-axis after -90 degree rotation."""
        ax, ay, az = correct_gyro(1.0, 0.0, 0.0)
        # -90 degree rotation about z: x -> -y
        assert abs(ax - 0.0) < 0.001
        assert abs(ay - 1.0) < 0.001
        assert abs(az - 0.0) < 0.001

    def test_90_degree_z_rotation_y_to_x(self):
        """Y-axis input becomes X-axis after -90 degree rotation."""
        ax, ay, az = correct_gyro(0.0, 1.0, 0.0)
        # -90 degree rotation about z: y -> x
        assert abs(ax - (-1.0)) < 0.001
        assert abs(ay - 0.0) < 0.001
        assert abs(az - 0.0) < 0.001

    def test_z_axis_unchanged(self):
        """Z-axis is unchanged by z-rotation."""
        ax, ay, az = correct_gyro(0.0, 0.0, 1.0)
        assert abs(ax - 0.0) < 0.001
        assert abs(ay - 0.0) < 0.001
        assert abs(az - 1.0) < 0.001

    def test_combined_rotation(self):
        """Combined input rotates correctly."""
        # Input (1, 1, 0) should become (−1, 1, 0) after -90 degree z rotation
        ax, ay, az = correct_gyro(1.0, 1.0, 0.0)
        assert abs(ax - (-1.0)) < 0.001
        assert abs(ay - 1.0) < 0.001
        assert abs(az - 0.0) < 0.001

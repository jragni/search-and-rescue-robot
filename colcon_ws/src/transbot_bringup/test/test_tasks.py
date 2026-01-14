"""Baseline tests for MissionTask enum.

These tests document the current behavior and serve as a safety net
for refactoring. If any of these tests fail after refactoring,
the behavior has changed unexpectedly.
"""

import pytest
import sys
from pathlib import Path

# Add package to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from transbot_bringup.tasks import MissionTask


class TestMissionTaskEnumValues:
    """Tests for MissionTask enum values."""

    def test_none_equals_zero(self):
        """NONE state has value 0."""
        assert MissionTask.NONE.value == 0

    def test_searching_equals_one(self):
        """SEARCHING state has value 1."""
        assert MissionTask.SEARCHING.value == 1

    def test_approaching_victim_equals_two(self):
        """APPROACHING_VICTIM state has value 2."""
        assert MissionTask.APPROACHING_VICTIM.value == 2

    def test_rescuing_equals_three(self):
        """RESCUING state has value 3."""
        assert MissionTask.RESCUING.value == 3

    def test_returning_to_base_equals_four(self):
        """RETURNING_TO_BASE state has value 4."""
        assert MissionTask.RETURNING_TO_BASE.value == 4


class TestMissionTaskEnumProperties:
    """Tests for MissionTask enum properties and behavior."""

    def test_all_states_are_distinct(self):
        """All mission states have unique values."""
        values = [task.value for task in MissionTask]
        assert len(values) == len(set(values)), "Duplicate enum values found"

    def test_enum_has_five_states(self):
        """MissionTask has exactly 5 states."""
        assert len(list(MissionTask)) == 5

    def test_enum_comparison_by_identity(self):
        """Enum values can be compared by identity."""
        task = MissionTask.SEARCHING
        assert task is MissionTask.SEARCHING
        assert task is not MissionTask.NONE

    def test_enum_comparison_by_equality(self):
        """Enum values can be compared by equality."""
        task = MissionTask.RESCUING
        assert task == MissionTask.RESCUING
        assert task != MissionTask.SEARCHING

    def test_enum_comparison_with_value(self):
        """Enum can be compared with integer value."""
        assert MissionTask.NONE.value == 0
        assert MissionTask.SEARCHING.value == 1
        # Note: MissionTask.SEARCHING != 1 (enum vs int)
        # Use .value for integer comparison

    def test_enum_can_be_created_from_value(self):
        """Enum can be created from integer value."""
        assert MissionTask(0) == MissionTask.NONE
        assert MissionTask(1) == MissionTask.SEARCHING
        assert MissionTask(2) == MissionTask.APPROACHING_VICTIM
        assert MissionTask(3) == MissionTask.RESCUING
        assert MissionTask(4) == MissionTask.RETURNING_TO_BASE

    def test_enum_name_attribute(self):
        """Enum has correct name attributes."""
        assert MissionTask.NONE.name == "NONE"
        assert MissionTask.SEARCHING.name == "SEARCHING"
        assert MissionTask.APPROACHING_VICTIM.name == "APPROACHING_VICTIM"
        assert MissionTask.RESCUING.name == "RESCUING"
        assert MissionTask.RETURNING_TO_BASE.name == "RETURNING_TO_BASE"

    def test_invalid_value_raises_error(self):
        """Creating enum from invalid value raises ValueError."""
        with pytest.raises(ValueError):
            MissionTask(99)

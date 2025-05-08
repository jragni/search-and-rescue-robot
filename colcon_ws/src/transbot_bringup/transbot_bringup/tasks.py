from enum import Enum


class MissionTask(Enum):
    """Tasks for search and rescue.

    The mission state begins with 'NONE'. Once the rescue
    request is received, the robot will be tasked with 'SEARCHING'
    If a human is detected outside of the safe zone and
    is in 'SEARCHING' state, the robot will switch cancel the current goal
    and move to 'APPROACHING'.
    Once at the pose to rescue, the robot will be in the 'RESCUE' state where
    the arm will grasp the victim and then return to base, 'RETURNING_TO_BASE'.
    """
    NONE = 0
    SEARCHING = 1
    APPROACHING_VICTIM = 2
    RESCUING = 3
    RETURNING_TO_BASE = 4
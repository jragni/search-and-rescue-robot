#!usr/bin/env python3

from math import cos, sin, pi

def is_arm_within_limits(
    angles: list[float | int],
    x_min: float,
    y_min: float,
) -> bool:
    """Checks if the arm is within limits

    x = L1 * cos(t1) + L2 * cos(t1 + t2) >= x_min
    y = L1 * sin(t1) + L2 * sin(t1 + t2) >= y_min
    """
    # Horizontal is at 90
    t1 = (angles[0] - 90.0) * pi / 180.0
    # Horizontal is at 180
    t2 = (angles[1] - 180.0) * pi / 180.0

    # Measured links
    L1 = 0.08  # [cm]
    L2 = 0.14  # [cm]

    # Forward Kinematics
    x = L1 * cos(t1) + (L2 * cos(t1 + t2))
    y = L1 * sin(t1) + (L2 * sin(t1 + t2))

    print(f'x: {x}, y: {y}')
    return x >= x_min and y >= y_min
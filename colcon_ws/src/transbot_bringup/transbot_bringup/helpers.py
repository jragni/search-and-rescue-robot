#!usr/bin/env python3

from math import cos, sin, pi
import numpy as np

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


def correct_gyro(ax, ay, az):
    """correct the gyro readings.

    Since the IMU is mounted with the x facing 90 degrees clockwise
    about the z-axis, we need to adjust it
    """
    p = np.transpose(np.array([ax, ay, az]))
    rotation_matrix = [
        [cos(-pi / 2), sin(-pi/2), 0],
        [sin(-pi / 2), cos(-pi / 2), 0],
        [0, 0, 1]
    ]
    corrected = np.dot(rotation_matrix, p)
    return corrected[0], corrected[1], corrected[2]

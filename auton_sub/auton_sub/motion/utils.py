"""
Various mathematical utility functions for robot navigation and control.
Compatible with ROS 2.
"""

import math

def heading_error(heading: float, target: float) -> float:
    """
    Calculate the signed heading error (difference between target and current heading).
    
    Args:
        heading (float): Current heading in degrees [0, 360)
        target (float): Target heading in degrees [0, 360)

    Returns:
        float: Error in degrees, within [-180, 180], useful for PID heading control
    """
    error = (target - heading + 180) % 360 - 180
    return error


def get_norm(x: float, y: float) -> float:
    """
    Compute the Euclidean norm (2D vector magnitude).
    
    Args:
        x (float): x-component
        y (float): y-component

    Returns:
        float: sqrt(x² + y²), the length of the vector
    """
    return math.sqrt(x**2 + y**2)


def get_distance(v1: tuple, v2: tuple) -> float:
    """
    Calculate the straight-line (Euclidean) distance between two 2D points.

    Args:
        v1 (tuple): (x1, y1)
        v2 (tuple): (x2, y2)

    Returns:
        float: distance between the two points
    """
    return math.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)


def rotate_vector(x: float, y: float, heading: float) -> tuple:
    """
    Rotate a vector (x, y) counter-clockwise by a given angle (heading in degrees).

    Args:
        x (float): x-component of vector
        y (float): y-component of vector
        heading (float): rotation angle in degrees

    Returns:
        tuple: rotated (x, y)
    """
    theta = math.radians(heading)
    x_rot = x * math.cos(theta) - y * math.sin(theta)
    y_rot = x * math.sin(theta) + y * math.cos(theta)
    return x_rot, y_rot


def inv_rotate_vector(x: float, y: float, heading: float) -> tuple:
    """
    Perform the inverse of `rotate_vector`, i.e., rotate clockwise by `heading`.

    Args:
        x (float): x-component
        y (float): y-component
        heading (float): angle in degrees

    Returns:
        tuple: inversely rotated vector (x, y)
    """
    theta = math.radians(heading)
    x_rot = x * math.cos(theta) + y * math.sin(theta)
    y_rot = -x * math.sin(theta) + y * math.cos(theta)
    return x_rot, y_rot


def get_heading_from_coords(x: float, y: float) -> float:
    """
    Compute heading angle (in degrees) from 2D coordinates, assuming:
        - x-axis points East
        - y-axis points North (standard navigation frame)

    Args:
        x (float): x-coordinate (East)
        y (float): y-coordinate (North)

    Returns:
        float: heading in degrees from North (0°) clockwise
    """
    return math.degrees(math.atan2(x, y))

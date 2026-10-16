"""Pure helpers for the odometry-based differential-drive controller."""

from math import atan2, cos, hypot, sin


def yaw_from_quaternion(x, y, z, w):
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle):
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle < -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle


def drive_command(pose, goal, distance_tolerance=0.08):
    x, y, yaw = pose
    goal_x, goal_y = goal
    dx, dy = goal_x - x, goal_y - y
    distance = hypot(dx, dy)
    if distance <= distance_tolerance:
        return 0.0, 0.0, True
    heading_error = wrap_angle(atan2(dy, dx) - yaw)
    angular = max(-0.8, min(0.8, 2.0 * heading_error))
    if abs(heading_error) > 0.28:
        return 0.0, angular, False
    linear = min(0.18, 0.7 * distance) * max(0.15, cos(heading_error))
    return linear, angular, False

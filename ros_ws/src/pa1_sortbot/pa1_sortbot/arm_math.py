"""Planar inverse kinematics for the mobile sorter arm."""

from math import acos, atan2, cos, hypot, sin

ARM_BASE_X = 0.04
ARM_BASE_Z = 0.07
BASE_LINK_WORLD_Z = 0.19
LINK_LENGTHS = (0.22, 0.22)
TOOL_OFFSET = 0.04
GRIPPER_CLOSED = 0.03
GRIPPER_OPEN = 0.0
SHOULDER_LIMITS = (-2.0, 2.0)
ELBOW_LIMITS = (-0.1, 2.9)
WRIST_LIMITS = (-2.9, 2.9)


def solve_arm_ik(target_x, target_z):
    """Return shoulder, elbow, and wrist pitch for a base-frame target."""
    link_1, link_2 = LINK_LENGTHS
    wrist_x = target_x - ARM_BASE_X - TOOL_OFFSET
    wrist_z = target_z - ARM_BASE_Z
    distance = hypot(wrist_x, wrist_z)
    if distance <= 1e-6 or distance > link_1 + link_2 - 1e-4:
        raise ValueError(
            f"Arm target is outside the reachable workspace: {distance:.3f} m"
        )

    elbow_cos = (
        distance * distance - link_1 * link_1 - link_2 * link_2
    ) / (2.0 * link_1 * link_2)
    elbow = acos(max(-1.0, min(1.0, elbow_cos)))
    shoulder = atan2(-wrist_z, wrist_x) - atan2(
        link_2 * sin(elbow), link_1 + link_2 * cos(elbow)
    )
    wrist = -(shoulder + elbow)
    for name, angle, limits in (
        ("shoulder", shoulder, SHOULDER_LIMITS),
        ("elbow", elbow, ELBOW_LIMITS),
        ("wrist", wrist, WRIST_LIMITS),
    ):
        if not limits[0] <= angle <= limits[1]:
            raise ValueError(
                f"Arm target exceeds the {name} joint limits: {angle:.3f} rad"
            )
    return shoulder, elbow, wrist


def make_arm_positions(target_x, target_z, gripper_position):
    """Return the five position commands for the arm and parallel gripper."""
    shoulder, elbow, wrist = solve_arm_ik(target_x, target_z)
    return (
        shoulder,
        elbow,
        wrist,
        gripper_position,
        gripper_position,
    )

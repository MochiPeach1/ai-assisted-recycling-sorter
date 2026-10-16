import math
import unittest

from pa1_sortbot.arm_math import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    LINK_LENGTHS,
    TOOL_OFFSET,
    make_arm_positions,
    solve_arm_ik,
)


class ArmKinematicsTests(unittest.TestCase):
    def assert_target_reached(self, target_x, target_z):
        shoulder, elbow, wrist = solve_arm_ik(target_x, target_z)
        link_1, link_2 = LINK_LENGTHS
        tool_angle = shoulder + elbow + wrist
        actual_x = (
            link_1 * math.cos(shoulder)
            + link_2 * math.cos(shoulder + elbow)
            + TOOL_OFFSET * math.cos(tool_angle)
            + 0.04
        )
        actual_z = (
            -link_1 * math.sin(shoulder)
            - link_2 * math.sin(shoulder + elbow)
            - TOOL_OFFSET * math.sin(tool_angle)
            + 0.07
        )
        self.assertAlmostEqual(actual_x, target_x, places=6)
        self.assertAlmostEqual(actual_z, target_z, places=6)
        self.assertAlmostEqual(tool_angle, 0.0, places=6)

    def test_reaches_package_pickup_pose(self):
        self.assert_target_reached(0.30, -0.05)

    def test_reaches_pregrasp_pose(self):
        self.assert_target_reached(0.30, 0.04)

    def test_reaches_bin_drop_pose(self):
        self.assert_target_reached(0.43, 0.0)

    def test_reaches_stowed_carry_pose(self):
        self.assert_target_reached(0.25, 0.32)

    def test_rejects_target_outside_arm_workspace(self):
        with self.assertRaisesRegex(ValueError, "outside the reachable workspace"):
            solve_arm_ik(1.0, 0.0)

    def test_rejects_target_outside_joint_limits(self):
        with self.assertRaisesRegex(ValueError, "shoulder joint limits"):
            solve_arm_ik(0.12, 0.25)

    def test_gripper_positions_are_symmetric(self):
        opened = make_arm_positions(0.30, -0.20, GRIPPER_OPEN)
        closed = make_arm_positions(0.30, -0.20, GRIPPER_CLOSED)
        self.assertEqual(opened[3:], (GRIPPER_OPEN, GRIPPER_OPEN))
        self.assertEqual(closed[3:], (GRIPPER_CLOSED, GRIPPER_CLOSED))


if __name__ == "__main__":
    unittest.main()

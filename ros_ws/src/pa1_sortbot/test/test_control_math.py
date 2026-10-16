import math
import unittest

from pa1_sortbot.control_math import drive_command, wrap_angle, yaw_from_quaternion


class ControlMathTests(unittest.TestCase):
    def test_arrival_stops_robot(self):
        self.assertEqual(drive_command((1.0, 2.0, 0.0), (1.05, 2.0)), (0.0, 0.0, True))

    def test_turns_before_moving_toward_sideways_goal(self):
        linear, angular, arrived = drive_command((0.0, 0.0, 0.0), (0.0, 2.0))
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)
        self.assertFalse(arrived)

    def test_moves_forward_for_aligned_goal(self):
        linear, angular, arrived = drive_command((0.0, 0.0, 0.0), (1.0, 0.0))
        self.assertGreater(linear, 0.0)
        self.assertEqual(angular, 0.0)
        self.assertFalse(arrived)

    def test_wraps_angle_to_shortest_turn(self):
        self.assertAlmostEqual(wrap_angle(3.5), 3.5 - 6.283185307179586)

    def test_extracts_planar_yaw_from_quaternion(self):
        half_yaw = math.pi / 4.0
        self.assertAlmostEqual(
            yaw_from_quaternion(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
            math.pi / 2.0,
        )


if __name__ == "__main__":
    unittest.main()

"""ROS 2 mission node for the mobile-arm recycling sorter."""

from collections import Counter, deque
from math import atan2, cos, sin
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pa1_sortbot.arm_math import (
    BASE_LINK_WORLD_Z,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    make_arm_positions,
)
from pa1_sortbot.control_math import drive_command, wrap_angle, yaw_from_quaternion
from pa1_sortbot.mission import BIN_APPROACH_OFFSET, BIN_POSES, ITEMS
from pa1_sortbot.vision import NearestCentroidClassifier, sample_center_pixels

ARM_JOINTS = (
    "arm_shoulder_joint",
    "arm_elbow_joint",
    "arm_wrist_joint",
    "left_gripper_joint",
    "right_gripper_joint",
)
ITEM_CENTER_Z = 0.07
BIN_ITEM_CENTER_Z = 0.12
CARRY_TARGET = (0.25, 0.32)


class SorterNode(Node):
    def __init__(self):
        super().__init__("ai_recycling_sorter")
        sample_file = Path(get_package_share_directory("pa1_sortbot")) / "data" / "training_rgb.csv"
        self.classifier = NearestCentroidClassifier.from_csv(sample_file)
        self.velocity_publisher = self.create_publisher(Twist, "/sortbot/cmd_vel", 10)
        self.arm_publisher = self.create_publisher(
            JointTrajectory, "/sortbot/set_joint_trajectory", 10
        )
        self.class_publisher = self.create_publisher(String, "/sortbot/recognized_class", 10)
        self.status_publisher = self.create_publisher(String, "/sortbot/sort_status", 10)
        self.create_subscription(Odometry, "/sortbot/odom", self.on_odometry, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self.on_model_states, 10)
        self.create_subscription(Image, "/sortbot/camera/image_raw", self.on_image, 5)
        self.create_timer(0.1, self.tick)
        self.pose = None
        self.item_index = 0
        self.phase = "stow_arm"
        self.arm_next_phase = None
        self.arm_deadline_ns = 0
        self.arm_controller_wait_started_ns = None
        self.arm_positions = (0.0, 0.0, 0.0, GRIPPER_OPEN, GRIPPER_OPEN)
        self.label_window = deque(maxlen=5)
        self.current_label = None
        self.model_poses = {}
        self.phase_started = self.get_clock().now()
        self.last_status = None
        self.get_logger().info("Mobile-arm sorter ready: physical grasping with a parallel gripper.")
        self.publish_status("Starting the sorting mission")

    def on_odometry(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.pose = (position.x, position.y, yaw)

    def on_model_states(self, message):
        self.model_poses = {
            name: (pose.position.x, pose.position.y, pose.position.z)
            for name, pose in zip(message.name, message.pose)
        }

    def on_image(self, image):
        try:
            pixels = sample_center_pixels(image)
        except ValueError as error:
            self.get_logger().warning(f"Skipping camera frame: {error}")
            return
        label, confidence, count = self.classifier.classify_pixels(pixels)
        if label is None:
            self.label_window.append(None)
            return
        self.label_window.append(label)
        message = String()
        message.data = f"{label} (confidence={confidence:.2f}, pixels={count})"
        self.class_publisher.publish(message)

    def tick(self):
        if self.phase in ("done", "failed") or self.pose is None:
            self.publish_velocity(0.0, 0.0)
            if self.phase in ("done", "failed"):
                self.publish_arm_hold()
            return

        if self.phase == "arm_wait":
            self.publish_velocity(0.0, 0.0)
            if self.get_clock().now().nanoseconds >= self.arm_deadline_ns:
                self.phase = self.arm_next_phase
                if self.phase in ("verify_grasp", "verify_placement"):
                    self.phase_started = self.get_clock().now()
                self.publish_arm_hold()
            return

        if self.phase in (
            "approach_item",
            "align_item",
            "classify_item",
            "verify_grasp",
            "approach_bin",
            "verify_transport",
            "align_bin",
            "finish_item",
            "verify_placement",
        ):
            self.publish_arm_hold()

        if self.phase == "stow_arm":
            self.move_arm_to_base(
                *CARRY_TARGET,
                GRIPPER_OPEN,
                "approach_item",
                1.8,
            )
            return

        if self.phase == "approach_item":
            _, _, _, goal_x, goal_y = ITEMS[self.item_index]
            self.navigate_to(goal_x, goal_y)
            if self.reached(goal_x, goal_y):
                self.phase = "align_item"
                self.set_status(f"Aligning camera with package {self.item_index + 1}")
            return

        if self.phase == "align_item":
            _, item_x, item_y, _, _ = ITEMS[self.item_index]
            self.align_to(item_x, item_y, "classify_item")
            return

        if self.phase == "classify_item":
            self.publish_velocity(0.0, 0.0)
            label = self.stable_label()
            if label is not None:
                self.current_label = label
                self.set_status(f"Recognized {label}; approaching package with the arm")
                _, item_x, item_y, _, _ = ITEMS[self.item_index]
                self.move_arm_to_world(
                    item_x,
                    item_y,
                    ITEM_CENTER_Z + 0.09,
                    GRIPPER_OPEN,
                    "lower_arm",
                    1.8,
                )
            elif (self.get_clock().now() - self.phase_started).nanoseconds > 15_000_000_000:
                self.fail("No stable package classification in 15 seconds; check camera alignment.")
            return

        if self.phase == "lower_arm":
            _, item_x, item_y, _, _ = ITEMS[self.item_index]
            self.move_arm_to_world(
                item_x,
                item_y,
                ITEM_CENTER_Z,
                GRIPPER_OPEN,
                "close_gripper",
                1.5,
            )
            self.set_status(f"Lowering gripper onto package {self.item_index + 1}")
            return

        if self.phase == "close_gripper":
            self.command_arm(
                (
                    self.arm_positions[0],
                    self.arm_positions[1],
                    self.arm_positions[2],
                    GRIPPER_CLOSED,
                    GRIPPER_CLOSED,
                ),
                "lift_package",
                0.8,
            )
            self.set_status(f"Closing the gripper around package {self.item_index + 1}")
            return

        if self.phase == "lift_package":
            self.move_arm_to_base(
                *CARRY_TARGET,
                GRIPPER_CLOSED,
                "verify_grasp",
                1.8,
            )
            self.set_status(f"Lifting package {self.item_index + 1}")
            return

        if self.phase == "verify_grasp":
            in_gripper = self.package_is_in_gripper()
            if in_gripper is True:
                self.phase = "approach_bin"
                self.set_status(f"Package secured; driving to the {self.current_label} area")
            elif in_gripper is False:
                self.fail(f"Gripper did not pick up package {self.item_index + 1}.")
            else:
                self.wait_for_model_pose(
                    f"Verifying that package {self.item_index + 1} is held by the gripper"
                )
            return

        if self.phase == "approach_bin":
            bin_x, bin_y = BIN_POSES[self.current_label]
            approach_x = bin_x - BIN_APPROACH_OFFSET
            self.navigate_to(approach_x, bin_y, 0.015)
            if self.reached(approach_x, bin_y, 0.015):
                self.phase = "verify_transport"
                self.phase_started = self.get_clock().now()
                self.publish_velocity(0.0, 0.0)
            return

        if self.phase == "verify_transport":
            in_gripper = self.package_is_in_gripper()
            if in_gripper is True:
                self.phase = "align_bin"
                self.set_status(f"Package secured at the {self.current_label} area")
            elif in_gripper is None:
                self.wait_for_model_pose("Waiting for Gazebo package pose updates")
            else:
                self.fail(
                    f"Package {self.item_index + 1} was not held throughout the drive."
                )
            return

        if self.phase == "align_bin":
            bin_x, bin_y = BIN_POSES[self.current_label]
            self.align_to(bin_x, bin_y, "lower_package")
            return

        if self.phase == "lower_package":
            bin_x, bin_y = BIN_POSES[self.current_label]
            self.set_status(f"Lowering package into the {self.current_label} area")
            self.move_arm_to_world(
                bin_x,
                bin_y,
                BIN_ITEM_CENTER_Z,
                GRIPPER_CLOSED,
                "release_package",
                1.8,
            )
            return

        if self.phase == "release_package":
            self.command_arm(
                (
                    self.arm_positions[0],
                    self.arm_positions[1],
                    self.arm_positions[2],
                    GRIPPER_OPEN,
                    GRIPPER_OPEN,
                ),
                "retreat_arm",
                0.8,
            )
            self.set_status(f"Releasing package in the {self.current_label} area")
            return

        if self.phase == "retreat_arm":
            self.move_arm_to_base(
                *CARRY_TARGET,
                GRIPPER_OPEN,
                "verify_placement",
                1.8,
            )
            return

        if self.phase == "verify_placement":
            placed = self.package_is_in_bin()
            if placed is True:
                self.phase = "finish_item"
                self.set_status(
                    f"Package {self.item_index + 1} placed in the {self.current_label} area"
                )
            elif placed is False:
                self.fail(
                    f"Package {self.item_index + 1} did not land in the "
                    f"{self.current_label} area."
                )
            else:
                self.wait_for_model_pose("Waiting to verify package placement")
            return

        if self.phase == "finish_item":
            self.item_index += 1
            if self.item_index == len(ITEMS):
                self.phase = "done"
                self.publish_status("All packages sorted successfully")
                self.get_logger().info("Mission complete: all packages were grasped and placed.")
            else:
                self.phase = "approach_item"
                self.set_status(f"Moving to package {self.item_index + 1}")

    def move_arm_to_world(self, world_x, world_y, world_z, gripper, next_phase, seconds):
        dx = world_x - self.pose[0]
        dy = world_y - self.pose[1]
        local_x = cos(self.pose[2]) * dx + sin(self.pose[2]) * dy
        local_y = -sin(self.pose[2]) * dx + cos(self.pose[2]) * dy
        if abs(local_y) > 0.06:
            self.fail(
                f"Arm target is {local_y:.2f} m off the robot centerline; "
                "realign the mobile base before grasping."
            )
            return
        try:
            positions = make_arm_positions(
                local_x,
                world_z - BASE_LINK_WORLD_Z,
                gripper,
            )
        except ValueError as error:
            self.fail(str(error))
            return
        self.command_arm(positions, next_phase, seconds)

    def move_arm_to_base(self, target_x, target_z, gripper, next_phase, seconds):
        try:
            positions = make_arm_positions(target_x, target_z, gripper)
        except ValueError as error:
            self.fail(str(error))
            return
        self.command_arm(positions, next_phase, seconds)

    def command_arm(self, target, next_phase, seconds):
        if self.arm_publisher.get_subscription_count() == 0:
            now_ns = self.get_clock().now().nanoseconds
            if self.arm_controller_wait_started_ns is None:
                self.arm_controller_wait_started_ns = now_ns
            elif now_ns - self.arm_controller_wait_started_ns > 10_000_000_000:
                self.fail("Gazebo arm trajectory controller did not become available.")
                return
            self.set_status("Waiting for the Gazebo arm controller")
            return
        self.arm_controller_wait_started_ns = None

        trajectory = JointTrajectory()
        trajectory.header.frame_id = "world"
        trajectory.joint_names = list(ARM_JOINTS)
        steps = max(2, int(seconds * 20))
        interval_ns = round(seconds / steps * 1_000_000_000)
        for step in range(1, steps + 1):
            fraction = step / steps
            point = JointTrajectoryPoint()
            point.positions = [
                current + fraction * (goal - current)
                for current, goal in zip(self.arm_positions, target)
            ]
            point.time_from_start = Duration(
                sec=interval_ns // 1_000_000_000,
                nanosec=interval_ns % 1_000_000_000,
            )
            trajectory.points.append(point)

        self.arm_publisher.publish(trajectory)
        self.arm_positions = target
        self.arm_next_phase = next_phase
        self.arm_deadline_ns = self.get_clock().now().nanoseconds + int(
            (seconds + 0.1) * 1_000_000_000
        )
        self.phase = "arm_wait"

    def publish_arm_hold(self):
        if self.arm_publisher.get_subscription_count() == 0:
            self.fail("Gazebo arm trajectory controller connection was lost.")
            return
        trajectory = JointTrajectory()
        trajectory.header.frame_id = "world"
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(self.arm_positions)
        trajectory.points.append(point)
        self.arm_publisher.publish(trajectory)

    def package_is_in_gripper(self):
        package = self.model_poses.get(ITEMS[self.item_index][0])
        if package is None:
            return None
        expected_x = self.pose[0] + CARRY_TARGET[0] * cos(self.pose[2])
        expected_y = self.pose[1] + CARRY_TARGET[0] * sin(self.pose[2])
        distance = ((package[0] - expected_x) ** 2 + (package[1] - expected_y) ** 2) ** 0.5
        return distance <= 0.16 and 0.40 <= package[2] <= 0.80

    def package_is_in_bin(self):
        package = self.model_poses.get(ITEMS[self.item_index][0])
        if package is None:
            return None
        bin_x, bin_y = BIN_POSES[self.current_label]
        distance = ((package[0] - bin_x) ** 2 + (package[1] - bin_y) ** 2) ** 0.5
        return distance <= 0.16 and 0.02 <= package[2] <= 0.24

    def wait_for_model_pose(self, status):
        if (self.get_clock().now() - self.phase_started).nanoseconds > 3_000_000_000:
            self.fail("Gazebo model-state updates were unavailable for package verification.")
        else:
            self.set_status(status)

    def align_to(self, world_x, world_y, next_phase):
        target_yaw = atan2(world_y - self.pose[1], world_x - self.pose[0])
        heading_error = wrap_angle(target_yaw - self.pose[2])
        if abs(heading_error) <= 0.05:
            self.publish_velocity(0.0, 0.0)
            self.phase = next_phase
            self.phase_started = self.get_clock().now()
            if next_phase == "classify_item":
                self.label_window.clear()
                self.set_status(f"Inspecting package {self.item_index + 1}")
            return
        angular = max(-0.5, min(0.5, 1.5 * heading_error))
        self.publish_velocity(0.0, angular)

    def navigate_to(self, goal_x, goal_y, tolerance=0.08):
        linear, angular, _ = drive_command(self.pose, (goal_x, goal_y), tolerance)
        self.publish_velocity(linear, angular)

    def reached(self, goal_x, goal_y, tolerance=0.08):
        return drive_command(self.pose, (goal_x, goal_y), tolerance)[2]

    def stable_label(self):
        labels = [label for label in self.label_window if label is not None]
        if not labels:
            return None
        label, count = Counter(labels).most_common(1)[0]
        return label if count >= 4 and len(self.label_window) >= 4 else None

    def publish_velocity(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.velocity_publisher.publish(message)

    def set_status(self, status):
        if status != self.last_status:
            self.publish_status(status)
            self.get_logger().info(status)
            self.last_status = status

    def publish_status(self, status):
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def fail(self, message):
        self.phase = "failed"
        self.publish_velocity(0.0, 0.0)
        self.get_logger().error(message)
        self.publish_status("Mission failed: " + message)


def main(args=None):
    rclpy.init(args=args)
    node = SorterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

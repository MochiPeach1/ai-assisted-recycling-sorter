"""ROS 2 mission node for the camera-guided recycling sorter."""

from collections import Counter, deque
from math import atan2, cos, sin
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pa1_sortbot.control_math import drive_command, wrap_angle, yaw_from_quaternion
from pa1_sortbot.mission import BIN_APPROACH_OFFSET, BIN_POSES, ITEMS
from pa1_sortbot.vision import NearestCentroidClassifier, sample_center_pixels


class SorterNode(Node):
    def __init__(self):
        super().__init__("ai_recycling_sorter")
        sample_file = Path(get_package_share_directory("pa1_sortbot")) / "data" / "training_rgb.csv"
        self.classifier = NearestCentroidClassifier.from_csv(sample_file)
        self.velocity_publisher = self.create_publisher(Twist, "/sortbot/cmd_vel", 10)
        self.class_publisher = self.create_publisher(String, "/sortbot/recognized_class", 10)
        self.status_publisher = self.create_publisher(String, "/sortbot/sort_status", 10)
        self.create_subscription(Odometry, "/sortbot/odom", self.on_odometry, 10)
        self.create_subscription(Image, "/sortbot/camera/image_raw", self.on_image, 5)
        self.entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.timer = self.create_timer(0.1, self.tick)
        self.pose = None
        self.item_index = 0
        self.phase = "approach_item"
        self.label_window = deque(maxlen=5)
        self.current_label = None
        self.carried_entity = None
        self.pending_future = None
        self.pending_action = None
        self.last_carry_update = self.get_clock().now()
        self.phase_started = self.get_clock().now()
        self.last_status = None
        self.get_logger().info("Sorter ready: three packages, local RGB classifier, no external AI service.")
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
            return

        if self.pending_future is not None and self.pending_future.done():
            self.finish_entity_request()
        if self.phase in ("pickup_pending", "drop_pending"):
            self.publish_velocity(0.0, 0.0)
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
            target_yaw = atan2(item_y - self.pose[1], item_x - self.pose[0])
            heading_error = wrap_angle(target_yaw - self.pose[2])
            if abs(heading_error) <= 0.05:
                self.publish_velocity(0.0, 0.0)
                self.phase = "classify_item"
                self.phase_started = self.get_clock().now()
                self.label_window.clear()
                self.set_status(f"Inspecting package {self.item_index + 1}")
            else:
                angular = max(-0.5, min(0.5, 1.5 * heading_error))
                self.publish_velocity(0.0, angular)
            return

        if self.phase == "classify_item":
            self.publish_velocity(0.0, 0.0)
            label = self.stable_label()
            if label is not None:
                self.current_label = label
                if not self.entity_client.service_is_ready():
                    self.set_status("Waiting for Gazebo entity-state service")
                    return
                self.set_status(f"Recognized {label}; picking up package {self.item_index + 1}")
                self.request_entity_pose(
                    ITEMS[self.item_index][0],
                    self.pose[0] + 0.25 * cos(self.pose[2]),
                    self.pose[1] + 0.25 * sin(self.pose[2]),
                    0.38,
                    "pickup",
                )
            elif (self.get_clock().now() - self.phase_started).nanoseconds > 15_000_000_000:
                self.fail("No stable package classification in 15 seconds; check camera alignment.")
            return

        if self.phase == "approach_bin":
            bin_x, bin_y = BIN_POSES[self.current_label]
            approach_x = bin_x - BIN_APPROACH_OFFSET
            self.navigate_to(approach_x, bin_y)
            self.update_carried_pose()
            if self.reached(approach_x, bin_y) and self.pending_future is None:
                self.set_status(f"Placing package in the {self.current_label} bin")
                self.request_entity_pose(
                    ITEMS[self.item_index][0],
                    bin_x,
                    bin_y,
                    0.07,
                    "drop",
                )

    def navigate_to(self, goal_x, goal_y):
        linear, angular, _ = drive_command(self.pose, (goal_x, goal_y))
        self.publish_velocity(linear, angular)

    def reached(self, goal_x, goal_y):
        return drive_command(self.pose, (goal_x, goal_y))[2]

    def stable_label(self):
        labels = [label for label in self.label_window if label is not None]
        if not labels:
            return None
        label, count = Counter(labels).most_common(1)[0]
        return label if count >= 4 and len(self.label_window) >= 4 else None

    def request_entity_pose(self, entity_name, x, y, z, action):
        if not self.entity_client.service_is_ready():
            self.set_status("Waiting for Gazebo entity-state service")
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = entity_name
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.w = 1.0
        self.pending_action = action
        self.pending_future = self.entity_client.call_async(request)
        if action == "pickup":
            self.phase = "pickup_pending"
        elif action == "drop":
            self.phase = "drop_pending"

    def finish_entity_request(self):
        future = self.pending_future
        action = self.pending_action
        self.pending_future = None
        self.pending_action = None
        error = future.exception()
        if error is not None:
            self.fail(f"Gazebo entity-state request failed: {error}")
            return
        result = future.result()
        if result is None or not result.success:
            detail = "no response" if result is None else result.status_message
            self.fail(f"Gazebo could not update {ITEMS[self.item_index][0]}: {detail}")
            return
        if action == "pickup":
            self.carried_entity = ITEMS[self.item_index][0]
            self.phase = "approach_bin"
            self.set_status(f"Carrying package to the {self.current_label} bin")
        elif action == "drop":
            self.carried_entity = None
            self.item_index += 1
            if self.item_index == len(ITEMS):
                self.phase = "done"
                self.publish_status("All packages sorted successfully")
                self.get_logger().info("Mission complete: all three packages sorted.")
            else:
                self.phase = "approach_item"
                self.set_status(f"Moving to package {self.item_index + 1}")
        elif action == "carry":
            self.last_carry_update = self.get_clock().now()

    def update_carried_pose(self):
        if self.carried_entity is None or self.pending_future is not None:
            return
        now = self.get_clock().now()
        if (now - self.last_carry_update).nanoseconds < 400_000_000:
            return
        x = self.pose[0] + 0.25 * cos(self.pose[2])
        y = self.pose[1] + 0.25 * sin(self.pose[2])
        self.request_entity_pose(self.carried_entity, x, y, 0.38, "carry")

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

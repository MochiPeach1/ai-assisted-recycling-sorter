"""Record the Gazebo overview camera and the mission status into an MP4 file.

Usage inside the ROS 2 container (after starting sort_demo.launch.py):
    ros2 run gazebo_ros spawn_entity.py -entity demo_overview_camera -file scripts/demo_camera.sdf
    python3 scripts/record_demo.py arm_sorting_demo.mp4
"""

import subprocess
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

FRAME_RATE = 15
PLAYBACK_SPEED = 2


class DemoRecorder(Node):
    def __init__(self, output_path):
        super().__init__("demo_recorder")
        self.output_path = output_path
        self.encoder = None
        self.status = "Starting the sorting mission"
        self.finished_frames = None
        self.create_subscription(Image, "/demo/overview/image_raw", self.on_image, 5)
        self.create_subscription(String, "/sortbot/sort_status", self.on_status, 10)

    def on_status(self, message):
        self.status = message.data
        if message.data.startswith(("All packages", "Mission failed")):
            self.finished_frames = 0
            self.get_logger().info(f"Final status: {message.data}")

    def open_encoder(self, width, height):
        text = "textfile=/tmp/demo_status.txt:reload=1"
        self.encoder = subprocess.Popen(
            [
                "ffmpeg", "-y", "-loglevel", "error",
                "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", f"{width}x{height}", "-r", str(FRAME_RATE * PLAYBACK_SPEED),
                "-i", "-",
                "-vf",
                f"drawtext={text}:x=20:y=20:fontsize=26:fontcolor=white:"
                "box=1:boxcolor=black@0.55:boxborderw=8",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26",
                self.output_path,
            ],
            stdin=subprocess.PIPE,
        )

    def on_image(self, image):
        if self.encoder is None:
            self.open_encoder(image.width, image.height)
        with open("/tmp/demo_status.txt.tmp", "w", encoding="utf-8") as handle:
            handle.write(self.status)
        subprocess.run(["mv", "/tmp/demo_status.txt.tmp", "/tmp/demo_status.txt"], check=True)
        self.encoder.stdin.write(bytes(image.data))
        if self.finished_frames is not None:
            self.finished_frames += 1
            if self.finished_frames > FRAME_RATE * 4:
                raise SystemExit

    def close(self):
        if self.encoder is not None:
            self.encoder.stdin.close()
            self.encoder.wait()


def main():
    rclpy.init()
    with open("/tmp/demo_status.txt", "w", encoding="utf-8") as handle:
        handle.write("Starting")
    recorder = DemoRecorder(sys.argv[1] if len(sys.argv) > 1 else "arm_sorting_demo.mp4")
    try:
        rclpy.spin(recorder)
    except SystemExit:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

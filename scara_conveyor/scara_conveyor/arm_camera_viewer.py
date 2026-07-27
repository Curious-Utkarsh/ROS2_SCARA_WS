#!/usr/bin/env python3
"""Show the SCARA arm-mounted camera feed in a small window (view only).

This camera plays no role in the pick-and-place logic -- it just displays the
live video from the camera on the robot, next to the fixed-camera detector.
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WINDOW = "SCARA Arm Camera"


class ArmCameraViewer(Node):
    def __init__(self):
        super().__init__("arm_camera_viewer")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        topic = self.get_parameter("image_topic").value
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)

        self.bridge = CvBridge()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, width, height)
        self.create_subscription(Image, topic, self.on_image, 10)
        self.get_logger().info(f"Showing arm camera feed from {topic}.")

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow(WINDOW, image)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArmCameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

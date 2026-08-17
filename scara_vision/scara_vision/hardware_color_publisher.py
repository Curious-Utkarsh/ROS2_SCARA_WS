#!/usr/bin/env python3
"""Publish the colour the webcam sees, with sliders to tune it and boxes to see it.

    ros2 run scara_vision color_publisher
    ros2 run scara_vision color_publisher --ros-args -p camera:=0

    ros2 topic echo /detected_color

Drag the sliders until each object gets a box and nothing else does.
Press 's' to print the tuned numbers, 'q' to quit.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Starting thresholds: H min, S min, V min, H max, S max, V max.
# Red's H min is ABOVE its H max on purpose -- red sits at both ends of the hue
# scale, so its range wraps around. make_mask() below handles that.
#
# conveyor_color_gate.py reads this table too, so tune it here -- with the
# sliders -- and the conveyor cell picks the tuning up with no changes.
THRESHOLDS = {
    "red":    [170, 100, 70, 10, 255, 255],
    "green":  [40, 70, 50, 85, 255, 255],
    "yellow": [20, 120, 120, 35, 255, 255],
    # Blue starts from the simulation's range (a deep navy, capped in value so
    # the robot's own blue links do not match). A real webcam will almost
    # certainly need this one retuned with the sliders.
    "blue":   [105, 120, 30, 130, 255, 160],
}

# Colour to draw each box in, in BGR.
BOX_COLOUR = {
    "red":    (0, 0, 255),
    "green":  (0, 255, 0),
    "yellow": (0, 255, 255),
    "blue":   (255, 0, 0),
}

SLIDER_NAMES = ["H min", "S min", "V min", "H max", "S max", "V max"]
SLIDER_MAXES = [179, 255, 255, 179, 255, 255]

MIN_PIXELS = 2000        # smaller blobs than this are ignored
SLIDERS = "thresholds"   # name of the slider window


def make_mask(hsv, threshold):
    """White where the image matches this colour, black everywhere else."""
    h_lo, s_lo, v_lo, h_hi, s_hi, v_hi = threshold

    if h_lo <= h_hi:
        mask = cv2.inRange(hsv, (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))
    else:
        # A wrapping range, like red: take both ends and add them together.
        mask = (cv2.inRange(hsv, (0, s_lo, v_lo), (h_hi, s_hi, v_hi))
                | cv2.inRange(hsv, (h_lo, s_lo, v_lo), (179, s_hi, v_hi)))

    # Rub out stray specks so one object gives one blob.
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))


class ColorPublisher(Node):

    def __init__(self):
        super().__init__("color_publisher")

        self.declare_parameter("camera", 4)
        self.camera = cv2.VideoCapture(self.get_parameter("camera").value)

        self.publisher = self.create_publisher(String, "detected_color", 10)
        self.last = None
        self.done = False

        # Six sliders per colour, all in one window.
        cv2.namedWindow(SLIDERS, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(SLIDERS, 400, 560)
        for colour, values in THRESHOLDS.items():
            for name, value, top in zip(SLIDER_NAMES, values, SLIDER_MAXES):
                cv2.createTrackbar(f"{colour} {name}", SLIDERS, value, top, lambda _: None)

        self.create_timer(0.03, self.tick)

    def read_sliders(self, colour):
        """Where the six sliders for this colour are sitting right now."""
        return [cv2.getTrackbarPos(f"{colour} {name}", SLIDERS) for name in SLIDER_NAMES]

    def draw_box(self, frame, mask, colour):
        """Box the biggest blob of this colour. Returns how big it was."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0

        biggest = max(contours, key=cv2.contourArea)
        pixels = int(cv2.contourArea(biggest))
        if pixels < MIN_PIXELS:
            return 0

        x, y, w, h = cv2.boundingRect(biggest)
        cv2.rectangle(frame, (x, y), (x + w, y + h), BOX_COLOUR[colour], 2)
        cv2.putText(frame, colour, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, BOX_COLOUR[colour], 2)
        return pixels

    def print_values(self):
        """Print the sliders so you can paste them back into THRESHOLDS above."""
        print("THRESHOLDS = {")
        for colour in THRESHOLDS:
            print(f'    "{colour}": {self.read_sliders(colour)},')
        print("}")

    def tick(self):
        ok, frame = self.camera.read()
        if not ok:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Box every colour we can see, and keep the biggest one.
        best, most = "none", MIN_PIXELS
        for colour in THRESHOLDS:
            mask = make_mask(hsv, self.read_sliders(colour))
            pixels = self.draw_box(frame, mask, colour)
            if pixels > most:
                best, most = colour, pixels

        self.publisher.publish(String(data=best))

        if best != self.last:
            self.get_logger().info(f"saw {best}")
            self.last = best

        cv2.putText(frame, best, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imshow("camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            self.print_values()
        if key in (ord("q"), 27):
            self.done = True


def main():
    rclpy.init()
    node = ColorPublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.camera.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

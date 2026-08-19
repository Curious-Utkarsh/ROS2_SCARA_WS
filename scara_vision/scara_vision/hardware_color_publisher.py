#!/usr/bin/env python3
"""Publish the colour the webcam sees, with sliders to tune it and boxes to see it.

    ros2 run scara_vision color_publisher
    ros2 run scara_vision color_publisher --ros-args -p camera:=0

    ros2 topic echo /detected_color

Three windows open:

    camera    the picture, with a box round every colour that was found
    masks     what each colour's thresholds actually select, side by side
    cropped   the same picture with the top and bottom cut off, so you can see
              what is left once the near edge of the belt and the background
              above it are out of the way

Drag the sliders until each object is white in ITS OWN mask panel and black in
the other two. That is easier to judge than watching the boxes flicker, because
a mask shows you the specks and the holes that a box hides.

Press 's' to print the tuned numbers, 'q' to quit.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Starting thresholds: H min, S min, V min, H max, S max, V max.
# A colour whose H min is ABOVE its H max wraps around the end of the hue scale,
# which is how red is usually described. make_mask() below handles that.
#
# conveyor_color_gate.py reads this table too, so tune it here -- with the
# sliders -- and the conveyor cell picks the tuning up with no changes.
#
# Measured on this cell's webcam, 2026-08-19.
THRESHOLDS = {
    "red":    [144, 118, 195, 135, 229, 255],
    "green":  [0, 85, 15, 83, 255, 211],
    "yellow": [11, 34, 0, 48, 102, 255],
}

# Colour to draw each box in, in BGR.
BOX_COLOUR = {
    "red":    (0, 0, 255),
    "green":  (0, 255, 0),
    "yellow": (0, 255, 255),
}

SLIDER_NAMES = ["H min", "S min", "V min", "H max", "S max", "V max"]
SLIDER_MAXES = [179, 255, 255, 179, 255, 255]

MIN_PIXELS = 2000        # smaller blobs than this are ignored
CROP_ROWS = 130          # rows cut off the top AND the bottom by default
SLIDERS = "thresholds"   # name of the slider window
MASKS = "masks"          # name of the mask window
CROPPED = "cropped"      # name of the cut-down view

PANEL_WIDTH = 320        # how wide each mask panel is drawn


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


def crop_edges(image, rows):
    """`rows` taken off the TOP and the same off the BOTTOM of a frame.

    What is left is the middle band: the belt, without the near edge and the
    table below it or the background above it. conveyor_color_gate.py measures
    on the result of this and this node draws it, so the cut is defined in
    exactly one place.

    Because the top is cut too, a point in the returned picture is `rows` lower
    in the original -- see how the gate offsets its boxes when it draws them on
    the whole frame. An impossible number is ignored rather than obeyed, so no
    caller can end up with an empty picture.
    """
    height = image.shape[0]
    if rows <= 0 or 2 * rows >= height:
        return image
    return image[rows:height - rows]


def mask_gallery(masks):
    """Every mask side by side in one picture, each with its name on it.

    The masks are black and white, so they are tinted with the colour they
    belong to before being joined up -- otherwise three white-on-black panels
    are impossible to tell apart at a glance.
    """
    panels = []
    for colour, mask in masks.items():
        height = int(mask.shape[0] * PANEL_WIDTH / mask.shape[1])
        small = cv2.resize(mask, (PANEL_WIDTH, height), interpolation=cv2.INTER_NEAREST)

        # Grey -> BGR, then knock out the channels this colour does not use, so
        # the green mask comes out green and the red one red.
        panel = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
        panel[:] = panel * (np.array(BOX_COLOUR[colour], dtype=np.float32) / 255.0)

        white = int(cv2.countNonZero(small))
        cv2.putText(panel, f"{colour}  {white} px", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        panels.append(panel)

    return cv2.hconcat(panels) if panels else None


class ColorPublisher(Node):

    def __init__(self):
        super().__init__("color_publisher")

        self.declare_parameter("camera", 4)

        # How many rows to cut off the TOP and off the BOTTOM of the picture for
        # the "cropped" window. In THIS node it is a view, not a filter: the
        # boxes and /detected_color still come from the whole frame, so you can
        # see what the picture would look like with the near edge of the belt
        # and the background out of shot. Tune it here, then pass the same
        # number to conveyor_color_gate, which really does measure on the
        # cut-down picture.
        self.declare_parameter("crop_rows", CROP_ROWS)

        self.camera = cv2.VideoCapture(self.get_parameter("camera").value)
        self.crop_rows = int(self.get_parameter("crop_rows").value)
        self.warned_about_crop = False

        self.publisher = self.create_publisher(String, "detected_color", 10)
        self.last = None
        self.done = False

        # Six sliders per colour, all in one window.
        cv2.namedWindow(SLIDERS, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(SLIDERS, 400, 460)
        for colour, values in THRESHOLDS.items():
            for name, value, top in zip(SLIDER_NAMES, values, SLIDER_MAXES):
                cv2.createTrackbar(f"{colour} {name}", SLIDERS, value, top, lambda _: None)

        cv2.namedWindow(MASKS, cv2.WINDOW_NORMAL)
        cv2.namedWindow(CROPPED, cv2.WINDOW_NORMAL)

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

    def crop(self, image):
        """`crop_rows` off the top and the bottom, complaining if it cannot.

        The camera view, the cropped view and every mask panel all go through
        here, so the three windows can never disagree about where the cuts are.
        """
        height = image.shape[0]

        if 2 * self.crop_rows >= height and not self.warned_about_crop:
            self.get_logger().warn(
                f"crop_rows is {self.crop_rows} top and bottom, which is the "
                f"whole {height}-row picture -- showing it uncropped"
            )
            self.warned_about_crop = True

        return crop_edges(image, self.crop_rows)

    def tick(self):
        ok, frame = self.camera.read()
        if not ok:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Box every colour we can see, and keep the biggest one. The masks are
        # kept as we go so the gallery below shows exactly what was measured,
        # not a second set built from sliders that may have moved since.
        masks = {}
        best, most = "none", MIN_PIXELS
        for colour in THRESHOLDS:
            masks[colour] = make_mask(hsv, self.read_sliders(colour))
            pixels = self.draw_box(frame, masks[colour], colour)
            if pixels > most:
                best, most = colour, pixels

        self.publisher.publish(String(data=best))

        if best != self.last:
            self.get_logger().info(f"saw {best}")
            self.last = best

        cv2.putText(frame, best, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imshow("camera", frame)

        gallery = mask_gallery({c: self.crop(m) for c, m in masks.items()})
        if gallery is not None:
            cv2.imshow(MASKS, gallery)

        cv2.imshow(CROPPED, self.crop(frame))

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

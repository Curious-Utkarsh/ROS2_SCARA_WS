#!/usr/bin/env python3
"""Watch the belt and shout when the wanted colour reaches the middle.

    ros2 run scara_vision conveyor_color_gate
    ros2 run scara_vision conveyor_color_gate --ros-args -p camera:=0 -p target_color:=red

This is the eye of the real conveyor cell. It does one job and then keeps quiet:

    a red box drifts into view ... still off to the left ... still off ...
    its middle crosses the middle of the picture  ->  say so, once

It says so by publishing on /conveyor/box_at_pickup, which is the very same
message the simulated belt controller sends when a box settles at the pick
point. That is the whole trick: everything downstream -- freezing the belt,
triggering the pick, waiting for it to finish, starting the belt again -- is
conveyor_state_machine.py, running completely unchanged. The simulation and the
real cell are the same pipeline with a different pair of eyes.

    camera --> [this node] --> /conveyor/box_at_pickup
                                        |
                             conveyor_state_machine
                                 |            |
                        /conveyor/run     /conveyor/execute_pick
                          (belt off)         (arm picks it up)

WHICH COLOUR ARE WE WATCHING FOR?
The pick node announces it on /conveyor/target_color, exactly as in simulation,
so start that node and this one follows. The target_color parameter is only a
fallback for testing this node on its own.

HOW DO WE KNOW WHEN THE BOX HAS GONE?
The state machine sends /conveyor/box_removed once the arm has finished with it.
Until then this node stays quiet, so one box can never be announced twice --
which matters, because a box being grasped sits in the middle of the picture for
several seconds.

The HSV ranges come from hardware_color_publisher.py, so tune them there (it has
sliders) and this node picks the tuning up with no changes.

WHAT IS ACTUALLY LOOKED AT
Everything is measured on the CROPPED picture -- `crop_rows` taken off the top
and the same off the bottom, the same cut hardware_color_publisher.py shows you
in its "cropped" window, and the same number is the parameter here. Outside that
band sit the near edge of the belt, the table below it and the background above
it, which is where phantom blobs come from. The whole frame is never measured,
only drawn.

Three windows open:

    conveyor colour gate     the whole picture, for looking at. Both crop lines
                             are drawn on it: only the band between them counts.
    gate: measured on this   the cropped picture -- what the decision was made
                             from, with every colour boxed
    gate: masks              what each colour's thresholds select on the cropped
                             picture, side by side

Every colour in the table is found and boxed, not just the wanted one: seeing
green boxed while you wait for red is what tells you the camera is fine and the
red box simply is not there yet. Only the wanted colour stops the belt.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from scara_vision.hardware_color_publisher import (
    CROP_ROWS,
    THRESHOLDS,
    crop_edges,
    make_mask,
    mask_gallery,
)

# Latched, matching conveyor_state_machine.py: a colour announced before this
# node started is still waiting for us when we subscribe.
LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# The state machine and the pick node speak in letters; the HSV table and the
# human speak in words.
LETTER_FOR = {"red": "R", "green": "G", "blue": "B", "yellow": "Y"}
WORD_FOR = {letter: word for word, letter in LETTER_FOR.items()}

BOX_COLOUR = {"red": (0, 0, 255), "green": (0, 255, 0),
              "blue": (255, 0, 0), "yellow": (0, 255, 255)}

WINDOW_CAMERA = "conveyor colour gate"      # whole frame, drawn on only
WINDOW_CROPPED = "gate: measured on this"   # the cut-down frame the decision uses
WINDOW_MASKS = "gate: masks"                # what the thresholds select

# Topics -- the same names scara_conveyor/common.py uses.
TOPIC_TARGET_COLOR = "/conveyor/target_color"
TOPIC_BOX_AT_PICKUP = "/conveyor/box_at_pickup"
TOPIC_BOX_REMOVED = "/conveyor/box_removed"


class ConveyorColorGate(Node):

    def __init__(self):
        super().__init__("conveyor_color_gate")

        self.declare_parameter("camera", 4)
        self.declare_parameter("target_color", "")      # fallback; topic wins
        self.declare_parameter("center_tolerance_px", 40)
        self.declare_parameter("min_pixels", 2000)
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("show_window", True)

        # Rows cut off the top, and the same off the bottom, before anything is
        # measured. Same cut, and the same default, as
        # hardware_color_publisher.py's "cropped" window -- so tune it by eye
        # over there, then pass the number you settled on here.
        self.declare_parameter("crop_rows", CROP_ROWS)

        self.tolerance = int(self.get_parameter("center_tolerance_px").value)
        self.min_pixels = int(self.get_parameter("min_pixels").value)
        self.confirm_frames = int(self.get_parameter("confirm_frames").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.crop_rows = int(self.get_parameter("crop_rows").value)

        self.camera = cv2.VideoCapture(self.get_parameter("camera").value)
        if not self.camera.isOpened():
            raise RuntimeError(
                f"Cannot open camera {self.get_parameter('camera').value}. "
                "Try a different -p camera:=N (v4l2-ctl --list-devices)."
            )

        self.target = self.starting_target()
        self.centred_frames = 0     # how many frames in a row it has been centred
        self.announced = False      # said so about the box in front of us
        self.counter = 0            # names the boxes we announce
        self.done = False

        self.pickup_publisher = self.create_publisher(String, TOPIC_BOX_AT_PICKUP, 10)
        self.create_subscription(
            String, TOPIC_TARGET_COLOR, self.target_callback, LATCHED
        )
        self.create_subscription(
            String, TOPIC_BOX_REMOVED, self.removed_callback, 10
        )

        self.create_timer(0.03, self.tick)
        self.get_logger().info(
            f"Watching for {self.target or 'a colour nobody has named yet'}; "
            f"centre band is +/-{self.tolerance} pixels, "
            f"top and bottom {self.crop_rows} rows ignored."
        )

    # -- who are we looking for -------------------------------------------

    def starting_target(self):
        """The target_color parameter, as a word, or None."""
        given = str(self.get_parameter("target_color").value).strip().lower()
        if not given:
            return None
        if given in THRESHOLDS:
            return given
        if given.upper() in WORD_FOR:
            return WORD_FOR[given.upper()]
        self.get_logger().warn(f"Do not know the colour {given!r}; ignoring it")
        return None

    def target_callback(self, message):
        """The pick node has said which colour it wants. It wins over the param."""
        letter = message.data.strip().upper()
        colour = WORD_FOR.get(letter)

        if colour is None:
            self.get_logger().warn(f"Do not know the colour {message.data!r}")
            return
        if colour not in THRESHOLDS:
            self.get_logger().warn(
                f"No HSV range for {colour} -- add one to hardware_color_publisher.py"
            )
            return

        if colour != self.target:
            self.get_logger().info(f"Now watching for {colour}")
            self.rearm()
        self.target = colour

    def removed_callback(self, message):
        """That box has been dealt with. Start looking for the next one."""
        self.get_logger().info(f"{message.data.strip()} dealt with; watching again")
        self.rearm()

    def rearm(self):
        self.announced = False
        self.centred_frames = 0

    # -- looking ------------------------------------------------------------

    def tick(self):
        ok, frame = self.camera.read()
        if not ok:
            return

        # `view` is a numpy slice of `frame`, so drawing on it would draw on
        # `frame` too. Nothing is ever drawn on it: it is measured, and the
        # copy made further down is what gets the boxes.
        view = crop_edges(frame, self.crop_rows)
        middle = view.shape[1] // 2

        masks, blobs = self.find_all(view)
        blob = blobs.get(self.target)

        if blob is None:
            self.centred_frames = 0
        else:
            x, _, w, _ = blob
            off_by = (x + w // 2) - middle

            if abs(off_by) <= self.tolerance:
                self.centred_frames += 1
            else:
                self.centred_frames = 0

            # Wait for a few frames in a row before believing it. A single frame
            # can be a reflection, or the arm passing through the picture.
            if self.centred_frames >= self.confirm_frames and not self.announced:
                self.announce()

        if self.show_window:
            self.show(frame, view, masks, blobs, middle)

    def find_all(self, view):
        """Every colour's mask, and the biggest blob of each, on the cropped view.

        All of them, not just the target: the gate only acts on the wanted
        colour, but boxing the others is what tells you at a glance that the
        camera and the thresholds are working and the box is simply not there.
        """
        hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)

        masks, blobs = {}, {}
        for colour, threshold in THRESHOLDS.items():
            masks[colour] = make_mask(hsv, threshold)

            contours, _ = cv2.findContours(
                masks[colour], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            biggest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest) >= self.min_pixels:
                blobs[colour] = cv2.boundingRect(biggest)

        return masks, blobs

    # -- speaking -----------------------------------------------------------

    def announce(self):
        """Tell the state machine a box of the wanted colour is at the pickup.

        The name is ours to invent -- unlike the simulation there is no Gazebo
        entity behind it -- but the state machine quotes it back in
        /conveyor/execute_pick and /conveyor/box_removed, so it needs to be
        different every time to be worth reading in a log.
        """
        self.counter += 1
        name = f"cam_box_{self.counter}"
        letter = LETTER_FOR[self.target]

        self.pickup_publisher.publish(String(data=f"{name};{letter}"))
        self.announced = True
        self.get_logger().info(f"{name} ({self.target}) is centred -- stop the belt")

    # -- drawing ------------------------------------------------------------

    def show(self, frame, view, masks, blobs, middle):
        """The three windows: the whole picture, what was measured, the masks."""
        # How far down the whole frame the measured band starts. Taken from the
        # two pictures rather than from crop_rows, so a cut that was too big to
        # obey -- and therefore ignored -- cannot shift the boxes.
        top = (frame.shape[0] - view.shape[0]) // 2

        cropped = view.copy()
        self.draw_blobs(cropped, blobs, middle, 0)
        self.draw_overlay(cropped, middle)

        # The same boxes on the whole frame, pushed back down by the rows that
        # were cut off the top.
        self.draw_blobs(frame, blobs, middle, top)
        self.draw_overlay(frame, middle)
        self.draw_crop_band(frame, top)

        cv2.imshow(WINDOW_CROPPED, cropped)
        cv2.imshow(WINDOW_CAMERA, frame)

        gallery = mask_gallery(masks)
        if gallery is not None:
            cv2.imshow(WINDOW_MASKS, gallery)

        if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
            self.done = True

    def draw_blobs(self, image, blobs, middle, y_offset):
        """Box every colour found, `y_offset` rows further down the picture.

        Cropping takes rows off the top, so a blob measured at y is at
        y + crop_rows in the whole frame; the cropped window passes 0.
        """
        for colour, (x, y, w, h) in blobs.items():
            y += y_offset
            wanted = colour == self.target
            shade = BOX_COLOUR.get(colour, (255, 255, 255))
            weight = 2 if wanted else 1

            cv2.rectangle(image, (x, y), (x + w, y + h), shade, weight)
            if wanted:
                cv2.line(image, (x + w // 2, y), (x + w // 2, y + h), shade, 1)
                label = f"{colour} {(x + w // 2) - middle:+d} px"
            else:
                label = colour

            cv2.putText(image, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, shade, weight)

    def draw_crop_band(self, frame, top):
        """The two lines the measured band lives between."""
        if top <= 0:
            return

        width, bottom = frame.shape[1], frame.shape[0] - top
        for y in (top, bottom):
            cv2.line(frame, (0, y), (width, y), (0, 165, 255), 2)

        cv2.putText(frame, "measured between these lines", (10, top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    def draw_overlay(self, image, middle):
        height = image.shape[0]

        # The centre line, and the band a box has to be inside to count.
        cv2.line(image, (middle, 0), (middle, height), (255, 255, 255), 1)
        for edge in (middle - self.tolerance, middle + self.tolerance):
            cv2.line(image, (edge, 0), (edge, height), (120, 120, 120), 1)

        if self.target is None:
            state = "waiting to be told a colour"
        elif self.announced:
            state = f"{self.target}: announced, belt should be stopped"
        elif self.centred_frames:
            state = f"{self.target}: centred {self.centred_frames}/{self.confirm_frames}"
        else:
            state = f"{self.target}: watching"

        cv2.putText(image, state, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)


def main():
    rclpy.init()
    node = ConveyorColorGate()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.camera.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

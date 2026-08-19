#!/usr/bin/env python3
"""The pick node for the REAL conveyor cell -- a blind pick at the belt centre.

    ros2 run scara_conveyor hardware_pick_executor --ros-args -p target_color:=R

Same job as conveyor_pick_executor.py, same drop positions, but it never looks
at the box it is picking. It does not have to:

    conveyor_color_gate stops the belt when a box of the wanted colour has its
    middle on the middle of the camera picture. Every box therefore comes to
    rest at the SAME PLACE, under the camera centre. So the pick pose is a fixed
    pose, worked out once.

That is a better fit for real hardware than looking, for two reasons. The arm
covers the box as it comes down, so a camera would lose it at the worst moment;
and this robot has no depth camera on the bench anyway.

    conveyor_state_machine --> /conveyor/execute_pick --> [this node]
                                                              |
                    open, down, close, up, drop, home  (MoveIt)
                                                              |
                        /scara/pick_completed --> belt runs again

WHERE THE POSES COME FROM

    drop positions   IDENTICAL to the simulation. They are imported from
                     conveyor_pick_executor.py, not copied, so the two can
                     never drift apart. Red and Green are the calibrated pair;
                     Yellow and Blue are their exact mirror images.

    pick position    two ways, chosen with pick_source:

                     "taught" (the default) uses the four numbers in
                     pick_approach -- TAUGHT_PICK_APPROACH below, read off
                     scara_gui.py with the arm parked on a box at the belt
                     centre on 2026-08-19. Teach it again the same way if the
                     belt or the camera moves: park the arm, read the four
                     joint values, put them in pick_approach and put the
                     shoulder value in grasp_shoulder.

                     "ik" asks MoveIt for the joint angles that reach the belt
                     pickup point the SIMULATION uses, scara_conveyor/common.py's
                     pickup_grasp_point_base(). Right only if your belt sits
                     where the simulated one does.

The grasp itself copies the simulation's trick: keep the approach pose and just
wind the shoulder down. One joint moves, so nothing can swing into the box on
the way down. The distance it winds down is the difference between the two
shoulder numbers below -- 84 mm here, where the simulation drops the full 150.

    *** THE ARM IS OPEN LOOP. Park it at zero before powering the drivers, or
    every pose in this file points somewhere else. ***
"""

import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import scara

from scara_conveyor import common

# The drop positions, straight from the simulation's pick node. Importing them
# rather than copying is the whole point: retune Red there and the real cell
# follows, mirrors and all.
from scara_conveyor.conveyor_pick_executor import (
    DROP_TARGETS,
    GRASP_JOINT_INDEX,
    _mirror_pose,
)

LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# How far above the box to line up before coming down, in metres. Only the IK
# path uses this; the taught pose below carries its own approach height in its
# shoulder number.
APPROACH_HEIGHT = 0.10

# ---- The real cell's pick pose, taught on the hardware (2026-08-19) ---------
#
# Read off scara_gui.py with the arm parked on a box at the belt centre:
#
#     J -0.5770 -0.0840 1.7180 1.0750 0.0000
#       column   shoulder forearm wrist  gripper
#
# The three that aim the arm -- column, forearm, wrist -- are used as taught.
# The shoulder is the one joint that moves between lining up and grasping, so
# it appears twice:
#
#     TAUGHT_PICK_APPROACH's shoulder   0.0     held while lining up, 84 mm
#                                               clear of the box
#     TAUGHT_GRASP_SHOULDER            -0.084   wound down to, the taught height
#
# Only the shoulder differs between the two, so the arm comes straight down and
# nothing can swing sideways into the box. The shoulder's range is -0.15 to
# 0.02 (arm.xacro), so both numbers are inside it with room to spare.
#
# The SIMULATION grasps at -0.15, the bottom of the travel, because its belt is
# lower under the arm than the real one. That is why this node no longer takes
# the simulation's GRASP_SHOULDER_POSITION as its default: -0.15 would drive the
# gripper 66 mm THROUGH the real belt.
# How long to stand still on either side of a gripper command, in seconds.
#
# The arm is OPEN LOOP with wait_for_done off, so MoveIt reporting a move as
# executed only means it finished sending the trajectory -- the steppers are
# still catching up for a moment afterwards. Closing the gripper at that instant
# closes it while the arm is still coming down, on the edge of the box or on
# nothing at all. The same wait after the command gives the servo time to
# actually grip before the arm lifts.
SETTLE_SECONDS = 1.0

# The shoulder's real speed, from scara_moveit/config/joint_limits.yaml: 8 mm
# lead, 100000 steps/m, capped at 1200 steps/s. It is the slowest joint on the
# arm by a long way -- 84 mm of descent is seven seconds of stepping.
SHOULDER_SPEED = 0.012          # m/s

# Standing-still time after the shoulder moves, as a fraction of how long that
# move really takes at SHOULDER_SPEED.
#
# 1.0 -- the WHOLE travel time -- because joint_limits.yaml now declares the
# shoulder at 0.4 m/s (2026-08-19, all four joints set to 0.4/0.2). The slide
# still physically does 0.012 m/s, so MoveIt plans a 132 mm drop as a third of a
# second, hands the trajectory over, and calls it executed while the stepper has
# eleven seconds of travel left. Practically none of the real motion happens
# inside wait_until_executed() any more, so this node has to wait it out itself.
#
# Put joint_limits.yaml back to the derived hardware speeds and 0.25 is enough
# again: MoveIt would then be doing the waiting and this would only be covering
# the acceleration ramps.
SHOULDER_CATCHUP = 1.0

# The height the shoulder is wound up to before the arm swings anywhere. Every
# move across the cell happens at this height -- see move_up_then_over().
#
# The shoulder's travel is -0.15 to 0.02 (arm.xacro). The rule for this cell is
# that it must be at least HALFWAY up before any rotation turns, so 0.0 -- the
# top of the useful travel -- clears it easily. SHOULDER_HALFWAY is only used to
# check a travel_shoulder someone passes in.
SHOULDER_LOWEST = -0.15
SHOULDER_HIGHEST = 0.02
SHOULDER_HALFWAY = (SHOULDER_LOWEST + SHOULDER_HIGHEST) / 2      # -0.065
TRAVEL_SHOULDER = 0.0

TAUGHT_PICK_APPROACH = [-0.577, TRAVEL_SHOULDER, 1.718, 1.075]
TAUGHT_GRASP_SHOULDER = -0.084

# ---- Where each colour is put down, taught the same way -------------------
#
# Joint order column, shoulder, forearm, wrist, and this is the RELEASE pose:
# the arm as it stands over the bin with the box still held. Nothing describes
# the way in, because there is only one way in -- up, across, down.
#
#     R  read off scara_gui.py as J -0.4030 -0.1320 -1.7320 -0.6320 (2026-08-19)
#     G  read off the same way as    J  1.7320 -0.1240 -1.7850  0.0000 (2026-08-19)
#
#     Y  the mirror image of R, and B the mirror image of G: reflecting a pose
#        across the robot centre flips every rotational joint and leaves the
#        vertical shoulder alone. Exactly the pairing the simulation uses for
#        its bins, so the two cells stay consistent.
#
# All four colours are covered, so the startup warning about an untaught bin
# should never appear. If you move a bin, re-teach its colour the same way: park
# the arm over it holding a box, read the four numbers off the GUI, put them
# here -- and if it is R or G, its mirror follows on its own.
TAUGHT_DROPS = {
    "R": [-0.403, -0.132, -1.732, -0.632],
    "G": [1.732, -0.124, -1.785, 0.0],
}
TAUGHT_DROPS["Y"] = _mirror_pose(TAUGHT_DROPS["R"])   # Yellow = mirror of Red
TAUGHT_DROPS["B"] = _mirror_pose(TAUGHT_DROPS["G"])   # Blue   = mirror of Green


class HardwarePickExecutor(Node):

    def __init__(self):
        super().__init__("hardware_pick_executor")

        # Dynamic typing: ROS 2 arg parsing reads a bare `Y` (also y/yes/no/on/
        # off/true...) as a YAML boolean, so `-p target_color:=Y` arrives as
        # True. Allow it, then coerce back to the colour letter below.
        self.declare_parameter(
            "target_color", "R", ParameterDescriptor(dynamic_typing=True)
        )
        # "taught" by default: the pose above was measured on this cell, and it
        # is what the real belt needs. "ik" asks MoveIt for the simulated belt's
        # pickup point instead -- right only if your belt sits where the
        # simulation puts it.
        self.declare_parameter("pick_source", "taught")   # "taught" or "ik"
        # column, shoulder, forearm, wrist -- only read when pick_source=taught.
        # Its shoulder is the height the arm lines up at; the grasp always winds
        # the shoulder down to grasp_shoulder.
        self.declare_parameter("pick_approach", TAUGHT_PICK_APPROACH)
        self.declare_parameter("approach_height", APPROACH_HEIGHT)
        self.declare_parameter("grasp_shoulder", TAUGHT_GRASP_SHOULDER)
        # The height every sideways move is made at. Raise it if the gripper
        # clips anything on its way across the cell.
        self.declare_parameter("travel_shoulder", TRAVEL_SHOULDER)
        # Stand-still either side of every gripper command. Raise it if the
        # gripper still closes early, or on a box it has not quite reached.
        self.declare_parameter("settle_s", SETTLE_SECONDS)
        # ...and on top of that, time for the shoulder to finish arriving,
        # scaled by how far it just travelled. Raise it if the arm still stops
        # short of the taught height on the longer drops.
        self.declare_parameter("shoulder_catchup", SHOULDER_CATCHUP)

        # MoveIt scales every move by these. joint_limits.yaml already holds the
        # REAL stepper speeds, so 1.0 means "as fast as the motors can manage",
        # not "dangerously fast". The file's own default of 0.1 would be ten
        # times slower again, which on this arm is a very long wait.
        self.declare_parameter("velocity_scaling", 1.0)
        self.declare_parameter("acceleration_scaling", 1.0)

        # How far the gripper closes, in metres. The simulation uses -0.03; the
        # real firmware maps -0.06 to a fully shut gripper, so a real box may
        # need more than the simulated one. Measure it and set it here.
        self.declare_parameter("grip_closed", scara.CLOSED_GRIPPER_JOINT_POSITIONS[0])

        raw_color = self.get_parameter("target_color").value
        # `Y`/`y`/`yes` came in as boolean True (see above) -> map back to Y.
        if isinstance(raw_color, bool):
            raw_color = "Y" if raw_color else ""
        self.target_color = str(raw_color).strip().upper()
        if self.target_color not in DROP_TARGETS:
            raise ValueError(f"target_color must be one of {sorted(DROP_TARGETS)}")

        self.pick_source = str(self.get_parameter("pick_source").value).lower()
        if self.pick_source not in ("ik", "taught"):
            raise ValueError("pick_source must be 'ik' or 'taught'")

        self.taught_approach = [
            float(v) for v in self.get_parameter("pick_approach").value
        ]
        if len(self.taught_approach) != len(scara.joint_names()):
            raise ValueError(
                f"pick_approach needs {len(scara.joint_names())} numbers "
                f"({', '.join(scara.joint_names())})"
            )
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.grasp_shoulder = float(self.get_parameter("grasp_shoulder").value)
        self.travel_shoulder = float(self.get_parameter("travel_shoulder").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.shoulder_catchup = float(self.get_parameter("shoulder_catchup").value)
        # How far the shoulder has been told to travel but has not been waited
        # out yet. Set by move_to, cleared by wait_for_shoulder.
        self.shoulder_pending = 0.0
        if self.travel_shoulder < SHOULDER_HALFWAY:
            self.get_logger().warning(
                f"travel_shoulder {self.travel_shoulder} is below halfway up the "
                f"shoulder's travel ({SHOULDER_HALFWAY}). The arm is meant to be "
                "at least half raised before anything turns; expect the gripper "
                "to catch on the belt or a bin."
            )
        self.grip_closed = float(self.get_parameter("grip_closed").value)

        self.busy = False
        self.pick_requested = False
        # Where the arm was last told to go. The arm is open loop, so this is
        # the only idea of "here" there is -- and it starts at zero because
        # zero is where the arm has to be parked before the drivers are
        # powered (see the warning at the top of this file).
        self.here = [0.0] * len(scara.joint_names())
        # Worked out on the first pick and kept, so the second box does not wait
        # for the IK solver again.
        self.approach_joints = None

        group = ReentrantCallbackGroup()
        self.arm = MoveIt2(
            node=self,
            joint_names=scara.joint_names(),
            base_link_name=scara.base_link_name(),
            end_effector_name=scara.end_effector_name(),
            group_name=scara.MOVE_GROUP_ARM,
            callback_group=group,
            use_move_group_action=True,
        )
        self.arm.max_velocity = float(self.get_parameter("velocity_scaling").value)
        self.arm.max_acceleration = float(
            self.get_parameter("acceleration_scaling").value
        )

        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=scara.gripper_joint_names(),
            open_gripper_joint_positions=scara.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=[self.grip_closed],
            gripper_group_name=scara.MOVE_GROUP_GRIPPER,
            callback_group=group,
        )

        # Announce the wanted colour. The colour gate watches for it and the
        # state machine only stops the belt for it. Latched, so both of them get
        # it whenever they start -- before or after this node.
        self.target_pub = self.create_publisher(
            String, common.TOPIC_TARGET_COLOR, LATCHED
        )
        self.completed_pub = self.create_publisher(
            Bool, common.TOPIC_PICK_COMPLETED, LATCHED
        )
        self.target_pub.publish(String(data=self.target_color))

        self.create_subscription(
            String, common.TOPIC_EXECUTE_PICK, self.on_execute_pick, 10
        )

        where = (
            f"the joint pose {self.taught_approach}" if self.pick_source == "taught"
            else f"the belt pickup point {common.pickup_grasp_point_base()} (by IK)"
        )
        self.get_logger().info(
            f"Hardware pick node ready. Picking {self.target_color} blind at "
            f"{where}; every other colour rides past."
        )
        if self.target_color in TAUGHT_DROPS:
            self.get_logger().info(
                f"Dropping at the taught {self.target_color} pose "
                f"{[round(v, 3) for v in self.drop_release()]}."
            )
        else:
            self.get_logger().warning(
                f"No taught drop pose for {self.target_color} -- falling back to "
                "the SIMULATION's, which is probably not where your bin is. "
                "Park the arm over the bin, read the four joint values off "
                "scara_gui.py and add them to TAUGHT_DROPS in this file."
            )

    # ----------------------------------------------------------- callbacks --

    def on_execute_pick(self, message):
        """The state machine has stopped the belt and wants this box picked."""
        if self.busy:
            self.get_logger().warning(
                f"Still busy; ignoring the request to pick {message.data}"
            )
            return
        self.pick_requested = True

    # --------------------------------------------------------------- poses --

    def solve_ik(self, position):
        """Joint angles that put the tool at this point, or None."""
        result = self.arm.compute_ik(position=position, quat_xyzw=[0.0, 0.0, 0.0, 1.0])
        if result is None:
            return None
        return [result.position[result.name.index(name)] for name in scara.joint_names()]

    def find_approach(self):
        """The pose to line up in, above the box. Worked out once and kept."""
        if self.approach_joints is not None:
            return self.approach_joints

        if self.pick_source == "taught":
            self.approach_joints = list(self.taught_approach)
            return self.approach_joints

        x, y, z = common.pickup_grasp_point_base()
        above = [x, y, z + self.approach_height]
        joints = self.solve_ik(above)
        if joints is None:
            self.get_logger().error(
                f"MoveIt cannot reach {tuple(round(v, 3) for v in above)}. Either "
                "the belt is not where common.py says, or the arm cannot get "
                "there -- teach the pose instead: -p pick_source:=taught "
                "-p pick_approach:='[column, shoulder, forearm, wrist]'"
            )
            return None

        self.get_logger().info(
            "Pick pose solved once and kept: "
            + ", ".join(f"{n}={v:.3f}" for n, v in zip(scara.joint_names(), joints))
        )
        self.approach_joints = joints
        return joints

    def drop_release(self):
        """Where this colour is let go of -- taught if it has been, else the
        simulation's pose, which the startup warning already complained about."""
        if self.target_color in TAUGHT_DROPS:
            return list(TAUGHT_DROPS[self.target_color])
        return list(DROP_TARGETS[self.target_color]["drop_release"])

    def at_travel_height(self, pose):
        """The same pose with the shoulder wound up to the travel height."""
        lifted = list(pose)
        lifted[GRASP_JOINT_INDEX] = self.travel_shoulder
        return lifted

    def grasp_from(self, approach):
        """The same pose with the shoulder wound down to the belt.

        Only the vertical slide moves, so nothing can swing sideways into the
        box on the way down. This is what the simulation does too.
        """
        grasp = list(approach)
        grasp[GRASP_JOINT_INDEX] = self.grasp_shoulder
        return grasp

    # -------------------------------------------------------------- motion --

    def move_to(self, joint_positions, what):
        self.get_logger().info(f"-> {what}")
        moved = abs(joint_positions[GRASP_JOINT_INDEX] - self.here[GRASP_JOINT_INDEX])
        self.arm.move_to_configuration(joint_positions, joint_names=scara.joint_names())
        if self.arm.wait_until_executed():
            self.here = list(joint_positions)
            if moved:
                self.shoulder_pending = moved
            return True
        self.get_logger().error(f"Could not move to {what}.")
        return False

    def move_up_then_over(self, target, what):
        """Up, across, down -- never diagonally.

        MoveIt asked for one pose moves all four joints together, so the
        gripper cuts the corner: it travels in a straight-ish line from where
        it is to where it is going, at whatever height that line happens to
        pass through. Loaded, on a bench, that line goes through the belt, the
        next box along, and the near wall of the bin.

        So each leg of the job is three moves instead of one:

            1. wind the shoulder -- the prismatic joint -- up to the travel
               height, with the rotations held still
            2. swing the rotations across, at that height, shoulder held still
            3. come straight back down onto the target

        Step 2 is ONE MoveIt goal, so the rotations all turn together rather
        than one after another -- the arm swings across as a unit, it just does
        it high up. Steps that would not move anything are skipped, so a target
        already at the travel height simply never gets step 3.
        """
        if self.here[GRASP_JOINT_INDEX] != self.travel_shoulder:
            if not self.move_to(self.at_travel_height(self.here), f"up, clear of {what}"):
                return False

        across = self.at_travel_height(target)
        if across != self.here:
            # THE important wait. Turning while the slide is still on its way up
            # drags whatever is in the gripper -- or the gripper itself --
            # sideways through the bin it has just been lowered into.
            self.wait_for_shoulder("before turning anything")
            if not self.move_to(across, f"across to {what}"):
                return False

        if target[GRASP_JOINT_INDEX] == self.travel_shoulder:
            return True
        return self.move_to(target, f"down to {what}")

    def work_the_gripper(self, close):
        """Grip or let go -- but only once the arm has actually stopped there.

        The settle either side is the point of this method. MoveIt says a move
        is executed when it has finished handing the trajectory over; on an open
        loop stepper arm the joints are still arriving. Gripping then is
        gripping early -- on the corner of the box, or on air. So: stand still,
        work the gripper, stand still again while the servo does its travel,
        and only then let the caller move on.
        """
        self.wait_for_shoulder("before working the gripper")
        self.settle("before working the gripper")

        if close:
            self.gripper.close()
        else:
            self.gripper.open()

        if not self.gripper.wait_until_executed():
            self.get_logger().error(
                f"Gripper would not {'close' if close else 'open'}."
            )
            return False

        self.settle("while the gripper travels")
        return True

    def wait_for_shoulder(self, why):
        """Stand still until the slide has really finished its last move.

        MoveIt hands the trajectory over and calls it executed; the stepper is
        still travelling, and at the limits in joint_limits.yaml it is still
        travelling by a lot. Nothing else may start until it has arrived --
        neither a turn nor the gripper -- so every caller that is about to do
        one of those comes through here first.

        The wait is the distance it was told to cover at the speed it really
        manages, so it scales: about seven seconds for the 84 mm grasp, eleven
        for the 132 mm drop.
        """
        if self.shoulder_pending <= 0.0:
            return

        wait = self.shoulder_pending / SHOULDER_SPEED * self.shoulder_catchup
        self.get_logger().info(
            f"   waiting {wait:.2f}s for the shoulder to finish its "
            f"{self.shoulder_pending * 1000:.0f} mm {why}"
        )
        self.shoulder_pending = 0.0
        if wait > 0.0:
            time.sleep(wait)

    def settle(self, why):
        """A fixed pause, for the things that are not the shoulder -- mainly the
        gripper servo, which reports its command sent long before it has moved."""
        if self.settle_s <= 0.0:
            return
        self.get_logger().info(f"   settling {self.settle_s:.2f}s {why}")
        time.sleep(self.settle_s)

    def run_pick(self):
        """One box: down, grab, up, over to its bin, let go, home.

        Whatever happens, /scara/pick_completed goes out at the end -- true if
        the box was dealt with, false if anything went wrong. The state machine
        is waiting on it either way, and if it never arrives the belt stays
        frozen until the watchdog gives up.
        """
        picked = False
        home = DROP_TARGETS[self.target_color]["drop_home"]

        try:
            approach = self.find_approach()
            if approach is None:
                return

            if not self.work_the_gripper(close=False):
                return
            # Every leg goes up, across and down; only the grasp and the lift
            # off the belt are bare vertical moves, and those are vertical by
            # definition.
            if not self.move_up_then_over(approach, "above the box"):
                return
            if not self.move_to(self.grasp_from(approach), "down onto the box"):
                return
            if not self.work_the_gripper(close=True):
                return
            if not self.move_to(approach, "lift the box clear"):
                return
            if not self.move_up_then_over(
                self.drop_release(), f"the {self.target_color} bin"
            ):
                return
            if not self.work_the_gripper(close=False):
                return
            self.move_up_then_over(home, "home")

            picked = True
            self.get_logger().info(f"{self.target_color} pick-and-drop complete.")
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Pick failed: {error}")
        finally:
            self.completed_pub.publish(Bool(data=picked))


def main():
    rclpy.init()
    node = HardwarePickExecutor()
    try:
        # The pick runs here rather than in the callback: it blocks for a long
        # time on this arm, and blocking inside a callback would stop this node
        # hearing anything else while it did.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pick_requested and not node.busy:
                node.busy = True
                node.pick_requested = False
                node.run_pick()
                node.busy = False
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

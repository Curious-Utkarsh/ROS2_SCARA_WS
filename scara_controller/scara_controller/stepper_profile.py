#!/usr/bin/env python3
"""A pretend copy of the real arm's motors, so RViz can move at the true speed.

The robot in RViz has no motors, so it arrives the instant you ask it to. The
real arm has four steppers that can only take so many steps a second, so it
takes its time. This file closes that gap: it works out how fast each joint can
REALLY move, and gives you a small model that crawls toward a target at exactly
that speed.

It is not feedback. Nothing here ever asks the robot where it is -- there are no
encoders to ask. It is a copy of the arithmetic the Arduino is doing, run on
this side of the cable so the picture can keep up with the metal.

    NUMBERS BELOW MUST MATCH scara_moveit_control.ino
    Change a gear ratio, a microstepping jumper or a top speed in the sketch and
    you must change it here too, or RViz will start lying again.
"""

import math

# ---------------------------------------------------------------------------
# Straight out of firmware/scara_moveit_control/scara_moveit_control.ino
# ---------------------------------------------------------------------------

STEPS_PER_MOTOR_TURN = 800.0        # 200 full steps x 4 microsteps (jumper on M1)
ONE_TURN_IN_RADIANS = 2.0 * math.pi

COLUMN_GEARING = 19.0               # motor turns per joint turn
FOREARM_GEARING = 16.0
WRIST_GEARING = 1.0                 # NOT MEASURED YET -- same caveat as the sketch

SHOULDER_MM_PER_TURN = 0.008        # leadscrew: metres lifted per motor turn

# Steps per second each driver is allowed. They add up to 3500, near the ~4000
# an Uno can sustain across all four axes.
COLUMN_TOP_SPEED = 1000.0
SHOULDER_TOP_SPEED = 1200.0
FOREARM_TOP_SPEED = 800.0
WRIST_TOP_SPEED = 500.0

# AccelStepper is told to reach top speed in half a second.
SPEED_UP_RATE = 2.0

# The real arm finishes a second or two after RViz does, and this is the whole
# fix: drive the picture a little slower than the motors could manage, and the
# metal keeps up.
#
# Why it is behind at all: the Arduino plans to come to a FULL STOP on every
# target it is sent, and we send it a new one fifty times a second. So it spends
# the whole move braking for a target just ahead of it, and never quite catches
# the picture until the picture stops.
#
# 0.85 means "use 85% of the speed the motors are capable of". Turn it DOWN if
# RViz still gets there first; turn it UP toward 1.0 if RViz now trails behind
# the real arm. It is one number and nothing else has to change with it.
CATCH_UP = 0.85

# The gripper is a servo, not a stepper. It cannot say when it has finished, so
# the sketch simply waits GRIPPER_WAIT_TIME for it. Model it as a joint that
# crosses its whole travel in that time.
GRIPPER_TRAVEL_M = 0.06
GRIPPER_WAIT_TIME_S = 0.400


def _steps_per_unit(gearing):
    """How many microsteps make one radian of a geared rotary joint."""
    return STEPS_PER_MOTOR_TURN * gearing / ONE_TURN_IN_RADIANS


# ---------------------------------------------------------------------------
# What the sums come out to
#
#   joint     steps per unit    cap         top speed      speed-up
#   column     2419 /rad        1000 st/s   0.413 rad/s    0.827 rad/s^2
#   shoulder 100000 /m          1200 st/s   0.012 m/s      0.024 m/s^2
#   forearm    2037 /rad         800 st/s   0.393 rad/s    0.785 rad/s^2
#   wrist       127 /rad         500 st/s   3.927 rad/s    7.854 rad/s^2
#
# The shoulder is the one that shows why this file exists: its full 0.15 m of
# travel really takes about 12.5 seconds.
#
# Those are the MOTOR numbers. What the model actually uses is CATCH_UP times
# each of them -- so the column runs at 0.351 rad/s, not 0.413. The node prints
# the numbers it ended up with when it starts, so you never have to work them
# out by hand.
# ---------------------------------------------------------------------------

def _rotary(top_speed_steps, gearing):
    velocity = CATCH_UP * top_speed_steps / _steps_per_unit(gearing)
    return velocity, velocity * SPEED_UP_RATE


def _leadscrew(top_speed_steps, metres_per_turn):
    velocity = CATCH_UP * top_speed_steps / (STEPS_PER_MOTOR_TURN / metres_per_turn)
    return velocity, velocity * SPEED_UP_RATE


# joint name -> (top speed, how fast it may build up to that speed)
LIMITS = {
    "column_joint": _rotary(COLUMN_TOP_SPEED, COLUMN_GEARING),
    "shoulder_joint": _leadscrew(SHOULDER_TOP_SPEED, SHOULDER_MM_PER_TURN),
    "forearm_joint": _rotary(FOREARM_TOP_SPEED, FOREARM_GEARING),
    "wrist_joint": _rotary(WRIST_TOP_SPEED, WRIST_GEARING),

    # The servo has no ramp worth speaking of: give it a big speed-up rate so it
    # spends its whole time at top speed, which is what "moves for 0.4 s and
    # stops" actually looks like.
    #
    # No CATCH_UP here. The gripper is not a stepper chasing a target, it is a
    # servo the sketch simply waits GRIPPER_WAIT_TIME for, so it has nothing to
    # fall behind by.
    "left_finger_joint": (GRIPPER_TRAVEL_M / GRIPPER_WAIT_TIME_S,
                          GRIPPER_TRAVEL_M / GRIPPER_WAIT_TIME_S * 20.0),
}


class TrapezoidalAxis:
    """One joint, moving toward a target the way a stepper motor really does.

    Speed builds up, holds at the top, then comes back down in time to stop on
    the target. This is what AccelStepper.run() does on the Arduino, and doing
    the same sum here is what makes the two agree.

    Give it a NEW target at any time and it simply re-aims, without stopping
    first -- which is why dragging a slider looks like smooth travel rather than
    a series of little jumps.
    """

    # Closer than this to the target and we call it arrived, rather than
    # chasing a last few millionths forever.
    ARRIVED = 1e-6

    def __init__(self, max_velocity, max_acceleration, position=0.0):
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.position = float(position)
        self.velocity = 0.0

    def at(self, target):
        """Has this axis finished travelling to `target`?"""
        return abs(target - self.position) <= self.ARRIVED and self.velocity == 0.0

    def advance(self, target, dt, speed_scale=1.0):
        """Move for `dt` seconds toward `target`. Returns the new position."""
        if dt <= 0.0:
            return self.position

        top_speed = self.max_velocity * speed_scale
        speed_up = self.max_acceleration * speed_scale

        remaining = target - self.position
        if abs(remaining) <= self.ARRIVED:
            self.position = target
            self.velocity = 0.0
            return self.position

        direction = 1.0 if remaining > 0.0 else -1.0

        # How much room would we need to come to a complete stop from here?
        # v^2 / 2a is the standard braking distance. Once what is left is less
        # than that, it is time to slow down instead of speed up.
        braking_distance = (self.velocity * self.velocity) / (2.0 * speed_up)

        going_the_right_way = self.velocity * direction >= 0.0

        if not going_the_right_way or abs(remaining) > braking_distance:
            # Push toward the target. If we are already travelling that way this
            # speeds us up; if the slider was just dragged back past us it slows
            # us down, stops us and sends us the other way -- which is the same
            # sum, so it is written once.
            self.velocity += direction * speed_up * dt
        else:
            # Close enough that it is time to brake, or we sail past.
            self.velocity -= direction * speed_up * dt

        self.velocity = max(-top_speed, min(top_speed, self.velocity))

        step = self.velocity * dt

        # Never sail past the target: land on it and stop dead. Without this the
        # axis would jitter around the target by a fraction of a step.
        if abs(step) >= abs(remaining):
            self.position = target
            self.velocity = 0.0
        else:
            self.position += step

        return self.position


def axes_for(joint_names, start_positions=None):
    """Build one TrapezoidalAxis per joint, in the order given.

    A joint with no entry in LIMITS gets a fast axis rather than an error, so an
    extra joint in the URDF can never wedge the whole model.
    """
    starts = start_positions or {}
    axes = []
    for name in joint_names:
        max_velocity, max_acceleration = LIMITS.get(name, (10.0, 100.0))
        axes.append(TrapezoidalAxis(max_velocity, max_acceleration,
                                    starts.get(name, 0.0)))
    return axes


def travel_time(joint_name, distance):
    """Roughly how long the real joint takes to move `distance`. For logging."""
    max_velocity, max_acceleration = LIMITS.get(joint_name, (10.0, 100.0))
    distance = abs(distance)
    ramp_distance = max_velocity * max_velocity / max_acceleration
    if distance <= ramp_distance:
        return 2.0 * math.sqrt(distance / max_acceleration)   # never reaches top speed
    return max_velocity / max_acceleration + distance / max_velocity

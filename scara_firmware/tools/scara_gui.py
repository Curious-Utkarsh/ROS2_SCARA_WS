#!/usr/bin/env python3
"""A small control panel for the SCARA robot.

Drag a slider, the real arm moves. Use it to check the arm works before any of
ROS is running.

    ros2 run scara_firmware scara_gui.py
    # or just:  python3 scara_gui.py

This talks STRAIGHT to the Arduino over the USB cable. ROS is not involved at
all, so you can use it before MoveIt is running, and nothing here can upset the
simulation. Close it before you launch real_bringup.launch.py, because only one
program can hold the serial port at a time.

It speaks the same language as the firmware:
    "J column shoulder forearm wrist gripper"   in radians and metres
    "H" zero here   "F" free the motors   "L" lock the motors
    "C 1" run the conveyor belt   "C 0" stop it
"""

import time
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# The joint ranges must match the robot model (URDF) and the limits in
# scara_moveit_control.ino. Change a limit there and change it here too.
# ---------------------------------------------------------------------------

# name, lowest, highest, unit, "nice" unit, how many nice units in one SI unit
JOINTS = [
    ("column",   -2.00,  2.00, "rad", "deg", 57.2958),
    ("shoulder", -0.15,  0.02, "m",   "mm",  1000.0),   # leadscrew
    ("forearm",  -2.00,  2.00, "rad", "deg", 57.2958),
    ("wrist",   -4.712, 4.712, "rad", "deg", 57.2958),
]

GRIPPER_OPEN = 0.00                 # metres
GRIPPER_CLOSED = -0.06

# How long to give the Arduino to answer a belt command before we say out loud
# that it did not. It answers in about a millisecond when it is listening, so
# anything past this means it is not listening at all.
CONVEYOR_ANSWER_TIME = 1.0          # seconds


class ScaraGui:
    def __init__(self, root):
        self.root = root
        self.arduino = None
        self.sliders = {}
        self.readouts = {}
        self.gripper_closed = False
        self.conveyor_running = False
        self.incoming = ""              # half a line of text from the Arduino
        self.conveyor_asked_at = None   # when we last sent a belt command

        root.title("SCARA control panel")

        self.build_connection_row()
        self.build_joint_sliders()
        self.build_gripper_toggle()
        self.build_buttons()
        self.build_conveyor_toggle()
        self.build_status_bar()

        self.poll_serial()

    # -- the parts of the window -------------------------------------------

    def build_connection_row(self):
        frame = ttk.LabelFrame(self.root, text="1. Connect to the Arduino")
        frame.pack(fill="x", padx=10, pady=6)

        ttk.Label(frame, text="Port:").pack(side="left", padx=5, pady=6)

        self.port_box = ttk.Combobox(frame, width=22, values=self.find_ports())
        if self.port_box["values"]:
            self.port_box.current(0)
        self.port_box.pack(side="left", padx=5)

        ttk.Button(frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=3)
        self.connect_button = ttk.Button(frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=5)

    def build_joint_sliders(self):
        frame = ttk.LabelFrame(self.root, text="2. Move the joints")
        frame.pack(fill="x", padx=10, pady=6)

        for row, (name, lowest, highest, unit, nice_unit, _) in enumerate(JOINTS):
            ttk.Label(frame, text=name, width=9).grid(row=row, column=0, padx=6, pady=5, sticky="w")

            slider = tk.Scale(
                frame, from_=lowest, to=highest, resolution=0.001,
                orient="horizontal", length=330, showvalue=False,
                command=lambda value, n=name: self.slider_moved(n),
            )
            slider.set(0.0)
            slider.grid(row=row, column=1, padx=5)
            self.sliders[name] = slider

            readout = ttk.Label(frame, text="", width=22)
            readout.grid(row=row, column=2, padx=6, sticky="w")
            self.readouts[name] = readout

            ttk.Button(frame, text="0", width=3,
                       command=lambda n=name: self.set_joint(n, 0.0)).grid(row=row, column=3, padx=3)

            self.update_readout(name)

    def build_gripper_toggle(self):
        frame = ttk.LabelFrame(self.root, text="3. Gripper")
        frame.pack(fill="x", padx=10, pady=6)

        self.gripper_button = tk.Button(
            frame, text="OPEN  (click to close)", width=26, height=2,
            bg="#d6f5d6", command=self.toggle_gripper,
        )
        self.gripper_button.pack(side="left", padx=8, pady=6)

        ttk.Label(frame, text="left_finger_joint is 0.00 m open, -0.06 m closed").pack(
            side="left", padx=10)

    def build_buttons(self):
        frame = ttk.LabelFrame(self.root, text="4. Motors")
        frame.pack(fill="x", padx=10, pady=6)

        ttk.Button(frame, text="Zero here  (H)", command=lambda: self.send_command("H")
                   ).pack(side="left", padx=6, pady=6)
        ttk.Button(frame, text="Free motors  (F)", command=lambda: self.send_command("F")
                   ).pack(side="left", padx=6)
        ttk.Button(frame, text="Lock motors  (L)", command=lambda: self.send_command("L")
                   ).pack(side="left", padx=6)
        ttk.Button(frame, text="All joints to 0", command=self.all_to_zero
                   ).pack(side="left", padx=16)

    def build_conveyor_toggle(self):
        """Switch the belt relay by hand, with no ROS running.

        Useful on its own: plug the relay into the Y+ header, press this, and
        you know whether the wiring works before any of the sorting pipeline is
        involved. If the button says one thing and the belt does the other, flip
        CONVEYOR_ACTIVE_LOW in scara_moveit_control.ino.
        """
        frame = ttk.LabelFrame(self.root, text="5. Conveyor belt")
        frame.pack(fill="x", padx=10, pady=6)

        self.conveyor_button = tk.Button(
            frame, text="STOPPED  (click to run)", width=26, height=2,
            bg="#f5d6d6", command=self.toggle_conveyor,
        )
        self.conveyor_button.pack(side="left", padx=8, pady=6)

        tk.Button(
            frame, text="STOP belt now", width=14, height=2,
            bg="#ffd9b3", command=lambda: self.set_conveyor(False),
        ).pack(side="left", padx=4)

        right = ttk.Frame(frame)
        right.pack(side="left", padx=10)
        ttk.Label(right, text="relay on the Y+ endstop header (Arduino pin 10)").pack(
            anchor="w")
        self.conveyor_reply = ttk.Label(right, text="", foreground="#004488")
        self.conveyor_reply.pack(anchor="w")

    def toggle_conveyor(self):
        self.set_conveyor(not self.conveyor_running)

    def set_conveyor(self, run):
        """Ask for one belt state, and say we are still waiting to hear back.

        The toggle asks for "the opposite of what I last asked for", which is
        only right while the GUI and the Arduino agree. If the board was reset,
        or a message was chewed up on the way, they stop agreeing and the
        toggle then asks for the wrong one - you press stop and it starts.
        "STOP belt now" comes through here too and always asks for OFF, so
        there is one button that is right no matter what either side believes.
        """
        self.conveyor_running = run

        if run:
            self.conveyor_button.config(text="RUNNING  (click to stop)", bg="#d6f5d6")
        else:
            self.conveyor_button.config(text="STOPPED  (click to run)", bg="#f5d6d6")

        self.conveyor_reply.config(text="waiting for the Arduino...", foreground="#886600")
        self.conveyor_asked_at = time.monotonic()
        self.send_command("C 1" if run else "C 0")

    def build_status_bar(self):
        self.status = ttk.Label(self.root, text="Not connected", relief="sunken", anchor="w")
        self.status.pack(fill="x", side="bottom", ipady=3)

        self.board_reply = ttk.Label(
            self.root, text="Arduino says: -", relief="sunken", anchor="w",
            foreground="#333333")
        self.board_reply.pack(fill="x", side="bottom", ipady=3)

    # -- serial port -------------------------------------------------------

    def find_ports(self):
        """List the serial ports that look like a plugged-in board."""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        real = [p for p in ports if "USB" in p or "ACM" in p]
        return real if real else ports

    def refresh_ports(self):
        self.port_box["values"] = self.find_ports()
        if self.port_box["values"]:
            self.port_box.current(0)

    def toggle_connection(self):
        if self.arduino is not None:
            self.disconnect()
            return

        port = self.port_box.get()
        if not port:
            self.say("Pick a port first")
            return

        try:
            self.arduino = serial.Serial(port, 115200, timeout=0)
        except (serial.SerialException, OSError) as error:
            self.say(f"Could not open {port}: {error}")
            return

        self.connect_button.config(text="Disconnect")
        self.say(f"Connected to {port} - waiting for the Arduino to start up")
        # The board reboots when we open the port, so give it a moment.
        self.root.after(2500, lambda: self.say("Ready. Park the arm at zero, then press 'Zero here'."))

    def disconnect(self):
        # Stop the belt on the way out. Closing the window should never leave a
        # mains motor running with nothing left holding the port to stop it.
        try:
            self.arduino.write(b"C 0\n")
            self.arduino.flush()
        except (serial.SerialException, OSError, AttributeError):
            pass

        try:
            self.arduino.close()
        except (serial.SerialException, OSError):
            pass
        self.arduino = None
        self.incoming = ""
        self.conveyor_asked_at = None
        self.connect_button.config(text="Connect")
        self.say("Disconnected")

    def send_command(self, letter):
        """Send a one letter command like H, F or L."""
        self.write_line(letter)

    def send_joints(self):
        """Send where every joint should go, as one J message."""
        values = [self.sliders[name].get() for name, *_ in JOINTS]
        gripper = GRIPPER_CLOSED if self.gripper_closed else GRIPPER_OPEN
        values.append(gripper)
        self.write_line("J " + " ".join(f"{v:.4f}" for v in values))

    def write_line(self, text):
        if self.arduino is None:
            self.say("Not connected - press Connect first")
            return
        try:
            self.arduino.write((text + "\n").encode("ascii"))
            self.say(f"sent:  {text}")
        except (serial.SerialException, OSError) as error:
            self.say(f"Lost the connection: {error}")
            self.disconnect()

    # -- listening to the Arduino ------------------------------------------

    def poll_serial(self):
        """Read whatever the Arduino has said and show the last line of it.

        This window used to write and never read, which is fine right up until
        something does not work: with nothing coming back there is no way to
        tell "the Arduino never got the message" from "the Arduino did as it
        was told and the relay is wired backwards". Those two need opposite
        fixes, so the answer is now on screen.
        """
        if self.arduino is not None:
            try:
                waiting = self.arduino.in_waiting
                if waiting:
                    self.incoming += self.arduino.read(waiting).decode("ascii", "replace")
            except (serial.SerialException, OSError) as error:
                self.say(f"Lost the connection: {error}")
                self.disconnect()

            while "\n" in self.incoming:
                line, self.incoming = self.incoming.split("\n", 1)
                self.handle_reply(line.strip())

        self.check_conveyor_answer()
        self.root.after(50, self.poll_serial)

    def handle_reply(self, line):
        if not line:
            return

        self.board_reply.config(text=f"Arduino says: {line}")

        if line.startswith("# conveyor"):
            self.conveyor_asked_at = None
            self.conveyor_reply.config(text=line, foreground="#006600")

    def check_conveyor_answer(self):
        """Say so if a belt command went unanswered.

        The Arduino replies to every "C" the moment it reads one. Silence means
        it is not reading, and the usual reason is that it locked up or reset
        when the relay closed - a mains motor next to an Arduino throws a lot
        of noise about. A belt that starts and then will not stop is that
        story: the pin was pulled low, then nothing was left running to pull it
        back up. Press the board's reset button; if that stops the belt, fit a
        snubber across the motor and keep the relay wiring away from the USB
        lead.
        """
        if self.conveyor_asked_at is None:
            return
        if time.monotonic() - self.conveyor_asked_at < CONVEYOR_ANSWER_TIME:
            return

        self.conveyor_asked_at = None
        self.conveyor_reply.config(
            text="no answer - the Arduino is not reading the port", foreground="#aa0000")

    # -- reacting to the controls ------------------------------------------

    def slider_moved(self, name):
        self.update_readout(name)
        self.send_joints()

    def set_joint(self, name, value):
        self.sliders[name].set(value)

    def all_to_zero(self):
        for name, *_ in JOINTS:
            self.sliders[name].set(0.0)

    def toggle_gripper(self):
        self.gripper_closed = not self.gripper_closed
        if self.gripper_closed:
            self.gripper_button.config(text="CLOSED  (click to open)", bg="#f5d6d6")
        else:
            self.gripper_button.config(text="OPEN  (click to close)", bg="#d6f5d6")
        self.send_joints()

    def update_readout(self, name):
        for joint_name, _, _, unit, nice_unit, per_si in JOINTS:
            if joint_name != name:
                continue
            value = self.sliders[name].get()
            self.readouts[name].config(
                text=f"{value:+.3f} {unit}   ({value * per_si:+.1f} {nice_unit})")

    def say(self, message):
        self.status.config(text=message)


def main():
    root = tk.Tk()
    ScaraGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

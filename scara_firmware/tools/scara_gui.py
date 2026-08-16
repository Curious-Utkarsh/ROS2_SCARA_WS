#!/usr/bin/env python3
"""A small control panel for the SCARA robot.

Drag a slider, the real arm moves. Use it to check the arm works, and to work
out the gear ratios for scara_moveit_control.ino.

    ros2 run scara_firmware scara_gui.py
    # or just:  python3 scara_gui.py

This talks STRAIGHT to the Arduino over the USB cable. ROS is not involved at
all, so you can use it before MoveIt is running, and nothing here can upset the
simulation. Close it before you launch real_bringup.launch.py, because only one
program can hold the serial port at a time.

It speaks the same language as the firmware:
    "J column shoulder forearm wrist gripper"   in radians and metres
    "H" zero here   "F" free the motors   "L" lock the motors
"""

import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# These must match scara_moveit_control.ino and the robot model (URDF).
# If you change a gear ratio in the sketch, change it here too.
# ---------------------------------------------------------------------------

STEPS_PER_MOTOR_TURN = 800.0        # 200 steps x 4 microsteps (jumper on M1)

# name, lowest, highest, unit, "nice" unit, how many nice units in one SI unit,
# and the gearing the sketch is currently using
JOINTS = [
    ("column",   -2.00,  2.00, "rad", "deg", 57.2958, 19.0),
    ("shoulder", -0.15,  0.02, "m",   "mm",  1000.0,  None),   # leadscrew
    ("forearm",  -2.00,  2.00, "rad", "deg", 57.2958, 16.0),
    ("wrist",    -2.00,  2.00, "rad", "deg", 57.2958, 1.0),
]

SHOULDER_LEAD_M = 0.008             # leadscrew travel per motor turn

GRIPPER_OPEN = 0.00                 # metres
GRIPPER_CLOSED = -0.06


class ScaraGui:
    def __init__(self, root):
        self.root = root
        self.arduino = None
        self.sliders = {}
        self.readouts = {}
        self.gripper_closed = False

        root.title("SCARA control panel")

        self.build_connection_row()
        self.build_joint_sliders()
        self.build_gripper_toggle()
        self.build_buttons()
        self.build_ratio_helper()
        self.build_status_bar()

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

        for row, (name, lowest, highest, unit, nice_unit, _, _) in enumerate(JOINTS):
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

    def build_ratio_helper(self):
        frame = ttk.LabelFrame(self.root, text="5. Work out a gear ratio")
        frame.pack(fill="x", padx=10, pady=6)

        explain = (
            "Move one joint with the slider, measure how far it REALLY went with a\n"
            "protractor or ruler, then type both numbers in. Put the answer into the\n"
            "matching *_GEARING in scara_moveit_control.ino and flash it again."
        )
        ttk.Label(frame, text=explain, justify="left").grid(
            row=0, column=0, columnspan=6, padx=8, pady=4, sticky="w")

        ttk.Label(frame, text="Joint:").grid(row=1, column=0, padx=6, sticky="e")
        self.ratio_joint = ttk.Combobox(
            frame, width=10, values=[j[0] for j in JOINTS], state="readonly")
        self.ratio_joint.current(0)
        self.ratio_joint.grid(row=1, column=1, padx=4)

        ttk.Label(frame, text="asked for:").grid(row=1, column=2, padx=6, sticky="e")
        self.asked_entry = ttk.Entry(frame, width=9)
        self.asked_entry.grid(row=1, column=3)

        ttk.Label(frame, text="really moved:").grid(row=1, column=4, padx=6, sticky="e")
        self.actual_entry = ttk.Entry(frame, width=9)
        self.actual_entry.grid(row=1, column=5)

        ttk.Button(frame, text="Work it out", command=self.calculate_ratio).grid(
            row=2, column=0, columnspan=2, padx=6, pady=6)
        self.ratio_result = ttk.Label(frame, text="", foreground="#004488")
        self.ratio_result.grid(row=2, column=2, columnspan=4, sticky="w")

    def build_status_bar(self):
        self.status = ttk.Label(self.root, text="Not connected", relief="sunken", anchor="w")
        self.status.pack(fill="x", side="bottom", ipady=3)

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
        try:
            self.arduino.close()
        except (serial.SerialException, OSError):
            pass
        self.arduino = None
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
        for joint_name, _, _, unit, nice_unit, per_si, _ in JOINTS:
            if joint_name != name:
                continue
            value = self.sliders[name].get()
            self.readouts[name].config(
                text=f"{value:+.3f} {unit}   ({value * per_si:+.1f} {nice_unit})")

    # -- the gear ratio sum -------------------------------------------------

    def calculate_ratio(self):
        """new ratio = old ratio x (what we asked for / what really happened)."""
        name = self.ratio_joint.get()

        try:
            asked = float(self.asked_entry.get())
            actual = float(self.actual_entry.get())
        except ValueError:
            self.ratio_result.config(text="Please type two numbers.")
            return

        if actual == 0:
            self.ratio_result.config(text="'really moved' cannot be zero.")
            return

        if name == "shoulder":
            # The shoulder has a leadscrew, so the number to correct is the lead.
            new_lead = SHOULDER_LEAD_M * (actual / asked)
            self.ratio_result.config(
                text=f"SHOULDER_LEAD_M should be about {new_lead:.5f}  "
                     f"(it is {SHOULDER_LEAD_M})")
            return

        for joint_name, _, _, _, _, _, gearing in JOINTS:
            if joint_name == name:
                new_gearing = gearing * (asked / actual)
                steps = STEPS_PER_MOTOR_TURN * new_gearing / 6.283185
                self.ratio_result.config(
                    text=f"{name.upper()}_GEARING should be about {new_gearing:.2f}  "
                         f"(it is {gearing})   ->  {steps:.0f} steps per radian")
                return

    def say(self, message):
        self.status.config(text=message)


def main():
    root = tk.Tk()
    ScaraGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

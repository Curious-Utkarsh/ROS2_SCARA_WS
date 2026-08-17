// ===========================================================================
// READ THIS FIRST
//
// This is the piece that sits between ROS and the Arduino.  It has to be C++
// because ros2_control loads it as a plugin, but only four functions actually
// matter, and each one is short:
//
//   on_init()      Runs once, when ROS reads the robot description.
//                  Reads the serial port name out of the URDF.
//
//   on_activate()  Runs when the controllers start.  Opens the serial port and
//                  waits for the Arduino to finish rebooting.
//
//   read()         Runs ~50 times a second.  Would normally ask the robot where
//                  it is -- but this robot has no sensors, so it just copies
//                  the last command into the "measured" position.  That single
//                  line is the whole open-loop idea.
//
//   write()        Runs ~50 times a second.  Sends "J ..." to the Arduino, but
//                  only when the target changed AND the Arduino said it is
//                  finished with the previous one.
//
// Everything else in this file is bookkeeping that ros2_control requires.
// The Arduino side of the conversation is in
// firmware/scara_moveit_control/scara_moveit_control.ino
// ===========================================================================

#include "scara_firmware/scara_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace scara_firmware
{
namespace
{

// Joint order on the serial wire.  The Arduino sketch unpacks "J ..." in exactly
// this order, so these two lists must never drift apart.
//
// right_finger_joint is deliberately absent: in the URDF it is a <mimic> of
// left_finger_joint, so the real gripper has a single actuator and
// robot_state_publisher draws the second finger for us.
const std::vector<std::string> WIRE_JOINTS = {
  "column_joint",       // rad
  "shoulder_joint",     // m    (prismatic vertical slide)
  "forearm_joint",      // rad
  "wrist_joint",        // rad
  "left_finger_joint",  // m    (prismatic gripper)
};

// If the Arduino never answers with 'D', give up waiting after this long and
// carry on.  Without it a missing/dead sketch would freeze the arm forever.
constexpr double BUSY_TIMEOUT_S = 5.0;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("ScaraHardwareInterface");
}

}  // namespace

ScaraHardwareInterface::~ScaraHardwareInterface()
{
  closePort();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CallbackReturn ScaraHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  // Lets the base class parse the <ros2_control> block into info_.
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // <param name="port">/dev/ttyUSB0</param> in ros2control.xacro
  const auto port_param = info_.hardware_parameters.find("port");
  if (port_param == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(logger(), "No 'port' parameter in the <hardware> tag of the URDF");
    return CallbackReturn::ERROR;
  }
  port_ = port_param->second;

  // Optional: <param name="wait_for_done">true</param>
  const auto wait_param = info_.hardware_parameters.find("wait_for_done");
  if (wait_param != info_.hardware_parameters.end()) {
    wait_for_done_ = (wait_param->second == "true" || wait_param->second == "True");
  }

  const size_t num_joints = info_.joints.size();
  position_commands_.assign(num_joints, 0.0);
  prev_position_commands_.assign(num_joints, 0.0);
  position_states_.assign(num_joints, 0.0);
  velocity_states_.assign(num_joints, 0.0);

  // Map wire order -> URDF order, once.
  wire_index_.clear();
  for (const auto & wanted : WIRE_JOINTS) {
    bool found = false;
    for (size_t i = 0; i < num_joints; ++i) {
      if (info_.joints[i].name == wanted) {
        wire_index_.push_back(i);
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_FATAL(logger(), "The URDF has no joint called '%s'", wanted.c_str());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    logger(), "Initialised for %zu joints, %zu of them driven, on port %s",
    num_joints, wire_index_.size(), port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScaraHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger(), "Opening %s ...", port_.c_str());
  try {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "Cannot open %s: %s", port_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  // Throw away anything stale sitting in the buffers, THEN wait.  Opening the
  // port toggles DTR, which reboots an Arduino Uno: anything we send during the
  // next couple of seconds is swallowed by the bootloader.
  //
  // The order matters.  Flushing after the wait would discard the banner and
  // the 'R' the sketch prints as it comes up, which are the only evidence that
  // the right firmware is running.
  arduino_.FlushIOBuffers();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // OPEN LOOP, NO ENCODERS: the firmware assumes the arm is parked at zero when
  // it powers up, so ROS has to start from the same belief.  Park the arm before
  // switching the drivers on -- see docs/real_hardware.md.
  std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
  std::fill(prev_position_commands_.begin(), prev_position_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  rx_buffer_.clear();
  busy_ = false;
  have_sent_ = false;

  // The sketch boots with the belt stopped; say so out loud anyway, so ROS and
  // the Arduino cannot disagree about it after a reconnect.  Then start
  // listening -- a latched /conveyor/run will arrive straight away and override
  // this if the belt is supposed to be running.
  sendConveyor(false);
  conveyor_.start();

  RCLCPP_INFO(logger(), "Hardware active -- assuming the arm is parked at zero");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScaraHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Order matters: stop listening first, then stop the belt, then hang up.
  // Leaving a conveyor running after the controllers have gone away would mean
  // nothing was left that could switch it off.
  conveyor_.stop();
  sendConveyor(false);
  closePort();
  RCLCPP_INFO(logger(), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interfaces offered to the controllers
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface> ScaraHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  // Export exactly what the URDF declares, nothing more.  This is what lets the
  // mimic joint (state only, no command) coexist with the five driven joints.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & declared : info_.joints[i].state_interfaces) {
      if (declared.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
      } else if (declared.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ScaraHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  // Position only.  The mimic joint declares no command interface, so it is
  // simply skipped here.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & declared : info_.joints[i].command_interfaces) {
      if (declared.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
      }
    }
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

hardware_interface::return_type ScaraHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // ===== The entire open-loop model, in one line =====
  // There are no encoders, so we believe the arm is wherever we last told it to
  // go.  This is what makes /joint_states -- and therefore RViz and MoveIt's
  // start state -- follow the commands.
  position_states_ = position_commands_;

  // Velocity is reported as zero: we have no way to measure it, and the
  // trajectory controllers run with open_loop_control:true so they never use it.
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  // Pick up any status bytes the Arduino has already sent.  Guarding on
  // GetNumberOfBytesAvailable() keeps this non-blocking -- we must never wait on
  // the wire inside the control loop.
  try {
    while (arduino_.IsOpen() && arduino_.GetNumberOfBytesAvailable() > 0) {
      char c = 0;
      arduino_.ReadByte(c, 1);
      if (c == '\n' || c == '\r') {
        if (!rx_buffer_.empty()) {
          handleStatusLine(rx_buffer_);
          rx_buffer_.clear();
        }
      } else if (rx_buffer_.size() < 128) {
        rx_buffer_.push_back(c);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_ONCE(logger(), "Serial read problem (will keep trying): %s", e.what());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ScaraHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // --- the conveyor belt, first -------------------------------------------
  // Before anything else, and before every early return below.  The belt must
  // be able to stop while the arm is standing still, and it must not be made to
  // wait behind the arm's DONE handshake -- "stop the belt" is the one message
  // here that is ever urgent.
  bool run_belt = false;
  if (conveyor_.takePending(run_belt)) {
    sendConveyor(run_belt);
  }

  // --- the optional handshake gate -----------------------------------------
  // With wait_for_done_ ON we hold back the next target until the Arduino says
  // 'D'.  That is strict point-to-point motion: the motors come to a complete
  // stop at every setpoint.  Because a trajectory arrives as ~100 setpoints a
  // second, that means ~100 stop-starts a second, which is felt as jerking.
  //
  // With it OFF (the default) we send every new setpoint straight away.
  // AccelStepper simply re-aims at the newer target without stopping, so the
  // motion is continuous and smooth.  The 'B'/'D' replies are still read and
  // still tell us whether the firmware is alive, they just do not hold us up.
  //
  // Note we SKIP, we never BLOCK.  write() runs inside controller_manager's
  // update loop; waiting on a serial reply here would stall every controller in
  // the process.
  if (wait_for_done_ && busy_) {
    if ((time - busy_since_).seconds() < BUSY_TIMEOUT_S) {
      return hardware_interface::return_type::OK;
    }
    RCLCPP_WARN(
      logger(), "No 'DONE' from the Arduino after %.0f s -- is the right sketch flashed?",
      BUSY_TIMEOUT_S);
    busy_ = false;
  }

  // Nothing new to say: keep the wire quiet while the arm is idle.
  if (have_sent_ && position_commands_ == prev_position_commands_) {
    return hardware_interface::return_type::OK;
  }

  // "J <column> <shoulder> <forearm> <wrist> <gripper>\n" in SI units (rad / m).
  // Same frame the manual teaching sketch streams back, just in the other
  // direction -- one protocol to learn instead of two.
  char frame[96];
  std::snprintf(
    frame, sizeof(frame), "J %.4f %.4f %.4f %.4f %.4f\n",
    position_commands_[wire_index_[0]], position_commands_[wire_index_[1]],
    position_commands_[wire_index_[2]], position_commands_[wire_index_[3]],
    position_commands_[wire_index_[4]]);

  try {
    arduino_.Write(frame);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Failed to write to %s: %s", port_.c_str(), e.what());
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;
  have_sent_ = true;

  // Assume the move started; the 'B' reply confirms it and 'D' clears it.
  busy_ = true;
  busy_since_ = time;

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void ScaraHardwareInterface::handleStatusLine(const std::string & line)
{
  switch (line[0]) {
    case 'R':  // the sketch has (re)booted and is ready for targets
      busy_ = false;
      RCLCPP_INFO(logger(), "Firmware ready");
      break;
    case 'B':  // target accepted, steppers are moving
      busy_ = true;
      break;
    case 'D':  // every axis reached its target
      busy_ = false;
      break;
    case '#':  // human-readable log line from the sketch
      RCLCPP_INFO(logger(), "firmware:%s", line.c_str() + 1);
      break;
    default:
      break;  // line noise, partial frame after a reset, etc.
  }
}

void ScaraHardwareInterface::sendConveyor(bool run)
{
  if (!arduino_.IsOpen()) {
    return;
  }
  try {
    // "C 1" runs the belt, "C 0" stops it.  Same wire, same line-based
    // protocol as the "J ..." frames -- see readMessage() in the sketch.
    arduino_.Write(run ? "C 1\n" : "C 0\n");
    RCLCPP_INFO(logger(), "Conveyor belt %s", run ? "running" : "stopped");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Failed to switch the conveyor belt: %s", e.what());
  }
}

void ScaraHardwareInterface::closePort()
{
  if (!arduino_.IsOpen()) {
    return;
  }
  try {
    arduino_.Close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Problem closing %s: %s", port_.c_str(), e.what());
  }
}

}  // namespace scara_firmware

// Makes the class loadable by ros2_control through pluginlib.
PLUGINLIB_EXPORT_CLASS(
  scara_firmware::ScaraHardwareInterface, hardware_interface::SystemInterface)

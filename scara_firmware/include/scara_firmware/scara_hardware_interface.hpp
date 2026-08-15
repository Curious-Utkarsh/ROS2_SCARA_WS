// SCARA hardware interface -- the bridge between ros2_control and the Arduino.
//
// This is an OPEN LOOP interface.  The robot has stepper motors and no encoders,
// so nothing ever tells us where the arm really is.  We therefore report the last
// commanded position as the measured position (see read() in the .cpp) and trust
// the steppers to get there.
//
// See docs/real_hardware.md for wiring, homing and the serial protocol.

#ifndef SCARA_FIRMWARE__SCARA_HARDWARE_INTERFACE_HPP_
#define SCARA_FIRMWARE__SCARA_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace scara_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ScaraHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ScaraHardwareInterface() = default;
  ~ScaraHardwareInterface() override;

  // --- lifecycle ------------------------------------------------------------
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // --- what we offer to the controllers -------------------------------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // --- called by controller_manager on every update cycle --------------------
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void closePort();
  void handleStatusLine(const std::string & line);

  LibSerial::SerialPort arduino_;
  std::string port_;

  // One entry per joint in the URDF's <ros2_control> block, in URDF order.
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // wire_index_[k] is where the k-th joint of the serial frame lives in the
  // vectors above.  Resolved once by name in on_init(), so read()/write() stay
  // plain array indexing and can never send joints in the wrong order.
  std::vector<size_t> wire_index_;

  // Incoming status bytes, accumulated until a '\n' completes a line.
  std::string rx_buffer_;

  // true while the Arduino is still driving the steppers to the last target.
  bool busy_ = false;
  rclcpp::Time busy_since_;

  // When true, wait for the Arduino's 'D' before sending the next target.
  // Off by default -- see the note above write() in the .cpp.
  bool wait_for_done_ = false;

  // false until the very first frame goes out, so we always send one even if
  // the commanded pose happens to equal the start pose.
  bool have_sent_ = false;
};

}  // namespace scara_firmware

#endif  // SCARA_FIRMWARE__SCARA_HARDWARE_INTERFACE_HPP_

// The conveyor belt's on/off switch, delivered over the arm's serial cable.
//
// WHY THIS EXISTS
//
// The belt is driven by a relay wired to the same Arduino as the arm, so its
// "stop" has to travel down the same USB cable as the "J ..." frames. That
// cable is held open by ScaraHardwareInterface for as long as the controllers
// are running, and a serial port can only have one owner -- so no separate
// node can ever open it and say so itself.
//
// This class is how the message gets in. It runs a tiny ROS node of its own on
// a background thread, listens on /conveyor/run, and hands what it heard to the
// hardware interface, which writes "C 1" or "C 0" on the wire it already owns.
//
// It NEVER blocks. takePending() is called from inside the controller_manager
// update loop, where waiting on anything would stall every controller in the
// process, so the two sides talk through atomics and nothing else.

#ifndef SCARA_FIRMWARE__CONVEYOR_RELAY_HPP_
#define SCARA_FIRMWARE__CONVEYOR_RELAY_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace scara_firmware
{

class ConveyorRelay
{
public:
  ConveyorRelay() = default;
  ~ConveyorRelay();

  // Start listening. Safe to call twice; the second call does nothing.
  void start();

  // Stop listening and join the background thread.
  void stop();

  // Is there a belt command waiting to go out?  Returns true at most once per
  // change, writing the wanted state into `run`.  Call it every write().
  bool takePending(bool & run);

  // What we last heard, whether or not it has been sent yet.
  bool wanted() const { return wanted_.load(); }

private:
  void onRun(const std_msgs::msg::Bool::SharedPtr message);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::thread spin_thread_;

  std::atomic<bool> wanted_{false};      // run the belt (true) or stop it
  std::atomic<bool> have_heard_{false};  // has anybody told us anything yet?
  std::atomic<bool> pending_{false};     // there is a change to send
};

}  // namespace scara_firmware

#endif  // SCARA_FIRMWARE__CONVEYOR_RELAY_HPP_

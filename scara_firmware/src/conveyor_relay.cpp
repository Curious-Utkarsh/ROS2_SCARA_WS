#include "scara_firmware/conveyor_relay.hpp"

namespace scara_firmware
{
namespace
{

// The topic the conveyor state machine already publishes on -- the same one the
// simulated belt listens to (scara_conveyor/common.py, TOPIC_RUN). Reusing it
// means the sim and the real cell are driven by identical messages.
constexpr const char * RUN_TOPIC = "/conveyor/run";

// Must match the publisher, or the messages never arrive.
//
// conveyor_state_machine.py publishes this one LATCHED (reliable +
// transient-local, depth 1) so a late subscriber still learns the current belt
// state. Subscribing the same way means we are told whether the belt should be
// running the moment the controllers come up, without waiting for the next
// change. ANY OTHER PUBLISHER ON THIS TOPIC MUST BE LATCHED TOO.
rclcpp::QoS runQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace

ConveyorRelay::~ConveyorRelay()
{
  stop();
}

void ConveyorRelay::start()
{
  if (node_) {
    return;
  }

  // rclcpp is already initialised -- we are living inside controller_manager's
  // process, which did that long before it loaded this plugin.
  node_ = std::make_shared<rclcpp::Node>("scara_conveyor_relay");
  subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
    RUN_TOPIC, runQos(),
    [this](const std_msgs::msg::Bool::SharedPtr message) { onRun(message); });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(node_->get_logger(), "Listening on %s for the conveyor belt", RUN_TOPIC);
}

void ConveyorRelay::stop()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  subscription_.reset();
  executor_.reset();
  node_.reset();

  have_heard_.store(false);
  pending_.store(false);
}

void ConveyorRelay::onRun(const std_msgs::msg::Bool::SharedPtr message)
{
  const bool run = message->data;

  // Only raise the flag when something actually changed, so a publisher that
  // repeats itself cannot fill the serial line with belt commands the arm then
  // has to wait behind.
  if (have_heard_.load() && run == wanted_.load()) {
    return;
  }

  wanted_.store(run);
  have_heard_.store(true);
  pending_.store(true);

  RCLCPP_INFO(node_->get_logger(), "Conveyor belt: %s", run ? "RUN" : "STOP");
}

bool ConveyorRelay::takePending(bool & run)
{
  if (!pending_.exchange(false)) {
    return false;
  }
  run = wanted_.load();
  return true;
}

}  // namespace scara_firmware

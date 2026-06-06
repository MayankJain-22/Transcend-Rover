#include "transcend_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace transcend_hardware
{

hardware_interface::CallbackReturn
TranscendHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  if (info_.joints.size() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("TranscendHardware"),
      "Expected 4 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(rclcpp::get_logger("TranscendHardware"),
        "Joint '%s' must have exactly one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("TranscendHardware"), "on_init OK");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TranscendHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < 4; i++) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
TranscendHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < 4; i++) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn
TranscendHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("TranscendHardware"), "Activating...");

  node_ = rclcpp::Node::make_shared("transcend_hw_bridge");

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/wheel_vel_cmd", rclcpp::QoS(10));

  state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/wheel_vel_state", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if (msg->data.size() < 4) return;
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (size_t i = 0; i < 4; i++) latest_vel_[i] = msg->data[i];
      state_received_ = true;
    });

  hw_states_position_.fill(0.0);
  hw_states_velocity_.fill(0.0);
  hw_commands_.fill(0.0);
  latest_vel_.fill(0.0f);
  pending_cmd_.fill(0.0f);
  state_received_ = false;
  stop_spin_ = false;
  new_cmd_pending_ = false;

  // Executor for subscriber callbacks
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  // Single background thread handles BOTH:
  //   1. Spinning the executor (receives /wheel_vel_state at 50Hz)
  //   2. Publishing /wheel_vel_cmd (keeps DDS publish off the RT thread)
  spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("TranscendHardware"), "Background thread started.");
    while (!stop_spin_ && rclcpp::ok()) {
      // Publish any pending command — this is the key fix.
      // write() deposits into pending_cmd_, this thread actually publishes.
      // DDS network I/O never touches the RT thread again.
      if (new_cmd_pending_.exchange(false)) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(4);
        {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          for (size_t i = 0; i < 4; i++) msg.data[i] = pending_cmd_[i];
        }
        cmd_pub_->publish(msg);
      }
      // Spin to receive /wheel_vel_state callbacks
      executor_->spin_some(std::chrono::milliseconds(5));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    RCLCPP_INFO(rclcpp::get_logger("TranscendHardware"), "Background thread stopped.");
  });

  RCLCPP_INFO(rclcpp::get_logger("TranscendHardware"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
TranscendHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Send stop command via the background thread
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    pending_cmd_.fill(0.0f);
  }
  new_cmd_pending_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // let it publish

  stop_spin_ = true;
  if (spin_thread_.joinable()) spin_thread_.join();

  executor_.reset();
  cmd_pub_.reset();
  state_sub_.reset();
  node_.reset();

  RCLCPP_INFO(rclcpp::get_logger("TranscendHardware"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// read() — RT thread, must be fast. Just copy under mutex.
hardware_interface::return_type
TranscendHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_received_) {
      for (size_t i = 0; i < 4; i++)
        hw_states_velocity_[i] = static_cast<double>(latest_vel_[i]);
    }
  }
  double dt = period.seconds();
  for (size_t i = 0; i < 4; i++)
    hw_states_position_[i] += hw_states_velocity_[i] * dt;

  return hardware_interface::return_type::OK;
}

// write() — RT thread, must be fast. Deposit into pending_cmd_, signal background thread.
hardware_interface::return_type
TranscendHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (size_t i = 0; i < 4; i++)
      pending_cmd_[i] = static_cast<float>(hw_commands_[i]);
  }
  new_cmd_pending_ = true;   // signal background thread to publish
  return hardware_interface::return_type::OK;
}

}  // namespace transcend_hardware

PLUGINLIB_EXPORT_CLASS(
  transcend_hardware::TranscendHardwareInterface,
  hardware_interface::SystemInterface)
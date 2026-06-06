#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <vector>
#include <string>
#include <array>
#include <mutex>
#include <thread>
#include <atomic>

namespace transcend_hardware
{

class TranscendHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spin_{false};

  // State from ESP32 — written by subscriber (spin_thread_), read by read() (RT)
  std::array<float, 4>  latest_vel_  = {0.0f, 0.0f, 0.0f, 0.0f};
  std::mutex            state_mutex_;
  bool                  state_received_ = false;

  // Commands to ESP32 — written by write() (RT), published by spin_thread_
  // Keeps DDS publish off the RT thread entirely
  std::array<float, 4>  pending_cmd_ = {0.0f, 0.0f, 0.0f, 0.0f};
  std::mutex            cmd_mutex_;
  std::atomic<bool>     new_cmd_pending_{false};

  // ros2_control data arrays
  std::array<double, 4> hw_states_position_ = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> hw_states_velocity_ = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> hw_commands_        = {0.0, 0.0, 0.0, 0.0};

  const std::array<std::string, 4> joint_names_ = {
    "base_front_left_wheel_joint",
    "base_front_right_wheel_joint",
    "base_rear_left_wheel_joint",
    "base_rear_right_wheel_joint"
  };
};

}  // namespace transcend_hardware
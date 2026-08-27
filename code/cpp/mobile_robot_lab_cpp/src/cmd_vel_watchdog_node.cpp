// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace mobile_robot_lab_cpp
{

class CmdVelWatchdogNode : public rclcpp::Node
{
public:
  CmdVelWatchdogNode()
  : Node("cmd_vel_watchdog")
  {
    input_topic_ = declare_parameter("input_topic", "/cmd_vel_safe");
    output_topic_ = declare_parameter("output_topic", "/cmd_vel");
    timeout_ = declare_parameter("timeout", 0.3);
    if (timeout_ <= 0.0) {
      throw std::invalid_argument("watchdog timeout must be positive");
    }
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr message) {
        last_command_ = *message;
        last_receipt_ = now();
      });
    timer_ = create_wall_timer(50ms, std::bind(&CmdVelWatchdogNode::update, this));
  }

private:
  void update()
  {
    const bool stale = !last_receipt_ || (now() - *last_receipt_).seconds() > timeout_;
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    if (!stale && last_command_) {
      output.twist = last_command_->twist;
    }
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double timeout_;
  std::optional<geometry_msgs::msg::TwistStamped> last_command_;
  std::optional<rclcpp::Time> last_receipt_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mobile_robot_lab_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_robot_lab_cpp::CmdVelWatchdogNode>());
  rclcpp::shutdown();
  return 0;
}

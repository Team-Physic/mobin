// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// Semantic safety의 `/cmd_vel_safe`를 robot 구동용 `/cmd_vel`로 전달하는 마지막 watchdog node다.
// 최근 command 수신시각을 ROS clock으로 측정하고 기본 0.3초 동안 갱신되지 않으면 zero command를 낸다.
// 출력은 20 Hz `TwistStamped`이며 frame_id는 robot body를 나타내는 `base_link`다.
// Safety node가 종료되어도 이전의 비영속 속도 명령이 actuator에 남지 않게 하는 것이 목적이다.

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mobile_robot_lab_cpp/semantic_safety.hpp"

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
    const double receipt_age = last_receipt_ ?
      (now() - *last_receipt_).seconds() : std::numeric_limits<double>::infinity();
    // 안전 node가 죽거나 command 갱신이 끊기면 이전 속도를 재사용하지 않고 즉시 0을 보낸다.
    const bool stale = command_is_stale(last_receipt_.has_value(), receipt_age, timeout_);
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

// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "mobile_robot_lab_cpp/scan_utils.hpp"

using namespace std::chrono_literals;

namespace mobile_robot_lab_cpp
{

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  ObstacleAvoidanceNode()
  : Node("obstacle_avoidance")
  {
    linear_velocity_ = declare_parameter("linear_velocity", 0.15);
    angular_velocity_ = declare_parameter("angular_velocity", 0.8);
    stop_distance_ = declare_parameter("stop_distance", 0.45);
    clear_distance_ = declare_parameter("clear_distance", 0.55);
    side_distance_ = declare_parameter("side_distance", 0.40);
    scan_timeout_ = declare_parameter("scan_timeout", 0.5);
    front_half_angle_ = declare_parameter("front_half_angle_deg", 15.0) * M_PI / 180.0;
    side_angle_ = declare_parameter("side_angle_deg", 60.0) * M_PI / 180.0;
    preferred_turn_direction_ = declare_parameter("preferred_turn_direction", "auto");

    const bool valid = linear_velocity_ >= 0.0 && angular_velocity_ > 0.0 &&
      stop_distance_ > 0.0 && clear_distance_ > stop_distance_ &&
      side_distance_ > 0.0 && scan_timeout_ > 0.0 &&
      front_half_angle_ > 0.0 && front_half_angle_ < side_angle_ &&
      side_angle_ <= M_PI &&
      (preferred_turn_direction_ == "auto" ||
      preferred_turn_direction_ == "left" ||
      preferred_turn_direction_ == "right");
    if (!valid) {
      throw std::invalid_argument("invalid obstacle avoidance parameters");
    }

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidanceNode::scan_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(50ms, std::bind(&ObstacleAvoidanceNode::update, this));
  }

  ~ObstacleAvoidanceNode() override
  {
    if (rclcpp::ok()) {
      publish(0.0, 0.0);
    }
  }

private:
  int choose_turn_direction() const
  {
    if (preferred_turn_direction_ == "left") {
      return 1;
    }
    if (preferred_turn_direction_ == "right") {
      return -1;
    }
    return *distances_.left >= *distances_.right ? 1 : -1;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    distances_ = sector_distances(*message, front_half_angle_, side_angle_);
    last_scan_time_ = now();
    has_scan_ = true;
  }

  void publish(const double linear, const double angular)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.twist.linear.x = linear;
    command.twist.angular.z = angular;
    publisher_->publish(command);
  }

  void publish_mode(
    const double linear, const double angular, const std::string & mode)
  {
    publish(linear, angular);
    if (mode != last_mode_) {
      RCLCPP_INFO(get_logger(), "avoidance mode: %s", mode.c_str());
      last_mode_ = mode;
    }
  }

  void update()
  {
    const bool timed_out = !has_scan_ ||
      (now() - last_scan_time_).seconds() > scan_timeout_;
    const bool invalid = !distances_.front || !distances_.left || !distances_.right;
    if (timed_out || invalid) {
      avoiding_ = true;
      publish_mode(0.0, 0.0, timed_out ? "scan_timeout" : "invalid_scan");
      return;
    }

    if (*distances_.front <= stop_distance_ && !avoiding_) {
      avoiding_ = true;
      turn_direction_ = choose_turn_direction();
    } else if (avoiding_ && *distances_.front >= clear_distance_) {
      avoiding_ = false;
      turn_direction_ = 0;
    }

    if (!avoiding_) {
      publish_mode(linear_velocity_, 0.0, "forward");
      return;
    }

    const bool left_blocked = *distances_.left <= side_distance_;
    const bool right_blocked = *distances_.right <= side_distance_;
    if (left_blocked && right_blocked) {
      publish_mode(0.0, 0.0, "blocked");
      return;
    }
    const bool preferred_side_blocked =
      (turn_direction_ > 0 && left_blocked) ||
      (turn_direction_ < 0 && right_blocked);
    if (preferred_turn_direction_ != "auto" && preferred_side_blocked) {
      publish_mode(0.0, 0.0, "blocked");
      return;
    }
    if (turn_direction_ > 0 && left_blocked) {
      turn_direction_ = -1;
    } else if (turn_direction_ < 0 && right_blocked) {
      turn_direction_ = 1;
    } else if (turn_direction_ == 0) {
      turn_direction_ = choose_turn_direction();
    }
    publish_mode(
      0.0, turn_direction_ * angular_velocity_,
      turn_direction_ > 0 ? "turn_left" : "turn_right");
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  SectorDistances distances_;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  std::string last_mode_;
  std::string preferred_turn_direction_;
  bool has_scan_{false};
  bool avoiding_{false};
  int turn_direction_{0};
  double linear_velocity_;
  double angular_velocity_;
  double stop_distance_;
  double clear_distance_;
  double side_distance_;
  double scan_timeout_;
  double front_half_angle_;
  double side_angle_;
};

}  // namespace mobile_robot_lab_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_robot_lab_cpp::ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}

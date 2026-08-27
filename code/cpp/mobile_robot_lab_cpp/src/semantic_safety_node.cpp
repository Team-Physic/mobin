// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mobile_robot_lab_msgs/msg/tracked_person_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mobile_robot_lab_cpp/semantic_safety.hpp"

using namespace std::chrono_literals;

namespace mobile_robot_lab_cpp
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

class SemanticSafetyNode : public rclcpp::Node
{
public:
  SemanticSafetyNode()
  : Node("semantic_safety"), policy_(safety_config())
  {
    command_topic_ = declare_parameter("command_topic", "/cmd_vel_mux");
    output_topic_ = declare_parameter("output_topic", "/cmd_vel_safe");
    scan_topic_ = declare_parameter("scan_topic", "/scan");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    tracks_topic_ = declare_parameter("tracks_topic", "/tracking/persons");
    scan_timeout_ = declare_parameter("scan_timeout", 0.2);
    odom_timeout_ = declare_parameter("odom_timeout", 0.2);
    tracking_timeout_ = declare_parameter("tracking_timeout", 0.5);
    front_half_angle_ = declare_parameter("front_half_angle_deg", 30.0) * M_PI / 180.0;
    side_limit_angle_ = declare_parameter("side_limit_angle_deg", 120.0) * M_PI / 180.0;
    if (scan_timeout_ <= 0.0 || odom_timeout_ <= 0.0 || tracking_timeout_ <= 0.0 ||
      front_half_angle_ <= 0.0 || side_limit_angle_ <= front_half_angle_ ||
      side_limit_angle_ > M_PI)
    {
      throw std::invalid_argument("invalid semantic safety node configuration");
    }

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/safety/state", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/safety/markers", 10);
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      command_topic_, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr message) {
        requested_ = *message;
      });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SemanticSafetyNode::on_scan, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SemanticSafetyNode::on_odom, this, std::placeholders::_1));
    tracks_subscription_ = create_subscription<mobile_robot_lab_msgs::msg::TrackedPersonArray>(
      tracks_topic_, 10,
      std::bind(&SemanticSafetyNode::on_tracks, this, std::placeholders::_1));
    timer_ = create_wall_timer(50ms, std::bind(&SemanticSafetyNode::update, this));
  }

private:
  SafetyConfig safety_config()
  {
    SafetyConfig config;
    config.caution_ttc = declare_parameter("caution_ttc", 4.0);
    config.danger_ttc = declare_parameter("danger_ttc", 2.0);
    config.prediction_horizon = declare_parameter("prediction_horizon", 6.0);
    config.caution_scale = declare_parameter("caution_scale", 0.5);
    config.robot_radius = declare_parameter("robot_radius", 0.30);
    config.person_radius = declare_parameter("person_radius", 0.35);
    config.safety_margin = declare_parameter("safety_margin", 0.20);
    config.front_emergency_distance = declare_parameter("front_emergency_distance", 0.60);
    config.rear_emergency_distance = declare_parameter("rear_emergency_distance", 0.40);
    config.side_emergency_distance = declare_parameter("side_emergency_distance", 0.45);
    config.tracking_stale_speed = declare_parameter("tracking_stale_speed", 0.15);
    config.stable_safe_duration = declare_parameter("stable_safe_duration", 1.0);
    return config;
  }

  double receipt_age(const std::optional<rclcpp::Time> & receipt) const
  {
    return receipt ? (now() - *receipt).seconds() : std::numeric_limits<double>::infinity();
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    DirectionalRanges ranges;
    for (std::size_t index = 0; index < message->ranges.size(); ++index) {
      const double distance = message->ranges[index];
      if (!std::isfinite(distance) || distance < message->range_min ||
        distance > message->range_max)
      {
        continue;
      }
      const double raw_angle = message->angle_min + index * message->angle_increment;
      const double angle = std::atan2(std::sin(raw_angle), std::cos(raw_angle));
      if (std::abs(angle) <= front_half_angle_) {
        update_minimum(ranges.front, distance);
      } else if (std::abs(std::abs(angle) - M_PI) <= front_half_angle_) {
        update_minimum(ranges.rear, distance);
      } else if (front_half_angle_ < angle && angle <= side_limit_angle_) {
        update_minimum(ranges.left, distance);
      } else if (-side_limit_angle_ <= angle && angle < -front_half_angle_) {
        update_minimum(ranges.right, distance);
      }
    }
    ranges_ = ranges;
    scan_receipt_ = now();
  }

  static void update_minimum(std::optional<double> & current, const double value)
  {
    current = current ? std::min(*current, value) : value;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double yaw = yaw_from_quaternion(message->pose.pose.orientation);
    const double body_x = message->twist.twist.linear.x;
    const double body_y = message->twist.twist.linear.y;
    robot_.position = {message->pose.pose.position.x, message->pose.pose.position.y};
    // Odometry twist를 base_footprint에서 odom으로 회전해 person velocity와 비교한다.
    robot_.velocity = {
      std::cos(yaw) * body_x - std::sin(yaw) * body_y,
      std::sin(yaw) * body_x + std::cos(yaw) * body_y};
    odom_receipt_ = now();
  }

  void on_tracks(const mobile_robot_lab_msgs::msg::TrackedPersonArray::SharedPtr message)
  {
    persons_.clear();
    persons_.reserve(message->persons.size());
    for (const auto & person : message->persons) {
      persons_.push_back({
          person.track_id,
          {person.position.x, person.position.y},
          {person.velocity.x, person.velocity.y},
          person.uncertainty_radius});
    }
    tracks_receipt_ = now();
  }

  void update()
  {
    VelocityCommand command;
    if (requested_) {
      command.linear = requested_->twist.linear.x;
      command.angular = requested_->twist.angular.z;
    }
    const auto result = policy_.evaluate(
      command, robot_, persons_, ranges_,
      receipt_age(scan_receipt_) > scan_timeout_,
      receipt_age(odom_receipt_) > odom_timeout_,
      receipt_age(tracks_receipt_) > tracking_timeout_, now().seconds());

    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.twist.linear.x = result.command.linear;
    output.twist.angular.z = result.command.angular;
    publisher_->publish(output);
    publish_diagnostics(result);
    marker_publisher_->publish(markers(result));
  }

  visualization_msgs::msg::MarkerArray markers(const SafetyResult & result)
  {
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = now();
    clear.header.frame_id = "odom";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);
    const auto selected = std::find_if(
      persons_.begin(), persons_.end(),
      [&result](const TrackedPerson2 & person) {return person.id == result.selected_track_id;});
    if (selected == persons_.end()) {
      return output;
    }

    visualization_msgs::msg::Marker circle;
    circle.header = clear.header;
    circle.ns = "selected_person_safety_radius";
    circle.id = selected->id;
    circle.type = visualization_msgs::msg::Marker::CYLINDER;
    circle.action = visualization_msgs::msg::Marker::ADD;
    circle.pose.position.x = selected->position.x;
    circle.pose.position.y = selected->position.y;
    circle.pose.orientation.w = 1.0;
    circle.scale.x = circle.scale.y = result.selected_safe_distance * 2.0;
    circle.scale.z = 0.02;
    circle.color.r = 1.0F;
    circle.color.g = 0.5F;
    circle.color.a = 0.25F;
    output.markers.push_back(circle);

    visualization_msgs::msg::Marker cpa;
    cpa.header = clear.header;
    cpa.ns = "person_cpa";
    cpa.id = selected->id;
    cpa.type = visualization_msgs::msg::Marker::SPHERE;
    cpa.action = visualization_msgs::msg::Marker::ADD;
    cpa.pose.position.x = selected->position.x + selected->velocity.x * result.selected_ttc;
    cpa.pose.position.y = selected->position.y + selected->velocity.y * result.selected_ttc;
    cpa.pose.orientation.w = 1.0;
    cpa.scale.x = cpa.scale.y = cpa.scale.z = 0.16;
    cpa.color.r = cpa.color.a = 1.0F;
    output.markers.push_back(cpa);
    return output;
  }

  void publish_diagnostics(const SafetyResult & result)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "semantic_safety";
    status.hardware_id = "mobin_sim";
    status.level = result.mode == SafetyMode::kSafe ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      (result.mode == SafetyMode::kCaution ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.message = result.reason;
    diagnostic_msgs::msg::KeyValue reason;
    reason.key = "reason";
    reason.value = result.reason;
    status.values.push_back(reason);
    const auto add = [&status](const std::string & key, const auto & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = std::to_string(value);
        status.values.push_back(item);
      };
    add("selected_track_id", result.selected_track_id);
    add("t_cpa", result.selected_ttc);
    add("d_cpa", result.selected_distance);
    add("d_safe", result.selected_safe_distance);
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  SemanticSafety policy_;
  std::string command_topic_;
  std::string output_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string tracks_topic_;
  double scan_timeout_;
  double odom_timeout_;
  double tracking_timeout_;
  double front_half_angle_;
  double side_limit_angle_;
  RobotMotion2 robot_;
  DirectionalRanges ranges_;
  std::vector<TrackedPerson2> persons_;
  std::optional<geometry_msgs::msg::TwistStamped> requested_;
  std::optional<rclcpp::Time> scan_receipt_;
  std::optional<rclcpp::Time> odom_receipt_;
  std::optional<rclcpp::Time> tracks_receipt_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<mobile_robot_lab_msgs::msg::TrackedPersonArray>::SharedPtr
    tracks_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mobile_robot_lab_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_robot_lab_cpp::SemanticSafetyNode>());
  rclcpp::shutdown();
  return 0;
}

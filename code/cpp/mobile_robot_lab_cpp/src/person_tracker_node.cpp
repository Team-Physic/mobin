// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iterator>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/point.hpp>
#include <mobile_robot_lab_msgs/msg/tracked_person.hpp>
#include <mobile_robot_lab_msgs/msg/tracked_person_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mobile_robot_lab_cpp/constant_velocity_tracker.hpp"

namespace mobile_robot_lab_cpp
{

class PersonTrackerNode : public rclcpp::Node
{
public:
  PersonTrackerNode()
  : Node("person_tracker"), tracker_(tracker_config())
  {
    input_topic_ = declare_parameter("input_topic", "/fusion/detections_3d");
    output_topic_ = declare_parameter("output_topic", "/tracking/persons");
    marker_topic_ = declare_parameter("marker_topic", "/tracking/markers");
    trajectory_length_ = declare_parameter("trajectory_length", 50);
    if (trajectory_length_ < 1) {
      throw std::invalid_argument("trajectory_length must be positive");
    }
    persons_publisher_ = create_publisher<mobile_robot_lab_msgs::msg::TrackedPersonArray>(
      output_topic_, 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PersonTrackerNode::on_detections, this, std::placeholders::_1));
  }

private:
  TrackerConfig tracker_config()
  {
    TrackerConfig config;
    config.euclidean_gate = declare_parameter("euclidean_gate", 0.75);
    config.mahalanobis_gate_squared = declare_parameter("mahalanobis_gate_squared", 5.991);
    config.max_age = declare_parameter("track_max_age", 0.5);
    config.measurement_stddev = declare_parameter("measurement_stddev", 0.20);
    config.acceleration_stddev = declare_parameter("acceleration_stddev", 1.0);
    config.initial_velocity_stddev = declare_parameter("initial_velocity_stddev", 1.0);
    config.uncertainty_scale = declare_parameter("uncertainty_scale", 2.447);
    config.uncertainty_min = declare_parameter("uncertainty_min", 0.05);
    config.uncertainty_max = declare_parameter("uncertainty_max", 0.40);
    config.confirmation_window = static_cast<std::size_t>(
      declare_parameter("confirmation_window", 5));
    config.confirmation_hits = static_cast<std::size_t>(
      declare_parameter("confirmation_hits", 3));
    return config;
  }

  static double seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return stamp.sec + stamp.nanosec * 1e-9;
  }

  void on_detections(const vision_msgs::msg::Detection3DArray::SharedPtr message)
  {
    std::vector<Detection2> detections;
    detections.reserve(message->detections.size());
    for (const auto & detection : message->detections) {
      if (detection.results.empty()) {
        continue;
      }
      const auto & point = detection.results.front().pose.pose.position;
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        detections.push_back({point.x, point.y});
      }
    }

    const auto & tracks = tracker_.update(detections, seconds(message->header.stamp));
    mobile_robot_lab_msgs::msg::TrackedPersonArray output;
    output.header = message->header;
    output.persons.reserve(tracks.size());
    for (const auto & track : tracks) {
      mobile_robot_lab_msgs::msg::TrackedPerson person;
      person.track_id = track.id;
      person.state = static_cast<std::uint8_t>(track.state);
      person.position.x = track.estimate[0];
      person.position.y = track.estimate[1];
      person.velocity.x = track.estimate[2];
      person.velocity.y = track.estimate[3];
      person.position_covariance = {
        track.covariance(0, 0), track.covariance(0, 1),
        track.covariance(1, 0), track.covariance(1, 1)};
      person.uncertainty_radius = tracker_.uncertainty_radius(track);
      person.last_measurement_stamp = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(
          static_cast<int64_t>(track.last_measurement_stamp * 1e9)));
      output.persons.push_back(person);
      auto & trajectory = trajectories_[track.id];
      trajectory.push_back(person.position);
      while (trajectory.size() > static_cast<std::size_t>(trajectory_length_)) {
        trajectory.pop_front();
      }
    }
    remove_deleted_trajectories(tracks);
    persons_publisher_->publish(output);
    marker_publisher_->publish(markers(output));
  }

  void remove_deleted_trajectories(const std::vector<PersonTrack> & tracks)
  {
    for (auto iterator = trajectories_.begin(); iterator != trajectories_.end();) {
      const bool exists = std::any_of(
        tracks.begin(), tracks.end(),
        [&iterator](const PersonTrack & track) {return track.id == iterator->first;});
      iterator = exists ? std::next(iterator) : trajectories_.erase(iterator);
    }
  }

  visualization_msgs::msg::MarkerArray markers(
    const mobile_robot_lab_msgs::msg::TrackedPersonArray & persons) const
  {
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.header = persons.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);
    for (const auto & person : persons.persons) {
      add_position_marker(persons.header, person, output);
      add_label_marker(persons.header, person, output);
      add_velocity_marker(persons.header, person, output);
      add_trajectory_marker(persons.header, person, output);
      add_covariance_marker(persons.header, person, output);
    }
    return output;
  }

  static void set_state_color(
    const mobile_robot_lab_msgs::msg::TrackedPerson & person,
    std_msgs::msg::ColorRGBA & color)
  {
    color.a = 0.9F;
    if (person.state == mobile_robot_lab_msgs::msg::TrackedPerson::CONFIRMED) {
      color.g = 1.0F;
    } else if (person.state == mobile_robot_lab_msgs::msg::TrackedPerson::LOST) {
      color.r = 1.0F;
      color.g = 0.7F;
    } else {
      color.r = 1.0F;
    }
  }

  static void add_position_marker(
    const std_msgs::msg::Header & header,
    const mobile_robot_lab_msgs::msg::TrackedPerson & person,
    visualization_msgs::msg::MarkerArray & output)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "person_position";
    marker.id = person.track_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = person.position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.24;
    set_state_color(person, marker.color);
    output.markers.push_back(marker);
  }

  static void add_label_marker(
    const std_msgs::msg::Header & header,
    const mobile_robot_lab_msgs::msg::TrackedPerson & person,
    visualization_msgs::msg::MarkerArray & output)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "person_id";
    marker.id = person.track_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = person.position;
    marker.pose.position.z += 0.45;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.20;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0F;
    const char * state = person.state == mobile_robot_lab_msgs::msg::TrackedPerson::CONFIRMED ?
      "confirmed" :
      (person.state == mobile_robot_lab_msgs::msg::TrackedPerson::LOST ? "lost" : "tentative");
    marker.text = "person " + std::to_string(person.track_id) + " " + state;
    output.markers.push_back(marker);
  }

  static void add_velocity_marker(
    const std_msgs::msg::Header & header,
    const mobile_robot_lab_msgs::msg::TrackedPerson & person,
    visualization_msgs::msg::MarkerArray & output)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "person_velocity";
    marker.id = person.track_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.09;
    marker.scale.z = 0.09;
    marker.color.b = marker.color.a = 1.0F;
    marker.points.push_back(person.position);
    auto end = person.position;
    end.x += person.velocity.x;
    end.y += person.velocity.y;
    marker.points.push_back(end);
    output.markers.push_back(marker);
  }

  void add_trajectory_marker(
    const std_msgs::msg::Header & header,
    const mobile_robot_lab_msgs::msg::TrackedPerson & person,
    visualization_msgs::msg::MarkerArray & output) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "person_trajectory";
    marker.id = person.track_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.g = marker.color.a = 1.0F;
    const auto found = trajectories_.find(person.track_id);
    if (found != trajectories_.end()) {
      marker.points.assign(found->second.begin(), found->second.end());
    }
    output.markers.push_back(marker);
  }

  static void add_covariance_marker(
    const std_msgs::msg::Header & header,
    const mobile_robot_lab_msgs::msg::TrackedPerson & person,
    visualization_msgs::msg::MarkerArray & output)
  {
    Eigen::Matrix2d covariance;
    covariance << person.position_covariance[0], person.position_covariance[1],
      person.position_covariance[2], person.position_covariance[3];
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    const Eigen::Vector2d values = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt() * 2.447;
    const Eigen::Matrix2d vectors = solver.eigenvectors();

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "person_covariance";
    marker.id = person.track_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.color.r = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 0.9F;
    constexpr int samples = 36;
    for (int index = 0; index <= samples; ++index) {
      const double angle = 2.0 * M_PI * index / samples;
      const Eigen::Vector2d local{values[0] * std::cos(angle), values[1] * std::sin(angle)};
      const Eigen::Vector2d point = vectors * local;
      geometry_msgs::msg::Point output_point = person.position;
      output_point.x += point[0];
      output_point.y += point[1];
      marker.points.push_back(output_point);
    }
    output.markers.push_back(marker);
  }

  ConstantVelocityTracker tracker_;
  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  int trajectory_length_;
  std::unordered_map<int, std::deque<geometry_msgs::msg::Point>> trajectories_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
  rclcpp::Publisher<mobile_robot_lab_msgs::msg::TrackedPersonArray>::SharedPtr persons_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace mobile_robot_lab_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_robot_lab_cpp::PersonTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#ifndef MOBILE_ROBOT_LAB_CPP__SCAN_UTILS_HPP_
#define MOBILE_ROBOT_LAB_CPP__SCAN_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace mobile_robot_lab_cpp
{

struct SectorDistances
{
  std::optional<double> front;
  std::optional<double> left;
  std::optional<double> right;
};

inline void update_min(std::optional<double> & current, const double value)
{
  current = current ? std::min(*current, value) : value;
}

inline SectorDistances sector_distances(
  const sensor_msgs::msg::LaserScan & scan,
  const double front_half_angle,
  const double side_angle)
{
  SectorDistances result;
  if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment == 0.0)
  {
    return result;
  }

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    double distance = scan.ranges[index];
    if (std::isinf(distance) && distance > 0.0) {
      distance = scan.range_max;
    }
    if (!std::isfinite(distance) || distance < scan.range_min ||
      distance > scan.range_max)
    {
      continue;
    }

    const double raw_angle = scan.angle_min +
      static_cast<double>(index) * scan.angle_increment;
    const double angle = std::atan2(std::sin(raw_angle), std::cos(raw_angle));
    if (std::abs(angle) <= front_half_angle) {
      update_min(result.front, distance);
    } else if (angle > front_half_angle && angle <= side_angle) {
      update_min(result.left, distance);
    } else if (angle < -front_half_angle && angle >= -side_angle) {
      update_min(result.right, distance);
    }
  }
  return result;
}

}  // namespace mobile_robot_lab_cpp

#endif  // MOBILE_ROBOT_LAB_CPP__SCAN_UTILS_HPP_

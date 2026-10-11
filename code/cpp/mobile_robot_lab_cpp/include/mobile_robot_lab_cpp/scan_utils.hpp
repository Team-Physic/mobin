// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// LaserScan을 robot 전방·좌측·우측 각도 구간으로 나누고 구간별 최소거리를 계산한다.
// 입력 각도는 radian, range는 meter이며 NaN과 sensor 유효범위 밖의 값은 제외한다.
// 양의 infinity는 장애물 없음으로 보고 range_max로 바꿔 구간의 유효 관측으로 유지한다.
// 구간에 유효 sample이 없으면 nullopt를 반환하여 controller가 안전 정지를 선택하게 한다.

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

// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <cassert>
#include <cmath>
#include <vector>

#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"

int main()
{
  using mobile_robot_lab_cpp::BoundingBox;
  using mobile_robot_lab_cpp::Intrinsics;
  using mobile_robot_lab_cpp::Point3;
  using mobile_robot_lab_cpp::Rotation3;

  const std::vector<Point3> points{
    {-0.1, 0.0, 2.0}, {0.0, 0.0, 2.0}, {0.1, 0.0, 2.0},
    {2.0, 0.0, 2.0}, {0.0, 0.0, -1.0}};
  const Rotation3 identity{{
    {{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
  const auto result = mobile_robot_lab_cpp::associate_bbox(
    points, identity, {0.0, 0.0, 0.0}, {100.0, 100.0, 320.0, 240.0},
    {310.0, 230.0, 330.0, 250.0});
  assert(result);
  assert(result->point_count == 3);
  assert(result->matched_points_lidar.size() == 3);
  assert(std::abs(result->position_lidar.z - 2.0) < 1e-9);
  assert(std::abs(result->lidar_range - 2.0) < 1e-9);

  const auto too_few = mobile_robot_lab_cpp::associate_bbox(
    {{0.0, 0.0, 2.0}}, identity, {0.0, 0.0, 0.0},
    Intrinsics{100.0, 100.0, 320.0, 240.0}, BoundingBox{310.0, 230.0, 330.0, 250.0});
  assert(!too_few);
}

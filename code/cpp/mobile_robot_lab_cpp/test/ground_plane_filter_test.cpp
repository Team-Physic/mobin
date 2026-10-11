// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// Constrained PCL RANSAC이 robot 주변 바닥을 허용하고 먼 수평면을 거부하는지 검증한다.
// 합성 point는 Ground Reference Frame의 meter 좌표이며 외부 ROS node나 file을 사용하지 않는다.
// 유효 plane 실패 시 nullopt를 반환하여 runtime fallback이 point를 보존할 수 있어야 한다.

#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include "mobile_robot_lab_cpp/ground_plane_filter.hpp"

int main()
{
  using mobile_robot_lab_cpp::GroundPlaneConfig;
  using mobile_robot_lab_cpp::Point3;

  std::vector<Point3> floor_points;
  std::vector<std::size_t> indices;
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      indices.push_back(floor_points.size());
      floor_points.push_back({x * 0.1, y * 0.1, 0.03});
    }
  }
  const GroundPlaneConfig config{
    0.05, 20.0 * 3.14159265358979323846 / 180.0, 0.15, 50};
  const auto floor = mobile_robot_lab_cpp::estimate_ground_plane(
    floor_points, indices, config);
  assert(floor);
  assert(floor->inlier_count == floor_points.size());
  assert(std::abs(floor->origin_distance - 0.03) < 1e-3);
  assert(mobile_robot_lab_cpp::point_plane_distance({0.0, 0.0, 0.04}, *floor) < 0.05);
  assert(mobile_robot_lab_cpp::point_plane_distance({0.0, 0.0, 0.30}, *floor) > 0.05);

  std::vector<Point3> shelf_points = floor_points;
  for (auto & point : shelf_points) {
    point.z = 0.80;
  }
  const auto shelf = mobile_robot_lab_cpp::estimate_ground_plane(
    shelf_points, indices, config);
  assert(!shelf);
}

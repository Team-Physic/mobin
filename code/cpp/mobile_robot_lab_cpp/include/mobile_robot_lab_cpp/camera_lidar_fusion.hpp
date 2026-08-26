// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#ifndef MOBILE_ROBOT_LAB_CPP__CAMERA_LIDAR_FUSION_HPP_
#define MOBILE_ROBOT_LAB_CPP__CAMERA_LIDAR_FUSION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace mobile_robot_lab_cpp
{

struct Point3
{
  double x;
  double y;
  double z;
};

struct Intrinsics
{
  double fx;
  double fy;
  double cx;
  double cy;
};

struct BoundingBox
{
  double u_min;
  double v_min;
  double u_max;
  double v_max;
};

struct Association
{
  Point3 position_lidar;
  Point3 position_camera;
  double lidar_range;
  std::size_t point_count;
  std::vector<Point3> matched_points_lidar;
};

using Rotation3 = std::array<std::array<double, 3>, 3>;

inline Point3 transform_point(
  const Point3 & point, const Rotation3 & rotation, const Point3 & translation)
{
  return {
    rotation[0][0] * point.x + rotation[0][1] * point.y +
    rotation[0][2] * point.z + translation.x,
    rotation[1][0] * point.x + rotation[1][1] * point.y +
    rotation[1][2] * point.z + translation.y,
    rotation[2][0] * point.x + rotation[2][1] * point.y +
    rotation[2][2] * point.z + translation.z,
  };
}

inline std::optional<std::array<double, 2>> project_point(
  const Point3 & camera, const Intrinsics & intrinsics)
{
  if (camera.z <= 0.0) {
    return std::nullopt;
  }
  return std::array<double, 2>{
    intrinsics.fx * camera.x / camera.z + intrinsics.cx,
    intrinsics.fy * camera.y / camera.z + intrinsics.cy,
  };
}

inline double median(std::vector<double> values)
{
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 == 1) {
    return *middle;
  }
  const double upper = *middle;
  return (upper + *std::max_element(values.begin(), middle)) / 2.0;
}

inline std::optional<Association> associate_bbox(
  const std::vector<Point3> & points_lidar,
  const Rotation3 & rotation,
  const Point3 & translation,
  const Intrinsics & intrinsics,
  const BoundingBox & bbox,
  std::size_t min_points = 3)
{
  if (min_points == 0 || bbox.u_min > bbox.u_max || bbox.v_min > bbox.v_max) {
    return std::nullopt;
  }
  std::vector<Point3> lidar_matches;
  std::vector<Point3> camera_matches;
  for (const auto & lidar : points_lidar) {
    const auto camera = transform_point(lidar, rotation, translation);
    const auto pixel = project_point(camera, intrinsics);
    if (pixel && bbox.u_min <= (*pixel)[0] && (*pixel)[0] <= bbox.u_max &&
      bbox.v_min <= (*pixel)[1] && (*pixel)[1] <= bbox.v_max)
    {
      lidar_matches.push_back(lidar);
      camera_matches.push_back(camera);
    }
  }
  if (lidar_matches.size() < min_points) {
    return std::nullopt;
  }

  const auto component_median = [](const std::vector<Point3> & points, auto member) {
      std::vector<double> values;
      values.reserve(points.size());
      for (const auto & point : points) {
        values.push_back(point.*member);
      }
      return median(std::move(values));
    };
  const Point3 lidar{
    component_median(lidar_matches, &Point3::x),
    component_median(lidar_matches, &Point3::y),
    component_median(lidar_matches, &Point3::z)};
  const Point3 camera{
    component_median(camera_matches, &Point3::x),
    component_median(camera_matches, &Point3::y),
    component_median(camera_matches, &Point3::z)};
  return Association{
    lidar, camera, std::sqrt(lidar.x * lidar.x + lidar.y * lidar.y + lidar.z * lidar.z),
    lidar_matches.size(), std::move(lidar_matches)};
}

}  // namespace mobile_robot_lab_cpp

#endif  // MOBILE_ROBOT_LAB_CPP__CAMERA_LIDAR_FUSION_HPP_

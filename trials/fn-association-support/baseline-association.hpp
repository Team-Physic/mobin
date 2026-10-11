// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// Camera–LiDAR fusion에 필요한 3D 좌표 변환, pixel 투영, bbox association을 제공한다.
// 입력은 LiDAR frame point, T_camera_lidar, CameraInfo 내부 파라미터, image bbox다.
// 출력은 선택된 전경 point와 LiDAR/Camera frame의 대표 3D 사람 위치다.
// 모든 3D 길이는 meter이며 Camera optical frame은 x=right, y=down, z=forward를 따른다.

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

enum class AssociationStatus
{
  SUCCESS,
  NO_POINTS,
  MULTIPLE_DEPTH_CANDIDATES,
};

struct AssociationResult
{
  AssociationStatus status;
  std::optional<Association> association;
};

using Rotation3 = std::array<std::array<double, 3>, 3>;

// source frame의 point[m]에 T_target_source를 적용해 target frame 좌표를 반환한다.
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

// ROS camera optical convention(x=right, y=down, z=forward)과 pinhole model을 가정한다.
// Camera 뒤쪽(z<=0) point는 유효한 image pixel로 투영할 수 없다.
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

// 소수의 outlier가 사람 중심을 끌어가지 않도록 평균 대신 중앙값을 사용한다.
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

// Instance mask 안의 LiDAR point를 Camera 깊이로 군집화한다.
// 충분한 depth 군집이 정확히 하나일 때만 관측을 반환해 사람과 배경을 임의로 선택하지 않는다.
inline AssociationResult analyze_instance_points(
  const std::vector<Point3> & points_lidar,
  const Rotation3 & rotation,
  const Point3 & translation,
  std::size_t min_points = 3,
  double depth_window = 0.35)
{
  if (min_points == 0 || depth_window <= 0.0 || points_lidar.size() < min_points) {
    return {AssociationStatus::NO_POINTS, std::nullopt};
  }
  std::vector<Point3> camera_matches;
  camera_matches.reserve(points_lidar.size());
  for (const auto & lidar : points_lidar) {
    camera_matches.push_back(transform_point(lidar, rotation, translation));
  }

  // Camera optical frame의 z는 Camera 전방 깊이다. 큰 depth gap에서 서로 다른 표면으로 나눈다.
  std::vector<std::size_t> order(camera_matches.size());
  for (std::size_t index = 0; index < order.size(); ++index) {
    order[index] = index;
  }
  std::sort(order.begin(), order.end(), [&camera_matches](const auto left, const auto right) {
      return camera_matches[left].z < camera_matches[right].z;
    });
  std::vector<std::pair<std::size_t, std::size_t>> candidates;
  std::size_t begin = 0;
  for (std::size_t index = 1; index <= order.size(); ++index) {
    const bool boundary = index == order.size() ||
      camera_matches[order[index]].z - camera_matches[order[index - 1]].z > depth_window;
    if (boundary) {
      if (index - begin >= min_points) {
        candidates.emplace_back(begin, index);
      }
      begin = index;
    }
  }
  if (candidates.empty()) {
    return {AssociationStatus::NO_POINTS, std::nullopt};
  }
  if (candidates.size() > 1) {
    return {AssociationStatus::MULTIPLE_DEPTH_CANDIDATES, std::nullopt};
  }
  const auto [selected_begin, selected_end] = candidates.front();
  std::vector<Point3> selected_lidar;
  std::vector<Point3> selected_camera;
  selected_lidar.reserve(selected_end - selected_begin);
  selected_camera.reserve(selected_end - selected_begin);
  for (std::size_t index = selected_begin; index < selected_end; ++index) {
    selected_lidar.push_back(points_lidar[order[index]]);
    selected_camera.push_back(camera_matches[order[index]]);
  }

  // 선택한 군집의 축별 중앙값을 대표 3D 위치로 사용해 edge point와 sensor noise 영향을 줄인다.
  const auto component_median = [](const std::vector<Point3> & points, auto member) {
      std::vector<double> values;
      values.reserve(points.size());
      for (const auto & point : points) {
        values.push_back(point.*member);
      }
      return median(std::move(values));
    };
  const Point3 lidar{
    component_median(selected_lidar, &Point3::x),
    component_median(selected_lidar, &Point3::y),
    component_median(selected_lidar, &Point3::z)};
  const Point3 camera{
    component_median(selected_camera, &Point3::x),
    component_median(selected_camera, &Point3::y),
    component_median(selected_camera, &Point3::z)};
  return {
    AssociationStatus::SUCCESS,
    Association{
      lidar, camera, std::sqrt(lidar.x * lidar.x + lidar.y * lidar.y + lidar.z * lidar.z),
      selected_lidar.size(), std::move(selected_lidar)}};
}

inline std::optional<Association> associate_instance_points(
  const std::vector<Point3> & points_lidar,
  const Rotation3 & rotation,
  const Point3 & translation,
  std::size_t min_points = 3,
  double depth_window = 0.35)
{
  return analyze_instance_points(
    points_lidar, rotation, translation, min_points, depth_window).association;
}

// LiDAR point를 Camera pixel로 투영하고 bbox 내부 point에 동일한 ambiguity 규칙을 적용한다.
// Runtime은 Instance Label Map을 사용하며 이 함수는 bbox-only 비교 test를 위해 유지한다.
inline std::optional<Association> associate_bbox(
  const std::vector<Point3> & points_lidar,
  const Rotation3 & rotation,
  const Point3 & translation,
  const Intrinsics & intrinsics,
  const BoundingBox & bbox,
  std::size_t min_points = 3,
  double depth_window = 0.35)
{
  if (bbox.u_min > bbox.u_max || bbox.v_min > bbox.v_max) {
    return std::nullopt;
  }
  std::vector<Point3> matches;
  for (const auto & lidar : points_lidar) {
    const auto camera = transform_point(lidar, rotation, translation);
    const auto pixel = project_point(camera, intrinsics);
    if (pixel && bbox.u_min <= (*pixel)[0] && (*pixel)[0] <= bbox.u_max &&
      bbox.v_min <= (*pixel)[1] && (*pixel)[1] <= bbox.v_max)
    {
      matches.push_back(lidar);
    }
  }
  return associate_instance_points(
    matches, rotation, translation, min_points, depth_window);
}

}  // namespace mobile_robot_lab_cpp

#endif  // MOBILE_ROBOT_LAB_CPP__CAMERA_LIDAR_FUSION_HPP_

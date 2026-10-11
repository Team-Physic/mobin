// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// Camera 시야의 LiDAR point에서 주행 바닥으로 가능한 plane을 PCL RANSAC으로 추정한다.
// 입력과 plane 계수는 Ground Reference Frame 기준이며 모든 거리는 meter 단위다.
// 바닥 normal 방향, robot 원점 거리와 최소 inlier 수를 함께 검사한다.
// 유효한 plane을 찾지 못하면 nullopt를 반환하여 호출자가 point를 제거하지 않게 한다.

#ifndef MOBILE_ROBOT_LAB_CPP__GROUND_PLANE_FILTER_HPP_
#define MOBILE_ROBOT_LAB_CPP__GROUND_PLANE_FILTER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"

namespace mobile_robot_lab_cpp
{

struct GroundPlaneConfig
{
  double inlier_distance{0.05};
  double max_tilt_radians{20.0 * 3.14159265358979323846 / 180.0};
  double max_origin_distance{0.15};
  std::size_t min_inliers{50};
};

struct GroundPlane
{
  // ax + by + cz + d = 0, Ground Reference Frame 기준이다.
  std::array<double, 4> coefficients;
  std::size_t inlier_count;
  double origin_distance;
};

inline double point_plane_distance(const Point3 & point, const GroundPlane & plane)
{
  const auto & c = plane.coefficients;
  return std::abs(c[0] * point.x + c[1] * point.y + c[2] * point.z + c[3]);
}

inline std::optional<GroundPlane> estimate_ground_plane(
  const std::vector<Point3> & points,
  const std::vector<std::size_t> & candidate_indices,
  const GroundPlaneConfig & config)
{
  if (config.inlier_distance <= 0.0 || config.max_tilt_radians < 0.0 ||
    config.max_origin_distance < 0.0 || config.min_inliers < 3 ||
    candidate_indices.size() < config.min_inliers)
  {
    return std::nullopt;
  }

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(candidate_indices.size());
  for (const auto index : candidate_indices) {
    if (index >= points.size()) {
      return std::nullopt;
    }
    const auto & point = points[index];
    cloud->push_back(pcl::PointXYZ(
        static_cast<float>(point.x), static_cast<float>(point.y),
        static_cast<float>(point.z)));
  }

  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(config.inlier_distance);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(config.max_tilt_radians);
  segmentation.setInputCloud(cloud);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  if (inliers.indices.size() < config.min_inliers || coefficients.values.size() < 4) {
    return std::nullopt;
  }

  double a = coefficients.values[0];
  double b = coefficients.values[1];
  double c = coefficients.values[2];
  double d = coefficients.values[3];
  const double normal_norm = std::sqrt(a * a + b * b + c * c);
  if (normal_norm <= 1e-9) {
    return std::nullopt;
  }
  a /= normal_norm;
  b /= normal_norm;
  c /= normal_norm;
  d /= normal_norm;
  if (c < 0.0) {
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }

  // 수직 벽이나 선반 상판을 바닥으로 채택하지 않도록 normal과 robot 원점 거리를 재검증한다.
  const double tilt = std::acos(std::clamp(c, -1.0, 1.0));
  const double origin_distance = std::abs(d);
  if (tilt > config.max_tilt_radians || origin_distance > config.max_origin_distance) {
    return std::nullopt;
  }
  return GroundPlane{{a, b, c, d}, inliers.indices.size(), origin_distance};
}

}  // namespace mobile_robot_lab_cpp

#endif  // MOBILE_ROBOT_LAB_CPP__GROUND_PLANE_FILTER_HPP_

// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// Camera projection과 mono8 mask 이후의 depth candidate 선택 규칙을 검증한다.
// 합성 좌표는 meter, image 좌표는 pixel이며 ROS runtime이나 외부 file을 사용하지 않는다.
// 전경 point 지지가 충분할 때만 복수 후보를 허용하고 비슷한 지지·배경 우세는 거부한다.
// 동일 pixel의 뒤쪽 점만 제외하며 이웃 pixel·작은 깊이 차이·희소 관측을 보존한다.

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"

int main()
{
  using mobile_robot_lab_cpp::BoundingBox;
  using mobile_robot_lab_cpp::AssociationStatus;
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

  const auto ambiguous = mobile_robot_lab_cpp::associate_bbox(
    {{-0.1, 0.0, 2.0}, {0.0, 0.0, 2.1}, {0.1, 0.0, 2.2},
      {-0.1, 0.0, 5.0}, {0.0, 0.0, 5.0}, {0.1, 0.0, 5.0}, {0.2, 0.0, 5.0}},
    identity, {0.0, 0.0, 0.0}, {100.0, 100.0, 320.0, 240.0},
    {300.0, 220.0, 340.0, 260.0}, 3, 0.35);
  assert(!ambiguous);
  const auto ambiguous_analysis = mobile_robot_lab_cpp::analyze_instance_points(
    {{-0.1, 0.0, 2.0}, {0.0, 0.0, 2.1}, {0.1, 0.0, 2.2},
      {-0.1, 0.0, 5.0}, {0.0, 0.0, 5.0}, {0.1, 0.0, 5.0}},
    identity, {0.0, 0.0, 0.0}, 3, 0.35);
  assert(ambiguous_analysis.status == AssociationStatus::MULTIPLE_DEPTH_CANDIDATES);

  // 전경 6개와 배경 3개: 기존 구현은 둘 모두 거부했지만 전경은 2배 지지를 가진다.
  std::vector<Point3> foreground_with_background(6, Point3{0.0, 0.0, 2.0});
  foreground_with_background.insert(
    foreground_with_background.end(), 3, Point3{0.0, 0.0, 5.0});
  const auto supported = mobile_robot_lab_cpp::associate_instance_points(
    foreground_with_background, identity, {0.0, 0.0, 0.0});
  assert(supported && supported->point_count == 6);
  assert(supported->position_camera.z == 2.0);
  assert(!mobile_robot_lab_cpp::associate_instance_points(
      foreground_with_background, identity, {0.0, 0.0, 0.0}, 3, 0.35, 2.01));

  // 여러 작은 배경 군집도 합산해야 각각과만 비교해서 지지를 과대평가하지 않는다.
  foreground_with_background.insert(
    foreground_with_background.end(), 3, Point3{0.0, 0.0, 8.0});
  assert(!mobile_robot_lab_cpp::associate_instance_points(
      foreground_with_background, identity, {0.0, 0.0, 0.0}));

  // 큰 배경 군집을 사람으로 선택하거나 작은 전경을 무조건 채택하지 않는다.
  std::vector<Point3> background_dominant(3, Point3{0.0, 0.0, 2.0});
  background_dominant.insert(background_dominant.end(), 9, Point3{0.0, 0.0, 5.0});
  assert(!mobile_robot_lab_cpp::associate_instance_points(
      background_dominant, identity, {0.0, 0.0, 0.0}));
  for (const double ratio : {1.0, std::numeric_limits<double>::quiet_NaN()}) {
    assert(!mobile_robot_lab_cpp::associate_instance_points(
        background_dominant, identity, {0.0, 0.0, 0.0}, 3, 0.35, ratio));
  }

  const auto single_candidate = mobile_robot_lab_cpp::associate_instance_points(
    {{-0.1, 0.0, 2.0}, {0.0, 0.0, 2.1}, {0.1, 0.0, 2.2}},
    identity, {0.0, 0.0, 0.0}, 3, 0.35);
  assert(single_candidate);
  assert(single_candidate->point_count == 3);
  assert(single_candidate->position_camera.z < 2.2);

  const auto too_few = mobile_robot_lab_cpp::associate_bbox(
    {{0.0, 0.0, 2.0}}, identity, {0.0, 0.0, 0.0},
    Intrinsics{100.0, 100.0, 320.0, 240.0}, BoundingBox{310.0, 230.0, 330.0, 250.0});
  assert(!too_few);
  const auto too_few_analysis = mobile_robot_lab_cpp::analyze_instance_points(
    {{0.0, 0.0, 2.0}}, identity, {0.0, 0.0, 0.0}, 3, 0.35);
  assert(too_few_analysis.status == AssociationStatus::NO_POINTS);

  // 같은 pixel의 5m 점만 제거한다. 이웃 pixel과 허용 오차 경계 2.125m는 보존한다.
  const Intrinsics camera_info{100.0, 100.0, 80.0, 60.0};
  std::vector<Point3> visibility_points{
    {0.0, 0.0, 2.0}, {0.0, 0.0, 5.0}, {0.05, 0.0, 5.0},
    {0.0, 0.0, 2.125}, {0.0, 0.0, 2.126}, {0.0, 0.0, -1.0},
    {0.0, 0.0, 0.0}, {10.0, 0.0, 1.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},
    {0.0, 0.0, std::numeric_limits<double>::infinity()}};
  const std::vector<bool> expected{
    false, true, false, false, true, false, false, false, false, false};
  assert(mobile_robot_lab_cpp::camera_occluded_points(
      visibility_points, identity, {0.0, 0.0, 0.0}, camera_info, 160, 120, 0.125) == expected);
  std::reverse(visibility_points.begin(), visibility_points.end());
  auto reversed = mobile_robot_lab_cpp::camera_occluded_points(
    visibility_points, identity, {0.0, 0.0, 0.0}, camera_info, 160, 120, 0.125);
  std::reverse(reversed.begin(), reversed.end());
  assert(reversed == expected);

  // LiDAR zではなくCamera zで判定する。Camera前方でもLiDAR座標ではzが負になり得る。
  assert((mobile_robot_lab_cpp::camera_occluded_points(
      {{0.0, 0.0, -1.0}, {0.0, 0.0, 2.0}}, identity, {0.0, 0.0, 3.0},
      camera_info, 160, 120, 0.05) == std::vector<bool>{false, true}));

  // 3点の人＋9点の壁: 以前のassociationは拒否。同一pixelの壁だけ除外すれば人を採択。
  const auto occluded = mobile_robot_lab_cpp::camera_occluded_points(
    background_dominant, identity, {0.0, 0.0, 0.0}, camera_info, 160, 120, 0.05);
  std::vector<Point3> preserved;
  for (std::size_t index = 0; index < background_dominant.size(); ++index) {
    if (!occluded[index]) {
      preserved.push_back(background_dominant[index]);
    }
  }
  const auto visible = mobile_robot_lab_cpp::associate_instance_points(
    preserved, identity, {0.0, 0.0, 0.0});
  assert(visible && visible->point_count == 3 && visible->position_camera.z == 2.0);
  // 앞쪽 관측 근거가 없으면 벽만 있는 입력은 제외하지 못한다.
  assert((mobile_robot_lab_cpp::camera_occluded_points(
      {{0.0, 0.0, 5.0}}, identity, {0.0, 0.0, 0.0}, camera_info, 160, 120, 0.05) ==
    std::vector<bool>{false}));
  for (const double tolerance : {-0.01, std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try {
      mobile_robot_lab_cpp::camera_occluded_points(
        points, identity, {0.0, 0.0, 0.0}, camera_info, 160, 120, tolerance);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  }
}

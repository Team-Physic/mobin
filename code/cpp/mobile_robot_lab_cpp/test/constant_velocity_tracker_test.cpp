// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// Constant Velocity Kalman tracker의 예측·관측 갱신과 다인 Hungarian assignment를 검증한다.
// 합성 입력은 odom 평면의 meter 위치와 second timestamp이며 ROS runtime을 사용하지 않는다.
// 교차·관측 누락·거리/Mahalanobis gate·lifecycle·clock reset에서 ID와 covariance를 확인한다.

#include <cassert>
#include <cmath>

#include "mobile_robot_lab_cpp/constant_velocity_tracker.hpp"

int main()
{
  using mobile_robot_lab_cpp::ConstantVelocityTracker;
  using mobile_robot_lab_cpp::Detection2;
  using mobile_robot_lab_cpp::TrackState;
  using mobile_robot_lab_cpp::TrackerConfig;

  TrackerConfig config;
  config.measurement_stddev = 0.30;
  ConstantVelocityTracker tracker(config);

  tracker.update({Detection2{-0.4, 0.0}, Detection2{0.4, 0.0}}, 0.0);
  tracker.update({Detection2{-0.2, 0.0}, Detection2{0.2, 0.0}}, 0.1);
  const auto & crossing = tracker.update(
    {Detection2{0.0, 0.0}, Detection2{0.01, 0.0}}, 0.2);
  assert(crossing.size() == 2);
  assert(crossing[0].state == TrackState::kConfirmed);
  assert(crossing[1].state == TrackState::kConfirmed);

  const int first_id = crossing[0].id;
  tracker.update({}, 0.3);
  const auto & recovered = tracker.update({Detection2{0.35, 0.0}}, 0.4);
  bool recovered_first = false;
  for (const auto & track : recovered) {
    recovered_first = recovered_first || track.id == first_id;
    assert(std::isfinite(tracker.uncertainty_radius(track)));
    assert(tracker.uncertainty_radius(track) >= 0.05);
    assert(tracker.uncertainty_radius(track) <= 0.40);
  }
  assert(recovered_first);

  const auto & timed_out = tracker.update({}, 1.0);
  assert(timed_out.empty());

  ConstantVelocityTracker hard_gate(config);
  hard_gate.update({Detection2{0.0, 0.0}}, 0.0);
  const auto & far = hard_gate.update({Detection2{0.76, 0.0}}, 0.1);
  assert(far.size() == 2);

  hard_gate.update({}, -1.0);
  assert(hard_gate.tracks().empty());
}

// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#ifndef MOBILE_ROBOT_LAB_CPP__SEMANTIC_SAFETY_HPP_
#define MOBILE_ROBOT_LAB_CPP__SEMANTIC_SAFETY_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mobile_robot_lab_cpp
{

struct Vector2
{
  double x{0.0};
  double y{0.0};
};

struct VelocityCommand
{
  double linear{0.0};
  double angular{0.0};
};

struct TrackedPerson2
{
  int id;
  Vector2 position;
  Vector2 velocity;
  double uncertainty_radius;
};

struct RobotMotion2
{
  Vector2 position;
  Vector2 velocity;
};

struct DirectionalRanges
{
  std::optional<double> front;
  std::optional<double> rear;
  std::optional<double> left;
  std::optional<double> right;
};

struct SafetyConfig
{
  double caution_ttc{4.0};
  double danger_ttc{2.0};
  double prediction_horizon{6.0};
  double caution_scale{0.5};
  double robot_radius{0.30};
  double person_radius{0.35};
  double safety_margin{0.20};
  double front_emergency_distance{0.60};
  double rear_emergency_distance{0.40};
  double side_emergency_distance{0.45};
  double tracking_stale_speed{0.15};
  double stable_safe_duration{1.0};
};

enum class SafetyMode {kSafe, kCaution, kDanger, kEmergency, kInputStale, kRecoveryHold};

struct ClosestApproach
{
  double time;
  double distance;
};

struct SafetyResult
{
  VelocityCommand command;
  SafetyMode mode{SafetyMode::kSafe};
  int selected_track_id{-1};
  double selected_ttc{std::numeric_limits<double>::infinity()};
  double selected_distance{std::numeric_limits<double>::infinity()};
  double selected_safe_distance{0.0};
  std::string reason{"safe"};
};

inline ClosestApproach closest_approach(
  const RobotMotion2 & robot, const TrackedPerson2 & person,
  const double horizon)
{
  const Vector2 relative_position{
    person.position.x - robot.position.x,
    person.position.y - robot.position.y};
  const Vector2 relative_velocity{
    person.velocity.x - robot.velocity.x,
    person.velocity.y - robot.velocity.y};
  const double speed_squared = relative_velocity.x * relative_velocity.x +
    relative_velocity.y * relative_velocity.y;
  double time = 0.0;
  if (speed_squared > 1e-9) {
    time = std::clamp(
      -(relative_position.x * relative_velocity.x +
      relative_position.y * relative_velocity.y) / speed_squared,
      0.0, horizon);
  }
  const double x = relative_position.x + relative_velocity.x * time;
  const double y = relative_position.y + relative_velocity.y * time;
  return {time, std::hypot(x, y)};
}

inline bool lidar_emergency(
  const VelocityCommand & command, const DirectionalRanges & ranges,
  const SafetyConfig & config)
{
  if (command.linear > 0.0 &&
    (!ranges.front || *ranges.front <= config.front_emergency_distance))
  {
    return true;
  }
  if (command.linear < 0.0 &&
    (!ranges.rear || *ranges.rear <= config.rear_emergency_distance))
  {
    return true;
  }
  if (command.angular > 0.0 &&
    (!ranges.left || *ranges.left <= config.side_emergency_distance))
  {
    return true;
  }
  if (command.angular < 0.0 &&
    (!ranges.right || *ranges.right <= config.side_emergency_distance))
  {
    return true;
  }
  return false;
}

class SemanticSafety
{
public:
  explicit SemanticSafety(SafetyConfig config = {})
  : config_(config)
  {
    const bool valid = config_.caution_ttc > config_.danger_ttc &&
      config_.danger_ttc >= 0.0 && config_.prediction_horizon >= config_.caution_ttc &&
      0.0 <= config_.caution_scale && config_.caution_scale <= 1.0 &&
      config_.robot_radius > 0.0 && config_.person_radius > 0.0 &&
      config_.safety_margin >= 0.0 && config_.front_emergency_distance > 0.0 &&
      config_.rear_emergency_distance > 0.0 && config_.side_emergency_distance > 0.0 &&
      config_.tracking_stale_speed >= 0.0 && config_.stable_safe_duration >= 0.0;
    if (!valid) {
      throw std::invalid_argument("invalid semantic safety configuration");
    }
  }

  SafetyResult evaluate(
    const VelocityCommand & requested, const RobotMotion2 & robot,
    const std::vector<TrackedPerson2> & persons, const DirectionalRanges & ranges,
    const bool scan_stale, const bool odom_stale, const bool tracking_stale,
    const double now)
  {
    SafetyResult result;
    result.command = requested;
    if (scan_stale || odom_stale) {
      return stop(result, SafetyMode::kInputStale, scan_stale ? "scan_stale" : "odom_stale", now);
    }
    if (lidar_emergency(requested, ranges, config_)) {
      return stop(result, SafetyMode::kEmergency, "lidar_emergency", now);
    }

    SafetyMode risk_mode = SafetyMode::kSafe;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & person : persons) {
      const auto cpa = closest_approach(robot, person, config_.prediction_horizon);
      const double safe_distance = config_.robot_radius + config_.person_radius +
        person.uncertainty_radius + config_.safety_margin;
      if (cpa.distance > safe_distance) {
        continue;
      }
      const SafetyMode mode = cpa.time <= config_.danger_ttc ?
        SafetyMode::kDanger :
        (cpa.time <= config_.caution_ttc ? SafetyMode::kCaution : SafetyMode::kSafe);
      const double score = cpa.time + cpa.distance / std::max(safe_distance, 1e-6);
      if (mode > risk_mode || (mode == risk_mode && score < best_score)) {
        risk_mode = mode;
        best_score = score;
        result.selected_track_id = person.id;
        result.selected_ttc = cpa.time;
        result.selected_distance = cpa.distance;
        result.selected_safe_distance = safe_distance;
      }
    }

    if (risk_mode == SafetyMode::kDanger) {
      return stop(result, SafetyMode::kDanger, "person_cpa_danger", now);
    }
    if (risk_mode == SafetyMode::kCaution) {
      result.command.linear *= config_.caution_scale;
      result.command.angular *= config_.caution_scale;
      result.mode = SafetyMode::kCaution;
      result.reason = "person_cpa_caution";
      safe_since_.reset();
      return result;
    }

    if (tracking_stale) {
      const double magnitude = std::abs(result.command.linear);
      if (magnitude > config_.tracking_stale_speed && magnitude > 0.0) {
        const double scale = config_.tracking_stale_speed / magnitude;
        result.command.linear *= scale;
        result.command.angular *= scale;
      }
      result.mode = SafetyMode::kCaution;
      result.reason = "tracking_stale_speed_limit";
      safe_since_.reset();
      return result;
    }

    if (latched_stop_) {
      if (!safe_since_) {
        safe_since_ = now;
      }
      if (now - *safe_since_ < config_.stable_safe_duration) {
        result.command = {};
        result.mode = SafetyMode::kRecoveryHold;
        result.reason = "stable_safe_hold";
        return result;
      }
      latched_stop_ = false;
      safe_since_.reset();
    }
    return result;
  }

private:
  SafetyResult stop(
    SafetyResult result, const SafetyMode mode, std::string reason, const double)
  {
    result.command = {};
    result.mode = mode;
    result.reason = std::move(reason);
    latched_stop_ = true;
    safe_since_.reset();
    return result;
  }

  SafetyConfig config_;
  bool latched_stop_{false};
  std::optional<double> safe_since_;
};

}  // namespace mobile_robot_lab_cpp

#endif  // MOBILE_ROBOT_LAB_CPP__SEMANTIC_SAFETY_HPP_

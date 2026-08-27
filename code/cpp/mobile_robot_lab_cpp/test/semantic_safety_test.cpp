// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <cassert>
#include <cmath>

#include "mobile_robot_lab_cpp/semantic_safety.hpp"

int main()
{
  using namespace mobile_robot_lab_cpp;
  const DirectionalRanges clear_ranges{2.0, 2.0, 2.0, 2.0};
  const VelocityCommand requested{0.4, 0.2};
  const RobotMotion2 robot{{0.0, 0.0}, {1.0, 0.0}};

  SemanticSafety danger_policy;
  const auto danger = danger_policy.evaluate(
    requested, robot, {{1, {2.0, 0.0}, {0.0, 0.0}, 0.05}},
    clear_ranges, false, false, false, 0.0);
  assert(danger.mode == SafetyMode::kDanger);
  assert(danger.command.linear == 0.0);

  const auto holding = danger_policy.evaluate(
    requested, robot, {}, clear_ranges, false, false, false, 0.99);
  assert(holding.mode == SafetyMode::kRecoveryHold);
  const auto released = danger_policy.evaluate(
    requested, robot, {}, clear_ranges, false, false, false, 2.0);
  assert(released.mode == SafetyMode::kSafe);

  SemanticSafety caution_policy;
  const auto caution = caution_policy.evaluate(
    requested, robot, {{2, {3.0, 0.7}, {0.0, 0.0}, 0.10}},
    clear_ranges, false, false, false, 0.0);
  assert(caution.mode == SafetyMode::kCaution);
  assert(std::abs(caution.command.linear - 0.2) < 1e-9);
  assert(std::abs(caution.command.angular - 0.1) < 1e-9);

  SemanticSafety stale_policy;
  const auto stale = stale_policy.evaluate(
    requested, robot, {}, clear_ranges, false, false, true, 0.0);
  assert(stale.mode == SafetyMode::kCaution);
  assert(std::abs(stale.command.linear - 0.15) < 1e-9);
  assert(std::abs(stale.command.angular - 0.075) < 1e-9);

  SemanticSafety emergency_policy;
  const DirectionalRanges blocked_front{0.5, 2.0, 2.0, 2.0};
  const auto emergency = emergency_policy.evaluate(
    requested, robot, {}, blocked_front, false, false, false, 0.0);
  assert(emergency.mode == SafetyMode::kEmergency);

  SemanticSafety input_policy;
  const auto input_stale = input_policy.evaluate(
    requested, robot, {}, clear_ranges, true, false, false, 0.0);
  assert(input_stale.mode == SafetyMode::kInputStale);
}

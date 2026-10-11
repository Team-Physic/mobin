// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// odom frame의 사람 측정을 지속적인 track ID, 위치, 속도, covariance로 추적한다.
// 입력은 timestamp가 붙은 2D 위치 측정이며 Hungarian assignment로 기존 track과 연결한다.
// Constant Velocity Kalman Filter가 측정 사이 상태를 예측하고 sensor noise를 완화한다.
// 위치·거리는 meter, 속도는 m/s, timestamp는 second 단위를 사용한다.

#ifndef MOBILE_ROBOT_LAB_CPP__CONSTANT_VELOCITY_TRACKER_HPP_
#define MOBILE_ROBOT_LAB_CPP__CONSTANT_VELOCITY_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace mobile_robot_lab_cpp
{

struct Detection2
{
  double x;
  double y;
};

enum class TrackState : std::uint8_t {kTentative = 0, kConfirmed = 1, kLost = 2};

struct PersonTrack
{
  int id;
  TrackState state{TrackState::kTentative};
  Eigen::Vector4d estimate{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};
  double state_stamp{0.0};
  double last_measurement_stamp{0.0};
  std::deque<bool> recent_hits;
};

struct TrackerConfig
{
  double euclidean_gate{0.75};
  double mahalanobis_gate_squared{5.991};
  double max_age{0.5};
  double measurement_stddev{0.20};
  double acceleration_stddev{1.0};
  double initial_velocity_stddev{1.0};
  double uncertainty_scale{2.447};
  double uncertainty_min{0.05};
  double uncertainty_max{0.40};
  std::size_t confirmation_window{5};
  std::size_t confirmation_hits{3};
};

// 모든 row가 서로 다른 column을 선택하면서 전체 비용 합이 최소인 row→column 대응을 구한다.
// Tracker에서는 row가 예측 track, 실제 detection column과 dummy column이 매칭 후보가 된다.
inline std::vector<int> hungarian_assignment(const std::vector<std::vector<double>> & costs)
{
  if (costs.empty()) {
    return {};
  }
  const std::size_t rows = costs.size();
  const std::size_t columns = costs.front().size();
  if (columns < rows || columns == 0) {
    throw std::invalid_argument("Hungarian assignment requires columns >= rows");
  }
  for (const auto & row : costs) {
    if (row.size() != columns) {
      throw std::invalid_argument("cost matrix must be rectangular");
    }
  }

  std::vector<double> row_potential(rows + 1, 0.0);
  std::vector<double> column_potential(columns + 1, 0.0);
  std::vector<std::size_t> matched_row(columns + 1, 0);
  std::vector<std::size_t> path(columns + 1, 0);
  for (std::size_t row = 1; row <= rows; ++row) {
    matched_row[0] = row;
    std::size_t current_column = 0;
    std::vector<double> minimum(columns + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(columns + 1, false);
    do {
      used[current_column] = true;
      const std::size_t current_row = matched_row[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0;
      for (std::size_t column = 1; column <= columns; ++column) {
        if (used[column]) {
          continue;
        }
        const double reduced = costs[current_row - 1][column - 1] -
          row_potential[current_row] - column_potential[column];
        if (reduced < minimum[column]) {
          minimum[column] = reduced;
          path[column] = current_column;
        }
        if (minimum[column] < delta) {
          delta = minimum[column];
          next_column = column;
        }
      }
      for (std::size_t column = 0; column <= columns; ++column) {
        if (used[column]) {
          row_potential[matched_row[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      current_column = next_column;
    } while (matched_row[current_column] != 0);

    do {
      const std::size_t previous_column = path[current_column];
      matched_row[current_column] = matched_row[previous_column];
      current_column = previous_column;
    } while (current_column != 0);
  }

  std::vector<int> assignment(rows, -1);
  for (std::size_t column = 1; column <= columns; ++column) {
    if (matched_row[column] != 0) {
      assignment[matched_row[column] - 1] = static_cast<int>(column - 1);
    }
  }
  return assignment;
}

class ConstantVelocityTracker
{
public:
  explicit ConstantVelocityTracker(TrackerConfig config = {})
  : config_(std::move(config))
  {
    const bool valid = config_.euclidean_gate > 0.0 &&
      config_.mahalanobis_gate_squared > 0.0 && config_.max_age >= 0.0 &&
      config_.measurement_stddev > 0.0 && config_.acceleration_stddev >= 0.0 &&
      config_.initial_velocity_stddev > 0.0 && config_.uncertainty_scale > 0.0 &&
      config_.uncertainty_min >= 0.0 &&
      config_.uncertainty_max >= config_.uncertainty_min &&
      config_.confirmation_window > 0 && config_.confirmation_hits > 0 &&
      config_.confirmation_hits <= config_.confirmation_window;
    if (!valid) {
      throw std::invalid_argument("invalid tracker configuration");
    }
  }

  const std::vector<PersonTrack> & update(
    const std::vector<Detection2> & detections, const double stamp)
  {
    if (!std::isfinite(stamp)) {
      throw std::invalid_argument("tracker stamp must be finite");
    }
    if (last_update_stamp_ && stamp < *last_update_stamp_) {
      // rosbag 반복 재생처럼 시간이 역행하면 이전 상태와 새 측정을 섞지 않는다.
      reset();
    }
    // 모든 track을 현재 측정 시각까지 먼저 예측해야 같은 시각의 detection과 비교할 수 있다.
    for (auto & track : tracks_) {
      predict(track, stamp);
    }

    // Hungarian 결과를 한 번만 적용해 하나의 detection이 여러 track을 갱신하지 못하게 한다.
    const auto matches = assign(detections);
    std::vector<bool> matched_detections(detections.size(), false);
    for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
      const int detection_index = matches[track_index];
      if (detection_index >= 0) {
        update_measurement(
          tracks_[track_index], detections[static_cast<std::size_t>(detection_index)], stamp);
        matched_detections[static_cast<std::size_t>(detection_index)] = true;
      } else {
        // lost는 삭제가 아니라 현재 frame에서 측정과 연결되지 않았다는 상태다.
        add_hit(tracks_[track_index], false);
        tracks_[track_index].state = TrackState::kLost;
      }
    }

    // 마지막 실제 측정 이후 max_age가 지난 예측 track만 제거한다.
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this, stamp](const PersonTrack & track) {
          return stamp - track.last_measurement_stamp > config_.max_age;
        }),
      tracks_.end());

    for (std::size_t index = 0; index < detections.size(); ++index) {
      if (!matched_detections[index]) {
        // 기존 track에 속하지 않는 측정만 새 tentative ID를 만든다.
        tracks_.push_back(make_track(detections[index], stamp));
      }
    }
    last_update_stamp_ = stamp;
    return tracks_;
  }

  const std::vector<PersonTrack> & tracks() const {return tracks_;}

  double uncertainty_radius(const PersonTrack & track) const
  {
    // 2D covariance의 가장 긴 고유축에 95% chi-square 계수를 적용해 보수적인 원 반경을 만든다.
    const Eigen::Matrix2d position_covariance = track.covariance.topLeftCorner<2, 2>();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(position_covariance);
    const double largest = std::max(0.0, solver.eigenvalues().maxCoeff());
    return std::clamp(
      config_.uncertainty_scale * std::sqrt(largest),
      config_.uncertainty_min, config_.uncertainty_max);
  }

  void reset()
  {
    tracks_.clear();
    next_id_ = 1;
    last_update_stamp_.reset();
  }

private:
  // 새 track은 odom 위치[m]에서 시작하며 속도 정보가 없으므로 0 m/s와 큰 초기 분산을 둔다.
  PersonTrack make_track(const Detection2 & detection, const double stamp)
  {
    PersonTrack track;
    track.id = next_id_++;
    track.estimate << detection.x, detection.y, 0.0, 0.0;
    track.covariance.setZero();
    const double position_variance = config_.measurement_stddev * config_.measurement_stddev;
    const double velocity_variance =
      config_.initial_velocity_stddev * config_.initial_velocity_stddev;
    track.covariance.diagonal() <<
      position_variance, position_variance, velocity_variance, velocity_variance;
    track.state_stamp = stamp;
    track.last_measurement_stamp = stamp;
    add_hit(track, true);
    return track;
  }

  void predict(PersonTrack & track, const double stamp) const
  {
    const double dt = stamp - track.state_stamp;
    if (dt <= 0.0) {
      track.state_stamp = stamp;
      return;
    }
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    // Constant Velocity model: p(t+dt)=p(t)+v(t)dt, v(t+dt)=v(t).
    transition(0, 2) = dt;
    transition(1, 3) = dt;
    const double variance = config_.acceleration_stddev * config_.acceleration_stddev;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Eigen::Matrix4d process_noise;
    // 알 수 없는 가속도를 white noise로 보고 시간 간격에 따라 position/velocity 불확실성을 키운다.
    process_noise <<
      dt4 / 4.0, 0.0, dt3 / 2.0, 0.0,
      0.0, dt4 / 4.0, 0.0, dt3 / 2.0,
      dt3 / 2.0, 0.0, dt2, 0.0,
      0.0, dt3 / 2.0, 0.0, dt2;
    track.estimate = transition * track.estimate;
    track.covariance = transition * track.covariance * transition.transpose() +
      variance * process_noise;
    track.state_stamp = stamp;
  }

  std::vector<int> assign(const std::vector<Detection2> & detections) const
  {
    if (tracks_.empty()) {
      return {};
    }
    if (detections.empty()) {
      return std::vector<int>(tracks_.size(), -1);
    }
    const std::size_t size = tracks_.size() + detections.size();
    // Dummy row/column을 추가해야 강제로 잘못 매칭하는 대신 unmatched를 선택할 수 있다.
    const double unmatched_cost = config_.mahalanobis_gate_squared;
    const double invalid_cost = unmatched_cost * 1000.0;
    std::vector<std::vector<double>> costs(size, std::vector<double>(size, 0.0));
    for (std::size_t row = 0; row < tracks_.size(); ++row) {
      std::fill(costs[row].begin(), costs[row].begin() + detections.size(), invalid_cost);
      std::fill(costs[row].begin() + detections.size(), costs[row].end(), unmatched_cost);
      const Eigen::Matrix2d innovation_covariance =
        tracks_[row].covariance.topLeftCorner<2, 2>() + measurement_covariance();
      const Eigen::Matrix2d inverse = innovation_covariance.inverse();
      for (std::size_t column = 0; column < detections.size(); ++column) {
        const Eigen::Vector2d innovation{
          detections[column].x - tracks_[row].estimate[0],
          detections[column].y - tracks_[row].estimate[1]};
        const double euclidean = innovation.norm();
        // Mahalanobis 거리는 covariance가 큰 방향의 오차를 덜 벌점화한다. 절대거리 gate도 함께 둬
        // 불확실성이 큰 track이 멀리 있는 사람을 가로채지 못하게 한다.
        const double mahalanobis_squared = innovation.dot(inverse * innovation);
        if (euclidean <= config_.euclidean_gate &&
          mahalanobis_squared <= config_.mahalanobis_gate_squared)
        {
          costs[row][column] = mahalanobis_squared;
        }
      }
    }
    for (std::size_t row = tracks_.size(); row < size; ++row) {
      std::fill(costs[row].begin(), costs[row].begin() + detections.size(), unmatched_cost);
    }

    const auto assignment = hungarian_assignment(costs);
    std::vector<int> matches(tracks_.size(), -1);
    for (std::size_t row = 0; row < tracks_.size(); ++row) {
      const int column = assignment[row];
      if (column >= 0 && static_cast<std::size_t>(column) < detections.size() &&
        costs[row][static_cast<std::size_t>(column)] <= config_.mahalanobis_gate_squared)
      {
        matches[row] = column;
      }
    }
    return matches;
  }

  void update_measurement(
    PersonTrack & track, const Detection2 & detection, const double stamp)
  {
    // 측정은 position(px, py)만 제공하므로 observation matrix가 velocity 성분은 직접 관측하지 않는다.
    Eigen::Matrix<double, 2, 4> observation = Eigen::Matrix<double, 2, 4>::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;
    const Eigen::Vector2d measurement{detection.x, detection.y};
    const Eigen::Vector2d innovation = measurement - observation * track.estimate;
    const Eigen::Matrix2d innovation_covariance =
      observation * track.covariance * observation.transpose() + measurement_covariance();
    const Eigen::Matrix<double, 4, 2> gain =
      track.covariance * observation.transpose() * innovation_covariance.inverse();
    track.estimate += gain * innovation;
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d correction = identity - gain * observation;
    // Joseph form은 반복 update 뒤에도 covariance의 대칭·양의 준정부호를 보존한다.
    track.covariance = correction * track.covariance * correction.transpose() +
      gain * measurement_covariance() * gain.transpose();
    track.last_measurement_stamp = stamp;
    add_hit(track, true);
    // 최근 confirmation_window 안의 hit 수로 일시적인 false positive의 즉시 확정을 막는다.
    if (confirmed(track)) {
      track.state = TrackState::kConfirmed;
    } else {
      track.state = TrackState::kTentative;
    }
  }

  void add_hit(PersonTrack & track, const bool hit) const
  {
    track.recent_hits.push_back(hit);
    while (track.recent_hits.size() > config_.confirmation_window) {
      track.recent_hits.pop_front();
    }
  }

  bool confirmed(const PersonTrack & track) const
  {
    return std::count(track.recent_hits.begin(), track.recent_hits.end(), true) >=
           static_cast<std::ptrdiff_t>(config_.confirmation_hits);
  }

  Eigen::Matrix2d measurement_covariance() const
  {
    return Eigen::Matrix2d::Identity() *
           config_.measurement_stddev * config_.measurement_stddev;
  }

  TrackerConfig config_;
  std::vector<PersonTrack> tracks_;
  int next_id_{1};
  std::optional<double> last_update_stamp_;
};

}  // namespace mobile_robot_lab_cpp

#endif  // MOBILE_ROBOT_LAB_CPP__CONSTANT_VELOCITY_TRACKER_HPP_

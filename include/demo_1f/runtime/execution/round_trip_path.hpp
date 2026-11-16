#pragma once
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace demo_1f {
// Only complete position/velocity/acceleration knots with quintic interpolation are supported.
inline void validate_round_trip_path(
    const moveit_msgs::msg::RobotTrajectory & forward,
    const std::vector<std::string> & controlled) {
  const auto & source = forward.joint_trajectory;
  const std::set<std::string> names(source.joint_names.begin(), source.joint_names.end());
  const std::set<std::string> expected(controlled.begin(), controlled.end());
  if (names.empty() || names != expected || names.size() != source.joint_names.size() ||
      source.points.size() < 2 || !forward.multi_dof_joint_trajectory.joint_names.empty() ||
      !forward.multi_dof_joint_trajectory.points.empty()) {
    throw std::runtime_error("Invalid round-trip joint group or point count");
  }
  const auto nanos = [](const builtin_interfaces::msg::Duration & time) -> int64_t {
    if (time.sec < 0 || time.nanosec >= 1000000000u) {
      throw std::runtime_error("Invalid round-trip timestamp");
    }
    return int64_t(time.sec) * 1000000000LL + time.nanosec;
  };
  int64_t previous = -1;
  for (const auto & point : source.points) {
    const auto time = nanos(point.time_from_start);
    if (time <= previous || (previous == -1 && time != 0) || !point.effort.empty()) {
      throw std::runtime_error("Round-trip needs increasing times from zero and no effort");
    }
    previous = time;
    for (const auto * values : {&point.positions, &point.velocities, &point.accelerations}) {
      if (values->size() != names.size() ||
          !std::all_of(values->begin(), values->end(), [](double v) { return std::isfinite(v); })) {
        throw std::runtime_error("Round-trip needs finite, complete quintic knots");
      }
    }
  }
  for (const auto * point : {&source.points.front(), &source.points.back()}) {
    for (const auto * values : {&point->velocities, &point->accelerations}) {
      if (std::any_of(values->begin(), values->end(),
              [](double v) { return std::abs(v) > 1e-8; })) {
        throw std::runtime_error("Round-trip endpoints must be at rest");
      }
    }
  }
}

inline moveit_msgs::msg::RobotTrajectory reverse_round_trip_path(
    const moveit_msgs::msg::RobotTrajectory & forward,
    const std::vector<std::string> & controlled) {
  validate_round_trip_path(forward, controlled);
  const auto nanos = [](const builtin_interfaces::msg::Duration & time) -> int64_t {
    return int64_t(time.sec) * 1000000000LL + time.nanosec;
  };
  const auto total = nanos(forward.joint_trajectory.points.back().time_from_start);
  auto backward = forward;
  auto & result = backward.joint_trajectory;
  result.header.stamp = builtin_interfaces::msg::Time{};
  std::reverse(result.points.begin(), result.points.end());
  for (auto & point : result.points) {
    const int64_t time = total - nanos(point.time_from_start);
    point.time_from_start.sec = static_cast<int32_t>(time / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(time % 1000000000LL);
    for (auto & value : point.velocities) value = -value;
  }
  return backward;
}
} // namespace demo_1f

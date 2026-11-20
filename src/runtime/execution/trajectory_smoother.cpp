#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <moveit/utils/logger.hpp>
#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>
#include <ruckig/ruckig.hpp>
#include <set>
#include <stdexcept>

namespace demo_1f {
void initialize_moveit_logging() {
  const int level = rcutils_logging_get_default_logger_level();
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
  // No ROS context exists yet, so the logger retains no hidden node. Restore
  // normal logging immediately; only the expected fallback warnings are muted.
  (void)moveit::getLogger("demo_1f");
  rcutils_logging_set_default_logger_level(level);
}

namespace {
using Positions = std::vector<double>;
using Point = trajectory_msgs::msg::JointTrajectoryPoint;

double seconds(const builtin_interfaces::msg::Duration & time) {
  return rclcpp::Duration(time).seconds();
}

double distance(const Positions & first, const Positions & second) {
  double result = 0;
  for (size_t j = 0; j < first.size(); ++j) {
    result = std::max(result, std::abs(first[j] - second[j]));
  }
  return result;
}

std::vector<size_t> simplify(const std::vector<Positions> & positions, double tolerance) {
  std::vector<size_t> indices;
  std::function<void(size_t, size_t)> visit = [&](size_t first, size_t last) {
    double norm = 0;
    for (size_t j = 0; j < positions[first].size(); ++j) {
      norm += std::pow(positions[last][j] - positions[first][j], 2);
    }
    double largest = tolerance;
    size_t split = first;
    for (size_t i = first + 1; i < last; ++i) {
      double projection = 0;
      for (size_t j = 0; j < positions[first].size(); ++j) {
        projection +=
            (positions[i][j] - positions[first][j]) * (positions[last][j] - positions[first][j]);
      }
      projection = norm > 0 ? std::clamp(projection / norm, 0.0, 1.0) : 0;
      double error = 0;
      for (size_t j = 0; j < positions[first].size(); ++j) {
        error = std::max(error,
            std::abs(positions[i][j] - positions[first][j] -
                     projection * (positions[last][j] - positions[first][j])));
      }
      if (error > largest) {
        largest = error;
        split = i;
      }
    }
    if (split != first) {
      visit(first, split);
      visit(split, last);
    } else {
      indices.push_back(last);
    }
  };
  indices.push_back(0);
  if (positions.size() > 1) {
    visit(0, positions.size() - 1);
  }
  return indices;
}

struct Sample {
  double position, velocity, acceleration, jerk;
};

Sample quintic(const Point & first, const Point & last, size_t joint, double u) {
  const double dt = seconds(last.time_from_start) - seconds(first.time_from_start);
  const double c0 = first.positions[joint];
  const double c1 = first.velocities[joint] * dt;
  const double c2 = 0.5 * first.accelerations[joint] * dt * dt;
  const double d = last.positions[joint] - c0 - c1 - c2;
  const double v = last.velocities[joint] * dt - c1 - 2 * c2;
  const double a = last.accelerations[joint] * dt * dt - 2 * c2;
  const double c3 = 10 * d - 4 * v + a / 2;
  const double c4 = -15 * d + 7 * v - a;
  const double c5 = 6 * d - 3 * v + a / 2;
  return {c0 + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5)))),
      (c1 + u * (2 * c2 + u * (3 * c3 + u * (4 * c4 + u * 5 * c5)))) / dt,
      (2 * c2 + u * (6 * c3 + u * (12 * c4 + u * 20 * c5))) / (dt * dt),
      (6 * c3 + u * (24 * c4 + u * 60 * c5)) / (dt * dt * dt)};
}

struct TcpLine {
  std::string link;
  Eigen::Vector3d start, end;
  double off{0}, drop{0}; // Largest seen over the whole segment.
};

std::string millimeters(double meters) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f", meters * 1000);
  return text;
}

// Track distance from the straight start-end segment and depth below the lower end.
void measure_tcp_line(TcpLine & line, const Eigen::Vector3d & tcp) {
  const Eigen::Vector3d span = line.end - line.start;
  const double length = span.squaredNorm();
  const double u = length > 0 ? std::clamp((tcp - line.start).dot(span) / length, 0.0, 1.0) : 0.0;
  line.off = std::max(line.off, (tcp - (line.start + u * span)).norm());
  line.drop = std::max(line.drop, std::min(line.start.z(), line.end.z()) - tcp.z());
}

// After the whole segment: reject with both maxima so tolerances can be tuned from one log.
void check_tcp_line(const Step & step, const TcpLine & line) {
  if ((step.tcp_line_tolerance > 0 && line.off > step.tcp_line_tolerance) ||
      (step.tcp_drop_tolerance > 0 && line.drop > step.tcp_drop_tolerance)) {
    throw std::runtime_error(step.name + ": smoothed TCP leaves the straight line by up to " +
        millimeters(line.off) + " mm (limit " + millimeters(step.tcp_line_tolerance) +
        " mm) and dips up to " + millimeters(line.drop) + " mm below its lower end (limit " +
        millimeters(step.tcp_drop_tolerance) + " mm)");
  }
}
} // namespace

std::map<std::string, double> held_arm_targets(const Step & step, const Config & config) {
  if (step.hold_arm.empty()) return {};
  const auto found = config.arms.find(step.hold_arm);
  if (step.type != "dual_joint" || found == config.arms.end() ||
      (step.hold_arm != "left" && step.hold_arm != "right") ||
      !std::isfinite(step.hold_tolerance) || step.hold_tolerance <= 0 || step.hold_tolerance > .01) {
    throw std::runtime_error("Invalid held-arm corridor: " + step.name);
  }
  const auto & target = step.hold_arm == "left" ? step.left_target : step.right_target;
  if (target.size() != found->second.joints.size() || target.empty()) {
    throw std::runtime_error("Incomplete held-arm goal: " + step.name);
  }
  std::map<std::string, double> result;
  for (size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i])) throw std::runtime_error("Nonfinite held-arm goal");
    result.emplace(found->second.joints[i], target[i]);
  }
  return result;
}

void validate_held_arm_path(const trajectory_msgs::msg::JointTrajectory & path,
    const Step & step, const Config & config) {
  const auto targets = held_arm_targets(step, config);
  if (targets.empty()) return;
  if (path.points.empty()) throw std::runtime_error("Empty held-arm path");
  for (const auto & [name, target] : targets) {
    const auto found = std::find(path.joint_names.begin(), path.joint_names.end(), name);
    if (found == path.joint_names.end()) throw std::runtime_error("Held-arm path lacks " + name);
    const auto index = found - path.joint_names.begin();
    for (const auto & point : path.points) {
      const auto value = point.positions.at(index);
      if (!std::isfinite(value) || std::abs(value - target) > step.hold_tolerance + 1e-10) {
        throw std::runtime_error("Planned held arm leaves corridor: " + step.name + " / " + name +
            " offset=" + std::to_string(value - target) + " limit=" + std::to_string(step.hold_tolerance));
      }
    }
  }
}

MotionLimits effective_limits(const Config & config,
    const std::string & name,
    const Step & step,
    const moveit::core::RobotModelConstPtr & model) {
  MotionLimits limits;
  if (config.motion_limits.count(name)) {
    limits = config.motion_limits.at(name);
  }
  if (model) {
    const auto & names = model->getVariableNames();
    if (std::find(names.begin(), names.end(), name) == names.end()) {
      throw std::runtime_error("Unknown trajectory joint: " + name);
    }
    const auto & bounds = model->getVariableBounds(name);
    if (bounds.velocity_bounded_) {
      limits.velocity = std::min(limits.velocity, bounds.max_velocity_);
    }
  }
  limits.velocity *= step.velocity_scale > 0 ? step.velocity_scale : config.velocity_scale;
  limits.acceleration *=
      step.acceleration_scale > 0 ? step.acceleration_scale : config.acceleration_scale;
  if (!std::isfinite(limits.velocity) || !std::isfinite(limits.acceleration) ||
      !std::isfinite(limits.jerk) || limits.velocity <= 0 || limits.acceleration <= 0 ||
      limits.jerk <= 0) {
    throw std::runtime_error("Invalid motion limits: " + name);
  }
  return limits;
}

SmoothPath smooth_segments(const std::vector<moveit_msgs::msg::RobotTrajectory> & segments,
    const std::vector<Step> & steps,
    const Config & config,
    const moveit::core::RobotModelConstPtr & model) {
  if (segments.empty() || segments.size() != steps.size()) {
    throw std::runtime_error("Missing planned motion segments");
  }
  SmoothPath output;
  auto & trajectory = output.trajectory.joint_trajectory;
  trajectory.joint_names = segments.front().joint_trajectory.joint_names;
  const size_t dofs = trajectory.joint_names.size();
  if (std::set<std::string>(trajectory.joint_names.begin(), trajectory.joint_names.end()).size() !=
      dofs) {
    throw std::runtime_error("Duplicate planned joint names");
  }
  if (dofs == 0) {
    throw std::runtime_error("Empty planned joint list");
  }
  std::vector<Positions> positions;
  std::vector<size_t> owners, boundaries;
  Positions planned_endpoint;
  std::set<std::string> previous_held_joints;
  for (size_t segment = 0; segment < segments.size(); ++segment) {
    const auto & source = segments[segment].joint_trajectory;
    validate_held_arm_path(source, steps[segment], config);
    if (source.points.empty() || source.joint_names.size() != dofs ||
        !segments[segment].multi_dof_joint_trajectory.points.empty()) {
      throw std::runtime_error("Invalid coordinated trajectory");
    }
    std::vector<Positions> reordered;
    for (const auto & point : source.points) {
      if (point.positions.size() != dofs) {
        throw std::runtime_error("Malformed planned point");
      }
      Positions values;
      for (const auto & name : trajectory.joint_names) {
        const auto found = std::find(source.joint_names.begin(), source.joint_names.end(), name);
        if (found == source.joint_names.end()) {
          throw std::runtime_error("Joint lists differ between segments");
        }
        const double value = point.positions[found - source.joint_names.begin()];
        if (!std::isfinite(value)) {
          throw std::runtime_error("Nonfinite planned position");
        }
        values.push_back(value);
      }
      reordered.push_back(std::move(values));
    }
    if (!planned_endpoint.empty() && distance(planned_endpoint, reordered.front()) > 1e-6) {
      throw std::runtime_error("Discontinuous planning start between segments");
    }
    planned_endpoint = reordered.back();
    const auto & step = steps[segment];
    if (step.endpoints_only && reordered.size() > 2) {
      reordered = {reordered.front(), reordered.back()};
    }
    std::set<std::string> held_joints;
    for (const auto & [side, arm] : config.arms) {
      if (step.type == "joint_gripper" && side != step.arm) {
        held_joints.insert(arm.joints.begin(), arm.joints.end());
      }
      if (step.type == "dual_joint" || side != step.arm) {
        held_joints.insert(arm.gripper_joint);
      }
    }
    for (size_t joint = 0; joint < dofs; ++joint) {
      const auto & name = trajectory.joint_names[joint];
      if (held_joints.count(name)) {
        const double planned_start = reordered.front()[joint];
        const double held_position = positions.empty() ? planned_start : positions.back()[joint];
        for (auto & point : reordered) {
          if (std::abs(point[joint] - planned_start) > std::min(step.tolerance, 0.001) + 1e-8) {
            throw std::runtime_error("Planned path moves a held joint: " + name);
          }
          // Planner tolerances allow small drift. Remove it before blending so
          // held joints stay constant; the modified path is collision-checked below.
          point[joint] = held_position;
        }
      } else if (!positions.empty() && previous_held_joints.count(name)) {
        // The next segment starts at the corrected held endpoint, then may move.
        reordered.front()[joint] = positions.back()[joint];
      }
    }
    previous_held_joints = std::move(held_joints);
    for (const auto index : simplify(reordered, config.path_simplification_tolerance)) {
      if (!positions.empty() && distance(positions.back(), reordered[index]) < 1e-9) {
        continue;
      }
      positions.push_back(reordered[index]);
      owners.push_back(segment);
    }
    boundaries.push_back(positions.size() - 1);
  }
  std::vector<std::vector<MotionLimits>> limits(positions.size());
  std::vector<double> estimates(positions.size(), 0.02);
  for (size_t i = 1; i < positions.size(); ++i) {
    for (size_t j = 0; j < dofs; ++j) {
      const auto bound =
          effective_limits(config, trajectory.joint_names[j], steps[owners[i]], model);
      limits[i].push_back(bound);
      const double delta = std::abs(positions[i][j] - positions[i - 1][j]);
      estimates[i] = std::max({estimates[i],
          delta / bound.velocity,
          std::sqrt(delta / bound.acceleration),
          std::cbrt(delta / bound.jerk)});
    }
  }
  std::vector<Positions> velocities(positions.size(), Positions(dofs, 0.0));
  for (size_t i = 1; i + 1 < positions.size(); ++i) {
    if (steps[owners[i + 1]].endpoints_only) {
      // Enter a straight segment along its own joint direction, so it stays straight.
      double scale = 1 / estimates[i + 1];
      for (size_t j = 0; j < dofs; ++j) {
        const double delta = std::abs(positions[i + 1][j] - positions[i][j]);
        const double cap = 0.5 * std::min(limits[i][j].velocity, limits[i + 1][j].velocity);
        if (delta > 0) scale = std::min(scale, cap / delta);
      }
      for (size_t j = 0; j < dofs; ++j) {
        velocities[i][j] = (positions[i + 1][j] - positions[i][j]) * scale;
      }
      continue;
    }
    for (size_t j = 0; j < dofs; ++j) {
      const double before = (positions[i][j] - positions[i - 1][j]) / estimates[i];
      const double after = (positions[i + 1][j] - positions[i][j]) / estimates[i + 1];
      if (before * after > 0) {
        const double tangent = before * after / (before + after);
        const double cap = 0.5 * std::min(limits[i][j].velocity, limits[i + 1][j].velocity);
        velocities[i][j] = std::clamp(tangent, -cap, cap);
      }
    }
  }
  Point first;
  first.positions = positions.front();
  first.velocities = first.accelerations = Positions(dofs, 0.0);
  trajectory.points.push_back(first);
  output.point_steps.push_back(0);
  std::vector<double> node_times(positions.size(), 0.0);
  double time = 0;
  ruckig::Ruckig<ruckig::DynamicDOFs> generator(dofs);
  for (size_t i = 1; i < positions.size(); ++i) {
    ruckig::InputParameter<ruckig::DynamicDOFs> input(dofs);
    input.current_position = positions[i - 1];
    input.current_velocity = velocities[i - 1];
    input.current_acceleration = Positions(dofs, 0.0);
    input.target_position = positions[i];
    input.target_velocity = velocities[i];
    input.target_acceleration = Positions(dofs, 0.0);
    for (size_t j = 0; j < dofs; ++j) {
      input.max_velocity[j] = limits[i][j].velocity * 0.98;
      input.max_acceleration[j] = limits[i][j].acceleration * 0.95;
      // Leave interpolation margin around jerk switches in the controller spline.
      input.max_jerk[j] = limits[i][j].jerk * 0.5;
    }
    if (steps[owners[i]].type == "gripper") {
      input.minimum_duration = steps[owners[i]].duration;
    }
    if (steps[owners[i]].endpoints_only) {
      // Scaled copies of one profile: a straight joint line (Ruckig falls back if impossible).
      input.synchronization = ruckig::Synchronization::Phase;
    }
    ruckig::Trajectory<ruckig::DynamicDOFs> curve(dofs);
    if (generator.calculate(input, curve) < ruckig::Result::Working) {
      throw std::runtime_error("Ruckig could not connect planned waypoints");
    }
    const double duration = curve.get_duration();
    const size_t samples =
        std::max<size_t>(1, std::ceil(duration / config.trajectory_sample_period));
    for (size_t sample = 1; sample <= samples; ++sample) {
      const double local_time = duration * sample / samples;
      Point point;
      point.positions.resize(dofs);
      point.velocities.resize(dofs);
      point.accelerations.resize(dofs);
      curve.at_time(local_time, point.positions, point.velocities, point.accelerations);
      point.time_from_start = rclcpp::Duration::from_seconds(time + local_time);
      trajectory.points.push_back(std::move(point));
      output.point_steps.push_back(owners[i]);
    }
    time += duration;
    node_times[i] = time;
  }
  if (trajectory.points.size() == 1) {
    trajectory.points.push_back(first);
    trajectory.points.back().time_from_start = rclcpp::Duration::from_seconds(
        steps.back().type == "gripper" ? std::max(steps.back().duration, 0.02) : 0.02);
    output.point_steps.push_back(steps.size() - 1);
  }
  for (auto boundary : boundaries) {
    output.segment_end_times.push_back(node_times[boundary]);
  }
  return output;
}

void validate_smooth_path(const SmoothPath & path,
    const std::vector<Step> & steps,
    const Config & config,
    planning_scene::PlanningScene & scene) {
  const auto & trajectory = path.trajectory.joint_trajectory;
  auto state = scene.getCurrentState();
  // Limits and held joints are checked at every sample; collisions once any joint moved
  // collision_check_step since the last checked sample, and always at the first and last.
  std::vector<double> sampled(trajectory.joint_names.size()), checked;
  std::vector<std::optional<collision_detection::AllowedCollisionMatrix>> strict(steps.size());
  for (size_t s = 0; s < steps.size(); ++s) {
    if (steps[s].strict_objects.empty()) continue;
    strict[s] = scene.getAllowedCollisionMatrix();
    for (const auto & id : steps[s].strict_objects) strict[s]->setEntry(id, false);
  }
  // TCP line per checked step: from the point before its first sample to its last sample.
  std::vector<std::optional<TcpLine>> lines(steps.size());
  const auto & owners = path.point_steps;
  for (size_t s = 0; s < steps.size(); ++s) {
    if (steps[s].tcp_line_tolerance <= 0 && steps[s].tcp_drop_tolerance <= 0) continue;
    const auto arm = config.arms.find(steps[s].arm);
    if (arm == config.arms.end() || !scene.getRobotModel()->hasLinkModel(arm->second.tcp)) {
      throw std::runtime_error("TCP line check needs a known arm TCP: " + steps[s].name);
    }
    const auto begin = std::find(owners.begin(), owners.end(), s);
    if (begin == owners.end()) throw std::runtime_error("No smoothed samples for " + steps[s].name);
    const size_t start = begin == owners.begin() ? 0 : begin - owners.begin() - 1;
    const size_t end = owners.rend() - std::find(owners.rbegin(), owners.rend(), s) - 1;
    const auto tcp_at = [&](size_t point) {
      auto probe = state;
      const auto & positions = trajectory.points.at(point).positions;
      for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
        probe.setVariablePosition(trajectory.joint_names[j], positions[j]);
      }
      probe.update();
      return Eigen::Vector3d(probe.getGlobalLinkTransform(arm->second.tcp).translation());
    };
    lines[s] = TcpLine{arm->second.tcp, tcp_at(start), tcp_at(end)};
  }
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & first = trajectory.points[i - 1];
    const auto & last = trajectory.points[i];
    const double dt = seconds(last.time_from_start) - seconds(first.time_from_start);
    if (!(dt > 0)) {
      throw std::runtime_error("Non-increasing trajectory timestamps");
    }
    const auto & corridor_step = steps.at(path.point_steps.at(i));
    for (const auto & [name, target] : held_arm_targets(corridor_step, config)) {
      const auto found = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
      if (found == trajectory.joint_names.end()) throw std::runtime_error("Smoothed path lacks " + name);
      const size_t j = found - trajectory.joint_names.begin();
      // Quintic Bernstein control points bound the complete interval, including between samples.
      const double p0 = first.positions.at(j), p1 = last.positions.at(j);
      const double v0 = first.velocities.at(j), v1 = last.velocities.at(j);
      const double a0 = first.accelerations.at(j), a1 = last.accelerations.at(j);
      const std::vector<double> controls = {p0, p0 + v0 * dt / 5,
          p0 + 2 * v0 * dt / 5 + a0 * dt * dt / 20,
          p1 - 2 * v1 * dt / 5 + a1 * dt * dt / 20, p1 - v1 * dt / 5, p1};
      for (const double value : controls) {
        if (!std::isfinite(value) || std::abs(value - target) > corridor_step.hold_tolerance + 1e-10) {
          throw std::runtime_error("Smoothed held arm leaves certified corridor: " +
              corridor_step.name + " / " + name);
        }
      }
    }
    const size_t samples = std::max<size_t>(5, std::ceil(dt / 0.002));
    for (size_t sample = 0; sample <= samples; ++sample) {
      const double u = double(sample) / samples;
      for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
        const auto & name = trajectory.joint_names[joint];
        const auto bound =
            effective_limits(config, name, steps[path.point_steps[i]], scene.getRobotModel());
        const auto value = quintic(first, last, joint, u);
        if (!std::isfinite(value.position) || !std::isfinite(value.velocity) ||
            !std::isfinite(value.acceleration) || !std::isfinite(value.jerk) ||
            std::abs(value.velocity) > bound.velocity + 1e-5 ||
            std::abs(value.acceleration) > bound.acceleration + 1e-5 ||
            std::abs(value.jerk) > bound.jerk + 1e-3) {
          throw std::runtime_error(
              "Smoothed trajectory exceeds a motion limit: " + name +
              " v=" + std::to_string(value.velocity) + "/" + std::to_string(bound.velocity) +
              " a=" + std::to_string(value.acceleration) + "/" +
              std::to_string(bound.acceleration) + " j=" + std::to_string(value.jerk) + "/" +
              std::to_string(bound.jerk));
        }
        state.setVariablePosition(name, value.position);
        sampled[joint] = value.position;
      }
      state.update();
      if (!state.satisfiesBounds(1e-8)) {
        throw std::runtime_error("Smoothed path exceeds joint bounds");
      }
      if (auto & line = lines[path.point_steps[i]]) {
        measure_tcp_line(*line, state.getGlobalLinkTransform(line->link).translation());
      }
      bool check = config.collision_check_step <= 0 || checked.empty() ||
                   (i + 1 == trajectory.points.size() && sample == samples);
      for (size_t joint = 0; !check && joint < sampled.size(); ++joint) {
        check = std::abs(sampled[joint] - checked[joint]) >= config.collision_check_step;
      }
      if (check) {
        checked = sampled;
        collision_detection::CollisionRequest request;
        request.group_name = config.coordinated_group;
        request.contacts = true;
        request.max_contacts = 10;
        collision_detection::CollisionResult collision;
        const auto & acm = strict.at(path.point_steps.at(i));
        if (acm) {
          scene.checkCollision(request, collision, state, *acm);
        } else {
          scene.checkCollision(request, collision, state);
        }
        if (collision.collision) {
          std::string pairs;
          for (const auto & contact : collision.contacts) {
            pairs += "; " + contact.first.first + " / " + contact.first.second;
          }
          throw PathCollisionError("Smoothed path is in collision" + pairs);
        }
      }
      // A giver gripper must stay fixed throughout the entire approach.
      const auto & step = steps[path.point_steps[i]];
      for (const auto & [side, arm] : config.arms) {
        if (step.type == "joint_gripper" && side != step.arm) {
          for (const auto & joint : arm.joints) {
            const auto index =
                std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), joint);
            if (index == trajectory.joint_names.end() ||
                std::abs(
                    state.getVariablePosition(joint) -
                    trajectory.points.front().positions[index - trajectory.joint_names.begin()]) >
                    std::min(step.tolerance, 0.001)) {
              throw std::runtime_error("Held arm moved during smoothing");
            }
          }
        }
        if (step.type == "dual_joint" || side != step.arm) {
          const auto index = std::find(
              trajectory.joint_names.begin(), trajectory.joint_names.end(), arm.gripper_joint);
          if (index != trajectory.joint_names.end() &&
              std::abs(
                  state.getVariablePosition(arm.gripper_joint) -
                  trajectory.points.front().positions[index - trajectory.joint_names.begin()]) >
                  0.001) {
            throw std::runtime_error("Held gripper moved during smoothing");
          }
        }
      }
    }
  }
  for (size_t s = 0; s < steps.size(); ++s) {
    if (!lines[s]) continue;
    check_tcp_line(steps[s], *lines[s]);
    RCLCPP_INFO(rclcpp::get_logger("handover_demo"), "%s TCP line: up to %s mm off, %s mm dip",
        steps[s].name.c_str(), millimeters(lines[s]->off).c_str(),
        millimeters(lines[s]->drop).c_str());
  }
}
JointRows sample_joint_path(const trajectory_msgs::msg::JointTrajectory & path, double max_step) {
  const size_t dofs = path.joint_names.size();
  if (path.points.empty() || dofs == 0 || !(max_step > 0)) return {};
  const auto quintic_ready = [&](const Point & point) {
    return point.positions.size() == dofs && point.velocities.size() == dofs &&
           point.accelerations.size() == dofs;
  };
  JointRows dense;
  dense.push_back(path.points.front().positions);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & first = path.points[i - 1];
    const auto & last = path.points[i];
    const double delta = distance(first.positions, last.positions);
    const size_t count = std::max<size_t>(4, std::ceil(4 * delta / max_step));
    const bool smooth = quintic_ready(first) && quintic_ready(last) &&
                        seconds(last.time_from_start) > seconds(first.time_from_start);
    for (size_t sample = 1; sample <= count; ++sample) {
      const double u = double(sample) / count;
      Positions row(dofs);
      for (size_t joint = 0; joint < dofs; ++joint) {
        // Controller quintic when velocities and accelerations exist, else a straight line.
        row[joint] = smooth ? quintic(first, last, joint, u).position
                            : first.positions[joint] +
                                  u * (last.positions[joint] - first.positions[joint]);
      }
      dense.push_back(std::move(row));
    }
  }
  JointRows rows{dense.front()};
  for (size_t i = 1; i + 1 < dense.size(); ++i) {
    if (distance(rows.back(), dense[i + 1]) > max_step) rows.push_back(dense[i]);
  }
  if (dense.size() > 1) rows.push_back(dense.back());
  return rows;
}

std::string check_path_pairs(planning_scene::PlanningScene & scene,
    const Config & config,
    const std::vector<std::string> & first_names,
    const JointRows & first,
    const std::vector<std::string> & second_names,
    const JointRows & second,
    size_t & pairs) {
  pairs = 0;
  auto state = scene.getCurrentState();
  collision_detection::CollisionRequest request;
  request.group_name = config.coordinated_group;
  request.contacts = true;
  request.max_contacts = 10;
  for (size_t a = 0; a < first.size(); ++a) {
    for (size_t j = 0; j < first_names.size(); ++j) {
      state.setVariablePosition(first_names[j], first[a][j]);
    }
    for (size_t b = 0; b < second.size(); ++b) {
      for (size_t j = 0; j < second_names.size(); ++j) {
        state.setVariablePosition(second_names[j], second[b][j]);
      }
      state.update();
      ++pairs;
      collision_detection::CollisionResult collision;
      scene.checkCollision(request, collision, state);
      if (collision.collision) {
        std::string contacts;
        for (const auto & contact : collision.contacts) {
          contacts += "; " + contact.first.first + " / " + contact.first.second;
        }
        return "path pair " + std::to_string(a) + "/" + std::to_string(first.size()) + " x " +
               std::to_string(b) + "/" + std::to_string(second.size()) + " collides" + contacts;
      }
    }
  }
  return {};
}
} // namespace demo_1f

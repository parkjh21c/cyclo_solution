#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>

namespace {
using namespace std::chrono_literals;
using Move = moveit_msgs::action::MoveGroup;
using Codes = moveit_msgs::msg::MoveItErrorCodes;
using demo_1f::Config;
using demo_1f::SmoothPath;
using demo_1f::Step;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Options {
  std::string out_dir{"/tmp/return_compare"};
  std::string config_node{"/handover_demo"};
  std::string ompl_planner{"RRTConnectkConfigDefault"};
  std::string stomp_planner{""}; // Default stomp_planning.yaml defines no planner config ID.
  int runs{20};
  double stop_threshold{0.05};
  bool from_current{false};
  bool skip_planners{false};
  std::vector<std::vector<double>> vias, via_alphas; // Extra straight-via candidates.
  std::vector<std::string> planners{"ompl", "stomp", "stomp_seed"};
  double padding_mm{0}; // World objects grown by this much in planning requests only.
};

std::vector<double> numbers(const std::string & text, const std::string & name) {
  std::vector<double> values;
  std::stringstream in(text);
  std::string item;
  while (std::getline(in, item, ',')) values.push_back(std::stod(item));
  if (values.empty()) throw std::invalid_argument(name + " needs comma-separated values");
  return values;
}

Options parse(const std::vector<std::string> & args) {
  Options options;
  for (size_t i = 1; i < args.size(); ++i) {
    const auto & name = args[i];
    const auto value = [&]() {
      if (i + 1 >= args.size()) throw std::invalid_argument("Missing value for " + name);
      return args[++i];
    };
    if (name == "--out-dir") options.out_dir = value();
    else if (name == "--config-node") options.config_node = value();
    else if (name == "--ompl-planner") options.ompl_planner = value();
    else if (name == "--stomp-planner") options.stomp_planner = value();
    else if (name == "--runs") options.runs = std::stoi(value());
    else if (name == "--stop-threshold") options.stop_threshold = std::stod(value());
    else if (name == "--from-current") options.from_current = true;
    else if (name == "--skip-planners") options.skip_planners = true;
    else if (name == "--via") options.vias.push_back(numbers(value(), name));
    else if (name == "--via-alpha") options.via_alphas.push_back(numbers(value(), name));
    else if (name == "--padding-mm") options.padding_mm = std::stod(value());
    else if (name == "--planners") {
      options.planners.clear();
      std::stringstream in(value());
      std::string item;
      while (std::getline(in, item, ',')) {
        if (item != "ompl" && item != "stomp" && item != "stomp_seed") {
          throw std::invalid_argument("--planners takes ompl, stomp, stomp_seed: " + item);
        }
        options.planners.push_back(item);
      }
    }
    else throw std::invalid_argument("Unknown argument: " + name);
  }
  if (options.runs < 1 || !(options.stop_threshold > 0) || !(options.padding_mm >= 0)) {
    throw std::invalid_argument("--runs must be >= 1, --stop-threshold > 0, --padding-mm >= 0");
  }
  if (options.skip_planners) options.planners.clear();
  return options;
}

std::string error_name(int code) {
  switch (code) {
    case Codes::SUCCESS: return "SUCCESS";
    case Codes::FAILURE: return "FAILURE";
    case Codes::PLANNING_FAILED: return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case Codes::TIMED_OUT: return "TIMED_OUT";
    case Codes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Codes::START_STATE_INVALID: return "START_STATE_INVALID";
    case Codes::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case Codes::GOAL_STATE_INVALID: return "GOAL_STATE_INVALID";
    case Codes::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case Codes::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case Codes::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "MoveIt error " + std::to_string(code);
  }
}

template<class Future>
void wait(const rclcpp::Node::SharedPtr & node, Future & future, double seconds) {
  if (rclcpp::spin_until_future_complete(node, future,
          std::chrono::duration<double>(seconds)) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Read/planning request timed out or was interrupted");
  }
}

const Step & find_step(const Config & config, const std::string & sequence, const std::string & name) {
  const auto & steps = config.motion_sequences.at(sequence).steps;
  const auto found = std::find_if(steps.begin(), steps.end(),
      [&](const Step & step) { return step.name == name; });
  if (found == steps.end()) throw std::runtime_error("Missing step " + sequence + "/" + name);
  return *found;
}

std::string joined(const std::vector<double> & values) {
  std::ostringstream out;
  out << std::setprecision(6);
  for (size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << values[i];
  return out.str();
}

// One run's outcome; metrics are filled only when status == "ok".
struct Sample {
  double t{0};
  std::vector<double> q, qd;
  double speed{0};
  Eigen::Vector3d tcp{Eigen::Vector3d::Zero()};
  double clearance_mm{kNaN};
};

struct Run {
  std::string method;
  int run{0};
  std::string status{"ok"}, fail_stage, fail_detail, fail_key;
  double plan_time{kNaN};
  size_t raw_points{0}, knots{0};
  double duration{kNaN}, min_mid_speed{kNaN}, joint_len{kNaN}, tcp_len{kNaN};
  int stop_count{0}, reversing{0};
  double min_clearance_mm{kNaN}, clearance_pct{kNaN};
  std::string clearance_link, clearance_object, reversing_names;
  std::vector<double> via; // Straight-via methods only.
  std::vector<Sample> samples;
};

void fail(Run & run, const std::string & stage, const std::string & detail, const std::string & key) {
  run.status = "fail";
  run.fail_stage = stage;
  run.fail_detail = detail;
  run.fail_key = stage + ": " + key;
}

// First collision pair, or the message head, as a grouping key for failure counts.
std::string short_reason(const std::string & detail) {
  const auto pair = detail.find("; ");
  if (pair != std::string::npos) {
    const auto end = detail.find("; ", pair + 2);
    return detail.substr(pair + 2, end == std::string::npos ? std::string::npos : end - pair - 2);
  }
  return detail.substr(0, 60);
}

// Speed, TCP, clearance and path metrics of the smoothed, validated path.
void measure(Run & run, const SmoothPath & path, const Config & config, const std::string & arm_name,
    const planning_scene::PlanningScene & scene, const moveit::core::RobotState & start,
    double stop_threshold) {
  const auto & arm = config.arms.at(arm_name);
  const auto & trajectory = path.trajectory.joint_trajectory;
  std::vector<size_t> index;
  for (const auto & name : arm.joints) {
    const auto found = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
    if (found == trajectory.joint_names.end()) throw std::runtime_error("Smoothed path lacks " + name);
    index.push_back(found - trajectory.joint_names.begin());
  }
  const auto model = scene.getRobotModel();
  collision_detection::DistanceRequest request;
  request.group_name = arm.group;
  request.enableGroup(model);
  request.acm = &scene.getAllowedCollisionMatrix();
  request.enable_signed_distance = true;
  request.type = collision_detection::DistanceRequestType::GLOBAL;
  const auto frame = scene.getFrameTransform(config.planning_frame).inverse();
  auto state = start;
  double last_clearance = -1;
  run.min_clearance_mm = std::numeric_limits<double>::infinity();
  for (size_t p = 0; p < trajectory.points.size(); ++p) {
    const auto & point = trajectory.points[p];
    Sample sample;
    sample.t = rclcpp::Duration(point.time_from_start).seconds();
    for (size_t j = 0; j < index.size(); ++j) {
      sample.q.push_back(point.positions.at(index[j]));
      sample.qd.push_back(point.velocities.empty() ? 0.0 : point.velocities.at(index[j]));
      sample.speed += sample.qd.back() * sample.qd.back();
      state.setJointPositions(arm.joints[j], &sample.q.back());
    }
    sample.speed = std::sqrt(sample.speed);
    state.update();
    sample.tcp = frame * state.getGlobalLinkTransform(arm.tcp).translation();
    // Clearance at most every 0.05 s, always at the first and last sample.
    if (p == 0 || p + 1 == trajectory.points.size() || sample.t - last_clearance >= 0.05 - 1e-9) {
      last_clearance = sample.t;
      collision_detection::DistanceResult result;
      scene.getCollisionEnv()->distanceRobot(request, result, state);
      const auto & nearest = result.minimum_distance;
      if (std::isfinite(nearest.distance) && nearest.distance < 1e6) {
        sample.clearance_mm = nearest.distance * 1000;
        if (sample.clearance_mm < run.min_clearance_mm) {
          run.min_clearance_mm = sample.clearance_mm;
          const bool first_world = nearest.body_types[0] == collision_detection::BodyType::WORLD_OBJECT;
          run.clearance_object = nearest.link_names[first_world ? 0 : 1];
          run.clearance_link = nearest.link_names[first_world ? 1 : 0];
          run.clearance_pct = sample.t;
        }
      }
    }
    run.samples.push_back(std::move(sample));
  }
  const auto & samples = run.samples;
  run.duration = samples.back().t;
  if (!std::isfinite(run.min_clearance_mm)) run.min_clearance_mm = kNaN;
  run.clearance_pct = run.duration > 0 ? run.clearance_pct / run.duration * 100 : 0;
  run.knots = path.knots;
  // Mid window 10-90 %: minimum speed and entries below the stop threshold.
  run.min_mid_speed = std::numeric_limits<double>::infinity();
  bool below = false;
  for (const auto & sample : samples) {
    if (sample.t < 0.1 * run.duration || sample.t > 0.9 * run.duration) continue;
    run.min_mid_speed = std::min(run.min_mid_speed, sample.speed);
    const bool now_below = sample.speed < stop_threshold;
    if (now_below && !below) ++run.stop_count;
    below = now_below;
  }
  if (!std::isfinite(run.min_mid_speed)) run.min_mid_speed = kNaN;
  run.joint_len = 0;
  run.tcp_len = 0;
  for (size_t j = 0; j < index.size(); ++j) {
    int sign = 0;
    bool reversed = false;
    for (size_t i = 0; i < samples.size(); ++i) {
      if (i > 0) run.joint_len += std::abs(samples[i].q[j] - samples[i - 1].q[j]);
      const double v = samples[i].qd[j];
      if (std::abs(v) < 1e-3) continue;
      const int now = v > 0 ? 1 : -1;
      reversed = reversed || (sign != 0 && now != sign);
      sign = now;
    }
    run.reversing += reversed ? 1 : 0;
    if (reversed) run.reversing_names += (run.reversing_names.empty() ? "j" : " j") + std::to_string(j + 1);
  }
  for (size_t i = 1; i < samples.size(); ++i) run.tcp_len += (samples[i].tcp - samples[i - 1].tcp).norm();
}

// Smooth with the runtime smoother, validate in a fresh copy of the start scene, then measure.
void finish(Run & run, const std::vector<moveit_msgs::msg::RobotTrajectory> & segments,
    const std::vector<Step> & steps, const Config & config,
    const moveit::core::RobotModelConstPtr & model,
    const planning_scene::PlanningScenePtr & scene, const moveit::core::RobotState & start,
    double stop_threshold) {
  SmoothPath path;
  try {
    path = demo_1f::smooth_segments(segments, steps, config, model);
  } catch (const std::exception & error) {
    fail(run, "smooth", error.what(), short_reason(error.what()));
    return;
  }
  auto local = scene->diff();
  local->setCurrentState(start);
  try {
    demo_1f::validate_smooth_path(path, steps, config, *local);
  } catch (const demo_1f::PathCollisionError & error) {
    fail(run, "validate", error.what(), "collision " + short_reason(error.what()));
    return;
  } catch (const std::exception & error) {
    fail(run, "validate", error.what(), short_reason(error.what()));
    return;
  }
  measure(run, path, config, steps.back().arm, *local, start, stop_threshold);
}

// Joint goal as the single-arm "joint" branch of runtime/execution/moveit_goal.cpp.
Move::Goal joint_goal(const Config & config, const Step & step, const std::string & pipeline,
    const std::string & planner, const moveit_msgs::msg::PlanningScene & scene,
    const moveit_msgs::msg::RobotState & start) {
  const auto & arm = config.arms.at(step.arm);
  Move::Goal goal;
  auto & request = goal.request;
  request.group_name = arm.group;
  request.pipeline_id = pipeline;
  request.planner_id = planner;
  request.num_planning_attempts = pipeline == "ompl" ? config.ompl_planning_attempts : 1;
  request.allowed_planning_time = step.planning_time > 0 ? step.planning_time : config.planning_time;
  request.max_velocity_scaling_factor = step.velocity_scale > 0 ? step.velocity_scale : config.velocity_scale;
  request.max_acceleration_scaling_factor =
      step.acceleration_scale > 0 ? step.acceleration_scale : config.acceleration_scale;
  request.start_state = start;
  request.start_state.is_diff = false;
  const double tolerance = step.planning_tolerance > 0 ? step.planning_tolerance : step.tolerance;
  moveit_msgs::msg::Constraints constraints;
  for (size_t i = 0; i < arm.joints.size(); ++i) {
    moveit_msgs::msg::JointConstraint joint;
    joint.joint_name = arm.joints[i];
    joint.position = step.target.at(i);
    joint.tolerance_above = tolerance;
    joint.tolerance_below = tolerance;
    joint.weight = 1.0;
    constraints.joint_constraints.push_back(joint);
  }
  request.goal_constraints.push_back(constraints);
  goal.planning_options.plan_only = true;
  goal.planning_options.replan = false;
  goal.planning_options.look_around = false;
  // Complete private scene and explicit start: the live robot state is never used.
  goal.planning_options.planning_scene_diff = scene;
  goal.planning_options.planning_scene_diff.is_diff = false;
  goal.planning_options.planning_scene_diff.robot_state = start;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = false;
  return goal;
}

// One plan-only request; failures are recorded in run, never thrown.
bool plan_once(const rclcpp::Node::SharedPtr & node, const rclcpp_action::Client<Move>::SharedPtr & client,
    const Move::Goal & goal, const Config & config, const moveit::core::RobotState & start,
    Run & run, moveit_msgs::msg::RobotTrajectory & trajectory) {
  auto accepted = client->async_send_goal(goal);
  try {
    wait(node, accepted, 10);
  } catch (...) {
    fail(run, "plan", "goal not accepted in time", "GOAL_SEND_TIMEOUT");
    return false;
  }
  const auto handle = accepted.get();
  if (!handle) {
    fail(run, "plan", "planning goal rejected", "GOAL_REJECTED");
    return false;
  }
  auto pending = client->async_get_result(handle);
  try {
    wait(node, pending, std::max(60., goal.request.allowed_planning_time + 10));
  } catch (...) {
    auto cancellation = client->async_cancel_goal(handle);
    try { wait(node, cancellation, 5); } catch (...) {}
    fail(run, "plan", "no result in time; goal cancelled", "RESULT_TIMEOUT");
    return false;
  }
  const auto response = pending.get();
  if (!response.result) {
    fail(run, "plan", "missing planning result", "NO_RESULT");
    return false;
  }
  const auto & result = *response.result;
  if (!result.executed_trajectory.joint_trajectory.points.empty() ||
      !result.executed_trajectory.multi_dof_joint_trajectory.points.empty()) {
    throw std::runtime_error("Server reported execution for a plan-only request; aborting");
  }
  run.plan_time = result.planning_time;
  if (response.code != rclcpp_action::ResultCode::SUCCEEDED || result.error_code.val != Codes::SUCCESS) {
    const auto name = error_name(result.error_code.val);
    fail(run, "plan", name + (result.error_code.message.empty() ? "" : ": " + result.error_code.message),
        name);
    return false;
  }
  trajectory = result.planned_trajectory;
  const auto & path = trajectory.joint_trajectory;
  run.raw_points = path.points.size();
  if (path.points.empty()) {
    fail(run, "plan", "empty planned trajectory", "EMPTY_TRAJECTORY");
    return false;
  }
  for (size_t i = 0; i < path.joint_names.size(); ++i) {
    if (path.points.front().positions.size() != path.joint_names.size() ||
        std::abs(path.points.front().positions[i] - start.getVariablePosition(path.joint_names[i])) >
            config.continuous_start_tolerance) {
      fail(run, "plan", "planner changed the requested start: " + path.joint_names[i], "START_CHANGED");
      return false;
    }
  }
  return true;
}

// Fresh, stationary joint feedback for every independent model joint (as check_pour_plans).
std::map<std::string, double> measured_positions(const rclcpp::Node::SharedPtr & node,
    const Config & config, const moveit::core::RobotModelConstPtr & model) {
  struct Feedback {
    double position, velocity;
    std::chrono::steady_clock::time_point received;
  };
  std::map<std::string, Feedback> feedback;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      config.joint_states_topic, rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::ConstSharedPtr sample) {
        if (sample->name.size() != sample->position.size()) return;
        const double age = (node->now() - rclcpp::Time(sample->header.stamp,
            node->get_clock()->get_clock_type())).seconds();
        if (!std::isfinite(age) || age < -.1 || age > config.state_max_age) return;
        for (size_t i = 0; i < sample->name.size(); ++i) {
          feedback[sample->name[i]] = {sample->position[i],
              i < sample->velocity.size() ? sample->velocity[i] : kNaN, std::chrono::steady_clock::now()};
        }
      });
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  std::string missing;
  do {
    rclcpp::spin_some(node);
    missing.clear();
    for (const auto & name : model->getVariableNames()) {
      if (model->getJointOfVariable(name)->getMimic()) continue;
      const auto found = feedback.find(name);
      if (found == feedback.end() || !std::isfinite(found->second.position) ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - found->second.received).count() >
              config.state_max_age) {
        missing = name;
        break;
      }
    }
    if (missing.empty()) break;
    std::this_thread::sleep_for(10ms);
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
  if (!missing.empty()) throw std::runtime_error("Missing fresh joint feedback: " + missing);
  std::map<std::string, double> positions;
  const auto & variables = model->getVariableNames();
  for (const auto & [name, sample] : feedback) {
    if (std::find(variables.begin(), variables.end(), name) == variables.end() ||
        model->getJointOfVariable(name)->getMimic()) {
      continue;
    }
    bool gripper = false;
    for (const auto & entry : config.arms) gripper = gripper || name == entry.second.gripper_joint;
    if (!gripper && (!std::isfinite(sample.velocity) || std::abs(sample.velocity) > config.stopped_velocity)) {
      throw std::runtime_error("Joint is moving or has unknown velocity: " + name);
    }
    positions[name] = sample.position;
  }
  return positions;
}

double median(std::vector<double> values) {
  if (values.empty()) return kNaN;
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

void print_limits() {
  std::cout << "LIMITS:\n"
            << "  - Default start is the configured retreat_together targets; the real start is the"
               " measured retreat pose and differs slightly (use --from-current to plan from it).\n"
            << "  - The left arm is a static obstacle; simultaneous left motion and the main/side"
               " pair check are not reproduced.\n"
            << "  - Runtime side returns do not call a planner; using B or C needs separate work"
               " in the side preparation stage. This tool previews that result only.\n"
            << "  - Clearance covers world objects only (no self or attached-object distances).\n";
}

// Copy of the scene with world primitives grown by pad on every side; meshes are left as they are.
moveit_msgs::msg::PlanningScene inflated(moveit_msgs::msg::PlanningScene message, double pad) {
  using Primitive = shape_msgs::msg::SolidPrimitive;
  size_t grown = 0, skipped = 0;
  for (auto & object : message.world.collision_objects) {
    for (auto & primitive : object.primitives) {
      auto & d = primitive.dimensions;
      if (primitive.type == Primitive::BOX && d.size() >= 3) {
        for (size_t i = 0; i < 3; ++i) d[i] += 2 * pad;
      } else if ((primitive.type == Primitive::CYLINDER || primitive.type == Primitive::CONE) && d.size() >= 2) {
        d[Primitive::CYLINDER_HEIGHT] += 2 * pad;
        d[Primitive::CYLINDER_RADIUS] += pad;
      } else if (primitive.type == Primitive::SPHERE && !d.empty()) {
        d[Primitive::SPHERE_RADIUS] += pad;
      } else {
        ++skipped;
        continue;
      }
      ++grown;
    }
    skipped += object.meshes.size();
  }
  std::cout << "padding " << pad * 1000 << " mm: " << grown << " primitives grown, " << skipped
            << " shapes left unpadded (planning requests only; validation uses the real scene)\n";
  return message;
}

// STOMP seed: the straight path start -> via -> home, evenly spaced, in the group's joint order.
moveit_msgs::msg::TrajectoryConstraints seed_path(const moveit::core::RobotModel & model,
    const Config & config, const std::string & arm_name, const std::vector<double> & start,
    const std::vector<double> & via, const std::vector<double> & home, size_t points) {
  const auto & arm = config.arms.at(arm_name);
  const auto & names = model.getJointModelGroup(arm.group)->getActiveJointModelNames();
  std::vector<size_t> index;
  for (const auto & name : names) {
    const auto found = std::find(arm.joints.begin(), arm.joints.end(), name);
    if (found == arm.joints.end()) throw std::runtime_error("Seed: group joint not in arm joints: " + name);
    index.push_back(found - arm.joints.begin());
  }
  const auto span = [](const std::vector<double> & a, const std::vector<double> & b) {
    double length = 0;
    for (size_t i = 0; i < a.size(); ++i) length = std::max(length, std::abs(b[i] - a[i]));
    return length;
  };
  const double first = span(start, via), total = first + span(via, home);
  moveit_msgs::msg::TrajectoryConstraints seed;
  for (size_t k = 0; k < points; ++k) {
    const double s = total * k / (points - 1);
    const bool before = s <= first && first > 0;
    const auto & a = before ? start : via;
    const auto & b = before ? via : home;
    const double u = before ? s / first : (total > first ? (s - first) / (total - first) : 1.0);
    moveit_msgs::msg::Constraints point;
    for (size_t j = 0; j < names.size(); ++j) {
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name = names[j];
      joint.position = a[index[j]] + u * (b[index[j]] - a[index[j]]);
      joint.weight = 1.0;
      point.joint_constraints.push_back(joint);
    }
    seed.constraints.push_back(point);
  }
  return seed;
}

// STOMP settings of the running move_group, printed for the record; num_timesteps sizes the seed.
size_t stomp_settings(const rclcpp::Node::SharedPtr & node) {
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  size_t timesteps = 60;
  if (!client->wait_for_service(5s)) {
    std::cout << "WARNING /move_group parameters unavailable; seed uses 60 points\n";
    return timesteps;
  }
  const std::vector<std::string> names = {"stomp.num_timesteps", "stomp.num_iterations",
      "stomp.num_iterations_after_valid", "stomp.num_rollouts", "stomp.control_cost_weight"};
  for (const auto & parameter : client->get_parameters(names, 5s)) {
    std::cout << "move_group " << parameter.get_name() << '=' << parameter.value_to_string() << '\n';
    if (parameter.get_name() == "stomp.num_timesteps" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER && parameter.as_int() > 1) {
      timesteps = static_cast<size_t>(parameter.as_int());
    }
  }
  return timesteps;
}

int run(const rclcpp::Node::SharedPtr & node, const Options & options) {
  const auto wall_start = std::chrono::steady_clock::now();
  // Live configuration, as check_pour_plans.
  auto parameters = std::make_shared<rclcpp::SyncParametersClient>(node, options.config_node);
  if (!parameters->wait_for_service(10s)) throw std::runtime_error("Configuration node unavailable");
  const auto names = parameters->list_parameters({}, 0, 10s).names;
  const auto values = parameters->get_parameters(names, 10s);
  if (names.empty() || values.size() != names.size()) throw std::runtime_error("Incomplete parameter snapshot");
  std::map<std::string, rclcpp::Parameter> snapshot;
  for (const auto & value : values) snapshot.emplace(value.get_name(), value);
  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(false).parameter_overrides(values);
  node_options.start_parameter_services(false).start_parameter_event_publisher(false);
  auto settings = std::make_shared<rclcpp::Node>("return_compare_settings", node_options);
  const auto config = demo_1f::load_config(*settings);
  for (const auto & name : settings->list_parameters({}, 0).names) {
    if (!snapshot.count(name)) throw std::runtime_error("Live configuration lacks " + name);
  }
  const auto urdf = urdf::parseURDF(snapshot.at("robot_description").as_string());
  auto semantic = std::make_shared<srdf::Model>();
  if (!urdf || !semantic->initString(*urdf, snapshot.at("robot_description_semantic").as_string())) {
    throw std::runtime_error("Invalid live robot model");
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, semantic);
  if (snapshot.count("use_sim_time")) node->set_parameter(snapshot.at("use_sim_time"));

  // Copies: GCC's -Wdangling-reference misreads the returned config reference.
  const Step retreat = find_step(config, "right_to_left", "retreat_together");
  const Step release = find_step(config, "right_to_left", "giver_release");
  const Step close = find_step(config, "right_to_left", "receiver_close");
  const Step via = find_step(config, "right_return", "giver_home_via");
  const Step home = find_step(config, "right_return", "giver_home");
  const auto & return_error = config.motion_sequences.at("right_return").validation_error;
  if (!return_error.empty()) std::cout << "WARNING right_return validation: " << return_error << '\n';
  const auto & arm = config.arms.at(home.arm);
  if (via.arm != home.arm || via.type != "joint" || home.type != "joint" ||
      via.target.size() != arm.joints.size() || home.target.size() != arm.joints.size()) {
    throw std::runtime_error("right_return must be two joint steps of one arm");
  }
  if (!model->hasJointModelGroup(arm.group) || !model->hasLinkModel(arm.tcp)) {
    throw std::runtime_error("Missing return arm group or TCP in the live model");
  }

  std::map<std::string, double> measured;
  if (options.from_current) measured = measured_positions(node, config, model);

  // The only planning scene read; every evaluation uses local copies of it.
  auto scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!scene_client->wait_for_service(10s)) throw std::runtime_error("Planning scene unavailable");
  auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  scene_request->components.components = 1023;
  auto pending = scene_client->async_send_request(scene_request);
  wait(node, pending, 10);
  auto message = pending.get()->scene;
  const auto read_time = std::time(nullptr);
  const double read_ros = node->now().seconds();
  std::cout << std::setprecision(6) << std::fixed;
  std::cout << "scene_read local=" << std::put_time(std::localtime(&read_time), "%F %T")
            << " ros_time=" << read_ros << " name=" << std::quoted(message.name) << '\n';
  bool table = false, wall = false, slope = false;
  for (const auto & object : message.world.collision_objects) {
    table = table || object.id == "demo_table";
    wall = wall || object.id.rfind("basket_wall_", 0) == 0;
    slope = slope || object.id.rfind("basket_slope_", 0) == 0;
    std::cout << "world_object " << std::quoted(object.id) << " frame=" << object.header.frame_id
              << " xyz=" << object.pose.position.x << ',' << object.pose.position.y << ','
              << object.pose.position.z << '\n';
  }
  for (const auto & object : message.robot_state.attached_collision_objects) {
    std::cout << "attached_object " << std::quoted(object.object.id) << " link=" << object.link_name << '\n';
  }
  std::string missing;
  if (!table) missing += " demo_table";
  if (!wall) missing += " basket_wall_*";
  if (!slope) missing += " basket_slope_*";
  if (!missing.empty()) throw std::runtime_error("Planning scene lacks required objects:" + missing);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  if (message.is_diff || message.robot_state.is_diff || !scene->setPlanningSceneMsg(message)) {
    throw std::runtime_error("A complete planning scene is required");
  }
  if (!scene->knowsFrameTransform(config.planning_frame)) throw std::runtime_error("Unknown planning frame");

  // Common start: right arm at retreat, left at its retreat target, grippers after the release.
  auto start = scene->getCurrentState();
  const auto set = [&](const std::string & name, double value) { start.setJointPositions(name, &value); };
  if (options.from_current) {
    for (const auto & [name, value] : measured) set(name, value);
    std::cout << "start=measured joint_states snapshot\n";
  } else {
    const auto & left = config.arms.at("left");
    const auto & right = config.arms.at("right");
    if (retreat.left_target.size() != left.joints.size() || retreat.right_target.size() != right.joints.size()) {
      throw std::runtime_error("retreat_together lacks left/right targets");
    }
    for (size_t i = 0; i < right.joints.size(); ++i) set(right.joints[i], retreat.right_target[i]);
    for (size_t i = 0; i < left.joints.size(); ++i) set(left.joints[i], retreat.left_target[i]);
    for (const auto * gripper_step : {&release, &close}) {
      const auto & joint = config.arms.at(gripper_step->arm).gripper_joint;
      if (!gripper_step->target.empty()) set(joint, gripper_step->target.front());
      std::cout << "start_gripper " << gripper_step->name << ' ' << joint << '='
                << start.getVariablePosition(joint)
                << (gripper_step->target.empty() ? " (scene value)" : " (step target)") << '\n';
    }
    std::cout << "start=configured retreat_together targets\n";
  }
  start.zeroVelocities();
  start.zeroAccelerations();
  start.update();
  for (const auto & name : model->getVariableNames()) {
    std::cout << "start_joint " << name << '=' << start.getVariablePosition(name) << '\n';
  }
  if (!start.satisfiesBounds(1e-8)) std::cout << "WARNING start state is outside joint bounds\n";
  {
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 20;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, start);
    if (result.collision) {
      std::cerr << "START_IN_COLLISION; no method evaluated\n";
      for (const auto & contact : result.contacts) {
        std::cerr << "  " << contact.first.first << " / " << contact.first.second << '\n';
      }
      return 2;
    }
  }
  scene->setCurrentState(start);
  moveit_msgs::msg::RobotState start_message;
  moveit::core::robotStateToRobotStateMsg(start, start_message);
  std::vector<double> start_arm;
  for (const auto & name : arm.joints) start_arm.push_back(start.getVariablePosition(name));
  std::cout << "S=" << joined(start_arm) << "\nvia=" << joined(via.target) << "\nH=" << joined(home.target) << '\n';
  std::cout << "clearance: FCL distanceRobot, GLOBAL minimum signed distance between the updated links of "
            << arm.group << " (gripper included) and world objects, scene ACM applied, sampled every"
               " <=0.05 s; negative = penetration depth estimate; meshes use FCL mesh distance.\n";

  std::vector<Run> runs;
  std::vector<std::string> methods;
  // A: the runtime side path, two straight joint segments through a via (YAML via, then extras).
  std::vector<std::pair<std::string, std::vector<double>>> candidates = {{"straight_via", via.target}};
  for (const auto & target : options.vias) {
    candidates.push_back({"via_" + std::to_string(candidates.size()), target});
  }
  for (const auto & alpha : options.via_alphas) {
    if (alpha.size() != arm.joints.size()) throw std::invalid_argument("--via-alpha needs one value per arm joint");
    std::vector<double> target;
    for (size_t i = 0; i < alpha.size(); ++i) {
      target.push_back(start_arm[i] + alpha[i] * (home.target[i] - start_arm[i]));
    }
    candidates.push_back({"via_" + std::to_string(candidates.size()), target});
  }
  for (const auto & [method, target] : candidates) {
    if (target.size() != arm.joints.size()) throw std::invalid_argument("--via needs one value per arm joint");
    std::vector<double> alpha;
    for (size_t i = 0; i < target.size(); ++i) {
      const double span = home.target[i] - start_arm[i];
      alpha.push_back(std::abs(span) > 1e-9 ? (target[i] - start_arm[i]) / span : kNaN);
    }
    std::cout << method << " via=" << joined(target) << " alpha=" << joined(alpha) << '\n';
    Step step = via;
    step.target = target;
    Run run{method, 0};
    run.via = target;
    std::vector<moveit_msgs::msg::RobotTrajectory> segments;
    auto from = start_arm;
    const Step * order[] = {&step, &home};
    for (const auto * next : order) {
      moveit_msgs::msg::RobotTrajectory segment;
      segment.joint_trajectory.joint_names = arm.joints;
      trajectory_msgs::msg::JointTrajectoryPoint first, last;
      first.positions = from;
      last.positions = next->target;
      segment.joint_trajectory.points = {first, last};
      segments.push_back(segment);
      from = next->target;
    }
    run.raw_points = 4;
    finish(run, segments, {step, home}, config, model, scene, start, options.stop_threshold);
    methods.push_back(method);
    runs.push_back(std::move(run));
  }
  // B and C: plan-only to giver_home, then the runtime smoothing/validation. Only stomp_seed sees the via.
  rclcpp_action::Client<Move>::SharedPtr client;
  moveit_msgs::msg::PlanningScene request_scene = message;
  size_t timesteps = 60;
  if (!options.planners.empty()) {
    client = rclcpp_action::create_client<Move>(node, config.move_group_action);
    if (!client->wait_for_action_server(10s)) throw std::runtime_error("MoveGroup unavailable");
    request_scene = inflated(message, options.padding_mm / 1000);
    timesteps = stomp_settings(node);
  }
  for (const auto & method : options.planners) {
    methods.push_back(method);
    const bool ompl = method == "ompl";
    auto goal = joint_goal(config, home, ompl ? "ompl" : "stomp",
        ompl ? options.ompl_planner : options.stomp_planner, request_scene, start_message);
    if (method == "stomp_seed") {
      goal.request.trajectory_constraints =
          seed_path(*model, config, home.arm, start_arm, via.target, home.target, timesteps);
      std::cout << method << " seed: start -> via -> home, " << timesteps << " points\n";
    }
    std::cout << method << " request=" << goal.request.pipeline_id << '/' << std::quoted(goal.request.planner_id)
              << " padding_mm=" << options.padding_mm << " group=" << arm.group
              << " attempts=" << goal.request.num_planning_attempts
              << " planning_time=" << goal.request.allowed_planning_time
              << " speed=" << goal.request.max_velocity_scaling_factor
              << " acceleration=" << goal.request.max_acceleration_scaling_factor
              << " tolerance=" << goal.request.goal_constraints[0].joint_constraints[0].tolerance_above << '\n';
    for (int i = 1; i <= options.runs; ++i) {
      Run run{method, i};
      moveit_msgs::msg::RobotTrajectory trajectory;
      if (plan_once(node, client, goal, config, start, run, trajectory)) {
        finish(run, {trajectory}, {home}, config, model, scene, start, options.stop_threshold);
      }
      std::cout << method << " run " << i << '/' << options.runs << ' ' << run.status
                << (run.status == "ok" ? "" : " [" + run.fail_key + "]") << " plan=" << run.plan_time
                << "s duration=" << run.duration << "s min_mid_speed=" << run.min_mid_speed << std::endl;
      runs.push_back(std::move(run));
    }
  }

  // CSV output.
  std::filesystem::create_directories(options.out_dir);
  const auto csv = [](const std::string & text) {
    std::string out = "\"";
    for (const char c : text) out += c == '"' ? std::string("\"\"") : std::string(1, c);
    return out + "\"";
  };
  {
    std::ofstream out(options.out_dir + "/summary.csv");
    out << std::setprecision(9)
        << "method,run,status,fail_stage,fail_detail,plan_time,duration,raw_points,simplified_points,"
           "min_mid_speed,stop_count,reversing_joints,joint_path_len,tcp_path_len,min_clearance_mm,"
           "clearance_link,clearance_object,clearance_at_pct,reversing_names,via\n";
    for (const auto & r : runs) {
      const bool ok = r.status == "ok";
      const auto number = [&](double v) { return ok && std::isfinite(v) ? std::to_string(v) : std::string(); };
      out << r.method << ',' << r.run << ',' << r.status << ',' << r.fail_stage << ',' << csv(r.fail_detail)
          << ',' << (std::isfinite(r.plan_time) ? std::to_string(r.plan_time) : "") << ',' << number(r.duration)
          << ',' << r.raw_points << ',' << (ok ? std::to_string(r.knots) : "") << ',' << number(r.min_mid_speed)
          << ',' << (ok ? std::to_string(r.stop_count) : "") << ',' << (ok ? std::to_string(r.reversing) : "")
          << ',' << number(r.joint_len) << ',' << number(r.tcp_len) << ',' << number(r.min_clearance_mm)
          << ',' << r.clearance_link << ',' << r.clearance_object << ',' << number(r.clearance_pct)
          << ',' << (ok ? r.reversing_names : std::string()) << ',' << csv(joined(r.via)) << '\n';
    }
  }
  {
    std::ofstream out(options.out_dir + "/samples.csv");
    out << std::setprecision(9) << "method,run,t";
    for (size_t j = 1; j <= arm.joints.size(); ++j) out << ",q" << j;
    for (size_t j = 1; j <= arm.joints.size(); ++j) out << ",qd" << j;
    out << ",speed_norm,tcp_x,tcp_y,tcp_z,clearance_mm\n";
    for (const auto & r : runs) {
      for (const auto & s : r.samples) {
        out << r.method << ',' << r.run << ',' << s.t;
        for (const double v : s.q) out << ',' << v;
        for (const double v : s.qd) out << ',' << v;
        out << ',' << s.speed << ',' << s.tcp.x() << ',' << s.tcp.y() << ',' << s.tcp.z() << ',';
        if (std::isfinite(s.clearance_mm)) out << s.clearance_mm;
        out << '\n';
      }
    }
  }
  {
    std::ofstream out(options.out_dir + "/scene_objects.csv");
    out << std::setprecision(9) << "id,shape_index,shape,dim1,dim2,dim3,x,y,z,qx,qy,qz,qw\n";
    const auto frame = scene->getFrameTransform(config.planning_frame).inverse();
    const auto world = scene->getWorld();
    for (const auto & id : world->getObjectIds()) {
      const auto object = world->getObject(id);
      for (size_t k = 0; k < object->shapes_.size(); ++k) {
        const auto & shape = object->shapes_[k];
        std::string type = "other";
        double d[3] = {0, 0, 0};
        if (shape->type == shapes::BOX) {
          const auto * box = static_cast<const shapes::Box *>(shape.get());
          type = "box";
          d[0] = box->size[0], d[1] = box->size[1], d[2] = box->size[2];
        } else if (shape->type == shapes::CYLINDER) {
          const auto * cylinder = static_cast<const shapes::Cylinder *>(shape.get());
          type = "cylinder";
          d[0] = cylinder->radius, d[1] = cylinder->length;
        } else if (shape->type == shapes::SPHERE) {
          type = "sphere";
          d[0] = static_cast<const shapes::Sphere *>(shape.get())->radius;
        } else if (shape->type == shapes::MESH) {
          type = "mesh";
        }
        const Eigen::Isometry3d pose = frame * object->global_shape_poses_[k];
        const Eigen::Quaterniond q(pose.rotation());
        out << csv(id) << ',' << k << ',' << type << ',' << d[0] << ',' << d[1] << ',' << d[2] << ','
            << pose.translation().x() << ',' << pose.translation().y() << ',' << pose.translation().z()
            << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
      }
    }
  }

  // Console summary.
  std::cout << "\nSUMMARY (metrics over validated runs: median [min, max])\n";
  for (const auto & method : methods) {
    std::vector<const Run *> all, ok;
    std::map<std::string, int> failures;
    for (const auto & r : runs) {
      if (r.method != method) continue;
      all.push_back(&r);
      if (r.status == "ok") ok.push_back(&r);
      else ++failures[r.fail_key];
    }
    std::cout << method << ": " << ok.size() << '/' << all.size() << " validated\n";
    for (const auto & [key, count] : failures) std::cout << "  fail x" << count << "  " << key << '\n';
    const auto stat = [&](const std::string & label, auto get) {
      std::vector<double> values;
      for (const auto * r : ok) {
        const double v = get(*r);
        if (std::isfinite(v)) values.push_back(v);
      }
      if (values.empty()) return;
      std::cout << "  " << std::left << std::setw(20) << label << std::right << std::setprecision(3)
                << median(values) << " [" << *std::min_element(values.begin(), values.end()) << ", "
                << *std::max_element(values.begin(), values.end()) << "]\n";
    };
    stat("plan_time_s", [](const Run & r) { return r.plan_time; });
    stat("duration_s", [](const Run & r) { return r.duration; });
    stat("raw_points", [](const Run & r) { return double(r.raw_points); });
    stat("simplified_points", [](const Run & r) { return double(r.knots); });
    stat("min_mid_speed", [](const Run & r) { return r.min_mid_speed; });
    stat("stop_count", [](const Run & r) { return double(r.stop_count); });
    stat("reversing_joints", [](const Run & r) { return double(r.reversing); });
    stat("joint_path_len_rad", [](const Run & r) { return r.joint_len; });
    stat("tcp_path_len_m", [](const Run & r) { return r.tcp_len; });
    stat("min_clearance_mm", [](const Run & r) { return r.min_clearance_mm; });
    for (const auto * r : all) {
      if (r->via.empty()) continue;
      std::cout << "  via " << joined(r->via) << '\n';
      if (r->status == "ok") {
        std::cout << "  clearance at " << r->clearance_pct << "% " << r->clearance_link << " / "
                  << r->clearance_object << "; reversing: "
                  << (r->reversing_names.empty() ? "none" : r->reversing_names) << '\n';
      }
    }
  }
  print_limits();
  std::cout << "csv=" << options.out_dir << " elapsed="
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count() << "s\n";
  return 0;
}
} // namespace

int main(int argc, char ** argv) {
  demo_1f::initialize_moveit_logging();
  rclcpp::init(argc, argv);
  int status = 2;
  try {
    const auto options = parse(rclcpp::remove_ros_arguments(argc, argv));
    auto node = std::make_shared<rclcpp::Node>("compare_return_paths");
    status = run(node, options);
  } catch (const std::exception & error) {
    std::cerr << "CHECK_INCOMPLETE: " << error.what() << '\n';
    print_limits();
  }
  rclcpp::shutdown();
  return status;
}

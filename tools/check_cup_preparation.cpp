#include "demo_1f/msg/demo_status.hpp"
#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using namespace std::chrono_literals;
using Move = moveit_msgs::action::MoveGroup;
using Codes = moveit_msgs::msg::MoveItErrorCodes;
using demo_1f::Config;

template<class Future>
void wait(const rclcpp::Node::SharedPtr & node, Future & future, double seconds) {
  if (rclcpp::spin_until_future_complete(node, future,
          std::chrono::duration<double>(seconds)) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Read/planning request timed out or was interrupted");
  }
}

// This executable has no trajectory execution client and never constructs TrajectoryExecutor.
Move::Goal preparation_goal(const Config & config, const demo_1f::Step & step,
    const moveit_msgs::msg::PlanningScene & scene, const moveit::core::RobotState & state) {
  Move::Goal goal;
  auto & request = goal.request;
  request.group_name = config.coordinated_group;
  request.pipeline_id = step.pipeline;
  request.planner_id = step.planner;
  request.num_planning_attempts = config.ompl_planning_attempts;
  request.allowed_planning_time = step.planning_time > 0 ? step.planning_time : config.planning_time;
  request.max_velocity_scaling_factor = step.velocity_scale;
  request.max_acceleration_scaling_factor = step.acceleration_scale;
  moveit::core::robotStateToRobotStateMsg(state, request.start_state);
  request.start_state.is_diff = false;
  moveit_msgs::msg::Constraints constraints;
  const auto held = demo_1f::held_arm_targets(step, config);
  const double tolerance = step.planning_tolerance > 0 ? step.planning_tolerance : step.tolerance;
  for (const auto & [side, arm] : config.arms) {
    const auto & targets = side == "left" ? step.left_target : step.right_target;
    auto names = arm.joints;
    names.push_back(arm.gripper_joint);
    for (size_t i = 0; i < names.size(); ++i) {
      const bool gripper = i == arm.joints.size();
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name = names[i];
      joint.position = gripper ? state.getVariablePosition(names[i]) : targets.at(i);
      joint.tolerance_above = joint.tolerance_below = gripper ? std::max(tolerance, config.gripper_goal_tolerance) : tolerance;
      joint.weight = 1;
      constraints.joint_constraints.push_back(joint);
      if (gripper || held.count(names[i])) {
        joint.tolerance_above = joint.tolerance_below = gripper ? std::min(tolerance, .001) : step.hold_tolerance;
        request.path_constraints.joint_constraints.push_back(joint);
      }
    }
  }
  request.goal_constraints.push_back(constraints);
  goal.planning_options.plan_only = true;
  goal.planning_options.replan = false;
  goal.planning_options.look_around = false;
  goal.planning_options.planning_scene_diff = scene;
  goal.planning_options.planning_scene_diff.is_diff = false;
  goal.planning_options.planning_scene_diff.robot_state = request.start_state;
  return goal;
}

void print_path(const std::string & label, const trajectory_msgs::msg::JointTrajectory & path) {
  for (const auto & name : path.joint_names) std::cout << label << " joint=" << name << '\n';
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & point = path.points[i];
    std::cout << label << " point=" << i << " time=" << rclcpp::Duration(point.time_from_start).seconds();
    for (double value : point.positions) std::cout << " " << value;
    std::cout << " velocities=";
    for (double value : point.velocities) std::cout << " " << value;
    std::cout << " accelerations=";
    for (double value : point.accelerations) std::cout << " " << value;
    std::cout << '\n';
  }
}


void print_hold_deviation(const trajectory_msgs::msg::JointTrajectory & path,
    const demo_1f::Step & step, const Config & config, double start_time = 0) {
  for (const auto & [name, target] : demo_1f::held_arm_targets(step, config)) {
    const auto found = std::find(path.joint_names.begin(), path.joint_names.end(), name);
    if (found == path.joint_names.end()) throw std::runtime_error("Missing held joint in report");
    double maximum = 0;
    for (const auto & point : path.points) {
      if (rclcpp::Duration(point.time_from_start).seconds() + 1e-9 < start_time) continue;
      maximum = std::max(maximum, std::abs(point.positions.at(found - path.joint_names.begin()) - target));
    }
    std::cout << "held_joint=" << name << " nominal=" << target << " max_knot_offset=" << maximum
              << " limit=" << step.hold_tolerance << '\n';
  }
}

int run(const rclcpp::Node::SharedPtr & node) {
  const auto remote = node->declare_parameter<std::string>("config_node", "/handover_demo");
  const std::string cup_id = "demo_place_cup";
  auto parameters = std::make_shared<rclcpp::SyncParametersClient>(node, remote);
  if (!parameters->wait_for_service(10s)) throw std::runtime_error("Configuration node unavailable");
  const auto names = parameters->list_parameters({}, 0, 10s).names;
  const auto values = parameters->get_parameters(names, 10s);
  if (names.empty() || values.size() != names.size()) throw std::runtime_error("Incomplete parameter snapshot");
  std::map<std::string, rclcpp::Parameter> snapshot;
  for (const auto & value : values) snapshot.emplace(value.get_name(), value);
  rclcpp::NodeOptions options;
  options.use_global_arguments(false).parameter_overrides(values);
  options.start_parameter_services(false).start_parameter_event_publisher(false);
  auto settings = std::make_shared<rclcpp::Node>("cup_preparation_settings", options);
  const auto config = demo_1f::load_config(*settings);
  for (const auto & name : settings->list_parameters({}, 0).names) {
    if (!snapshot.count(name)) throw std::runtime_error("Live configuration lacks " + name);
  }

  const auto & sequence = config.motion_sequences.at("cup_left_to_right");
  const auto & steps = sequence.steps;
  if (!config.smooth_motion || !sequence.validation_error.empty() || steps.size() != 2 ||
      !steps[0].blend_with_next || steps[1].blend_with_next || steps[1].hold_arm != "left" ||
      steps[0].left_target != steps[1].left_target || steps[1].hold_tolerance != .001) {
    throw std::runtime_error("Live B3 configuration must contain the two blended steps and 0.001 rad left corridor");
  }
  for (const auto & step : steps) {
    if (step.type != "dual_joint" || step.pipeline != "ompl" ||
        step.planner != "RRTConnectkConfigDefault" || step.velocity_scale != .3 || step.acceleration_scale != .3) {
      throw std::runtime_error("Live B3 planner/scales do not match the OMPL preparation configuration");
    }
  }
  const auto urdf = urdf::parseURDF(snapshot.at("robot_description").as_string());
  auto semantic = std::make_shared<srdf::Model>();
  if (!urdf || !semantic->initString(*urdf, snapshot.at("robot_description_semantic").as_string())) {
    throw std::runtime_error("Invalid live robot model");
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, semantic);
  if (snapshot.count("use_sim_time")) node->set_parameter(snapshot.at("use_sim_time"));
  demo_1f::msg::DemoStatus::ConstSharedPtr status;
  auto status_subscription = node->create_subscription<demo_1f::msg::DemoStatus>(
      remote + "/status", rclcpp::QoS(1).reliable().transient_local(),
      [&](demo_1f::msg::DemoStatus::ConstSharedPtr sample) { status = sample; });
  struct Sample {
    double position;
    double velocity;
    std::chrono::steady_clock::time_point received;
  };
  std::map<std::string, Sample> feedback;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      config.joint_states_topic, rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::JointState::ConstSharedPtr sample) {
        if (sample->name.size() != sample->position.size()) return;
        const double age = (node->now() - rclcpp::Time(sample->header.stamp,
            node->get_clock()->get_clock_type())).seconds();
        if (!std::isfinite(age) || age < -.1 || age > config.state_max_age) return;
        for (size_t i = 0; i < sample->name.size(); ++i) {
          feedback[sample->name[i]] = {sample->position[i],
              i < sample->velocity.size() ? sample->velocity[i] :
                  std::numeric_limits<double>::quiet_NaN(), std::chrono::steady_clock::now()};
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
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
              found->second.received).count() > config.state_max_age) {
        missing = name;
        break;
      }
    }
    if (missing.empty()) break;
    std::this_thread::sleep_for(10ms);
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
  if (!missing.empty()) throw std::runtime_error("Missing fresh joint feedback: " + missing);
  auto scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!scene_client->wait_for_service(10s)) throw std::runtime_error("Planning scene unavailable");
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023;
  auto pending = scene_client->async_send_request(request);
  wait(node, pending, 10);
  auto message = pending.get()->scene;
  if (std::none_of(message.world.collision_objects.begin(), message.world.collision_objects.end(),
          [](const auto & object) { return object.id.rfind("basket_", 0) == 0; })) {
    throw std::runtime_error("Planning scene has no basket_* collision objects");
  }
  planning_scene::PlanningScene scene(model);
  if (message.is_diff || message.robot_state.is_diff || !scene.setPlanningSceneMsg(message)) {
    throw std::runtime_error("A complete planning scene is required");
  }
  const auto & joints = message.robot_state.joint_state;
  if (joints.name.size() != joints.position.size()) throw std::runtime_error("Malformed scene joint state");
  for (const auto & name : model->getVariableNames()) {
    if (model->getJointOfVariable(name)->getMimic()) continue;
    const auto found = std::find(joints.name.begin(), joints.name.end(), name);
    if (found == joints.name.end() || !std::isfinite(joints.position[found - joints.name.begin()])) {
      throw std::runtime_error("Scene lacks a finite explicit starting joint: " + name);
    }
  }
  // Drain feedback queued while constructing the local planning scene before checking freshness.
  rclcpp::spin_some(node);
  auto state = scene.getCurrentState();
  for (const auto & name : model->getVariableNames()) {
    if (model->getJointOfVariable(name)->getMimic()) continue;
    const auto & sample = feedback.at(name);
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.received).count() >
        config.state_max_age) throw std::runtime_error("Feedback became stale: " + name);
    bool gripper = false;
    for (const auto & entry : config.arms) gripper = gripper || name == entry.second.gripper_joint;
    if (!gripper && (!std::isfinite(sample.velocity) || std::abs(sample.velocity) > config.stopped_velocity)) {
      throw std::runtime_error("Joint is moving or has unknown velocity: " + name);
    }
    state.setVariablePosition(name, sample.position);
  }
  const auto * group = model->getJointModelGroup(config.coordinated_group);
  if (!group) throw std::runtime_error("Missing coordinated group");
  std::vector<double> active_positions;
  for (const auto * joint : group->getActiveJointModels()) {
    for (const auto & name : joint->getVariableNames()) active_positions.push_back(state.getVariablePosition(name));
  }
  state.setJointGroupActivePositions(group, active_positions);
  state.update();
  scene.setCurrentState(state);
  const auto * attached = state.getAttachedBody(cup_id);
  if (!attached || attached->getAttachedLinkName() != config.arms.at("left").tcp ||
      attached->getShapes().empty()) {
    throw std::runtime_error("Cup geometry must already be attached to the left TCP: " + cup_id);
  }

  if (scene.getWorld()->hasObject(cup_id)) throw std::runtime_error("Cup duplicated in world");
  std::vector<std::string> controlled;
  for (const auto * joint : group->getActiveJointModels()) {
    for (const auto & name : joint->getVariableNames()) controlled.push_back(name);
  }
  const auto measured = state;
  const auto verify_stationary = [&] {
    rclcpp::spin_some(node);
    if (!status || status->phase != "PICK_COMPLETE" || status->task_state != "SUCCEEDED" ||
        status->direction != "left") {
      throw std::runtime_error("Demo status must confirm the successful left-hand menu 6 endpoint");
    }
    for (const auto & name : model->getVariableNames()) {
      if (model->getJointOfVariable(name)->getMimic()) continue;
      const auto & sample = feedback.at(name);
      if (!std::isfinite(sample.position) ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.received).count() > config.state_max_age ||
          std::abs(sample.position - measured.getVariablePosition(name)) > config.continuous_start_tolerance) {
        throw std::runtime_error("Measured start changed or became stale: " + name);
      }
      bool gripper = false;
      for (const auto & [side, arm] : config.arms) { (void)side; gripper = gripper || arm.gripper_joint == name; }
      if (!gripper && (!std::isfinite(sample.velocity) || std::abs(sample.velocity) > config.stopped_velocity)) {
        throw std::runtime_error("Robot moved during the read-only check: " + name);
      }
    }
    auto pending_scene = scene_client->async_send_request(request);
    wait(node, pending_scene, 10);
    const auto live = pending_scene.get()->scene;
    if (live.world != message.world || live.allowed_collision_matrix != message.allowed_collision_matrix ||
        live.fixed_frame_transforms != message.fixed_frame_transforms || live.link_padding != message.link_padding ||
        live.link_scale != message.link_scale || live.robot_state.attached_collision_objects != message.robot_state.attached_collision_objects) {
      throw std::runtime_error("World, attachment or ACM changed during planning");
    }
  };
  std::cout << std::setprecision(17) << "plan_only=true config_node=" << remote << " group=" << config.coordinated_group << '\n';
  std::cout << "PRECONDITION: operator selected the actual stationary menu 6 end state; no IK reconstruction.\n";
  for (const auto & name : model->getVariableNames()) std::cout << "initial_joint " << name << "=" << state.getVariablePosition(name) << '\n';
  for (const auto & object : message.world.collision_objects) std::cout << "world=" << object.id << '\n';
  for (const auto & object : message.robot_state.attached_collision_objects) std::cout << "attached=" << object.object.id << " link=" << object.link_name << '\n';
  auto client = rclcpp_action::create_client<Move>(node, config.move_group_action);
  if (!client->wait_for_action_server(10s)) throw std::runtime_error("MoveGroup unavailable");
  std::vector<moveit_msgs::msg::RobotTrajectory> segments;
  for (const auto & step : steps) {
    verify_stationary();
    const auto goal = preparation_goal(config, step, message, state);
    std::cout << "planning=" << step.name << " pipeline=" << step.pipeline << "/" << step.planner
              << " hold=" << step.hold_arm << " tolerance=" << step.hold_tolerance << std::endl;
    auto accepted = client->async_send_goal(goal);
    wait(node, accepted, 10);
    auto handle = accepted.get();
    if (!handle) throw std::runtime_error("Planning goal rejected");
    auto pending_plan = client->async_get_result(handle);
    try { wait(node, pending_plan, std::max(60., goal.request.allowed_planning_time + 10)); }
    catch (...) { client->async_cancel_goal(handle); throw; }
    const auto response = pending_plan.get();
    if (!response.result) throw std::runtime_error("Missing planning result");
    const auto & result = *response.result;
    std::cout << "plan=" << step.name << " MoveIt=" << result.error_code.val << " reason=" << result.error_code.message << std::endl;
    if (response.code != rclcpp_action::ResultCode::SUCCEEDED || result.error_code.val != Codes::SUCCESS ||
        !result.executed_trajectory.joint_trajectory.points.empty() ||
        !result.executed_trajectory.multi_dof_joint_trajectory.points.empty()) {
      throw std::runtime_error("Planning failed or server reported unexpected execution");
    }
    const auto & path = result.planned_trajectory.joint_trajectory;
    // Smoothing consumes geometric positions and regenerates timing, velocity and acceleration.
    const std::set<std::string> names(path.joint_names.begin(), path.joint_names.end());
    const std::set<std::string> expected(controlled.begin(), controlled.end());
    if (names.empty() || names != expected || names.size() != path.joint_names.size() ||
        path.points.empty() || !result.planned_trajectory.multi_dof_joint_trajectory.joint_names.empty() ||
        !result.planned_trajectory.multi_dof_joint_trajectory.points.empty()) {
      throw std::runtime_error("Invalid preparation joint group or empty geometric path");
    }
    for (const auto & point : path.points) {
      if (point.positions.size() != names.size() ||
          !std::all_of(point.positions.begin(), point.positions.end(),
              [](double value) { return std::isfinite(value); })) {
        throw std::runtime_error("Preparation path needs complete finite joint positions");
      }
    }
    for (size_t i = 0; i < path.joint_names.size(); ++i) {
      if (std::abs(path.points.front().positions[i] - state.getVariablePosition(path.joint_names[i])) > config.continuous_start_tolerance) {
        throw std::runtime_error("Planner changed the requested start");
      }
    }
    demo_1f::validate_held_arm_path(path, step, config);
    print_path("raw " + step.name, path);
    print_hold_deviation(path, step, config);
    segments.push_back(result.planned_trajectory);
    state.setVariablePositions(path.joint_names, path.points.back().positions);
    state.zeroVelocities(); state.zeroAccelerations(); state.update();
  }
  const auto path = demo_1f::smooth_segments(segments, steps, config, model);
  demo_1f::validate_smooth_path(path, steps, config, scene);
  verify_stationary();
  print_path("smoothed", path.trajectory.joint_trajectory);
  print_hold_deviation(path.trajectory.joint_trajectory, steps[1], config, path.segment_end_times.at(0));
  for (size_t i = 0; i < path.segment_end_times.size(); ++i) std::cout << "segment=" << steps[i].name << " end=" << path.segment_end_times[i] << '\n';
  std::cout << "PASS: planned, smoothed and validated only; no trajectory was sent.\n";
  std::cout << "Collision/dynamics checks are sampled; the explicit held-arm quintic corridor uses a conservative continuous bound.\n";
  return 0;
}
} // namespace

int main(int argc, char ** argv) {
  demo_1f::initialize_moveit_logging();
  rclcpp::init(argc, argv);
  int code = 1;
  try { code = run(std::make_shared<rclcpp::Node>("check_cup_preparation")); }
  catch (const std::exception & error) { std::cerr << "FAIL: " << error.what() << '\n'; }
  rclcpp::shutdown();
  return code;
}

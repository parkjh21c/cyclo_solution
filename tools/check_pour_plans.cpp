#include "demo_1f/motions/common/cup_config.hpp"
#include "demo_1f/motions/pour/pour_geometry.hpp"
#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/round_trip_path.hpp"
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
#include <thread>

namespace {
using namespace std::chrono_literals;
using Move = moveit_msgs::action::MoveGroup;
using Codes = moveit_msgs::msg::MoveItErrorCodes;
using demo_1f::Config;

std::string error_name(int code) {
  switch (code) {
    case Codes::SUCCESS: return "SUCCESS";
    case Codes::FAILURE: return "FAILURE";
    case Codes::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    case Codes::PLANNING_FAILED: return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case Codes::TIMED_OUT: return "TIMED_OUT";
    case Codes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case Codes::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
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

geometry_msgs::msg::Pose pose_message(const Eigen::Isometry3d & transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// Match the ordinary pose/LIN branch of runtime/execution/moveit_goal.cpp.
Move::Goal planning_goal(const Config & config, const moveit_msgs::msg::PlanningScene & scene,
    const moveit_msgs::msg::RobotState & start, const geometry_msgs::msg::Pose & pose,
    const std::string & planner, double speed, double acceleration) {
  Move::Goal goal;
  auto & request = goal.request;
  const auto & arm = config.arms.at("right");
  request.group_name = arm.group;
  request.pipeline_id = config.pipeline;
  request.planner_id = planner;
  request.num_planning_attempts = config.pipeline == "ompl" ? config.ompl_planning_attempts : 1;
  request.allowed_planning_time = config.planning_time;
  request.max_velocity_scaling_factor = speed;
  request.max_acceleration_scaling_factor = acceleration;
  request.start_state = start;
  request.start_state.is_diff = false;
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::PositionConstraint position;
  position.header.frame_id = config.planning_frame;
  position.link_name = arm.tcp;
  position.weight = 1;
  shape_msgs::msg::SolidPrimitive region;
  region.type = region.SPHERE;
  region.dimensions = {config.pose_position_tolerance};
  position.constraint_region.primitives.push_back(region);
  position.constraint_region.primitive_poses.push_back(pose);
  constraints.position_constraints.push_back(position);
  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.header.frame_id = config.planning_frame;
  orientation.link_name = arm.tcp;
  orientation.orientation = pose.orientation;
  orientation.absolute_x_axis_tolerance = config.pose_orientation_tolerance;
  orientation.absolute_y_axis_tolerance = config.pose_orientation_tolerance;
  orientation.absolute_z_axis_tolerance = config.pose_orientation_tolerance;
  orientation.weight = 1;
  constraints.orientation_constraints.push_back(orientation);
  request.goal_constraints.push_back(constraints);
  goal.planning_options.plan_only = true;
  goal.planning_options.replan = false;
  goal.planning_options.look_around = false;
  // A complete private scene snapshot prevents mixing live and hypothetical joint states.
  goal.planning_options.planning_scene_diff = scene;
  goal.planning_options.planning_scene_diff.is_diff = false;
  goal.planning_options.planning_scene_diff.robot_state = start;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = false;
  return goal;
}

bool plan(const rclcpp::Node::SharedPtr & node,
    const rclcpp_action::Client<Move>::SharedPtr & client, const Config & config,
    const moveit_msgs::msg::PlanningScene & scene, moveit::core::RobotState & state,
    const geometry_msgs::msg::Pose & target, const std::string & label,
    const std::string & planner, double speed, double acceleration) {
  moveit_msgs::msg::RobotState start;
  moveit::core::robotStateToRobotStateMsg(state, start);
  const auto goal = planning_goal(config, scene, start, target, planner, speed, acceleration);
  std::cout << label << " request=" << goal.request.pipeline_id << '/' << planner
            << " plan_only=true speed=" << speed << " acceleration=" << acceleration
            << " planning_time=" << config.planning_time
            << " position_tolerance=" << config.pose_position_tolerance
            << " orientation_tolerance=" << config.pose_orientation_tolerance << '\n';
  std::cout << label << " target=" << target.position.x << ',' << target.position.y << ','
            << target.position.z << ',' << target.orientation.x << ',' << target.orientation.y << ','
            << target.orientation.z << ',' << target.orientation.w << '\n';
  auto accepted = client->async_send_goal(goal);
  wait(node, accepted, 10);
  const auto handle = accepted.get();
  if (!handle) throw std::runtime_error(label + ": planning goal rejected");
  auto pending = client->async_get_result(handle);
  try {
    wait(node, pending, std::max(60., config.planning_time + 10));
  } catch (...) {
    auto cancellation = client->async_cancel_goal(handle);
    try { wait(node, cancellation, 5); } catch (...) {}
    throw;
  }
  const auto response = pending.get();
  if (!response.result) throw std::runtime_error(label + ": missing planning result");
  const auto & result = *response.result;
  std::cout << label << " code=" << result.error_code.val
            << " name=" << error_name(result.error_code.val)
            << " action_status=" << static_cast<int>(response.code)
            << " planning_seconds=" << result.planning_time
            << " source=" << std::quoted(result.error_code.source)
            << " reason=" << std::quoted(result.error_code.message.empty()
                ? (result.error_code.val == Codes::SUCCESS ? "success" :
                    "MoveIt supplied no reason; inspect move_group logs for this request")
                : result.error_code.message) << std::endl;
  if (!result.executed_trajectory.joint_trajectory.points.empty() ||
      !result.executed_trajectory.multi_dof_joint_trajectory.points.empty()) {
    throw std::runtime_error("Server reported execution for a plan-only request; aborting checks");
  }
  if (response.code != rclcpp_action::ResultCode::SUCCEEDED ||
      result.error_code.val != Codes::SUCCESS) return false;
  demo_1f::validate_round_trip_path(result.planned_trajectory, config.arms.at("right").joints);
  const auto & path = result.planned_trajectory.joint_trajectory;
  for (size_t i = 0; i < path.joint_names.size(); ++i) {
    if (std::abs(path.points.front().positions[i] - state.getVariablePosition(path.joint_names[i])) >
        config.continuous_start_tolerance) {
      throw std::runtime_error(label + ": planner changed the requested starting state");
    }
  }
  state.setVariablePositions(path.joint_names, path.points.back().positions);
  state.zeroVelocities();
  state.zeroAccelerations();
  state.update();
  return true;
}

int run(const rclcpp::Node::SharedPtr & node, std::set<int, std::greater<int>> & remaining) {
  const auto remote = node->declare_parameter<std::string>("config_node", "/handover_demo");
  const auto frame = node->declare_parameter<std::string>("basket_frame", "");
  const double z = node->declare_parameter<double>("basket_z", std::numeric_limits<double>::quiet_NaN());
  const auto cup_id = node->declare_parameter<std::string>("attached_cup_id", "demo_place_cup");
  if (frame.empty() || !std::isfinite(z)) {
    throw std::runtime_error("Explicit basket_z and basket_frame required; historical values are unknown");
  }
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
  auto settings = std::make_shared<rclcpp::Node>("pour_plan_settings", options);
  const auto config = demo_1f::load_config(*settings);
  const auto approach = demo_1f::load_approach_config(*settings);
  const auto pour = demo_1f::load_pour_config(*settings);
  for (const auto & name : settings->list_parameters({}, 0).names) {
    if (!snapshot.count(name)) throw std::runtime_error("Live configuration lacks " + name);
  }
  if (frame != config.planning_frame) throw std::runtime_error("basket_frame must equal live planning_frame");
  if (pour.angle_deg != 130 || pour.fallback_angles_deg != std::vector<double>{115, 100}) {
    throw std::runtime_error("Live pour angles differ from 130/115/100; refusing to override production settings");
  }
  geometry_msgs::msg::PoseStamped::ConstSharedPtr observed_basket;
  auto basket_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/demo/basket_pose", rclcpp::QoS(1).reliable().transient_local(),
      [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) { observed_basket = pose; });
  const auto basket_deadline = std::chrono::steady_clock::now() + 5s;
  while (rclcpp::ok() && !observed_basket && std::chrono::steady_clock::now() < basket_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  if (!observed_basket) throw std::runtime_error("No retained /demo/basket_pose observation");
  const auto & bp = observed_basket->pose.position;
  const auto & bq = observed_basket->pose.orientation;
  const double basket_quaternion_norm = bq.x * bq.x + bq.y * bq.y + bq.z * bq.z + bq.w * bq.w;
  const double observed_yaw = std::atan2(2 * (bq.w * bq.z + bq.x * bq.y),
      1 - 2 * (bq.y * bq.y + bq.z * bq.z));
  const double basket_offset = std::hypot(std::hypot(bp.x - .412, bp.y + .227), bp.z - z);
  if (observed_basket->header.frame_id != frame || !std::isfinite(basket_offset) ||
      !std::isfinite(observed_yaw) || !std::isfinite(basket_quaternion_norm) ||
      std::abs(basket_quaternion_norm - 1) > .01 || basket_offset > .001 ||
      std::abs(std::remainder(observed_yaw - 92.3 * M_PI / 180, 2 * M_PI)) > .1 * M_PI / 180) {
    throw std::runtime_error("Current basket observation differs from the reproduction reference");
  }
  const auto urdf = urdf::parseURDF(snapshot.at("robot_description").as_string());
  auto semantic = std::make_shared<srdf::Model>();
  if (!urdf || !semantic->initString(*urdf, snapshot.at("robot_description_semantic").as_string())) {
    throw std::runtime_error("Invalid live robot model");
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, semantic);
  if (!model->hasLinkModel(config.arms.at("right").tcp)) throw std::runtime_error("Missing right TCP model");
  if (snapshot.count("use_sim_time")) node->set_parameter(snapshot.at("use_sim_time"));
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
  if (!attached || attached->getAttachedLinkName() != config.arms.at("right").tcp ||
      attached->getShapes().empty()) {
    throw std::runtime_error("Cup geometry must already be attached to the right TCP: " + cup_id);
  }
  if (!scene.knowsFrameTransform(frame)) throw std::runtime_error("Planning frame unavailable in scene");
  const auto tcp = pose_message(scene.getFrameTransform(frame).inverse() *
      state.getGlobalLinkTransform(config.arms.at("right").tcp));
  const auto & grasp = config.motion_sequences.at("cup_left_to_right_grasp");
  const auto hold = std::find_if(grasp.steps.begin(), grasp.steps.end(), [](const auto & step) {
    return step.type == "linear" && step.target.size() == 7;
  });
  if (hold == grasp.steps.end() || hold->frame != frame ||
      std::hypot(std::hypot(tcp.position.x - hold->target[0], tcp.position.y - hold->target[1]),
          tcp.position.z - hold->target[2]) > pour.hold_tolerance) {
    throw std::runtime_error("Scene is not at the transfer hold pose; cannot reproduce the pour approach");
  }
  geometry_msgs::msg::Pose basket;
  basket.position.x = .412;
  basket.position.y = -.227;
  basket.position.z = z;
  basket.orientation.z = std::sin(92.3 * M_PI / 360);
  basket.orientation.w = std::cos(92.3 * M_PI / 360);
  std::string reason;
  const auto geometry = demo_1f::plan_pour(tcp, basket, approach.cup_height, pour, reason);
  if (!geometry) throw std::runtime_error(reason);
  std::cout << std::setprecision(17) << "basket frame=" << frame << " x=0.412 y=-0.227 z=" << z
            << " yaw_deg=92.3 config_node=" << remote << " cup=" << cup_id << '\n';
  std::cout << "scene=" << std::quoted(message.name) << " world_objects=" << message.world.collision_objects.size()
            << " attachments=" << message.robot_state.attached_collision_objects.size()
            << " joint_stamp=" << joints.header.stamp.sec << '.' << joints.header.stamp.nanosec << '\n';
  for (const auto & object : message.world.collision_objects) {
    std::cout << "scene_object " << std::quoted(object.id) << " frame=" << object.header.frame_id
              << " xyz=" << object.pose.position.x << ',' << object.pose.position.y << ','
              << object.pose.position.z << '\n';
  }
  for (const auto & name : model->getVariableNames()) {
    std::cout << "initial_joint " << name << '=' << state.getVariablePosition(name) << '\n';
  }
  state.zeroVelocities();
  state.zeroAccelerations();
  auto client = rclcpp_action::create_client<Move>(node, config.move_group_action);
  if (!client->wait_for_action_server(10s)) throw std::runtime_error("MoveGroup unavailable");
  if (!plan(node, client, config, message, state, geometry->above, "pour_above", "PTP",
          pour.ptp_speed, pour.ptp_speed) ||
      (geometry->raise > 0 && !plan(node, client, config, message, state, geometry->lowered,
          "pour_lower", "LIN", pour.lin_speed, pour.lin_speed / 4))) {
    throw std::runtime_error("Approach planning failed; no valid pre-tilt start state");
  }
  for (const auto & name : model->getVariableNames()) {
    std::cout << "pre_tilt_joint " << name << '=' << state.getVariablePosition(name) << '\n';
  }
  const std::vector<geometry_msgs::msg::Pose> targets = {
      geometry->tilted, geometry->fallbacks.at(0), geometry->fallbacks.at(1)};
  bool success = true;
  for (size_t i = 0; i < targets.size(); ++i) {
    auto candidate = state;
    const auto angle = i == 0 ? pour.angle_deg : pour.fallback_angles_deg[i - 1];
    success = plan(node, client, config, message, candidate, targets[i],
        "pour_tilt angle=" + std::to_string(angle), "LIN", pour.tilt_speed,
        pour.tilt_acceleration) && success;
    remaining.erase(static_cast<int>(angle));
  }
  return success ? 0 : 1;
}
} // namespace

int main(int argc, char ** argv) {
  demo_1f::initialize_moveit_logging();
  rclcpp::init(argc, argv);
  int status = 2;
  std::set<int, std::greater<int>> remaining = {130, 115, 100};
  try {
    auto node = std::make_shared<rclcpp::Node>("check_pour_plans");
    status = run(node, remaining);
  } catch (const std::exception & error) {
    std::cerr << "CHECK_INCOMPLETE: " << error.what()
              << "; any angle without a result was NOT_REQUESTED or INCOMPLETE, not an IK failure\n";
    for (const int angle : remaining) {
      std::cerr << "pour_tilt angle=" << angle << " code=UNAVAILABLE status=INCOMPLETE reason="
                << std::quoted(error.what()) << '\n';
    }
  }
  rclcpp::shutdown();
  return status;
}

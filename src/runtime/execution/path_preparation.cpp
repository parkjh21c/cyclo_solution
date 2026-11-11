#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/trajectory_executor.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <moveit/robot_state/conversions.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace demo_1f {
void TrajectoryExecutor::load_robot_model() {
  const auto text = [this](const std::string & name) {
    return node_.has_parameter(name) ? node_.get_parameter(name).as_string()
                                     : node_.declare_parameter<std::string>(name, "");
  };
  const auto urdf_text = text("robot_description");
  const auto srdf_text = text("robot_description_semantic");
  const auto urdf = urdf::parseURDF(urdf_text);
  auto semantic = std::make_shared<srdf::Model>();
  if (!urdf || !semantic->initString(*urdf, srdf_text)) {
    throw std::runtime_error(
        "smooth_motion requires valid robot_description and robot_description_semantic");
  }
  robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf, semantic);
  if (!robot_model_->getJointModelGroup(config_.coordinated_group)) {
    throw std::runtime_error("Missing coordinated group in the smoothing robot model");
  }
  scene_template_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
}

bool TrajectoryExecutor::start_sequence(const std::vector<Step> & steps) {
  const auto previous = result().state;
  if (previous == ExecutionState::Running || previous == ExecutionState::Cancelling ||
      previous == ExecutionState::StopUnconfirmed) {
    return false;
  }
  auto op = std::make_shared<Operation>();
  set_operation(op);
  MotionSequence sequence;
  sequence.steps = steps;
  std::string reason = validate_motion_sequence(sequence, config_);
  for (const auto & step : steps) {
    if (step.type != "dual_joint" && step.type != "dual_joint_gripper" &&
        !(steps.size() == 1 && step.type == "joint_gripper")) {
      reason = "Continuous paths accept coordinated arm moves only";
    }
  }
  if (!config_.smooth_motion || !robot_model_ || !reason.empty() || !context_.ready(reason)) {
    op->result = {ExecutionState::Failed,
        "INVALID_COMMAND",
        reason.empty() ? "Continuous motion is not configured" : reason};
    publish_state();
    return false;
  }
  op->steps = steps;
  op->step = steps.back();
  op->step.timeout = 0;
  for (const auto & step : steps) {
    op->step.timeout += step.timeout;
  }
  op->result.phase = steps.front().name;
  try {
    request_scene(op, false);
  } catch (const std::exception & error) {
    op->result = {ExecutionState::Failed, "SCENE_UNAVAILABLE", error.what()};
  }
  publish_state();
  return op->result.state == ExecutionState::Running;
}

void TrajectoryExecutor::request_scene(
    const std::shared_ptr<Operation> & op, bool prepare_execution) {
  if (!scene_client_->service_is_ready()) {
    throw std::runtime_error("Planning scene service unavailable");
  }
  // There is no outstanding motion action while this read-only service is pending.
  op->action_terminal = true;
  op->action_success = false;
  op->cancel_action = {};
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023;
  scene_client_->async_send_request(request,
      [this, op, prepare_execution](
          rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future) {
        if (op->cancel_requested || op->result.state != ExecutionState::Running) {
          return;
        }
        try {
          const auto scene = future.get()->scene;
          if (op->steps.empty() && op->step.type == "gripper") {
            prepare_path(op, scene);
            return;
          }
          if (prepare_execution) {
            prepare_path(op, scene);
            return;
          }
          op->planned_start = scene.robot_state;
          op->planning_scene = scene_template_->diff();
          if (!op->planning_scene->setPlanningSceneMsg(scene)) {
            throw std::runtime_error("Cannot load the planning scene");
          }
          const auto snapshot = context_.snapshot();
          auto & joints = op->planned_start.joint_state;
          // Use the complete scene (lift, attached objects, ACM, etc.) and current controlled
          // joints.
          for (const auto & [side, arm] : config_.arms) {
            (void)side;
            auto controlled = arm.joints;
            controlled.push_back(arm.gripper_joint);
            for (const auto & name : controlled) {
              const auto found = std::find(joints.name.begin(), joints.name.end(), name);
              if (found == joints.name.end()) {
                joints.name.push_back(name);
                joints.position.push_back(snapshot.joints.at(name).position);
              } else {
                joints.position.at(found - joints.name.begin()) = snapshot.joints.at(name).position;
              }
            }
          }
          joints.velocity.clear();
          joints.effort.clear();
          op->planned_start.is_diff = false;
          send_move(op, true);
        } catch (const std::exception & error) {
          op->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", error.what()};
        }
      });
}

std::string TrajectoryExecutor::joint_goal_diagnostic(
    const Operation & op, const std::string & group_name) const {
  if (!op.planning_scene || op.joint_targets.empty()) return {};
  const auto * group = robot_model_->getJointModelGroup(group_name);
  if (!group) return {};
  auto goal = op.planning_scene->getCurrentState();
  moveit::core::robotStateMsgToRobotState(op.planned_start, goal);
  std::vector<double> positions;
  for (const auto * joint : group->getActiveJointModels()) {
    for (const auto & name : joint->getVariableNames()) {
      const auto target = op.joint_targets.find(name);
      positions.push_back(target == op.joint_targets.end()
                              ? goal.getVariablePosition(name) : target->second);
    }
  }
  goal.setJointGroupActivePositions(group, positions); // Propagate mimic fingers too.
  goal.update();
  std::string detail;
  for (const auto & name : group->getVariableNames()) {
    const auto & bounds = robot_model_->getVariableBounds(name);
    const double value = goal.getVariablePosition(name);
    if (bounds.position_bounded_ &&
        (value < bounds.min_position_ - 1e-8 || value > bounds.max_position_ + 1e-8)) {
      detail += "; nominal goal outside joint bounds: " + name + "=" + std::to_string(value);
    }
  }
  collision_detection::CollisionRequest request;
  request.group_name = group_name;
  request.contacts = true;
  request.max_contacts = 10;
  collision_detection::CollisionResult collision;
  op.planning_scene->checkCollision(request, collision, goal);
  for (const auto & contact : collision.contacts) {
    detail += "; nominal goal collision: " + contact.first.first + " / " + contact.first.second;
  }
  if (!detail.empty()) {
    for (const auto & object : op.planned_start.attached_collision_objects) {
      detail += "; attached object: " + object.object.id + " @ " + object.link_name;
    }
  }
  return detail;
}

void TrajectoryExecutor::prepare_path(
    const std::shared_ptr<Operation> & op, const moveit_msgs::msg::PlanningScene & scene_message) {
  const auto model = robot_model_;
  const auto scene_template = scene_template_;
  const auto config = config_;
  auto segments = op->planned_segments;
  auto steps = op->steps;
  const bool gripper = op->step.type == "gripper";
  const auto snapshot = context_.snapshot();
  for (const auto & [side, arm] : config_.arms) {
    (void)side;
    auto controlled = arm.joints;
    controlled.push_back(arm.gripper_joint);
    for (const auto & name : controlled) {
      op->preparation_positions[name] = snapshot.joints.at(name).position;
    }
  }
  if (gripper) {
    const auto & arm = config_.arms.at(op->step.arm);
    moveit_msgs::msg::RobotTrajectory segment;
    segment.joint_trajectory.joint_names = {arm.gripper_joint};
    trajectory_msgs::msg::JointTrajectoryPoint initial, target;
    initial.positions = {snapshot.joints.at(arm.gripper_joint).position};
    target.positions = op->step.target;
    segment.joint_trajectory.points = {initial, target};
    segments = {segment};
    steps = {op->step};
  }
  // Dense collision validation runs off the ROS callback thread for arms AND grippers.
  op->preparation_started = Clock::now();
  op->preparation = std::async(std::launch::async,
      [model, scene_template, config, segments, steps, scene_message, snapshot, gripper]() {
        // Reuse immutable collision geometry; each child gets a fresh complete scene.
        auto scene = scene_template->diff();
        if (!scene->setPlanningSceneMsg(scene_message)) {
          throw std::runtime_error("Cannot load the planning scene");
        }
        if (gripper) {
          auto & state = scene->getCurrentStateNonConst();
          const auto & variables = model->getVariableNames();
          for (size_t i = 0; i < variables.size(); ++i) {
            if (model->getJointOfVariable(i)->getMimic()) {
              continue;
            }
            const auto measured = snapshot.joints.find(variables[i]);
            if (measured != snapshot.joints.end()) {
              state.setVariablePosition(variables[i], measured->second.position);
            }
          }
          state.update();
        }
        auto path = smooth_segments(segments, steps, config, model);
        validate_smooth_path(path, steps, config, *scene);
        return path;
      });
}

void TrajectoryExecutor::update_path_phase(const std::shared_ptr<Operation> & op) {
  if (!op->executing_path || op->path.trajectory.joint_trajectory.points.empty()) {
    return;
  }
  const auto snapshot = context_.snapshot();
  const auto & trajectory = op->path.trajectory.joint_trajectory;
  double closest = std::numeric_limits<double>::infinity();
  size_t best = op->progress_point;
  const size_t last = std::min(trajectory.points.size(), op->progress_point + 100);
  for (size_t point = op->progress_point; point < last; ++point) {
    double error = 0;
    for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
      const auto found = snapshot.joints.find(trajectory.joint_names[joint]);
      if (found == snapshot.joints.end() || !context_.joint_fresh(found->second)) {
        return;
      }
      error += std::pow(found->second.position - trajectory.points[point].positions[joint], 2);
    }
    if (error < closest) {
      closest = error;
      best = point;
    }
  }
  op->progress_point = best;
  op->result.phase = op->steps.at(op->path.point_steps.at(best)).name;
}
} // namespace demo_1f

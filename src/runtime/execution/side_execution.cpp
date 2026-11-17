#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/trajectory_executor.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace demo_1f {
namespace {
// Still commanding or confirming a stop.
bool live(ExecutionState state) {
  return state == ExecutionState::Running || state == ExecutionState::Cancelling;
}

// Joint-space length: sum of the largest joint change between consecutive points.
double path_length(const trajectory_msgs::msg::JointTrajectory & path) {
  double length = 0;
  for (size_t i = 1; i < path.points.size(); ++i) {
    double step = 0;
    for (size_t j = 0; j < path.joint_names.size(); ++j) {
      step = std::max(step, std::abs(path.points[i].positions[j] - path.points[i - 1].positions[j]));
    }
    length += step;
  }
  return length;
}

double duration_of(const trajectory_msgs::msg::JointTrajectory & path) {
  return path.points.empty() ? 0.0 : rclcpp::Duration(path.points.back().time_from_start).seconds();
}

// Measured arm and gripper joints over the scene's robot state, as main planning uses.
void apply_measured(
    planning_scene::PlanningScene & scene, const Config & config, const Snapshot & snapshot) {
  auto & state = scene.getCurrentStateNonConst();
  for (const auto & [side, arm] : config.arms) {
    (void)side;
    auto joints = arm.joints;
    joints.push_back(arm.gripper_joint);
    for (const auto & name : joints) {
      const auto found = snapshot.joints.find(name);
      if (found != snapshot.joints.end()) state.setVariablePosition(name, found->second.position);
    }
  }
  state.update();
}
} // namespace

bool TrajectoryExecutor::side_live() const {
  return side_ && side_->result.state == ExecutionState::Running;
}

bool TrajectoryExecutor::side_blocks(const Step & step, std::string & reason) const {
  if (!side_) return false;
  if (side_->result.state == ExecutionState::Cancelling) {
    reason = "Side arm stop is being confirmed: " + side_->arm;
    return true;
  }
  if (side_->result.state != ExecutionState::Running) return false;
  // These reach the side arm's controller or the coordinated group and would replace its goal.
  if (step.type == "dual_joint" || step.type == "dual_joint_gripper" ||
      step.type == "joint_group" || step.type == "joint_gripper" || step.arm == side_->arm) {
    reason = "SIDE_ARM_BUSY: " + side_->arm + " arm is returning (" + step.type + " " +
             step.name + ")";
    return true;
  }
  return false;
}

ExecutionResult TrajectoryExecutor::side_result() const {
  if (!side_) return {};
  auto outcome = side_->result;
  outcome.retryable_before_motion =
      outcome.state == ExecutionState::Failed && (!side_->sent || side_->rejected);
  return outcome;
}

bool TrajectoryExecutor::start_side(const std::vector<Step> & steps) {
  if (side_ && (live(side_->result.state) ||
                   side_->result.state == ExecutionState::StopUnconfirmed)) {
    RCLCPP_WARN(node_.get_logger(), "Side start refused: %s side operation is %s",
        side_->arm.c_str(), to_string(side_->result.state));
    return false;
  }
  auto side = std::make_shared<SideOperation>();
  side->steps = steps;
  side->arm = steps.empty() ? std::string() : steps.front().arm;
  for (const auto & step : steps) side->timeout += step.timeout;
  if (side_ && side_->preparation.valid() &&
      side_->preparation.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    retired_sides_.push_back(side_);
  }
  side_ = side;
  const auto refuse = [&](const std::string & code, const std::string & detail) {
    side->result = {ExecutionState::Failed, code, detail};
    RCLCPP_WARN(node_.get_logger(), "Side %s not started: %s: %s", side->arm.c_str(),
        code.c_str(), detail.c_str());
    publish_state();
    return false;
  };
  MotionSequence sequence;
  sequence.steps = steps;
  std::string reason = validate_motion_sequence(sequence, config_);
  if (steps.empty()) reason = "No side steps";
  for (const auto & step : steps) {
    if (step.type != "joint" || step.arm != side->arm || !config_.arms.count(step.arm) ||
        step.target.size() != config_.arms.at(step.arm).joints.size()) {
      reason = "Side operation takes one arm's joint steps only";
    }
  }
  if (!reason.empty()) return refuse("INVALID_COMMAND", reason);
  const auto & arm = config_.arms.at(side->arm);
  if (!config_.smooth_motion || !robot_model_ || !scene_client_->service_is_ready() ||
      !joint_clients_.at(side->arm)->action_server_is_ready()) {
    return refuse("SIDE_NOT_READY",
        "Side motion needs smooth_motion, a planning scene and " + arm.trajectory_action);
  }
  const auto op = operation_;
  if (op && (op->result.state == ExecutionState::Cancelling ||
                op->result.state == ExecutionState::StopUnconfirmed)) {
    return refuse("SIDE_NOT_READY", "Main stop is not confirmed");
  }
  if (op && op->result.state == ExecutionState::Running) {
    const auto & step = op->step;
    if (op->round_trip || !op->paired_steps.empty() || !op->steps.empty() ||
        step.type == "dual_joint" || step.type == "dual_joint_gripper" ||
        step.type == "joint_group" || step.type == "joint_gripper" || step.arm == side->arm) {
      return refuse("SIDE_ARM_BUSY", "Main operation uses the " + side->arm + " arm: " + step.name);
    }
    // MoveIt plans and executes this one internally: its path cannot be pair-checked.
    const bool unknown_path = !op->planning_only && op->dispatched_path.points.empty() &&
        (step.type == "joint" || step.type == "pose" || step.type == "linear");
    if (unknown_path) return refuse("MAIN_PATH_UNKNOWN", "Main path is not known: " + step.name);
  }
  if (!context_.ready(reason)) return refuse("ROBOT_NOT_READY", reason);
  const auto snapshot = context_.snapshot();
  const auto & last = steps.back();
  bool home = true;
  for (size_t i = 0; i < arm.joints.size(); ++i) {
    const auto found = snapshot.joints.find(arm.joints[i]);
    if (found == snapshot.joints.end() ||
        !context_.joint_ready_for_motion(arm.joints[i], found->second, reason)) {
      return refuse("ROBOT_NOT_READY", reason.empty() ? "Missing " + arm.joints[i] : reason);
    }
    home = home && std::abs(found->second.position - last.target[i]) <= last.tolerance;
  }
  side->result.phase = steps.front().name;
  if (home) {
    side->result = {ExecutionState::Succeeded, {}, "Already at " + last.name};
    side->result.last_completed_step = last.name;
    RCLCPP_INFO(node_.get_logger(), "Side %s already at %s; nothing sent", side->arm.c_str(),
        last.name.c_str());
    publish_state();
    return true;
  }
  if (op && op->result.state == ExecutionState::Running) op->side_linked = true;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023;
  scene_client_->async_send_request(request,
      [this, side](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future) {
        if (side_ != side || side->result.state != ExecutionState::Running) return;
        try {
          prepare_side(side, future.get()->scene);
        } catch (const std::exception & error) {
          side->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", error.what()};
        }
      });
  RCLCPP_INFO(node_.get_logger(), "Side %s started: %zu step(s) to %s", side->arm.c_str(),
      steps.size(), last.name.c_str());
  publish_state();
  return true;
}

void TrajectoryExecutor::prepare_side(
    const std::shared_ptr<SideOperation> & side, const moveit_msgs::msg::PlanningScene & message) {
  const auto snapshot = context_.snapshot();
  const auto & arm = config_.arms.at(side->arm);
  // Straight joint-space segments from the measured pose through each target.
  std::vector<moveit_msgs::msg::RobotTrajectory> segments;
  std::vector<double> from;
  for (const auto & name : arm.joints) {
    from.push_back(snapshot.joints.at(name).position);
    side->preparation_positions[name] = from.back();
  }
  for (const auto & step : side->steps) {
    moveit_msgs::msg::RobotTrajectory segment;
    segment.joint_trajectory.joint_names = arm.joints;
    trajectory_msgs::msg::JointTrajectoryPoint start, target;
    start.positions = from;
    target.positions = step.target;
    segment.joint_trajectory.points = {start, target};
    segments.push_back(segment);
    from = step.target;
  }
  // The main arm's dispatched path, conservatively whole, joins the pair check.
  trajectory_msgs::msg::JointTrajectory main_path;
  if (operation_ && operation_->result.state == ExecutionState::Running) {
    main_path = operation_->dispatched_path;
  }
  side->preparation_started = Clock::now();
  side->preparation = std::async(std::launch::async,
      [model = robot_model_, scene_template = scene_template_, config = config_,
          steps = side->steps, segments, message, snapshot, main_path]() {
        auto scene = scene_template->diff();
        if (!scene->setPlanningSceneMsg(message)) {
          throw std::runtime_error("Cannot load the planning scene");
        }
        apply_measured(*scene, config, snapshot);
        SidePlan plan;
        plan.path = smooth_segments(segments, steps, config, model);
        validate_smooth_path(plan.path, steps, config, *scene);
        if (!main_path.points.empty()) {
          const auto started = Clock::now();
          const auto & side_path = plan.path.trajectory.joint_trajectory;
          plan.pairs.collision = check_path_pairs(*scene, config, main_path.joint_names,
              sample_joint_path(main_path, config.side_pair_step), side_path.joint_names,
              sample_joint_path(side_path, config.side_pair_step), plan.pairs.pairs);
          plan.pairs.seconds = elapsed(started);
        }
        return plan;
      });
}

void TrajectoryExecutor::send_side(const std::shared_ptr<SideOperation> & side) {
  std::string reason;
  if (!context_.ready(reason)) throw std::runtime_error(reason);
  const auto snapshot = context_.snapshot();
  const auto & trajectory = side->path.trajectory.joint_trajectory;
  if (trajectory.points.empty()) throw std::runtime_error("Empty side path");
  for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
    const auto & name = trajectory.joint_names[j];
    const auto & measured = snapshot.joints.at(name);
    const double planned = trajectory.points.front().positions[j];
    if (!context_.joint_ready_for_motion(name, measured, reason, true) ||
        std::abs(measured.position - planned) > config_.continuous_start_tolerance) {
      char text[128];
      std::snprintf(text, sizeof(text), " (planned %.3f, measured %.3f, tolerance %.3f)", planned,
          measured.position, config_.continuous_start_tolerance);
      throw std::runtime_error("Side arm moved during preparation: " + name + text + "; " + reason);
    }
  }
  const double duration = duration_of(trajectory);
  if (duration + config_.settle_time >= side->timeout - elapsed(side->started)) {
    throw std::runtime_error("Side timeout is too short for the prepared path");
  }
  const auto client = joint_clients_.at(side->arm);
  if (!client->action_server_is_ready()) {
    throw std::runtime_error("Side controller unavailable: " + config_.arms.at(side->arm).trajectory_action);
  }
  Joint::Goal goal;
  goal.trajectory = trajectory; // Arm joints only; the controller accepts partial goals.
  goal.trajectory.header.stamp = builtin_interfaces::msg::Time();
  const size_t generation = ++side->generation;
  rclcpp_action::Client<Joint>::SendGoalOptions options;
  options.goal_response_callback =
      [this, side, client, generation](rclcpp_action::ClientGoalHandle<Joint>::SharedPtr handle) {
        if (side_ != side || generation != side->generation) {
          if (handle) client->async_cancel_goal(handle);
          return;
        }
        if (!handle) {
          side->rejected = side->action_terminal = true;
          if (!side->cancel_requested && side->result.state == ExecutionState::Running) {
            side->result = {ExecutionState::Failed, "GOAL_REJECTED", side->arm + " side goal rejected"};
          }
          return;
        }
        side->cancel_action = [client, handle]() {
          client->async_cancel_goal(handle);
        };
        if (side->cancel_requested) side->cancel_action(); // Never orphan an accepted goal.
      };
  options.result_callback =
      [this, side, generation](const rclcpp_action::ClientGoalHandle<Joint>::WrappedResult & response) {
        if (side_ != side || generation != side->generation) return;
        side->action_terminal = true;
        side->cancel_action = {};
        side->action_success = response.code == rclcpp_action::ResultCode::SUCCEEDED &&
            response.result && response.result->error_code == Joint::Result::SUCCESSFUL;
        if (!side->action_success && !side->cancel_requested &&
            side->result.state == ExecutionState::Running) {
          cancel_side("EXECUTION_FAILED", side->arm + " side controller: " +
              (response.result ? response.result->error_string : std::string("no result")));
        }
      };
  client->async_send_goal(goal, options);
  side->sent = true;
  side->sent_at = Clock::now();
  side->result.phase = side->steps.back().name;
  RCLCPP_INFO(node_.get_logger(), "Side %s sent: %zu step(s), path %.2f rad, %zu points, %.1f s",
      side->arm.c_str(), side->steps.size(), path_length(trajectory), trajectory.points.size(),
      duration);
}

bool TrajectoryExecutor::settled_side(const std::shared_ptr<SideOperation> & side, bool check_target) {
  const auto & arm = config_.arms.at(side->arm);
  const auto & last = side->steps.back();
  const auto state = context_.snapshot();
  const auto after = side->cancel_requested ? side->cancelling : side->sent_at;
  std::string reason;
  for (size_t i = 0; i < arm.joints.size(); ++i) {
    const auto found = state.joints.find(arm.joints[i]);
    if (found == state.joints.end() ||
        !context_.joint_stationary(arm.joints[i], found->second, reason, after) ||
        (check_target && std::abs(found->second.position - last.target[i]) > last.tolerance)) {
      side->stable_since.reset();
      return false;
    }
  }
  if (!side->stable_since) side->stable_since = Clock::now();
  return elapsed(*side->stable_since) >= config_.settle_time;
}

void TrajectoryExecutor::cancel_side(const std::string & code, const std::string & reason) {
  const auto side = side_;
  if (!side || side->result.state != ExecutionState::Running) return;
  side->cancel_requested = true;
  side->cancel_code = code;
  side->cancel_reason = reason;
  side->cancelling = Clock::now();
  side->stable_since.reset();
  if (!side->sent || side->rejected) { // Nothing reached the controller: no stop to confirm.
    side->result = {code == "CANCELLED" ? ExecutionState::Cancelled : ExecutionState::Failed,
        code, reason};
    publish_state();
    return;
  }
  side->result = {ExecutionState::Cancelling, code, reason};
  if (side->cancel_action && !side->action_terminal) {
    try {
      side->cancel_action();
    } catch (const std::exception & error) {
      side->cancel_reason += "; " + std::string(error.what());
    }
  }
  publish_state();
}

void TrajectoryExecutor::tick_side() {
  for (auto it = retired_sides_.begin(); it != retired_sides_.end();) {
    if ((*it)->preparation.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        (*it)->preparation.get();
      } catch (const std::exception &) {
      }
      it = retired_sides_.erase(it);
    } else {
      ++it;
    }
  }
  const auto side = side_;
  if (!side) return;
  if (side->result.state == ExecutionState::Running && side->preparation.valid() &&
      side->preparation.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      auto plan = side->preparation.get();
      RCLCPP_INFO(node_.get_logger(), "Side %s prepared in %.3f s", side->arm.c_str(),
          elapsed(side->preparation_started));
      if (plan.pairs.pairs > 0) {
        RCLCPP_INFO(node_.get_logger(), "Side %s pair check: %zu pairs in %.3f s%s",
            side->arm.c_str(), plan.pairs.pairs, plan.pairs.seconds,
            plan.pairs.collision.empty() ? "" : ", collision");
      }
      side->path = std::move(plan.path);
      side->path_ready = true;
      if (!plan.pairs.collision.empty()) {
        side->result = {ExecutionState::Failed, "SIDE_PAIR_COLLISION",
            "Side path meets the main path: " + plan.pairs.collision};
      } else {
        send_side(side);
      }
    } catch (const std::exception & error) {
      if (side->result.state == ExecutionState::Running && !side->sent) {
        side->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", error.what()};
      }
    }
    if (side->result.state == ExecutionState::Failed) {
      RCLCPP_WARN(node_.get_logger(), "Side %s failed before motion: %s: %s", side->arm.c_str(),
          side->result.code.c_str(), side->result.detail.c_str());
    }
  }
  if (side->result.state == ExecutionState::Running) {
    if (elapsed(side->started) >= side->timeout) {
      if (side->sent) {
        cancel_side("TIMEOUT", "Side " + side->arm + " timed out");
      } else {
        side->result = {ExecutionState::Failed, "TIMEOUT", "Side preparation timed out"};
      }
    } else if (side->sent && side->action_terminal && side->action_success &&
               settled_side(side, true)) {
      side->result.state = ExecutionState::Succeeded;
      side->result.last_completed_step = side->steps.back().name;
      side->result.detail = "Reached " + side->steps.back().name;
      RCLCPP_INFO(node_.get_logger(), "Side %s arrived at %s: %.1f s after start, %.1f s moving",
          side->arm.c_str(), side->steps.back().name.c_str(), elapsed(side->started),
          elapsed(side->sent_at));
    }
  }
  if (side->result.state == ExecutionState::Cancelling) {
    if ((side->rejected || side->action_terminal) && settled_side(side, false)) {
      side->result = {side->cancel_code == "CANCELLED" ? ExecutionState::Cancelled
                                                       : ExecutionState::Failed,
          side->cancel_code, side->cancel_reason};
    } else if (elapsed(side->cancelling) >= config_.cancel_timeout) {
      // Latched until restart, as for the main operation.
      side->result = {ExecutionState::StopUnconfirmed, "STOP_UNCONFIRMED",
          side->cancel_reason + "; stop not confirmed"};
    }
  }
  couple_side();
}

void TrajectoryExecutor::couple_side() {
  const auto side = side_;
  const auto op = operation_;
  if (!side || !op) return;
  const auto state = side->result.state;
  // Main stop unconfirmed is a fault: everything stops.
  if (!op->side_notified && state == ExecutionState::Running &&
      op->result.state == ExecutionState::StopUnconfirmed) {
    op->side_notified = true;
    cancel_side("MAIN_STOP_UNCONFIRMED", "Main " + op->step.name + " stop not confirmed");
    return;
  }
  // Other main failures or cancels leave the side running: all path pairs were checked.
  if (!op->side_linked) return;
  const bool side_ended_after_motion = side->sent && !side->rejected &&
      (state == ExecutionState::Failed || state == ExecutionState::Cancelled ||
          state == ExecutionState::StopUnconfirmed);
  if (side_ended_after_motion && !side->main_notified &&
      op->result.state == ExecutionState::Running) {
    side->main_notified = true;
    cancel_main("SIDE_ARM_FAILED",
        side->arm + " side arm " + side->result.code + ": " + side->result.detail);
  }
}

void TrajectoryExecutor::gate_main(const std::shared_ptr<Operation> & op,
    const trajectory_msgs::msg::JointTrajectory & path, std::function<void()> send) {
  const auto side = side_;
  const bool needed = side && (side_live() || (op->side_linked && side->sent && !side->rejected));
  if (!needed) {
    op->dispatched_path = path;
    send();
    return;
  }
  // No action is outstanding while the path waits for its pair check.
  op->action_terminal = true;
  op->action_success = false;
  op->cancel_action = {};
  op->gate.emplace();
  op->gate->path = path;
  op->gate->send = std::move(send);
  op->side_linked = true;
  tick_gate(op);
}

void TrajectoryExecutor::tick_gate(const std::shared_ptr<Operation> & op) {
  if (!op->gate || op->cancel_requested || op->result.state != ExecutionState::Running) return;
  auto & gate = *op->gate;
  if (gate.check.valid() &&
      gate.check.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return; // Still checking.
  }
  const auto side = side_;
  const auto release = [&]() {
    auto send = std::move(gate.send);
    const double waited = elapsed(gate.started);
    op->dispatched_path = gate.path;
    op->gate.reset();
    if (waited > 0.05) {
      RCLCPP_INFO(node_.get_logger(), "Main %s waited %.2f s for the side arm",
          op->step.name.c_str(), waited);
    }
    try {
      send();
    } catch (const std::exception & error) {
      op->result = {ExecutionState::Failed, "SEND_FAILED", error.what()};
    }
  };
  if (gate.check.valid()) {
    PairOutcome outcome;
    try {
      outcome = gate.check.get();
    } catch (const std::exception & error) {
      op->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED",
          std::string("Pair check failed: ") + error.what()};
      return;
    }
    RCLCPP_INFO(node_.get_logger(), "Main %s pair check: %zu pairs in %.3f s%s",
        op->step.name.c_str(), outcome.pairs, outcome.seconds,
        outcome.collision.empty() ? "" : ", collision");
    if (outcome.collision.empty()) {
      release();
      return;
    }
    gate.waiting_side = true;
    RCLCPP_WARN(node_.get_logger(), "Main %s waits for the side arm to finish: %s",
        op->step.name.c_str(), outcome.collision.c_str());
  }
  if (gate.waiting_side) {
    if (side_live()) {
      op->result.phase = "WAIT_SIDE_ARM";
      op->result.detail = "Waiting for the side arm: main and side paths collide";
      op->completion_wait_reason = op->result.detail;
      return;
    }
    if (side && side->result.state == ExecutionState::Succeeded) restart_main(op);
    return; // A side failure after motion cancels this operation in couple_side().
  }
  if (!side || (!side_live() && (!side->sent || side->rejected))) {
    release(); // The side arm never moved.
    return;
  }
  if (!side->path_ready) {
    op->completion_wait_reason = "Waiting for the side arm path";
    return;
  }
  if (gate.scene_requested) return;
  if (!scene_client_->service_is_ready()) {
    op->result = {ExecutionState::Failed, "SCENE_UNAVAILABLE", "Planning scene service unavailable"};
    return;
  }
  gate.scene_requested = true;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023;
  scene_client_->async_send_request(request,
      [this, op](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture future) {
        if (operation_ != op || !op->gate || op->cancel_requested ||
            op->result.state != ExecutionState::Running || !side_) {
          return;
        }
        try {
          op->gate->check = std::async(std::launch::async,
              [model = robot_model_, scene_template = scene_template_, config = config_,
                  message = future.get()->scene, snapshot = context_.snapshot(),
                  main_path = op->gate->path,
                  side_path = side_->path.trajectory.joint_trajectory]() {
                PairOutcome outcome;
                const auto started = Clock::now();
                auto scene = scene_template->diff();
                if (!scene->setPlanningSceneMsg(message)) {
                  throw std::runtime_error("Cannot load the planning scene");
                }
                apply_measured(*scene, config, snapshot);
                // The side path is checked whole: a conservative stand-in for what remains.
                outcome.collision = check_path_pairs(*scene, config, main_path.joint_names,
                    sample_joint_path(main_path, config.side_pair_step), side_path.joint_names,
                    sample_joint_path(side_path, config.side_pair_step), outcome.pairs);
                outcome.seconds = elapsed(started);
                return outcome;
              });
        } catch (const std::exception & error) {
          op->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", error.what()};
        }
      });
}

void TrajectoryExecutor::restart_main(const std::shared_ptr<Operation> & op) {
  RCLCPP_INFO(node_.get_logger(), "Main %s waited %.2f s for the side arm; planning again",
      op->step.name.c_str(), elapsed(op->gate->started));
  op->gate.reset();
  op->planned_segments.clear();
  op->path = SmoothPath();
  op->preparation_positions.clear();
  op->joint_targets.clear();
  op->executing_path = false;
  op->progress_point = 0;
  op->dispatched_path = trajectory_msgs::msg::JointTrajectory();
  op->stable_since.reset();
  op->planning_scene.reset();
  op->planned_start = moveit_msgs::msg::RobotState();
  op->plan_attempts.clear();
  op->fallback_tries = 0;
  op->fallback_start.clear();
  op->side_linked = op->side_notified = false;
  op->result = {ExecutionState::Running, {}, {}, op->step.name};
  op->completion_wait_reason.clear();
  try {
    begin_main(op); // Same step; the original start time keeps the remaining timeout.
  } catch (const std::exception & error) {
    op->result = {ExecutionState::Failed, "SEND_FAILED", error.what()};
  }
}
} // namespace demo_1f

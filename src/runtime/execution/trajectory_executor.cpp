#include "demo_1f/runtime/execution/trajectory_executor.hpp"
#include "action_dispatch.hpp"
#include "demo_1f/runtime/config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace demo_1f {
namespace {
constexpr double HELD_GRIPPER_NOTE = 0.005; // rad: held gripper drift worth an INFO line

// True when a step opens or closes this gripper joint (moveit_goal: the step arm's gripper).
bool moves_gripper(const Config & config, const Step & step, const std::string & joint) {
  return (step.type == "gripper" || step.type == "joint_gripper" ||
             step.type == "dual_joint_gripper") &&
         config.arms.count(step.arm) && config.arms.at(step.arm).gripper_joint == joint;
}

std::string start_mismatch(
    const std::string & name, double planned, double measured, double tolerance) {
  char text[96];
  std::snprintf(text, sizeof(text), " (planned %.3f, measured %.3f, tolerance %.3f)", planned,
      measured, tolerance);
  return name + text;
}
} // namespace

TrajectoryExecutor::TrajectoryExecutor(
    rclcpp::Node & node, const Config & config, RobotContext & context)
    : node_(node), config_(config), context_(context) {
  move_client_ = rclcpp_action::create_client<Move>(&node, config.move_group_action);
  execute_client_ = rclcpp_action::create_client<Execute>(&node, "/execute_trajectory");
  scene_client_ = node.create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  planned_trajectory_publisher_ = node.create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/planned_trajectory", rclcpp::QoS(1).reliable());
  if (config_.smooth_motion) {
    load_robot_model();
  }
  for (const auto & [name, arm] : config.arms) {
    joint_clients_[name] = rclcpp_action::create_client<Joint>(&node, arm.trajectory_action);
    publishers_[name] = node.create_publisher<trajectory_msgs::msg::JointTrajectory>(
        arm.trajectory_topic, rclcpp::QoS(1).reliable());
  }
}

bool TrajectoryExecutor::joint_group_ready(const Step & step, std::string & reason) const {
  if (step.type != "joint_group") return true;
  if (robot_model_) {
    const auto * group = robot_model_->getJointModelGroup(step.group);
    if (!group || group->getVariableCount() != step.joint_names.size()) {
      reason = "Joint group does not match robot model: " + step.group;
      return false;
    }
    const auto & names = group->getVariableNames();
    for (const auto & name : step.joint_names) {
      if (std::find(names.begin(), names.end(), name) == names.end()) {
        reason = "Joint is not in planning group: " + name;
        return false;
      }
    }
  }
  const auto snapshot = context_.snapshot();
  for (const auto & name : step.joint_names) {
    const auto it = snapshot.joints.find(name);
    if (it == snapshot.joints.end()) { reason = "Missing joint feedback: " + name; return false; }
    if (!context_.joint_stationary(name, it->second, reason)) return false;
  }
  return true;
}

bool TrajectoryExecutor::ready(const MotionSequence & sequence, std::string & reason) const {
  const auto state = result().state;
  if (state == ExecutionState::Running || state == ExecutionState::Cancelling ||
      state == ExecutionState::StopUnconfirmed) {
    reason = "Executor is busy or stop is unconfirmed";
    return false;
  }
  if (side_ && side_->result.state == ExecutionState::Cancelling) {
    reason = "Side arm stop is being confirmed";
    return false;
  }
  const auto snapshot = context_.snapshot();
  for (const auto & [name, arm] : config_.arms) {
    if (side_live() && name == side_->arm) continue; // The side arm is moving on its own.
    auto joints = arm.joints;
    joints.push_back(arm.gripper_joint);
    for (const auto & joint : joints) {
      const auto it = snapshot.joints.find(joint);
      if (it == snapshot.joints.end()) {
        reason = "Missing joint feedback before starting: " + joint;
        return false;
      }
      if (!context_.joint_ready_for_motion(
              joint, it->second, reason, state == ExecutionState::Succeeded)) {
        return false;
      }
    }
  }
  if (config_.smooth_motion && (!robot_model_ || !scene_client_->service_is_ready() ||
                                   !execute_client_->action_server_is_ready())) {
    reason = "Smoothed motion needs a robot model, planning scene and trajectory execution server";
    return false;
  }
  for (const auto & step : sequence.steps) {
    if (side_blocks(step, reason)) return false;
    if (!joint_group_ready(step, reason)) return false;
    if ((step.type == "joint" || step.type == "pose" || step.type == "linear" ||
            step.type == "dual_joint" || step.type == "joint_gripper" ||
            step.type == "dual_joint_gripper" || step.type == "joint_group") &&
        !move_client_->action_server_is_ready()) {
      reason = "MoveIt action server is unavailable";
      return false;
    }
    if (step.type == "pose" && step.tcp_path_margin > 0 &&
        !execute_client_->action_server_is_ready()) {
      reason = "Trajectory execution server is unavailable for planned correction";
      return false;
    }
    if (step.type == "gripper") {
      bool available = config_.gripper_transport == "action"
                           ? joint_clients_.at(step.arm)->action_server_is_ready()
                           : publishers_.at(step.arm)->get_subscription_count() > 0;
      if (!available) {
        reason = "Gripper trajectory endpoint is unavailable: " + step.arm;
        return false;
      }
    }
  }
  reason.clear();
  return true;
}

ExecutionResult TrajectoryExecutor::result() const {
  if (side_ && side_->result.state == ExecutionState::StopUnconfirmed) {
    return {ExecutionState::StopUnconfirmed, "STOP_UNCONFIRMED",
        side_->arm + " side arm: " + side_->result.detail};
  }
  if (!operation_) return ExecutionResult{};
  auto outcome = operation_->result;
  // start_paired failed before its combined path was sent: neither arm was commanded.
  if (!operation_->paired_steps.empty() && outcome.state == ExecutionState::Failed &&
      !operation_->executing_path && !operation_->cancel_requested) {
    outcome.detail = outcome.code + ": " + outcome.detail;
    outcome.code = "PAIRED_BEFORE_MOTION";
  }
  if (operation_->round_trip) {
    const auto & trip = *operation_->round_trip;
    outcome.phase = operation_->step.name;
    outcome.last_completed_step = trip.last_completed;
    outcome.retryable_before_motion = outcome.state == ExecutionState::Failed &&
        trip.retryable && !trip.execution_requested && !operation_->cancel_requested;
  }
  return outcome;
}

void TrajectoryExecutor::publish_state() {
  context_.set_execution(result());
}

namespace {
template <class Value>
bool pending(const std::future<Value> & future) {
  return future.valid() && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
}
} // namespace

void TrajectoryExecutor::set_operation(const std::shared_ptr<Operation> & op) {
  // An unfinished std::async future blocks in its destructor: keep it until it is ready.
  if (operation_ && (pending(operation_->preparation) ||
                        (operation_->gate && pending(operation_->gate->check)))) {
    retired_operations_.push_back(operation_);
  }
  operation_ = op;
}

bool TrajectoryExecutor::refuse_for_side(const Step & step) {
  std::string reason;
  if (!side_blocks(step, reason)) return false;
  auto op = std::make_shared<Operation>();
  op->step = step;
  op->result = {ExecutionState::Failed, "SIDE_ARM_BUSY", reason};
  set_operation(op);
  publish_state();
  return true;
}

bool TrajectoryExecutor::start(const Step & step) {
  if (config_.smooth_motion && (step.type == "dual_joint" || step.type == "dual_joint_gripper" ||
                                   step.type == "joint_gripper")) {
    return start_sequence({step});
  }
  const auto previous = result().state;
  if (previous == ExecutionState::Running || previous == ExecutionState::Cancelling ||
      previous == ExecutionState::StopUnconfirmed) {
    return false;
  }
  if (refuse_for_side(step)) return false;
  MotionSequence single_step_sequence;
  single_step_sequence.steps = {step};
  const auto validation = validate_motion_sequence(single_step_sequence, config_);
  auto op = std::make_shared<Operation>();
  op->step = step;
  set_operation(op);
  std::string reason;
  if (!validation.empty() || !context_.ready(reason) || !joint_group_ready(step, reason)) {
    op->result = {
        ExecutionState::Failed, "INVALID_COMMAND", validation.empty() ? reason : validation};
    publish_state();
    return false;
  }
  op->side_linked = side_live();
  try {
    begin_main(op);
  } catch (const std::exception & error) {
    op->result = {ExecutionState::Failed, "SEND_FAILED", error.what()};
  }
  publish_state();
  return op->result.state == ExecutionState::Running;
}

void TrajectoryExecutor::begin_main(const std::shared_ptr<Operation> & op) {
  const auto & step = op->step;
  if (step.type == "joint" || step.type == "pose" || step.type == "linear" ||
      step.type == "dual_joint" || step.type == "joint_gripper" ||
      step.type == "dual_joint_gripper" || step.type == "joint_group") {
    // Corridor corrections, and arm moves beside a live side path, are planned first.
    const bool plan_first = (step.type == "pose" && step.tcp_path_margin > 0) ||
        (side_live() && (step.type == "joint" || step.type == "pose" || step.type == "linear"));
    send_move(op, plan_first);
  } else if (step.type == "gripper") {
    if (config_.smooth_motion) {
      request_scene(op, false);
    } else {
      send_gripper(op);
    }
  }
  // wait is intentionally non-blocking; tick() completes it.
}

void TrajectoryExecutor::cancel(const std::string & code, const std::string & reason) {
  cancel_main(code, reason); // The side stops only through cancel_side().
}

void TrajectoryExecutor::cancel_main(const std::string & code, const std::string & reason) {
  const auto op = operation_;
  if (!op || op->result.state != ExecutionState::Running) {
    return;
  }
  if (op->round_trip) {
    const auto & trip = *op->round_trip;
    auto cause = reason;
    std::replace(cause.begin(), cause.end(), '\n', ' ');
    std::replace(cause.begin(), cause.end(), '\r', ' ');
    const char * phase = "planning";
    switch (trip.phase) {
      case RoundPhase::Planning: phase = "planning"; break;
      case RoundPhase::Validating: phase = "validating"; break;
      case RoundPhase::Tilt: phase = "tilt"; break;
      case RoundPhase::Hold: phase = "hold"; break;
      case RoundPhase::ReturnValidating: phase = "return_validating"; break;
      case RoundPhase::Untilt: phase = "untilt"; break;
    }
    RCLCPP_WARN(node_.get_logger(),
        "round trip: cancel decision phase=%s step=%s execution_result_received=%s "
        "success=%s phase_elapsed=%.3f s code=%s cause=%s",
        phase, op->step.name.c_str(), trip.execution_result_at ? "true" : "false",
        trip.execution_result_at && op->action_success ? "true" : "false",
        elapsed(trip.phase_started), code.c_str(), cause.c_str());
    log_wheel_peaks(*op); // Only while a tilt/untilt execution has not logged its peaks yet.
  }
  op->cancel_requested = true;
  op->cancel_code = code;
  op->cancel_reason = reason;
  op->cancelling = Clock::now();
  op->stable_since.reset();
  op->result = {ExecutionState::Cancelling, code, reason};
  if (op->round_trip) {
    op->round_trip->retryable = false;
    op->round_trip->forward = moveit_msgs::msg::RobotTrajectory{};
    op->round_trip->backward = moveit_msgs::msg::RobotTrajectory{};
    op->path = SmoothPath();
  }
  if (op->step.type == "wait" && !op->round_trip) {
    op->result.state = code == "CANCELLED" ? ExecutionState::Cancelled : ExecutionState::Failed;
  } else if (op->topic_command) {
    publish_hold(op);
  } else if (op->cancel_action && !op->action_terminal) {
    try {
      op->cancel_action();
    } catch (const std::exception & e) {
      op->cancel_reason += "; " + std::string(e.what());
    }
  }
  publish_state();
}

void TrajectoryExecutor::tick() {
  for (auto it = retired_operations_.begin(); it != retired_operations_.end();) {
    if (!pending((*it)->preparation) && !((*it)->gate && pending((*it)->gate->check))) {
      try {
        if ((*it)->preparation.valid()) (*it)->preparation.get();
      } catch (const std::exception &) {
      }
      it = retired_operations_.erase(it);
    } else {
      ++it;
    }
  }
  const auto op = operation_;
  if (!op) {
    tick_side();
    return;
  }
  if (op->round_trip) {
    tick_round_trip(op);
  }
  if (!op->round_trip && op->preparation.valid() &&
      op->preparation.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      auto path = op->preparation.get();
      RCLCPP_INFO(node_.get_logger(),
          "Prepared %s in %.3f s",
          op->step.name.c_str(),
          elapsed(op->preparation_started));
      if (op->line_replan) {
        RCLCPP_INFO(node_.get_logger(), "OMPL replan of the joint-line bundle ending at %s passed "
            "smoothing and collision checks", op->step.name.c_str());
      }
      if (!op->cancel_requested && op->result.state == ExecutionState::Running) {
        std::string reason;
        if (!context_.ready(reason)) {
          throw std::runtime_error(reason);
        }
        const auto snapshot = context_.snapshot();
        const auto & trajectory = path.trajectory.joint_trajectory;
        // A gripper no step of this operation moves (it may be squeezing an object) is held.
        const auto held_gripper = [&](const std::string & name) {
          return context_.is_gripper_joint(name) && !moves_gripper(config_, op->step, name) &&
                 std::none_of(op->steps.begin(), op->steps.end(), [&](const Step & step) {
                   return moves_gripper(config_, step, name);
                 });
        };
        const auto check_start = [&](const std::string & what, const std::string & name,
                                     double planned) {
          const auto & measured = snapshot.joints.at(name);
          const bool held = held_gripper(name);
          const double tolerance = held && config_.held_gripper_start_tolerance > 0
                                       ? config_.held_gripper_start_tolerance
                                       : config_.continuous_start_tolerance;
          const double error = std::abs(measured.position - planned);
          if (!context_.joint_ready_for_motion(name, measured, reason, true) || error > tolerance) {
            throw std::runtime_error(what + ": " +
                start_mismatch(name, planned, measured.position, tolerance) + "; " + reason);
          }
          if (held && error > HELD_GRIPPER_NOTE) {
            RCLCPP_INFO(node_.get_logger(), "Held gripper drift before %s (%s): %s",
                op->step.name.c_str(), what.c_str(),
                start_mismatch(name, planned, measured.position, tolerance).c_str());
          }
        };
        for (const auto & [name, planned] : op->preparation_positions) {
          const bool side_arm = op->side_linked && side_ &&
              (name == config_.arms.at(side_->arm).gripper_joint ||
                  std::count(config_.arms.at(side_->arm).joints.begin(),
                      config_.arms.at(side_->arm).joints.end(), name));
          if (side_arm) continue; // Moved by the side operation; covered by the pair check.
          check_start("Robot moved during trajectory preparation", name, planned);
        }
        const double duration =
            rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
        if (duration + config_.settle_time + config_.gripper_stop_window >=
            op->step.timeout - elapsed(op->started)) {
          throw std::runtime_error(
              "Motion timeout is too short for the prepared path and stop verification");
        }
        for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
          check_start("Robot moved during planning", trajectory.joint_names[joint],
              trajectory.points.front().positions[joint]);
        }
        op->path = std::move(path);
        if (op->step.type == "gripper") {
          gate_main(op, op->path.trajectory.joint_trajectory, [this, op]() { send_gripper(op); });
        } else {
          Execute::Goal goal;
          goal.trajectory = op->path.trajectory;
          if (!config_.require_passive_gripper_stationary) {
            // Planning still checks the measured finger geometry. Do not send a
            // position-hold goal to a current-limited gripper while moving arms.
            auto & command = goal.trajectory.joint_trajectory;
            for (size_t i = command.joint_names.size(); i-- > 0;) {
              const auto & name = command.joint_names[i];
              if (!context_.is_gripper_joint(name)) continue;
              const bool commanded = std::any_of(op->steps.begin(), op->steps.end(),
                  [&](const Step & step) {
                    return (step.type == "joint_gripper" || step.type == "dual_joint_gripper") &&
                           config_.arms.at(step.arm).gripper_joint == name;
                  });
              if (commanded) continue;
              command.joint_names.erase(command.joint_names.begin() + i);
              for (auto & point : command.points) {
                for (auto * values : {&point.positions, &point.velocities,
                         &point.accelerations, &point.effort}) {
                  if (!values->empty()) values->erase(values->begin() + i);
                }
              }
            }
          }
          gate_main(op, goal.trajectory.joint_trajectory, [this, op, goal]() {
            planned_trajectory_publisher_->publish(goal.trajectory.joint_trajectory);
            dispatch<Execute>(execute_client_, goal, op, [](const Execute::Result & result) {
              return result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS
                         ? std::string{}
                         : "MoveIt execution error " + std::to_string(result.error_code.val);
            });
            op->executing_path = true;
            RCLCPP_INFO(node_.get_logger(),
                "Executing %zu continuous segment(s), duration %.3f s",
                op->steps.size(),
                rclcpp::Duration(goal.trajectory.joint_trajectory.points.back().time_from_start)
                    .seconds());
            if (!op->paired_log.empty()) {
              RCLCPP_INFO(node_.get_logger(), "%s, duration %.1f s", op->paired_log.c_str(),
                  rclcpp::Duration(goal.trajectory.joint_trajectory.points.back().time_from_start)
                      .seconds());
            }
          });
        }
      }
    } catch (const PathCollisionError & error) {
      // Only a collision of the prepared path may replan a joint-line bundle, before any motion.
      if (!op->cancel_requested && op->result.state == ExecutionState::Running) {
        std::string detail = error.what();
        if (op->line_replan) {
          detail = "OMPL replan also rejected: " + detail + "; joint line: " + op->line_rejection;
          RCLCPP_WARN(node_.get_logger(), "%s: %s", op->step.name.c_str(), detail.c_str());
        }
        bool retried = false;
        try {
          retried = retry_line_bundle(op, error.what());
        } catch (const std::exception & failure) {
          detail += std::string("; OMPL replan not started: ") + failure.what();
        }
        if (!retried) op->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", detail};
      }
    } catch (const std::exception & error) {
      if (!op->cancel_requested && op->result.state == ExecutionState::Running) {
        op->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", error.what()};
      }
    }
  }
  tick_gate(op);
  update_path_phase(op);
  if (!op->round_trip && op->result.state == ExecutionState::Running) {
    if (elapsed(op->started) >= op->step.timeout) {
      cancel("TIMEOUT",
          "Step timed out: " + op->step.name + "; " + op->completion_wait_reason +
              "; action_terminal=" + (op->action_terminal ? "true" : "false") +
              ", action_success=" + (op->action_success ? "true" : "false"));
    } else if (op->step.type == "wait") {
      if (elapsed(op->started) >= op->step.duration) {
        op->result.state = ExecutionState::Succeeded;
      }
    } else {
      const bool finished = op->topic_command ? elapsed(op->started) >= op->step.duration
                                              : op->action_terminal && op->action_success;
      const bool wait_only = op->step.type == "gripper" && op->step.gripper_wait_only;
      const bool duration_elapsed = op->gripper_sent &&
          elapsed(*op->gripper_sent) >= op->step.duration;
      if (finished && (wait_only ? duration_elapsed : settled(op, true))) {
        op->result.state = ExecutionState::Succeeded;
        if (op->fallback_tries > 0) {  // which planner made it, after which failures
          std::string failed;
          for (const auto & attempt : op->plan_attempts) {
            failed += (failed.empty() ? "" : ", ") + attempt;
          }
          op->result.detail = "planned with " + op->planner_label + " on fallback attempt " +
                              std::to_string(op->fallback_tries) + "/" +
                              std::to_string(op->step.fallback_attempts) + " after " + failed;
          RCLCPP_INFO(node_.get_logger(), "%s: %s", op->step.name.c_str(),
              op->result.detail.c_str());
        }
      } else if (finished && wait_only) {
        op->completion_wait_reason = "Waiting for gripper command duration; grasp not verified";
      } else if (!finished) {
        op->completion_wait_reason = op->topic_command ? "Waiting for commanded duration"
                                                       : "Waiting for controller/action result";
      }
    }
  }
  if (op->result.state == ExecutionState::Cancelling) {
    if (op->topic_command && !op->hold_sent) {
      publish_hold(op);
    }
    const bool stopped_request =
        op->topic_command ? op->hold_sent && elapsed(op->cancelling) >= 0.2 : op->action_terminal;
    if (stopped_request && settled(op, false)) {
      op->result = {
          op->cancel_code == "CANCELLED" ? ExecutionState::Cancelled : ExecutionState::Failed,
          op->cancel_code,
          op->cancel_reason};
    } else if (elapsed(op->cancelling) >= config_.cancel_timeout) {
      // Latched until restart: a late acceptance/result must not unlock a new motion.
      op->result = {ExecutionState::StopUnconfirmed,
          "STOP_UNCONFIRMED",
          op->cancel_reason + "; stop not confirmed"};
    }
  }
  if (op->round_trip && op->result.state != ExecutionState::Running &&
      op->result.state != ExecutionState::Cancelling) {
    if (!op->round_trip->total_logged) {
      RCLCPP_INFO(node_.get_logger(), "round trip: terminal %s, total %.3f s, code=%s",
          to_string(op->result.state), elapsed(op->round_trip->started), op->result.code.c_str());
      op->round_trip->total_logged = true;
    }
    op->round_trip->forward = moveit_msgs::msg::RobotTrajectory{};
    op->round_trip->backward = moveit_msgs::msg::RobotTrajectory{};
    op->path = SmoothPath();
  }
  tick_side();
  publish_state();
}
} // namespace demo_1f

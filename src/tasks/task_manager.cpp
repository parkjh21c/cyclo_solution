#include "demo_1f/tasks/task_manager.hpp"
#include <algorithm>
#include <utility>

namespace demo_1f {
TaskManager::TaskManager(rclcpp::Node & node,
    RobotContext & context,
    HandoverMotion & motion,
    std::vector<CupMotionRegistration> cup_motions, InitialPoseMotion * initial_motion,
    HeadCommandPort * head, SideMotionPort * side)
    : head_(head), side_(side), node_(node), context_(context), motion_(motion), initial_motion_(initial_motion),
      cup_motions_(std::move(cup_motions)) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto action_name =
      node.declare_parameter<std::string>("handover_action", "/handover", descriptor);
  status_publisher_ =
      node.create_publisher<demo_1f::msg::DemoStatus>("~/status", rclcpp::QoS(1).transient_local());
  capture_service_ = node.create_service<std_srvs::srv::Trigger>("~/capture_targets",
      [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::string reason;
        const auto targets = context_.capture_targets(reason);
        response->success = targets.has_value();
        response->message = targets ? *targets : reason;
      });

  server_ = make_handover_server(action_name, false);
  prepare_server_ = make_handover_server("/prepare_handover", true);
  if (!cup_motions_.empty()) {
    setup_cup_servers();
  }
  setup_demo_server();
  timer_ = node.create_wall_timer(std::chrono::milliseconds(20), [this]() {
    tick();
  });
  publish_status();
}

std::string TaskManager::head_pose_for_task(const std::string & task) {
  if (task == "PICK_DYNAMIXEL" || task == "POUR_CUP" || task == "/pour_cup") {
    return "look_right";
  }
  if (task == "APPROACH_CUP" || task == "REFINE_CUP" || task == "PICK_CUP" ||
      task == "PLACE_CUP" || task == "/approach_cup" || task == "/refine_cup" ||
      task == "/pick_cup" || task == "/place_cup") {
    return "look_left";
  }
  return "look_middle";
}

void TaskManager::command_head(const std::string & task) {
  const auto name = head_pose_for_task(task);
  if (!head_ || name == head_last_sent_) return;
  if (head_->command(name)) head_last_sent_ = name;
  else head_last_sent_.clear(); // Not sent: the next task tries again.
}

void TaskManager::cancel_head() {
  if (head_) head_->cancel();
  head_last_sent_.clear();
}

bool TaskManager::waits_for_side(const std::string & stage) {
  // Stages that move the side arm or both arms start only after the side has arrived.
  return stage == "PICK_DYNAMIXEL" || stage == "HANDOVER" || stage == "WITHDRAW_CUP" ||
         stage == "HANDOVER_CUP" || stage == "POUR_CUP" || stage == "HANDOVER_CUP_BACK" ||
         stage == "INITIALIZE_ROBOT" || stage == "RETURN_INITIAL";
}

bool TaskManager::side_active() const {
  if (!side_) return false;
  const auto state = side_->result().state;
  return state == ExecutionState::Running || state == ExecutionState::Cancelling;
}

bool TaskManager::b2_return_ready(std::string & reason) const {
  if (!side_ || side_->arm() != "right") {
    reason = "Independent cup return requires a configured right-arm side port";
    return false;
  }
  return true;
}

bool TaskManager::start_side_return(std::string & reason) {
  if (!side_) {
    reason = "Side return port missing";
    return false;
  }
  if (side_active()) return true; // A stage retry keeps the return already under way.
  side_failure_.reset();
  side_cancel_requested_ = side_stage_stop_sent_ = false;
  if (!side_->start(reason)) {
    RCLCPP_WARN(node_.get_logger(), "%s arm return not started: %s", side_->arm().c_str(),
        reason.c_str());
    return false;
  }
  side_started_at_ = Clock::now();
  side_return_seconds_ = -1;
  side_watching_ = true;
  watch_side(); // Already home: done at once.
  return true;
}

void TaskManager::cancel_side_return() {
  if (!side_active()) return;
  side_cancel_requested_ = true;
  side_->cancel();
}

void TaskManager::watch_side() {
  if (!side_watching_ || side_active()) return;
  side_watching_ = false;
  const auto outcome = side_->result();
  if (outcome.state == ExecutionState::Succeeded) {
    side_return_seconds_ = elapsed(side_started_at_);
    RCLCPP_INFO(node_.get_logger(), "%s arm return done in %.1f s", side_->arm().c_str(),
        side_return_seconds_);
    return;
  }
  if (outcome.state == ExecutionState::StopUnconfirmed) demo_faulted_ = true;
  if (side_cancel_requested_ && outcome.state == ExecutionState::Cancelled) return;
  const bool before_motion = outcome.retryable_before_motion;
  const auto code = before_motion ? std::string("SIDE_BEFORE_MOTION") :
      outcome.state == ExecutionState::StopUnconfirmed ? std::string("STOP_UNCONFIRMED") :
      std::string("SIDE_ARM_FAILED");
  side_failure_ = SideFailure{code, side_->arm() + " arm return " + outcome.code + ": " +
      outcome.detail, before_motion};
  RCLCPP_WARN(node_.get_logger(), "%s: %s", code.c_str(), side_failure_->detail.c_str());
  // After the demo already ended: no demo stage reports it, so the saved checkpoint is dropped.
  if (!demo_goal_ && !before_motion) demo_resumable_ = false;
}

bool TaskManager::side_blocks_cup(const std::string & name, const std::string & arm) const {
  if (!side_active()) return false;
  // Only one-arm cup actions on the other arm may run beside the side return.
  const bool one_arm = name == "/approach_cup" || name == "/refine_cup" || name == "/pick_cup" ||
                       name == "/place_cup" || name == "/withdraw_cup";
  return !one_arm || arm == "both" || arm == side_->arm();
}

rclcpp_action::Server<TaskManager::Action>::SharedPtr TaskManager::make_handover_server(
    const std::string & action_name, bool prepare_only) {
  return rclcpp_action::create_server<Action>(
      &node_,
      action_name,
      [this, prepare_only](
          const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> request) {
        std::string reason;
        if (demo_busy() || side_active() || shutdown_requested_ || goal_start_pending_ ||
            active_goal_handle_ != nullptr || cup_goal_ != nullptr || cup_motion_blocked() ||
            !motion_.is_ready_to_start(request->direction, reason, prepare_only)) {
          if (reason.empty()) {
            reason = "Task manager is busy or shutting down";
          }
          RCLCPP_WARN(node_.get_logger(), "Rejected handover: %s", reason.c_str());
          if (!active_goal_handle_ && !cup_goal_ && !goal_start_pending_ && !motion_.running()) {
            context_.set_task("REJECTED", request->direction, "NOT_READY", reason);
          }
          return rclcpp_action::GoalResponse::REJECT;
        }
        goal_start_pending_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        if (handle != active_goal_handle_ || !motion_.running()) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        motion_.cancel();
        cancel_head();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this, prepare_only](const std::shared_ptr<GoalHandle> handle) {
        goal_start_pending_ = false;
        std::string reason;
        if (shutdown_requested_ ||
            !motion_.start(handle->get_goal()->direction, reason, prepare_only)) {
          auto result = std::make_shared<Action::Result>();
          result->error_code = "NOT_READY";
          result->message = reason.empty() ? "Shutting down" : reason;
          handle->abort(result);
          return;
        }
        command_head("HANDOVER");
        demo_resumable_ = false; // Manual motions invalidate a saved automatic checkpoint.
        active_goal_handle_ = handle;
        context_.set_task("RUNNING", handle->get_goal()->direction);
        RCLCPP_INFO(node_.get_logger(), "Starting %s", handle->get_goal()->direction.c_str());
      });
}

void TaskManager::request_shutdown() {
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  demo_cancel_requested_ = true;
  cancel_demo_motion();
  cancel_head();
  cancel_side_return();
  motion_.cancel();
  for (const auto & module : cup_motions_) {
    module.motion.cancel();
  }
}

bool TaskManager::cup_motion_blocked() const {
  return std::any_of(cup_motions_.begin(), cup_motions_.end(), [](const auto & module) {
    return module.motion.blocks_other_motion();
  });
}

bool TaskManager::can_shutdown() const {
  if (demo_faulted_ && !demo_goal_) return true;
  return !demo_goal_ && !pick_unsettled() && !goal_start_pending_ && !active_goal_handle_ &&
         !motion_.running() && !side_active() &&
         (!initial_motion_ || !initial_motion_->running()) && !cup_goal_ &&
         std::none_of(cup_motions_.begin(), cup_motions_.end(), [](const auto & module) {
           return module.motion.running();
         });
}

void TaskManager::tick() {
  maintain_pick_requests();
  if (side_) side_->tick(); // Progresses even when no module runs.
  watch_side();
  if (demo_faulted_) cancel_side_return(); // Fault latch: everything stops.
  if (demo_goal_) {
    tick_demo();
  } else if (cup_goal_) {
    tick_cup_action();

  } else {
    motion_.tick();
  }
  if (active_goal_handle_) {
    const auto & outcome = motion_.result();
    context_.set_task(
        outcome.state, active_goal_handle_->get_goal()->direction, outcome.code, outcome.detail);
    if (!motion_.running()) {
      auto result = std::make_shared<Action::Result>();
      result->sequence_completed = outcome.sequence_completed;
      result->handover_confirmed = outcome.handover_confirmed;
      result->error_code = outcome.code;
      result->message = outcome.detail;
      result->last_completed_step = outcome.last_completed_step;
      if (active_goal_handle_->is_canceling()) {
        active_goal_handle_->canceled(result);
      } else if (outcome.sequence_completed) {
        active_goal_handle_->succeed(result);
      } else {
        active_goal_handle_->abort(result);
      }
      RCLCPP_INFO(
          node_.get_logger(), "Handover %s: %s", outcome.state.c_str(), outcome.detail.c_str());
      active_goal_handle_.reset();
      publish_status();
    }
  }
  if (elapsed(last_publish_) >= 0.2) {
    publish_status();
    if (active_goal_handle_) {
      const auto state = context_.snapshot();
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->direction = state.direction;
      feedback->phase = state.phase;
      feedback->execution_state = to_string(state.execution.state);
      feedback->holding_state = "UNVERIFIED";
      active_goal_handle_->publish_feedback(feedback);
    }
    last_publish_ = Clock::now();
  }
}

void TaskManager::publish_status() {
  const auto state = context_.snapshot();
  demo_1f::msg::DemoStatus status;
  status.task_state = state.task_state;
  status.direction = state.direction;
  status.phase = state.phase;
  status.last_completed_step = state.last_completed_step;
  status.execution_state = to_string(state.execution.state);
  status.holding_state = "UNVERIFIED";
  status.error_code = state.error_code;
  status.detail = state.detail;
  std::string reason;
  status.robot_state_fresh = context_.ready(reason);
  if (!status.robot_state_fresh && status.detail.empty()) {
    status.detail = reason;
  }
  status_publisher_->publish(status);
}
} // namespace demo_1f

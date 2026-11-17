#include "demo_1f/runtime/execution/head_commander.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace demo_1f {
HeadCommander::HeadCommander(rclcpp::Node & node, const Config & config, RobotContext & context)
    : node_(node), config_(config), context_(context) {
  action_name_ = node.declare_parameter<std::string>(
      "head_command.action", "/head_controller/follow_joint_trajectory");
  max_velocity_ = node.declare_parameter<double>("head_command.max_velocity", 0.5);
  min_duration_ = node.declare_parameter<double>("head_command.min_duration", 1.0);
  fallback_duration_ = node.declare_parameter<double>("head_command.fallback_duration", 8.0);
  if (!(max_velocity_ > 0) || !(min_duration_ > 0) || !(fallback_duration_ >= min_duration_)) {
    throw std::invalid_argument("Invalid head_command parameters");
  }
  client_ = rclcpp_action::create_client<Action>(&node_, action_name_);
}

double HeadCommander::duration_to(const Step & step) const {
  const double scale = step.velocity_scale > 0 ? step.velocity_scale : config_.velocity_scale;
  const double speed = max_velocity_ * std::clamp(scale, 1e-3, 1.0);
  const auto state = context_.snapshot();
  double largest = 0;
  for (size_t i = 0; i < step.joint_names.size(); ++i) {
    const auto joint = state.joints.find(step.joint_names[i]);
    // Missing or stale head feedback: use the conservative fixed time.
    if (joint == state.joints.end() || !context_.joint_fresh(joint->second) ||
        !std::isfinite(joint->second.position)) return fallback_duration_;
    largest = std::max(largest, std::abs(step.target[i] - joint->second.position));
  }
  return std::max(min_duration_, largest / speed);
}

bool HeadCommander::command(const std::string & name) {
  const auto found = config_.motion_sequences.find(name);
  const bool single = found != config_.motion_sequences.end() &&
      found->second.steps.size() == 1 && found->second.validation_error.empty();
  const Step * step = single ? &found->second.steps.front() : nullptr;
  if (!step || step->type != "joint_group" || step->joint_names.empty() ||
      step->joint_names.size() != step->target.size() ||
      !std::all_of(step->target.begin(), step->target.end(), [](double v) {
        return std::isfinite(v);
      })) {
    RCLCPP_WARN(node_.get_logger(), "Head %s not sent: no valid joint_group step", name.c_str());
    return false;
  }
  if (!client_->action_server_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "Head %s not sent: %s unavailable", name.c_str(),
        action_name_.c_str());
    return false;
  }
  const double seconds = duration_to(*step);
  Action::Goal goal;
  goal.trajectory.joint_names = step->joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = step->target;
  point.velocities.assign(step->target.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(seconds);
  goal.trajectory.points.push_back(point);
  const auto sequence = ++latest_;
  cancel_latest_ = false;
  handle_.reset(); // The controller replaces the previous goal.
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, sequence, name](Handle::SharedPtr handle) {
    if (!handle) {
      RCLCPP_WARN(node_.get_logger(), "Head %s rejected by the controller", name.c_str());
      return;
    }
    if (sequence != latest_) return; // Superseded by a newer goal.
    if (cancel_latest_) {
      cancel_latest_ = false;
      client_->async_cancel_goal(handle);
      return;
    }
    handle_ = handle;
  };
  options.result_callback = [this, sequence, name](const Handle::WrappedResult & result) {
    if (sequence != latest_) return; // Replaced by a newer goal; its end is expected.
    handle_.reset();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) return;
    if (result.code == rclcpp_action::ResultCode::CANCELED && sequence == cancelled_) return;
    RCLCPP_WARN(node_.get_logger(), "Head %s ended %s: %s", name.c_str(),
        result.code == rclcpp_action::ResultCode::CANCELED ? "cancelled" : "without success",
        result.result ? result.result->error_string.c_str() : "no result");
  };
  try {
    client_->async_send_goal(goal, options);
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_.get_logger(), "Head %s not sent: %s", name.c_str(), error.what());
    return false;
  }
  RCLCPP_INFO(node_.get_logger(), "Head %s sent (%.1f s)", name.c_str(), seconds);
  return true;
}

void HeadCommander::cancel() {
  cancelled_ = latest_;
  if (handle_) {
    client_->async_cancel_goal(handle_);
    handle_.reset();
  } else if (latest_ > 0) {
    cancel_latest_ = true; // No-op when the latest goal already ended or was rejected.
  }
}
} // namespace demo_1f

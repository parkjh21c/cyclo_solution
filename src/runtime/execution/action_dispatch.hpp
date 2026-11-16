#pragma once
#include "demo_1f/runtime/execution/trajectory_executor.hpp"
#include <type_traits>

// Shared ROS action lifecycle; kept private to the execution implementation.
namespace demo_1f {
template <class Action>
void TrajectoryExecutor::dispatch(const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const typename Action::Goal & goal,
    const std::shared_ptr<Operation> & op,
    std::function<std::string(const typename Action::Result &)> error,
    std::function<void(const typename Action::Result &)> on_success) {
  if (!client->action_server_is_ready()) {
    throw std::runtime_error("Action server unavailable");
  }
  op->action_terminal = false;
  op->action_success = false;
  op->cancel_action = {};
  const size_t generation = ++op->action_generation;
  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback =
      [this, op, client, generation](
          typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle) {
        if (operation_ != op || generation != op->action_generation ||
            (op->result.state != ExecutionState::Running &&
                op->result.state != ExecutionState::Cancelling)) {
          if (handle) client->async_cancel_goal(handle);
          return;
        }
        if (!handle) {
          op->action_terminal = true;
          if (!op->cancel_requested) {
            if (op->round_trip && op->round_trip->execution_requested) {
              cancel("GOAL_REJECTED", op->step.name + ": Remote goal rejected");
            } else {
              op->result = {ExecutionState::Failed, "GOAL_REJECTED", "Remote goal rejected"};
            }
          }
          return;
        }
        op->cancel_action = [client, handle]() {
          client->async_cancel_goal(handle);
        };
        // Cancellation may arrive before the server accepts the goal. Never orphan it.
        if (op->cancel_requested) {
          try {
            op->cancel_action();
          } catch (const std::exception & e) {
            op->cancel_reason += "; " + std::string(e.what());
          }
        }
      };
  options.result_callback =
      [this, op, error, on_success, generation, planning_only = op->planning_only](
          const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult & response) {
        if (operation_ != op || generation != op->action_generation ||
            (op->result.state != ExecutionState::Running &&
                op->result.state != ExecutionState::Cancelling)) return;
        op->action_terminal = true;
        op->cancel_action = {};
        const auto detail = response.result ? error(*response.result) : "Missing action result";
        op->action_success =
            response.code == rclcpp_action::ResultCode::SUCCEEDED && detail.empty();
        if constexpr (std::is_same_v<Action, Execute>) {
          if (op->round_trip) {
            op->round_trip->execution_result_at = Clock::now();
            RCLCPP_INFO(node_.get_logger(), "round trip: %s execution result after %.3f s, "
                "success=%s", op->step.name.c_str(), elapsed(op->round_trip->phase_started),
                op->action_success ? "true" : "false");
          }
        }
        if (!op->cancel_requested && op->action_success && on_success &&
            op->result.state == ExecutionState::Running) {
          try {
            on_success(*response.result);
          } catch (const std::exception & error) {
            if (op->round_trip) {
              fail_round_trip(op, "ROUND_TRIP_PREPARATION_FAILED", error.what());
            } else {
              op->result = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", error.what()};
            }
          }
          return;
        }
        if (!op->cancel_requested && !op->action_success) {
          std::string message = detail.empty() ? "Action did not succeed" : detail;
          if (op->round_trip) {
            if (round_trip_timed_out(op)) return;
            if constexpr (std::is_same_v<Action, Move>) {
              using Codes = moveit_msgs::msg::MoveItErrorCodes;
              const int code = response.result ? response.result->error_code.val : 0;
              const bool planning_failure = (planning_only && code == Codes::FAILURE) ||
                  code == Codes::PLANNING_FAILED ||
                  code == Codes::INVALID_MOTION_PLAN || code == Codes::NO_IK_SOLUTION ||
                  code == Codes::TIMED_OUT || code == Codes::GOAL_STATE_INVALID ||
                  code == Codes::START_STATE_IN_COLLISION ||
                  code == Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS ||
                  code == Codes::GOAL_IN_COLLISION || code == Codes::GOAL_CONSTRAINTS_VIOLATED ||
                  code == Codes::GOAL_VIOLATES_PATH_CONSTRAINTS;
              const bool no_execution = response.result &&
                  response.result->executed_trajectory.joint_trajectory.points.empty() &&
                  response.result->executed_trajectory.multi_dof_joint_trajectory.points.empty();
              if (response.result && !no_execution) {
                cancel("ROBOT_STATE_INVALID", op->step.name +
                    ": Unexpected execution in a plan-only failure; " + message);
                return;
              }
              fail_round_trip(op, "ROUND_TRIP_PLAN_FAILED", message,
                  planning_failure && no_execution &&
                      response.code != rclcpp_action::ResultCode::CANCELED);
            } else {
              cancel("EXECUTION_FAILED", op->step.name + ": " + message);
            }
            return;
          }
          if constexpr (std::is_same_v<Action, Move>) {
            // A joint step with a planner fallback may plan again instead of failing here.
            if (response.result && replan_with_fallback(op, *response.result, message)) {
              return;
            }
          }
          op->result = {ExecutionState::Failed, "EXECUTION_FAILED", message};
          if constexpr (std::is_same_v<Action, Move>) {
            // Generic OMPL failures are retryable only for an explicit plan-only request.
            // Combined planning/execution errors must never be mistaken for pre-motion failure.
            if (response.result &&
                (response.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT ||
                    (planning_only &&
                        (response.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::FAILURE ||
                            response.result->error_code.val ==
                                moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED))) &&
                response.result->planned_trajectory.joint_trajectory.points.empty() &&
                response.result->planned_trajectory.multi_dof_joint_trajectory.points.empty() &&
                response.result->executed_trajectory.joint_trajectory.points.empty() &&
                response.result->executed_trajectory.multi_dof_joint_trajectory.points.empty()) {
              op->result.code =
                  response.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT
                      ? "PLANNING_TIMEOUT" : "PLANNING_FAILED";
            }
          }
        }
      };
  if constexpr (std::is_same_v<Action, Execute>) {
    if (op->round_trip) op->round_trip->execution_requested = true;
  }
  client->async_send_goal(goal, options);
}
} // namespace demo_1f

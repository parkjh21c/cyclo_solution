#include "action_dispatch.hpp"
#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/round_trip_path.hpp"
#include "demo_1f/runtime/execution/round_trip_collision.hpp"
#include <moveit/robot_state/conversions.hpp>

namespace demo_1f {
namespace {
constexpr double WHEEL_SETTLE_TIMEOUT = 1.0;

bool is_drive_wheel(const std::string & name) {
  return name == "left_wheel_drive" || name == "right_wheel_drive" ||
         name == "rear_wheel_drive";
}

double wheel_velocity_limit(const Config & config) {
  return config.round_trip_wheel_velocity > 0 ? config.round_trip_wheel_velocity
                                              : config.stopped_velocity;
}

SmoothPath exact_path(const moveit_msgs::msg::RobotTrajectory & trajectory) {
  SmoothPath path;
  path.trajectory = trajectory;
  path.trajectory.joint_trajectory.header.stamp = builtin_interfaces::msg::Time{};
  path.point_steps.assign(trajectory.joint_trajectory.points.size(), 0);
  path.segment_end_times = {
      rclcpp::Duration(trajectory.joint_trajectory.points.back().time_from_start).seconds()};
  return path;
}
} // namespace

bool TrajectoryExecutor::start_round_trip(
    const Step & tilt, const Step & hold, const Step & untilt) {
  const auto previous = result().state;
  if (previous == ExecutionState::Running || previous == ExecutionState::Cancelling ||
      previous == ExecutionState::StopUnconfirmed) return false;
  Step probe; // Both controllers or the coordinated group.
  probe.type = "dual_joint";
  if (refuse_for_side(probe)) return false;
  auto op = std::make_shared<Operation>();
  op->step = tilt;
  op->round_trip.emplace();
  auto & trip = *op->round_trip;
  trip.tilt = tilt;
  trip.hold = hold;
  trip.untilt = untilt;
  trip.timeout = 3 * tilt.timeout + hold.timeout + 2 * untilt.timeout;
  MotionSequence sequence;
  sequence.steps = {tilt, hold, untilt};
  std::string reason = validate_motion_sequence(sequence, config_);
  if (reason.empty() && (tilt.type != "linear" || hold.type != "wait" ||
                           untilt.type != "linear" || tilt.arm != hold.arm ||
                           tilt.arm != untilt.arm || !std::isfinite(trip.timeout))) {
    reason = "Round-trip needs same-arm LIN, wait and LIN steps with finite timeouts";
  }
  const bool available = reason.empty() && ready(sequence, reason);
  set_operation(op);
  if (!available) {
    fail_round_trip(op, "NOT_READY", reason);
    publish_state();
    return false;
  }
  try {
    if (!robot_model_) load_robot_model();
    if (!scene_client_->service_is_ready() || !execute_client_->action_server_is_ready()) {
      throw std::runtime_error("Round-trip needs planning scene and trajectory execution servers");
    }
    const auto snapshot = context_.snapshot();
    for (const auto & [side, arm] : config_.arms) {
      (void)side;
      auto controlled = arm.joints;
      controlled.push_back(arm.gripper_joint);
      for (const auto & name : controlled) {
        trip.initial_positions[name] = snapshot.joints.at(name).position;
      }
    }
    op->result.detail = "Planning tilt and validating the complete round trip";
    request_scene(op, false);
  } catch (const std::exception & error) {
    fail_round_trip(op, "SCENE_UNAVAILABLE", error.what());
  }
  publish_state();
  return op->result.state == ExecutionState::Running;
}

void TrajectoryExecutor::fail_round_trip(const std::shared_ptr<Operation> & op,
    const std::string & code, const std::string & detail, bool retryable) {
  auto & trip = *op->round_trip;
  if (trip.execution_requested) {
    cancel(code, op->step.name + ": " + detail);
    return;
  }
  std::string reason;
  trip.retryable = retryable && !trip.execution_requested && !op->cancel_requested &&
                   context_.ready(reason) && round_trip_start_matches(*op, true, false, reason);
  op->result = {ExecutionState::Failed, code, op->step.name + ": " + detail};
  if (!trip.total_logged) {
    RCLCPP_INFO(node_.get_logger(), "round trip: terminal FAILED, total %.3f s, code=%s",
        elapsed(trip.started), code.c_str());
    trip.total_logged = true;
  }
  trip.forward = moveit_msgs::msg::RobotTrajectory{};
  trip.backward = moveit_msgs::msg::RobotTrajectory{};
}

bool TrajectoryExecutor::round_trip_start_matches(const Operation & op,
    bool check_active, bool returning, std::string & reason, RoundStateIssue * issue) const {
  if (issue) *issue = RoundStateIssue::None;
  const auto invalid = [&](RoundStateIssue kind, const std::string & detail) {
    if (issue) *issue = kind;
    reason = detail;
    return false;
  };
  std::string wheel_reason;
  const auto & trip = *op.round_trip;
  const auto snapshot = context_.snapshot();
  const auto & active = config_.arms.at(trip.tilt.arm).joints;
  const auto & trajectory =
      returning ? trip.backward.joint_trajectory : trip.forward.joint_trajectory;
  for (const auto & [name, initial] : trip.initial_positions) {
    const bool moving = std::find(active.begin(), active.end(), name) != active.end();
    const auto measured = snapshot.joints.find(name);
    if (measured == snapshot.joints.end()) {
      return invalid(RoundStateIssue::Feedback, "Missing related joint feedback: " + name);
    }
    if (!context_.joint_fresh(measured->second)) {
      return invalid(RoundStateIssue::Feedback, "Missing fresh round-trip feedback: " + name);
    }
    if (moving && !check_active) continue;
    double target = initial;
    if (moving && !trajectory.points.empty()) {
      const auto found =
          std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
      if (found == trajectory.joint_names.end()) {
        return invalid(RoundStateIssue::JointState, "Round-trip path lacks " + name);
      }
      target = trajectory.points.front().positions.at(found - trajectory.joint_names.begin());
    }
    const double tolerance = context_.is_gripper_joint(name) &&
            config_.held_gripper_start_tolerance > 0
        ? config_.held_gripper_start_tolerance : config_.continuous_start_tolerance;
    if ((moving && !returning &&
            std::abs(measured->second.position - initial) > tolerance) ||
        std::abs(measured->second.position - target) > tolerance) {
      return invalid(RoundStateIssue::Position, "Round-trip start/state mismatch: " + name);
    }
    if (!moving && !context_.is_gripper_joint(name) && is_drive_wheel(name)) {
      const auto & velocity = measured->second.velocity;
      if (!velocity || !std::isfinite(*velocity)) {
        return invalid(RoundStateIssue::Feedback,
            "Missing finite wheel velocity feedback: " + name);
      }
      const double wheel_limit = wheel_velocity_limit(config_);
      if (std::abs(*velocity) > wheel_limit) {
        // Continue checking every joint so a wheel warning cannot mask another failure.
        if (wheel_reason.empty()) {
          wheel_reason = "Wheel velocity is not stationary: " + name + " (velocity=" +
              std::to_string(*velocity) + " rad/s, limit=" +
              std::to_string(wheel_limit) + " rad/s)";
        }
      }
      continue;
    }
    std::string joint_reason;
    if (!context_.joint_ready_for_motion(name, measured->second, joint_reason, true)) {
      return invalid(RoundStateIssue::JointState,
          "Round-trip start/state mismatch: " + name + "; " + joint_reason);
    }
  }
  if (!wheel_reason.empty()) return invalid(RoundStateIssue::WheelVelocity, wheel_reason);
  if (trip.initial_positions.empty()) {
    return invalid(RoundStateIssue::JointState, "Missing round-trip initial positions");
  }
  reason.clear();
  return true;
}

void TrajectoryExecutor::round_trip_scene(const std::shared_ptr<Operation> & op,
    const moveit_msgs::msg::PlanningScene & message, bool prepare_execution) {
  if (round_trip_timed_out(op)) return;
  if (message.is_diff || message.robot_state.is_diff) {
    throw std::runtime_error("Round-trip requires a complete planning scene");
  }
  auto & trip = *op->round_trip;
  auto scene = scene_template_->diff();
  if (!scene->setPlanningSceneMsg(message)) {
    throw std::runtime_error("Cannot load the round-trip planning scene");
  }
  const auto snapshot = context_.snapshot();
  std::string reason;
  if (!context_.ready(reason)) {
    fail_round_trip(op, "ROBOT_STATE_INVALID", reason);
    return;
  }
  auto & state = scene->getCurrentStateNonConst();
  const auto & variables = robot_model_->getVariableNames();
  for (size_t index = 0; index < variables.size(); ++index) {
    if (robot_model_->getJointOfVariable(index)->getMimic()) continue;
    const auto & name = variables[index];
    const auto measured = snapshot.joints.find(name);
    if (measured != snapshot.joints.end()) {
      state.setVariablePosition(name, measured->second.position);
    }
    trip.scene_positions[name] = state.getVariablePosition(name);
    if (!std::isfinite(trip.scene_positions.at(name))) {
      throw std::runtime_error("Nonfinite scene joint: " + name);
    }
  }
  const auto * group = robot_model_->getJointModelGroup(config_.coordinated_group);
  std::vector<double> active_positions;
  for (const auto * joint : group->getActiveJointModels()) {
    for (const auto & name : joint->getVariableNames()) {
      active_positions.push_back(state.getVariablePosition(name));
    }
  }
  // Propagate measured gripper positions to mimic fingers before collision checks.
  state.setJointGroupActivePositions(group, active_positions);
  state.update();
  if (!prepare_execution) {
    trip.initial_positions = trip.scene_positions;
    // The head turns on its own look_* commands; round trips neither check nor wait for it.
    for (const auto * name : {"head_joint1", "head_joint2"}) trip.initial_positions.erase(name);
    for (const auto & [side, arm] : config_.arms) {
      (void)side;
      auto controlled = arm.joints;
      controlled.push_back(arm.gripper_joint);
      for (const auto & name : controlled) {
        if (!trip.initial_positions.count(name) || !snapshot.joints.count(name)) {
          throw std::runtime_error("Round-trip model or feedback lacks " + name);
        }
      }
    }
    if (!round_trip_start_matches(*op, true, false, reason)) {
      fail_round_trip(op, "ROBOT_STATE_INVALID", reason);
      return;
    }
    op->planning_scene = scene;
    moveit::core::robotStateToRobotStateMsg(state, op->planned_start);
    op->planned_start.is_diff = false;
    send_move(op, true);
    return;
  }
  const bool returning = trip.phase == RoundPhase::ReturnValidating;
  if (!round_trip_start_matches(*op, true, returning, reason)) {
    fail_round_trip(op, "ROBOT_STATE_INVALID", reason);
    return;
  }
  const auto forward = trip.forward;
  const auto backward = trip.backward;
  const auto controlled = config_.arms.at(trip.tilt.arm).joints;
  const auto config = config_;
  const auto logger = node_.get_logger();
  op->preparation_started = Clock::now();
  op->preparation = std::async(std::launch::async,
      [scene, config, forward, backward, controlled, returning, logger]() {
        const auto started = Clock::now();
        if (reverse_round_trip_path(forward, controlled) != backward) {
          throw std::runtime_error("Round-trip collision sharing requires an exact time reversal");
        }
        const auto & trajectory = returning ? backward : forward;
        try {
          const auto checks = validate_round_trip_collision(trajectory, controlled, config, *scene);
          RCLCPP_INFO(logger, "round trip: %s collision checks %.3f s, samples %zu",
              returning ? "return" : "initial", elapsed(started), checks.samples);
        } catch (const std::exception &) {
          RCLCPP_INFO(logger, "round trip: %s collision checks failed in %.3f s",
              returning ? "return" : "initial", elapsed(started));
          throw;
        }
        return exact_path(trajectory);
      });
}

void TrajectoryExecutor::accept_round_trip_plan(
    const std::shared_ptr<Operation> & op, const Move::Result & result) {
  if (round_trip_timed_out(op)) return;
  if (!result.executed_trajectory.joint_trajectory.points.empty() ||
      !result.executed_trajectory.multi_dof_joint_trajectory.points.empty()) {
    cancel("ROBOT_STATE_INVALID", op->step.name + ": Unexpected execution in a plan-only response");
    return;
  }
  auto & trip = *op->round_trip;
  try {
    validate_round_trip_path(result.planned_trajectory,
        config_.arms.at(trip.tilt.arm).joints);
  } catch (const std::exception & error) {
    fail_round_trip(op, "ROUND_TRIP_INVALID", error.what(), true);
    return;
  }
  trip.forward = result.planned_trajectory;
  trip.forward.joint_trajectory.header.stamp = builtin_interfaces::msg::Time{};
  trip.backward = reverse_round_trip_path(trip.forward, config_.arms.at(trip.tilt.arm).joints);
  const double duration =
      rclcpp::Duration(trip.forward.joint_trajectory.points.back().time_from_start).seconds();
  if (duration + config_.settle_time + config_.gripper_stop_window >=
      std::min(trip.tilt.timeout, trip.untilt.timeout)) {
    fail_round_trip(op, "ROUND_TRIP_INVALID", "Pilz path exceeds an execution timeout", true);
    return;
  }
  RCLCPP_INFO(node_.get_logger(),
      "round trip: original Pilz tilt %.3f s, exact return %.3f s, hold %.3f s",
      duration, duration, trip.hold.duration);
  trip.phase = RoundPhase::Validating;
  trip.phase_started = Clock::now();
  op->result.detail = "Validating tilt: original Pilz collision check shared with exact return";
  request_scene(op, true);
}

void TrajectoryExecutor::send_round_trip_path(
    const std::shared_ptr<Operation> & op, SmoothPath path) {
  if (round_trip_timed_out(op)) return;
  auto & trip = *op->round_trip;
  const bool returning = trip.phase == RoundPhase::ReturnValidating;
  if (path.trajectory != (returning ? trip.backward : trip.forward)) {
    throw std::runtime_error("Stored and validated round-trip paths differ");
  }
  std::string reason;
  if (!context_.ready(reason) || !round_trip_start_matches(*op, true, returning, reason)) {
    fail_round_trip(op, "ROBOT_STATE_INVALID", reason);
    return;
  }
  const auto & trajectory = path.trajectory.joint_trajectory;
  const double duration = rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
  const auto & step = returning ? trip.untilt : trip.tilt;
  if (duration + config_.settle_time + config_.gripper_stop_window >= step.timeout) {
    fail_round_trip(op, "ROUND_TRIP_INVALID", "Execution timeout cannot cover path and settling",
        !returning);
    return;
  }
  op->joint_targets.clear();
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    op->joint_targets[trajectory.joint_names[i]] = trajectory.points.back().positions[i];
  }
  trip.phase = returning ? RoundPhase::Untilt : RoundPhase::Tilt;
  trip.phase_started = op->started = Clock::now();
  op->stable_since.reset();
  trip.execution_result_at.reset();
  trip.wheel_settle_started.reset();
  trip.wheel_stable_since.reset();
  trip.wheel_peak_velocity = trip.wheel_peak_offset = 0;
  trip.wheel_peak_velocity_joint.clear();
  trip.wheel_peak_offset_joint.clear();
  trip.wheel_peaks_pending = true;
  op->path = std::move(path);
  op->planning_only = false;
  op->result.detail = returning ? "Executing exact time-reversed tilt path" : "Executing tilt path";
  Execute::Goal goal;
  goal.trajectory = op->path.trajectory;
  planned_trajectory_publisher_->publish(goal.trajectory.joint_trajectory);
  dispatch<Execute>(execute_client_, goal, op, [](const Execute::Result & result) {
    return result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS
        ? std::string{} : "MoveIt execution error " + std::to_string(result.error_code.val);
  });
  RCLCPP_INFO(node_.get_logger(), "round trip: executing %s, planned duration %.3f s",
      op->step.name.c_str(), duration);
}

bool TrajectoryExecutor::round_trip_timed_out(const std::shared_ptr<Operation> & op) {
  auto & trip = *op->round_trip;
  const double phase_timeout = trip.phase == RoundPhase::Hold ? trip.hold.timeout :
      trip.phase == RoundPhase::ReturnValidating || trip.phase == RoundPhase::Untilt
          ? trip.untilt.timeout : trip.tilt.timeout;
  if (elapsed(trip.started) >= trip.timeout || elapsed(trip.phase_started) >= phase_timeout) {
    cancel("TIMEOUT", op->step.name + ": round-trip phase or total timeout; " +
        op->completion_wait_reason);
    return true;
  }
  return false;
}

void TrajectoryExecutor::tick_round_trip(const std::shared_ptr<Operation> & op) {
  auto & trip = *op->round_trip;
  if (op->result.state != ExecutionState::Running) return;
  sample_wheel_peaks(*op);
  if (round_trip_timed_out(op)) return;
  std::string reason;
  const bool hold = trip.phase == RoundPhase::Hold;
  if (!context_.ready(reason)) {
    cancel("ROBOT_STATE_INVALID", op->step.name + ": " + reason);
    return;
  }
  RoundStateIssue issue = RoundStateIssue::None;
  const bool matches = trip.initial_positions.empty() ||
      round_trip_start_matches(*op, hold, hold, reason, &issue);
  const bool executing = trip.phase == RoundPhase::Tilt || trip.phase == RoundPhase::Untilt;
  const bool executed = executing && trip.execution_result_at &&
      op->action_terminal && op->action_success;
  if (executing && trip.execution_result_at) log_wheel_peaks(*op);
  // Arm deceleration kicks the wheels briefly; speed alone waits for the result.
  const bool awaiting_result = executing && !trip.execution_result_at;
  if (!matches && !(awaiting_result && issue == RoundStateIssue::WheelVelocity)) {
    if (!executed || issue != RoundStateIssue::WheelVelocity) {
      cancel("ROBOT_STATE_INVALID", op->step.name + ": " + reason);
      return;
    }
    if (!trip.wheel_settle_started) {
      trip.wheel_settle_started = Clock::now();
      RCLCPP_INFO(node_.get_logger(),
          "round trip: %s wheel stabilization started, limit %.3f s, settle %.3f s, cause=%s",
          op->step.name.c_str(), WHEEL_SETTLE_TIMEOUT, config_.settle_time, reason.c_str());
    }
    trip.wheel_stable_since.reset();
    op->stable_since.reset();
    op->completion_wait_reason = "Waiting for wheel stabilization: " + reason;
  }
  if (executed && trip.wheel_settle_started) {
    if (matches) {
      if (!trip.wheel_stable_since) trip.wheel_stable_since = Clock::now();
      // Verify the arm endpoint concurrently with continuous wheel stability.
      settled(op, true);
    }
    const bool stable = trip.wheel_stable_since &&
        elapsed(*trip.wheel_stable_since) >= config_.settle_time &&
        std::chrono::duration<double>(
            *trip.wheel_stable_since - *trip.wheel_settle_started).count() +
            config_.settle_time <= WHEEL_SETTLE_TIMEOUT;
    if (!stable) {
      if (elapsed(*trip.wheel_settle_started) >= WHEEL_SETTLE_TIMEOUT) {
        cancel("ROBOT_STATE_INVALID", op->step.name +
            ": Wheel stabilization timed out; " + op->completion_wait_reason);
      }
      return;
    }
  }
  if (op->preparation.valid() &&
      op->preparation.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const char * validation = trip.phase == RoundPhase::ReturnValidating
        ? "return revalidation" : "pre-tilt round-trip validation";
    SmoothPath path;
    try {
      path = op->preparation.get();
    } catch (const std::exception & error) {
      RCLCPP_INFO(node_.get_logger(),
          "round trip: %s failed, preparation %.3f s, phase %.3f s",
          validation, elapsed(op->preparation_started), elapsed(trip.phase_started));
      fail_round_trip(op, "ROUND_TRIP_INVALID", error.what(), true);
      return;
    }
    RCLCPP_INFO(node_.get_logger(),
        "round trip: %s passed, preparation %.3f s, phase %.3f s",
        validation, elapsed(op->preparation_started), elapsed(trip.phase_started));
    try {
      send_round_trip_path(op, std::move(path));
    } catch (const std::exception & error) {
      fail_round_trip(op, "SEND_FAILED", error.what());
    }
    return;
  }
  if (trip.phase == RoundPhase::Tilt || trip.phase == RoundPhase::Untilt) {
    if (!op->action_terminal || !op->action_success || !settled(op, true)) return;
    RCLCPP_INFO(node_.get_logger(), "round trip: %s completed and stopped in %.3f s",
        op->step.name.c_str(), elapsed(trip.phase_started));
    if (trip.execution_result_at) {
      RCLCPP_INFO(node_.get_logger(), "round trip: %s stop verification %.3f s",
          op->step.name.c_str(), elapsed(*trip.execution_result_at));
    }
    trip.last_completed = op->step.name;
    if (trip.phase == RoundPhase::Untilt) {
      op->result.state = ExecutionState::Succeeded;
      op->result.detail = "Round trip completed at the original tilt start joints and stopped";
      return;
    }
    trip.phase = RoundPhase::Hold;
    trip.phase_started = Clock::now();
    op->step = trip.hold;
    op->result.detail = "Holding after tilt endpoint and stop verification";
    return;
  }
  if (hold && elapsed(trip.phase_started) >= trip.hold.duration) {
    trip.last_completed = trip.hold.name;
    trip.phase = RoundPhase::ReturnValidating;
    trip.phase_started = Clock::now();
    op->step = trip.untilt;
    op->result.detail = "Revalidating the return against the latest complete planning scene";
    try {
      request_scene(op, true);
    } catch (const std::exception & error) {
      fail_round_trip(op, "SCENE_UNAVAILABLE", error.what());
    }
  }
}

void TrajectoryExecutor::sample_wheel_peaks(Operation & op) const {
  auto & trip = *op.round_trip;
  if (!trip.wheel_peaks_pending) return;
  const auto snapshot = context_.snapshot();
  for (const auto & [name, initial] : trip.initial_positions) {
    if (!is_drive_wheel(name)) continue;
    const auto measured = snapshot.joints.find(name);
    if (measured == snapshot.joints.end()) continue;
    const auto & velocity = measured->second.velocity;
    if (velocity && std::isfinite(*velocity) && std::abs(*velocity) > trip.wheel_peak_velocity) {
      trip.wheel_peak_velocity = std::abs(*velocity);
      trip.wheel_peak_velocity_joint = name;
    }
    const double offset = std::abs(measured->second.position - initial);
    if (std::isfinite(offset) && offset > trip.wheel_peak_offset) {
      trip.wheel_peak_offset = offset;
      trip.wheel_peak_offset_joint = name;
    }
  }
}

void TrajectoryExecutor::log_wheel_peaks(Operation & op) const {
  auto & trip = *op.round_trip;
  if (!trip.wheel_peaks_pending) return;
  trip.wheel_peaks_pending = false;
  const auto joint = [](const std::string & name) { return name.empty() ? "none" : name.c_str(); };
  RCLCPP_INFO(node_.get_logger(),
      "round trip: %s wheel peak velocity %.3f rad/s (%s), peak offset %.3f rad (%s), "
      "limits %.3f / %.3f",
      op.step.name.c_str(), trip.wheel_peak_velocity, joint(trip.wheel_peak_velocity_joint),
      trip.wheel_peak_offset, joint(trip.wheel_peak_offset_joint),
      wheel_velocity_limit(config_), config_.continuous_start_tolerance);
}
} // namespace demo_1f

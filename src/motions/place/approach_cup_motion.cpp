#include "demo_1f/motions/place/approach_cup_motion.hpp"
#include <algorithm>
#include <cmath>

namespace demo_1f {
namespace {
rclcpp::Logger logger() {
  return rclcpp::get_logger("handover_demo");
}
} // namespace

Step ApproachCupMotion::approach_step(const geometry_msgs::msg::Pose & pose, double speed) const {
  Step step;
  step.name = phase_;
  step.type = "pose";
  step.position_only = true;
  step.tcp_roll_limit = config_.tcp_roll_limit_deg * M_PI / 180;
  step.tcp_pitch_limit = config_.tcp_pitch_limit_deg * M_PI / 180;
  step.tcp_pitch_min = config_.tcp_pitch_min_deg * M_PI / 180;
  step.tcp_pitch_downward_only = config_.tcp_pitch_downward_only;
  step.tcp_yaw_inward_only = config_.tcp_yaw_inward_only;
  step.tcp_yaw_limit = config_.tcp_yaw_limit_deg * M_PI / 180;
  step.arm = arm_;
  step.frame = robot_.planning_frame;
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  step.target = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  step.timeout = 45;
  step.velocity_scale = step.acceleration_scale = speed;
  return step;
}

bool ApproachCupMotion::dispatch(const geometry_msgs::msg::Pose & pose, double speed) {
  return execute_step(approach_step(pose, speed));
}

std::vector<Step> ApproachCupMotion::other_arm_steps() const {
  const auto & name = config_.other_arm_return_sequence;
  if (name.empty()) return {};
  const auto sequence = robot_.motion_sequences.find(name);
  if (sequence == robot_.motion_sequences.end() || !sequence->second.validation_error.empty()) {
    RCLCPP_WARN(logger(), "paired approach off: sequence %s %s", name.c_str(),
        sequence == robot_.motion_sequences.end() ? "is not loaded"
                                                  : sequence->second.validation_error.c_str());
    return {};
  }
  std::vector<Step> steps;
  for (const auto & step : sequence->second.steps) {
    if (step.type == "joint" && step.arm != arm_) steps.push_back(step);
  }
  if (steps.empty()) return {};
  // Already at the sequence's last target (its own tolerance): nothing to pair.
  const auto & last = steps.back();
  const auto snapshot = context_.snapshot();
  const auto & joints = robot_.arms.at(last.arm).joints;
  bool home = true;
  for (size_t i = 0; i < joints.size() && i < last.target.size(); ++i) {
    const auto it = snapshot.joints.find(joints[i]);
    home = home && it != snapshot.joints.end() &&
           std::abs(it->second.position - last.target[i]) <= last.tolerance;
  }
  if (home) {
    RCLCPP_INFO(logger(), "paired approach skipped: %s arm already at %s/%s", last.arm.c_str(),
        name.c_str(), last.name.c_str());
    return {};
  }
  return steps;
}

bool ApproachCupMotion::next_sequential() {
  if (other_next_ < other_steps_.size()) {
    return execute_step(other_steps_[other_next_++]);
  }
  sequential_ = false;
  return dispatch(approach_target_.pose, config_.travel_scale);
}

bool ApproachCupMotion::begin_approach() {
  // The saved head cup target and obstacle were captured together in FIND_CUP.
  // Later head-camera noise or occlusion must not change this approach.
  phase("MOVE_TO_OBSERVE");
  // Independent return: TaskManager already started the other arm; this moves one arm only.
  other_steps_ = config_.independent_return ? std::vector<Step>{} : other_arm_steps();
  other_next_ = 0;
  if (other_steps_.empty()) {
    return dispatch(approach_target_.pose, config_.travel_scale);
  }
  std::string reason;
  if (!context_.ready(reason)) {
    fail("ROBOT_NOT_READY", reason);
    return false;
  }
  if (executor_.start_paired(approach_step(approach_target_.pose, config_.travel_scale),
          other_steps_)) {
    paired_ = moving_ = true;
    return true;
  }
  const auto outcome = executor_.result();
  RCLCPP_INFO(logger(), "paired approach failed before motion (%s: %s); falling back to sequential",
      outcome.code.c_str(), outcome.detail.c_str());
  sequential_ = true;
  return next_sequential();
}

void ApproachCupMotion::tick() {
  if (!running()) {
    return;
  }
  if (execute_) {
    executor_.tick();
  }
  if (scene_pending_) {
    const auto outcome = scene_.poll();
    if (outcome.state == ExecutionState::Running) {
      return;
    }
    scene_pending_ = false;
    if (outcome.state != ExecutionState::Succeeded) {
      fail(outcome.code, outcome.detail);
      return;
    }
    if (result_.state == "CANCELLING") {
      result_.state = "CANCELLED";
      result_.code = "CANCELLED";
      return;
    }
    begin_approach();
    return;
  }
  std::string reason;
  if (execute_ && !context_.ready(reason)) {
    if (moving_) {
      executor_.cancel("ROBOT_STATE_INVALID", reason);
    } else {
      fail("ROBOT_STATE_INVALID", reason);
      return;
    }
  }
  if (moving_) {
    const auto outcome = executor_.result();
    if (outcome.state == ExecutionState::Running || outcome.state == ExecutionState::Cancelling) {
      return;
    }
    moving_ = false;
    if (paired_ && outcome.state == ExecutionState::Failed &&
        outcome.code == "PAIRED_BEFORE_MOTION" && result_.state != "CANCELLING") {
      paired_ = false;
      RCLCPP_INFO(logger(), "paired approach failed before motion (%s); falling back to sequential",
          outcome.detail.c_str());
      sequential_ = true;
      next_sequential();
      return;
    }
    if (outcome.state == ExecutionState::StopUnconfirmed ||
        outcome.state == ExecutionState::Failed) {
      fail(outcome.code, outcome.detail);
      return;
    }
    if (result_.state == "CANCELLING" || outcome.state == ExecutionState::Cancelled) {
      result_.state = "CANCELLED";
      result_.code = "CANCELLED";
      return;
    }
    if (outcome.state != ExecutionState::Succeeded) {
      fail("EXECUTOR_STATE", "Unexpected executor state");
      return;
    }
    if (sequential_) { // another other-arm step, or the approach alone
      next_sequential();
      return;
    }
    paired_ = false;
    result_.last_completed_step = phase_;
    at_observation_pose_ = true;
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
    result_.detail = "Cup BODY approach reached; no gripper command; collision checks cover "
                     "registered scene; held-object geometry not modeled";
    phase("OBSERVATION_READY");
    return;
  }
  if (phase_ != "FIND_CUP") {
    return;
  }
  double head_error = 0;
  if (!observer_.poll(cup_id_, estimated_cup_pose_, head_error, reason)) {
    result_.detail = reason;
    if (elapsed(phase_started_) > config_.observation_timeout) {
      fail("CUP_NOT_FOUND", reason);
    }
    return;
  }
  const auto & body = estimated_cup_pose_.pose;
  approach_target_.header = estimated_cup_pose_.header;
  auto target = approach_tcp_target(body, config_, arm_, head_error, reason);
  if (!target) {
    fail("APPROACH_TARGET_INVALID", reason);
    return;
  }
  approach_target_.pose = *target;
  if (!execute_) {
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
    result_.detail = "Geometry preview only; IK/path not verified; BODY Z assumed at cup middle; "
                     "target Z = BODY Z + cup height + offset_z; held-object geometry not modeled";
    result_.last_completed_step = "PREVIEW_TARGET";
    phase("PREVIEW_READY");
    return;
  }
  phase("PREPARE_SCENE");
  scene_pending_ = scene_.apply(make_approach_scene(robot_, config_, body, head_error));
  if (!scene_pending_) {
    fail("SCENE_UNAVAILABLE", "Cannot prepare cup collision geometry");
  }
}
} // namespace demo_1f

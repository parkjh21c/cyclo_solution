#include "demo_1f/motions/pour/cup_pour_motion.hpp"
#include "demo_1f/runtime/config.hpp"
#include <cmath>
#include <sstream>

namespace demo_1f {
CupPourMotion::CupPourMotion(const Config & robot,
    const ApproachConfig & config,
    const PourConfig & pour,
    RobotContext & context,
    ExecutorPort & executor,
    ApproachScenePort & scene)
    : CupMotion(robot, config, context, executor, scene), pour_(pour),
      sequence_(robot, context, executor) {}

void CupPourMotion::reset() {
  stage_ = Stage::Idle;
  tilt_ = 0;
  tilt_start_joints_.clear();
  retries_.clear();
  started_.reset();
  total_seconds_.reset();
}

namespace {
Step lin(const std::string & name, const std::string & arm, const std::string & frame,
    const geometry_msgs::msg::Pose & pose, double speed, const std::string & type = "linear",
    double acceleration = 0) {
  Step s;
  s.name = name;
  s.type = type;
  s.arm = arm;
  s.frame = frame;
  s.velocity_scale = speed;
  s.acceleration_scale = acceleration > 0 ? acceleration : type == "linear" ? speed / 4 : speed;
  s.timeout = 60;
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  s.target = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  return s;
}

const Step * hold_insert(const Config & robot) {
  const auto it = robot.motion_sequences.find("cup_left_to_right_grasp");
  if (it == robot.motion_sequences.end()) return nullptr;
  for (const auto & step : it->second.steps) {
    if (step.type == "linear" && step.target.size() == 7) return &step;
  }
  return nullptr;
}

MotionSequence part(const std::string & arm, std::vector<Step> steps) {
  MotionSequence sequence;
  sequence.direction = "cup_pour_" + arm;
  sequence.giver = sequence.receiver = arm;
  sequence.steps = std::move(steps);
  return sequence;
}
} // namespace

std::optional<CupPourMotion::Parts> CupPourMotion::build(
    const std::string & arm, std::string & reason) const {
  const auto basket = context_.basket();
  if (!basket) {
    reason = "No basket measured yet; the right arm must visit the initial pose (wrist_basket_init)";
    return std::nullopt;
  }
  if (basket->header.frame_id != robot_.planning_frame) {
    reason = "Basket pose is not in " + robot_.planning_frame;
    return std::nullopt;
  }
  const auto tcp = context_.pose_of(robot_.arms.at(arm).tcp);
  if (!tcp) {
    reason = "Cannot read the " + arm + " TCP";
    return std::nullopt;
  }
  // The pour assumes the cup sits as the left-to-right handover leaves it.
  if (const auto * insert = hold_insert(robot_)) {
    const auto & p = tcp->pose.position;
    const double off = std::hypot(std::hypot(p.x - insert->target[0], p.y - insert->target[1]),
        p.z - insert->target[2]);
    if (off > pour_.hold_tolerance) {
      reason = "Right TCP is " + std::to_string(off) + " m from the transfer hold pose";
      return std::nullopt;
    }
  }
  const auto plan = plan_pour(tcp->pose, basket->pose, config_.cup_height, pour_, reason);
  if (!plan) return std::nullopt;
  std::vector<double> start_joints;
  const auto snapshot = context_.snapshot();
  for (const auto & name : robot_.arms.at(arm).joints) {
    const auto joint = snapshot.joints.find(name);
    if (joint == snapshot.joints.end()) {
      reason = "Missing joint feedback: " + name;
      return std::nullopt;
    }
    start_joints.push_back(joint->second.position);
  }
  const auto & frame = robot_.planning_frame;
  Parts parts;
  parts.basket = *basket;
  std::vector<Step> before = {lin("pour_above", arm, frame, plan->above, pour_.ptp_speed, "pose")};
  if (plan->raise > 0) {
    before.push_back(lin("pour_lower", arm, frame, plan->lowered, pour_.lin_speed));
  }
  parts.before = part(arm, before);
  parts.angles.push_back(pour_.angle_deg);
  parts.tilts.push_back(part(arm, {lin("pour_tilt", arm, frame, plan->tilted,
      pour_.tilt_speed, "linear", pour_.tilt_acceleration)}));
  for (size_t i = 0; i < plan->fallbacks.size(); ++i) {
    parts.angles.push_back(pour_.fallback_angles_deg[i]);
    parts.tilts.push_back(
        part(arm, {lin("pour_tilt", arm, frame, plan->fallbacks[i],
            pour_.tilt_speed, "linear", pour_.tilt_acceleration)}));
  }
  Step hold;
  hold.name = "pour_hold";
  hold.type = "wait";
  hold.arm = arm;
  hold.duration = pour_.hold_duration;
  hold.timeout = pour_.hold_duration + 5;
  parts.hold = hold;
  parts.untilt = lin("pour_untilt", arm, frame, plan->lowered,
      pour_.tilt_speed, "linear", pour_.tilt_acceleration);
  std::vector<Step> after;
  if (plan->raise > 0) {
    after.push_back(lin("pour_raise", arm, frame, plan->above, pour_.lin_speed));
  }
  Step back;
  back.name = "pour_return";
  back.type = "joint";
  back.arm = arm;
  back.target = start_joints;
  back.velocity_scale = back.acceleration_scale = pour_.ptp_speed;
  back.tolerance = M_PI / 180;
  back.planning_tolerance = .0005;
  back.timeout = 60;
  after.push_back(back);
  parts.after = part(arm, after);
  for (const auto * sequence : {&parts.before, &parts.after}) {
    reason = validate_motion_sequence(*sequence, robot_);
    if (!reason.empty()) return std::nullopt;
  }
  for (const auto & tilt : parts.tilts) {
    auto round_trip = tilt;
    round_trip.steps.push_back(parts.hold);
    round_trip.steps.push_back(parts.untilt);
    reason = validate_motion_sequence(round_trip, robot_);
    if (!reason.empty()) return std::nullopt;
  }
  return parts;
}

bool CupPourMotion::is_ready_to_start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) const {
  if (!CupMotion::is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  const auto parts = build(arm, reason);
  return parts && (!execute || sequence_.is_ready_to_start(parts->before, reason));
}

bool CupPourMotion::start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) {
  const auto started = accepted_at_.value_or(Clock::now());
  const bool accepted = accepted_at_.has_value();
  accepted_at_.reset();
  const auto rejected = [&]() {
    if (accepted) {
      started_ = started;
      total_seconds_ = elapsed(started);
      RCLCPP_INFO(rclcpp::get_logger("handover_demo"),
          "cup pour: total %.3f s, result=FAILED, code=NOT_READY, detail=%s",
          *total_seconds_, reason.c_str());
    }
    return false;
  };
  if (!is_ready_to_start(arm, cup_id, reason, execute)) {
    return rejected();
  }
  auto parts = build(arm, reason); // Frozen at start: the basket may update while pouring.
  if (!parts || !CupMotion::start(arm, cup_id, reason, execute)) {
    return rejected();
  }
  parts_ = *parts;
  started_ = started;
  const auto & b = parts_.basket;
  const auto & q = b.pose.orientation;
  const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  RCLCPP_INFO(rclcpp::get_logger("handover_demo"),
      "cup pour: basket %.3f %.3f %.3f yaw %.1f deg, age %.1f s, execute=%s",
      b.pose.position.x, b.pose.position.y, b.pose.position.z, yaw * 180 / M_PI,
      (context_.now() - rclcpp::Time(b.header.stamp)).seconds(),
      execute ? "true" : "false");
  return true;
}

void CupPourMotion::finish_timing() {
  if (!started_ || running() || total_seconds_) return;
  total_seconds_ = elapsed(*started_);
  RCLCPP_INFO(rclcpp::get_logger("handover_demo"),
      "cup pour: total %.3f s, result=%s, code=%s, last=%s, phase=%s, execute=%s",
      *total_seconds_, result_.state.c_str(), result_.code.c_str(),
      result_.last_completed_step.c_str(), context_.snapshot().phase.c_str(),
      execute_ ? "true" : "false");
}

bool CupPourMotion::run(const MotionSequence & sequence) {
  std::string reason;
  if (!sequence_.start(sequence, reason, "Cup pour; poured contents are not sensor-verified")) {
    fail("NOT_READY", reason);
    return false;
  }
  return true;
}

bool CupPourMotion::arm_unmoved() const {
  const auto snapshot = context_.snapshot();
  const auto & names = robot_.arms.at(arm_).joints;
  if (tilt_start_joints_.size() != names.size()) return false;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto joint = snapshot.joints.find(names[i]);
    if (joint == snapshot.joints.end() ||
        std::abs(joint->second.position - tilt_start_joints_[i]) >
            robot_.continuous_start_tolerance) {
      return false;
    }
  }
  return true;
}

bool CupPourMotion::run_round_trip() {
  phase(parts_.tilts[tilt_].steps.front().name);
  if (executor_.start_round_trip(parts_.tilts[tilt_].steps.front(), parts_.hold, parts_.untilt)) {
    return true;
  }
  const auto outcome = executor_.result();
  fail(outcome.code.empty() ? "COMMAND_REJECTED" : outcome.code, retries_ + outcome.detail);
  return false;
}

void CupPourMotion::finish_cancelled() {
  result_.state = "CANCELLED";
  result_.code = "CANCELLED";
  result_.detail = "Stopped; the cup may still be tilted, no automatic untilt was issued";
}

void CupPourMotion::cancel() {
  if (!running()) {
    return;
  }
  result_.state = "CANCELLING";
  if (stage_ == Stage::Tilt) {
    executor_.cancel();
  } else if (sequence_.running()) {
    sequence_.cancel();
  } else {
    finish_cancelled();
  }
  finish_timing();
}

void CupPourMotion::tick() {
  if (!running()) return;
  tick_pour();
  finish_timing();
}

void CupPourMotion::tick_pour() {
  if (!running()) {
    return;
  }
  if (!execute_) {
    // Preview: report the tilt pose as the target; nothing moves.
    const auto & target = parts_.tilts.front().steps.front().target;
    approach_target_.header.frame_id = robot_.planning_frame;
    approach_target_.pose.position.x = target[0];
    approach_target_.pose.position.y = target[1];
    approach_target_.pose.position.z = target[2];
    approach_target_.pose.orientation.x = target[3];
    approach_target_.pose.orientation.y = target[4];
    approach_target_.pose.orientation.z = target[5];
    approach_target_.pose.orientation.w = target[6];
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
    result_.detail = "Cup pour preview from the measured basket; nothing moved; "
                     "target_pose is the tilted TCP pose";
    return;
  }
  if (stage_ == Stage::Idle) {
    if (run(parts_.before)) stage_ = Stage::Before;
    return;
  }
  if (stage_ == Stage::Tilt) {
    executor_.tick();
    const auto outcome = executor_.result();
    if (!outcome.last_completed_step.empty()) {
      result_.last_completed_step = outcome.last_completed_step;
    }
    if (!outcome.phase.empty()) {
      context_.set_phase(outcome.phase, result_.last_completed_step);
    }
    result_.detail = outcome.detail;
    if (outcome.state == ExecutionState::Running || outcome.state == ExecutionState::Cancelling) {
      return;
    }
    if (outcome.state == ExecutionState::StopUnconfirmed) {
      fail(outcome.code, retries_ + outcome.detail);
      return;
    }
    if (outcome.state == ExecutionState::Cancelled) {
      finish_cancelled();
      return;
    }
    if (outcome.state != ExecutionState::Succeeded) {
      std::string reason;
      if (result_.state != "CANCELLING" && outcome.state == ExecutionState::Failed &&
          outcome.retryable_before_motion && tilt_ + 1 < parts_.tilts.size() &&
          context_.ready(reason) && arm_unmoved()) {
        std::ostringstream note;
        note << parts_.angles[tilt_] << " deg failed (" << outcome.detail << "); ";
        retries_ += note.str();
        ++tilt_;
        run_round_trip();
        return;
      }
      fail(outcome.code.empty() ? "EXECUTION_FAILED" : outcome.code, retries_ + outcome.detail);
      return;
    }
    if (result_.state == "CANCELLING") {
      finish_cancelled();
      return;
    }
    stage_ = Stage::After;
    run(parts_.after);
    return;
  }
  sequence_.tick();
  if (sequence_.running()) {
    result_.detail = sequence_.result().detail;
    return;
  }
  const auto & outcome = sequence_.result();
  if (!outcome.last_completed_step.empty()) {
    result_.last_completed_step = outcome.last_completed_step;
  }
  if (outcome.code == "STOP_UNCONFIRMED") {
    fail(outcome.code, retries_ + outcome.detail);
    return;
  }
  if (outcome.state == "CANCELLED") {
    finish_cancelled();
    return;
  }
  if (outcome.state != "SUCCEEDED") {
    fail(outcome.code.empty() ? "EXECUTION_FAILED" : outcome.code, retries_ + outcome.detail);
    return;
  }
  if (result_.state == "CANCELLING") {
    finish_cancelled();
    return;
  }
  if (stage_ == Stage::Before) {
    tilt_start_joints_.clear();
    const auto snapshot = context_.snapshot();
    for (const auto & name : robot_.arms.at(arm_).joints) {
      const auto joint = snapshot.joints.find(name);
      if (joint != snapshot.joints.end()) tilt_start_joints_.push_back(joint->second.position);
    }
    stage_ = Stage::Tilt;
    run_round_trip();
    return;
  }
  std::ostringstream detail;
  detail << arm_ << " hand poured at " << parts_.angles[tilt_]
         << " deg and returned holding the cup; not sensor-verified";
  if (!retries_.empty()) detail << "; " << retries_;
  at_observation_pose_ = true;
  result_.state = "SUCCEEDED";
  result_.sequence_completed = true;
  result_.detail = detail.str();
  phase("CUP_POUR_COMPLETE");
}
} // namespace demo_1f

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
}

namespace {
Step lin(const std::string & name, const std::string & arm, const std::string & frame,
    const geometry_msgs::msg::Pose & pose, double speed, const std::string & type = "linear") {
  Step s;
  s.name = name;
  s.type = type;
  s.arm = arm;
  s.frame = frame;
  s.velocity_scale = speed;
  s.acceleration_scale = type == "linear" ? speed / 4 : speed;
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
  std::vector<Step> before = {lin("pour_above", arm, frame, plan->above, pour_.ptp_speed, "pose")};
  if (plan->raise > 0) {
    before.push_back(lin("pour_lower", arm, frame, plan->lowered, pour_.lin_speed));
  }
  parts.before = part(arm, before);
  parts.angles.push_back(pour_.angle_deg);
  parts.tilts.push_back(part(arm, {lin("pour_tilt", arm, frame, plan->tilted, pour_.lin_speed)}));
  for (size_t i = 0; i < plan->fallbacks.size(); ++i) {
    parts.angles.push_back(pour_.fallback_angles_deg[i]);
    parts.tilts.push_back(
        part(arm, {lin("pour_tilt", arm, frame, plan->fallbacks[i], pour_.lin_speed)}));
  }
  Step hold;
  hold.name = "pour_hold";
  hold.type = "wait";
  hold.arm = arm;
  hold.duration = pour_.hold_duration;
  hold.timeout = pour_.hold_duration + 5;
  std::vector<Step> after = {hold, lin("pour_untilt", arm, frame, plan->lowered, pour_.lin_speed)};
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
    reason = validate_motion_sequence(tilt, robot_);
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
  if (!is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  auto parts = build(arm, reason); // Frozen at start: the basket may update while pouring.
  if (!parts || !CupMotion::start(arm, cup_id, reason, execute)) {
    return false;
  }
  parts_ = *parts;
  return true;
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
  if (sequence_.running()) {
    sequence_.cancel();
  } else {
    finish_cancelled();
  }
}

void CupPourMotion::tick() {
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
  sequence_.tick();
  if (sequence_.running()) {
    result_.detail = sequence_.result().detail;
    return;
  }
  const auto & outcome = sequence_.result();
  result_.last_completed_step = outcome.last_completed_step;
  if (outcome.state == "CANCELLED" || (result_.state == "CANCELLING" && outcome.state != "SUCCEEDED")) {
    finish_cancelled();
    return;
  }
  if (outcome.state != "SUCCEEDED") {
    // A tilt that failed before the arm moved (e.g. a wrist limit) is retried less steeply.
    if (stage_ == Stage::Tilt && outcome.state == "FAILED" && tilt_ + 1 < parts_.tilts.size() &&
        arm_unmoved()) {
      std::ostringstream note;
      note << parts_.angles[tilt_] << " deg failed (" << outcome.detail << "); ";
      retries_ += note.str();
      ++tilt_;
      run(parts_.tilts[tilt_]);
      return;
    }
    fail(outcome.code.empty() ? "EXECUTION_FAILED" : outcome.code, retries_ + outcome.detail);
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
    run(parts_.tilts[tilt_]);
    return;
  }
  if (stage_ == Stage::Tilt) {
    stage_ = Stage::After;
    run(parts_.after);
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

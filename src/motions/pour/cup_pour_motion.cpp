#include "demo_1f/motions/pour/cup_pour_motion.hpp"
#include "demo_1f/runtime/config.hpp"

namespace demo_1f {
CupPourMotion::CupPourMotion(const Config & robot,
    const ApproachConfig & config,
    RobotContext & context,
    ExecutorPort & executor,
    ApproachScenePort & scene)
    : CupMotion(robot, config, context, executor, scene), sequence_(robot, context, executor) {}

void CupPourMotion::reset() {
  started_ = false;
}

const MotionSequence * CupPourMotion::sequence(
    const std::string & arm, std::string & reason) const {
  const std::string name = "cup_pour_" + arm;
  const auto it = robot_.motion_sequences.find(name);
  if (it == robot_.motion_sequences.end() || !it->second.validation_error.empty()) {
    reason = name + " not configured: " +
             (it == robot_.motion_sequences.end() ? "missing" : it->second.validation_error);
    return nullptr;
  }
  return &it->second;
}

bool CupPourMotion::is_ready_to_start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) const {
  if (!CupMotion::is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  const auto * selected = sequence(arm, reason);
  return selected && (!execute || sequence_.is_ready_to_start(*selected, reason));
}

bool CupPourMotion::start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) {
  if (!is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  return CupMotion::start(arm, cup_id, reason, execute);
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
    const auto * selected = sequence(arm_, result_.detail);
    if (!selected) {
      fail("NOT_READY", result_.detail);
      return;
    }
    approach_target_.header.frame_id = robot_.planning_frame;
    for (const auto & step : selected->steps) {
      if (step.name == "pour_tilt" && step.type == "linear") {
        approach_target_.pose.position.x = step.target[0];
        approach_target_.pose.position.y = step.target[1];
        approach_target_.pose.position.z = step.target[2];
        approach_target_.pose.orientation.x = step.target[3];
        approach_target_.pose.orientation.y = step.target[4];
        approach_target_.pose.orientation.z = step.target[5];
        approach_target_.pose.orientation.w = step.target[6];
      }
    }
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
    result_.detail = "Cup pour preview; sequence is configured, nothing moved; "
                     "target_pose is the tilted TCP pose";
    return;
  }
  if (!started_) {
    std::string reason;
    const auto * selected = sequence(arm_, reason);
    if (!selected ||
        !sequence_.start(*selected, reason, "Cup pour; poured contents are not sensor-verified")) {
      fail("NOT_READY", reason);
      return;
    }
    started_ = true;
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
    fail(outcome.code.empty() ? "EXECUTION_FAILED" : outcome.code, outcome.detail);
    return;
  }
  at_observation_pose_ = true;
  result_.state = "SUCCEEDED";
  result_.sequence_completed = true;
  result_.detail = arm_ + " hand poured and returned holding the cup; not sensor-verified";
  phase("CUP_POUR_COMPLETE");
}
} // namespace demo_1f

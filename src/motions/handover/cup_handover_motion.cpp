#include "demo_1f/motions/handover/cup_handover_motion.hpp"
#include "demo_1f/runtime/config.hpp"

namespace demo_1f {
CupHandoverMotion::CupHandoverMotion(const Config & robot,
    const ApproachConfig & config,
    RobotContext & context,
    ExecutorPort & executor,
    ApproachScenePort & scene,
    std::string giver)
    : CupMotion(robot, config, context, executor, scene),
      giver_(giver),
      receiver_(giver == "left" ? "right" : "left"),
      sequence_(robot, context, executor) {}

void CupHandoverMotion::reset() {
  running_sequence_.clear();
  cancel_requested_ = false;
}

const MotionSequence * CupHandoverMotion::sequence(
    const std::string & name, std::string & reason) const {
  const auto it = robot_.motion_sequences.find(name);
  if (it == robot_.motion_sequences.end() || !it->second.validation_error.empty()) {
    reason = name + " not configured: " +
             (it == robot_.motion_sequences.end() ? "missing" : it->second.validation_error);
    return nullptr;
  }
  return &it->second;
}

bool CupHandoverMotion::is_ready_to_start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) const {
  if (!CupMotion::is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  const std::string base = "cup_" + giver_ + "_to_" + receiver_;
  const auto * first = sequence(base, reason);
  if (!first || !sequence(base + "_grasp", reason) || !sequence(base + "_release", reason)) {
    return false;
  }
  return !execute || sequence_.is_ready_to_start(*first, reason);
}

bool CupHandoverMotion::start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) {
  if (!is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  return CupMotion::start(arm, cup_id, reason, execute);
}

bool CupHandoverMotion::start_sequence(const std::string & name) {
  std::string reason;
  const auto * selected = sequence(name, reason);
  if (!selected || !sequence_.start(*selected, reason,
                       "Cup handover; transfer and grasp are not sensor-verified")) {
    fail("NOT_READY", reason);
    return false;
  }
  running_sequence_ = name;
  return true;
}

bool CupHandoverMotion::apply_swap_scene() {
  // In order: receiver finger contact, detach from the giver, attach to the receiver.
  const auto scene = phase_ == "CUP_CONTACT_SCENE" ? make_cup_contact_scene(robot_, receiver_) :
      phase_ == "CUP_DETACH" ? make_cup_detach_scene(robot_, giver_) :
                               make_cup_attach_scene(robot_, receiver_, giver_);
  scene_pending_ = scene_.apply(scene);
  if (!scene_pending_) {
    fail("SCENE_UNAVAILABLE", "Cannot move the cup model between hands");
  }
  return scene_pending_;
}

void CupHandoverMotion::finish_cancelled() {
  result_.state = "CANCELLED";
  result_.code = "CANCELLED";
  result_.detail = "Stopped; no automatic release or retreat was issued";
}

void CupHandoverMotion::cancel() {
  if (!running()) {
    return;
  }
  cancel_requested_ = true;
  result_.state = "CANCELLING";
  if (sequence_.running()) {
    sequence_.cancel();
  } else if (!scene_pending_ && phase_ != "CUP_DETACH" && phase_ != "CUP_ATTACH") {
    finish_cancelled();
  }
  // A scene swap in flight completes first so the cup model never ends up detached.
}

void CupHandoverMotion::tick() {
  if (!running()) {
    return;
  }
  const std::string base = "cup_" + giver_ + "_to_" + receiver_;
  if (!execute_) {
    const auto * first = sequence(base + "_grasp", result_.detail);
    approach_target_.header.frame_id = robot_.planning_frame;
    if (first && sequence(base, result_.detail)) {
      for (const auto & step : first->steps) {
        if (step.type == "linear" && step.arm == receiver_) {
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
      result_.detail = "Cup handover preview; sequences are configured, nothing moved; "
                       "target_pose is the receiver's insert pose";
    } else {
      fail("NOT_READY", result_.detail);
    }
    return;
  }
  if (scene_pending_) {
    const auto outcome = scene_.poll();
    if (outcome.state == ExecutionState::Running) {
      return;
    }
    scene_pending_ = false;
    if (outcome.state != ExecutionState::Succeeded) {
      fail(outcome.code, phase_ + ": " + outcome.detail);
      return;
    }
    result_.last_completed_step = phase_;
    if (phase_ == "CUP_CONTACT_SCENE") {
      if (cancel_requested_) {
        finish_cancelled();
        return;
      }
      phase("CUP_HANDOVER_GRASP");
      start_sequence(base + "_grasp");
      return;
    }
    if (phase_ == "CUP_DETACH") {
      phase("CUP_ATTACH");
      apply_swap_scene();
      return;
    }
    if (cancel_requested_) {
      finish_cancelled();
      return;
    }
    phase("CUP_HANDOVER_RELEASE");
    start_sequence(base + "_release");
    return;
  }
  if (running_sequence_.empty()) {
    start_sequence(base);
    return;
  }
  sequence_.tick();
  if (sequence_.running()) {
    result_.detail = sequence_.result().detail;
    return;
  }
  const auto & outcome = sequence_.result();
  const std::string done = running_sequence_;
  running_sequence_.clear();
  if (outcome.state == "CANCELLED" || (cancel_requested_ && outcome.state != "SUCCEEDED")) {
    finish_cancelled();
    return;
  }
  if (outcome.state != "SUCCEEDED") {
    fail(outcome.code.empty() ? "EXECUTION_FAILED" : outcome.code,
        done + ": " + outcome.detail);
    return;
  }
  result_.last_completed_step = outcome.last_completed_step;
  if (done == base) {
    if (cancel_requested_) {
      finish_cancelled();
      return;
    }
    phase("CUP_CONTACT_SCENE"); // Only now may the receiver touch the cup.
    apply_swap_scene();
    return;
  }
  if (done == base + "_grasp") {
    if (cancel_requested_) {
      finish_cancelled();
      return;
    }
    phase("CUP_DETACH");
    apply_swap_scene();
    return;
  }
  at_observation_pose_ = true;
  result_.state = "SUCCEEDED";
  result_.sequence_completed = true;
  result_.detail = receiver_ + " hand holds the cup; transfer is not sensor-verified";
  phase("CUP_HANDOVER_COMPLETE");
}
} // namespace demo_1f

#include "demo_1f/motions/handover/cup_handover_motion.hpp"
#include "demo_1f/runtime/config.hpp"
#include <algorithm>
#include <cmath>

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
  cancel_requested_ = continuous_tried_ = inserted_ = false;
  start_joints_.clear();
}

std::vector<Step> CupHandoverMotion::continuous_steps(std::string & reason) const {
  const std::string base = "cup_" + giver_ + "_to_" + receiver_;
  if (!robot_.cup_handover_continuous || giver_ != "left") {
    reason = "disabled";
    return {};
  }
  const auto * transfer = sequence(base, reason);
  const auto * insert = transfer ? sequence(base + "_continuous", reason) : nullptr;
  if (!insert) return {};
  if (transfer->steps.size() != 1 || insert->steps.size() != 1 ||
      transfer->steps[0].type != "dual_joint" || insert->steps[0].type != "dual_joint") {
    reason = "needs one dual_joint step each in " + base + " and " + base + "_continuous";
    return {};
  }
  auto ready = transfer->steps[0];
  auto line = insert->steps[0];
  if (line.left_target != ready.left_target) {
    reason = "insert left_target must equal cup_ready (the giver holds still)";
    return {};
  }
  ready.strict_objects = {"demo_place_cup"}; // Cup checks stay on until the insert.
  line.arm = receiver_;
  line.endpoints_only = true; // Straight joint segment, entered along its own direction.
  line.tcp_line_tolerance = robot_.cup_handover_insert_line_tolerance;
  return {ready, line};
}

void CupHandoverMotion::start_continuous() {
  phase("CUP_HANDOVER_CONTINUOUS");
  std::string reason;
  const auto steps = continuous_steps(reason);
  if (steps.empty()) {
    fall_back(reason);
    return;
  }
  if (!context_.ready(reason) || !executor_.ready(MotionSequence{"", "", "", steps, ""}, reason)) {
    fall_back("not ready: " + reason);
    return;
  }
  if (!executor_.start_sequence(steps)) {
    fall_back("not started: " + executor_.result().detail);
    return;
  }
  moving_ = true;
}

void CupHandoverMotion::finish_continuous() {
  executor_.tick(); // Advances preparation, execution and timeouts; nothing else ticks it here.
  const auto r = executor_.result();
  if (r.state == ExecutionState::Running || r.state == ExecutionState::Cancelling) {
    result_.detail = r.detail;
    return;
  }
  moving_ = false;
  if (r.state == ExecutionState::StopUnconfirmed) {
    fail("STOP_UNCONFIRMED", r.detail);
    return;
  }
  if (r.state == ExecutionState::Cancelled ||
      (cancel_requested_ && r.state != ExecutionState::Succeeded)) {
    finish_cancelled();
    return;
  }
  if (r.state != ExecutionState::Succeeded) {
    if (r.state == ExecutionState::Failed && arms_unmoved()) {
      fall_back("failed before motion: " + r.detail);
      return;
    }
    fail(r.code.empty() ? "EXECUTION_FAILED" : r.code, phase_ + ": " + r.detail);
    return;
  }
  result_.last_completed_step = r.last_completed_step;
  if (cancel_requested_) {
    finish_cancelled();
    return;
  }
  inserted_ = true; // The grasp sequence skips its LIN insert.
  phase("CUP_HANDOVER_GRASP");
  start_sequence("cup_" + giver_ + "_to_" + receiver_ + "_grasp");
}

void CupHandoverMotion::fall_back(const std::string & why) {
  RCLCPP_WARN(rclcpp::get_logger("handover_demo.cup"),
      "Cup handover falls back to separate steps: %s", why.c_str());
  phase("CUP_CONTACT_REVOKE_SCENE"); // Cup checks back on before the separate cup_ready.
  apply_swap_scene();
}

bool CupHandoverMotion::arms_unmoved() const {
  const auto snapshot = context_.snapshot();
  for (const auto & [name, position] : start_joints_) {
    const auto joint = snapshot.joints.find(name);
    if (joint == snapshot.joints.end() ||
        std::abs(joint->second.position - position) > robot_.continuous_start_tolerance) {
      return false;
    }
  }
  return !start_joints_.empty();
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
  if (execute && giver_ == "right" && config_.independent_return) {
    const auto error = independent_return_error(robot_, config_);
    if (!error.empty()) {
      reason = "Independent cup return: " + error;
      return false;
    }
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
  MotionSequence execution;
  if (selected) {
    execution = *selected;
    if (inserted_ && name == "cup_" + giver_ + "_to_" + receiver_ + "_grasp") {
      execution.steps.erase(std::remove_if(execution.steps.begin(), execution.steps.end(),
          [this](const Step & step) { return step.type == "linear" && step.arm == receiver_; }),
          execution.steps.end());
    }
    if (giver_ == "right" && config_.independent_return &&
        name == "cup_right_to_left_release") {
      execution.steps.erase(std::remove_if(execution.steps.begin(), execution.steps.end(),
          [](const Step & step) { return step.name == "giver_via" || step.name == "giver_home"; }),
          execution.steps.end());
    }
  }
  if (!selected || !sequence_.start(execution, reason,
                       "Cup handover; transfer and grasp are not sensor-verified")) {
    fail("NOT_READY", reason);
    return false;
  }
  running_sequence_ = name;
  return true;
}

bool CupHandoverMotion::apply_swap_scene() {
  // In order: receiver finger contact, detach from the giver, attach to the receiver.
  const bool contact = phase_ == "CUP_CONTACT_SCENE" || phase_ == "CUP_CONTINUOUS_SCENE";
  const auto scene = contact ? make_cup_contact_scene(robot_, receiver_) :
      phase_ == "CUP_CONTACT_REVOKE_SCENE" ? make_cup_contact_scene(robot_, receiver_, false) :
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
  } else if (moving_) {
    executor_.cancel();
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
    if (phase_ == "CUP_CONTINUOUS_SCENE" || phase_ == "CUP_CONTACT_REVOKE_SCENE") {
      if (cancel_requested_) {
        finish_cancelled();
      } else if (phase_ == "CUP_CONTINUOUS_SCENE") {
        start_continuous();
      } else {
        phase("CUP_HANDOVER_TRANSFER");
        start_sequence(base);
      }
      return;
    }
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
  if (moving_) {
    finish_continuous();
    return;
  }
  if (running_sequence_.empty()) {
    std::string why;
    if (!continuous_tried_ && !continuous_steps(why).empty()) {
      continuous_tried_ = true;
      // Both arms' start joints: a rejection with neither arm moved may fall back.
      const auto snapshot = context_.snapshot();
      for (const auto & [side, arm] : robot_.arms) {
        (void)side;
        for (const auto & name : arm.joints) {
          const auto joint = snapshot.joints.find(name);
          if (joint != snapshot.joints.end()) start_joints_[name] = joint->second.position;
        }
      }
      phase("CUP_CONTINUOUS_SCENE"); // Contact allowed first; cup_ready keeps strict cup checks.
      apply_swap_scene();
      return;
    }
    continuous_tried_ = true;
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

#include "demo_1f/motions/initial/initial_pose_motion.hpp"
#include <cmath>
#include <map>
#include <utility>

namespace demo_1f {
namespace {
const char * const sequence_detail = "Moving arms, head and lift to the DYNAMIXEL observation pose";
} // namespace

bool InitialPoseMotion::is_ready_to_start(std::string & reason) const {
  const auto it = config_.motion_sequences.find("initial_pose");
  if (it == config_.motion_sequences.end()) {
    reason = "Initial pose sequence is not configured";
    return false;
  }
  if (releasing_) {
    reason = "Initial pose is releasing the held cup model";
    return false;
  }
  if (scene_ && !scene_->ready()) {
    reason = "Planning scene busy or unavailable";
    return false;
  }
  return SequenceMotion::is_ready_to_start(it->second, reason);
}

bool InitialPoseMotion::at_initial_pose(const std::string & arm, std::string & reason) const {
  const auto sequence = config_.motion_sequences.find("initial_pose");
  const auto side = config_.arms.find(arm);
  if (sequence == config_.motion_sequences.end() || side == config_.arms.end()) {
    reason = "Initial pose or arm not configured: " + arm;
    return false;
  }
  std::map<std::string, std::pair<double, double>> targets; // Joint -> last target, tolerance.
  for (const auto & step : sequence->second.steps) {
    std::vector<std::string> names;
    if (step.type == "joint_group") names = step.joint_names;
    else if (step.type == "joint" && config_.arms.count(step.arm)) {
      names = config_.arms.at(step.arm).joints;
    }
    for (size_t i = 0; i < names.size() && i < step.target.size(); ++i) {
      targets[names[i]] = {step.target[i], step.tolerance};
    }
  }
  auto names = side->second.joints;
  if (targets.count("lift_joint")) names.push_back("lift_joint"); // The lift carries both arms.
  const auto snapshot = context_.snapshot();
  for (const auto & name : names) {
    const auto target = targets.find(name);
    const auto joint = snapshot.joints.find(name);
    if (target == targets.end()) {
      reason = "No initial pose target for " + name;
      return false;
    }
    if (joint == snapshot.joints.end()) {
      reason = "Missing joint feedback: " + name;
      return false;
    }
    if (std::abs(joint->second.position - target->second.first) > target->second.second) {
      reason = "Not at the initial pose: " + name;
      return false;
    }
    if (!context_.joint_stationary(name, joint->second, reason)) return false;
  }
  reason.clear();
  return true;
}

bool InitialPoseMotion::start(std::string & reason) {
  if (!is_ready_to_start(reason)) return false;
  release_result_ = {};
  cancel_requested_ = false;
  if (!scene_) {
    return SequenceMotion::start(
        config_.motion_sequences.at("initial_pose"), reason, sequence_detail);
  }
  if (!scene_->apply(make_cup_release_scene(config_))) {
    reason = "Cannot release held cup model";
    return false;
  }
  releasing_ = true;
  release_result_.state = "RUNNING";
  release_result_.detail = "Releasing held cup model before opening grippers";
  context_.set_phase("RELEASE_CUP_MODEL", {});
  return true;
}

void InitialPoseMotion::tick() {
  if (!releasing_) {
    SequenceMotion::tick();
    return;
  }
  const auto outcome = scene_->poll();
  if (outcome.state == ExecutionState::Running) return;
  releasing_ = false;
  if (outcome.state != ExecutionState::Succeeded) {
    release_result_.state = outcome.code == "SCENE_UNCONFIRMED" ? "FAULT" : "FAILED";
    release_result_.code = outcome.code;
    release_result_.detail = "RELEASE_CUP_MODEL: " + outcome.detail;
    return;
  }
  if (cancel_requested_) {
    release_result_.state = "CANCELLED";
    release_result_.code = "CANCELLED";
    release_result_.detail = "Cancelled before any motion";
    return;
  }
  std::string reason;
  if (!SequenceMotion::start(
          config_.motion_sequences.at("initial_pose"), reason, sequence_detail)) {
    release_result_.state = "FAILED";
    release_result_.code = "NOT_READY";
    release_result_.detail = reason;
    return;
  }
  release_result_ = {}; // The sequence owns the result from here.
}

void InitialPoseMotion::cancel() {
  if (releasing_) {
    cancel_requested_ = true; // The scene call cannot be cancelled; stop before moving.
    release_result_.state = "CANCELLING";
    return;
  }
  SequenceMotion::cancel();
}
} // namespace demo_1f

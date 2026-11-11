#include "demo_1f/motions/place/place_cup_motion.hpp"
#include <cmath>

namespace demo_1f {
PlaceCupMotion::PlaceCupMotion(const Config & robot,
    const ApproachConfig & config,
    const PickCupConfig & pick,
    RobotContext & context,
    ExecutorPort & executor,
    ApproachScenePort & scene,
    const PickCupMotion & picker)
    : CupMotion(robot, config, context, executor, scene), pick_(pick), picker_(picker) {}

void PlaceCupMotion::reset() {
  started_ = false;
}

bool PlaceCupMotion::is_ready_to_start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) const {
  if (!CupMotion::is_ready_to_start(arm, cup_id, reason, execute)) {
    return false;
  }
  reason = pick_.validate(config_.cup_height);
  if (!reason.empty()) {
    return false;
  }
  const auto & record = picker_.last_pick();
  if (!record) {
    reason = "No recorded pick to reverse; run /pick_cup first";
    return false;
  }
  if (record->arm != arm) {
    reason = "The recorded pick used the " + record->arm + " hand";
    return false;
  }
  if (record->lifted_joints.size() != robot_.arms.at(arm).joints.size()) {
    reason = "Recorded pick has incomplete joints";
    return false;
  }
  return true;
}

bool PlaceCupMotion::start(const std::string & arm,
    const std::string & cup_id,
    std::string & reason,
    bool execute) {
  if (!is_ready_to_start(arm, cup_id, reason, execute) ||
      !CupMotion::start(arm, cup_id, reason, execute)) {
    return false;
  }
  record_ = *picker_.last_pick();
  release_ = record_.grasp;
  release_.position.z += pick_.place_clearance;
  return true;
}

bool PlaceCupMotion::carry() {
  Step s;
  s.name = "PLACE_CARRY";
  s.type = "joint"; // Same joint line as cup_carry, reversed.
  s.arm = arm_;
  s.target = record_.lifted_joints;
  s.velocity_scale = .3;
  s.acceleration_scale = .3;
  s.tolerance = M_PI / 180;
  s.planning_tolerance = .0005;
  s.timeout = 60;
  phase(s.name);
  return execute_step(s);
}

bool PlaceCupMotion::linear(const std::string & name, const geometry_msgs::msg::Pose & pose) {
  Step s;
  s.name = name;
  s.type = "linear";
  s.arm = arm_;
  s.frame = robot_.planning_frame;
  s.velocity_scale = pick_.insert_speed;
  s.acceleration_scale = s.velocity_scale * .25; // Same LIN ratio as the pick.
  s.timeout = 30;
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  s.target = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  phase(name);
  approach_target_.header.frame_id = robot_.planning_frame;
  approach_target_.pose = pose;
  return execute_step(s);
}

bool PlaceCupMotion::open_gripper() {
  Step s;
  s.name = "PLACE_OPEN";
  s.type = "gripper";
  s.arm = arm_;
  s.target = {0.0};
  s.duration = 1.0;
  s.timeout = 8;
  phase(s.name);
  return execute_step(s);
}

bool PlaceCupMotion::detach() {
  phase("PLACE_DETACH");
  scene_pending_ = scene_.apply(make_cup_place_scene(robot_, arm_));
  if (!scene_pending_) {
    fail("SCENE_UNAVAILABLE", "Cannot leave the cup model at the place pose");
  }
  return scene_pending_;
}

void PlaceCupMotion::tick() {
  if (!running()) {
    return;
  }
  if (!execute_) {
    approach_target_.header.frame_id = robot_.planning_frame;
    approach_target_.pose = release_;
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
    result_.detail = "Place preview; nothing moved; target_pose is the release TCP pose";
    return;
  }
  if (!started_) {
    started_ = true;
    carry();
    return;
  }
  std::string health;
  if (!context_.ready(health)) {
    if (moving_) {
      executor_.cancel("ROBOT_STATE_INVALID", health);
    } else if (!scene_pending_) {
      fail("ROBOT_STATE_INVALID", health);
      return;
    }
  }
  executor_.tick();
  if (scene_pending_) {
    const auto r = scene_.poll();
    if (r.state == ExecutionState::Running) {
      return;
    }
    scene_pending_ = false;
    if (r.state != ExecutionState::Succeeded) {
      fail(r.code, phase_ + ": " + r.detail);
      return;
    }
    result_.last_completed_step = phase_;
    if (result_.state == "CANCELLING") {
      result_.state = "CANCELLED";
      result_.code = "CANCELLED";
      result_.detail = "Stopped after the release; the hand is still at the cup";
      return;
    }
    linear("PLACE_RETREAT", record_.pregrasp);
    return;
  }
  if (!moving_) {
    return;
  }
  const auto r = executor_.result();
  if (r.state == ExecutionState::Running || r.state == ExecutionState::Cancelling) {
    return;
  }
  moving_ = false;
  if (r.state == ExecutionState::StopUnconfirmed) {
    fail("STOP_UNCONFIRMED", r.detail);
    return;
  }
  if (result_.state == "CANCELLING" || r.state == ExecutionState::Cancelled) {
    result_.state = "CANCELLED";
    result_.code = "CANCELLED";
    result_.detail = "Stopped in place; no automatic release or retreat was issued";
    return;
  }
  if (r.state != ExecutionState::Succeeded) {
    fail(r.code.empty() ? "EXECUTION_FAILED" : r.code, phase_ + ": " + r.detail);
    return;
  }
  result_.last_completed_step = phase_;
  if (phase_ == "PLACE_CARRY") {
    linear("PLACE_LOWER", release_);
  } else if (phase_ == "PLACE_LOWER") {
    open_gripper(); // Only after the cup is down.
  } else if (phase_ == "PLACE_OPEN") {
    detach();
  } else if (phase_ == "PLACE_RETREAT") {
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
    result_.detail = arm_ + " hand placed the cup at the pick spot and stopped open at the "
                            "pregrasp; placement is not sensor-verified";
    phase("PLACE_COMPLETE");
  }
}
} // namespace demo_1f

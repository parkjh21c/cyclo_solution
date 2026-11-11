#include "demo_1f/motions/pick/pick_cup_motion.hpp"
#include "demo_1f/runtime/geometry/tcp_roll.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace demo_1f {
PickCupMotion::PickCupMotion(const Config & robot,
    const ApproachConfig & config,
    const PickCupConfig & pick,
    RobotContext & context,
    ExecutorPort & executor,
    ApproachScenePort & scene)
    : CupMotion(robot, config, context, executor, scene), observer_(context, config), pick_(pick) {}

void PickCupMotion::reset() {
  start_joints_.clear();
  direct_failure_.clear();
}

void PickCupMotion::record_pick() {
  PickRecord record{arm_, pregrasp_, grasp_tcp_, {}};
  const auto snapshot = context_.snapshot();
  for (const auto & name : robot_.arms.at(arm_).joints) {
    const auto joint = snapshot.joints.find(name);
    if (joint == snapshot.joints.end()) {
      return; // No place target without full joint feedback.
    }
    record.lifted_joints.push_back(joint->second.position);
  }
  last_pick_ = record;
}

bool PickCupMotion::move(const std::string & name, const geometry_msgs::msg::Pose & pose) {
  Step s;
  s.name = name;
  s.type = "pose"; // Full pose: Pilz PTP from the nearest IK solution; no wide arm swings.
  s.arm = arm_;
  s.frame = robot_.planning_frame;
  s.velocity_scale = pick_.speed;
  s.acceleration_scale = .08;
  s.timeout = 45;
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  s.target = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  phase(name);
  approach_target_.header.frame_id = robot_.planning_frame;
  approach_target_.pose = pose;
  return execute_step(s);
}

bool PickCupMotion::linear(const std::string & name, const geometry_msgs::msg::Pose & pose) {
  Step s;
  s.name = name;
  s.type = "linear";
  s.arm = arm_;
  s.frame = robot_.planning_frame;
  s.velocity_scale = pick_.insert_speed;
  s.acceleration_scale = s.velocity_scale * .25; // Same LIN ratio as refine/withdraw.
  s.timeout = 30;
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  s.target = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  phase(name);
  approach_target_.header.frame_id = robot_.planning_frame;
  approach_target_.pose = pose;
  return execute_step(s);
}

bool PickCupMotion::open_gripper() {
  Step s;
  s.name = "PICK_OPEN";
  s.type = "gripper";
  s.arm = arm_;
  s.target = {0.0};
  s.duration = 1.0;
  s.timeout = 8;
  phase(s.name);
  return execute_step(s);
}

bool PickCupMotion::close_gripper() {
  Step s;
  s.name = "PICK_CLOSE";
  s.type = "gripper";
  s.arm = arm_;
  s.target = {pick_.close_target};
  s.duration = pick_.close_duration;
  s.timeout = 12;
  s.gripper_wait_only = true; // Fingers stop on the cup before the target; grasp unverified.
  phase(s.name);
  return execute_step(s);
}

std::vector<std::string> PickCupMotion::finger_links(bool with_base) const {
  const std::string prefix = arm_ == "left" ? "gripper_l_" : "gripper_r_";
  std::vector<std::string> links = {prefix + "rh_p12_rn_r1",
      prefix + "rh_p12_rn_r2",
      prefix + "rh_p12_rn_l1",
      prefix + "rh_p12_rn_l2"};
  if (with_base) {
    links.push_back(prefix + "rh_p12_rn_base");
  }
  return links;
}

bool PickCupMotion::apply_scene() {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  moveit_msgs::msg::CollisionObject cup;
  cup.id = "demo_place_cup";
  cup.header.frame_id = robot_.planning_frame;
  cup.operation = cup.ADD;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape.CYLINDER;
  shape.dimensions = {config_.cup_height, config_.inner_radius + config_.wall_thickness};
  cup.primitives = {shape};
  cup.primitive_poses = {body_};
  scene.world.collision_objects.push_back(cup);
  // Model the table from the measured rim and configured height; never delete external objects.
  moveit_msgs::msg::CollisionObject table;
  table.id = "demo_pick_table";
  table.header.frame_id = robot_.planning_frame;
  table.operation = table.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {.4, .4, .04};
  auto pose = body_;
  pose.position.z -= config_.cup_height / 2 + .02;
  table.primitives = {box};
  table.primitive_poses = {pose};
  scene.world.collision_objects.push_back(table);
  // Restore cup/finger checks left by an unfinished insert.
  scene.allowed_collision_matrix.entry_names = {cup.id};
  for (const auto & link : finger_links(false)) {
    scene.allowed_collision_matrix.entry_names.push_back(link);
  }
  const size_t entries = scene.allowed_collision_matrix.entry_names.size();
  scene.allowed_collision_matrix.entry_values.resize(entries);
  for (auto & row : scene.allowed_collision_matrix.entry_values) {
    row.enabled.resize(entries, false);
  }
  phase("PICK_SCENE");
  scene_pending_ = scene_.apply(scene);
  if (!scene_pending_) {
    fail("SCENE_UNAVAILABLE", "Cannot update cup grasp scene");
  }
  return scene_pending_;
}

bool PickCupMotion::allow_contact() {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  // Model radius exceeds the real cup at grasp height; only this hand's fingers may touch it.
  auto & acm = scene.allowed_collision_matrix;
  acm.entry_names = {"demo_place_cup"};
  for (const auto & link : finger_links(false)) {
    acm.entry_names.push_back(link);
  }
  acm.entry_values.resize(acm.entry_names.size());
  for (size_t i = 0; i < acm.entry_names.size(); ++i) {
    auto & row = acm.entry_values[i].enabled;
    row.resize(acm.entry_names.size());
    for (size_t j = 0; j < row.size(); ++j) {
      row[j] = (i == 0) != (j == 0); // Cup-finger pairs only.
    }
  }
  phase("PICK_CONTACT_SCENE");
  scene_pending_ = scene_.apply(scene);
  if (!scene_pending_) {
    fail("SCENE_UNAVAILABLE", "Cannot allow finger contact with cup");
  }
  return scene_pending_;
}

bool PickCupMotion::attach_cup() {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject held;
  held.link_name = robot_.arms.at(arm_).tcp;
  held.object.id = "demo_place_cup";
  held.object.header.frame_id = robot_.planning_frame;
  held.object.operation = held.object.ADD; // No geometry: MoveIt moves the world cup to the hand.
  held.touch_links = finger_links(true);
  scene.robot_state.attached_collision_objects.push_back(held);
  // The pick table top is flush with the cup bottom; allow that contact for the lift start.
  scene.allowed_collision_matrix.entry_names = {held.object.id, "demo_pick_table"};
  scene.allowed_collision_matrix.entry_values.resize(2);
  scene.allowed_collision_matrix.entry_values[0].enabled = {false, true};
  scene.allowed_collision_matrix.entry_values[1].enabled = {true, false};
  phase("PICK_ATTACH");
  scene_pending_ = scene_.apply(scene);
  if (!scene_pending_) {
    fail("SCENE_UNAVAILABLE", "Cannot attach cup to hand");
  }
  return scene_pending_;
}

void PickCupMotion::prepare_insert() {
  const auto tcp = context_.pose_of(robot_.arms.at(arm_).tcp);
  if (!tcp) {
    fail("TCP_TF_MISSING", "Cannot read measured pregrasp TCP");
    return;
  }
  pregrasp_ = tcp->pose;
  auto rim = body_.position;
  rim.z += config_.cup_height / 2;
  const auto insert = cup_insert_pose(pregrasp_, rim, config_.cup_height, pick_);
  if (!insert) {
    fail("PICK_GEOMETRY_INVALID", "Cannot determine measured pregrasp heading");
    return;
  }
  const auto & a = pregrasp_.position;
  const auto & b = insert->position;
  if (std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z) >
      pick_.stand_off + pick_.insert_depth + .03) {
    fail("PICK_GEOMETRY_INVALID", "Insert distance exceeds stand_off + insert_depth + 3cm");
    return;
  }
  insert_ = *insert;
  allow_contact();
}

bool PickCupMotion::capture_arm_joints() {
  start_joints_.clear();
  const auto snapshot = context_.snapshot();
  for (const auto & name : robot_.arms.at(arm_).joints) {
    const auto joint = snapshot.joints.find(name);
    if (joint == snapshot.joints.end()) {
      fail("ROBOT_NOT_READY", "Missing joint feedback: " + name);
      return false;
    }
    start_joints_[name] = joint->second.position;
  }
  return true;
}

bool PickCupMotion::arm_unmoved() const {
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

void PickCupMotion::clear_above() {
  auto clear = pregrasp_;
  clear.position.z = body_.position.z + config_.cup_height / 2 + pick_.clear_height;
  move("PICK_CLEAR", clear);
}

void PickCupMotion::lift() {
  const auto tcp = context_.pose_of(robot_.arms.at(arm_).tcp);
  if (!tcp) {
    fail("TCP_TF_MISSING", "Cannot compute lift from measured TCP");
    return;
  }
  grasp_tcp_ = tcp->pose;
  auto target = tcp->pose;
  target.position.z += pick_.lift_height;
  linear("PICK_LIFT", target);
}

void PickCupMotion::finish_approach() {
  const auto tcp = context_.pose_of(robot_.arms.at(arm_).tcp);
  if (!tcp || !tcp_yaw(tcp->pose.orientation)) {
    fail("TCP_TF_MISSING", "Cannot save measured pregrasp orientation");
    return;
  }
  pregrasp_ = tcp->pose;
  approach_target_.pose = pregrasp_;
  at_observation_pose_ = true;
  result_.state = "SUCCEEDED";
  result_.sequence_completed = true;
  result_.detail = "Pregrasp reached; stopped before contact. No grasp command issued";
  phase("PICK_APPROACH_READY");
}

void PickCupMotion::tick() {
  if (!running()) {
    return;
  }
  if (phase_ == "PICK_APPROACH") {
    if (execute_) {
      last_pick_.reset(); // A real attempt invalidates the old place target; a preview does not.
    }
    const auto error = pick_.validate(config_.cup_height);
    if (!error.empty()) {
      fail("PICK_CONFIG_INVALID", error);
      return;
    }
    phase("PICK_OBSERVE");
  }
  if (phase_ == "PICK_OBSERVE") {
    std::string reason;
    double position_error = 0;
    if (!observer_.poll(cup_id_, estimated_cup_pose_, position_error, reason)) {
      result_.detail = reason;
      if (elapsed(phase_started_) > config_.observation_timeout) {
        fail("CUP_NOT_FOUND", reason);
      }
      return;
    }
    body_ = estimated_cup_pose_.pose;
    // Fixed heading: the same grasp from the initial pose or right after release.
    const double yaw = pick_.approach_yaw_deg * M_PI / 180;
    auto rim = body_.position;
    rim.z += config_.cup_height / 2;
    pregrasp_ = cup_grasp_pose(rim, config_.cup_height, pick_, yaw);
    pregrasp_.position.x -= pick_.stand_off * std::cos(yaw);
    pregrasp_.position.y -= pick_.stand_off * std::sin(yaw);
    approach_target_.header = estimated_cup_pose_.header;
    approach_target_.pose = pregrasp_;
    if (!execute_) {
      result_.state = "SUCCEEDED";
      result_.sequence_completed = true;
      result_.detail = "Pregrasp preview; BODY assumed cup middle; IK/path not verified; "
                       "insert, close and lift are not previewed";
    } else {
      apply_scene();
    }
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
      fail(r.code, r.detail);
      return;
    }
    if (result_.state == "CANCELLING") {
      result_.state = "CANCELLED";
      result_.code = "CANCELLED";
      return;
    }
    if (phase_ == "PICK_SCENE") {
      open_gripper();
    } else if (phase_ == "PICK_CONTACT_SCENE") {
      linear("PICK_INSERT", insert_);
    } else if (phase_ == "PICK_ATTACH") {
      lift();
    }
    return;
  }
  if (moving_) {
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
      return;
    }
    if (r.state != ExecutionState::Succeeded) {
      // Direct PTP blocked before motion: clear above the pregrasp, then lower straight down.
      if (phase_ == "PICK_PREGRASP" && r.state == ExecutionState::Failed && arm_unmoved()) {
        direct_failure_ = "; direct PTP failed first: " + r.detail;
        clear_above();
        return;
      }
      fail(r.code, phase_ + ": " + r.detail + direct_failure_);
      return;
    }
    result_.last_completed_step = phase_;
    if (phase_ == "PICK_OPEN") {
      if (capture_arm_joints()) {
        move("PICK_PREGRASP", pregrasp_);
      }
    } else if (phase_ == "PICK_CLEAR") {
      linear("PICK_LOWER", pregrasp_);
    } else if (phase_ == "PICK_PREGRASP" || phase_ == "PICK_LOWER") {
      if (pick_.stop_at_pregrasp) {
        finish_approach();
      } else {
        prepare_insert();
      }
    } else if (phase_ == "PICK_INSERT") {
      close_gripper();
    } else if (phase_ == "PICK_CLOSE") {
      attach_cup();
    } else if (phase_ == "PICK_LIFT") {
      record_pick();
      at_observation_pose_ = true;
      result_.state = "SUCCEEDED";
      result_.sequence_completed = true;
      result_.detail = "Cup grasped and lifted; holding is not sensor-verified";
      phase("PICK_COMPLETE");
    }
  }
}
} // namespace demo_1f

#include "demo_1f/motions/common/cup_scene.hpp"
#include "demo_1f/runtime/config.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace demo_1f {
std::string ApproachConfig::validate(const std::string & arm) const {
  if (arm != "left" && arm != "right") {
    return "Arm must be left or right";
  }
  if (!std::isfinite(tcp_roll_limit_deg) || tcp_roll_limit_deg < 0 ||
      (tcp_roll_limit_deg > 0 && (tcp_roll_limit_deg < 5 || tcp_roll_limit_deg > 90))) {
    return "TCP roll limit must be zero (disabled) or 5..90 degrees";
  }
  if (!std::isfinite(tcp_pitch_limit_deg) || tcp_pitch_limit_deg < 0 ||
      (tcp_pitch_limit_deg > 0 && (tcp_pitch_limit_deg < 5 || tcp_pitch_limit_deg > 89))) {
    return "TCP pitch limit must be zero (disabled) or 5..89 degrees";
  }
  if (tcp_pitch_downward_only && tcp_pitch_limit_deg == 0) {
    return "Downward-only pitch requires a nonzero pitch limit";
  }
  if (!std::isfinite(tcp_pitch_min_deg) || tcp_pitch_min_deg < 0 ||
      (tcp_pitch_min_deg > 0 &&
          (!tcp_pitch_downward_only || tcp_pitch_min_deg + 2 >= tcp_pitch_limit_deg))) {
    return "Minimum downward pitch needs downward-only mode and more than 2 degrees below maximum";
  }
  if (!std::isfinite(tcp_yaw_deg) || std::abs(tcp_yaw_deg) > 90 ||
      !std::isfinite(tcp_yaw_limit_deg) || tcp_yaw_limit_deg < 0 ||
      (tcp_yaw_limit_deg > 0 && (tcp_yaw_limit_deg < 5 || tcp_yaw_limit_deg > 90))) {
    return "TCP yaw must be within 90 degrees; its limit zero (disabled) or 5..90 degrees";
  }
  for (double value : {cup_height,
           inner_radius,
           wall_thickness,
           scene_radius,
           max_age,
           observation_timeout,
           max_position_error,
           travel_scale}) {
    if (!std::isfinite(value) || value <= 0) {
      return "Approach dimensions, timing and limits must be positive and finite";
    }
  }
  if (scene_radius > .3 || !std::isfinite(scene_margin) || scene_margin < 0 ||
      scene_margin > .1) {
    return "Cup scene radius must be at most 0.3 m and its margin 0..0.1 m";
  }
  if (cup_source != "head" && cup_source != "wrist_left_init") {
    return "Cup source must be head or wrist_left_init";
  }
  if (cup_height > .5 || inner_radius > .3 || max_position_error > .2 || travel_scale > 1 ||
      offset_xy.size() != 2 ||
      !std::all_of(offset_xy.begin(),
          offset_xy.end(),
          [](double x) {
            return std::isfinite(x);
          }) ||
      std::hypot(offset_xy[0], offset_xy[1]) > .3 || !std::isfinite(offset_z) ||
      std::abs(offset_z) > .3) {
    return "Invalid approach dimensions, offset or limits";
  }
  if (!std::isfinite(forward_offset) || forward_offset < 0 || forward_offset > .06) {
    return "Forward offset must be 0..0.06 m";
  }
  if (fixed_pose && (!std::isfinite(tcp_pitch_deg) || tcp_pitch_deg < tcp_pitch_min_deg ||
                        tcp_pitch_deg > (tcp_pitch_limit_deg > 0 ? tcp_pitch_limit_deg : 89))) {
    return "Fixed-pose pitch must be within tcp_pitch_min_deg..tcp_pitch_limit_deg";
  }
  for (double value : {refine_pitch_step_deg,
           refine_max_correction,
           refine_speed,
           refine_clearance,
           refine_wait,
           refine_view_wait,
           refine_search_timeout,
           refine_pitch_ceiling_deg,
           refine_correction_speed,
           refine_correction_acceleration,
           refine_lower_distance}) {
    if (!std::isfinite(value) || value <= 0) {
      return "Refinement limits must be positive and finite";
    }
  }
  if (refine_view_wait >= refine_wait) {
    return "View retry delay must be shorter than wrist observation timeout";
  }
  if (refine_pitch_step_deg > 5 || refine_pitch_ceiling_deg > 80 ||
      refine_max_correction > .15 || refine_speed > .1 || refine_clearance < .03 ||
      refine_clearance > .15) {
    return "Invalid refinement tilt, correction, speed or clearance limits";
  }
  if (refine_correction_speed > .2 || refine_correction_acceleration > .15 ||
      refine_lower_distance > .03 || refine_clearance - refine_lower_distance < .015 ||
      !std::isfinite(refine_gripper_open) || refine_gripper_open < 0 || refine_gripper_open > .8) {
    return "Invalid refinement release or speed settings";
  }
  if (!std::isfinite(gripper_scale) || gripper_scale <= 0 || gripper_scale > 1) {
    return "Invalid gripper_scale (0 < scale <= 1)";
  }
  return {};
}

ApproachConfig load_approach_config(rclcpp::Node & node) {
  ApproachConfig c;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  auto number = [&](const std::string & key, double & value) {
    value = node.declare_parameter("approach." + key, value, descriptor);
  };
  number("cup_height", c.cup_height);
  number("offset_z", c.offset_z);
  number("inner_radius", c.inner_radius);
  number("wall_thickness", c.wall_thickness);
  number("scene_radius", c.scene_radius);
  number("scene_margin", c.scene_margin);
  number("max_age", c.max_age);
  number("observation_timeout", c.observation_timeout);
  number("max_position_error", c.max_position_error);
  number("travel_scale", c.travel_scale);
  number("tcp_roll_limit_deg", c.tcp_roll_limit_deg);
  number("tcp_pitch_limit_deg", c.tcp_pitch_limit_deg);
  number("tcp_pitch_min_deg", c.tcp_pitch_min_deg);
  number("tcp_yaw_deg", c.tcp_yaw_deg);
  number("tcp_yaw_limit_deg", c.tcp_yaw_limit_deg);
  number("tcp_pitch_deg", c.tcp_pitch_deg);
  number("forward_offset", c.forward_offset);
  c.fixed_pose = node.declare_parameter("approach.fixed_pose", c.fixed_pose, descriptor);
  c.tcp_pitch_downward_only = node.declare_parameter(
      "approach.tcp_pitch_downward_only", c.tcp_pitch_downward_only, descriptor);
  c.tcp_yaw_inward_only =
      node.declare_parameter("approach.tcp_yaw_inward_only", c.tcp_yaw_inward_only, descriptor);
  number("refine_pitch_step_deg", c.refine_pitch_step_deg);
  number("refine_pitch_ceiling_deg", c.refine_pitch_ceiling_deg);
  number("refine_max_correction", c.refine_max_correction);
  number("refine_speed", c.refine_speed);
  number("refine_correction_speed", c.refine_correction_speed);
  number("refine_correction_acceleration", c.refine_correction_acceleration);
  number("refine_lower_distance", c.refine_lower_distance);
  number("refine_gripper_open", c.refine_gripper_open);
  number("gripper_scale", c.gripper_scale);

  number("refine_clearance", c.refine_clearance);
  number("refine_wait", c.refine_wait);
  number("refine_view_wait", c.refine_view_wait);
  number("refine_search_timeout", c.refine_search_timeout);
  c.refine_observe =
      node.declare_parameter("approach.refine_observe", c.refine_observe, descriptor);
  c.offset_xy = node.declare_parameter("approach.offset_xy", c.offset_xy, descriptor);
  c.cup_source = node.declare_parameter("approach.cup_source", c.cup_source, descriptor);
  c.other_arm_return_sequence = node.declare_parameter(
      "approach.other_arm_return_sequence", c.other_arm_return_sequence, descriptor);
  c.place_return_sequence = node.declare_parameter(
      "approach.place_return_sequence", c.place_return_sequence, descriptor);
  c.independent_return =
      node.declare_parameter("demo.independent_return", c.independent_return, descriptor);
  return c;
}

std::string independent_return_error(const Config & robot, const ApproachConfig & config) {
  if (config.other_arm_return_sequence != "right_return") {
    return "approach.other_arm_return_sequence must be right_return";
  }
  for (const auto & name : {config.other_arm_return_sequence, config.place_return_sequence}) {
    if (name.empty()) continue; // Empty place sequence: PLACE_CUP uses right_return.
    const auto found = robot.motion_sequences.find(name);
    if (found == robot.motion_sequences.end() || !found->second.validation_error.empty() ||
        !validate_motion_sequence(found->second, robot).empty() || found->second.steps.empty() ||
        std::any_of(found->second.steps.begin(), found->second.steps.end(), [](const Step & step) {
          return step.type != "joint" || step.arm != "right";
        })) {
      return "Independent return sequence " + name + " needs valid right-arm joint steps";
    }
  }
  return {};
}

std::optional<geometry_msgs::msg::Pose> approach_tcp_target(const geometry_msgs::msg::Pose & body,
    const ApproachConfig & config,
    const std::string & arm,
    double uncertainty,
    std::string & reason) {
  reason = config.validate(arm);
  if (!reason.empty()) {
    return {};
  }
  for (double value : {body.position.x, body.position.y, body.position.z, uncertainty}) {
    if (!std::isfinite(value)) {
      reason = "Non-finite TCP target input";
      return {};
    }
  }
  if (uncertainty < 0 || uncertainty > config.max_position_error) {
    reason = "Excessive cup uncertainty";
    return {};
  }
  auto target = body;
  const double yaw = (arm == "left" ? 1 : -1) * config.tcp_yaw_deg * M_PI / 180;
  target.position.x += config.offset_xy[0] + config.forward_offset * std::cos(yaw);
  target.position.y += config.offset_xy[1] + config.forward_offset * std::sin(yaw);
  target.position.z += config.cup_height + config.offset_z;
  target.orientation = geometry_msgs::msg::Quaternion();
  target.orientation.w = 1; // Position goal placeholder; roll/pitch have separate goal constraints.
  if (config.tcp_yaw_limit_deg > 0) {
    // Heading only: forward is TCP -Z; the right hand mirrors the left.
    tf2::Quaternion q;
    q.setRPY(0, -M_PI / 2, (arm == "left" ? 1 : -1) * config.tcp_yaw_deg * M_PI / 180);
    target.orientation = tf2::toMsg(q);
  }
  reason.clear();
  return target;
}

geometry_msgs::msg::Quaternion approach_fixed_orientation(
    const ApproachConfig & config, const std::string & arm) {
  // Same convention as cup_grasp_pose: forward is TCP -Z, positive pitch tilts it down.
  tf2::Quaternion q;
  q.setRPY(0, -M_PI / 2 + config.tcp_pitch_deg * M_PI / 180,
      (arm == "left" ? 1 : -1) * config.tcp_yaw_deg * M_PI / 180);
  return tf2::toMsg(q);
}
} // namespace demo_1f

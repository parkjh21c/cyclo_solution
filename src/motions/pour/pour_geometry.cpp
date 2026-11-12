#include "demo_1f/motions/pour/pour_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace demo_1f {
std::string PourConfig::validate() const {
  for (double v : {angle_deg, design_angle_deg, hand_yaw_deg, direction_deg, shift, clearance,
           approach_margin, wall_height, lip_radius, axis_behind_tcp, grasp_height,
           hold_duration, lin_speed, ptp_speed, hold_tolerance, inner_length, inner_width,
           inner_margin}) {
    if (!std::isfinite(v)) return "Pour settings must be finite";
  }
  if (angle_deg < 10 || angle_deg > design_angle_deg || design_angle_deg > 170) {
    return "Pour angle must be 10..design_angle_deg, design up to 170 deg";
  }
  double previous = angle_deg;
  for (double a : fallback_angles_deg) {
    if (!std::isfinite(a) || a < 90 || a >= previous) {
      return "Fallback pour angles must decrease from angle_deg and stay at 90 deg or more";
    }
    previous = a;
  }
  if (std::abs(hand_yaw_deg) > 180 || std::abs(shift) > .15 || clearance < 0 || clearance > .2 ||
      approach_margin < 0 || wall_height <= 0 || lip_radius <= 0 || axis_behind_tcp < 0 ||
      grasp_height <= 0 || hold_duration < 0 || hold_duration > 30 || lin_speed <= 0 ||
      lin_speed > .3 || ptp_speed <= 0 || ptp_speed > .5 || hold_tolerance <= 0 ||
      inner_margin < 0 || inner_length <= 2 * inner_margin || inner_width <= 2 * inner_margin) {
    return "Invalid pour geometry, timing or speed";
  }
  return {};
}

PourConfig load_pour_config(rclcpp::Node & node) {
  PourConfig c;
  rcl_interfaces::msg::ParameterDescriptor d;
  d.read_only = true;
  auto number = [&](const char * name, double & value) {
    value = node.declare_parameter(std::string("pour.") + name, value, d);
  };
  number("angle_deg", c.angle_deg);
  c.fallback_angles_deg =
      node.declare_parameter("pour.fallback_angles_deg", c.fallback_angles_deg, d);
  number("design_angle_deg", c.design_angle_deg);
  number("hand_yaw_deg", c.hand_yaw_deg);
  number("direction_deg", c.direction_deg);
  number("shift", c.shift);
  number("clearance", c.clearance);
  number("approach_margin", c.approach_margin);
  number("wall_height", c.wall_height);
  number("lip_radius", c.lip_radius);
  number("axis_behind_tcp", c.axis_behind_tcp);
  number("grasp_height", c.grasp_height);
  number("hold_duration", c.hold_duration);
  number("lin_speed", c.lin_speed);
  number("ptp_speed", c.ptp_speed);
  number("hold_tolerance", c.hold_tolerance);
  number("inner_length", c.inner_length);
  number("inner_width", c.inner_width);
  number("inner_margin", c.inner_margin);
  return c;
}

namespace {
geometry_msgs::msg::Pose to_pose(const tf2::Vector3 & p, const tf2::Quaternion & q) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  pose.orientation = tf2::toMsg(q.normalized());
  return pose;
}
} // namespace

std::optional<PourPlan> plan_pour(const geometry_msgs::msg::Pose & tcp,
    const geometry_msgs::msg::Pose & basket_pose,
    double cup_height,
    const PourConfig & c,
    std::string & reason) {
  const auto & basket = basket_pose.position;
  reason = c.validate();
  if (!reason.empty()) return std::nullopt;
  tf2::Quaternion q0;
  tf2::fromMsg(tcp.orientation, q0);
  if (q0.length2() < .5) {
    reason = "Invalid TCP orientation";
    return std::nullopt;
  }
  q0.normalize();
  const tf2::Vector3 t0(tcp.position.x, tcp.position.y, tcp.position.z);
  // SG2 forward is TCP -Z; the transfer hold keeps it level.
  tf2::Vector3 heading = tf2::quatRotate(q0, tf2::Vector3(0, 0, -1));
  heading.setZ(0);
  if (heading.length() < .9) {
    reason = "TCP heading is not level; the cup is not held upright";
    return std::nullopt;
  }
  heading.normalize();
  const tf2::Vector3 axis = t0 - heading * c.axis_behind_tcp; // Cup axis at TCP height.
  // Turn hand and cup about the upright cup axis; the cup stays upright.
  const tf2::Quaternion yaw(tf2::Vector3(0, 0, 1), c.hand_yaw_deg * M_PI / 180);
  const tf2::Vector3 turned = axis + tf2::quatRotate(yaw, t0 - axis);
  const tf2::Quaternion hand = (yaw * q0).normalized();
  const double direction = c.direction_deg * M_PI / 180;
  const tf2::Vector3 d(std::cos(direction), std::sin(direction), 0);
  const tf2::Vector3 tilt_axis(-d.y(), d.x(), 0); // z x d: the top falls toward d.
  const tf2::Vector3 rim(axis.x(), axis.y(), t0.z() + cup_height - c.grasp_height);
  const tf2::Vector3 lip = rim + d * c.lip_radius;
  const double top = basket.z + c.wall_height;
  // Shifted lip target, kept inside the rim in the basket frame (any basket yaw).
  tf2::Quaternion basket_q;
  tf2::fromMsg(basket_pose.orientation, basket_q);
  tf2::Vector3 long_axis = basket_q.length2() > .5 ? tf2::quatRotate(basket_q.normalized(),
                                                        tf2::Vector3(1, 0, 0)) :
                                                    tf2::Vector3(1, 0, 0);
  long_axis.setZ(0);
  long_axis = long_axis.length() > .5 ? long_axis.normalized() : tf2::Vector3(1, 0, 0);
  const tf2::Vector3 short_axis(-long_axis.y(), long_axis.x(), 0);
  const tf2::Vector3 offset = d * c.shift;
  const double half_l = c.inner_length / 2 - c.inner_margin;
  const double half_w = c.inner_width / 2 - c.inner_margin;
  const double along = offset.dot(long_axis), across = offset.dot(short_axis);
  const double kept_along = std::clamp(along, -half_l, half_l);
  const double kept_across = std::clamp(across, -half_w, half_w);
  const tf2::Vector3 kept = long_axis * kept_along + short_axis * kept_across;
  const tf2::Vector3 goal(basket.x + kept.x(), basket.y + kept.y(), top + c.clearance);
  // LIN tilt keeps the TCP fixed: place it so the lip lands on the goal at the design angle.
  const tf2::Quaternion design(tilt_axis, c.design_angle_deg * M_PI / 180);
  const tf2::Vector3 pour = goal - tf2::quatRotate(design, lip - turned);
  const double bottom = rim.z() - cup_height + (pour.z() - t0.z());
  PourPlan plan;
  plan.lip.x = goal.x();
  plan.lip.y = goal.y();
  plan.lip.z = goal.z();
  plan.clamped = kept_along != along || kept_across != across;
  plan.raise = std::max(0.0, top + c.approach_margin - bottom);
  plan.lowered = to_pose(pour, hand);
  plan.above = to_pose(pour + tf2::Vector3(0, 0, plan.raise), hand);
  const tf2::Quaternion tilt(tilt_axis, c.angle_deg * M_PI / 180);
  plan.tilted = to_pose(pour, tilt * hand);
  for (double a : c.fallback_angles_deg) {
    plan.fallbacks.push_back(to_pose(pour, tf2::Quaternion(tilt_axis, a * M_PI / 180) * hand));
  }
  return plan;
}
} // namespace demo_1f

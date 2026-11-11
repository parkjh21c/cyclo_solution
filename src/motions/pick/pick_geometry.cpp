#include "demo_1f/motions/pick/pick_geometry.hpp"
#include "demo_1f/runtime/geometry/tcp_roll.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace demo_1f {
std::string PickCupConfig::validate(double height) const {
  for (double x : {grasp_height, stand_off, speed}) {
    if (!std::isfinite(x) || x <= 0) {
      return "Pick approach settings must be positive and finite";
    }
  }
  if (grasp_height < .05 || grasp_height > height - .04 || stand_off < .10 || stand_off > .20 ||
      speed > .2 || !std::isfinite(tcp_forward_offset) || std::abs(tcp_forward_offset) > .03) {
    return "Invalid cup pregrasp geometry or speed";
  }
  if (!std::isfinite(insert_speed) || insert_speed <= 0 || insert_speed > .2 ||
      !std::isfinite(lift_height) || lift_height < .02 || lift_height > .15 ||
      !std::isfinite(close_target) || close_target <= 0 || close_target > 1.1 ||
      !std::isfinite(close_duration) || close_duration <= 0 || close_duration > 3 ||
      !std::isfinite(insert_depth) || insert_depth < 0 || insert_depth > .04 ||
      !std::isfinite(clear_height) || clear_height < .02 || clear_height > .15 ||
      !std::isfinite(approach_yaw_deg) || std::abs(approach_yaw_deg) > 60 ||
      !std::isfinite(approach_pitch_deg) || approach_pitch_deg < 0 || approach_pitch_deg > 30) {
    return "Invalid cup insert, close or lift settings";
  }
  if (!std::isfinite(place_clearance) || place_clearance < 0 || place_clearance > .03) {
    return "Place clearance must be 0..0.03 m";
  }
  return {};
}

PickCupConfig load_pick_config(rclcpp::Node & node) {
  PickCupConfig c;
  rcl_interfaces::msg::ParameterDescriptor d;
  d.read_only = true;
  auto number = [&](const char * name, double & value) {
    value = node.declare_parameter(std::string("pick.") + name, value, d);
  };
  number("grasp_height", c.grasp_height);
  number("stand_off", c.stand_off);
  number("speed", c.speed);
  number("tcp_forward_offset", c.tcp_forward_offset);
  number("insert_speed", c.insert_speed);
  number("lift_height", c.lift_height);
  number("insert_depth", c.insert_depth);
  number("clear_height", c.clear_height);
  number("approach_yaw_deg", c.approach_yaw_deg);
  number("approach_pitch_deg", c.approach_pitch_deg);
  number("close_target", c.close_target);
  number("close_duration", c.close_duration);
  number("place_clearance", c.place_clearance);
  c.stop_at_pregrasp = node.declare_parameter("pick.stop_at_pregrasp", c.stop_at_pregrasp, d);
  return c;
}

geometry_msgs::msg::Pose cup_grasp_pose(const geometry_msgs::msg::Point & rim,
    double height,
    const PickCupConfig & config,
    double yaw) {
  // Pads sit on the cup axis at grasp_height; the TCP is tcp_forward_offset along the tilted forward.
  const double pitch = config.approach_pitch_deg * M_PI / 180;
  geometry_msgs::msg::Pose p;
  p.position = rim;
  p.position.x -= config.tcp_forward_offset * std::cos(pitch) * std::cos(yaw);
  p.position.y -= config.tcp_forward_offset * std::cos(pitch) * std::sin(yaw);
  p.position.z += config.grasp_height - height + config.tcp_forward_offset * std::sin(pitch);
  // SG2: forward is local -Z, jaw separation is local Y; local X points up. Pitch tilts forward down.
  tf2::Quaternion q;
  q.setRPY(0, -M_PI / 2 + pitch, yaw);
  p.orientation = tf2::toMsg(q);
  return p;
}

std::optional<geometry_msgs::msg::Pose> cup_insert_pose(const geometry_msgs::msg::Pose & pregrasp,
    const geometry_msgs::msg::Point & rim,
    double height,
    const PickCupConfig & config) {
  const auto yaw = tcp_yaw(pregrasp.orientation);
  if (!yaw) {
    return {};
  }
  // Pads end insert_depth past the cup axis along the arrived heading, at grasp_height.
  auto p = cup_grasp_pose(rim, height, config, *yaw);
  p.position.x += config.insert_depth * std::cos(*yaw);
  p.position.y += config.insert_depth * std::sin(*yaw);
  p.orientation = pregrasp.orientation;
  return p;
}

} // namespace demo_1f

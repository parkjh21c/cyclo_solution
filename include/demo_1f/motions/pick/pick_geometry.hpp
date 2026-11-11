#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace demo_1f {
struct PickCupConfig {
  double grasp_height{.065}, stand_off{.12}, speed{.12};
  double tcp_forward_offset{-.03};
  double insert_speed{.06}, lift_height{.08};
  double insert_depth{.02}; // Extra insert past pads-on-axis; moves the cup toward the palm.
  double clear_height{.05}; // Fallback waypoint height above the rim, over the pregrasp.
  // Fixed hand heading (robot +X = 0, left +) and downward tilt; jaws stay horizontal.
  double approach_yaw_deg{0}, approach_pitch_deg{0};
  double close_target{.8}, close_duration{1.0};
  bool stop_at_pregrasp{false}; // Leave the cup untouched for an external pickup policy.
  std::string validate(double cup_height) const;
};

PickCupConfig load_pick_config(rclcpp::Node & node);
geometry_msgs::msg::Pose cup_grasp_pose(const geometry_msgs::msg::Point & rim,
    double cup_height,
    const PickCupConfig & config,
    double yaw);

// Grasp position from the measured pregrasp heading; keeps the measured hand orientation.
std::optional<geometry_msgs::msg::Pose> cup_insert_pose(const geometry_msgs::msg::Pose & pregrasp,
    const geometry_msgs::msg::Point & rim,
    double cup_height,
    const PickCupConfig & config);

} // namespace demo_1f

#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace demo_1f {
// Pour over the measured basket, from the right-hand transfer hold (demo.yaml pour.*).
struct PourConfig {
  double angle_deg{130};        // Tilt commanded.
  // Smaller tilts tried in order when a tilt fails to plan before moving (wrist joint limits).
  std::vector<double> fallback_angles_deg{115, 100};
  double design_angle_deg{130}; // Tilt at which the lip sits over the target.
  double hand_yaw_deg{-90};     // Hand turn about the upright cup axis before the pour.
  double direction_deg{90};     // Lip direction in base_link; 90 = +y = right to left.
  double shift{-.05};           // Lip target along the direction from the basket center.
  double clearance{.09};        // Lip above the wall top at the design angle; .09 keeps the
                                // whole cup above the walls, so no lowering into the basket.
  double approach_margin{.03};  // Upright cup bottom above the wall top while moving over.
  double wall_height{.08};      // Basket wall top above the measured basket pose.
  double lip_radius{.043};      // Real cup radius at the rim.
  double axis_behind_tcp{.04};  // Cup axis behind the TCP along the hand heading.
  double grasp_height{.12};     // TCP above the cup bottom.
  double hold_duration{2.0};
  double lin_speed{.08}, ptp_speed{.2};
  double hold_tolerance{.02};   // TCP distance from the transfer hold pose allowed at start.
  double inner_length{.295}, inner_width{.21}; // Basket inside at the rim (ffw_grasp task).
  double inner_margin{.03};     // Lip target kept this far inside the rim, any basket yaw.
  std::string validate() const;
};

PourConfig load_pour_config(rclcpp::Node & node);

struct PourPlan {
  geometry_msgs::msg::Pose above, lowered, tilted; // above == lowered when no raise is needed
  std::vector<geometry_msgs::msg::Pose> fallbacks; // Tilted poses for fallback_angles_deg.
  geometry_msgs::msg::Point lip;                   // Lip target at the design angle.
  double raise{0};
  bool clamped{false}; // The shifted lip target was pulled inside the basket rim.
};

// TCP poses for a cup held upright at tcp; basket is the measured outer bottom center with
// x along its long side (wrist_basket_init).
std::optional<PourPlan> plan_pour(const geometry_msgs::msg::Pose & tcp,
    const geometry_msgs::msg::Pose & basket,
    double cup_height,
    const PourConfig & config,
    std::string & reason);
} // namespace demo_1f

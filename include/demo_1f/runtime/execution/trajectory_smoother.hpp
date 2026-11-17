#pragma once

#include "demo_1f/core/types.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <stdexcept>

namespace demo_1f {
// Call before rclcpp::init: use MoveIt's supported node-less logger fallback.
// The deployed Jazzy logger otherwise keeps a ROS node alive until process exit.
void initialize_moveit_logging();

struct SmoothPath {
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<size_t> point_steps;
  std::vector<double> segment_end_times;
};

MotionLimits effective_limits(const Config & config,
    const std::string & name,
    const Step & step,
    const moveit::core::RobotModelConstPtr & model = {});

// Explicit arm corridors are validation constraints; no positions are corrected.
std::map<std::string, double> held_arm_targets(const Step & step, const Config & config);
void validate_held_arm_path(const trajectory_msgs::msg::JointTrajectory & path,
    const Step & step, const Config & config);

// Ruckig runs locally, one target state at a time. No online waypoint service is used.
SmoothPath smooth_segments(const std::vector<moveit_msgs::msg::RobotTrajectory> & segments,
    const std::vector<Step> & steps,
    const Config & config,
    const moveit::core::RobotModelConstPtr & model = {});

// A sampled smoothed-path state collides with the scene; other validation errors stay runtime_error.
struct PathCollisionError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// Validate the controller's quintic interpolation, not just the supplied knots.
void validate_smooth_path(const SmoothPath & path,
    const std::vector<Step> & steps,
    const Config & config,
    planning_scene::PlanningScene & scene);

using JointRows = std::vector<std::vector<double>>;

// Interpolated path positions; consecutive rows differ by at most max_step per joint.
JointRows sample_joint_path(const trajectory_msgs::msg::JointTrajectory & path, double max_step);

// Every first row with every second row (other joints as in scene); "" when collision-free.
std::string check_path_pairs(planning_scene::PlanningScene & scene,
    const Config & config,
    const std::vector<std::string> & first_names,
    const JointRows & first,
    const std::vector<std::string> & second_names,
    const JointRows & second,
    size_t & pairs);
} // namespace demo_1f

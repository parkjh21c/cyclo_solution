#pragma once
#include "demo_1f/core/types.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace demo_1f {
struct RoundTripCollisionChecks {
  size_t samples{0};
};

// Preserve Pilz timing; check quintic positions and collisions without rechecking dynamics.
RoundTripCollisionChecks validate_round_trip_collision(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::vector<std::string> & controlled, const Config & config,
    const planning_scene::PlanningScene & scene);
} // namespace demo_1f

#pragma once
#include "demo_1f/core/head_command_port.hpp"
#include "demo_1f/runtime/state/robot_context.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <cstdint>
#include <rclcpp_action/rclcpp_action.hpp>

namespace demo_1f {
// Sends look_* poses straight to the head controller, outside move_group and TrajectoryExecutor.
class HeadCommander : public HeadCommandPort {
public:
  HeadCommander(rclcpp::Node & node, const Config & config, RobotContext & context);
  bool command(const std::string & name) override;
  void cancel() override;

private:
  using Action = control_msgs::action::FollowJointTrajectory;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  double duration_to(const Step & step) const;
  rclcpp::Node & node_;
  const Config & config_;
  RobotContext & context_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::string action_name_;
  double max_velocity_{0.5}, min_duration_{1.0}, fallback_duration_{8.0};
  Handle::SharedPtr handle_;      // Accepted latest goal; reset once cancel is requested.
  std::uint64_t latest_{0};       // Sequence of the last goal sent.
  std::uint64_t cancelled_{0};    // Latest sequence this commander asked to cancel.
  bool cancel_latest_{false};     // Cancel the latest goal once its acceptance arrives.
};
} // namespace demo_1f

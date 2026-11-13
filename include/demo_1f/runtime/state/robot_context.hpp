#pragma once
#include "demo_1f/core/types.hpp"
#include "demo_1f/msg/cup_observation.hpp"
#include "demo_1f/runtime/state/head_cup_tracker.hpp"
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace demo_1f {
// Shared data and coordinate queries. It never issues a robot command.
class RobotContext {
public:
  RobotContext(rclcpp::Node & node, const Config & config, HeadCupTrackConfig head_tracking = {});
  Snapshot snapshot() const;
  bool observe_cup(const msg::CupObservation & observation, TimePoint received = Clock::now());
  std::optional<CupState> cup_observation(const std::string & id, const std::string & source) const;
  std::optional<CupState> tracked_head_cup(
      const std::string & id, double maximum_age, std::string & reason) const;
  bool cup_acquired_after(const CupState & state, TimePoint after) const;
  void observe_joints(const sensor_msgs::msg::JointState & message);
  bool joint_fresh(const JointObservation & joint) const;
  bool is_gripper_joint(const std::string & name) const;
  bool joint_ready_for_motion(const std::string & name,
      const JointObservation & joint, std::string & reason,
      bool normal_completion = false) const;
  bool joint_stationary(const std::string & name,
      const JointObservation & joint,
      std::string & reason,
      TimePoint after = TimePoint{},
      bool normal_completion = false) const;
  bool ready(std::string & reason) const;
  std::optional<std::string> capture_targets(std::string & reason) const;
  std::optional<geometry_msgs::msg::PoseStamped> pose_of(const std::string & frame) const;
  // Latest /demo/basket_pose (wrist_basket_init): outer bottom center, base frame.
  void observe_basket(const geometry_msgs::msg::PoseStamped & pose);
  std::optional<geometry_msgs::msg::PoseStamped> basket() const;
  std::optional<geometry_msgs::msg::PoseStamped> transform_pose(
      const geometry_msgs::msg::PoseStamped & input) const;
  void set_task(const std::string & state,
      const std::string & direction,
      const std::string & code = {},
      const std::string & detail = {});
  void set_phase(const std::string & phase, const std::string & completed);
  void set_execution(const ExecutionResult & result);
  // One main loop pass (spin_some + sleep) in seconds, for the periodic "tf age" log.
  void note_loop(double seconds);

private:
  void queue_cup(msg::CupObservation::ConstSharedPtr observation);
  void process_pending_cups();
  void sample_tf_age();

  // Latest TCP transform age per arm and main loop passes since started (logged every 5 s).
  struct TfAgeWindow {
    std::map<std::string, double> max_age;
    int samples{0};
    int missing{0};
    double loop_max{0};
    double loop_sum{0};
    int loops{0};
    TimePoint started{Clock::now()};
  };
  std::mutex tf_window_mutex_;
  TfAgeWindow tf_window_;
  rclcpp::TimerBase::SharedPtr tf_age_timer_;

  struct PendingCup {
    msg::CupObservation::ConstSharedPtr observation;
    TimePoint received;
  };

  // Subscription and timer share the node's mutually exclusive callback group.
  std::deque<PendingCup> pending_cups_;
  rclcpp::TimerBase::SharedPtr cup_tf_timer_;
  rclcpp::Node & node_;
  const Config & config_;
  mutable std::mutex mutex_;
  Snapshot state_;
  std::map<std::string, CupState> cups_;
  HeadCupTracker head_cup_tracker_;
  rclcpp::Subscription<msg::CupObservation>::SharedPtr cup_subscription_, wrist_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr basket_subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> basket_;

  struct PositionSample {
    double position;
    int64_t stamp_ns;
    TimePoint received;
  };

  std::map<std::string, std::deque<PositionSample>> gripper_history_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
} // namespace demo_1f

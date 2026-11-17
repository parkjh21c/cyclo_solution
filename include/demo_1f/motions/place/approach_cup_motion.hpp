#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/head_cup_observer.hpp"
#include <map>

namespace demo_1f {
// Head-camera BODY approach. Never opens the gripper.
class ApproachCupMotion final : public CupMotion {
public:
  using CupMotion::CupMotion;
  void tick() override;

private:
  const char * initial_phase() const override {
    return "FIND_CUP";
  }

  void reset() override {
    other_steps_.clear();
    other_next_ = 0;
    paired_ = sequential_ = fixed_ = false;
    start_joints_.clear();
    method_.clear();
    fixed_failure_.clear();
  }

  bool begin_approach();
  bool fixed_mode() const; // Fixed PTP first: independent-return path only.
  Step fixed_step(const geometry_msgs::msg::Pose & pose, double speed) const;
  bool arm_unmoved() const; // Measured arm joints still at the fixed step's start.
  Step approach_step(const geometry_msgs::msg::Pose & pose, double speed) const;
  bool dispatch(const geometry_msgs::msg::Pose & pose, double speed);
  std::vector<Step> other_arm_steps() const;
  bool next_sequential();
  HeadCupObserver observer_{context_, config_};
  // other_arm_return_sequence steps: paired, or one by one before the approach
  std::vector<Step> other_steps_;
  size_t other_next_{0};
  bool paired_{false}, sequential_{false};
  bool fixed_{false};                         // The fixed-pose PTP step is in flight.
  std::map<std::string, double> start_joints_; // Arm joints when the fixed step was sent.
  geometry_msgs::msg::Pose range_target_;      // OMPL position-only goal (fallback).
  std::string method_, fixed_failure_;         // Success log; first failure for the fallback.
};
} // namespace demo_1f

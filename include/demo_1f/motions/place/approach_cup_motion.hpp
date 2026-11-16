#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/head_cup_observer.hpp"

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
    paired_ = sequential_ = false;
  }

  bool begin_approach();
  Step approach_step(const geometry_msgs::msg::Pose & pose, double speed) const;
  bool dispatch(const geometry_msgs::msg::Pose & pose, double speed);
  std::vector<Step> other_arm_steps() const;
  bool next_sequential();
  HeadCupObserver observer_{context_, config_};
  // other_arm_return_sequence steps: paired, or one by one before the approach
  std::vector<Step> other_steps_;
  size_t other_next_{0};
  bool paired_{false}, sequential_{false};
};
} // namespace demo_1f

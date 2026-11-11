#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/pick/pick_cup_motion.hpp"

namespace demo_1f {
// Puts the held cup back where the last pick took it, by reversing that pick:
// PTP to the recorded lifted joints -> LIN down to the grasp + place_clearance -> open ->
// detach the cup model into the world -> LIN back to the recorded pregrasp. Preview never moves.
class PlaceCupMotion final : public CupMotion {
public:
  PlaceCupMotion(const Config & robot,
      const ApproachConfig & config,
      const PickCupConfig & pick,
      RobotContext & context,
      ExecutorPort & executor,
      ApproachScenePort & scene,
      const PickCupMotion & picker);
  bool is_ready_to_start(const std::string & arm,
      const std::string & cup_id,
      std::string & reason,
      bool execute = true) const override;
  bool start(const std::string & arm,
      const std::string & cup_id,
      std::string & reason,
      bool execute = true) override;
  void tick() override;

private:
  const char * initial_phase() const override {
    return "PLACE_CARRY";
  }

  void reset() override;
  bool carry();
  bool linear(const std::string & phase, const geometry_msgs::msg::Pose & pose);
  bool open_gripper();
  bool detach();
  const PickCupConfig & pick_;
  const PickCupMotion & picker_;
  PickRecord record_;
  geometry_msgs::msg::Pose release_; // Recorded grasp TCP raised by place_clearance.
  bool started_{false};
};
} // namespace demo_1f

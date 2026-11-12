#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/sequence_motion.hpp"
#include "demo_1f/motions/pour/pour_geometry.hpp"

namespace demo_1f {
// Held cup is poured over the latest measured basket (/demo/basket_pose): PTP over the basket
// with the hand turned about the cup axis (LIN lower if needed), LIN tilt about the TCP, wait,
// untilt (LIN raise), PTP back to the start joints. Targets are computed at start from the
// right TCP at the transfer hold. A tilt that fails before the arm moved is retried at the
// smaller pour.fallback_angles_deg. The cup model stays attached; cancel stops in place.
// Preview never moves.
class CupPourMotion final : public CupMotion {
public:
  CupPourMotion(const Config & robot,
      const ApproachConfig & config,
      const PourConfig & pour,
      RobotContext & context,
      ExecutorPort & executor,
      ApproachScenePort & scene);
  bool is_ready_to_start(const std::string & arm,
      const std::string & cup_id,
      std::string & reason,
      bool execute = true) const override;
  bool start(const std::string & arm,
      const std::string & cup_id,
      std::string & reason,
      bool execute = true) override;
  void tick() override;
  void cancel() override;

private:
  const char * initial_phase() const override {
    return "CUP_POUR";
  }

  bool supports_arm(const std::string & arm) const override {
    return arm == "right";
  }

  struct Parts {
    MotionSequence before, after;  // Over the basket; hold, untilt and return.
    std::vector<MotionSequence> tilts; // pour_tilt at angle_deg, then each fallback angle.
    std::vector<double> angles;
  };

  void reset() override;
  std::optional<Parts> build(const std::string & arm, std::string & reason) const;
  bool run(const MotionSequence & part);
  bool arm_unmoved() const;
  void finish_cancelled();
  const PourConfig & pour_;
  SequenceMotion sequence_;
  Parts parts_;
  enum class Stage { Idle, Before, Tilt, After } stage_{Stage::Idle};
  size_t tilt_{0};
  std::vector<double> tilt_start_joints_;
  std::string retries_;
};
} // namespace demo_1f

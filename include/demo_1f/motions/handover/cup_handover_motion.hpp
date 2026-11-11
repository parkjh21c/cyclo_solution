#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/sequence_motion.hpp"

namespace demo_1f {
// Held cup moves between hands with taught YAML sequences cup_<giver>_to_<receiver> (transfer),
// *_grasp (insert and close) and *_release. Receiver contact is allowed only before the grasp;
// MoveIt's attached cup moves to the receiver before the release. Preview never moves.
class CupHandoverMotion final : public CupMotion {
public:
  CupHandoverMotion(const Config & robot,
      const ApproachConfig & config,
      RobotContext & context,
      ExecutorPort & executor,
      ApproachScenePort & scene,
      std::string giver = "left");
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
    return "CUP_HANDOVER_TRANSFER";
  }

  bool supports_arm(const std::string & arm) const override {
    return arm == giver_;
  }

  void reset() override;
  const MotionSequence * sequence(const std::string & name, std::string & reason) const;
  bool start_sequence(const std::string & name);
  void finish_sequence(const std::string & next);
  bool apply_swap_scene();
  void finish_cancelled();
  const std::string giver_, receiver_;
  SequenceMotion sequence_;
  std::string running_sequence_;
  bool cancel_requested_{false};
};
} // namespace demo_1f

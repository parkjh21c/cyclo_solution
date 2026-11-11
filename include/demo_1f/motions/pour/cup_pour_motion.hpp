#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/sequence_motion.hpp"

namespace demo_1f {
// Held cup is poured over the basket with the taught YAML sequence cup_pour_<arm>.
// The scene is not changed; the cup stays attached. Preview never moves.
class CupPourMotion final : public CupMotion {
public:
  CupPourMotion(const Config & robot,
      const ApproachConfig & config,
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

  void reset() override;
  const MotionSequence * sequence(const std::string & arm, std::string & reason) const;
  void finish_cancelled();
  SequenceMotion sequence_;
  bool started_{false};
};
} // namespace demo_1f

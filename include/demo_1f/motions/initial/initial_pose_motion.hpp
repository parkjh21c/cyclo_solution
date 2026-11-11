#pragma once
#include "demo_1f/motions/common/cup_scene.hpp"
#include "demo_1f/motions/common/sequence_motion.hpp"

namespace demo_1f {
// Initial pose sequence. With a scene port, a held cup model is released before the grippers open.
class InitialPoseMotion : public SequenceMotion {
public:
  InitialPoseMotion(const Config & config, RobotContext & context, ExecutorPort & executor,
      ApproachScenePort * scene = nullptr)
      : SequenceMotion(config, context, executor), context_(context), scene_(scene) {}
  bool is_ready_to_start(std::string & reason) const;
  bool start(std::string & reason);
  void tick();
  void cancel();
  bool running() const { return releasing_ || SequenceMotion::running(); }
  const MotionResult & result() const {
    return release_result_.state == "IDLE" ? SequenceMotion::result() : release_result_;
  }

private:
  RobotContext & context_;
  ApproachScenePort * scene_;
  bool releasing_{false}, cancel_requested_{false};
  MotionResult release_result_; // IDLE unless the release step is running or ended the motion.
};
} // namespace demo_1f

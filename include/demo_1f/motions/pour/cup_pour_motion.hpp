#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/sequence_motion.hpp"
#include "demo_1f/motions/pour/pour_geometry.hpp"

namespace demo_1f {
// Pour over the measured basket using an executor-owned tilt/hold/reversed-path round trip.
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
  void request_accepted(TimePoint time) override {
    accepted_at_ = time;
  }

  std::optional<double> total_duration() const {
    return total_seconds_;
  }

private:
  const char * initial_phase() const override {
    return "CUP_POUR";
  }

  bool supports_arm(const std::string & arm) const override {
    return arm == "right";
  }

  struct Parts {
    MotionSequence before, after; // Approach; optional raise and final return.
    std::vector<MotionSequence> tilts; // pour_tilt at angle_deg, then each fallback angle.
    std::vector<double> angles;
    Step hold, untilt;
    geometry_msgs::msg::PoseStamped basket; // Basket value the targets were built from.
  };

  void reset() override;
  std::optional<Parts> build(const std::string & arm, std::string & reason) const;
  bool run(const MotionSequence & part);
  bool run_round_trip();
  bool arm_unmoved() const;
  void finish_cancelled();
  void tick_pour();
  void finish_timing();
  const PourConfig & pour_;
  SequenceMotion sequence_;
  Parts parts_;
  enum class Stage { Idle, Before, Tilt, After } stage_{Stage::Idle};
  size_t tilt_{0};
  std::vector<double> tilt_start_joints_;
  std::string retries_;
  std::optional<TimePoint> accepted_at_, started_;
  std::optional<double> total_seconds_;
};
} // namespace demo_1f

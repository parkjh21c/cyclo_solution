#pragma once
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/common/head_cup_observer.hpp"
#include "demo_1f/motions/pick/pick_geometry.hpp"

namespace demo_1f {
// What the last successful pick did; PlaceCupMotion puts the cup back by reversing it.
struct PickRecord {
  std::string arm;
  geometry_msgs::msg::Pose pregrasp, grasp; // Measured TCP before the insert and when closed.
  std::vector<double> lifted_joints;         // Arm joints after the lift, in config order.
};

// Side grasp: PTP pregrasp (or clear + lower) -> insert -> close -> attach -> lift.
// Preview never moves.
class PickCupMotion final : public CupMotion {
public:
  PickCupMotion(const Config & robot,
      const ApproachConfig & config,
      const PickCupConfig & pick,
      RobotContext & context,
      ExecutorPort & executor,
      ApproachScenePort & scene);

  void tick() override;

  const std::optional<PickRecord> & last_pick() const {
    return last_pick_;
  }

private:
  const char * initial_phase() const override {
    return "PICK_APPROACH";
  }

  void reset() override;
  bool move(const std::string & phase, const geometry_msgs::msg::Pose & pose);
  bool linear(const std::string & phase, const geometry_msgs::msg::Pose & pose);
  bool open_gripper();
  bool close_gripper();
  bool apply_scene();
  bool allow_contact();
  bool attach_cup();
  void prepare_insert();
  void lift();
  void clear_above();
  bool capture_arm_joints();
  bool arm_unmoved() const;
  std::vector<std::string> finger_links(bool with_base) const;
  HeadCupObserver observer_;
  const PickCupConfig & pick_;
  void finish_approach();
  geometry_msgs::msg::Pose pregrasp_, body_, insert_;
  std::map<std::string, double> start_joints_; // Arm joints before the direct PTP.
  std::string direct_failure_;                  // Kept for the fallback failure message.
  geometry_msgs::msg::Pose grasp_tcp_;          // Measured TCP when the lift starts.
  std::optional<PickRecord> last_pick_;
  void record_pick();
};
} // namespace demo_1f

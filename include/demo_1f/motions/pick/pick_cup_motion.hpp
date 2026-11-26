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
// continuous_insert, from the initial pose: initial -> pregrasp -> insert as one smoothed joint
// path; the separate steps when not at the initial pose or when rejected before motion.
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
  Step pose_step(const std::string & name, const geometry_msgs::msg::Pose & pose) const;
  Step linear_step(const std::string & name, const geometry_msgs::msg::Pose & pose) const;
  bool move(const std::string & phase, const geometry_msgs::msg::Pose & pose);
  bool linear(const std::string & phase, const geometry_msgs::msg::Pose & pose);
  void prepare_continuous();
  void start_continuous();
  void step_by_step(const std::string & why); // Fallback before motion; why joins the failure.
  bool open_gripper();
  bool close_gripper();
  bool apply_scene(const std::string & name = "PICK_SCENE");
  bool allow_contact(const std::string & name = "PICK_CONTACT_SCENE");
  bool attach_cup();
  void prepare_insert();
  void lift();
  void lift_by_line(const std::string & why); // LIN lift after the joint line was rejected.
  void clear_above();
  bool capture_arm_joints();
  bool arm_unmoved() const;
  std::vector<std::string> finger_links(bool with_base) const;
  HeadCupObserver observer_;
  const PickCupConfig & pick_;
  void finish_approach();
  geometry_msgs::msg::Pose pregrasp_, body_, insert_;
  std::map<std::string, double> start_joints_; // Arm joints before a move with a fallback.
  std::string direct_failure_;                  // Kept for the fallback failure message.
  geometry_msgs::msg::Pose grasp_tcp_;          // Measured TCP when the lift starts.
  geometry_msgs::msg::Pose lift_target_;        // Lift end; the LIN fallback reuses it.
  bool lift_line_{false};                       // Lift running as a checked joint line.
  std::optional<PickRecord> last_pick_;
  void record_pick();
};
} // namespace demo_1f

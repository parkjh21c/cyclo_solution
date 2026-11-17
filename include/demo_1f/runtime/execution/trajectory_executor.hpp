#pragma once
#include "demo_1f/core/executor_port.hpp"
#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include "demo_1f/runtime/state/robot_context.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <functional>
#include <future>
#include <optional>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace demo_1f {
class TrajectoryExecutor final : public ExecutorPort {
public:
  TrajectoryExecutor(rclcpp::Node & node, const Config & config, RobotContext & context);
  bool ready(const MotionSequence & sequence, std::string & reason) const override;
  bool start(const Step & step) override;
  bool start_sequence(const std::vector<Step> & steps) override;
  bool start_paired(const Step & pose, const std::vector<Step> & other_arm) override;
  bool start_round_trip(const Step & tilt, const Step & hold, const Step & untilt) override;
  void tick() override;
  void cancel(
      const std::string & code = "CANCELLED", const std::string & reason = "Cancelled") override;
  ExecutionResult result() const override;
  // Side: one arm's joint steps sent straight to its controller beside the main operation.
  bool start_side(const std::vector<Step> & steps) override;
  void tick_side() override;
  void cancel_side(
      const std::string & code = "CANCELLED", const std::string & reason = "Cancelled") override;
  ExecutionResult side_result() const override;

private:
  using Move = moveit_msgs::action::MoveGroup;
  using Joint = control_msgs::action::FollowJointTrajectory;
  using Execute = moveit_msgs::action::ExecuteTrajectory;

  enum class RoundPhase { Planning, Validating, Tilt, Hold, ReturnValidating, Untilt };
  enum class RoundStateIssue { None, Feedback, Position, JointState, WheelVelocity };
  struct RoundTrip {
    Step tilt, hold, untilt;
    RoundPhase phase{RoundPhase::Planning};
    TimePoint started{Clock::now()}, phase_started{Clock::now()};
    double timeout{0};
    bool execution_requested{false};
    bool retryable{false};
    std::string last_completed;
    moveit_msgs::msg::RobotTrajectory forward, backward;
    std::optional<TimePoint> execution_result_at;
    std::optional<TimePoint> wheel_settle_started, wheel_stable_since;
    // Largest wheel |velocity| and |position - initial| seen during one tilt/untilt execution.
    double wheel_peak_velocity{0}, wheel_peak_offset{0};
    std::string wheel_peak_velocity_joint, wheel_peak_offset_joint;
    bool wheel_peaks_pending{false};
    bool total_logged{false};
    std::map<std::string, double> initial_positions, scene_positions;
  };

  struct PairOutcome {
    std::string collision; // Empty: every pair is collision-free.
    size_t pairs{0};
    double seconds{0};
  };

  // A main path held until it is checked against the running side path.
  struct MainGate {
    trajectory_msgs::msg::JointTrajectory path;
    std::function<void()> send;
    TimePoint started{Clock::now()};
    bool scene_requested{false};
    bool waiting_side{false}; // Pairs collide: wait for the side to end, then replan.
    std::future<PairOutcome> check;
  };

  struct SidePlan {
    SmoothPath path;
    PairOutcome pairs;
  };

  struct SideOperation {
    std::vector<Step> steps;
    std::string arm;
    double timeout{0};
    ExecutionResult result{ExecutionState::Running, {}, {}};
    TimePoint started{Clock::now()}, cancelling{}, sent_at{};
    bool sent{false};        // A goal reached the controller client: the arm may move.
    bool rejected{false};    // The controller rejected the goal: nothing moved.
    bool action_terminal{false}, action_success{false};
    bool cancel_requested{false}, main_notified{false};
    std::string cancel_code, cancel_reason;
    std::optional<TimePoint> stable_since;
    std::future<SidePlan> preparation;
    TimePoint preparation_started{};
    std::map<std::string, double> preparation_positions;
    SmoothPath path;
    bool path_ready{false};
    std::function<void()> cancel_action;
    size_t generation{0};
  };

  struct Operation {
    Step step;
    ExecutionResult result{ExecutionState::Running, {}, {}};
    TimePoint started{Clock::now()};
    TimePoint cancelling;
    std::optional<TimePoint> gripper_sent;
    std::optional<TimePoint> stable_since;
    std::function<void()> cancel_action;
    bool planning_only{false};
    bool action_terminal{false};
    bool action_success{false};
    bool cancel_requested{false};
    bool topic_command{false};
    bool hold_sent{false};
    std::string cancel_code;
    std::string cancel_reason;
    std::string completion_wait_reason;
    geometry_msgs::msg::PoseStamped pose_target;
    std::map<std::string, double> joint_targets;
    std::vector<Step> steps;
    std::vector<moveit_msgs::msg::RobotTrajectory> planned_segments;
    moveit_msgs::msg::RobotState planned_start;
    planning_scene::PlanningScenePtr planning_scene;
    SmoothPath path;
    size_t progress_point{0};
    bool executing_path{false};
    std::future<SmoothPath> preparation;
    TimePoint preparation_started;
    std::map<std::string, double> preparation_positions;
    // Planner fallback of a joint step (Step::fallback_*): attempts sent and their outcomes.
    std::string planner_label;                    // pipeline/planner of the request in flight
    int fallback_tries{0};                        // fallback plans sent so far
    std::vector<std::string> plan_attempts;       // "pipeline/planner: MoveIt error N" per failure
    std::map<std::string, double> fallback_start; // arm joints when the step was first sent
    // Joint-line bundle replanned with OMPL after its smoothed path collided (once per operation).
    bool line_replan{false};
    std::string line_rejection; // Collision that rejected the joint-line path.
    // start_paired: the other arm's joint steps joined to the planned pose path before smoothing.
    std::vector<Step> paired_steps;
    std::string paired_log; // "paired approach: left N pts, right M pts"
    std::optional<RoundTrip> round_trip;
    size_t action_generation{0};
    bool side_linked{false};   // Ran while a side operation was live.
    bool side_notified{false}; // Its failure already cancelled the side.
    std::optional<MainGate> gate;
    trajectory_msgs::msg::JointTrajectory dispatched_path; // Seen by a later side pair check.
  };

  void round_trip_scene(const std::shared_ptr<Operation> & op,
      const moveit_msgs::msg::PlanningScene & message, bool prepare_execution);
  void accept_round_trip_plan(const std::shared_ptr<Operation> & op, const Move::Result & result);
  void tick_round_trip(const std::shared_ptr<Operation> & op);
  bool round_trip_timed_out(const std::shared_ptr<Operation> & op);
  void send_round_trip_path(const std::shared_ptr<Operation> & op, SmoothPath path);
  bool round_trip_start_matches(const Operation & op, bool check_active,
      bool returning, std::string & reason, RoundStateIssue * issue = nullptr) const;
  void fail_round_trip(const std::shared_ptr<Operation> & op, const std::string & code,
      const std::string & detail, bool retryable = false);
  void sample_wheel_peaks(Operation & op) const;
  void log_wheel_peaks(Operation & op) const;

  void send_move(const std::shared_ptr<Operation> & op, bool plan_only = false);
  // True: replanned with the step's fallback planner; false: failure stands (history in detail).
  bool replan_with_fallback(
      const std::shared_ptr<Operation> & op, const Move::Result & result, std::string & detail);
  // Plan-only sequence segment: held-arm check, store, advance the start, then next or prepare.
  void accept_segment(const std::shared_ptr<Operation> & op,
      const moveit_msgs::msg::RobotTrajectory & planned);
  // True: the joint-line bundle is being planned again with OMPL; false: the failure stands.
  bool retry_line_bundle(const std::shared_ptr<Operation> & op, const std::string & collision);
  void execute_planned_pose(const std::shared_ptr<Operation> & op,
      const moveit_msgs::msg::RobotTrajectory & trajectory);
  void request_scene(const std::shared_ptr<Operation> & op, bool prepare_execution);
  // Planned pose path + the other arm's straight joint path -> one two-arm segment and step.
  void pair_with_other_arm(Operation & op);
  void prepare_path(
      const std::shared_ptr<Operation> & op, const moveit_msgs::msg::PlanningScene & scene);
  void update_path_phase(const std::shared_ptr<Operation> & op);
  void load_robot_model();
  std::string joint_goal_diagnostic(const Operation & op, const std::string & group) const;
  bool joint_group_ready(const Step & step, std::string & reason) const;
  void send_gripper(const std::shared_ptr<Operation> & op);
  template <class Action>
  void dispatch(const typename rclcpp_action::Client<Action>::SharedPtr & client,
      const typename Action::Goal & goal,
      const std::shared_ptr<Operation> & op,
      std::function<std::string(const typename Action::Result &)> error,
      std::function<void(const typename Action::Result &)> on_success = {});
  bool settled(const std::shared_ptr<Operation> & op, bool check_target);
  double goal_tolerance(const std::string & joint, const Step & step) const;
  void publish_hold(const std::shared_ptr<Operation> & op);
  void publish_state();
  void set_operation(const std::shared_ptr<Operation> & op);
  void cancel_main(const std::string & code, const std::string & reason);
  void begin_main(const std::shared_ptr<Operation> & op);
  void restart_main(const std::shared_ptr<Operation> & op);
  // Sends now, or holds the path until it is pair-checked against the live side path.
  void gate_main(const std::shared_ptr<Operation> & op,
      const trajectory_msgs::msg::JointTrajectory & path, std::function<void()> send);
  void tick_gate(const std::shared_ptr<Operation> & op);
  bool side_live() const;
  bool side_blocks(const Step & step, std::string & reason) const;
  bool refuse_for_side(const Step & step); // True: a SIDE_ARM_BUSY operation was recorded.
  void prepare_side(const std::shared_ptr<SideOperation> & side,
      const moveit_msgs::msg::PlanningScene & scene);
  void send_side(const std::shared_ptr<SideOperation> & side);
  bool settled_side(const std::shared_ptr<SideOperation> & side, bool check_target);
  void couple_side();
  rclcpp::Node & node_;
  const Config & config_;
  RobotContext & context_;
  rclcpp_action::Client<Move>::SharedPtr move_client_;
  rclcpp_action::Client<Execute>::SharedPtr execute_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr scene_template_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr planned_trajectory_publisher_;
  std::map<std::string, rclcpp_action::Client<Joint>::SharedPtr> joint_clients_;
  std::map<std::string, rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
      publishers_;
  std::shared_ptr<Operation> operation_;
  std::vector<std::shared_ptr<Operation>> retired_operations_;
  std::shared_ptr<SideOperation> side_;
  std::vector<std::shared_ptr<SideOperation>> retired_sides_;
};
} // namespace demo_1f

#pragma once
#include "demo_1f/action/approach_cup.hpp"
#include "demo_1f/action/handover.hpp"
#include "demo_1f/action/run_demo.hpp"
#include <ffw_vision_msgs/action/pick_dynamixel.hpp>
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/handover/handover_motion.hpp"
#include "demo_1f/motions/initial/initial_pose_motion.hpp"
#include "demo_1f/msg/demo_status.hpp"
#include <map>
#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace demo_1f {
// Modules are owned by the composition root and must outlive TaskManager.
struct CupMotionRegistration {
  std::string action_name;
  CupMotion & motion;
};

class TaskManager {
public:
  TaskManager(rclcpp::Node & node,
      RobotContext & context,
      HandoverMotion & motion,
      std::vector<CupMotionRegistration> cup_motions = {}, InitialPoseMotion * initial_motion = nullptr);
  void request_shutdown();
  bool can_shutdown() const;
  bool demo_faulted() const { return demo_faulted_; }

private:
  using DemoAction = demo_1f::action::RunDemo;
  using DemoHandle = rclcpp_action::ServerGoalHandle<DemoAction>;
  using PickAction = ffw_vision_msgs::action::PickDynamixel;
  using PickHandle = rclcpp_action::ClientGoalHandle<PickAction>;
  void setup_demo_server();
  void tick_demo();
  void start_demo_stage();
  void cancel_demo_motion();
  void finish_demo(bool success, const std::string & code, const std::string & detail,
      bool resumable = false);
  void finish_demo_now(bool success, const std::string & code, const std::string & detail,
      bool resumable);
  std::string send_pick_goal(bool hold); // '' when sent, else why (demo faulted).
  void cancel_pick();
  void prepare_pick();
  void tick_pick_release();
  void fail_demo_stage(const std::string & code, const std::string & detail, bool retryable);
  CupMotion * demo_cup_module(const std::string & action);
  bool demo_busy() const;
  rclcpp_action::Server<DemoAction>::SharedPtr demo_server_;
  rclcpp_action::Client<PickAction>::SharedPtr pick_client_;
  std::shared_ptr<DemoHandle> demo_goal_;
  PickHandle::SharedPtr pick_handle_;
  PickAction::Result::SharedPtr pick_result_;
  bool pick_pending_{false}, pick_cancel_sent_{false}, demo_stage_active_{false};
  bool demo_retry_wait_{false}, demo_resumable_{false}, demo_pick_only_{false};
  bool demo_initial_only_{false};
  bool demo_withdraw_cup_{false}; // Old WITHDRAW_CUP stage between release and return.
  bool demo_faulted_{false}, demo_cancel_requested_{false};
  std::string demo_stage_, demo_completed_stage_;
  std::string demo_cup_id_, demo_detail_, pick_phase_;
  int demo_retries_{0}, demo_max_retries_{2};
  double demo_retry_delay_{1.0}, demo_pick_timeout_{180.0};
  TimePoint demo_deadline_{}, demo_stage_started_{};

  // PICK_DYNAMIXEL requested at REFINE_CUP start; held before motion until released.
  bool demo_prepare_pick_{false};
  bool pick_owned_{false};          // The live pick request belongs to PICK_DYNAMIXEL.
  bool pick_early_{false};          // Sent during REFINE_CUP; PICK_DYNAMIXEL has not taken it.
  bool pick_holding_{false};        // PICK_HOLD feedback received.
  bool pick_held_at_entry_{false};  // Already holding when PICK_DYNAMIXEL began.
  bool pick_release_wanted_{false}, pick_cancel_wanted_{false};
  std::string pick_release_error_;
  TimePoint pick_sent_at_{}, pick_release_sent_at_{};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pick_release_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture pick_release_future_;
  struct DemoFinish {
    bool success;
    std::string code, detail;
    bool resumable;
  };
  std::optional<DemoFinish> demo_finish_; // Sent once an unowned pick request has ended.
  TimePoint demo_finish_deadline_{};

  // Cell rounds PICK_DYNAMIXEL -> HANDOVER -> APPROACH_CUP -> REFINE_CUP until the basket is empty.
  enum class BasketVerdict { Unknown, HasCell, EmptyConfirmed };
  void enter_demo_stage(const std::string & stage);
  void finish_cell_round();
  void decide_empty_basket(const std::string & pick_message);
  int demo_max_cell_rounds_{10};
  double demo_basket_empty_max_age_{5.0};
  int demo_rounds_{0};       // Toward demo.max_cell_rounds; a resume after the limit restarts it.
  int demo_total_rounds_{0}; // Since the last start command.
  bool demo_round_limit_hit_{false};
  BasketVerdict demo_basket_{BasketVerdict::Unknown};
  std::map<std::string, double> demo_round_stage_seconds_;
  double demo_cell_seconds_{0}, demo_cup_seconds_{0};
  TimePoint demo_started_{}, demo_stage_entered_{};

  using Action = demo_1f::action::Handover;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using ApproachAction = demo_1f::action::ApproachCup;
  using ApproachGoalHandle = rclcpp_action::ServerGoalHandle<ApproachAction>;
  rclcpp_action::Server<Action>::SharedPtr make_handover_server(
      const std::string & action_name, bool prepare_only);
  void setup_cup_servers();
  rclcpp_action::Server<ApproachAction>::SharedPtr make_cup_server(
      const std::string & name, CupMotion & module);
  void tick_cup_action();
  void publish_cup_target();
  void tick();
  void publish_status();
  rclcpp::Node & node_;
  RobotContext & context_;
  HandoverMotion & motion_;
  InitialPoseMotion * initial_motion_;
  std::vector<CupMotionRegistration> cup_motions_;
  CupMotion * active_cup_motion_{nullptr};
  bool cup_motion_blocked() const;
  std::vector<rclcpp_action::Server<ApproachAction>::SharedPtr> cup_servers_;
  std::shared_ptr<ApproachGoalHandle> cup_goal_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr approach_target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cup_estimate_publisher_;
  rclcpp_action::Server<Action>::SharedPtr server_, prepare_server_;
  rclcpp::Publisher<demo_1f::msg::DemoStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_, reset_demo_service_;
  // Accepted request retained until its final action result is sent.
  std::shared_ptr<GoalHandle> active_goal_handle_;
  // A goal was accepted; its accepted callback has not started yet.
  bool goal_start_pending_{false};
  bool shutdown_requested_{false};
  TimePoint last_publish_{Clock::now()};
};
} // namespace demo_1f

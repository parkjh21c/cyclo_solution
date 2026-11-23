#pragma once
#include "demo_1f/action/approach_cup.hpp"
#include "demo_1f/action/handover.hpp"
#include "demo_1f/action/run_demo.hpp"
#include "demo_1f/core/head_command_port.hpp"
#include "demo_1f/core/side_motion_port.hpp"
#include <ffw_vision_msgs/action/pick_dynamixel.hpp>
#include "demo_1f/motions/common/cup_motion.hpp"
#include "demo_1f/motions/handover/handover_motion.hpp"
#include "demo_1f/motions/initial/initial_pose_motion.hpp"
#include "demo_1f/msg/demo_status.hpp"
#include <cstdint>
#include <map>
#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <utility>
#include <vector>

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
      std::vector<CupMotionRegistration> cup_motions = {},
      InitialPoseMotion * initial_motion = nullptr, HeadCommandPort * head = nullptr,
      SideMotionPort * side = nullptr);
  void request_shutdown();
  bool can_shutdown() const;
  bool demo_faulted() const { return demo_faulted_; }

private:
  friend class TaskManagerTestAccess;
  using DemoAction = demo_1f::action::RunDemo;
  using DemoHandle = rclcpp_action::ServerGoalHandle<DemoAction>;
  using PickAction = ffw_vision_msgs::action::PickDynamixel;
  using PickHandle = rclcpp_action::ClientGoalHandle<PickAction>;
  static std::string head_pose_for_task(const std::string & task);
  void command_head(const std::string & task); // Never waits; skips the pose sent last.
  void cancel_head();                           // Request only; the head pose becomes unknown.
  HeadCommandPort * head_{nullptr};
  std::string head_last_sent_;
  // Independent other-arm return started with APPROACH_CUP or PLACE_CUP.
  struct SideFailure {
    std::string code, detail;
    bool before_motion;
  };
  static bool waits_for_side(const std::string & stage);
  bool side_active() const; // Running or stopping.
  bool b2_return_ready(std::string & reason) const;
  // Empty sequence: the port's configured return; otherwise that sequence in the same slot.
  // Its arm (not the name) decides the left or right return rules.
  bool start_side_return(std::string & reason, const std::string & sequence = {});
  void start_left_return(); // After REFINE_CUP; a failure before motion only warns.
  void cancel_side_return();
  void watch_side();
  bool side_blocks_cup(const std::string & name, const std::string & arm) const;
  SideMotionPort * side_{nullptr};
  bool demo_independent_return_{true};
  std::string place_return_sequence_; // approach.place_return_sequence; empty: the port's default
  bool side_watching_{false}, side_cancel_requested_{false}, side_waiting_{false};
  bool side_stage_stop_sent_{false};
  bool demo_prepare_deferred_{false}; // REFINE_CUP early pick waits for the side to arrive.
  bool cup_side_{false};              // The standalone cup action owns side failure and operator cancel handling.
  TimePoint side_started_at_{}, side_wait_started_{};
  std::string side_wait_arm_; // Arm whose return the stage waited for (logged at the end).
  double side_return_seconds_{-1};
  bool demo_left_return_{true};      // demo.left_return_after_refine
  bool side_left_return_{false};     // The side slot's last start was the left return.
  bool left_return_deferred_{false}; // Starts once the running right return has arrived.
  double left_return_seconds_{-1};   // Last left return, reported in the next round summary.
  std::optional<SideFailure> side_failure_;
  struct PickRequestId {
    std::uint64_t run;
    std::uint64_t request;
    bool operator==(const PickRequestId & other) const {
      return run == other.run && request == other.request;
    }
  };
  struct PickRequestContext {
    explicit PickRequestContext(PickRequestId value) : id(value) {}
    const PickRequestId id;
    bool pending{true}, goal_response_received{false}, result_received{false};
    bool dispatch_unknown{false};
    PickHandle::SharedPtr handle;
    PickAction::Feedback::ConstSharedPtr feedback;
    PickAction::Result::SharedPtr result;
    rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};
    bool cancel_wanted{false}, cancel_sent{false};
    bool owned{false}; // PICK_DYNAMIXEL has taken over this request.
    bool early{false}; // REFINE_CUP requested this pick before stage takeover.
    bool hold_requested{false}, holding{false}, held_at_entry{false};
    bool release_wanted{false}, release_sent{false}, release_response_received{false};
    bool release_timed_out{false};
    std::string phase{"PICK_STARTING"}, release_error;
    std_srvs::srv::Trigger::Response::SharedPtr release_response;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture release_future;
    TimePoint sent_at{}, stage_started_at{}, cancel_requested_at{}, release_sent_at{};
  };
  using PickRequest = std::shared_ptr<PickRequestContext>;
  bool pick_in_flight() const;
  bool pick_action_unsettled() const;
  bool pick_release_pending() const;
  bool pick_unsettled() const;
  void maintain_pick_requests();
  void cancel_pick(const PickRequest & request);
  void on_pick_goal_response(const PickRequest & request, PickHandle::SharedPtr handle);
  void on_pick_feedback(const PickRequest & request, PickAction::Feedback::ConstSharedPtr feedback);
  void on_pick_result(const PickRequest & request, const PickHandle::WrappedResult & result);
  void on_pick_release_response(const PickRequest & request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void poll_pick_release(const PickRequest & request);
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
  PickRequest pick_;
  std::vector<PickRequest> pick_requests_;
  std::uint64_t demo_run_id_{0};
  bool demo_stage_active_{false};
  bool demo_retry_wait_{false}, demo_resumable_{false}, demo_pick_only_{false};
  bool demo_initial_only_{false};
  bool demo_withdraw_cup_{false}; // Old WITHDRAW_CUP stage between release and return.
  bool demo_faulted_{false}, demo_cancel_requested_{false};
  std::string demo_stage_, demo_completed_stage_;
  std::string demo_cup_id_, demo_detail_;
  int demo_retries_{0}, demo_max_retries_{2};
  double demo_retry_delay_{1.0}, demo_pick_timeout_{180.0};
  TimePoint demo_deadline_{}, demo_stage_started_{};

  // PICK_DYNAMIXEL requested at REFINE_CUP start; held before motion until released.
  bool demo_prepare_pick_{false};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pick_release_client_;
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
  std::vector<std::pair<std::string, double>> demo_stage_seconds_; // Non-cell stages, in order.
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

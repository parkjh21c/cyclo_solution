#include "demo_1f/tasks/task_manager.hpp"
#include "demo_1f/tasks/demo_recovery.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <future>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

namespace demo_1f {
namespace {
std::uint64_t next_request_identity() {
  static std::atomic<std::uint64_t> sequence{0};
  return ++sequence;
}

TimePoint after(double seconds) {
  return Clock::now() + std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(seconds));
}

bool is_cell_stage(const std::string & stage) {
  return stage == "PICK_DYNAMIXEL" || stage == "HANDOVER" || stage == "APPROACH_CUP" ||
         stage == "REFINE_CUP";
}

bool is_cup_stage(const std::string & stage) {
  return stage == "WITHDRAW_CUP" || stage == "PICK_CUP" || stage == "HANDOVER_CUP" ||
         stage == "POUR_CUP" || stage == "HANDOVER_CUP_BACK" || stage == "PLACE_CUP";
}
} // namespace

bool TaskManager::demo_busy() const {
  return demo_goal_ || pick_unsettled() || demo_faulted_;
}

CupMotion * TaskManager::demo_cup_module(const std::string & action) {
  for (auto & module : cup_motions_) {
    if (module.action_name == action) return &module.motion;
  }
  return nullptr;
}

void TaskManager::setup_demo_server() {
  demo_max_retries_ = node_.declare_parameter<int>("demo.max_retries", 2);
  demo_retry_delay_ = node_.declare_parameter<double>("demo.retry_delay", 1.0);
  demo_pick_timeout_ = node_.declare_parameter<double>("demo.pick_timeout", 180.0);
  demo_withdraw_cup_ = node_.declare_parameter<bool>("demo.withdraw_cup", false);
  demo_max_cell_rounds_ = node_.declare_parameter<int>("demo.max_cell_rounds", 10);
  demo_basket_empty_max_age_ = node_.declare_parameter<double>("demo.basket_empty_max_age", 5.0);
  demo_prepare_pick_ = node_.declare_parameter<bool>("demo.prepare_pick_during_refine", false);
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  demo_left_return_ =
      node_.declare_parameter<bool>("demo.left_return_after_refine", true, read_only);
  // load_approach_config may already own this parameter.
  demo_independent_return_ = node_.has_parameter("demo.independent_return") ?
      node_.get_parameter("demo.independent_return").as_bool() :
      node_.declare_parameter<bool>("demo.independent_return", true);
  if (demo_max_retries_ < 0 || demo_retry_delay_ < 0 || demo_pick_timeout_ <= 0 ||
      demo_max_cell_rounds_ < 1 || !(demo_basket_empty_max_age_ > 0)) {
    throw std::invalid_argument("Invalid demo retry/timeout parameters");
  }
  reset_demo_service_ = node_.create_service<std_srvs::srv::Trigger>("~/reset_demo",
      [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = !shutdown_requested_ && !demo_busy() && !side_active() &&
            !goal_start_pending_ &&
            !active_goal_handle_ && !cup_goal_ && !motion_.running() && !cup_motion_blocked();
        if (!response->success) {
          response->message = "Cannot reset while busy or stop/scene is unconfirmed";
          return;
        }
        demo_resumable_ = false;
        demo_completed_stage_.clear();
        demo_stage_.clear();
        context_.set_task("IDLE", "", "", "Demo record reset by operator; no motion commanded");
        response->message = "Demo checkpoint reset; no motion commanded";
      });
  pick_client_ = rclcpp_action::create_client<PickAction>(&node_, "/dynamixel_pick/execute");
  pick_release_client_ = node_.create_client<std_srvs::srv::Trigger>("/dynamixel_pick/release");
  demo_server_ = rclcpp_action::create_server<DemoAction>(&node_, "/run_demo",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const DemoAction::Goal> request) {
        std::string reason;
        const bool resume = request->command == "resume";
        const bool initial_only = request->command == "initial" || (resume && demo_initial_only_);
        const bool valid = resume || request->command == "initial" ||
            request->command == "start" || request->command == "pick";
        if (!valid || shutdown_requested_ || demo_busy() || side_active() || goal_start_pending_ ||
            active_goal_handle_ || cup_goal_ || motion_.running() || cup_motion_blocked() ||
            (resume && !demo_resumable_) ||
            !context_.ready(reason) ||
            (!resume && request->command != "pick" &&
                (!initial_motion_ || !initial_motion_->is_ready_to_start(reason))) ||
            // A resumed cell round picks again after REFINE_CUP.
            (!initial_only &&
                (!resume || demo_stage_ == "INITIALIZE_ROBOT" || is_cell_stage(demo_stage_)) &&
                !pick_client_->action_server_is_ready()) ||
            (!initial_only && request->command != "pick" &&
                (!demo_cup_module("/approach_cup") || !demo_cup_module("/refine_cup") ||
                    !demo_cup_module("/pick_cup") || !demo_cup_module("/handover_cup") ||
                    !demo_cup_module("/pour_cup") || !demo_cup_module("/handover_cup_back") ||
                    !demo_cup_module("/place_cup") ||
                    (demo_withdraw_cup_ && !demo_cup_module("/withdraw_cup"))))) {
          RCLCPP_WARN(node_.get_logger(),
              "Rejected demo: %s", reason.empty() ?
              "busy, backend unavailable, invalid command, or no safe resume checkpoint" : reason.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        goal_start_pending_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<DemoHandle> handle) {
        if (handle != demo_goal_) return rclcpp_action::CancelResponse::REJECT;
        demo_cancel_requested_ = true;
        cancel_demo_motion();
        cancel_side_return(); // Operator cancel stops the side return too.
        left_return_deferred_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<DemoHandle> handle) {
        goal_start_pending_ = false;
        demo_goal_ = handle;
        demo_run_id_ = next_request_identity();
        pick_.reset();
        side_failure_.reset(); // An earlier run's side outcome is not this run's.
        side_waiting_ = demo_prepare_deferred_ = false;
        demo_cancel_requested_ = shutdown_requested_;
        demo_retries_ = 0;
        demo_retry_wait_ = false;
        demo_stage_active_ = false;
        if (handle->get_goal()->command != "resume") {
          enter_demo_stage(
              handle->get_goal()->command == "pick" ? "PICK_DYNAMIXEL" : "INITIALIZE_ROBOT");
          demo_completed_stage_.clear();
          demo_cup_id_ = handle->get_goal()->cup_id.empty() ? "cup" : handle->get_goal()->cup_id;
          demo_pick_only_ = handle->get_goal()->command == "pick";
          demo_initial_only_ = handle->get_goal()->command == "initial";
          demo_rounds_ = demo_total_rounds_ = 0;
          demo_round_limit_hit_ = false;
          demo_basket_ = BasketVerdict::Unknown;
          demo_round_stage_seconds_.clear();
          demo_cell_seconds_ = demo_cup_seconds_ = 0;
          demo_stage_seconds_.clear();
          left_return_seconds_ = -1; // A standalone menu 5 return is not this run's.
          demo_started_ = Clock::now();
        } else {
          demo_pick_only_ = false;
          if (demo_round_limit_hit_) demo_rounds_ = 0; // New round batch after CELL_ROUND_LIMIT.
          demo_round_limit_hit_ = false;
          demo_stage_entered_ = Clock::now(); // Stage times exclude the operator pause.
        }
        demo_resumable_ = false;
        demo_detail_ = "Starting " + demo_stage_;
        context_.set_task("RUNNING", "right_to_left");
      });
}

void TaskManager::cancel_demo_motion() {
  cancel_pick();
  if (demo_stage_ == "HANDOVER") motion_.cancel();
  if ((demo_stage_ == "INITIALIZE_ROBOT" || demo_stage_ == "RETURN_INITIAL") && initial_motion_)
    initial_motion_->cancel();
  if (active_cup_motion_ && demo_goal_) active_cup_motion_->cancel();
  cancel_head();
}

void TaskManager::start_demo_stage() {
  demo_stage_started_ = Clock::now();
  demo_stage_active_ = true;
  demo_prepare_deferred_ = false;
  if (side_waiting_) {
    side_waiting_ = false;
    RCLCPP_INFO(node_.get_logger(), "%s waited %.1f s for the %s arm return",
        demo_stage_.c_str(), elapsed(side_wait_started_), side_wait_arm_.c_str());
  }
  std::string reason;
  if (demo_stage_ == "INITIALIZE_ROBOT" || demo_stage_ == "RETURN_INITIAL") {
    cancel_head(); // The initial sequence moves the head through MoveIt on the same controller.
    if (!initial_motion_ || !initial_motion_->start(reason)) {
      finish_demo(false, "NOT_READY", reason.empty() ? "Initial pose module missing" : reason, true);
    }
    return;
  }
  command_head(demo_stage_); // Runs beside the stage; its outcome is only logged.
  if (demo_stage_ == "APPROACH_CUP" && side_ && demo_independent_return_ &&
      !start_side_return(reason)) {
    // Nothing moved: handled like the stage's other failures before motion.
    fail_demo_stage("SIDE_BEFORE_MOTION", reason,
        demo_can_retry(demo_stage_, "SIDE_BEFORE_MOTION"));
    return;
  }
  if (demo_stage_ == "PICK_DYNAMIXEL") {
    if (pick_ && pick_->early) { // Requested during REFINE_CUP: take it over instead of a new request.
      pick_->early = false;
      pick_->owned = true;
      pick_->stage_started_at = demo_stage_started_;
      pick_->held_at_entry = pick_->holding;
      if (pick_->result) {
        RCLCPP_INFO(node_.get_logger(),
            "Prepared pick: sent %.1f s into the demo at REFINE_CUP start; finished before "
            "PICK_DYNAMIXEL (%s)",
            std::chrono::duration<double>(pick_->sent_at - demo_started_).count(),
            pick_->result->error_code.empty() ? "no code" : pick_->result->error_code.c_str());
        return; // tick_demo handles the result as if it arrived now.
      }
      pick_->release_wanted = true;
      demo_detail_ = pick_->holding ? "Prepared pick holding; releasing" :
                                     "Prepared pick still planning";
      return;
    }
    const auto error = send_pick_goal(false);
    if (!error.empty()) finish_demo(false, "STOP_UNCONFIRMED", error);
    return;
  }
  if (demo_stage_ == "HANDOVER") {
    if (!motion_.start("right_to_left", reason)) {
      finish_demo(false, "HANDOVER_NOT_READY", reason);
    }
    return;
  }
  // Cup stage -> action and the hand that holds or takes the cup.
  static const std::map<std::string, std::pair<std::string, std::string>> cup_stages = {
      {"APPROACH_CUP", {"/approach_cup", "left"}}, {"REFINE_CUP", {"/refine_cup", "left"}},
      {"WITHDRAW_CUP", {"/withdraw_cup", "both"}}, {"PICK_CUP", {"/pick_cup", "left"}},
      {"HANDOVER_CUP", {"/handover_cup", "left"}}, {"POUR_CUP", {"/pour_cup", "right"}},
      {"HANDOVER_CUP_BACK", {"/handover_cup_back", "right"}}, {"PLACE_CUP", {"/place_cup", "left"}}};
  const auto stage = cup_stages.find(demo_stage_);
  active_cup_motion_ = stage == cup_stages.end() ? nullptr : demo_cup_module(stage->second.first);
  const auto arm = stage == cup_stages.end() ? std::string() : stage->second.second;
  if (demo_independent_return_ &&
      (demo_stage_ == "HANDOVER_CUP_BACK" || demo_stage_ == "PLACE_CUP") &&
      !b2_return_ready(reason)) {
    finish_demo(false, "NOT_READY", reason, false);
    return;
  }
  if (!active_cup_motion_ || !active_cup_motion_->start(arm, demo_cup_id_, reason, true)) {
    finish_demo(false, "NOT_READY", reason.empty() ? "Cup module missing" : reason, true);
    return;
  }
  if (demo_stage_ == "PLACE_CUP" && demo_independent_return_ && !start_side_return(reason)) {
    active_cup_motion_->cancel();
    fail_demo_stage("SIDE_BEFORE_MOTION", reason, false);
    return;
  }
  if (demo_stage_ == "REFINE_CUP") {
    // The early pick moves the side arm: it waits until the side return has arrived.
    demo_prepare_deferred_ = side_active();
    if (!demo_prepare_deferred_) prepare_pick();
  }
}

bool TaskManager::pick_in_flight() const {
  return std::any_of(pick_requests_.begin(), pick_requests_.end(), [](const auto & request) {
    return request->pending || request->handle;
  });
}

bool TaskManager::pick_action_unsettled() const {
  return pick_in_flight() || std::any_of(pick_requests_.begin(), pick_requests_.end(),
      [](const auto & request) {
        return request->dispatch_unknown || !request->result || !request->result->stop_confirmed;
      });
}

bool TaskManager::pick_release_pending() const {
  return std::any_of(pick_requests_.begin(), pick_requests_.end(), [](const auto & request) {
    return request->release_sent && !request->release_response_received;
  });
}

bool TaskManager::pick_unsettled() const {
  return pick_action_unsettled() || pick_release_pending();
}

void TaskManager::maintain_pick_requests() {
  for (const auto & request : pick_requests_) {
    poll_pick_release(request);
  }
  pick_requests_.erase(std::remove_if(pick_requests_.begin(), pick_requests_.end(),
      [this](const auto & request) {
        return request != pick_ && !request->pending && !request->handle &&
            !request->dispatch_unknown && request->result && request->result->stop_confirmed &&
            (!request->release_sent || request->release_response_received) &&
            !request->release_future.valid();
      }), pick_requests_.end());
}

std::string TaskManager::send_pick_goal(bool hold) {
  if (pick_release_pending()) return "An earlier pick release has no processing confirmation";
  if (demo_faulted_ || pick_unsettled()) {
    demo_faulted_ = true;
    return "An earlier pick request has no confirmed final stop";
  }
  const auto request = std::make_shared<PickRequestContext>(
      PickRequestId{demo_run_id_, next_request_identity()});
  request->owned = !hold;
  request->early = request->hold_requested = hold;
  request->sent_at = Clock::now();
  request->stage_started_at = demo_stage_started_;
  pick_ = request;
  pick_requests_.push_back(request);
  PickAction::Goal goal;
  goal.execute = true;
  goal.move_to_observe = false; // Full demo initializes first; pick-only uses the operator's current pose.
  goal.hold_before_motion = hold;
  rclcpp_action::Client<PickAction>::SendGoalOptions options;
  options.goal_response_callback = [this, request](PickHandle::SharedPtr handle) {
    on_pick_goal_response(request, handle);
  };
  options.feedback_callback = [this, request](PickHandle::SharedPtr,
      const std::shared_ptr<const PickAction::Feedback> feedback) {
    on_pick_feedback(request, feedback);
  };
  options.result_callback = [this, request](const PickHandle::WrappedResult & result) {
    on_pick_result(request, result);
  };
  try {
    pick_client_->async_send_goal(goal, options);
  } catch (const std::exception & error) {
    request->pending = request->early = false;
    request->dispatch_unknown = true;
    demo_faulted_ = true; // Dispatch outcome is unknown; do not send another motion.
    return error.what();
  }
  return {};
}

void TaskManager::on_pick_goal_response(const PickRequest & request, PickHandle::SharedPtr handle) {
  if (request->goal_response_received) return;
  request->goal_response_received = true;
  request->pending = false;
  if (request->result_received) return;
  request->handle = handle;
  if (!handle) {
    request->dispatch_unknown = false;
    request->result_received = true;
    request->result = std::make_shared<PickAction::Result>();
    request->result->stop_confirmed = true;
    request->result->error_code = "PICK_REJECTED";
    request->result->message = "Pick backend rejected request";
  } else if (request != pick_ || request->id.run != demo_run_id_ || !demo_goal_ ||
      demo_cancel_requested_ || demo_faulted_ || request->cancel_wanted) {
    cancel_pick(request);
  }
}

void TaskManager::on_pick_feedback(const PickRequest & request,
    PickAction::Feedback::ConstSharedPtr feedback) {
  if (request->result_received || !feedback) return;
  request->feedback = feedback;
  request->holding = request->holding || feedback->phase == "PICK_HOLD";
  request->phase = feedback->phase;
  if (request == pick_ && request->id.run == demo_run_id_ && demo_goal_ &&
      !request->early && !request->cancel_wanted) demo_detail_ = feedback->detail;
}

void TaskManager::on_pick_result(const PickRequest & request, const PickHandle::WrappedResult & result) {
  if (request->result_received) return;
  request->result_received = true;
  request->handle.reset();
  request->pending = request->dispatch_unknown = request->holding = false;
  request->result_code = result.code;
  if (!result.result) {
    request->result = std::make_shared<PickAction::Result>();
    request->result->error_code = "STOP_UNCONFIRMED";
    request->result->message = "Pick backend returned no result";
  } else {
    request->result = std::make_shared<PickAction::Result>(*result.result);
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) request->result->success = false;
  }
  if (!request->result->stop_confirmed) demo_faulted_ = true;
}

void TaskManager::cancel_pick(const PickRequest & request) {
  if (!request->cancel_wanted) request->cancel_requested_at = Clock::now();
  request->cancel_wanted = true; // Acceptance after cancellation still cancels this request's handle.
  request->early = request->release_wanted = false;
  if (request->handle && !request->cancel_sent) {
    request->cancel_sent = true;
    pick_client_->async_cancel_goal(request->handle);
  }
}

void TaskManager::cancel_pick() {
  for (const auto & request : pick_requests_) {
    if (request == pick_ || request->pending || request->handle || request->dispatch_unknown ||
        (request->release_sent && !request->release_response_received))
      cancel_pick(request);
  }
}

void TaskManager::prepare_pick() {
  if (!demo_prepare_pick_ || demo_pick_only_ || demo_initial_only_) return;
  std::string reason;
  if (demo_rounds_ + 1 >= demo_max_cell_rounds_) {
    reason = "this round reaches demo.max_cell_rounds";
  } else if (pick_unsettled()) {
    reason = "an earlier pick request is still ending";
  } else if (demo_cancel_requested_ || demo_faulted_) {
    reason = "demo stopping";
  } else if (!pick_client_->action_server_is_ready() || !pick_release_client_->service_is_ready()) {
    reason = "pick backend or /dynamixel_pick/release unavailable";
  } else if (!initial_motion_) {
    reason = "initial pose module missing";
  } else if (!initial_motion_->at_initial_pose("right", reason)) {
    reason = "right arm not still at the initial pose: " + reason;
  }
  if (!reason.empty()) {
    RCLCPP_INFO(node_.get_logger(), "Prepared pick: not sent (%s)", reason.c_str());
    return;
  }
  const auto error = send_pick_goal(true);
  if (!error.empty()) {
    RCLCPP_ERROR(node_.get_logger(), "Prepared pick: dispatch failed: %s", error.c_str());
    demo_cancel_requested_ = true; // demo_faulted_ is set: REFINE_CUP stops, the demo ends faulted.
    cancel_demo_motion();
  }
}

void TaskManager::on_pick_release_response(const PickRequest & request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (request->release_response_received) return;
  request->release_response_received = static_cast<bool>(response);
  request->release_response = response;
  if (!response || !response->success) {
    if (!request->release_timed_out) {
      request->release_error = "Pick release refused: " +
          (response ? response->message : std::string("no response"));
    }
    if (!response) demo_faulted_ = true;
    cancel_pick(request);
  }
}

void TaskManager::poll_pick_release(const PickRequest & request) {
  if (!request->release_future.valid()) return;
  if (request->release_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto response = request->release_future.get();
    request->release_future = {};
    on_pick_release_response(request, response);
  } else if (!request->release_timed_out && elapsed(request->release_sent_at) > 5.0) {
    // A local timeout does not cancel remote service processing or its future response.
    request->release_timed_out = true;
    request->release_error = "Pick release unanswered for 5 s";
    demo_faulted_ = true;
    cancel_pick(request);
  }
}

void TaskManager::tick_pick_release() {
  const auto request = pick_;
  if (!request) return;
  if (request->release_future.valid()) {
    poll_pick_release(request);
    return;
  }
  if (!request->release_wanted || !request->holding || !request->handle ||
      request->cancel_wanted) return;
  request->release_wanted = false;
  request->release_sent = true;
  request->release_sent_at = Clock::now();
  RCLCPP_INFO(node_.get_logger(),
      "Prepared pick: sent %.1f s into the demo at REFINE_CUP start, released %.1f s after "
      "sending; %s when PICK_DYNAMIXEL began",
      std::chrono::duration<double>(request->sent_at - demo_started_).count(),
      elapsed(request->sent_at), request->held_at_entry ? "already holding" : "still planning");
  request->release_future = pick_release_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
}

void TaskManager::finish_demo(bool success, const std::string & code,
    const std::string & detail, bool resumable) {
  if (!demo_goal_) return;
  if (pick_in_flight() && (!pick_ || !pick_->owned)) {
    // A prepared pick request is live: cancel it and finish once its final result is in.
    cancel_pick();
    if (!demo_finish_) {
      demo_finish_ = DemoFinish{success, code, detail, resumable};
      demo_finish_deadline_ = after(10.0);
    }
    return;
  }
  finish_demo_now(success, code, detail, resumable);
}

void TaskManager::finish_demo_now(bool success, const std::string & code,
    const std::string & detail, bool resumable) {
  if (!demo_goal_) return;
  demo_finish_.reset();
  demo_resumable_ = resumable && !demo_faulted_ && !demo_cancel_requested_;
  auto result = std::make_shared<DemoAction::Result>();
  result->success = success;
  result->error_code = code;
  result->message = detail;
  result->last_completed_stage = demo_completed_stage_;
  result->holding_state = "UNVERIFIED";
  result->can_resume = demo_resumable_;
  context_.set_task(success ? "SUCCEEDED" : demo_resumable_ ? "WAITING_FOR_OPERATOR" : "FAILED",
      "right_to_left", code, detail);
  if (demo_goal_->is_canceling()) demo_goal_->canceled(result);
  else if (success) demo_goal_->succeed(result);
  else demo_goal_->abort(result);
  RCLCPP_INFO(node_.get_logger(), "Demo %s: %s", success ? "SUCCEEDED" : "STOPPED", detail.c_str());
  if (!demo_pick_only_ && !demo_initial_only_) {
    RCLCPP_INFO(node_.get_logger(),
        "Demo summary: rounds %d, cell section %.1f s, cup section %.1f s, total %.1f s",
        demo_total_rounds_, demo_cell_seconds_, demo_cup_seconds_, elapsed(demo_started_));
    if (!demo_stage_seconds_.empty()) {
      std::string text; // Each from its stage entry (waits for the other arm included).
      for (const auto & [stage, value] : demo_stage_seconds_) {
        char part[64];
        std::snprintf(part, sizeof(part), "%s%s %.1f s", text.empty() ? "" : ", ", stage.c_str(),
            value);
        text += part;
      }
      RCLCPP_INFO(node_.get_logger(), "Demo stages outside the cell rounds: %s", text.c_str());
    }
  }
  demo_goal_.reset();
  active_cup_motion_ = nullptr;
  demo_stage_active_ = false;
  demo_retry_wait_ = false;
  publish_status();
}

void TaskManager::fail_demo_stage(const std::string & code,
    const std::string & detail, bool retryable) {
  if (code == "STOP_UNCONFIRMED" || code == "SCENE_UNCONFIRMED") demo_faulted_ = true;
  retryable = retryable && !demo_faulted_ && !demo_cancel_requested_ && !cup_motion_blocked();
  if (retryable && demo_retries_ < demo_max_retries_) {
    if (pick_in_flight() && (!pick_ || !pick_->owned)) cancel_pick(); // REFINE_CUP retry.
    ++demo_retries_;
    demo_stage_active_ = false;
    demo_retry_wait_ = true;
    demo_deadline_ = after(demo_retry_delay_);
    demo_detail_ = "Retry " + std::to_string(demo_retries_) + ": " + detail;
    return;
  }
  finish_demo(false, code, detail, retryable);
}

void TaskManager::enter_demo_stage(const std::string & stage) {
  demo_stage_ = stage;
  demo_stage_entered_ = Clock::now();
}

void TaskManager::finish_cell_round() {
  ++demo_rounds_;
  ++demo_total_rounds_;
  const auto seconds = [this](const char * stage) {
    const auto it = demo_round_stage_seconds_.find(stage);
    return it == demo_round_stage_seconds_.end() ? 0.0 : it->second;
  };
  double round_seconds = 0;
  for (const auto & [stage, value] : demo_round_stage_seconds_) round_seconds += value;
  demo_cell_seconds_ += round_seconds;
  char side_text[48] = "not used";
  if (side_return_seconds_ >= 0) {
    std::snprintf(side_text, sizeof(side_text), "%.1f s", side_return_seconds_);
  } else if (side_active() && !side_left_return_) {
    std::snprintf(side_text, sizeof(side_text), "still running");
  }
  // The left return starts after this summary; it reports the previous round's return.
  char left_text[48] = "not used";
  if (left_return_seconds_ >= 0) {
    std::snprintf(left_text, sizeof(left_text), "%.1f s", left_return_seconds_);
  }
  RCLCPP_INFO(node_.get_logger(),
      "Round %d done in %.1f s: PICK_DYNAMIXEL %.1f s, HANDOVER %.1f s, "
      "left APPROACH_CUP %.1f s, REFINE_CUP %.1f s, %s arm return %s, "
      "previous left arm return %s",
      demo_total_rounds_, round_seconds, seconds("PICK_DYNAMIXEL"), seconds("HANDOVER"),
      seconds("APPROACH_CUP"), seconds("REFINE_CUP"), side_ ? side_->arm().c_str() : "side",
      side_text, left_text);
  side_return_seconds_ = left_return_seconds_ = -1;
  demo_round_stage_seconds_.clear();
  enter_demo_stage("PICK_DYNAMIXEL");
}

void TaskManager::decide_empty_basket(const std::string & pick_message) {
  // The pick's "no object twice" counts only while the wrist observer saw the basket recently.
  const auto basket = context_.basket();
  std::optional<double> age;
  if (basket && basket->header.frame_id == context_.planning_frame()) {
    age = (context_.now() - rclcpp::Time(basket->header.stamp)).seconds();
  }
  const bool fresh = age && *age <= demo_basket_empty_max_age_;
  demo_basket_ = fresh ? BasketVerdict::EmptyConfirmed : BasketVerdict::Unknown;
  char age_text[32] = "none";
  if (age) std::snprintf(age_text, sizeof(age_text), "%.1f s", *age);
  RCLCPP_INFO(node_.get_logger(),
      "Basket check: %s, rounds completed %d, basket age %s, pick: %s",
      fresh ? "EMPTY_CONFIRMED" : "UNKNOWN", demo_total_rounds_, age_text, pick_message.c_str());
  if (!fresh) {
    fail_demo_stage("BASKET_UNKNOWN",
        std::string("Pick found no object but the basket view is ") +
            (age ? "older than demo.basket_empty_max_age" : "missing or not in the planning frame") +
            "; cup section not started; " + pick_message,
        true);
    return;
  }
  demo_cell_seconds_ += elapsed(demo_stage_entered_);
  demo_round_stage_seconds_.clear();
  demo_stage_active_ = false;
  demo_retries_ = 0;
  enter_demo_stage(demo_withdraw_cup_ && demo_total_rounds_ >= 1 ? "WITHDRAW_CUP" : "PICK_CUP");
  context_.set_phase(demo_stage_, demo_completed_stage_);
  demo_detail_ = "Basket empty confirmed; starting " + demo_stage_;
}

void TaskManager::tick_demo() {
  if (!demo_goal_) return;
  if (demo_finish_) {
    auto finish = *demo_finish_;
    if (pick_in_flight()) {
      if (Clock::now() < demo_finish_deadline_) return;
      demo_faulted_ = true; // No final result after cancel: the backend stop is unknown.
      finish_demo_now(false, "STOP_UNCONFIRMED",
          "Prepared pick request did not end after cancel; " + finish.detail, false);
      return;
    }
    if (pick_action_unsettled()) {
      demo_faulted_ = true;
      finish = {false, "STOP_UNCONFIRMED", finish.detail, false};
    }
    finish_demo_now(finish.success, finish.code, finish.detail, finish.resumable);
    return;
  }
  if (demo_cancel_requested_ && !demo_stage_active_) {
    finish_demo(false, "CANCELLED", "Cancelled between stages; grippers unchanged");
    return;
  }
  if (side_failure_ && !demo_cancel_requested_ && !demo_stage_active_) {
    const auto failure = *side_failure_;
    side_failure_.reset();
    finish_demo(false, failure.code, failure.detail, false);
    return;
  }
  if (side_failure_ && !demo_cancel_requested_ && !side_stage_stop_sent_) {
    side_stage_stop_sent_ = true; // The stage stops; its end reports the side failure.
    if (active_cup_motion_) active_cup_motion_->cancel();
    if (demo_stage_ == "PICK_DYNAMIXEL") cancel_pick(); // Runs beside the left return.
  }
  if (demo_prepare_deferred_ && demo_stage_ == "REFINE_CUP" && demo_stage_active_ &&
      !side_active()) {
    demo_prepare_deferred_ = false;
    if (side_->result().state == ExecutionState::Succeeded) prepare_pick();
  }
  if (demo_retry_wait_) {
    if (Clock::now() >= demo_deadline_) demo_retry_wait_ = false;
  } else if (!demo_stage_active_) {
    // The cell pick runs beside the left return, never beside a right one.
    const bool pick_waits =
        side_ && demo_stage_ == "PICK_DYNAMIXEL" && side_->active_arm() != "left";
    if ((waits_for_side(demo_stage_) || pick_waits) && side_active()) {
      if (!side_waiting_) {
        side_waiting_ = true;
        side_wait_started_ = Clock::now();
        side_wait_arm_ = side_->active_arm(); // The next return may start before the log.
        RCLCPP_INFO(node_.get_logger(), "%s waits for the %s arm return", demo_stage_.c_str(),
            side_->active_arm().c_str());
      }
      demo_detail_ = "Waiting for the " + side_->active_arm() + " arm return before " + demo_stage_;
    } else if (demo_stage_ == "PICK_DYNAMIXEL" && (!pick_ || !pick_->early) &&
        pick_release_pending()) {
      if (demo_faulted_) {
        finish_demo(false, "PICK_RELEASE_FAILED", "Previous pick release processing is unconfirmed");
        return;
      }
      demo_detail_ = "Waiting for previous pick release processing confirmation";
    } else if (demo_stage_ == "PICK_DYNAMIXEL" && (!pick_ || !pick_->early) && pick_in_flight()) {
      // A cancelled prepared request is still ending; the backend takes one request at a time.
      if (elapsed(demo_stage_entered_) > 10.0) {
        demo_faulted_ = true;
        finish_demo_now(false, "STOP_UNCONFIRMED", "Cancelled prepared pick request did not end",
            false);
        return;
      }
    } else {
      start_demo_stage();
    }
  } else {
    bool done = false;
    bool success = false;
    bool retryable = false;
    std::string code, detail;
    if (demo_stage_ == "PICK_DYNAMIXEL") {
      if (demo_cancel_requested_) cancel_demo_motion();
      if (!pick_->result && elapsed(pick_->stage_started_at) > demo_pick_timeout_) {
        demo_cancel_requested_ = true;
        cancel_demo_motion();
        if (elapsed(pick_->stage_started_at) > demo_pick_timeout_ + 10.0) {
          demo_faulted_ = true;
          finish_demo(false, "STOP_UNCONFIRMED", "Pick backend timed out without a final stop result");
          return;
        }
      }
      if (!pick_->result) tick_pick_release();
      if (pick_->result) {
        done = true;
        success = pick_->result->success && pick_->result->stop_confirmed;
        code = pick_->result->error_code;
        detail = pick_->result->message;
        retryable = pick_->result->retryable && !pick_->result->motion_started;
        if (!pick_->release_error.empty()) {
          success = retryable = false;
          code = "PICK_RELEASE_FAILED";
          detail = pick_->release_error + "; " + detail;
        }
        if (!pick_->result->stop_confirmed) code = "STOP_UNCONFIRMED";
      }
    } else {
      const MotionResult * outcome = nullptr;
      if ((demo_stage_ == "INITIALIZE_ROBOT" || demo_stage_ == "RETURN_INITIAL") && initial_motion_) {
        initial_motion_->tick();
        if (!initial_motion_->running()) outcome = &initial_motion_->result();
      } else if (demo_stage_ == "HANDOVER") {
        motion_.tick();
        if (!motion_.running()) outcome = &motion_.result();
      } else if (active_cup_motion_) {
        active_cup_motion_->tick();
        if (elapsed(last_publish_) >= .2) publish_cup_target();
        if (!active_cup_motion_->running()) outcome = &active_cup_motion_->result();
      }
      if (!outcome && active_cup_motion_) demo_detail_ = active_cup_motion_->result().detail;
      if (outcome) {
        done = true;
        success = outcome->sequence_completed;
        code = outcome->code;
        detail = outcome->detail;
        retryable = demo_can_retry(demo_stage_, code);
      }
    }
    if (done && side_failure_ && !demo_cancel_requested_) {
      const auto failure = *side_failure_;
      side_failure_.reset();
      if (failure.before_motion && demo_stage_ == "APPROACH_CUP") {
        fail_demo_stage(failure.code, failure.detail, demo_can_retry(demo_stage_, failure.code));
      } else {
        finish_demo(false, failure.code, failure.detail, false);
      }
      return;
    }
    if (done) {
      if (demo_cancel_requested_) {
        if (code == "STOP_UNCONFIRMED" || code == "SCENE_UNCONFIRMED") demo_faulted_ = true;
        finish_demo(false, demo_faulted_ ? "STOP_UNCONFIRMED" : "CANCELLED",
            detail.empty() ? "Stopped; grippers unchanged" : detail);
        return;
      }
      if (!success && demo_stage_ == "PICK_DYNAMIXEL" && !demo_pick_only_ &&
          code == "BASKET_EMPTY" && pick_->result && !pick_->result->motion_started &&
          pick_->result->stop_confirmed) {
        decide_empty_basket(detail); // Not a failure or retry when the basket view is fresh.
      } else if (!success) {
        fail_demo_stage(code.empty() ? "EXECUTION_FAILED" : code, detail, retryable);
      } else {
        const double stage_seconds = elapsed(demo_stage_entered_);
        if (is_cell_stage(demo_stage_)) demo_round_stage_seconds_[demo_stage_] = stage_seconds;
        if (is_cup_stage(demo_stage_)) demo_cup_seconds_ += stage_seconds;
        if (!is_cell_stage(demo_stage_)) demo_stage_seconds_.emplace_back(demo_stage_, stage_seconds);
        demo_completed_stage_ = demo_stage_;
        demo_stage_active_ = false;
        demo_retries_ = 0;
        active_cup_motion_ = nullptr;
        if (demo_stage_ == "INITIALIZE_ROBOT" || demo_stage_ == "RETURN_INITIAL") {
          head_last_sent_ = "look_middle"; // The initial sequence leaves the head there.
        }
        if (demo_stage_ == "INITIALIZE_ROBOT") {
          if (demo_initial_only_) {
            finish_demo(true, "", "Initial pose reached; no pick started");
            return;
          }
          enter_demo_stage("PICK_DYNAMIXEL");
        } else if (demo_stage_ == "PICK_DYNAMIXEL") {
          demo_basket_ = BasketVerdict::HasCell;
          enter_demo_stage("HANDOVER");
          if (demo_pick_only_) {
            finish_demo(true, "", "Pick complete; resume continues with handover", true);
            return;
          }
        } else if (demo_stage_ == "HANDOVER") enter_demo_stage("APPROACH_CUP");
        else if (demo_stage_ == "APPROACH_CUP") enter_demo_stage("REFINE_CUP");
        else if (demo_stage_ == "REFINE_CUP") {
          finish_cell_round(); // Next round picks again; the cup section waits for an empty basket.
          if (demo_left_return_ && side_) {
            // A running right return keeps the slot: the left return follows its arrival.
            left_return_deferred_ = side_active();
            if (!left_return_deferred_) start_left_return();
          }
          if (demo_rounds_ >= demo_max_cell_rounds_) {
            demo_round_limit_hit_ = true;
            finish_demo(false, "CELL_ROUND_LIMIT",
                std::to_string(demo_rounds_) + " cell rounds reached demo.max_cell_rounds; "
                "resume picks again with a new round count", true);
            return;
          }
        } else if (demo_stage_ == "WITHDRAW_CUP") enter_demo_stage("PICK_CUP");
        else if (demo_stage_ == "PICK_CUP") enter_demo_stage("HANDOVER_CUP");
        else if (demo_stage_ == "HANDOVER_CUP") enter_demo_stage("POUR_CUP");
        else if (demo_stage_ == "POUR_CUP") enter_demo_stage("HANDOVER_CUP_BACK");
        else if (demo_stage_ == "HANDOVER_CUP_BACK") enter_demo_stage("PLACE_CUP");
        else if (demo_stage_ == "PLACE_CUP") enter_demo_stage("RETURN_INITIAL");
        else {
          finish_demo(true, "", std::to_string(demo_total_rounds_) +
                                    " cell round(s) until the basket was empty, then cup pick, "
                                    "pour, cup handover back and place completed; both arms at "
                                    "the initial pose; transfers, pour and placement are not "
                                    "sensor-verified");
          return;
        }
        context_.set_phase(demo_stage_, demo_completed_stage_);
        demo_detail_ = "Starting " + demo_stage_;
      }
    }
  }
  if (demo_goal_ && elapsed(last_publish_) >= .2) {
    auto feedback = std::make_shared<DemoAction::Feedback>();
    feedback->stage = demo_stage_;
    feedback->phase = demo_retry_wait_ ? "RECOVERY_WAIT" : demo_stage_ == "PICK_DYNAMIXEL" ?
        (pick_ ? pick_->phase : std::string()) : context_.snapshot().phase;
    const bool in_round = !demo_pick_only_ && !demo_initial_only_ && is_cell_stage(demo_stage_);
    feedback->detail = in_round
        ? "Round " + std::to_string(demo_total_rounds_ + 1) + ": " + demo_detail_ : demo_detail_;
    feedback->retry_count = demo_retries_;
    feedback->holding_state = "UNVERIFIED";
    demo_goal_->publish_feedback(feedback);
    context_.set_task(demo_retry_wait_ ? "RECOVERING" : "RUNNING", "right_to_left", "",
        feedback->detail);
  }
}
} // namespace demo_1f

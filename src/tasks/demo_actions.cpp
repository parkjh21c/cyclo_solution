#include "demo_1f/tasks/task_manager.hpp"
#include "demo_1f/tasks/demo_recovery.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <future>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

namespace demo_1f {
namespace {
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
  return demo_goal_ || pick_pending_ || pick_handle_ || demo_faulted_;
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
  if (demo_max_retries_ < 0 || demo_retry_delay_ < 0 || demo_pick_timeout_ <= 0 ||
      demo_max_cell_rounds_ < 1 || !(demo_basket_empty_max_age_ > 0)) {
    throw std::invalid_argument("Invalid demo retry/timeout parameters");
  }
  reset_demo_service_ = node_.create_service<std_srvs::srv::Trigger>("~/reset_demo",
      [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = !shutdown_requested_ && !demo_busy() && !goal_start_pending_ &&
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
        if (!valid || shutdown_requested_ || demo_busy() || goal_start_pending_ ||
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
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<DemoHandle> handle) {
        goal_start_pending_ = false;
        demo_goal_ = handle;
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
}

void TaskManager::start_demo_stage() {
  demo_stage_started_ = Clock::now();
  demo_stage_active_ = true;
  std::string reason;
  if (demo_stage_ == "INITIALIZE_ROBOT" || demo_stage_ == "RETURN_INITIAL") {
    if (!initial_motion_ || !initial_motion_->start(reason)) {
      finish_demo(false, "NOT_READY", reason.empty() ? "Initial pose module missing" : reason, true);
    }
    return;
  }
  if (demo_stage_ == "PICK_DYNAMIXEL") {
    if (pick_early_) { // Requested during REFINE_CUP: take it over instead of a new request.
      pick_early_ = false;
      pick_owned_ = true;
      pick_held_at_entry_ = pick_holding_;
      if (pick_result_) {
        RCLCPP_INFO(node_.get_logger(),
            "Prepared pick: sent %.1f s into the demo at REFINE_CUP start; finished before "
            "PICK_DYNAMIXEL (%s)",
            std::chrono::duration<double>(pick_sent_at_ - demo_started_).count(),
            pick_result_->error_code.empty() ? "no code" : pick_result_->error_code.c_str());
        return; // tick_demo handles the result as if it arrived now.
      }
      pick_release_wanted_ = true;
      demo_detail_ = pick_holding_ ? "Prepared pick holding; releasing" :
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
  if (!active_cup_motion_ || !active_cup_motion_->start(arm, demo_cup_id_, reason, true)) {
    finish_demo(false, "NOT_READY", reason.empty() ? "Cup module missing" : reason, true);
    return;
  }
  if (demo_stage_ == "REFINE_CUP") prepare_pick();
}

std::string TaskManager::send_pick_goal(bool hold) {
  pick_result_.reset();
  pick_handle_.reset();
  pick_pending_ = true;
  pick_cancel_sent_ = pick_cancel_wanted_ = false;
  pick_holding_ = pick_held_at_entry_ = pick_release_wanted_ = false;
  pick_release_error_.clear();
  pick_release_future_ = {};
  pick_owned_ = !hold;
  pick_early_ = hold;
  pick_sent_at_ = Clock::now();
  pick_phase_ = "PICK_STARTING";
  PickAction::Goal goal;
  goal.execute = true;
  goal.move_to_observe = false; // Full demo initializes first; pick-only uses the operator's current pose.
  goal.hold_before_motion = hold;
  rclcpp_action::Client<PickAction>::SendGoalOptions options;
  options.goal_response_callback = [this](PickHandle::SharedPtr handle) {
    pick_pending_ = false;
    pick_handle_ = handle;
    if (!handle) {
      pick_result_ = std::make_shared<PickAction::Result>();
      pick_result_->stop_confirmed = true;
      pick_result_->error_code = "PICK_REJECTED";
      pick_result_->message = "Pick backend rejected request";
    } else if (!demo_goal_ || demo_cancel_requested_ || demo_faulted_ || pick_cancel_wanted_) {
      pick_cancel_sent_ = true;
      pick_client_->async_cancel_goal(handle);
    }
  };
  options.feedback_callback = [this](PickHandle::SharedPtr,
      const std::shared_ptr<const PickAction::Feedback> feedback) {
    pick_holding_ = pick_holding_ || feedback->phase == "PICK_HOLD";
    pick_phase_ = feedback->phase;
    if (!pick_early_) demo_detail_ = feedback->detail; // REFINE_CUP keeps its own progress.
  };
  options.result_callback = [this](const PickHandle::WrappedResult & result) {
    pick_handle_.reset();
    pick_pending_ = false;
    pick_holding_ = false;
    pick_result_ = result.result;
    if (!pick_result_) {
      pick_result_ = std::make_shared<PickAction::Result>();
      pick_result_->error_code = "STOP_UNCONFIRMED";
      pick_result_->message = "Pick backend returned no result";
    } else if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      pick_result_->success = false;
    }
  };
  try {
    pick_client_->async_send_goal(goal, options);
  } catch (const std::exception & error) {
    pick_pending_ = pick_early_ = false;
    demo_faulted_ = true; // Dispatch outcome is unknown; do not send another motion.
    return error.what();
  }
  return {};
}

void TaskManager::cancel_pick() {
  pick_cancel_wanted_ = true; // A request still awaiting acceptance is cancelled on acceptance.
  pick_early_ = pick_release_wanted_ = false;
  if (pick_handle_ && !pick_cancel_sent_) {
    pick_cancel_sent_ = true;
    pick_client_->async_cancel_goal(pick_handle_);
  }
}

void TaskManager::prepare_pick() {
  if (!demo_prepare_pick_ || demo_pick_only_ || demo_initial_only_) return;
  std::string reason;
  if (demo_rounds_ + 1 >= demo_max_cell_rounds_) {
    reason = "this round reaches demo.max_cell_rounds";
  } else if (pick_handle_ || pick_pending_) {
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

void TaskManager::tick_pick_release() {
  if (pick_release_future_.valid()) {
    std::string error;
    if (pick_release_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto response = pick_release_future_.get();
      if (!response || !response->success) {
        error = "Pick release refused: " + (response ? response->message : std::string("no response"));
      }
    } else if (elapsed(pick_release_sent_at_) > 5.0) {
      error = "Pick release unanswered for 5 s";
    } else {
      return;
    }
    pick_release_future_ = {};
    if (!error.empty()) {
      pick_release_error_ = error;
      cancel_pick(); // The arm has not moved; the result then fails the stage.
    }
    return;
  }
  if (!pick_release_wanted_ || !pick_holding_ || !pick_handle_ || pick_cancel_wanted_) return;
  pick_release_wanted_ = false;
  pick_release_sent_at_ = Clock::now();
  RCLCPP_INFO(node_.get_logger(),
      "Prepared pick: sent %.1f s into the demo at REFINE_CUP start, released %.1f s after "
      "sending; %s when PICK_DYNAMIXEL began",
      std::chrono::duration<double>(pick_sent_at_ - demo_started_).count(),
      elapsed(pick_sent_at_), pick_held_at_entry_ ? "already holding" : "still planning");
  pick_release_future_ = pick_release_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
}

void TaskManager::finish_demo(bool success, const std::string & code,
    const std::string & detail, bool resumable) {
  if (!demo_goal_) return;
  if ((pick_handle_ || pick_pending_) && !pick_owned_) {
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
    if ((pick_handle_ || pick_pending_) && !pick_owned_) cancel_pick(); // REFINE_CUP retry.
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
  RCLCPP_INFO(node_.get_logger(),
      "Round %d done in %.1f s: PICK_DYNAMIXEL %.1f s, HANDOVER %.1f s, APPROACH_CUP %.1f s, "
      "REFINE_CUP %.1f s",
      demo_total_rounds_, round_seconds, seconds("PICK_DYNAMIXEL"), seconds("HANDOVER"),
      seconds("APPROACH_CUP"), seconds("REFINE_CUP"));
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
    if (pick_handle_ || pick_pending_) {
      if (Clock::now() < demo_finish_deadline_) return;
      demo_faulted_ = true; // No final result after cancel: the backend stop is unknown.
      finish_demo_now(false, "STOP_UNCONFIRMED",
          "Prepared pick request did not end after cancel; " + finish.detail, false);
      return;
    }
    if (pick_result_ && !pick_result_->stop_confirmed) {
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
  if (demo_retry_wait_) {
    if (Clock::now() >= demo_deadline_) demo_retry_wait_ = false;
  } else if (!demo_stage_active_) {
    if (demo_stage_ == "PICK_DYNAMIXEL" && !pick_early_ && (pick_handle_ || pick_pending_)) {
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
      if (!pick_result_ && elapsed(demo_stage_started_) > demo_pick_timeout_) {
        demo_cancel_requested_ = true;
        cancel_demo_motion();
        if (elapsed(demo_stage_started_) > demo_pick_timeout_ + 10.0) {
          demo_faulted_ = true;
          finish_demo(false, "STOP_UNCONFIRMED", "Pick backend timed out without a final stop result");
          return;
        }
      }
      if (!pick_result_) tick_pick_release();
      if (pick_result_) {
        done = true;
        success = pick_result_->success && pick_result_->stop_confirmed;
        code = pick_result_->error_code;
        detail = pick_result_->message;
        retryable = pick_result_->retryable && !pick_result_->motion_started;
        if (!pick_release_error_.empty()) {
          success = retryable = false;
          code = "PICK_RELEASE_FAILED";
          detail = pick_release_error_ + "; " + detail;
        }
        if (!pick_result_->stop_confirmed) code = "STOP_UNCONFIRMED";
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
    if (done) {
      if (demo_cancel_requested_) {
        if (code == "STOP_UNCONFIRMED" || code == "SCENE_UNCONFIRMED") demo_faulted_ = true;
        finish_demo(false, demo_faulted_ ? "STOP_UNCONFIRMED" : "CANCELLED",
            detail.empty() ? "Stopped; grippers unchanged" : detail);
        return;
      }
      if (!success && demo_stage_ == "PICK_DYNAMIXEL" && !demo_pick_only_ &&
          code == "BASKET_EMPTY" && pick_result_ && !pick_result_->motion_started &&
          pick_result_->stop_confirmed) {
        decide_empty_basket(detail); // Not a failure or retry when the basket view is fresh.
      } else if (!success) {
        fail_demo_stage(code.empty() ? "EXECUTION_FAILED" : code, detail, retryable);
      } else {
        const double stage_seconds = elapsed(demo_stage_entered_);
        if (is_cell_stage(demo_stage_)) demo_round_stage_seconds_[demo_stage_] = stage_seconds;
        if (is_cup_stage(demo_stage_)) demo_cup_seconds_ += stage_seconds;
        demo_completed_stage_ = demo_stage_;
        demo_stage_active_ = false;
        demo_retries_ = 0;
        active_cup_motion_ = nullptr;
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
        pick_phase_ : context_.snapshot().phase;
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

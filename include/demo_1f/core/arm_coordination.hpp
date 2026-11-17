#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>

namespace demo_1f::coordination {

enum class Arm { Left, Right };
enum class Resources { Left, Right, Both };
enum class Operation { ArmTask, ReceiveReady, PickCell, HandoverCell, PlaceCell };
enum class Phase { Preparing, WaitingPermission, Sending, Executing, ConfirmingStop, Finished };
enum class Outcome { Pending, Success, BasketEmpty, Failed, Cancelled };
enum class FailureStage { None, Preparation, Execution };

struct RoundId {
  std::uint64_t run;
  std::uint64_t cell;
  std::uint32_t attempt;

  bool operator==(const RoundId & other) const {
    return std::tie(run, cell, attempt) == std::tie(other.run, other.cell, other.attempt);
  }
};

struct WorkId {
  RoundId round;
  Operation operation;
  std::uint32_t operation_attempt{1};

  bool operator==(const WorkId & other) const {
    return round == other.round && operation == other.operation &&
           operation_attempt == other.operation_attempt;
  }

  bool operator<(const WorkId & other) const {
    return std::tie(round.run, round.cell, round.attempt, operation, operation_attempt) <
           std::tie(other.round.run, other.round.cell, other.round.attempt, other.operation,
                    other.operation_attempt);
  }
};

struct WorkState {
  Resources resources;
  Phase phase{Phase::Preparing};
  Outcome outcome{Outcome::Pending};
  bool stop_confirmed{false};
  bool cancel_requested{false};
  bool execution_authorized{false};
  bool stop_requested{false};
  FailureStage failure_stage{FailureStage::None};
};

struct ArmState {
  std::optional<WorkId> owner;
  bool stop_confirmed{false};
  bool command_issued{false};
};

// Callers serialize all policy access and supply evidence from the execution adapters.
class ArmCoordination {
public:
  ArmCoordination(std::uint64_t run, std::uint32_t max_cell_rounds)
      : current_{run, 1, 1}, max_cell_rounds_(max_cell_rounds) {
    if (run == 0 || max_cell_rounds == 0) throw std::invalid_argument("Invalid run or round limit");
  }

  RoundId current_round() const { return current_; }
  std::optional<WorkId> current_placement() const {
    if (!last_handover_) return std::nullopt;
    return WorkId{*last_handover_, Operation::PlaceCell, placement_attempt_};
  }
  bool cancel_requested() const { return cancel_requested_; }
  bool fault_locked() const { return fault_locked_; }
  bool round_limit_reached() const { return current_.cell > max_cell_rounds_; }
  const ArmState & arm(Arm value) const { return arms_[index(value)]; }

  const WorkState * work(const WorkId & id) const {
    const auto found = work_.find(id);
    return found == work_.end() ? nullptr : &found->second;
  }

  // Initial evidence is scoped to this run and cannot overwrite command history.
  bool record_initial_stop(std::uint64_t run, Arm value) {
    auto & state = arms_[index(value)];
    if (blocked() || run != current_.run || state.owner || state.command_issued) return false;
    state.stop_confirmed = true;
    return true;
  }

  bool prepare(const WorkId & id, Resources resources) {
    if (blocked() || id.round.run != current_.run || id.round.cell == 0 ||
        id.round.attempt == 0 || !valid_identity(id) || work(id)) return false;
    switch (id.operation) {
      case Operation::ReceiveReady:
        if (!(id.round == current_) || current_.cell != 1 || resources != Resources::Left)
          return false;
        break;
      case Operation::PickCell:
        if (!(id.round == current_) || round_limit_reached() || resources != Resources::Right)
          return false;
        break;
      case Operation::HandoverCell:
        if (!(id.round == current_) || round_limit_reached() || resources != Resources::Both)
          return false;
        break;
      case Operation::PlaceCell:
        if (!last_handover_ || !(id.round == *last_handover_) || resources != Resources::Left)
          return false;
        break;
      case Operation::ArmTask:
        break;
    }
    work_.emplace(id, WorkState{resources});
    return true;
  }

  bool preparation_complete(const WorkId & id) {
    auto * state = mutable_work(id);
    if (blocked() || !state || !valid_identity(id) || state->phase != Phase::Preparing)
      return false;
    state->phase = Phase::WaitingPermission;
    return true;
  }

  // No command was authorized, so this result supplies no arm stop evidence.
  bool finish_before_execution(const WorkId & id, Outcome outcome) {
    auto * state = mutable_work(id);
    if (blocked() || !state || !valid_identity(id) || state->execution_authorized ||
        (state->phase != Phase::Preparing && state->phase != Phase::WaitingPermission) ||
        (outcome != Outcome::Failed && outcome != Outcome::BasketEmpty) ||
        (outcome == Outcome::BasketEmpty && id.operation != Operation::PickCell)) return false;
    state->outcome = outcome;
    state->failure_stage = outcome == Outcome::Failed ? FailureStage::Preparation : FailureStage::None;
    state->phase = Phase::Finished;
    return true;
  }

  // Joint and gripper commands share the selected arm's single ownership slot.
  bool acquire(const WorkId & id) {
    if (id.operation == Operation::HandoverCell) return false;
    return acquire_resources(id);
  }

  bool command_accepted(const WorkId & id) {
    auto * state = mutable_work(id);
    if (blocked() || !state || state->phase != Phase::Sending || !owns(id)) return false;
    state->phase = Phase::Executing;
    return true;
  }

  // A result is execution evidence only and never confirms physical object transfer.
  bool execution_complete(const WorkId & id, Outcome outcome) {
    auto * state = mutable_work(id);
    if (blocked() || !state || !owns(id) ||
        (state->phase != Phase::Sending && state->phase != Phase::Executing) ||
        outcome == Outcome::Pending ||
        (outcome == Outcome::BasketEmpty && id.operation != Operation::PickCell)) return false;
    state->outcome = outcome;
    state->phase = Phase::ConfirmingStop;
    if (outcome == Outcome::Failed) {
      state->failure_stage = FailureStage::Execution;
      latch_fault();
    } else if (outcome == Outcome::Cancelled) {
      request_cancel();
    }
    return true;
  }

  // For both arms, confirmed means that the adapter confirmed stopping of both arms.
  bool confirm_stop(const WorkId & id, bool confirmed) {
    auto * state = mutable_work(id);
    if (!state || state->phase != Phase::ConfirmingStop || !owns(id)) return false;
    if (!confirmed) {
      latch_fault();
      return true;
    }
    state->stop_confirmed = true;
    state->stop_requested = false;
    state->phase = Phase::Finished;
    for (auto value : {Arm::Left, Arm::Right}) {
      if (uses(state->resources, value)) arms_[index(value)].stop_confirmed = true;
    }
    return true;
  }

  bool release(const WorkId & id) {
    const auto * state = work(id);
    if (!state || state->phase != Phase::Finished || !state->stop_confirmed || !owns(id))
      return false;
    for (auto value : {Arm::Left, Arm::Right}) {
      if (uses(state->resources, value)) arms_[index(value)].owner.reset();
    }
    return true;
  }

  bool handover_ready(const WorkId & handover, const WorkId & left, const WorkId & pick) const {
    return !blocked() && !round_limit_reached() && handover.operation == Operation::HandoverCell &&
           handover.round == current_ && waiting(handover) && arms_stopped_and_free() &&
           left_ready(left) && pick.operation == Operation::PickCell && pick.round == current_ &&
           completed(pick, Outcome::Success);
  }

  bool start_handover(const WorkId & handover, const WorkId & left, const WorkId & pick) {
    if (!handover_ready(handover, left, pick) || !acquire_resources(handover)) return false;
    last_handover_ = current_;
    placement_attempt_ = 1;
    ++current_.cell;
    current_.attempt = 1;
    return true;
  }

  // BasketEmpty is a backend verdict whose freshness remains an adapter responsibility.
  bool cup_section_ready(const WorkId & left, const WorkId & empty_pick) const {
    return !blocked() && !round_limit_reached() && arms_stopped_and_free() && left_ready(left) &&
           empty_pick.operation == Operation::PickCell && empty_pick.round == current_ &&
           empty_pick_finished(empty_pick);
  }

  // A placement retry preserves the admitted handover and the next pick's identity.
  bool retry_placement(const WorkId & failed, std::uint32_t attempt) {
    const auto expected = current_placement();
    const auto * state = work(failed);
    if (blocked() || !expected || !(failed == *expected) || !state ||
        state->phase != Phase::Finished || state->outcome != Outcome::Failed ||
        state->failure_stage != FailureStage::Preparation || state->execution_authorized ||
        attempt <= placement_attempt_ || arm(Arm::Left).owner || !arm(Arm::Left).stop_confirmed ||
        !completed(WorkId{failed.round, Operation::HandoverCell}, Outcome::Success)) return false;
    placement_attempt_ = attempt;
    return true;
  }

  bool retry_round(std::uint32_t attempt) {
    if (blocked() || round_limit_reached() || attempt <= current_.attempt ||
        !arms_stopped_and_free()) return false;
    for (auto & entry : work_) {
      if (entry.first.round == current_ && entry.second.phase != Phase::Finished) {
        entry.second.cancel_requested = true;
        entry.second.outcome = Outcome::Cancelled;
        entry.second.phase = Phase::Finished;
      }
    }
    current_.attempt = attempt;
    return true;
  }

  void request_cancel() {
    cancel_requested_ = true;
    stop_outstanding();
  }

  void latch_fault() {
    fault_locked_ = true;
    stop_outstanding();
  }

private:
  static std::size_t index(Arm value) { return value == Arm::Left ? 0 : 1; }
  static bool uses(Resources resources, Arm value) {
    return resources == Resources::Both ||
           (resources == Resources::Left && value == Arm::Left) ||
           (resources == Resources::Right && value == Arm::Right);
  }

  bool blocked() const { return cancel_requested_ || fault_locked_; }

  bool valid_identity(const WorkId & id) const {
    if (id.operation == Operation::PlaceCell) {
      const auto expected = current_placement();
      return expected && id == *expected;
    }
    return id.round == current_ && id.operation_attempt == 1;
  }

  WorkState * mutable_work(const WorkId & id) {
    const auto found = work_.find(id);
    return found == work_.end() ? nullptr : &found->second;
  }

  bool owns(const WorkId & id) const {
    const auto * state = work(id);
    if (!state) return false;
    for (auto value : {Arm::Left, Arm::Right}) {
      if (uses(state->resources, value) &&
          (!arm(value).owner || !(*arm(value).owner == id))) return false;
    }
    return true;
  }

  bool waiting(const WorkId & id) const {
    const auto * state = work(id);
    return state && state->phase == Phase::WaitingPermission && !state->cancel_requested;
  }

  bool acquire_resources(const WorkId & id) {
    const auto * state = work(id);
    if (blocked() || !valid_identity(id) || !waiting(id)) return false;
    if (id.operation == Operation::PlaceCell &&
        !completed(WorkId{id.round, Operation::HandoverCell}, Outcome::Success)) return false;
    for (auto value : {Arm::Left, Arm::Right}) {
      if (uses(state->resources, value) && (arm(value).owner || !arm(value).stop_confirmed))
        return false;
    }
    for (auto value : {Arm::Left, Arm::Right}) {
      if (!uses(state->resources, value)) continue;
      auto & target = arms_[index(value)];
      target.owner = id;
      target.stop_confirmed = false;
      target.command_issued = true;
    }
    mutable_work(id)->phase = Phase::Sending;
    mutable_work(id)->execution_authorized = true;
    return true;
  }

  bool arms_stopped_and_free() const {
    return !arm(Arm::Left).owner && !arm(Arm::Right).owner &&
           arm(Arm::Left).stop_confirmed && arm(Arm::Right).stop_confirmed;
  }

  bool completed(const WorkId & id, Outcome outcome) const {
    const auto * state = work(id);
    return state && state->phase == Phase::Finished && state->outcome == outcome &&
           state->stop_confirmed && !state->cancel_requested && !owns(id);
  }

  bool empty_pick_finished(const WorkId & id) const {
    const auto * state = work(id);
    return state && state->phase == Phase::Finished && state->outcome == Outcome::BasketEmpty &&
           !state->cancel_requested &&
           (!state->execution_authorized || completed(id, Outcome::BasketEmpty));
  }

  bool left_ready(const WorkId & left) const {
    if (current_.cell == 1) {
      return left.operation == Operation::ReceiveReady && left.round == current_ &&
             completed(left, Outcome::Success);
    }
    if (!last_handover_) return false;
    const WorkId handover{*last_handover_, Operation::HandoverCell};
    return left == *current_placement() &&
           completed(handover, Outcome::Success) && completed(left, Outcome::Success);
  }

  void stop_outstanding() {
    for (auto & entry : work_) {
      auto & state = entry.second;
      if (state.phase == Phase::Finished) continue;
      state.cancel_requested = true;
      state.stop_requested = owns(entry.first);
      if (state.outcome == Outcome::Pending) state.outcome = Outcome::Cancelled;
      state.phase = owns(entry.first) ? Phase::ConfirmingStop : Phase::Finished;
    }
  }

  RoundId current_;
  std::uint32_t max_cell_rounds_;
  std::array<ArmState, 2> arms_{};
  std::map<WorkId, WorkState> work_;
  std::optional<RoundId> last_handover_;
  std::uint32_t placement_attempt_{1};
  bool cancel_requested_{false};
  bool fault_locked_{false};
};

} // namespace demo_1f::coordination

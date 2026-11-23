#include "demo_1f/motions/common/side_return.hpp"
#include <utility>

namespace demo_1f {
SideReturn::SideReturn(const Config & robot, std::string sequence, ExecutorPort & executor)
    : robot_(robot), sequence_(std::move(sequence)), active_(sequence_), executor_(executor) {}

std::vector<Step> SideReturn::steps(const std::string & sequence, std::string & reason) const {
  const auto found = robot_.motion_sequences.find(sequence);
  if (found == robot_.motion_sequences.end() || !found->second.validation_error.empty()) {
    reason = "Side sequence " + sequence + (found == robot_.motion_sequences.end()
        ? " is not loaded" : ": " + found->second.validation_error);
    return {};
  }
  std::vector<Step> steps;
  for (const auto & step : found->second.steps) {
    if (step.type == "joint") steps.push_back(step);
  }
  if (steps.empty()) reason = "Side sequence " + sequence + " has no joint steps";
  return steps;
}

bool SideReturn::start(std::string & reason) {
  return start_sequence(sequence_, reason);
}

bool SideReturn::start_sequence(const std::string & sequence, std::string & reason) {
  const auto steps = this->steps(sequence, reason);
  if (steps.empty()) return false;
  active_ = sequence;
  if (executor_.start_side(steps)) return true;
  const auto outcome = executor_.side_result();
  reason = outcome.state == ExecutionState::Failed ? outcome.code + ": " + outcome.detail
                                                   : "Side operation busy or stop unconfirmed";
  return false;
}

void SideReturn::tick() {
  executor_.tick_side();
}

void SideReturn::cancel() {
  executor_.cancel_side();
}

ExecutionResult SideReturn::result() const {
  return executor_.side_result();
}

std::string SideReturn::arm() const {
  return sequence_arm(sequence_);
}

std::string SideReturn::active_arm() const {
  return sequence_arm(active_);
}

std::string SideReturn::sequence_arm(const std::string & sequence) const {
  std::string reason;
  const auto steps = this->steps(sequence.empty() ? sequence_ : sequence, reason);
  return steps.empty() ? std::string() : steps.front().arm;
}
} // namespace demo_1f

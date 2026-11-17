#include "demo_1f/motions/common/side_return.hpp"
#include <utility>

namespace demo_1f {
SideReturn::SideReturn(const Config & robot, std::string sequence, ExecutorPort & executor)
    : robot_(robot), sequence_(std::move(sequence)), executor_(executor) {}

std::vector<Step> SideReturn::steps(std::string & reason) const {
  const auto found = robot_.motion_sequences.find(sequence_);
  if (found == robot_.motion_sequences.end() || !found->second.validation_error.empty()) {
    reason = "Side sequence " + sequence_ + (found == robot_.motion_sequences.end()
        ? " is not loaded" : ": " + found->second.validation_error);
    return {};
  }
  std::vector<Step> steps;
  for (const auto & step : found->second.steps) {
    if (step.type == "joint") steps.push_back(step);
  }
  if (steps.empty()) reason = "Side sequence " + sequence_ + " has no joint steps";
  return steps;
}

bool SideReturn::start(std::string & reason) {
  const auto steps = this->steps(reason);
  if (steps.empty()) return false;
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
  std::string reason;
  const auto steps = this->steps(reason);
  return steps.empty() ? std::string() : steps.front().arm;
}
} // namespace demo_1f

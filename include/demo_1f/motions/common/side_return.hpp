#pragma once
#include "demo_1f/core/executor_port.hpp"
#include "demo_1f/core/side_motion_port.hpp"

namespace demo_1f {
// Runs a configured sequence's joint steps as the executor's side operation.
class SideReturn final : public SideMotionPort {
public:
  SideReturn(const Config & robot, std::string sequence, ExecutorPort & executor);
  bool start(std::string & reason) override;
  bool start_sequence(const std::string & sequence, std::string & reason) override;
  void tick() override;
  void cancel() override;
  ExecutionResult result() const override;
  std::string arm() const override;
  std::string active_arm() const override;
  std::string sequence_arm(const std::string & sequence) const override;

private:
  std::vector<Step> steps(const std::string & sequence, std::string & reason) const;
  const Config & robot_;
  std::string sequence_;
  std::string active_; // Sequence of the last start attempt.
  ExecutorPort & executor_;
};
} // namespace demo_1f

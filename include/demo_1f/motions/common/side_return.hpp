#pragma once
#include "demo_1f/core/executor_port.hpp"
#include "demo_1f/core/side_motion_port.hpp"

namespace demo_1f {
// Runs a configured sequence's joint steps as the executor's side operation.
class SideReturn final : public SideMotionPort {
public:
  SideReturn(const Config & robot, std::string sequence, ExecutorPort & executor);
  bool start(std::string & reason) override;
  void tick() override;
  void cancel() override;
  ExecutionResult result() const override;
  std::string arm() const override;

private:
  std::vector<Step> steps(std::string & reason) const;
  const Config & robot_;
  std::string sequence_;
  ExecutorPort & executor_;
};
} // namespace demo_1f

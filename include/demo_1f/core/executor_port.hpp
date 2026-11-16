#pragma once
#include "demo_1f/core/types.hpp"

namespace demo_1f {
class ExecutorPort {
public:
  virtual ~ExecutorPort() = default;
  virtual bool ready(const MotionSequence & sequence, std::string & reason) const = 0;
  virtual bool start(const Step & step) = 0;

  virtual bool start_round_trip(const Step &, const Step &, const Step &) {
    return false;
  }

  virtual bool start_sequence(const std::vector<Step> &) {
    return false;
  }

  // One arm's pose step with the other arm's joint steps as one combined path (false: none).
  virtual bool start_paired(const Step &, const std::vector<Step> &) {
    return false;
  }

  virtual void tick() = 0;
  virtual void cancel(
      const std::string & code = "CANCELLED", const std::string & reason = "Cancelled") = 0;
  virtual ExecutionResult result() const = 0;
};

} // namespace demo_1f

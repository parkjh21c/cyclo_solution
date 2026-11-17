#pragma once
#include "demo_1f/core/types.hpp"
#include <string>

namespace demo_1f {
// One arm's independent return beside the other arm's work; TaskManager ticks it every cycle.
class SideMotionPort {
public:
  virtual ~SideMotionPort() = default;
  virtual bool start(std::string & reason) = 0; // False: failed before any motion.
  virtual void tick() = 0;
  virtual void cancel() = 0;                     // Request only; result() reports the stop.
  virtual ExecutionResult result() const = 0;
  virtual std::string arm() const = 0;
};
} // namespace demo_1f

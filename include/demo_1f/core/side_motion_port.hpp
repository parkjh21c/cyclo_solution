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
  virtual std::string arm() const = 0; // Arm of the configured default sequence.
  // Starts another configured sequence in the same side slot; false: failed before any motion.
  virtual bool start_sequence(const std::string & sequence, std::string & reason) {
    reason = "Side port cannot run " + sequence;
    return false;
  }
  virtual std::string active_arm() const { return arm(); } // Arm of the last started sequence.
};
} // namespace demo_1f

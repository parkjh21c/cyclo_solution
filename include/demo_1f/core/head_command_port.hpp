#pragma once
#include <string>

namespace demo_1f {
// Head pose goals that run beside arm motions; nothing waits for their outcome.
class HeadCommandPort {
public:
  virtual ~HeadCommandPort() = default;
  // Sends the named pose goal; false when it could not be sent (already logged).
  virtual bool command(const std::string & name) = 0;
  // Requests cancellation of the goal in flight, if any, without waiting.
  virtual void cancel() = 0;
};
} // namespace demo_1f

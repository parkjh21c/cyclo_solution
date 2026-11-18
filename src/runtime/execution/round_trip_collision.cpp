#include "demo_1f/runtime/execution/round_trip_collision.hpp"
#include "demo_1f/runtime/execution/round_trip_path.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace demo_1f {
namespace {
struct Segment {
  size_t samples;
  std::vector<std::array<double, 6>> coefficients;
};
} // namespace

RoundTripCollisionChecks validate_round_trip_collision(
    const moveit_msgs::msg::RobotTrajectory & planned,
    const std::vector<std::string> & controlled, const Config & config,
    const planning_scene::PlanningScene & scene) {
  validate_round_trip_path(planned, controlled);
  const auto & trajectory = planned.joint_trajectory;
  const auto model = scene.getRobotModel();
  std::vector<int> indices;
  indices.reserve(trajectory.joint_names.size());
  const auto & names = model->getVariableNames();
  for (const auto & name : trajectory.joint_names) {
    if (std::find(names.begin(), names.end(), name) == names.end()) {
      throw std::runtime_error("Unknown round-trip joint: " + name);
    }
    indices.push_back(model->getVariableIndex(name));
  }
  const auto nanos = [](const builtin_interfaces::msg::Duration & time) -> int64_t {
    return int64_t(time.sec) * 1000000000LL + time.nanosec;
  };
  std::vector<Segment> segments;
  segments.reserve(trajectory.points.size() - 1);
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & a = trajectory.points[i - 1];
    const auto & b = trajectory.points[i];
    const int64_t dt_ns = nanos(b.time_from_start) - nanos(a.time_from_start);
    const double dt = double(dt_ns) * 1e-9;
    Segment segment;
    segment.samples = std::max<size_t>(5, (dt_ns + 1999999) / 2000000);
    segment.coefficients.reserve(indices.size());
    for (size_t joint = 0; joint < indices.size(); ++joint) {
      const double c0 = a.positions[joint];
      const double c1 = a.velocities[joint] * dt;
      const double c2 = .5 * a.accelerations[joint] * dt * dt;
      const double d = b.positions[joint] - c0 - c1 - c2;
      const double v = b.velocities[joint] * dt - c1 - 2 * c2;
      const double acceleration = b.accelerations[joint] * dt * dt - 2 * c2;
      std::array<double, 6> coefficients = {c0, c1, c2, 10 * d - 4 * v + acceleration / 2,
          -15 * d + 7 * v - acceleration, 6 * d - 3 * v + acceleration / 2};
      if (!std::all_of(coefficients.begin(), coefficients.end(),
              [](double value) { return std::isfinite(value); })) {
        throw std::runtime_error("Nonfinite round-trip interpolation coefficients");
      }
      segment.coefficients.push_back(coefficients);
    }
    segments.push_back(std::move(segment));
  }
  auto state = scene.getCurrentState();
  collision_detection::CollisionRequest request;
  request.group_name = config.coordinated_group;
  request.contacts = false;
  collision_detection::CollisionResult collision;
  RoundTripCollisionChecks checks;
  // Collisions once any joint moved collision_check_step since the last check (0 = every
  // sample), and always at the first and last sample; bounds at every sample.
  std::vector<double> sampled(indices.size()), checked;
  for (size_t i = 0; i < segments.size(); ++i) {
    const auto & segment = segments[i];
    // The preceding segment already checked each shared boundary with identical PVA.
    for (size_t sample = i == 0 ? 0 : 1; sample <= segment.samples; ++sample) {
      const double u = double(sample) / segment.samples;
      for (size_t joint = 0; joint < indices.size(); ++joint) {
        const auto & c = segment.coefficients[joint];
        const double position = c[0] + u * (c[1] + u * (c[2] +
            u * (c[3] + u * (c[4] + u * c[5]))));
        if (!std::isfinite(position)) {
          throw std::runtime_error("Nonfinite round-trip interpolated position");
        }
        state.setVariablePosition(indices[joint], position);
        sampled[joint] = position;
      }
      state.update();
      if (!state.satisfiesBounds(1e-8)) {
        throw std::runtime_error("Round-trip path exceeds joint bounds");
      }
      bool check = config.collision_check_step <= 0 || checked.empty() ||
                   (i + 1 == segments.size() && sample == segment.samples);
      for (size_t joint = 0; !check && joint < sampled.size(); ++joint) {
        check = std::abs(sampled[joint] - checked[joint]) >= config.collision_check_step;
      }
      if (!check) continue;
      checked = sampled;
      ++checks.samples; // Collision-checked samples.
      collision.clear();
      scene.checkCollision(request, collision, state);
      if (collision.collision) {
        request.contacts = true;
        request.max_contacts = 10;
        collision.clear();
        scene.checkCollision(request, collision, state);
        std::string pairs;
        for (const auto & contact : collision.contacts) {
          pairs += "; " + contact.first.first + " / " + contact.first.second;
        }
        throw std::runtime_error("Round-trip path is in collision" + pairs);
      }
    }
  }
  return checks;
}
} // namespace demo_1f

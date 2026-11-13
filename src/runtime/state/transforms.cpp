#include "demo_1f/runtime/state/robot_context.hpp"
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace demo_1f {
std::optional<geometry_msgs::msg::PoseStamped> RobotContext::transform_pose(
    const geometry_msgs::msg::PoseStamped & input) const {
  if (input.header.frame_id.empty()) {
    return std::nullopt;
  }
  if (input.header.frame_id == config_.planning_frame) {
    return input;
  }
  try {
    const auto transform = buffer_.lookupTransform(
        config_.planning_frame, input.header.frame_id, rclcpp::Time(input.header.stamp));
    const auto stamp = rclcpp::Time(transform.header.stamp).nanoseconds();
    // A zero transform timestamp represents an entirely static transform chain.
    if (stamp != 0) {
      const double age = (node_.now().nanoseconds() - stamp) * 1e-9;
      if (age > config_.tf_max_age || age < -0.1) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(),
            *node_.get_clock(),
            2000,
            "TF timestamp rejected: %s -> %s age=%.3f s limit=%.3f s",
            input.header.frame_id.c_str(),
            config_.planning_frame.c_str(),
            age,
            config_.tf_max_age);
        return std::nullopt;
      }
    }
    geometry_msgs::msg::PoseStamped output;
    tf2::doTransform(input, output, transform);
    return output;
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(),
        *node_.get_clock(),
        2000,
        "TF lookup failed: %s -> %s: %s",
        input.header.frame_id.c_str(),
        config_.planning_frame.c_str(),
        error.what());
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::PoseStamped> RobotContext::pose_of(
    const std::string & frame) const {
  geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = frame;
  origin.pose.orientation.w = 1.0;
  return transform_pose(origin);
}

void RobotContext::note_loop(double seconds) {
  std::lock_guard<std::mutex> lock(tf_window_mutex_);
  tf_window_.loop_max = std::max(tf_window_.loop_max, seconds);
  tf_window_.loop_sum += seconds;
  ++tf_window_.loops;
}

void RobotContext::sample_tf_age() {
  const auto now = node_.now().nanoseconds();
  std::lock_guard<std::mutex> lock(tf_window_mutex_);
  for (const auto & [name, arm] : config_.arms) {
    // Same latest lookup as pose_of(), without its age check or warnings.
    try {
      const auto stamp = rclcpp::Time(
          buffer_.lookupTransform(config_.planning_frame, arm.tcp, tf2::TimePointZero)
              .header.stamp).nanoseconds();
      if (stamp != 0) {
        const double age = (now - stamp) * 1e-9;
        const auto it = tf_window_.max_age.try_emplace(name, age).first;
        it->second = std::max(it->second, age);
        ++tf_window_.samples;
      }
    } catch (const tf2::TransformException &) {
      ++tf_window_.missing;
    }
  }
  const double window = elapsed(tf_window_.started);
  if (window < 5.0) {
    return;
  }
  std::ostringstream ages;
  for (const auto & [name, arm] : config_.arms) {
    (void)arm;
    const auto it = tf_window_.max_age.find(name);
    ages << (ages.tellp() > 0 ? ", " : "") << name << " max ";
    if (it == tf_window_.max_age.end()) {
      ages << "-";
    } else {
      ages << std::fixed << std::setprecision(3) << it->second << " s";
    }
  }
  const double loop_mean = tf_window_.loops ? tf_window_.loop_sum / tf_window_.loops : 0.0;
  RCLCPP_INFO(node_.get_logger(),
      "tf age: %s (last %.0f s, %d samples, %d missing); loop max %.0f ms, mean %.0f ms"
      " (%d passes); tf_spin_thread %s",
      ages.str().c_str(),
      window,
      tf_window_.samples,
      tf_window_.missing,
      tf_window_.loop_max * 1000,
      loop_mean * 1000,
      tf_window_.loops,
      config_.tf_spin_thread ? "on" : "off");
  tf_window_ = TfAgeWindow{};
}
} // namespace demo_1f

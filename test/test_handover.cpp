#include "demo_1f/runtime/config.hpp"
#include "demo_1f/runtime/execution/trajectory_executor.hpp"
#include "demo_1f/runtime/execution/round_trip_path.hpp"
#include "demo_1f/tasks/task_manager.hpp"
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

using namespace std::chrono_literals;

namespace demo_1f {
namespace {
class RosEnvironment : public ::testing::Environment {
  void SetUp() override {
    initialize_moveit_logging();
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

auto * environment = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

TEST(PlanningConfig, CandidateCountIsBoundedAndReadOnly) {
  auto defaults = std::make_shared<rclcpp::Node>("planning_config_default");
  EXPECT_EQ(load_config(*defaults).ompl_planning_attempts, 3);
  EXPECT_TRUE(defaults->describe_parameter("ompl_planning_attempts").read_only);
  for (int count : {0, 1, 3, 8, 9}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("ompl_planning_attempts", count)});
    auto node = std::make_shared<rclcpp::Node>("planning_config_override", options);
    if (count == 0 || count == 9) {
      EXPECT_THROW(load_config(*node), std::invalid_argument);
    } else {
      EXPECT_EQ(load_config(*node).ompl_planning_attempts, count);
    }
  }
}

TEST(PlanningConfig, ExplicitHeldArmCorridorIsLoadedAndReadOnly) {
  rclcpp::NodeOptions options;
  const std::string prefix = "motion_sequences.cup_left_to_right.";
  options.parameter_overrides({
      rclcpp::Parameter(prefix + "steps", std::vector<std::string>{"cup_ready"}),
      rclcpp::Parameter(prefix + "cup_ready.type", "dual_joint"),
      rclcpp::Parameter(prefix + "cup_ready.left_target", std::vector<double>(7, .1)),
      rclcpp::Parameter(prefix + "cup_ready.right_target", std::vector<double>(7, -.1)),
      rclcpp::Parameter(prefix + "cup_ready.hold_arm", "left"),
      rclcpp::Parameter(prefix + "cup_ready.hold_tolerance", .001)});
  auto node = std::make_shared<rclcpp::Node>("held_arm_config", options);
  auto config = load_config(*node);
  auto sequence = config.motion_sequences.at("cup_left_to_right");
  ASSERT_EQ(sequence.steps.size(), 1u);
  EXPECT_EQ(sequence.steps.front().hold_arm, "left");
  EXPECT_DOUBLE_EQ(sequence.steps.front().hold_tolerance, .001);
  EXPECT_TRUE(node->describe_parameter(prefix + "cup_ready.hold_arm").read_only);
  EXPECT_TRUE(node->describe_parameter(prefix + "cup_ready.hold_tolerance").read_only);
  EXPECT_TRUE(validate_motion_sequence(sequence, config).empty());
  sequence.steps.front().type = "dual_joint_gripper";
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
  sequence.steps.front().type = "dual_joint";
  sequence.steps.front().hold_tolerance = 0;
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
}

TEST(PlanningConfig, JointLineIsLoadedAndRequiresAPlainDualJointStep) {
  rclcpp::NodeOptions options;
  const std::string prefix = "motion_sequences.cup_left_to_right.";
  options.parameter_overrides({
      rclcpp::Parameter(prefix + "steps", std::vector<std::string>{"cup_ready"}),
      rclcpp::Parameter(prefix + "cup_ready.type", "dual_joint"),
      rclcpp::Parameter(prefix + "cup_ready.left_target", std::vector<double>(7, .1)),
      rclcpp::Parameter(prefix + "cup_ready.right_target", std::vector<double>(7, -.1)),
      rclcpp::Parameter(prefix + "cup_ready.joint_line", true)});
  auto node = std::make_shared<rclcpp::Node>("joint_line_config", options);
  auto config = load_config(*node);
  auto sequence = config.motion_sequences.at("cup_left_to_right");
  ASSERT_EQ(sequence.steps.size(), 1u);
  EXPECT_TRUE(sequence.steps.front().joint_line);
  EXPECT_TRUE(node->describe_parameter(prefix + "cup_ready.joint_line").read_only);
  EXPECT_TRUE(validate_motion_sequence(sequence, config).empty());
  auto planned = sequence;
  planned.steps.front().pipeline = "ompl";
  planned.steps.front().planner = "RRTConnectkConfigDefault";
  EXPECT_NE(validate_motion_sequence(planned, config).find("Joint line"), std::string::npos);
  auto single = sequence;
  single.steps.front().type = "joint";
  single.steps.front().arm = "left";
  single.steps.front().target = std::vector<double>(7, .1);
  EXPECT_NE(validate_motion_sequence(single, config).find("Joint line"), std::string::npos);
  EXPECT_FALSE(Step{}.joint_line);
}

TEST(PlanningConfig, JointLineReplanIsLoadedAndGripperLinesAreAccepted) {
  rclcpp::NodeOptions options;
  const std::string prefix = "motion_sequences.left_to_right.";
  options.parameter_overrides({
      rclcpp::Parameter(prefix + "steps", std::vector<std::string>{"open_approach"}),
      rclcpp::Parameter(prefix + "open_approach.type", "dual_joint_gripper"),
      rclcpp::Parameter(prefix + "open_approach.arm", "receiver"),
      rclcpp::Parameter(prefix + "open_approach.left_target", std::vector<double>(7, .1)),
      rclcpp::Parameter(prefix + "open_approach.right_target", std::vector<double>(7, -.1)),
      rclcpp::Parameter(prefix + "open_approach.gripper_target", std::vector<double>{0.0}),
      rclcpp::Parameter(prefix + "open_approach.joint_line", true),
      rclcpp::Parameter(prefix + "open_approach.joint_line_replan", true)});
  auto node = std::make_shared<rclcpp::Node>("joint_line_replan_config", options);
  auto config = load_config(*node);
  auto sequence = config.motion_sequences.at("left_to_right");
  ASSERT_EQ(sequence.steps.size(), 1u);
  EXPECT_TRUE(sequence.steps.front().joint_line);
  EXPECT_TRUE(sequence.steps.front().joint_line_replan);
  EXPECT_TRUE(node->describe_parameter(prefix + "open_approach.joint_line_replan").read_only);
  EXPECT_TRUE(validate_motion_sequence(sequence, config).empty())
      << validate_motion_sequence(sequence, config);
  auto unlined = sequence;
  unlined.steps.front().joint_line = false; // Replan alone means nothing.
  EXPECT_NE(validate_motion_sequence(unlined, config).find("replan"), std::string::npos);
  auto gripper = sequence;
  gripper.steps.front().type = "gripper";
  gripper.steps.front().target = {0.0};
  gripper.steps.front().joint_line_replan = false;
  EXPECT_NE(validate_motion_sequence(gripper, config).find("Joint line"), std::string::npos);
  EXPECT_FALSE(Step{}.joint_line_replan);
}

TEST(PlanningConfig, LeftReturnKeepsItsExplicitLeftArm) {
  rclcpp::NodeOptions options;
  const std::string prefix = "motion_sequences.left_return.";
  options.parameter_overrides({
      rclcpp::Parameter(prefix + "steps", std::vector<std::string>{"left_home"}),
      rclcpp::Parameter(prefix + "left_home.type", "joint"),
      rclcpp::Parameter(prefix + "left_home.arm", "left"),
      rclcpp::Parameter(prefix + "left_home.target",
          std::vector<double>{1.05, 0.0, 0.0, -2.27, 0.0, 0.35, 0.0})});
  auto node = std::make_shared<rclcpp::Node>("left_return_config", options);
  const auto config = load_config(*node);
  const auto & sequence = config.motion_sequences.at("left_return");
  ASSERT_EQ(sequence.steps.size(), 1u);
  EXPECT_EQ(sequence.steps.front().arm, "left"); // Not resolved through giver/receiver.
  EXPECT_EQ(sequence.steps.front().type, "joint");
}

TEST(PlanningConfig, TfSpinThreadDefaultsOff) {
  auto defaults = std::make_shared<rclcpp::Node>("tf_spin_thread_default");
  EXPECT_FALSE(load_config(*defaults).tf_spin_thread);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("tf_spin_thread", true)});
  auto node = std::make_shared<rclcpp::Node>("tf_spin_thread_override", options);
  EXPECT_TRUE(load_config(*node).tf_spin_thread);
}

TEST(PlanningConfig, HeldGripperStartToleranceDefaultsToInherit) {
  auto defaults = std::make_shared<rclcpp::Node>("held_gripper_default");
  EXPECT_EQ(load_config(*defaults).held_gripper_start_tolerance, 0.0); // 0 = continuous value
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("held_gripper_start_tolerance", .05)});
  auto node = std::make_shared<rclcpp::Node>("held_gripper_override", options);
  EXPECT_EQ(load_config(*node).held_gripper_start_tolerance, .05);
  rclcpp::NodeOptions negative;
  negative.parameter_overrides({rclcpp::Parameter("held_gripper_start_tolerance", -.01)});
  auto bad = std::make_shared<rclcpp::Node>("held_gripper_negative", negative);
  EXPECT_THROW(load_config(*bad), std::invalid_argument);
}

Config fixture_config(const std::string & prefix) {
  Config c;
  c.require_tcp_tf = false;
  c.state_max_age = 0.15;
  c.settle_time = 0.02;
  c.cancel_timeout = 0.15;
  c.gripper_stop_window = 0.04;
  c.gripper_stop_max_gap = 0.025;
  c.gripper_transport = "action";
  c.joint_states_topic = prefix + "/joint_states";
  c.move_group_action = prefix + "/move_action";
  for (const std::string name : {"left", "right"}) {
    ArmConfig arm;
    arm.name = name;
    arm.group = name;
    arm.tcp = prefix.substr(1) + "_" + name + "_tcp";
    arm.gripper_joint = name + "_gripper";
    arm.trajectory_action = prefix + "/" + name + "/follow_joint_trajectory";
    arm.trajectory_topic = prefix + "/" + name + "/joint_trajectory";
    for (int i = 0; i < 7; ++i) {
      arm.joints.push_back(name + "_joint" + std::to_string(i));
    }
    c.arms.emplace(name, arm);
  }
  for (const std::string direction : {"left_to_right", "right_to_left"}) {
    MotionSequence sequence;
    sequence.direction = direction;
    sequence.giver = direction == "left_to_right" ? "left" : "right";
    sequence.receiver = direction == "left_to_right" ? "right" : "left";
    sequence.steps = {
        {"present", "joint", sequence.giver, std::vector<double>(7, 0.1), "", 0.03, 1.0, 0.01},
        {"receiver_close", "gripper", sequence.receiver, {0.4}, "", 0.03, 1.0, 0.01},
        {"giver_release", "gripper", sequence.giver, {0.0}, "", 0.03, 1.0, 0.01},
        {"retreat", "joint", sequence.giver, std::vector<double>(7, 0.2), "", 0.03, 1.0, 0.01}};
    c.motion_sequences.emplace(direction, sequence);
  }
  return c;
}

void configure_five_step_sequences(Config & config) {
  for (auto & [direction, sequence] : config.motion_sequences) {
    (void)direction;
    Step ready;
    ready.name = "face_each_other";
    ready.type = "dual_joint";
    ready.timeout = 1.0;
    ready.left_target = std::vector<double>(7, 0.1);
    ready.right_target = std::vector<double>(7, -0.1);
    Step approach;
    approach.name = "open_approach";
    approach.type = "dual_joint_gripper";
    approach.arm = sequence.receiver;
    approach.left_target = std::vector<double>(7, 0.2);
    approach.right_target = std::vector<double>(7, -0.2);
    approach.gripper_target = {0.0};
    approach.timeout = 1.0;
    approach.tolerance = 0.017453292519943295;
    approach.planning_tolerance = 0.0005;
    Step close = sequence.steps[1];
    Step release = sequence.steps[2];
    Step retreat = ready;
    retreat.name = "retreat_together";
    retreat.left_target = std::vector<double>(7, 0.3);
    retreat.right_target = std::vector<double>(7, -0.3);
    sequence.steps = {ready, approach, close, release, retreat};
  }
}

class FakeExecutor : public ExecutorPort {
public:
  bool ready(const MotionSequence &, std::string &) const override {
    return true;
  }

  bool start(const Step & step) override {
    steps.push_back(step);
    current = {ExecutionState::Running, {}, {}};
    return true;
  }

  bool start_sequence(const std::vector<Step> & sequence) override {
    ++continuous_starts;
    steps.insert(steps.end(), sequence.begin(), sequence.end());
    current = {ExecutionState::Running, {}, {}, sequence.front().name};
    return true;
  }

  void tick() override {}

  void cancel(const std::string & code, const std::string & detail) override {
    ++cancel_count;
    current = {
        code == "CANCELLED" ? ExecutionState::Cancelled : ExecutionState::Failed, code, detail};
  }

  ExecutionResult result() const override {
    return current;
  }

  ExecutionResult current;
  std::vector<Step> steps;
  int cancel_count{0};
  int continuous_starts{0};
};

class ContextFixture : public ::testing::Test {
protected:
  void SetUp() override {
    static int sequence = 0;
    prefix = "/handover_test_" + std::to_string(++sequence);
    node = std::make_shared<rclcpp::Node>(prefix.substr(1));
    config = fixture_config(prefix);
    context = std::make_unique<RobotContext>(*node, config);
    for (const auto & [name, arm] : config.arms) {
      for (const auto & joint : arm.joints) {
        positions[joint] = 0.0;
      }
      positions[arm.gripper_joint] = 0.2;
    }
    refresh();
  }

  sensor_msgs::msg::JointState sample() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    for (const auto & [joint, position] : positions) {
      msg.name.push_back(joint);
      msg.position.push_back(position);
      msg.velocity.push_back(velocities.count(joint) ? velocities.at(joint) : 0.0);
    }
    return msg;
  }

  void refresh() {
    context->observe_joints(sample());
  }

  void feed_for(double duration, const std::function<void(int)> & update = [](int) {}) {
    const auto start = Clock::now();
    int count = 0;
    do {
      update(count++);
      refresh();
      std::this_thread::sleep_for(5ms);
    } while (elapsed(start) < duration);
    refresh();
  }

  std::string prefix;
  std::shared_ptr<rclcpp::Node> node;
  Config config;
  std::unique_ptr<RobotContext> context;
  std::map<std::string, double> positions;
  std::map<std::string, double> velocities;
};

TEST_F(ContextFixture, NormalCompletionWindowDoesNotWeakenStopConfirmation) {
  config.gripper_stop_window = .12;
  config.gripper_completion_window = .04;
  const auto after = Clock::now();
  positions["right_gripper"] = .8;
  feed_for(.07);
  std::string reason;
  auto joint = context->snapshot().joints.at("right_gripper");
  EXPECT_TRUE(context->joint_stationary("right_gripper", joint, reason, after, true)) << reason;
  EXPECT_FALSE(context->joint_stationary("right_gripper", joint, reason, after, false));
  feed_for(.1);
  joint = context->snapshot().joints.at("right_gripper");
  EXPECT_TRUE(context->joint_stationary("right_gripper", joint, reason, after, false)) << reason;
  positions["right_gripper"] += .03;
  refresh();
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason, after, true));
  std::this_thread::sleep_for(35ms);
  refresh();
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason, after, true));
}

TEST_F(ContextFixture, GripperPositionWindowAcceptsRecordedJitterButArmVelocityStillBlocks) {
  std::string reason;
  const auto stopped = [&](const std::string & joint) {
    return context->joint_stationary(joint, context->snapshot().joints.at(joint), reason);
  };
  EXPECT_FALSE(stopped("right_gripper")); // one sample cannot confirm a stop
  feed_for(0.08, [&](int i) {
    positions["right_gripper"] = (i % 2 ? 1 : -1) * 0.0015339807878856412;
    velocities["right_gripper"] = i % 2 ? -0.083566 : 0.214885;
  });
  EXPECT_TRUE(stopped("right_gripper")) << reason;
  EXPECT_DOUBLE_EQ(
      context->snapshot().joints.at("right_gripper").velocity.value(), velocities["right_gripper"]);
  EXPECT_TRUE(context->capture_targets(reason).has_value()) << reason;
  velocities["right_joint0"] = 0.214885;
  refresh();
  EXPECT_FALSE(stopped("right_joint0"));
  EXPECT_FALSE(context->capture_targets(reason).has_value());
  config.gripper_stop_mode = "velocity";
  EXPECT_FALSE(stopped("right_gripper"));
}

TEST_F(ContextFixture, GripperPositionMotionCannotPassWithZeroReportedVelocity) {
  std::string reason;
  feed_for(0.08, [&](int i) {
    positions["right_gripper"] = 0.001 * i;
  });
  auto observed = context->snapshot().joints.at("right_gripper");
  ASSERT_TRUE(observed.position_window_span.has_value());
  EXPECT_FALSE(context->joint_stationary("right_gripper", observed, reason));
  // Returning to the original position must not conceal an excursion.
  feed_for(0.06, [&](int) {
    positions["right_gripper"] = 0.2;
  });
  positions["right_gripper"] = 0.23;
  refresh();
  std::this_thread::sleep_for(5ms);
  positions["right_gripper"] = 0.2;
  refresh();
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason));
}

TEST_F(ContextFixture, GripperStopNeedsContinuousFreshSamplesAfterCancellation) {
  std::string reason;
  feed_for(0.06);
  EXPECT_TRUE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason));
  const auto cancelled = Clock::now();
  refresh();
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason, cancelled));
  feed_for(0.06);
  EXPECT_TRUE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason, cancelled));
  std::this_thread::sleep_for(35ms); // a gap invalidates the old window
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason));
  refresh();
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason));
  auto replay = sample();
  context->observe_joints(replay);
  for (int i = 0; i < 10; ++i) {
    context->observe_joints(replay);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason));
  feed_for(0.06);
  config.state_max_age = 0.01;
  std::this_thread::sleep_for(15ms);
  EXPECT_FALSE(context->joint_stationary(
      "right_gripper", context->snapshot().joints.at("right_gripper"), reason));
}

TEST_F(ContextFixture, PassiveGripperPolicyKeepsFreshnessArmAndCancellationChecks) {
  config.require_passive_gripper_stationary = false;
  config.gripper_stop_mode = "velocity";
  velocities["right_gripper"] = .2;
  velocities["left_joint0"] = .2;
  positions["left_joint0"] = 0;
  refresh();
  std::string reason;
  const auto state = context->snapshot();
  EXPECT_TRUE(context->joint_ready_for_motion("right_gripper", state.joints.at("right_gripper"), reason));
  EXPECT_FALSE(context->joint_stationary("right_gripper", state.joints.at("right_gripper"), reason));
  EXPECT_FALSE(context->joint_ready_for_motion("left_joint0", state.joints.at("left_joint0"), reason));
  config.state_max_age = .001;
  std::this_thread::sleep_for(5ms);
  EXPECT_FALSE(context->joint_ready_for_motion("right_gripper", state.joints.at("right_gripper"), reason));
}

TEST_F(ContextFixture, PartialUpdatesDoNotRefreshMissingJoints) {
  config.state_max_age = 0.02;
  std::this_thread::sleep_for(25ms);
  auto msg = sample();
  msg.name = {"left_gripper"};
  msg.position = {0.3};
  msg.velocity = {0.0};
  context->observe_joints(msg);
  std::string reason;
  EXPECT_FALSE(context->ready(reason));
  EXPECT_NE(reason.find("joint"), std::string::npos);
  EXPECT_TRUE(context->joint_fresh(context->snapshot().joints.at("left_gripper")));
}

TEST_F(ContextFixture, InvalidOrReplayedSamplesCannotOverwriteObservations) {
  auto old = sample();
  context->observe_joints(old);
  old.position[0] = 99.0;
  context->observe_joints(old);
  EXPECT_NE(context->snapshot().joints.at(old.name[0]).position, 99.0);
  auto malformed = sample();
  malformed.position[0] = std::numeric_limits<double>::quiet_NaN();
  context->observe_joints(malformed);
  EXPECT_TRUE(std::isfinite(context->snapshot().joints.at(old.name[0]).position));
  malformed = sample();
  malformed.velocity.resize(1);
  context->observe_joints(malformed);
  EXPECT_EQ(context->snapshot().joints.size(), positions.size());
}

TEST_F(ContextFixture, RejectsMissingCalibrationBeforeSendingCommands) {
  config.motion_sequences.at("left_to_right").steps[0].target.clear();
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  EXPECT_FALSE(motion.start("left_to_right", reason));
  EXPECT_NE(reason.find("calibrated"), std::string::npos);
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ContextFixture, UnknownEffortDoesNotDiscardValidPositionFeedback) {
  auto msg = sample();
  msg.position[0] = 0.15;
  msg.effort.assign(msg.name.size(), std::numeric_limits<double>::quiet_NaN());
  context->observe_joints(msg);
  const auto observed = context->snapshot().joints.at(msg.name[0]);
  EXPECT_DOUBLE_EQ(observed.position, 0.15);
  EXPECT_FALSE(observed.effort.has_value());
  std::string reason;
  EXPECT_TRUE(context->ready(reason)) << reason;
}

TEST_F(ContextFixture, BothDirectionsUseCorrectArmAndNeverClaimPhysicalTransfer) {
  for (const std::string direction : {"left_to_right", "right_to_left"}) {
    FakeExecutor executor;
    HandoverMotion motion(config, *context, executor);
    std::string reason;
    ASSERT_TRUE(motion.start(direction, reason)) << reason;
    for (int i = 0; i < 20 && motion.running(); ++i) {
      refresh();
      if (executor.current.state == ExecutionState::Running) {
        executor.current.state = ExecutionState::Succeeded;
      }
      motion.tick();
    }
    ASSERT_TRUE(motion.result().sequence_completed);
    EXPECT_FALSE(motion.result().handover_confirmed);
    ASSERT_EQ(executor.steps.size(), 4u);
    EXPECT_EQ(executor.steps[0].arm, config.motion_sequences.at(direction).giver);
    EXPECT_EQ(executor.steps[1].arm, config.motion_sequences.at(direction).receiver);
    EXPECT_EQ(executor.steps[2].arm, config.motion_sequences.at(direction).giver);
  }
}

TEST_F(ContextFixture, ReceiverFailureDoesNotReleaseGiver) {
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason));
  motion.tick();
  executor.current.state = ExecutionState::Succeeded;
  motion.tick();
  motion.tick();
  ASSERT_EQ(executor.steps.back().name, "receiver_close");
  executor.current = {ExecutionState::Failed, "TIMEOUT", "receiver did not reach target"};
  motion.tick();
  motion.tick();
  EXPECT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(motion.result().state, "FAILED");
  EXPECT_EQ(motion.result().last_completed_step, "present");
}

TEST_F(ContextFixture, CancellationBetweenStepsDoesNotStartTheNextStep) {
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("right_to_left", reason));
  motion.tick();
  executor.current.state = ExecutionState::Succeeded;
  motion.tick();
  motion.cancel();
  motion.tick();
  EXPECT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(motion.result().state, "CANCELLED");
}

TEST_F(ContextFixture, StaleFeedbackCancelsActiveExecution) {
  config.state_max_age = 0.02;
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason));
  motion.tick();
  std::this_thread::sleep_for(25ms);
  motion.tick();
  EXPECT_EQ(executor.cancel_count, 1);
  EXPECT_EQ(motion.result().code, "ROBOT_STATE_INVALID");
  EXPECT_FALSE(motion.result().sequence_completed);
}

TEST_F(ContextFixture, ContinuousApproachKeepsTransferBarrierAndCorrectNextStep) {
  configure_five_step_sequences(config);
  config.smooth_motion = true;
  for (auto & [direction, sequence] : config.motion_sequences) {
    sequence.steps[0].blend_with_next = true;
  }
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason)) << reason;
  motion.tick();
  ASSERT_EQ(executor.continuous_starts, 1);
  ASSERT_EQ(executor.steps.size(), 2u);
  motion.tick();
  EXPECT_EQ(executor.steps.size(), 2u);
  executor.current.phase = "open_approach";
  motion.tick();
  EXPECT_EQ(context->snapshot().phase, "open_approach");
  executor.current.state = ExecutionState::Succeeded;
  motion.tick();
  EXPECT_EQ(motion.result().last_completed_step, "open_approach");
  EXPECT_EQ(executor.steps.size(), 2u);
  motion.tick();
  ASSERT_EQ(executor.steps.size(), 3u);
  EXPECT_EQ(executor.steps.back().name, "receiver_close");
}

TEST_F(ContextFixture, FailedContinuousApproachCannotCloseOrRelease) {
  configure_five_step_sequences(config);
  config.smooth_motion = true;
  config.motion_sequences.at("left_to_right").steps[0].blend_with_next = true;
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason));
  motion.tick();
  executor.current = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", "collision"};
  motion.tick();
  motion.tick();
  EXPECT_FALSE(motion.running());
  EXPECT_FALSE(motion.result().sequence_completed);
  EXPECT_EQ(executor.steps.size(), 2u);
}

TEST_F(ContextFixture, BlendingCannotCrossGripperBarrierOrEndOfSequence) {
  configure_five_step_sequences(config);
  config.smooth_motion = true;
  auto sequence = config.motion_sequences.at("left_to_right");
  sequence.steps[1].blend_with_next = true;
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
  sequence.steps[1].blend_with_next = false;
  sequence.steps.back().blend_with_next = true;
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
  sequence.steps.back().blend_with_next = false;
  sequence.steps[0].velocity_scale = 1.1;
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
}

class RosFixture : public ContextFixture {
protected:
  using Move = moveit_msgs::action::MoveGroup;
  using Joint = control_msgs::action::FollowJointTrajectory;
  using MoveHandle = rclcpp_action::ServerGoalHandle<Move>;
  using JointHandle = rclcpp_action::ServerGoalHandle<Joint>;

  void SetUp() override {
    ContextFixture::SetUp();
    mock = std::make_shared<rclcpp::Node>(prefix.substr(1) + "_mock");
    state_pub = mock->create_publisher<sensor_msgs::msg::JointState>(
        config.joint_states_topic, rclcpp::SensorDataQoS());
    move_server = rclcpp_action::create_server<Move>(
        mock,
        config.move_group_action,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal> goal) {
          requests.push_back("move:" + goal->request.group_name + ":" + goal->request.planner_id);
          move_goals.push_back(*goal);
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<MoveHandle>) {
          ++cancels;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<MoveHandle> handle) {
          moves.push_back(handle);
        });
    for (const auto & [side, arm] : config.arms) {
      joint_servers[side] = rclcpp_action::create_server<Joint>(
          mock,
          arm.trajectory_action,
          [this, side](const rclcpp_action::GoalUUID &, std::shared_ptr<const Joint::Goal> goal) {
            requests.push_back("gripper:" + side);
            joint_goals.push_back(*goal);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
          [this](std::shared_ptr<JointHandle>) {
            ++cancels;
            return rclcpp_action::CancelResponse::ACCEPT;
          },
          [this](std::shared_ptr<JointHandle> handle) {
            grippers.push_back(handle);
          });
      topic_subs[side] =
          mock->create_subscription<trajectory_msgs::msg::JointTrajectory>(arm.trajectory_topic,
              rclcpp::QoS(1).reliable(),
              [this, side](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
                requests.push_back("topic:" + side);
                if (follow_topic && !msg->points.empty()) {
                  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
                    positions.at(msg->joint_names[i]) = msg->points.back().positions.at(i);
                  }
                }
              });
    }
    timer = mock->create_wall_timer(5ms, [this]() {
      auto feedback = sample();
      if (transform_feedback) transform_feedback(feedback);
      if (++feedback_count % 2) {
        for (size_t i = 0; i < feedback.name.size(); ++i) {
          for (const auto & [side, arm] : config.arms) {
            (void)side;
            if (feedback.name[i] == arm.gripper_joint &&
                (jitter_joint.empty() || jitter_joint == feedback.name[i])) {
              feedback.position[i] += gripper_position_jitter;
            }
          }
        }
      }
      state_pub->publish(feedback);
      for (auto it = moves.begin(); it != moves.end();) {
        auto result = std::make_shared<Move::Result>();
        if ((*it)->is_canceling()) {
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          (*it)->canceled(result);
          it = moves.erase(it);
        } else if (auto_complete) {
          result->error_code.val = fail_on_move == 0 || move_goals.size() == fail_on_move
                                      ? move_error : moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
          if (!move_errors.empty()) {  // per request, in order; success past the list
            result->error_code.val = move_goals.size() <= move_errors.size()
                                         ? move_errors[move_goals.size() - 1]
                                         : moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
          }
          result->error_code.message = move_error_message;
          result->error_code.source = move_error_source;
          if (move_result_has_execution) {
            result->executed_trajectory.joint_trajectory.points.resize(1);
          }
          if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            const auto & goal = *(*it)->get_goal();
            if (goal.planning_options.plan_only) {
              result->trajectory_start = goal.request.start_state;
              auto & path = result->planned_trajectory.joint_trajectory;
              trajectory_msgs::msg::JointTrajectoryPoint initial, target;
              for (const auto & joint : goal.request.goal_constraints.front().joint_constraints) {
                path.joint_names.push_back(joint.joint_name);
                const auto & start = goal.request.start_state.joint_state;
                const auto found =
                    std::find(start.name.begin(), start.name.end(), joint.joint_name);
                initial.positions.push_back(found == start.name.end()
                                                ? positions.at(joint.joint_name)
                                                : start.position.at(found - start.name.begin()));
                target.positions.push_back(joint.position);
              }
              if (path.joint_names.empty() && !goal.request.goal_constraints.front().position_constraints.empty()) {
                for (const auto & name : config.arms.at("left").joints) {
                  path.joint_names.push_back(name);
                  initial.positions.push_back(positions.at(name) - plan_start_offset);
                  target.positions.push_back(positions.at(name) + .1);
                }
                initial.time_from_start = rclcpp::Duration::from_seconds(0);
                target.time_from_start = rclcpp::Duration::from_seconds(.2);
              }
              path.points = {initial, target};
              if (transform_plan) transform_plan(*result);
            } else {
              for (const auto & constraint : goal.request.goal_constraints) {
                for (const auto & joint : constraint.joint_constraints) {
                  if (joint.joint_name != omitted_joint) {
                    positions.at(joint.joint_name) = joint.position;
                  }
                }
              }
            }
          }
          (*it)->succeed(result);
          it = moves.erase(it);
        } else {
          ++it;
        }
      }
      for (auto it = executions.begin(); it != executions.end();) {
        auto result = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Result>();
        if ((*it)->is_canceling()) {
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
          (*it)->canceled(result);
          it = executions.erase(it);
        } else if (auto_execute) {
          result->error_code.val = execute_error;
          if (execute_error == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            const auto & path = (*it)->get_goal()->trajectory.joint_trajectory;
            for (size_t i = 0; i < path.joint_names.size(); ++i) {
              if (path.joint_names[i] != omitted_joint) {
                positions.at(path.joint_names[i]) = path.points.back().positions[i];
              }
            }
          }
          (*it)->succeed(result);
          ++completed_executions;
          it = executions.erase(it);
        } else {
          ++it;
        }
      }
      for (auto it = grippers.begin(); it != grippers.end();) {
        auto result = std::make_shared<Joint::Result>();
        if ((*it)->is_canceling()) {
          (*it)->canceled(result);
          it = grippers.erase(it);
        } else if (auto_gripper) {
          const auto & trajectory = (*it)->get_goal()->trajectory;
          for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            const auto & name = trajectory.joint_names[i];
            const double target = trajectory.points.back().positions[i];
            positions.at(name) = name == blocked_gripper ? std::min(target, .25) : target;
          }
          result->error_code = Joint::Result::SUCCESSFUL;
          (*it)->succeed(result);
          it = grippers.erase(it);
        } else {
          ++it;
        }
      }
    });
    ros.add_node(node);
    ros.add_node(mock);
  }

  void TearDown() override {
    ros.remove_node(node);
    ros.remove_node(mock);
  }

  bool until(
      const std::function<bool()> & condition,
      double timeout = 2.0,
      const std::function<void()> & tick = []() {}) {
    const auto started = Clock::now();
    do {
      ros.spin_some();
      tick();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    } while (elapsed(started) < timeout);
    return false;
  }

  void warmup(ExecutorPort & executor) {
    std::string reason;
    ASSERT_TRUE(until([&]() {
      return context->ready(reason) &&
             executor.ready(config.motion_sequences.at("left_to_right"), reason);
    })) << reason;
  }

  // arm_geometry: a box on left_joint0 and right_joint0; they meet when left - right = pi/2.
  void enable_smoothing(bool arm_geometry = false) {
    config.smooth_motion = true;
    configure_five_step_sequences(config);
    for (auto & [direction, sequence] : config.motion_sequences) {
      for (auto & step : sequence.steps) {
        step.timeout = 5.0;
      }
    }
    config.motion_sequences.at("left_to_right").steps[0].blend_with_next = true;
    config.motion_sequences.at("right_to_left").steps[0].blend_with_next = true;
    std::ostringstream urdf, srdf;
    urdf << "<robot name='smoothing_mock'><link name='base_link'/>";
    srdf << "<robot name='smoothing_mock'><group name='" << config.coordinated_group << "'>";
    for (const auto & [name, position] : positions) {
      (void)position;
      const bool body = arm_geometry && (name == "left_joint0" || name == "right_joint0");
      urdf << "<link name='" << name << "_link'>"
           << (body ? "<collision><origin xyz='0.5 0 0'/><geometry><box size='0.4 0.05 0.05'/>"
                      "</geometry></collision>" : "")
           << "</link><joint name='" << name << "' type='revolute'>"
           << (body && name == "right_joint0" ? "<origin rpy='0 0 1.5707963'/>" : "")
           << "<parent link='base_link'/><child link='" << name
           << "_link'/><axis xyz='0 0 1'/><limit lower='-3' upper='3' effort='10' "
              "velocity='2'/></joint>";
      srdf << "<joint name='" << name << "'/>";
    }
    urdf << "</robot>";
    srdf << "</group></robot>";
    node->declare_parameter("robot_description", urdf.str());
    node->declare_parameter("robot_description_semantic", srdf.str());
    scene_server = mock->create_service<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",
        [this](const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr,
            moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response) {
          response->scene = scene_response;
          response->scene.name = "mock_scene";
          response->scene.robot_model_name = "smoothing_mock";
          response->scene.robot_state.joint_state = sample();
          ++scene_requests;
          if (transform_scene) transform_scene(response->scene);
        });
    execution_server = rclcpp_action::create_server<moveit_msgs::action::ExecuteTrajectory>(
        mock,
        "/execute_trajectory",
        [this](const rclcpp_action::GoalUUID &,
            std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Goal> goal) {
          requests.push_back("execute:path");
          execution_goals.push_back(*goal);
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](auto) {
          ++cancels;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](auto handle) {
          executions.push_back(handle);
        });
  }

  Step correction_step() {
    // The real planner is replaced by an action server; this test exercises dispatch only.
    enable_smoothing();
    config.smooth_motion = false;
    broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(mock);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = config.planning_frame;
    tf.header.stamp = node->now();
    tf.child_frame_id = config.arms.at("left").tcp;
    tf.transform.translation.x = .5;
    tf.transform.translation.z = .8;
    tf.transform.rotation.w = 1;
    broadcaster->sendTransform(tf);
    EXPECT_TRUE(until([&]() { return context->pose_of(tf.child_frame_id).has_value(); }));
    Step step;
    step.name = "CORRECT_TO_CUP_CENTER";
    step.type = "pose";
    step.arm = "left";
    step.frame = config.planning_frame;
    step.target = {.5, 0., .8, 0., 0., 0., 1.};
    step.orientation_slack = .17;
    step.tcp_path_margin = .04;
    step.planning_time = 3.;
    step.timeout = 5.;
    return step;
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_server;
  rclcpp_action::Server<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execution_server;
  std::vector<
      std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>>>
      executions;
  std::vector<moveit_msgs::action::ExecuteTrajectory::Goal> execution_goals;
  moveit_msgs::msg::PlanningScene scene_response;
  int execute_error{moveit_msgs::msg::MoveItErrorCodes::SUCCESS};
  bool auto_execute{true};
  double plan_start_offset{0};
  std::function<void(Move::Result &)> transform_plan;
  std::function<void(sensor_msgs::msg::JointState &)> transform_feedback;
  std::function<void(moveit_msgs::msg::PlanningScene &)> transform_scene;
  size_t scene_requests{0};
  size_t completed_executions{0};
  std::shared_ptr<rclcpp::Node> mock;
  rclcpp::executors::SingleThreadedExecutor ros;
  rclcpp_action::Server<Move>::SharedPtr move_server;
  std::map<std::string, rclcpp_action::Server<Joint>::SharedPtr> joint_servers;
  std::map<std::string, rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
      topic_subs;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
  rclcpp::TimerBase::SharedPtr timer;
  std::vector<std::shared_ptr<MoveHandle>> moves;
  std::vector<std::shared_ptr<JointHandle>> grippers;
  std::vector<Joint::Goal> joint_goals;
  std::vector<std::string> requests;
  std::vector<Move::Goal> move_goals;
  std::string omitted_joint;
  bool move_result_has_execution{false};
  int move_error{moveit_msgs::msg::MoveItErrorCodes::SUCCESS};
  size_t fail_on_move{0};
  std::string move_error_message, move_error_source;
  bool auto_complete{true}, follow_topic{true}, auto_gripper{true};
  int cancels{0};
  int feedback_count{0};
  double gripper_position_jitter{0.0};
  std::string jitter_joint, blocked_gripper;
  std::vector<int> move_errors; // MoveIt code of each move request in order (empty: move_error)

  // A left arm joint step with an OMPL fallback after a failed PTP plan.
  Step fallback_step(int attempts) {
    Step step;
    step.name = "home_via";
    step.type = "joint";
    step.arm = "left";
    step.target = std::vector<double>(7, 0.1);
    step.timeout = 4;
    step.fallback_pipeline = attempts ? "ompl" : "";
    step.fallback_planner = attempts ? "RRTConnectkConfigDefault" : "";
    step.fallback_attempts = attempts;
    return step;
  }

  ExecutionResult run_step(TrajectoryExecutor & executor, const Step & step) {
    EXPECT_TRUE(executor.start(step));
    until([&] {
      executor.tick();
      return executor.result().state != ExecutionState::Running;
    }, 5.0);
    return executor.result();
  }

  // left_to_right with joint moved 0.02 rad once planning has started (after the planned start).
  std::string drifted_handover(const std::string & joint) {
    enable_smoothing();
    config.require_passive_gripper_stationary = false;
    TrajectoryExecutor executor(*node, config, *context);
    warmup(executor);
    HandoverMotion motion(config, *context, executor);
    std::string reason;
    if (!motion.start("left_to_right", reason)) return "not started: " + reason;
    bool drifted = false;
    until([&]() { return !motion.running(); }, 5.0, [&]() {
      if (!drifted && !move_goals.empty()) {
        positions.at(joint) += .02;
        drifted = true;
      }
      motion.tick();
    });
    return motion.result().sequence_completed ? "" : motion.result().detail;
  }
};

TEST(PlanningConfig, PlannerFallbackIsValidatedForJointStepsOnly) {
  const auto config = fixture_config("/fallback_validation");
  MotionSequence sequence;
  Step step;
  step.name = "home_via";
  step.type = "joint";
  step.arm = "left";
  step.target = std::vector<double>(7, 0.0);
  step.timeout = 5;
  step.fallback_pipeline = "ompl";
  step.fallback_planner = "RRTConnectkConfigDefault";
  step.fallback_attempts = 3;
  sequence.steps = {step};
  EXPECT_EQ(validate_motion_sequence(sequence, config), "");
  const auto with = [&](const std::function<void(Step &)> & change) {
    auto changed = sequence;
    change(changed.steps[0]);
    return validate_motion_sequence(changed, config);
  };
  EXPECT_NE(with([](Step & s) { s.fallback_attempts = 6; }), "");
  EXPECT_NE(with([](Step & s) { s.fallback_attempts = -1; }), "");
  EXPECT_NE(with([](Step & s) { s.fallback_attempts = 0; }), ""); // named but never used
  EXPECT_NE(with([](Step & s) { s.fallback_planner.clear(); }), "");
  EXPECT_NE(with([](Step & s) {
    s.type = "linear";
    s.target = {0.3, 0, 0.8, 0, 0, 0, 1};
  }), "");
}

TEST_F(RosFixture, JointStepWithoutFallbackFailsOnTheFirstPlan) {
  move_errors = {moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN};
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto result = run_step(executor, fallback_step(0));
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_EQ(result.code, "EXECUTION_FAILED");
  EXPECT_EQ(result.detail.find("plan attempts"), std::string::npos) << result.detail;
  EXPECT_EQ(move_goals.size(), 1u);
}

TEST_F(RosFixture, FailedPtpPlanFallsBackToOmplWithTheSameGoal) {
  move_errors = {moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN};
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto result = run_step(executor, fallback_step(3));
  EXPECT_EQ(result.state, ExecutionState::Succeeded) << result.detail;
  ASSERT_EQ(move_goals.size(), 2u);
  EXPECT_EQ(move_goals[0].request.planner_id, "PTP");
  EXPECT_EQ(move_goals[1].request.pipeline_id, "ompl");
  EXPECT_EQ(move_goals[1].request.planner_id, "RRTConnectkConfigDefault");
  EXPECT_EQ(move_goals[1].request.goal_constraints, move_goals[0].request.goal_constraints);
  EXPECT_EQ(move_goals[1].request.max_velocity_scaling_factor,
      move_goals[0].request.max_velocity_scaling_factor);
  EXPECT_FALSE(move_goals[1].planning_options.plan_only);
  EXPECT_NE(result.detail.find("planned with ompl/RRTConnectkConfigDefault on fallback attempt 1/3"
                               " after pilz_industrial_motion_planner/PTP: MoveIt error -2"),
      std::string::npos)
      << result.detail;
}

TEST_F(RosFixture, SecondFallbackAttemptCanSucceed) {
  move_errors = {moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN,
      moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED};
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto result = run_step(executor, fallback_step(3));
  EXPECT_EQ(result.state, ExecutionState::Succeeded) << result.detail;
  EXPECT_EQ(move_goals.size(), 3u);
  EXPECT_NE(result.detail.find("fallback attempt 2/3"), std::string::npos) << result.detail;
}

TEST_F(RosFixture, UsedUpFallbackFailsWithTheAttemptHistory) {
  move_errors = std::vector<int>(3, moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto result = run_step(executor, fallback_step(2));
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_EQ(move_goals.size(), 3u);
  EXPECT_NE(result.detail.find("plan attempts: pilz_industrial_motion_planner/PTP: MoveIt error -2,"
                               " ompl/RRTConnectkConfigDefault: MoveIt error -2,"
                               " ompl/RRTConnectkConfigDefault: MoveIt error -2"
                               " (no fallback: fallback attempts used up)"),
      std::string::npos)
      << result.detail;
}

TEST_F(RosFixture, ExecutionFailureIsNotReplanned) {
  move_errors = {moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED};
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto result = run_step(executor, fallback_step(3));
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_EQ(move_goals.size(), 1u);
  EXPECT_NE(result.detail.find("(no fallback: not a planning failure)"), std::string::npos)
      << result.detail;
}

TEST_F(RosFixture, PlanFailureWithAnExecutedTrajectoryIsNotReplanned) {
  move_errors = {moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN};
  move_result_has_execution = true;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto result = run_step(executor, fallback_step(3));
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_EQ(move_goals.size(), 1u);
  EXPECT_NE(result.detail.find("(no fallback: not a planning failure)"), std::string::npos)
      << result.detail;
}

TEST_F(RosFixture, PlanFailureAfterTheArmMovedIsNotReplanned) {
  move_errors = {moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN};
  auto_complete = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(fallback_step(3)));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return !move_goals.empty();
  }));
  positions.at("left_joint0") += .05; // moved before MoveIt answered
  until([&] {
    executor.tick();
    return false;
  }, .1);
  auto_complete = true;
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(move_goals.size(), 1u);
  EXPECT_NE(executor.result().detail.find("(no fallback: the arm moved)"), std::string::npos)
      << executor.result().detail;
}

Step paired_pose() {
  Step pose;
  pose.name = "MOVE_TO_OBSERVE";
  pose.type = "pose";
  pose.arm = "left";
  pose.position_only = true;
  pose.frame = "base_link";
  pose.target = {0.3, 0.2, 0.8, 0, 0, 0, 1};
  pose.timeout = 5;
  return pose;
}

std::vector<Step> right_home_steps() {
  Step via, home;
  via.name = "giver_home_via";
  home.name = "giver_home";
  for (auto * step : {&via, &home}) {
    step->type = "joint";
    step->arm = "right";
    step->timeout = 5;
  }
  via.target = std::vector<double>(7, 0.2);
  via.tolerance = .05;
  home.target = std::vector<double>(7, 0.3);
  home.tolerance = .01;
  return {via, home};
}

TEST_F(RosFixture, PairedApproachPlansThePoseOnceAndExecutesBothArmsTogether) {
  enable_smoothing();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_paired(paired_pose(), right_home_steps()))
      << executor.result().detail;
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }, 5.0));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  ASSERT_EQ(move_goals.size(), 1u); // the left pose only, plan-only
  EXPECT_TRUE(move_goals[0].planning_options.plan_only);
  ASSERT_EQ(execution_goals.size(), 1u);
  const auto & names = execution_goals[0].trajectory.joint_trajectory.joint_names;
  EXPECT_EQ(names.size(), 14u);
  EXPECT_EQ(names.front(), "left_joint0");
  EXPECT_EQ(names.back(), "right_joint6");
  EXPECT_NEAR(positions.at("right_joint3"), 0.3, 1e-6);
  EXPECT_NEAR(positions.at("left_joint3"), 0.1, 1e-6);
}

TEST_F(RosFixture, PairedPlanFailureIsReportedAsBeforeMotion) {
  enable_smoothing();
  move_error = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_paired(paired_pose(), right_home_steps()));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }, 5.0));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "PAIRED_BEFORE_MOTION") << executor.result().detail;
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RosFixture, PairedExecutionFailureKeepsItsCode) {
  enable_smoothing();
  execute_error = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_paired(paired_pose(), right_home_steps()));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }, 5.0));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "EXECUTION_FAILED") << executor.result().detail;
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RosFixture, PairedNeedsSmoothMotionAndTheOtherArmsJointSteps) {
  TrajectoryExecutor executor(*node, config, *context); // smooth_motion off
  warmup(executor);
  EXPECT_FALSE(executor.start_paired(paired_pose(), right_home_steps()));
  EXPECT_EQ(executor.result().code, "PAIRED_BEFORE_MOTION");
  auto same_arm = right_home_steps();
  for (auto & step : same_arm) step.arm = "left";
  EXPECT_FALSE(executor.start_paired(paired_pose(), same_arm));
  EXPECT_TRUE(move_goals.empty());
}

// Side return: right arm joint0 via then home; full speed keeps the prepared path short.
std::vector<Step> side_steps(double via, double home) {
  auto steps = right_home_steps();
  steps[0].target = std::vector<double>(7, 0.0);
  steps[0].target[0] = via;
  steps[1].target = std::vector<double>(7, 0.0);
  steps[1].target[0] = home;
  for (auto & step : steps) step.velocity_scale = step.acceleration_scale = 1.0;
  return steps;
}

Step left_joint_step(double first) {
  Step step;
  step.name = "left_move";
  step.type = "joint";
  step.arm = "left";
  step.target = std::vector<double>(7, 0.0);
  step.target[0] = first;
  step.tolerance = .01;
  step.timeout = 5;
  return step;
}

class SideFixture : public RosFixture {
protected:
  void begin(bool arm_geometry = false) {
    enable_smoothing(arm_geometry);
    // The mock plan has no timing; give the planned main path a duration.
    transform_plan = [](Move::Result & result) {
      result.planned_trajectory.joint_trajectory.points.back().time_from_start =
          rclcpp::Duration::from_seconds(.2);
    };
    executor = std::make_unique<TrajectoryExecutor>(*node, config, *context);
    warmup(*executor);
  }

  bool start_side(const std::vector<Step> & steps) {
    // Retries only while the right controller client is still discovering its server.
    return until([&] { return executor->start_side(steps); }, 2.0, tick());
  }

  std::function<void()> tick() {
    return [this] { executor->tick(); };
  }

  bool side_sent() {
    return until([&] { return !joint_goals.empty(); }, 5.0, tick());
  }

  ExecutionState side_state() const { return executor->side_result().state; }

  std::unique_ptr<TrajectoryExecutor> executor;
};

TEST_F(SideFixture, SideSendsOnlyArmJointsToItsControllerWithoutMoveGroup) {
  begin();
  ASSERT_TRUE(start_side(side_steps(.2, .3))) << executor->side_result().detail;
  ASSERT_TRUE(until([&] { return side_state() != ExecutionState::Running; }, 5.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Succeeded) << executor->side_result().detail;
  ASSERT_EQ(joint_goals.size(), 1u);
  EXPECT_EQ(joint_goals[0].trajectory.joint_names, config.arms.at("right").joints);
  EXPECT_TRUE(execution_goals.empty());
  EXPECT_TRUE(move_goals.empty());
  EXPECT_NEAR(positions.at("right_joint0"), .3, 1e-6);
  EXPECT_EQ(executor->result().state, ExecutionState::Idle); // The main slot was never used.
}

TEST_F(SideFixture, SideAlreadyHomeSendsNothing) {
  positions["right_joint0"] = .3;
  begin();
  const auto scenes = scene_requests;
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  EXPECT_EQ(side_state(), ExecutionState::Succeeded);
  EXPECT_TRUE(joint_goals.empty());
  EXPECT_EQ(scene_requests, scenes);
}

TEST_F(SideFixture, LeftMainRunsWhileTheRightSideMoves) {
  begin();
  auto_gripper = false; // The side goal stays in flight.
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(side_sent()) << executor->side_result().detail;
  positions["right_joint1"] = .05; // The right arm is moving.
  velocities["right_joint1"] = .3;
  const auto step = left_joint_step(.1);
  std::string reason;
  ASSERT_TRUE(until([&] {
    return executor->ready(MotionSequence{"", "", "", {step}, ""}, reason);
  }, 2.0, tick())) << reason;
  ASSERT_TRUE(executor->start(step)) << executor->result().detail;
  ASSERT_TRUE(until([&] {
    return executor->result().state != ExecutionState::Running;
  }, 5.0, tick()));
  EXPECT_EQ(executor->result().state, ExecutionState::Succeeded) << executor->result().detail;
  EXPECT_EQ(side_state(), ExecutionState::Running);
  ASSERT_EQ(execution_goals.size(), 1u);
  EXPECT_EQ(execution_goals[0].trajectory.joint_trajectory.joint_names,
      config.arms.at("left").joints);
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_TRUE(move_goals[0].planning_options.plan_only); // Planned first for the pair check.
  velocities["right_joint1"] = 0;
  auto_gripper = true;
  ASSERT_TRUE(until([&] { return side_state() != ExecutionState::Running; }, 5.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Succeeded) << executor->side_result().detail;
}

TEST_F(SideFixture, MainUsingTheSideArmIsRefused) {
  begin();
  auto_gripper = false;
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(side_sent());
  Step right = left_joint_step(.1);
  right.arm = "right";
  EXPECT_FALSE(executor->start(right));
  EXPECT_EQ(executor->result().code, "SIDE_ARM_BUSY");
  Step gripper;
  gripper.name = "right_open";
  gripper.type = "gripper";
  gripper.arm = "right";
  gripper.target = {0.0};
  gripper.timeout = 2;
  EXPECT_FALSE(executor->start(gripper));
  EXPECT_EQ(executor->result().code, "SIDE_ARM_BUSY");
  const auto & ready = config.motion_sequences.at("left_to_right").steps.front(); // dual_joint
  EXPECT_FALSE(executor->start_sequence({ready}));
  EXPECT_EQ(executor->result().code, "SIDE_ARM_BUSY");
  EXPECT_FALSE(executor->start_paired(paired_pose(), right_home_steps()));
  EXPECT_TRUE(move_goals.empty());
  ASSERT_EQ(joint_goals.size(), 1u); // Only the side goal reached the right controller.
  executor->cancel_side(); // cancel() stops the main operation only.
  ASSERT_TRUE(until([&] { return side_state() != ExecutionState::Cancelling; }, 2.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Cancelled);
}

TEST_F(SideFixture, MainPathMeetingTheSidePathWaitsAndReplansAfterTheSide) {
  begin(true);
  auto_gripper = false;
  // Side right_joint0 -> -0.6 -> -0.1; main left_joint0 -> 1.0 meets it at (1.0, -0.6).
  ASSERT_TRUE(start_side(side_steps(-.6, -.1))) << executor->side_result().detail;
  ASSERT_TRUE(side_sent()) << executor->side_result().detail;
  ASSERT_TRUE(executor->start(left_joint_step(1.0))) << executor->result().detail;
  ASSERT_TRUE(until([&] { return executor->result().phase == "WAIT_SIDE_ARM"; }, 5.0, tick()))
      << executor->result().detail;
  EXPECT_EQ(executor->result().state, ExecutionState::Running);
  EXPECT_TRUE(execution_goals.empty()); // Nothing sent while the paths collide.
  auto_gripper = true;
  ASSERT_TRUE(until([&] {
    return executor->result().state != ExecutionState::Running;
  }, 5.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Succeeded);
  EXPECT_EQ(executor->result().state, ExecutionState::Succeeded) << executor->result().detail;
  ASSERT_EQ(move_goals.size(), 2u); // Planned again from the start after the side arrived.
  EXPECT_FALSE(move_goals[1].planning_options.plan_only);
  EXPECT_NEAR(positions.at("left_joint0"), 1.0, 1e-6);
}

TEST_F(SideFixture, SideStartCollidingWithTheOtherArmSendsNothing) {
  positions["left_joint0"] = 1.2; // The right path crosses -0.37 where the boxes meet.
  begin(true);
  ASSERT_TRUE(start_side(side_steps(-.5, -.4)));
  ASSERT_TRUE(until([&] { return side_state() != ExecutionState::Running; }, 5.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Failed);
  EXPECT_TRUE(executor->side_result().retryable_before_motion);
  EXPECT_NE(executor->side_result().detail.find("collision"), std::string::npos)
      << executor->side_result().detail;
  EXPECT_TRUE(joint_goals.empty());
}

TEST_F(SideFixture, SideFailureAfterMotionCancelsMainAndSideOutlivesMainFailure) {
  begin();
  auto_gripper = false;
  auto_execute = false;
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(side_sent());
  ASSERT_TRUE(executor->start(left_joint_step(.1)));
  ASSERT_TRUE(until([&] { return !executions.empty(); }, 5.0, tick()));
  ASSERT_FALSE(grippers.empty());
  auto aborted = std::make_shared<Joint::Result>();
  aborted->error_code = Joint::Result::PATH_TOLERANCE_VIOLATED;
  grippers.back()->abort(aborted);
  grippers.pop_back();
  ASSERT_TRUE(until([&] {
    return executor->result().state != ExecutionState::Running &&
           executor->result().state != ExecutionState::Cancelling &&
           side_state() != ExecutionState::Cancelling;
  }, 5.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Failed);
  EXPECT_FALSE(executor->side_result().retryable_before_motion);
  EXPECT_EQ(executor->result().code, "SIDE_ARM_FAILED") << executor->result().detail;

  // A new side keeps going through a main failure and a main cancel, then arrives.
  positions["right_joint0"] = 0;
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(until([&] { return joint_goals.size() == 2; }, 5.0, tick()));
  move_error = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  ASSERT_TRUE(executor->start(left_joint_step(.1)));
  ASSERT_TRUE(until([&] {
    return executor->result().state == ExecutionState::Failed;
  }, 5.0, tick()));
  until([] { return false; }, .1, tick()); // Keep ticking: the side must not be cancelled.
  EXPECT_EQ(side_state(), ExecutionState::Running);
  move_error = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  ASSERT_TRUE(executor->start(left_joint_step(.1)));
  ASSERT_TRUE(until([&] { return executions.size() == 1; }, 5.0, tick()));
  executor->cancel();
  ASSERT_TRUE(until([&] {
    return executor->result().state == ExecutionState::Cancelled;
  }, 2.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Running);
  auto_gripper = true;
  ASSERT_TRUE(until([&] { return side_state() != ExecutionState::Running; }, 5.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Succeeded) << executor->side_result().detail;
  EXPECT_NEAR(positions.at("right_joint0"), .3, 1e-6);
}

TEST_F(SideFixture, ExecutorCancelStopsMainOnlyAndAnUnconfirmedSideStopLatches) {
  begin();
  auto_gripper = false;
  auto_execute = false;
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(side_sent());
  ASSERT_TRUE(executor->start(left_joint_step(.1)));
  ASSERT_TRUE(until([&] { return !executions.empty(); }, 5.0, tick()));
  executor->cancel();
  ASSERT_TRUE(until([&] {
    return executor->result().state == ExecutionState::Cancelled;
  }, 2.0, tick()));
  EXPECT_EQ(side_state(), ExecutionState::Running); // ExecutorPort::cancel is main only.
  executor->cancel_side();
  ASSERT_TRUE(until([&] { return side_state() == ExecutionState::Cancelled; }, 2.0, tick()));

  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(until([&] { return joint_goals.size() == 2; }, 5.0, tick()));
  executor->cancel_side();
  velocities["right_joint0"] = .5; // The right arm never reports a stop.
  ASSERT_TRUE(until([&] { return side_state() == ExecutionState::StopUnconfirmed; }, 2.0, tick()));
  EXPECT_EQ(executor->result().state, ExecutionState::StopUnconfirmed);
  EXPECT_FALSE(executor->start(left_joint_step(.1)));
}

TEST_F(SideFixture, MainStopUnconfirmedCancelsTheSide) {
  begin();
  auto_gripper = false;
  auto_execute = false;
  ASSERT_TRUE(start_side(side_steps(.2, .3)));
  ASSERT_TRUE(side_sent());
  ASSERT_TRUE(executor->start(left_joint_step(.1)));
  ASSERT_TRUE(until([&] { return !executions.empty(); }, 5.0, tick()));
  executor->cancel();
  velocities["left_joint0"] = .5; // The left arm never reports a stop.
  ASSERT_TRUE(until([&] {
    return executor->result().state == ExecutionState::StopUnconfirmed &&
           side_state() != ExecutionState::Running && side_state() != ExecutionState::Cancelling;
  }, 2.0, tick()));
  EXPECT_EQ(executor->side_result().code, "MAIN_STOP_UNCONFIRMED");
}

TEST_F(RosFixture, HeldGripperDriftFailsAtTheDefaultToleranceWithNumbers) {
  // The giver's gripper is held through face_each_other + open_approach (receiver opens).
  const auto held = config.arms.at(config.motion_sequences.at("left_to_right").giver).gripper_joint;
  const auto detail = drifted_handover(held);
  EXPECT_NE(detail.find("Robot moved during "), std::string::npos) << detail; // either check
  EXPECT_NE(detail.find(held + " (planned 0.200, measured 0.220, tolerance 0.010)"),
      std::string::npos)
      << detail;
}

TEST_F(RosFixture, HeldGripperDriftPassesWithItsOwnTolerance) {
  config.held_gripper_start_tolerance = .05;
  const auto held = config.arms.at(config.motion_sequences.at("left_to_right").giver).gripper_joint;
  EXPECT_EQ(drifted_handover(held), "");
}

TEST_F(RosFixture, ArmDriftStillFailsWithAWideHeldGripperTolerance) {
  config.held_gripper_start_tolerance = .05;
  const auto joint = config.arms.at(config.motion_sequences.at("left_to_right").giver).joints[0];
  const auto detail = drifted_handover(joint);
  EXPECT_NE(detail.find("Robot moved during "), std::string::npos) << detail;
  EXPECT_NE(detail.find(joint + " (planned 0.000, measured 0.020, tolerance 0.010)"),
      std::string::npos)
      << detail;
}

TEST_F(RosFixture, BodyJointGroupUsesMoveItAndChecksMeasuredCompletion) {
  positions["head_joint1"] = 0;
  positions["head_joint2"] = 0;
  refresh();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  Step step;
  step.name = "head_observe";
  step.type = "joint_group";
  step.group = "head";
  step.joint_names = {"head_joint1", "head_joint2"};
  step.target = {.69, 0};
  step.timeout = 2;
  step.tolerance = .02;
  omitted_joint = "head_joint1";
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return !move_goals.empty(); }, 1, [&]() { executor.tick(); }));
  const auto & request = move_goals.front().request;
  EXPECT_EQ(request.group_name, "head");
  EXPECT_EQ(request.pipeline_id, "ompl");
  EXPECT_EQ(request.goal_constraints.front().joint_constraints.size(), 2u);
  EXPECT_FALSE(until([&]() { return executor.result().state == ExecutionState::Succeeded; },
      .08, [&]() { executor.tick(); }));
  positions["head_joint1"] = .69;
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Succeeded; },
      1, [&]() { executor.tick(); }));
  EXPECT_TRUE(grippers.empty());
}

TEST_F(RosFixture, BodyJointGroupRejectsMissingFeedbackBeforeDispatch) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  Step step;
  step.name = "lift_observe";
  step.type = "joint_group";
  step.group = "lift";
  step.joint_names = {"lift_joint"};
  step.target = {-.1};
  EXPECT_FALSE(executor.start(step));
  EXPECT_NE(executor.result().detail.find("lift_joint"), std::string::npos);
  EXPECT_TRUE(move_goals.empty());
}

TEST_F(RosFixture, RealActionClientsExecuteBothDirectionsSerially) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  for (const std::string direction : {"left_to_right", "right_to_left"}) {
    requests.clear();
    std::string reason;
    ASSERT_TRUE(motion.start(direction, reason)) << reason;
    ASSERT_TRUE(until(
        [&]() {
          return !motion.running();
        },
        3.0,
        [&]() {
          motion.tick();
        }));
    EXPECT_TRUE(motion.result().sequence_completed) << motion.result().detail;
    EXPECT_FALSE(motion.result().handover_confirmed);
    const auto & sequence = config.motion_sequences.at(direction);
    EXPECT_EQ(requests,
        (std::vector<std::string>{"move:" + sequence.giver + ":PTP",
            "gripper:" + sequence.receiver,
            "gripper:" + sequence.giver,
            "move:" + sequence.giver + ":PTP"}));
    for (const auto & goal : move_goals) {
      EXPECT_EQ(goal.request.num_planning_attempts, 1); // Pilz stays single-shot.
    }
  }
}

TEST_F(RosFixture, FiveStagesCoordinateBothArmsAndGripperInOnePlan) {
  configure_five_step_sequences(config);
  velocities["right_gripper"] = 0.214885;
  velocities["left_gripper"] = -0.083566;
  gripper_position_jitter = 0.0030679615757712823;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  for (const auto & direction : {"left_to_right", "right_to_left"}) {
    requests.clear();
    move_goals.clear();
    std::string reason;
    ASSERT_TRUE(motion.start(direction, reason)) << reason;
    ASSERT_TRUE(until(
        [&]() {
          return !motion.running();
        },
        3.0,
        [&]() {
          motion.tick();
        }));
    ASSERT_TRUE(motion.result().sequence_completed) << motion.result().detail;
    EXPECT_FALSE(motion.result().handover_confirmed);
    const auto & sequence = config.motion_sequences.at(direction);
    const auto move_request = "move:" + config.coordinated_group + ":" + config.coordinated_planner;
    EXPECT_EQ(requests,
        (std::vector<std::string>{move_request,
            move_request,
            "gripper:" + sequence.receiver,
            "gripper:" + sequence.giver,
            move_request}));
    ASSERT_EQ(move_goals.size(), 3u);
    for (const auto & goal : move_goals) {
      EXPECT_EQ(goal.request.goal_constraints[0].joint_constraints.size(), 16u);
      EXPECT_EQ(goal.request.pipeline_id, config.coordinated_pipeline);
      EXPECT_EQ(goal.request.num_planning_attempts, 3);
    }
    EXPECT_EQ(move_goals[0].request.path_constraints.joint_constraints.size(), 2u);
    ASSERT_EQ(move_goals[1].request.path_constraints.joint_constraints.size(), 1u);
    for (const auto & joint : move_goals[1].request.path_constraints.joint_constraints) {
      const auto & giver = config.arms.at(sequence.giver);
      EXPECT_EQ(joint.joint_name, giver.gripper_joint);
      EXPECT_LE(joint.tolerance_above, 0.001);
    }
    std::map<std::string, double> target;
    for (const auto & joint : move_goals[1].request.goal_constraints[0].joint_constraints) {
      target[joint.joint_name] = joint.position;
    }
    EXPECT_DOUBLE_EQ(target.at(config.arms.at(sequence.receiver).gripper_joint), 0.0);
    for (const auto & joint : move_goals[1].request.goal_constraints[0].joint_constraints) {
      const bool gripper =
          joint.joint_name == "left_gripper" || joint.joint_name == "right_gripper";
      EXPECT_DOUBLE_EQ(joint.tolerance_above, gripper ? config.gripper_goal_tolerance : 0.0005);
    }
    for (const auto & side : {"left", "right"}) {
      for (const auto & name : config.arms.at(side).joints) {
        EXPECT_DOUBLE_EQ(target.at(name), std::string(side) == "left" ? 0.2 : -0.2);
      }
    }
    EXPECT_EQ(motion.result().last_completed_step, "retreat_together");
  }
}

TEST_F(RosFixture, DualArmStepWaitsForBothArmsMeasuredArrival) {
  configure_five_step_sequences(config);
  omitted_joint = config.arms.at("right").joints.back();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(config.motion_sequences.at("left_to_right").steps[0]));
  ASSERT_TRUE(until(
      [&]() {
        return !move_goals.empty() && moves.empty();
      },
      1.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_FALSE(until(
      [&]() {
        return executor.result().state == ExecutionState::Succeeded;
      },
      0.08,
      [&]() {
        executor.tick();
      }));
  positions.at(omitted_joint) = -0.1;
  EXPECT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Succeeded;
      },
      1.0,
      [&]() {
        executor.tick();
      }));
}

TEST_F(ContextFixture, FiveStageApproachFailureCannotCloseOrRelease) {
  configure_five_step_sequences(config);
  FakeExecutor executor;
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("right_to_left", reason));
  motion.tick();
  executor.current.state = ExecutionState::Succeeded;
  motion.tick();
  motion.tick();
  ASSERT_EQ(executor.steps.back().name, "open_approach");
  executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "coordinated plan failed"};
  motion.tick();
  motion.tick();
  EXPECT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(motion.result().last_completed_step, "face_each_other");
}

TEST_F(ContextFixture, InvalidCoordinatedTargetsAreRejectedBeforeSending) {
  configure_five_step_sequences(config);
  auto sequence = config.motion_sequences.at("left_to_right");
  sequence.steps[0].right_target.pop_back();
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
  sequence = config.motion_sequences.at("left_to_right");
  sequence.steps[1].gripper_target = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
  sequence = config.motion_sequences.at("left_to_right");
  sequence.steps[1].left_target.clear();
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
  sequence = config.motion_sequences.at("left_to_right");
  sequence.steps[1].arm = "unknown";
  EXPECT_FALSE(validate_motion_sequence(sequence, config).empty());
}

TEST_F(RosFixture, OnlyTimeoutWithoutTrajectoryIsRetryablePlanningFailure) {
  move_error = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  Step step;
  step.name = "timeout_test";
  step.type = "joint";
  step.arm = "left";
  step.target = std::vector<double>(7, 0.0);
  step.timeout = 2;
  for (bool execution : {false, true}) {
    move_result_has_execution = execution;
    ASSERT_TRUE(executor.start(step));
    ASSERT_TRUE(until([&] {
      executor.tick();
      return executor.result().state != ExecutionState::Running;
    }));
    EXPECT_EQ(executor.result().code, execution ? "EXECUTION_FAILED" : "PLANNING_TIMEOUT");
  }
}

TEST_F(RosFixture, MoveItPayloadFailurePreventsGripperCommands) {
  move_error = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason));
  ASSERT_TRUE(until(
      [&]() {
        return !motion.running();
      },
      2.0,
      [&]() {
        motion.tick();
      }));
  EXPECT_EQ(motion.result().state, "FAILED");
  EXPECT_EQ(requests.size(), 1u);
}

TEST_F(RosFixture, TopicDeliveryDoesNotCountAsReachingGripperTarget) {
  config.gripper_transport = "topic";
  follow_topic = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  auto step = config.motion_sequences.at("left_to_right").steps[1];
  step.timeout = 0.12;
  config.cancel_timeout = 0.4;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Failed;
      },
      1.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_EQ(executor.result().code, "TIMEOUT");
  EXPECT_NE(executor.result().detail.find("right_gripper"), std::string::npos);
  EXPECT_GE(requests.size(), 2u); // original trajectory plus the measured-position hold
}

TEST_F(RosFixture, GripperWaitOnlySkipsClosureButWaitsForDuration) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  timer->cancel(); // Deliberately keep measured fingers short of the target.
  auto step = config.motion_sequences.at("left_to_right").steps[1];
  step.gripper_wait_only = true;
  step.duration = 0.2;
  step.timeout = 1.0;
  const auto started = Clock::now();
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return !grippers.empty(); }));
  const auto & tolerance = grippers.front()->get_goal()->goal_tolerance.at(0);
  EXPECT_EQ(tolerance.position, -1.0);
  EXPECT_EQ(tolerance.velocity, -1.0);
  EXPECT_EQ(tolerance.acceleration, -1.0);
  auto result = std::make_shared<Joint::Result>();
  result->error_code = Joint::Result::SUCCESSFUL;
  grippers.front()->succeed(result);
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Succeeded; },
      1.0, [&]() { refresh(); executor.tick(); }));
  EXPECT_GE(elapsed(started), step.duration);
  EXPECT_DOUBLE_EQ(context->snapshot().joints.at("right_gripper").position, 0.2);
}

TEST_F(RosFixture, GripperWaitOnlyStillRejectsControllerErrors) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  timer->cancel();
  auto step = config.motion_sequences.at("left_to_right").steps[1];
  step.gripper_wait_only = true;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return !grippers.empty(); }));
  auto result = std::make_shared<Joint::Result>();
  result->error_code = Joint::Result::PATH_TOLERANCE_VIOLATED;
  grippers.front()->abort(result);
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Failed; },
      1.0, [&]() { refresh(); executor.tick(); }));
  EXPECT_EQ(executor.result().code, "EXECUTION_FAILED");
}

TEST_F(RosFixture, GripperWaitOnlyCanCancelBeforeAcceptance) {
  auto_gripper = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  auto step = config.motion_sequences.at("left_to_right").steps[1];
  step.gripper_wait_only = true;
  ASSERT_TRUE(executor.start(step));
  executor.cancel();
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Cancelled; },
      2.0, [&]() { executor.tick(); }));
  EXPECT_EQ(cancels, 1);
}

TEST_F(RosFixture, ActionSuccessWithoutMeasuredTargetExplainsTimeout) {
  omitted_joint = "left_joint0";
  config.cancel_timeout = 0.4;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  auto step = config.motion_sequences.at("left_to_right").steps[0];
  step.timeout = 0.12;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Failed;
      },
      1.0,
      [&]() {
        executor.tick();
      }));
  const auto result = executor.result();
  EXPECT_EQ(result.code, "TIMEOUT");
  EXPECT_NE(result.detail.find("Joint target not reached: left_joint0"), std::string::npos);
  EXPECT_NE(result.detail.find("error=0.100000, tolerance=0.010000"), std::string::npos);
  EXPECT_NE(result.detail.find("action_terminal=true, action_success=true"), std::string::npos);
  EXPECT_EQ(requests.size(), 1u);
}

TEST_F(RosFixture, PrecisePlanningStillAcceptsMeasuredErrorWithinOneDegree) {
  omitted_joint = "left_joint0";
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  auto step = config.motion_sequences.at("left_to_right").steps[0];
  step.target.assign(step.target.size(), .01);
  step.tolerance = 0.017453292519943295;
  step.planning_tolerance = .0005;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Succeeded;
      },
      1.0,
      [&]() {
        executor.tick();
      }));
  ASSERT_EQ(move_goals.size(), 1u);
  for (const auto & constraint : move_goals[0].request.goal_constraints[0].joint_constraints) {
    EXPECT_DOUBLE_EQ(constraint.tolerance_above, .0005);
  }
}

TEST_F(RosFixture, MissingActionResultExplainsTimeout) {
  auto_complete = false;
  config.cancel_timeout = 0.4;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  auto step = config.motion_sequences.at("left_to_right").steps[0];
  step.timeout = 0.12;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Failed;
      },
      1.0,
      [&]() {
        executor.tick();
      }));
  const auto result = executor.result();
  EXPECT_EQ(result.code, "TIMEOUT");
  EXPECT_NE(result.detail.find("Waiting for controller/action result"), std::string::npos);
  EXPECT_NE(result.detail.find("action_terminal=false, action_success=false"), std::string::npos);
}

TEST_F(RosFixture, TopicModeUsesMeasuredFeedbackToFinish) {
  config.gripper_transport = "topic";
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(config.motion_sequences.at("left_to_right").steps[1]));
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Succeeded;
      },
      1.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_DOUBLE_EQ(context->snapshot().joints.at("right_gripper").position, 0.4);
}

TEST_F(RosFixture, CancellationBeforeGoalAcceptanceCancelsLateGoal) {
  auto_complete = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(config.motion_sequences.at("left_to_right").steps[0]));
  executor.cancel(); // no callbacks have yet delivered the remote goal handle
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Cancelled;
      },
      2.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_EQ(cancels, 1);
}

TEST_F(RosFixture, UnconfirmedStopStaysLatchedAfterLateCancellationResult) {
  auto_complete = false;
  config.cancel_timeout = 0.03;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto step = config.motion_sequences.at("left_to_right").steps[0];
  ASSERT_TRUE(executor.start(step));
  executor.cancel();
  std::this_thread::sleep_for(40ms); // deliberately do not spin the remote server
  refresh();
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::StopUnconfirmed);
  ASSERT_TRUE(until(
      [&]() {
        return cancels == 1 && moves.empty();
      },
      2.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_EQ(executor.result().state, ExecutionState::StopUnconfirmed);
  EXPECT_FALSE(executor.start(step));
}

TEST_F(RosFixture, TaskActionRejectsOverlapAndCancelsOnlyItsActiveExecution) {
  auto_complete = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  TaskManager manager(*node, *context, motion);
  auto client = rclcpp_action::create_client<demo_1f::action::Handover>(mock, "/handover");
  ASSERT_TRUE(until([&]() {
    return client->action_server_is_ready();
  }));
  // Server construction/discovery can interrupt the short feedback window.
  warmup(executor);
  demo_1f::action::Handover::Goal request;
  request.direction = "left_to_right";
  auto first = client->async_send_goal(request);
  ASSERT_TRUE(until([&]() {
    return first.wait_for(0s) == std::future_status::ready;
  }));
  const auto handle = first.get();
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(until([&]() {
    return !moves.empty();
  }));
  auto second = client->async_send_goal(request);
  ASSERT_TRUE(until([&]() {
    return second.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_EQ(second.get(), nullptr);
  auto result = client->async_get_result(handle);
  client->async_cancel_goal(handle);
  ASSERT_TRUE(until([&]() {
    return result.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(result.get().result->sequence_completed);
  EXPECT_EQ(requests.size(), 1u);
}

TEST_F(RosFixture, PoseTargetUsesPtpAndMeasuredTcpInsteadOfJointTargets) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  tf2_ros::StaticTransformBroadcaster broadcaster(mock);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = config.planning_frame;
  tf.child_frame_id = config.arms.at("right").tcp;
  tf.header.stamp = node->now();
  tf.transform.translation.x = .4;
  tf.transform.translation.y = -.2;
  tf.transform.translation.z = .8;
  tf.transform.rotation.w = 1;
  broadcaster.sendTransform(tf);
  ASSERT_TRUE(until([&] {
    return context->pose_of(tf.child_frame_id).has_value();
  }));
  Step step;
  step.name = "cup_observe";
  step.type = "pose";
  step.arm = "right";
  step.frame = config.planning_frame;
  step.target = {.4, -.2, .8, 0, 0, 0, 1};
  step.timeout = 1;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_EQ(move_goals[0].request.planner_id, "PTP");
  EXPECT_EQ(move_goals[0].request.goal_constraints[0].orientation_constraints.size(), 1u);
  EXPECT_TRUE(move_goals[0].request.goal_constraints[0].joint_constraints.empty());
  EXPECT_EQ(move_goals[0].request.goal_constraints[0].position_constraints.size(), 1u);
}

TEST_F(RosFixture, TiltBoundsConstrainOnlyGoalAndRequireMeasuredArrivalTilt) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  tf2_ros::StaticTransformBroadcaster broadcaster(mock);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = config.planning_frame;
  tf.child_frame_id = config.arms.at("left").tcp;
  tf.transform.translation.x = .4;
  tf.transform.translation.y = .2;
  tf.transform.translation.z = .8;
  auto send_tilt = [&](double roll, double pitch, double yaw = -45) {
    tf2::Quaternion hand, tcp_axes;
    hand.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);
    tcp_axes.setRPY(0, -M_PI / 2, 0);
    const auto q = hand * tcp_axes;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf.header.stamp = node->now();
    broadcaster.sendTransform(tf);
    ASSERT_TRUE(until([&] {
      const auto current = context->pose_of(tf.child_frame_id);
      if (!current) {
        return false;
      }
      const auto & actual = current->pose.orientation;
      // TF may return the equivalent quaternion -q.
      return std::abs(actual.x * q.x() + actual.y * q.y() + actual.z * q.z() + actual.w * q.w()) >
             1 - 1e-10;
    }));
  };
  // Starting outside the goal tilt band must still allow a planning request.
  send_tilt(60, 20);
  Step step;
  step.name = "cup_goal_tilt";
  step.type = "pose";
  step.position_only = true;
  step.arm = "left";
  step.frame = config.planning_frame;
  step.target = {.4, .2, .8, 0, 0, 0, 1};
  step.tcp_roll_limit = 10 * M_PI / 180;
  step.tcp_pitch_min = 20 * M_PI / 180;
  step.tcp_pitch_limit = 60 * M_PI / 180;
  step.tcp_pitch_downward_only = true;
  step.tcp_yaw_inward_only = true;
  step.timeout = 3;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return !move_goals.empty() && moves.empty();
  }));
  const auto & request = move_goals[0].request;
  EXPECT_EQ(request.pipeline_id, "ompl");
  EXPECT_TRUE(request.path_constraints.orientation_constraints.empty());
  ASSERT_EQ(request.goal_constraints.size(), 1u);
  const auto & goal = request.goal_constraints[0];
  EXPECT_EQ(goal.position_constraints.size(), 2u);
  ASSERT_EQ(goal.orientation_constraints.size(), 3u);
  const auto & band = goal.orientation_constraints[0];
  EXPECT_EQ(band.header.frame_id, config.arms.at("left").tcp);
  EXPECT_EQ(band.link_name, config.planning_frame);
  EXPECT_NEAR(band.absolute_x_axis_tolerance, 9 * M_PI / 180, 1e-9);
  EXPECT_NEAR(band.absolute_y_axis_tolerance, 59 * M_PI / 180, 1e-9);
  EXPECT_NEAR(band.absolute_z_axis_tolerance, M_PI, 1e-9);
  // A successful controller result and correct XYZ cannot bypass measured tilt checks.
  for (const auto & tilt :
      std::vector<std::pair<double, double>>{{60, 40}, {0, 65}, {0, -20}, {0, 10}}) {
    send_tilt(tilt.first, tilt.second);
    EXPECT_FALSE(until(
        [&] {
          executor.tick();
          return executor.result().state != ExecutionState::Running;
        },
        .08));
  }
  for (double yaw : {20., -100.}) {
    send_tilt(0, 40, yaw);
    EXPECT_FALSE(until(
        [&] {
          executor.tick();
          return executor.result().state != ExecutionState::Running;
        },
        .08));
  }
  send_tilt(0, 40);
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
}

TEST_F(RosFixture, FlexiblePoseUsesOmplCorridorAndAcceptsSmallRotation) {
  enable_smoothing(); // Provide the separate trajectory execution action.
  config.smooth_motion = false;
  config.ompl_planning_attempts = 4;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  tf2_ros::StaticTransformBroadcaster broadcaster(mock);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = config.planning_frame;
  tf.child_frame_id = config.arms.at("left").tcp;
  tf.header.stamp = node->now();
  tf.transform.translation.x = .4;
  tf.transform.translation.y = .2;
  tf.transform.translation.z = .8;
  tf.transform.rotation.x = std::sin(5 * M_PI / 360);
  tf.transform.rotation.w = std::cos(5 * M_PI / 360);
  broadcaster.sendTransform(tf);
  ASSERT_TRUE(until([&] {
    return context->pose_of(tf.child_frame_id).has_value();
  }));
  Step step;
  step.name = "cup_position";
  step.type = "pose";
  step.orientation_slack = 10 * M_PI / 180;
  step.tcp_path_margin = .02;
  step.tcp_path_min_z = .795;
  step.planning_time = 3;
  step.arm = "left";
  step.frame = config.planning_frame;
  step.target = {.4, .2, .8, 0, 0, 0, 1};
  step.timeout = 5;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  ASSERT_EQ(move_goals.size(), 1u);
  const auto & request = move_goals[0].request;
  EXPECT_EQ(request.group_name, config.arms.at("left").group);
  EXPECT_EQ(request.pipeline_id, "ompl");
  EXPECT_EQ(request.planner_id, "RRTConnectkConfigDefault");
  ASSERT_EQ(request.goal_constraints.size(), 1u);
  EXPECT_EQ(request.goal_constraints[0].position_constraints.size(), 1u);
  EXPECT_DOUBLE_EQ(request.goal_constraints[0]
                       .position_constraints[0]
                       .constraint_region.primitives[0]
                       .dimensions[0],
      config.pose_position_tolerance);
  ASSERT_EQ(request.goal_constraints[0].orientation_constraints.size(), 1u);
  const auto & orientation = request.goal_constraints[0].orientation_constraints[0];
  EXPECT_EQ(orientation.parameterization, moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR);
  EXPECT_NEAR(orientation.absolute_x_axis_tolerance, 9 * M_PI / 180, 1e-9);
  ASSERT_EQ(request.path_constraints.position_constraints.size(), 1u);
  EXPECT_NEAR(request.path_constraints.position_constraints[0]
                  .constraint_region.primitives[0]
                  .dimensions[0],
      .04,
      1e-9);
  EXPECT_DOUBLE_EQ(request.allowed_planning_time, 3);
  EXPECT_EQ(request.num_planning_attempts, 4); // One shared 3 s budget, not 4 x 3 s.
  const auto & box = request.path_constraints.position_constraints[0].constraint_region;
  EXPECT_NEAR(box.primitives[0].dimensions[1], .04, 1e-9);
  EXPECT_NEAR(box.primitives[0].dimensions[2], .025, 1e-9);
  EXPECT_NEAR(box.primitive_poses[0].position.z - box.primitives[0].dimensions[2] / 2, .795, 1e-9);
  EXPECT_TRUE(request.goal_constraints[0].joint_constraints.empty());
  EXPECT_TRUE(request.path_constraints.orientation_constraints.empty());
  step.planning_time = 0;
  step.timeout = 1;
  step.target[3] = std::sin(40 * M_PI / 360);
  step.target[6] = std::cos(40 * M_PI / 360);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until(
      [&] {
        executor.tick();
        return executor.result().state != ExecutionState::Running;
      },
      2.0));
  EXPECT_NE(executor.result().state, ExecutionState::Succeeded);
}

TEST_F(RosFixture, PositionOnlyPoseUsesOmplAndCompletesAtDifferentOrientation) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  tf2_ros::StaticTransformBroadcaster broadcaster(mock);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = config.planning_frame;
  tf.child_frame_id = config.arms.at("left").tcp;
  tf.header.stamp = node->now();
  tf.transform.translation.x = .4;
  tf.transform.translation.y = .2;
  tf.transform.translation.z = .8;
  // A 180-degree difference must not block a position-only completion.
  tf.transform.rotation.x = 1;
  broadcaster.sendTransform(tf);
  ASSERT_TRUE(until([&] {
    return context->pose_of(tf.child_frame_id).has_value();
  }));
  Step step;
  step.name = "cup_position";
  step.type = "pose";
  step.position_only = true;
  step.arm = "left";
  step.frame = config.planning_frame;
  step.target = {.4, .2, .8, 0, 0, 0, 1};
  step.timeout = 1;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&] {
    executor.tick();
    return executor.result().state != ExecutionState::Running;
  }));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  ASSERT_EQ(move_goals.size(), 1u);
  const auto & request = move_goals[0].request;
  EXPECT_EQ(request.group_name, config.arms.at("left").group);
  EXPECT_EQ(request.pipeline_id, "ompl");
  EXPECT_EQ(request.planner_id, "RRTConnectkConfigDefault");
  EXPECT_EQ(request.num_planning_attempts, 3);
  ASSERT_EQ(request.goal_constraints.size(), 1u);
  EXPECT_EQ(request.goal_constraints[0].position_constraints.size(), 1u);
  EXPECT_DOUBLE_EQ(request.goal_constraints[0]
                       .position_constraints[0]
                       .constraint_region.primitives[0]
                       .dimensions[0],
      config.pose_position_tolerance * .5);
  EXPECT_TRUE(request.goal_constraints[0].orientation_constraints.empty());
  EXPECT_TRUE(request.goal_constraints[0].joint_constraints.empty());
  EXPECT_TRUE(request.path_constraints.orientation_constraints.empty());
  // Free orientation must not mean free position: offset XYZ still fails completion.
  step.target[0] += .1;
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until(
      [&] {
        executor.tick();
        return executor.result().state != ExecutionState::Running;
      },
      2.0));
  EXPECT_NE(executor.result().state, ExecutionState::Succeeded);
  // Other step types must not silently lose their constraints.
  step.type = "linear";
  MotionSequence invalid;
  invalid.steps = {step};
  EXPECT_FALSE(validate_motion_sequence(invalid, config).empty());
}

} // namespace
} // namespace demo_1f

namespace demo_1f {
namespace {
TEST_F(RosFixture, ContinuousPlansAreReadOnlyUntilOneCombinedExecution) {
  enable_smoothing();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason)) << reason;
  ASSERT_TRUE(until(
      [&]() {
        return !motion.running();
      },
      5.0,
      [&]() {
        motion.tick();
      }));
  EXPECT_TRUE(motion.result().sequence_completed) << motion.result().detail;
  ASSERT_EQ(move_goals.size(), 3u);
  for (const auto & goal : move_goals) {
    EXPECT_TRUE(goal.planning_options.plan_only);
    EXPECT_EQ(goal.request.num_planning_attempts, 3);
  }
  ASSERT_EQ(execution_goals.size(), 2u);
  EXPECT_EQ(execution_goals[0].trajectory.joint_trajectory.joint_names.size(), 16u);
  EXPECT_EQ(requests,
      (std::vector<std::string>{"move:handover_arms:RRTConnectkConfigDefault",
          "move:handover_arms:RRTConnectkConfigDefault",
          "execute:path",
          "gripper:right",
          "gripper:left",
          "move:handover_arms:RRTConnectkConfigDefault",
          "execute:path"}));
}

TEST_F(RosFixture, CupPreparationOmplDetourIsRejectedBeforeAnyTrajectoryIsSent) {
  enable_smoothing();
  auto steps = config.motion_sequences.at("left_to_right").steps;
  steps.resize(2);
  steps[1].type = "dual_joint";
  steps[1].gripper_target.clear();
  steps[1].left_target = steps[0].left_target;
  steps[1].hold_arm = "left";
  steps[1].hold_tolerance = .001;
  for (auto & step : steps) {
    step.pipeline = "ompl";
    step.planner = "RRTConnectkConfigDefault";
    step.velocity_scale = step.acceleration_scale = .3;
  }
  transform_plan = [this](Move::Result & result) {
    if (move_goals.size() != 2) return;
    auto & path = result.planned_trajectory.joint_trajectory;
    const auto found = std::find(path.joint_names.begin(), path.joint_names.end(),
        config.arms.at("left").joints.front());
    ASSERT_NE(found, path.joint_names.end());
    auto detour = path.points.front();
    detour.positions.at(found - path.joint_names.begin()) += .02;
    path.points.insert(path.points.begin() + 1, detour);
  };
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(steps));
  ASSERT_TRUE(until([&] { return executor.result().state != ExecutionState::Running; }, 5.,
      [&] { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_NE(executor.result().detail.find("corridor"), std::string::npos);
  EXPECT_TRUE(execution_goals.empty());
  ASSERT_EQ(move_goals.size(), 2u);
  EXPECT_EQ(move_goals[1].request.pipeline_id, "ompl");
  EXPECT_EQ(move_goals[1].request.planner_id, "RRTConnectkConfigDefault");
  int held_left = 0;
  for (const auto & constraint : move_goals[1].request.path_constraints.joint_constraints) {
    if (std::find(config.arms.at("left").joints.begin(), config.arms.at("left").joints.end(),
            constraint.joint_name) == config.arms.at("left").joints.end()) continue;
    ++held_left;
    EXPECT_DOUBLE_EQ(constraint.tolerance_above, .001);
    EXPECT_DOUBLE_EQ(constraint.tolerance_below, .001);
  }
  EXPECT_EQ(held_left, 7);
}

TEST_F(RosFixture, CupPreparationJointLineSkipsMoveGroupAndKeepsTheHeldArm) {
  enable_smoothing();
  for (const auto & name : config.arms.at("left").joints) positions.at(name) = .1;
  auto steps = config.motion_sequences.at("left_to_right").steps;
  steps.resize(2);
  steps[0].blend_with_next = true;
  steps[1].name = "cup_ready";
  steps[1].type = "dual_joint";
  steps[1].arm.clear();
  steps[1].gripper_target.clear();
  steps[1].left_target = steps[0].left_target;
  steps[1].hold_arm = "left";
  steps[1].hold_tolerance = .001;
  for (auto & step : steps) {
    step.joint_line = true;
    step.pipeline.clear();
    step.planner.clear();
    step.velocity_scale = step.acceleration_scale = .3;
  }
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(steps));
  ASSERT_TRUE(until([&] { return executor.result().state != ExecutionState::Running; }, 5.,
      [&] { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_TRUE(move_goals.empty());
  ASSERT_EQ(execution_goals.size(), 1u);
  const auto & path = execution_goals[0].trajectory.joint_trajectory;
  for (const auto & name : config.arms.at("left").joints) {
    const auto found = std::find(path.joint_names.begin(), path.joint_names.end(), name);
    ASSERT_NE(found, path.joint_names.end());
    for (const auto & point : path.points) {
      EXPECT_NEAR(point.positions.at(found - path.joint_names.begin()), .1, .001) << name;
    }
  }
  const auto right = std::find(path.joint_names.begin(), path.joint_names.end(),
      config.arms.at("right").joints.front());
  ASSERT_NE(right, path.joint_names.end());
  EXPECT_NEAR(path.points.back().positions.at(right - path.joint_names.begin()),
      steps[1].right_target.front(), 1e-6);
}

// Menu 3 bundles as straight joint lines; a world block sits on the left_joint0 sweep at 0.5 rad.
class JointLineFixture : public RosFixture {
protected:
  void SetUp() override {
    RosFixture::SetUp();
    enable_smoothing(true);
  }
  // face_each_other + open_approach of left_to_right (receiver right opens its gripper).
  std::vector<Step> bundle(bool replan, double left0 = 1.0) {
    auto steps = config.motion_sequences.at("left_to_right").steps;
    steps.resize(2);
    for (auto & step : steps) {
      step.joint_line = true;
      step.joint_line_replan = replan;
      step.left_target[0] = left0;
    }
    return steps;
  }
  void line_everywhere(bool replan, double left0 = 1.0) {
    for (auto & [direction, sequence] : config.motion_sequences) {
      (void)direction;
      for (const size_t i : {size_t(0), size_t(1), size_t(4)}) {
        sequence.steps[i].joint_line = true;
        sequence.steps[i].joint_line_replan = replan;
        if (i < 2) sequence.steps[i].left_target[0] = left0;
      }
    }
  }
  // which: scene request numbers since arm_block() that carry the block (empty: every one).
  void arm_block(std::vector<size_t> which = {}) {
    const size_t base = scene_requests;
    transform_scene = [this, base, which](moveit_msgs::msg::PlanningScene & scene) {
      const size_t number = scene_requests - base; // Counted before transform_scene runs.
      if (!which.empty() && std::find(which.begin(), which.end(), number) == which.end()) return;
      moveit_msgs::msg::CollisionObject block;
      block.id = "line_block";
      block.header.frame_id = "base_link";
      block.operation = block.ADD;
      shape_msgs::msg::SolidPrimitive box;
      box.type = box.BOX;
      box.dimensions = {.05, .05, .05};
      geometry_msgs::msg::Pose pose;
      pose.position.x = .5 * std::cos(.5);
      pose.position.y = .5 * std::sin(.5);
      pose.orientation.w = 1;
      block.primitives.push_back(box);
      block.primitive_poses.push_back(pose);
      scene.world.collision_objects.push_back(block);
    };
    block_base = base;
  }
  ExecutionResult run(TrajectoryExecutor & executor, double timeout = 5.) {
    until([&] { return executor.result().state != ExecutionState::Running &&
                       executor.result().state != ExecutionState::Cancelling; },
        timeout, [&] { executor.tick(); });
    return executor.result();
  }
  // Waits until the first joint-line path is being prepared, without ticking the executor.
  void wait_first_preparation() {
    ASSERT_TRUE(until([&] { return scene_requests >= block_base + 2; }));
    until([] { return false; }, .05);
  }
  size_t block_base{0};
};

TEST_F(JointLineFixture, GripperLineCarriesTheReceiverGripperAndSkipsMoveGroup) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(bundle(true, .1)));
  EXPECT_EQ(run(executor).state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_TRUE(move_goals.empty()); // Checked straight lines need no planner.
  ASSERT_EQ(execution_goals.size(), 1u);
  const auto & path = execution_goals[0].trajectory.joint_trajectory;
  const auto gripper = std::find(path.joint_names.begin(), path.joint_names.end(),
      config.arms.at("right").gripper_joint);
  ASSERT_NE(gripper, path.joint_names.end());
  const auto index = gripper - path.joint_names.begin();
  EXPECT_NEAR(path.points.front().positions.at(index), .2, 1e-6);
  EXPECT_NEAR(path.points.back().positions.at(index), 0.0, 1e-6); // gripper_target
  EXPECT_NEAR(positions.at(config.arms.at("right").gripper_joint), 0.0, 1e-6);
}

TEST_F(JointLineFixture, CollidingLinesReplanTheWholeBundleOnceWithOmplBeforeMotion) {
  arm_block({2}); // Only the scene that checks the joint lines has the block.
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(bundle(true)));
  EXPECT_EQ(run(executor).state, ExecutionState::Succeeded) << executor.result().detail;
  ASSERT_EQ(move_goals.size(), 2u); // Both steps, in order, from the measured start.
  for (const auto & goal : move_goals) {
    EXPECT_TRUE(goal.planning_options.plan_only);
    EXPECT_EQ(goal.request.pipeline_id, "ompl");
    EXPECT_EQ(goal.request.planner_id, "RRTConnectkConfigDefault");
  }
  const auto & start = move_goals[0].request.start_state.joint_state;
  const auto left0 = std::find(start.name.begin(), start.name.end(), "left_joint0");
  ASSERT_NE(left0, start.name.end());
  EXPECT_NEAR(start.position.at(left0 - start.name.begin()), 0.0, 1e-9); // Not the line end.
  EXPECT_EQ(scene_requests - block_base, 4u);
  ASSERT_EQ(execution_goals.size(), 1u);
  EXPECT_NEAR(positions.at("left_joint0"), 1.0, 1e-6);
}

TEST_F(JointLineFixture, OmplReplanThatStillCollidesSendsNoPathOrGripper) {
  arm_block();
  line_everywhere(true);
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason)) << reason;
  ASSERT_TRUE(until([&] { return !motion.running(); }, 5., [&] { motion.tick(); }));
  EXPECT_FALSE(motion.result().sequence_completed);
  EXPECT_EQ(motion.result().code, "PATH_PREPARATION_FAILED");
  EXPECT_NE(motion.result().detail.find("OMPL replan also rejected"), std::string::npos)
      << motion.result().detail;
  EXPECT_EQ(move_goals.size(), 2u); // One replan of the bundle, no second one.
  EXPECT_TRUE(execution_goals.empty());
  EXPECT_TRUE(joint_goals.empty()); // No receiver_close or giver_release.
}

TEST_F(JointLineFixture, WithoutReplanSettingACollidingLineFailsWithoutPlanning) {
  arm_block();
  auto steps = bundle(false);
  steps[1].type = "dual_joint"; // Menu 7 shape: held left arm, no replan setting.
  steps[1].arm.clear();
  steps[1].gripper_target.clear();
  steps[1].hold_arm = "left";
  steps[1].left_target = steps[0].left_target; // Hold the left arm at the preceding segment's endpoint.
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(steps));
  const auto result = run(executor);
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_EQ(result.code, "PATH_PREPARATION_FAILED");
  EXPECT_NE(result.detail.find("collision"), std::string::npos) << result.detail;
  EXPECT_TRUE(move_goals.empty());
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(JointLineFixture, CancelDuringLinePreparationIsNotReplanned) {
  arm_block();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(bundle(true)));
  wait_first_preparation();
  executor.cancel();
  const auto result = run(executor);
  EXPECT_NE(result.state, ExecutionState::Succeeded);
  EXPECT_TRUE(move_goals.empty());
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(JointLineFixture, StartChangeDuringLinePreparationIsNotReplanned) {
  arm_block();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(bundle(true)));
  wait_first_preparation();
  positions.at("left_joint1") += .05; // Beyond continuous_start_tolerance.
  until([] { return false; }, .05);
  const auto result = run(executor);
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_NE(result.detail.find("collision"), std::string::npos) << result.detail;
  EXPECT_TRUE(move_goals.empty());
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(JointLineFixture, ExhaustedTimeoutIsNotReplannedOrExtended) {
  arm_block();
  auto steps = bundle(true);
  for (auto & step : steps) step.timeout = .1;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(steps));
  wait_first_preparation();
  until([] { return false; }, .25); // Past the 0.2 s bundle timeout before the result is read.
  const auto result = run(executor, 1.);
  EXPECT_NE(result.state, ExecutionState::Succeeded);
  EXPECT_TRUE(move_goals.empty());
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(JointLineFixture, FailureAfterTheLineWasSentIsNotReplanned) {
  execute_error = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_sequence(bundle(true, .1)));
  const auto result = run(executor);
  EXPECT_EQ(result.state, ExecutionState::Failed);
  EXPECT_TRUE(move_goals.empty());
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(JointLineFixture, BothDirectionsKeepReceiverCloseBeforeGiverRelease) {
  line_everywhere(true, .1);
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  for (const std::string direction : {"left_to_right", "right_to_left"}) {
    requests.clear();
    std::string reason;
    ASSERT_TRUE(motion.start(direction, reason)) << reason;
    ASSERT_TRUE(until([&] { return !motion.running(); }, 5., [&] { motion.tick(); }));
    ASSERT_TRUE(motion.result().sequence_completed) << motion.result().detail;
    const auto & sequence = config.motion_sequences.at(direction);
    EXPECT_EQ(requests, (std::vector<std::string>{"execute:path", "gripper:" + sequence.receiver,
        "gripper:" + sequence.giver, "execute:path"})) << direction;
  }
  EXPECT_TRUE(move_goals.empty());
}

TEST_F(RosFixture, CurrentLimitedReceiverDoesNotBlockReleaseAndRetreat) {
  enable_smoothing();
  config.require_passive_gripper_stationary = false;
  for (auto & [direction, sequence] : config.motion_sequences) {
    sequence.steps[2].gripper_wait_only = true;
  }
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  for (const std::string direction : {"right_to_left", "left_to_right"}) {
    gripper_position_jitter = 0;
    const auto & sequence = config.motion_sequences.at(direction);
    blocked_gripper = config.arms.at(sequence.receiver).gripper_joint;
    jitter_joint = blocked_gripper;
    std::string reason;
    ASSERT_TRUE(motion.start(direction, reason)) << reason;
    ASSERT_TRUE(until([&]() { return !motion.running(); }, 6.0, [&]() {
      if (context->snapshot().phase == "receiver_close") gripper_position_jitter = .006;
      motion.tick();
    }));
    EXPECT_TRUE(motion.result().sequence_completed) << motion.result().detail;
    EXPECT_FALSE(motion.result().handover_confirmed);
    EXPECT_LT(positions.at(blocked_gripper), .3); // Closed goal .4 was never reached.
    ASSERT_GE(execution_goals.size(), 2u);
    EXPECT_EQ(execution_goals[execution_goals.size() - 2].trajectory.joint_trajectory.joint_names.size(), 15u);
    EXPECT_EQ(execution_goals.back().trajectory.joint_trajectory.joint_names.size(), 14u);
  }
}

TEST_F(RosFixture, ContinuousPlanningFailureReportsApproachAndAttachedCupCollision) {
  enable_smoothing();
  move_error = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  move_error_source = "mock_ompl";
  move_error_message = "Unable to sample a valid goal";
  fail_on_move = 2;
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "other_hand";
  obstacle.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = sphere.SPHERE;
  sphere.dimensions = {.03};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = std::cos(.2);
  pose.position.y = std::sin(.2);
  obstacle.primitives = {sphere};
  obstacle.primitive_poses = {pose};
  scene_response.world.collision_objects = {obstacle};
  moveit_msgs::msg::AttachedCollisionObject cup;
  cup.link_name = "left_joint1_link";
  cup.object.id = "demo_place_cup";
  cup.object.header.frame_id = cup.link_name;
  cup.object.primitives = {sphere};
  pose.position.x = 1;
  pose.position.y = 0;
  cup.object.primitive_poses = {pose};
  cup.touch_links = {cup.link_name};
  scene_response.robot_state.attached_collision_objects = {cup};
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("right_to_left", reason)) << reason;
  ASSERT_TRUE(until([&]() { return !motion.running(); }, 3., [&]() { motion.tick(); }));
  EXPECT_EQ(motion.result().code, "PLANNING_FAILED");
  const auto detail = motion.result().detail;
  EXPECT_NE(detail.find("open_approach: MoveIt error 99999"), std::string::npos) << detail;
  EXPECT_NE(detail.find("source=mock_ompl"), std::string::npos) << detail;
  EXPECT_NE(detail.find(move_error_message), std::string::npos) << detail;
  EXPECT_NE(detail.find("nominal goal collision:"), std::string::npos) << detail;
  EXPECT_NE(detail.find("other_hand"), std::string::npos) << detail;
  EXPECT_NE(detail.find("attached object: demo_place_cup @ left_joint1_link"),
      std::string::npos) << detail;
  ASSERT_EQ(move_goals.size(), 2u);
  EXPECT_TRUE(execution_goals.empty());
  EXPECT_TRUE(grippers.empty());
}

TEST_F(RosFixture, CancelDuringPlanningNeverDispatchesExecution) {
  enable_smoothing();
  auto_complete = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto & steps = config.motion_sequences.at("left_to_right").steps;
  ASSERT_TRUE(executor.start_sequence({steps[0], steps[1]}));
  ASSERT_TRUE(until([&]() {
    return !moves.empty();
  }));
  executor.cancel();
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Cancelled;
      },
      2.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_TRUE(execution_goals.empty());
  EXPECT_EQ(move_goals.size(), 1u);
}

TEST_F(RosFixture, CancelDuringContinuousExecutionWaitsForStop) {
  enable_smoothing();
  auto_execute = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto & steps = config.motion_sequences.at("left_to_right").steps;
  ASSERT_TRUE(executor.start_sequence({steps[0], steps[1]}));
  ASSERT_TRUE(until(
      [&]() {
        return !executions.empty();
      },
      3.0,
      [&]() {
        executor.tick();
      }));
  executor.cancel();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Cancelled;
      },
      2.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_GT(cancels, 0);
}

TEST_F(RosFixture, ContinuousExecutionFailureBlocksBothGripperActions) {
  enable_smoothing();
  execute_error = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  HandoverMotion motion(config, *context, executor);
  std::string reason;
  ASSERT_TRUE(motion.start("left_to_right", reason));
  ASSERT_TRUE(until(
      [&]() {
        return !motion.running();
      },
      3.0,
      [&]() {
        motion.tick();
      }));
  EXPECT_FALSE(motion.result().sequence_completed);
  EXPECT_TRUE(grippers.empty());
  for (const auto & request : requests) {
    EXPECT_EQ(request.find("gripper:"), std::string::npos);
  }
}
} // namespace
} // namespace demo_1f

namespace demo_1f {
namespace {
TEST_F(RosFixture, ContinuousApiRejectsGripperStepsBeforePlanning) {
  enable_smoothing();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto & steps = config.motion_sequences.at("left_to_right").steps;
  EXPECT_FALSE(executor.start_sequence({steps[2], steps[3]}));
  EXPECT_TRUE(move_goals.empty());
  EXPECT_TRUE(execution_goals.empty());
  EXPECT_TRUE(grippers.empty());
}
} // namespace
} // namespace demo_1f

namespace demo_1f {
namespace {
TEST_F(RosFixture, CancelBeforeGripperSceneResponseCannotSendGripperGoal) {
  enable_smoothing();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  const auto & step = config.motion_sequences.at("left_to_right").steps[2];
  ASSERT_TRUE(executor.start(step));
  executor.cancel();
  ASSERT_TRUE(until(
      [&]() {
        return executor.result().state == ExecutionState::Cancelled;
      },
      2.0,
      [&]() {
        executor.tick();
      }));
  EXPECT_TRUE(grippers.empty());
  for (const auto & request : requests) {
    EXPECT_EQ(request.find("gripper:"), std::string::npos);
  }
}

TEST_F(RosFixture, CorrectionPlansBeforeExecutingTheReturnedTrajectory) {
  const auto step = correction_step();
  auto_execute = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return !execution_goals.empty(); }, 2., [&]() { executor.tick(); }));
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_TRUE(move_goals.front().planning_options.plan_only);
  EXPECT_EQ(move_goals.front().request.num_planning_attempts, 3);
  EXPECT_EQ(execution_goals.front().trajectory.joint_trajectory.joint_names, config.arms.at("left").joints);
  EXPECT_EQ(execution_goals.front().trajectory.joint_trajectory.points.size(), 2u);
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  auto_execute = true;
  ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Running; }, 2., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
}

TEST_F(RosFixture, CorrectionClassifiesEmptyPlanningFailuresWithoutExecution) {
  const auto step = correction_step();
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  for (const auto error : {moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                          moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED,
                          moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT}) {
    move_error = error;
    ASSERT_TRUE(executor.start(step));
    ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Running; }, 2., [&]() { executor.tick(); }));
    EXPECT_TRUE(move_goals.back().planning_options.plan_only);
    EXPECT_EQ(executor.result().code, error == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT ? "PLANNING_TIMEOUT" : "PLANNING_FAILED");
    EXPECT_TRUE(execution_goals.empty());
  }
}

TEST_F(RosFixture, CorrectionExecutionFailureNeverBecomesPlanningFailure) {
  const auto step = correction_step();
  execute_error = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Running; }, 2., [&]() { executor.tick(); }));
  ASSERT_EQ(execution_goals.size(), 1u);
  EXPECT_EQ(executor.result().code, "EXECUTION_FAILED");
}

TEST_F(RosFixture, CorrectionDoesNotRetryAnUnexpectedExecutionTraceInPlanResult) {
  const auto step = correction_step();
  move_error = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  move_result_has_execution = true;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Running; }, 2., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().code, "EXECUTION_FAILED");
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RosFixture, CorrectionCancelledDuringPlanningDoesNotExecute) {
  const auto step = correction_step();
  auto_complete = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return !moves.empty(); }, 2., [&]() { executor.tick(); }));
  executor.cancel();
  ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Cancelling; }, 2., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_TRUE(execution_goals.empty());
}


TEST_F(RosFixture, CorrectionRejectsMovedStartBeforeExecutingPlan) {
  const auto step = correction_step();
  plan_start_offset = .1;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Running; }, 2., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().code, "PATH_PREPARATION_FAILED");
  EXPECT_NE(executor.result().detail.find("Robot moved"), std::string::npos);
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RosFixture, CorrectionRejectsUnexpectedExecutionEvenOnPlanSuccess) {
  const auto step = correction_step();
  move_result_has_execution = true;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start(step));
  ASSERT_TRUE(until([&]() { return executor.result().state != ExecutionState::Running; }, 2., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().code, "PATH_PREPARATION_FAILED");
  EXPECT_TRUE(execution_goals.empty());
}

class RoundTripFixture : public RosFixture {
protected:
  void SetUp() override {
    RosFixture::SetUp();
    for (const auto * name : {"left_wheel_drive", "right_wheel_drive", "rear_wheel_drive"}) {
      positions[name] = 0;
    }
    enable_smoothing();
    config.arms.at("right").tcp = "right_joint0_link";
    transform_plan = [this](Move::Result & result) {
      auto & path = result.planned_trajectory.joint_trajectory;
      path.joint_names = config.arms.at("right").joints;
      std::reverse(path.joint_names.begin(), path.joint_names.end());
      path.header.stamp.sec = 321;
      path.points.clear();
      const std::vector<int64_t> times = {0, 700000001, 2300000007};
      for (size_t i = 0; i < times.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration(std::chrono::nanoseconds(times[i]));
        for (const auto & name : path.joint_names) {
          point.positions.push_back(positions.at(name) + (i == 0 ? 0 : i == 1 ? .012 : .04));
          point.velocities.push_back(i == 1 ? .025 : 0);
          point.accelerations.push_back(i == 1 ? .006 : 0);
        }
        path.points.push_back(point);
      }
      planned = result.planned_trajectory;
    };
    tilt.name = "pour_tilt";
    tilt.type = "linear";
    tilt.arm = "right";
    tilt.frame = config.planning_frame;
    tilt.target = {0, 0, 0, 0, 0, std::sin(.04 / 2), std::cos(.04 / 2)};
    tilt.timeout = 4;
    tilt.velocity_scale = tilt.acceleration_scale = .5;
    hold.name = "pour_hold";
    hold.type = "wait";
    hold.arm = "right";
    hold.duration = .05;
    hold.timeout = 1;
    untilt = tilt;
    untilt.name = "pour_untilt";
    untilt.target = {0, 0, 0, 0, 0, 0, 1};
  }

  bool finish(TrajectoryExecutor & executor) {
    return until([&]() {
      const auto state = executor.result().state;
      return state != ExecutionState::Running && state != ExecutionState::Cancelling;
    }, 3., [&]() { executor.tick(); });
  }

  bool reach(TrajectoryExecutor & executor, const std::string & phase) {
    return until([&]() { return executor.result().phase == phase; }, 3.,
        [&]() { executor.tick(); });
  }

  void attached_cup(bool obstacle) {
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions = {.03};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    moveit_msgs::msg::AttachedCollisionObject cup;
    cup.link_name = "right_joint0_link";
    cup.object.header.frame_id = cup.link_name;
    cup.object.id = "round_trip_cup";
    cup.object.primitives = {sphere};
    cup.object.primitive_poses = {pose};
    cup.touch_links = {cup.link_name};
    scene_response.robot_state.attached_collision_objects = {cup};
    if (obstacle) {
      moveit_msgs::msg::CollisionObject wall;
      wall.header.frame_id = "base_link";
      wall.id = "round_trip_wall";
      wall.primitives = {sphere};
      wall.primitive_poses = {pose};
      scene_response.world.collision_objects = {wall};
    }
  }

  Step tilt, hold, untilt;
  moveit_msgs::msg::RobotTrajectory planned;
};

class RoundTripWheelFixture : public RoundTripFixture {
protected:
  void SetUp() override {
    RoundTripFixture::SetUp();
    auto_execute = false;
  }

  bool complete_with_wheel_speed(
      TrajectoryExecutor & executor, size_t count, double velocity = .034558) {
    if (!until([&]() { return execution_goals.size() == count; }, 3.,
            [&]() { executor.tick(); })) return false;
    auto_execute = true;
    // Pump action callbacks without advancing the phase while the result arrives.
    if (!until([&]() { return completed_executions == count; })) return false;
    auto_execute = false;
    velocities["left_wheel_drive"] = velocity;
    if (!until([&]() {
          return context->snapshot().joints.at("left_wheel_drive").velocity == velocity;
        })) return false;
    executor.tick();
    return true;
  }

  void stop_wheel() {
    velocities["left_wheel_drive"] = 0;
    refresh();
  }
};

TEST_F(RoundTripWheelFixture, TransientWheelSpeedAfterEachSuccessWaitsBeforeAdvancing) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  EXPECT_EQ(execution_goals.size(), 1u);
  stop_wheel();
  executor.tick();
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  ASSERT_TRUE(reach(executor, "pour_hold"));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 2));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().phase, "pour_untilt");
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  stop_wheel();
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded);
  EXPECT_EQ(execution_goals.size(), 2u);
  EXPECT_EQ(move_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, PersistentWheelSpeedTimesOutWithoutExecutingReturn) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  const auto waiting = Clock::now();
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Cancelling; },
      2., [&]() { executor.tick(); }));
  EXPECT_GE(elapsed(waiting), .95);
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_NE(executor.result().detail.find("Wheel stabilization timed out"), std::string::npos);
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  EXPECT_EQ(execution_goals.size(), 1u);
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, RepeatedWheelSpeedResetsTheContinuousStableWindow) {
  config.settle_time = .12;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  for (int attempt = 0; attempt < 2; ++attempt) {
    stop_wheel();
    const auto stable_started = Clock::now();
    ASSERT_TRUE(until([&]() { return elapsed(stable_started) >= .04; }, .5,
        [&]() { executor.tick(); }));
    EXPECT_EQ(executor.result().phase, "pour_tilt");
    velocities["left_wheel_drive"] = .034558;
    refresh();
    executor.tick();
    EXPECT_EQ(executor.result().state, ExecutionState::Running);
    EXPECT_TRUE(executor.result().last_completed_step.empty());
  }
  stop_wheel();
  executor.tick();
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  ASSERT_TRUE(reach(executor, "pour_hold"));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, PersistentReturnWheelSpeedNeverReportsSuccess) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  stop_wheel();
  ASSERT_TRUE(reach(executor, "pour_hold"));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 2));
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Cancelling; },
      2., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(execution_goals.size(), 2u);
}

TEST_F(RoundTripWheelFixture, WheelMotionStillBlocksStopConfirmationAfterGraceExpires) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::StopUnconfirmed);
  EXPECT_EQ(executor.result().code, "STOP_UNCONFIRMED");
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, WheelSpeedDuringExecutionWaitsForResultThenSettles) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 3.,
      [&]() { executor.tick(); }));
  velocities["left_wheel_drive"] = .034558;
  ASSERT_TRUE(until([&]() {
    return context->snapshot().joints.at("left_wheel_drive").velocity == .034558;
  }));
  // Speed over the limit before the result keeps the tilt running.
  const auto kicked = Clock::now();
  ASSERT_TRUE(until([&]() { return elapsed(kicked) >= .1; }, .5, [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_EQ(completed_executions, 0u);
  auto_execute = true;
  ASSERT_TRUE(until([&]() { return completed_executions == 1; }));
  auto_execute = false;
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  stop_wheel();
  ASSERT_TRUE(reach(executor, "pour_hold"));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().last_completed_step, "pour_tilt");
  EXPECT_EQ(execution_goals.size(), 1u);
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, WheelPositionDuringExecutionStillCancelsImmediately) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 3.,
      [&]() { executor.tick(); }));
  positions["left_wheel_drive"] += 2 * config.continuous_start_tolerance;
  refresh();
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_NE(executor.result().detail.find("start/state mismatch"), std::string::npos);
  EXPECT_EQ(completed_executions, 0u);
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, WheelSpeedFromExecutionPersistingAfterResultTimesOut) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 3.,
      [&]() { executor.tick(); }));
  velocities["left_wheel_drive"] = .034558;
  ASSERT_TRUE(until([&]() {
    return context->snapshot().joints.at("left_wheel_drive").velocity == .034558;
  }));
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  auto_execute = true;
  ASSERT_TRUE(until([&]() { return completed_executions == 1; }));
  auto_execute = false;
  const auto waiting = Clock::now();
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Cancelling; },
      2., [&]() { executor.tick(); }));
  EXPECT_GE(elapsed(waiting), .95);
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_NE(executor.result().detail.find("Wheel stabilization timed out"), std::string::npos);
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, StaleWheelFeedbackDuringStabilizationCancelsBeforeGraceExpires) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  positions.erase("left_wheel_drive");
  ASSERT_TRUE(until([&]() { return executor.result().state == ExecutionState::Cancelling; },
      .8, [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_NE(executor.result().detail.find("Missing fresh round-trip feedback"), std::string::npos);
  positions["left_wheel_drive"] = 0;
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, UnknownWheelVelocityAfterSuccessIsFeedbackFailure) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  positions.erase("left_wheel_drive");
  const auto interrupted = Clock::now();
  ASSERT_TRUE(until([&]() { return elapsed(interrupted) > config.state_max_age + .02; }));
  positions["left_wheel_drive"] = 0;
  auto feedback = sample();
  feedback.velocity.clear();
  context->observe_joints(feedback);
  ASSERT_FALSE(context->snapshot().joints.at("left_wheel_drive").velocity);
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  EXPECT_NE(executor.result().detail.find("Missing finite wheel velocity"), std::string::npos);
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, WheelSpeedDoesNotMaskAnotherJointsPositionMismatch) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  positions["right_gripper"] += 2 * config.continuous_start_tolerance;
  refresh();
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  EXPECT_NE(executor.result().detail.find("right_gripper"), std::string::npos);
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, NonWheelVelocityAfterSuccessStillCancelsImmediately) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  velocities["left_joint0"] = .034558;
  refresh();
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  EXPECT_NE(executor.result().detail.find("left_joint0"), std::string::npos);
  velocities["left_joint0"] = 0;
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, WheelPositionMismatchIsNotEligibleForStabilization) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  positions["left_wheel_drive"] += 2 * config.continuous_start_tolerance;
  refresh();
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_NE(executor.result().detail.find("start/state mismatch"), std::string::npos);
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripWheelFixture, CancellationDuringWheelStabilizationNeverAdvances) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(complete_with_wheel_speed(executor, 1));
  executor.cancel();
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelling);
  stop_wheel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, PlansOnceAndExecutesExactTimeReversalTwice) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_EQ(executor.result().last_completed_step, "pour_untilt");
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_TRUE(move_goals[0].planning_options.plan_only);
  EXPECT_EQ(move_goals[0].request.group_name, "right");
  EXPECT_EQ(move_goals[0].request.planner_id, "LIN");
  EXPECT_FALSE(move_goals[0].request.start_state.is_diff);
  ASSERT_EQ(execution_goals.size(), 2u);
  auto original = planned;
  original.joint_trajectory.header.stamp = builtin_interfaces::msg::Time{};
  EXPECT_EQ(execution_goals[0].trajectory, original);
  EXPECT_EQ(execution_goals[0].trajectory.joint_trajectory.points,
      planned.joint_trajectory.points);
  const auto reversed = reverse_round_trip_path(execution_goals[0].trajectory,
      config.arms.at("right").joints);
  EXPECT_EQ(execution_goals[1].trajectory, reversed);
  EXPECT_EQ(execution_goals[0].trajectory.joint_trajectory.points.back().time_from_start,
      execution_goals[1].trajectory.joint_trajectory.points.back().time_from_start);
  EXPECT_EQ(scene_requests, 3u);
  for (const auto & goal : execution_goals) {
    EXPECT_EQ(goal.trajectory.joint_trajectory.joint_names, planned.joint_trajectory.joint_names);
    EXPECT_EQ(goal.trajectory.joint_trajectory.header.stamp.sec, 0);
  }
}

TEST_F(RoundTripFixture, WorksWithoutEnablingSmoothing) {
  config.smooth_motion = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
}

TEST_F(RoundTripFixture, DoesNotRevalidatePilzDynamicsWithReturnScaling) {
  untilt.velocity_scale = .0001;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_EQ(executor.result().last_completed_step, "pour_untilt");
  EXPECT_EQ(execution_goals.size(), 2u);
}

TEST_F(RoundTripFixture, PlanFailureIsRetryableButStateAndServiceErrorsAreNot) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  for (const int code : {moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION,
           moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE,
           moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE}) {
    move_error = code;
    ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
    ASSERT_TRUE(finish(executor));
    EXPECT_EQ(executor.result().retryable_before_motion,
        code == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
    EXPECT_TRUE(execution_goals.empty());
  }
}

TEST_F(RoundTripFixture, GenericPlanFailureIsRetryableBeforeMotion) {
  move_error = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_TRUE(move_goals.front().planning_options.plan_only);
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "ROUND_TRIP_PLAN_FAILED");
  EXPECT_TRUE(executor.result().retryable_before_motion);
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RoundTripFixture, GenericPlanFailureWithExecutionTraceIsNotRetryable) {
  move_error = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  move_result_has_execution = true;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_TRUE(move_goals.front().planning_options.plan_only);
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RoundTripFixture, TiltExecutionFailureIsNeverRetryableEvenIfArmIsUnmoved) {
  execute_error = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_EQ(execution_goals.size(), 1u);
  EXPECT_EQ(move_goals.size(), 1u);
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_TRUE(executor.result().last_completed_step.empty());
}

TEST_F(RoundTripFixture, HoldStartsAfterTheTiltStopsAndReturnStaysRunningUntilSettled) {
  auto_execute = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 2.,
      [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  auto_execute = true;
  ASSERT_TRUE(reach(executor, "pour_hold"));
  EXPECT_EQ(executor.result().last_completed_step, "pour_tilt");
  auto_execute = false;
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 2; }, 2.,
      [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  auto_execute = true;
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded);
}

TEST_F(RoundTripFixture, StartMismatchAfterHoldPreventsReturnExecution) {
  transform_scene = [this](moveit_msgs::msg::PlanningScene &) {
    if (scene_requests == 3) {
      positions.at(config.arms.at("right").joints[0]) += .1;
      context->observe_joints(sample());
    }
  };
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_EQ(executor.result().phase, "pour_untilt");
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, LatestSceneCollisionAfterHoldPreventsReturnExecution) {
  attached_cup(false);
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(reach(executor, "pour_hold"));
  attached_cup(true);
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().code, "ROUND_TRIP_INVALID") << executor.result().detail;
  EXPECT_EQ(executor.result().phase, "pour_untilt");
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, AttachedCupAndAllowedContactsAreUsedDuringValidation) {
  attached_cup(true);
  scene_response.allowed_collision_matrix.entry_names = {"round_trip_cup", "round_trip_wall"};
  moveit_msgs::msg::AllowedCollisionEntry entry;
  entry.enabled = {true, true};
  scene_response.allowed_collision_matrix.entry_values = {entry, entry};
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_EQ(execution_goals.size(), 2u);
}

TEST_F(RoundTripFixture, ReturnExecutionFailureReportsHoldAsLastCompleted) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(reach(executor, "pour_hold"));
  execute_error = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().phase, "pour_untilt");
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_EQ(execution_goals.size(), 2u);
  EXPECT_EQ(move_goals.size(), 1u);
}

TEST_F(RoundTripFixture, CancelDuringPlanningIgnoresLateCallbacksAndNextRunPlansAgain) {
  auto_complete = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return move_goals.size() == 1; }, 2., [&]() { executor.tick(); }));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_TRUE(execution_goals.empty());
  auto_complete = true;
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_EQ(move_goals.size(), 2u);
  EXPECT_EQ(execution_goals.size(), 2u);
}

TEST_F(RoundTripFixture, CancelDuringValidationNeverExecutesTilt) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() {
    return executor.result().detail.find("Validating tilt") != std::string::npos;
  }, 2.));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RoundTripFixture, CancelDuringTiltNeverHoldsOrReturns) {
  auto_execute = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 2.,
      [&]() { executor.tick(); }));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, CancelDuringHoldNeverReturns) {
  hold.duration = .5;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(reach(executor, "pour_hold"));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_EQ(executor.result().last_completed_step, "pour_tilt");
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, CancelDuringReturnValidationNeverExecutesReturn) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(reach(executor, "pour_untilt"));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, CancelDuringReturnNeverReportsSuccess) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(reach(executor, "pour_hold"));
  auto_execute = false;
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 2; }, 2.,
      [&]() { executor.tick(); }));
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Cancelled);
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  EXPECT_EQ(execution_goals.size(), 2u);
}

TEST_F(RoundTripFixture, HoldStateChangeFailsWithoutReturning) {
  hold.duration = .5;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(reach(executor, "pour_hold"));
  positions.at(config.arms.at("left").joints[0]) += .1;
  context->observe_joints(sample());
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_EQ(executor.result().phase, "pour_hold");
  EXPECT_EQ(executor.result().last_completed_step, "pour_tilt");
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, HeldGripperDriftUsesTheOriginalPlanningStart) {
  config.require_passive_gripper_stationary = false;
  config.held_gripper_start_tolerance = .05;
  const auto grip = config.arms.at("right").gripper_joint;
  const double original = positions.at(grip);
  transform_scene = [this, grip, original](moveit_msgs::msg::PlanningScene &) {
    if (scene_requests >= 2) {
      positions.at(grip) = original + (scene_requests == 2 ? .03 : .06);
      context->observe_joints(sample());
    }
  };
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().code, "ROBOT_STATE_INVALID");
  EXPECT_EQ(executor.result().phase, "pour_untilt");
  ASSERT_EQ(execution_goals.size(), 1u);
  const auto & names = execution_goals[0].trajectory.joint_trajectory.joint_names;
  EXPECT_EQ(std::find(names.begin(), names.end(), grip), names.end());
}

TEST_F(RoundTripFixture, MissingSceneServiceNeverAllowsAngleFallback) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  scene_server.reset();
  ASSERT_TRUE(until([&]() {
    std::string reason;
    return !executor.ready(config.motion_sequences.at("left_to_right"), reason);
  }));
  EXPECT_FALSE(executor.start_round_trip(tilt, hold, untilt));
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_TRUE(move_goals.empty());
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RoundTripFixture, PlanningTimeoutCancelsAndNeverExecutes) {
  auto_complete = false;
  tilt.timeout = .2;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "TIMEOUT");
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RoundTripFixture, ExecutionTimeoutCannotFallBackToAnotherAngle) {
  auto_execute = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 2.,
      [&]() { executor.tick(); }));
  ASSERT_TRUE(until([&]() {
    return executor.result().state != ExecutionState::Running &&
        executor.result().state != ExecutionState::Cancelling;
  }, 5., [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Failed);
  EXPECT_EQ(executor.result().code, "TIMEOUT");
  EXPECT_FALSE(executor.result().retryable_before_motion);
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, UnconfirmedStopRemainsLatchedAgainstLateActionResults) {
  auto_execute = false;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return execution_goals.size() == 1; }, 2.,
      [&]() { executor.tick(); }));
  timer->cancel();
  executor.cancel();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::StopUnconfirmed);
  EXPECT_FALSE(executor.start_round_trip(tilt, hold, untilt));
  timer->reset();
  auto_execute = true;
  ASSERT_TRUE(until([&]() { return executions.empty(); }));
  executor.tick();
  EXPECT_EQ(executor.result().state, ExecutionState::StopUnconfirmed);
  EXPECT_EQ(execution_goals.size(), 1u);
}

TEST_F(RoundTripFixture, EveryNewRoundTripPlansFromItsOwnMeasuredStart) {
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  ASSERT_EQ(executor.result().state, ExecutionState::Succeeded);
  for (const auto & name : config.arms.at("right").joints) positions.at(name) += .2;
  context->observe_joints(sample());
  tilt.target[5] = std::sin(.24 / 2);
  tilt.target[6] = std::cos(.24 / 2);
  untilt.target[5] = std::sin(.2 / 2);
  untilt.target[6] = std::cos(.2 / 2);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_EQ(move_goals.size(), 2u);
  ASSERT_EQ(execution_goals.size(), 4u);
  EXPECT_NE(execution_goals[0].trajectory.joint_trajectory.points.front().positions,
      execution_goals[2].trajectory.joint_trajectory.points.front().positions);
  EXPECT_EQ(execution_goals[3].trajectory,
      reverse_round_trip_path(execution_goals[2].trajectory, config.arms.at("right").joints));
}

TEST_F(RoundTripFixture, ActionSuccessStillRequiresEndpointStopForTiltAndReturn) {
  size_t moving_goal = 1;
  transform_feedback = [this, &moving_goal](sensor_msgs::msg::JointState & feedback) {
    if (moving_goal == 0 || execution_goals.size() != moving_goal) return;
    const auto & arm = config.arms.at("right").joints;
    for (size_t i = 0; i < feedback.name.size(); ++i) {
      if (std::find(arm.begin(), arm.end(), feedback.name[i]) != arm.end()) {
        feedback.velocity[i] = .2;
      }
    }
  };
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return completed_executions == 1; }, 2.,
      [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  moving_goal = 0;
  ASSERT_TRUE(reach(executor, "pour_hold"));
  moving_goal = 2;
  ASSERT_TRUE(until([&]() { return completed_executions == 2; }, 2.,
      [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().phase, "pour_untilt");
  EXPECT_EQ(executor.result().last_completed_step, "pour_hold");
  moving_goal = 0;
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded);
}

TEST_F(RoundTripFixture, ActionSuccessWithoutTiltEndpointDoesNotStartHold) {
  omitted_joint = config.arms.at("right").joints[0];
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(until([&]() { return completed_executions == 1; }, 2.,
      [&]() { executor.tick(); }));
  EXPECT_EQ(executor.result().state, ExecutionState::Running);
  EXPECT_EQ(executor.result().phase, "pour_tilt");
  EXPECT_TRUE(executor.result().last_completed_step.empty());
  positions.at(omitted_joint) += .04;
  ASSERT_TRUE(reach(executor, "pour_hold"));
  omitted_joint.clear();
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded);
}

TEST_F(RoundTripFixture, AttachedCupCollisionBeforeTiltNeverExecutes) {
  attached_cup(true);
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().code, "ROUND_TRIP_INVALID") << executor.result().detail;
  EXPECT_TRUE(executor.result().retryable_before_motion);
  EXPECT_TRUE(execution_goals.empty());
}

TEST_F(RoundTripFixture, ExecutesOriginalJerkViolationWithoutRetiming) {
  const auto normal_plan = transform_plan;
  transform_plan = [this, normal_plan](Move::Result & result) {
    normal_plan(result);
    auto & trajectory = result.planned_trajectory.joint_trajectory;
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      trajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(i * .01);
    }
    planned = result.planned_trajectory;
  };
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  ASSERT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  ASSERT_EQ(execution_goals.size(), 2u);
  EXPECT_NO_THROW(validate_round_trip_path(planned, config.arms.at("right").joints));
  EXPECT_EQ(execution_goals[0].trajectory.joint_trajectory.points,
      planned.joint_trajectory.points);
  EXPECT_EQ(execution_goals[1].trajectory,
      reverse_round_trip_path(execution_goals[0].trajectory, config.arms.at("right").joints));
  ASSERT_EQ(move_goals.size(), 1u);
  EXPECT_DOUBLE_EQ(move_goals[0].request.max_velocity_scaling_factor, tilt.velocity_scale);
  EXPECT_DOUBLE_EQ(move_goals[0].request.max_acceleration_scaling_factor, tilt.acceleration_scale);
}

TEST_F(RoundTripFixture, DoesNotAddTcpGeometryChecksToThePilzResponse) {
  tilt.target[0] = .01;
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().state, ExecutionState::Succeeded) << executor.result().detail;
  EXPECT_EQ(execution_goals.size(), 2u);
}

TEST_F(RoundTripFixture, FinalJointBoundViolationPreventsAnyExecutionRequest) {
  tilt.timeout = untilt.timeout = 15;
  const auto normal_plan = transform_plan;
  transform_plan = [normal_plan](Move::Result & result) {
    normal_plan(result);
    for (auto & position : result.planned_trajectory.joint_trajectory.points.back().positions) {
      position = 3.2;
    }
  };
  TrajectoryExecutor executor(*node, config, *context);
  warmup(executor);
  ASSERT_TRUE(executor.start_round_trip(tilt, hold, untilt));
  ASSERT_TRUE(finish(executor));
  EXPECT_EQ(executor.result().code, "ROUND_TRIP_INVALID");
  EXPECT_TRUE(execution_goals.empty());
  EXPECT_TRUE(executor.result().retryable_before_motion);
}

} // namespace
} // namespace demo_1f

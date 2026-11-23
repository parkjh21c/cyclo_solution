#include "demo_1f/motions/handover/cup_handover_motion.hpp"
#include "demo_1f/motions/pick/pick_cup_motion.hpp"
#include "demo_1f/motions/place/approach_cup_motion.hpp"
#include "demo_1f/motions/place/refine_cup_motion.hpp"
#include "demo_1f/motions/place/withdraw_cup_motion.hpp"
#include "demo_1f/motions/place/place_cup_motion.hpp"
#include "demo_1f/motions/pour/cup_pour_motion.hpp"
#include "demo_1f/motions/pour/pour_geometry.hpp"
#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include "demo_1f/runtime/geometry/tcp_roll.hpp"
#include "demo_1f/tasks/task_manager.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
using namespace std::chrono_literals;

namespace demo_1f {
namespace {
class Environment : public ::testing::Environment {
  void SetUp() override {
    initialize_moveit_logging();
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

auto * environment = ::testing::AddGlobalTestEnvironment(new Environment);

TEST(ApproachGeometry, PreservesHeightAndCenterAssumptionWithoutExtraPadding) {
  ApproachConfig config;
  geometry_msgs::msg::Pose body;
  body.position.z = .8;
  body.orientation.w = 1;
  std::string reason;
  auto target = approach_tcp_target(body, config, "left", .09, reason);
  ASSERT_TRUE(target) << reason;
  EXPECT_NEAR(target->position.z, .965, 1e-10);
  EXPECT_FALSE(approach_tcp_target(body, config, "left", .2, reason));
  auto scene = make_approach_scene({}, config, body, .09);
  const auto & cup = scene.world.collision_objects.front();
  EXPECT_NEAR(cup.primitives.front().dimensions[0], .165, 1e-10);
  EXPECT_NEAR(cup.primitive_poses.front().position.z, .8, 1e-10);
  EXPECT_NEAR(cup.primitives.front().dimensions[1], .054, 1e-10); // error capped at the margin
  EXPECT_TRUE(scene.robot_state.attached_collision_objects.empty());
  const auto small = make_approach_scene({}, config, body, .004);
  EXPECT_NEAR(small.world.collision_objects.front().primitives.front().dimensions[1], .048, 1e-10);
}

TEST(ApproachGeometry, RejectsInvalidCupSceneSize) {
  ApproachConfig config;
  EXPECT_TRUE(config.validate("left").empty());
  for (double radius : {0., -.01, .31, std::numeric_limits<double>::quiet_NaN()}) {
    config.scene_radius = radius;
    EXPECT_FALSE(config.validate("left").empty()) << radius;
  }
  config = ApproachConfig{};
  for (double margin : {-.001, .11, std::numeric_limits<double>::quiet_NaN()}) {
    config.scene_margin = margin;
    EXPECT_FALSE(config.validate("left").empty()) << margin;
  }
}

TEST(ApproachGeometry, RejectsInvalidConfiguration) {
  ApproachConfig config;
  EXPECT_FALSE(config.validate("unknown").empty());
  for (double roll : {-1., 1., 91., std::numeric_limits<double>::quiet_NaN()}) {
    config.tcp_roll_limit_deg = roll;
    EXPECT_FALSE(config.validate("left").empty());
  }
}

TEST(ApproachGeometry, AcceptsOnlyKnownCupSources) {
  ApproachConfig config;
  for (const std::string source : {"head", "wrist_left_init"}) {
    config.cup_source = source;
    EXPECT_TRUE(config.validate("left").empty()) << source;
  }
  config.cup_source = "wrist_left";
  EXPECT_FALSE(config.validate("left").empty());
}

TEST(HeadCupTracking, SavesOnlyConfiguredSource) {
  HeadCupTracker tracker({.08, .005, .15, "wrist_left_init"});
  CupState sample;
  auto & o = sample.observation;
  o.observation_type = msg::CupObservation::BODY;
  o.header.frame_id = "base_link";
  o.cup_id = "cup";
  o.valid = true;
  o.body_position.x = .5;
  o.inner_radius = .08;
  o.position_error = .09;
  o.confidence = 1;
  const auto start = Clock::now();
  int frame = 0;
  auto observe = [&](const std::string & source) {
    for (int i = 0; i < 3; ++i, ++frame) {
      o.source = source;
      o.header.stamp = rclcpp::Time(1'000'000'000LL + frame * 200'000'000LL);
      sample.received = start + std::chrono::milliseconds(200 * frame);
      tracker.observe(sample);
    }
  };
  observe("head");
  EXPECT_FALSE(tracker.last_known("cup"));
  observe("wrist_left_init");
  ASSERT_TRUE(tracker.last_known("cup"));
  EXPECT_NEAR(tracker.last_known("cup")->observation.body_position.x, .5, 1e-12);
}

TEST(ApproachGeometry, LowersTargetWithoutMovingCupCollisionGeometry) {
  ApproachConfig config;
  config.offset_z = -.05;
  config.forward_offset = 0; // The heading shift has its own test.
  geometry_msgs::msg::Pose body;
  body.position.x = .5;
  body.position.y = .2;
  body.position.z = .8;
  body.orientation.w = 1;
  std::string reason;
  for (const std::string arm : {"left", "right"}) {
    const auto target = approach_tcp_target(body, config, arm, .01, reason);
    ASSERT_TRUE(target) << reason;
    EXPECT_NEAR(target->position.x, .5, 1e-10);
    EXPECT_NEAR(target->position.y, .2, 1e-10);
    EXPECT_NEAR(target->position.z, .915, 1e-10);
  }
  const auto scene = make_approach_scene({}, config, body, .01);
  const auto & cup = scene.world.collision_objects.front();
  EXPECT_NEAR(cup.primitives.front().dimensions[0], .165, 1e-10);
  EXPECT_NEAR(cup.primitive_poses.front().position.z, .8, 1e-10);
  for (double offset : {-.31, .31, std::numeric_limits<double>::quiet_NaN()}) {
    config.offset_z = offset;
    EXPECT_FALSE(approach_tcp_target(body, config, "left", .01, reason));
  }
}

TEST(ApproachGeometry, TargetsConfiguredHeadingMirroredForRightHand) {
  ApproachConfig config;
  geometry_msgs::msg::Pose body;
  body.orientation.w = 1;
  std::string reason;
  auto target = approach_tcp_target(body, config, "left", .01, reason);
  ASSERT_TRUE(target) << reason;
  EXPECT_NEAR(target->orientation.w, 1, 1e-10); // Limit zero keeps the placeholder.
  config.tcp_yaw_limit_deg = 10;
  for (const std::string arm : {"left", "right"}) {
    target = approach_tcp_target(body, config, arm, .01, reason);
    ASSERT_TRUE(target) << reason;
    const auto yaw = tcp_yaw(target->orientation);
    ASSERT_TRUE(yaw);
    EXPECT_NEAR(*yaw, (arm == "left" ? -1 : 1) * M_PI / 4, 1e-9);
    EXPECT_TRUE(tcp_yaw_is_inward(*yaw, arm));
  }
  config.tcp_yaw_limit_deg = 3;
  EXPECT_FALSE(approach_tcp_target(body, config, "left", .01, reason));
}

TEST(ApproachGeometry, FixedPosePitchAndForwardOffsetAreValidated) {
  ApproachConfig config;
  config.tcp_pitch_min_deg = 20;
  config.tcp_pitch_limit_deg = 60;
  EXPECT_TRUE(config.validate("left").empty());
  for (double pitch : {10., 65., std::numeric_limits<double>::quiet_NaN()}) {
    config.tcp_pitch_deg = pitch;
    EXPECT_FALSE(config.validate("left").empty()) << pitch;
  }
  config.fixed_pose = false; // The fixed pitch is unused without the fixed pose.
  config.tcp_pitch_deg = 65;
  EXPECT_TRUE(config.validate("left").empty());
  config.fixed_pose = true;
  config.tcp_pitch_deg = 40;
  for (double offset : {-.01, .07, std::numeric_limits<double>::quiet_NaN()}) {
    config.forward_offset = offset;
    EXPECT_FALSE(config.validate("left").empty()) << offset;
  }
  config.forward_offset = .06;
  EXPECT_TRUE(config.validate("left").empty());
}

TEST(ApproachGeometry, FixedPoseParametersLoadReadOnly) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("approach.fixed_pose", false),
      rclcpp::Parameter("approach.tcp_pitch_deg", 30.0),
      rclcpp::Parameter("approach.forward_offset", .01)});
  auto node = std::make_shared<rclcpp::Node>("approach_fixed_pose_config", options);
  const auto config = load_approach_config(*node);
  EXPECT_FALSE(config.fixed_pose);
  EXPECT_DOUBLE_EQ(config.tcp_pitch_deg, 30.0);
  EXPECT_DOUBLE_EQ(config.forward_offset, .01);
  EXPECT_TRUE(node->describe_parameter("approach.fixed_pose").read_only);
  auto defaults = std::make_shared<rclcpp::Node>("approach_fixed_pose_defaults");
  const auto loaded = load_approach_config(*defaults);
  EXPECT_TRUE(loaded.fixed_pose);
  EXPECT_DOUBLE_EQ(loaded.tcp_pitch_deg, 40.0);
  EXPECT_DOUBLE_EQ(loaded.forward_offset, .025);
}

TEST(ApproachGeometry, FixedPoseShiftsAlongTheHeadingAndMatchesTheGoalBands) {
  ApproachConfig config; // approach.yaml ranges.
  config.tcp_roll_limit_deg = 10;
  config.tcp_pitch_min_deg = 20;
  config.tcp_pitch_limit_deg = 60;
  geometry_msgs::msg::Pose body;
  body.position.x = .5;
  body.position.y = .2;
  body.position.z = .8;
  body.orientation.w = 1;
  std::string reason;
  const double shift = .025 * std::cos(M_PI / 4);
  for (const std::string arm : {"left", "right"}) {
    const auto target = approach_tcp_target(body, config, arm, .01, reason);
    ASSERT_TRUE(target) << reason;
    // Left yaw -45: +x, -y; the right hand mirrors to +x, +y.
    EXPECT_NEAR(target->position.x, .5 + shift, 1e-9) << arm;
    EXPECT_NEAR(target->position.y, .2 + (arm == "left" ? -shift : shift), 1e-9) << arm;
    const auto q = approach_fixed_orientation(config, arm);
    const auto pitch = tcp_pitch(q);
    const auto yaw = tcp_yaw(q);
    const auto roll = tcp_roll(q);
    ASSERT_TRUE(pitch && yaw && roll);
    EXPECT_NEAR(*pitch, 40 * M_PI / 180, 1e-9); // Positive: hand forward points down.
    EXPECT_GE(*pitch, config.tcp_pitch_min_deg * M_PI / 180);
    EXPECT_LE(*pitch, config.tcp_pitch_limit_deg * M_PI / 180);
    EXPECT_NEAR(*yaw, (arm == "left" ? -1 : 1) * M_PI / 4, 1e-9);
    EXPECT_TRUE(tcp_yaw_is_inward(*yaw, arm));
    EXPECT_LE(std::abs(*roll), config.tcp_roll_limit_deg * M_PI / 180);
  }
}

TEST(ApproachGeometry, RejectsInconsistentMinimumPitch) {
  ApproachConfig config;
  config.tcp_pitch_limit_deg = 60;
  config.tcp_pitch_min_deg = 20;
  EXPECT_TRUE(config.validate("left").empty());
  config.tcp_pitch_downward_only = false;
  EXPECT_FALSE(config.validate("left").empty());
  config.tcp_pitch_downward_only = true;
  for (double minimum : {-1., 58., 60., 70., std::numeric_limits<double>::quiet_NaN()}) {
    config.tcp_pitch_min_deg = minimum;
    EXPECT_FALSE(config.validate("left").empty());
  }
}

class Executor : public ExecutorPort {
public:
  bool ready(const MotionSequence &, std::string &) const override {
    return stationary;
  }

  bool start(const Step & s) override {
    steps.push_back(s);
    current = {ExecutionState::Running, {}, {}, s.name};
    return true;
  }

  bool start_sequence(const std::vector<Step> & sequence) override {
    if (!sequence_supported) return false; // As the port default: nothing started.
    continuous.push_back(sequence);
    steps.insert(steps.end(), sequence.begin(), sequence.end());
    current = {ExecutionState::Running, {}, {}, sequence.front().name};
    return true;
  }

  bool start_paired(const Step &, const std::vector<Step> &) override {
    ++paired_calls;
    return false; // As before motion: the approach falls back to one step at a time.
  }

  bool start_round_trip(const Step & tilt, const Step & hold, const Step & untilt) override {
    round_steps = {tilt, hold, untilt};
    round_trips.push_back(round_steps);
    round_index = 0;
    round_active = true;
    return start(tilt);
  }

  void succeed_step() {
    const auto completed = steps.back().name;
    if (round_active && ++round_index < round_steps.size()) {
      start(round_steps[round_index]);
      current.last_completed_step = completed;
    } else {
      round_active = false;
      current = {ExecutionState::Succeeded, {}, {}, completed, completed};
    }
  }

  void tick() override {}

  void cancel(const std::string & code, const std::string & reason) override {
    current = {ExecutionState::Cancelled, code, reason};
  }

  ExecutionResult result() const override {
    return current;
  }

  bool stationary = true;
  bool sequence_supported = true;
  int paired_calls = 0;
  ExecutionResult current;
  std::vector<Step> steps;
  std::vector<std::vector<Step>> continuous;
  std::vector<Step> round_steps;
  std::vector<std::vector<Step>> round_trips;
  size_t round_index{0};
  bool round_active{false};
};

class Scene : public ApproachScenePort {
public:
  bool ready() const override {
    return !pending;
  }

  bool apply(const moveit_msgs::msg::PlanningScene & s) override {
    scenes.push_back(s);
    pending = true;
    return true;
  }

  ExecutionResult poll() override {
    if (!finish) {
      return {ExecutionState::Running, {}, {}};
    }
    pending = false;
    return outcome;
  }

  bool pending = false, finish = true;
  ExecutionResult outcome{ExecutionState::Succeeded, {}, {}};
  std::vector<moveit_msgs::msg::PlanningScene> scenes;
};

class ApproachFixture : public ::testing::Test {
protected:
  void SetUp() override {
    static int index = 0;
    node = std::make_shared<rclcpp::Node>("approach_test_" + std::to_string(++index));
    robot.require_tcp_tf = false;
    robot.gripper_stop_mode = "velocity";
    robot.state_max_age = 1;
    for (const std::string side : {"left", "right"}) {
      ArmConfig a;
      a.name = side;
      a.joints = {side + "_joint"};
      a.gripper_joint = side + "_grip";
      a.tcp = node->get_name() + std::string("_") + side + "_tcp";
      robot.arms[side] = a;
    }
    config = ApproachConfig{};
    config.refine_view_wait = .001;
    config.inner_radius = .08;
    config.observation_timeout = .2;
    config.tcp_yaw_inward_only = false;
    config.tcp_roll_limit_deg = 0;
    config.tcp_pitch_limit_deg = 0;
    config.tcp_pitch_downward_only = false;
    config.fixed_pose = false; // Existing approach tests use the OMPL range goal; fixed tests opt in.
    config.forward_offset = 0;
    context = std::make_unique<RobotContext>(*node, robot,
        HeadCupTrackConfig{config.inner_radius, config.wall_thickness, config.max_position_error});
    motion = std::make_unique<ApproachCupMotion>(robot, config, *context, executor, scene);
    refinement = std::make_unique<RefineCupMotion>(robot, config, *context, executor, scene);
    withdrawal = std::make_unique<WithdrawCupMotion>(robot, config, *context, executor, scene);
    broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    ros.add_node(node);
    feedback();
  }

  void feedback(double grip = .8, double gripper_velocity = 0) {
    sensor_msgs::msg::JointState j;
    j.header.stamp = node->now();
    for (const auto & [side, a] : robot.arms) {
      j.name.push_back(a.joints[0]);
      j.position.push_back(arm_joint);
      j.velocity.push_back(0);
      j.name.push_back(a.gripper_joint);
      j.position.push_back(grip);
      j.velocity.push_back(gripper_velocity);
    }
    context->observe_joints(j);
  }

  void observe(const std::string & source,
      double shift = 0,
      bool valid = true,
      double error = .09,
      int frames = 1,
      const std::string & detail = "") {
    msg::CupObservation o;
    o.observation_type = msg::CupObservation::BODY;
    o.header.frame_id = robot.planning_frame;
    o.source = source;
    o.cup_id = "cup";
    o.valid = valid;
    o.body_position.x = .54 + shift;
    o.body_position.z = .84;
    o.inner_radius = .08;
    o.position_error = error;
    o.confidence = 1;
    o.detail = detail;
    for (int i = 0; i < frames; ++i) {
      o.header.stamp = node->now();
      context->observe_cup(o);
      std::this_thread::sleep_for(1ms);
    }
  }

  void capture_initial() {
    for (int i = 0; i < 3; ++i) {
      observe("head");
      motion->tick();
    }
  }

  bool relaxed_pick_arrival = false;
  double arm_joint = 0; // Measured arm joint position reported by feedback().

  void complete() {
    if (executor.current.state == ExecutionState::Running) {
      auto s = executor.steps.back();
      if (s.target.size() == 7) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = robot.planning_frame;
        tf.child_frame_id = robot.arms.at(s.arm).tcp;
        tf.header.stamp = node->now();
        tf.transform.translation.x = s.target[0];
        tf.transform.translation.y = s.target[1];
        tf.transform.translation.z = s.target[2];
        tf.transform.rotation.x = s.target[3];
        tf.transform.rotation.y = s.target[4];
        tf.transform.rotation.z = s.target[5];
        tf.transform.rotation.w = s.target[6];
        if (s.tcp_roll_limit > 0 && s.tcp_yaw_limit == 0) {
          tf.transform.rotation = geometry_msgs::msg::Quaternion{};
          tf.transform.rotation.y = -std::sin(M_PI / 4);
          tf.transform.rotation.w = std::cos(M_PI / 4);
        }
        if (relaxed_pick_arrival && s.name == "PICK_PREGRASP") {
          const double yaw = *tcp_yaw(tf.transform.rotation) + 8 * M_PI / 180;
          tf2::Quaternion hand, axes;
          hand.setRPY(4 * M_PI / 180, 5 * M_PI / 180, yaw);
          axes.setRPY(0, -M_PI / 2, 0);
          tf.transform.rotation = tf2::toMsg(hand * axes);
        }
        broadcaster->sendTransform(tf);
        for (int i = 0; i < 3; ++i) {
          ros.spin_some();
          std::this_thread::sleep_for(2ms);
        }
      }
      executor.succeed_step();
    }
  }

  bool start(const std::string & arm = "right") {
    std::string reason;
    return motion->start(arm, "cup", reason);
  }

  void advance(const std::string & desired) {
    for (int i = 0; i < 60 && motion->running(); ++i) {
      if (context->snapshot().phase == desired) {
        return;
      }
      feedback();
      observe("head");
      complete();
      motion->tick();
    }
    ASSERT_EQ(context->snapshot().phase, desired) << motion->result().detail;
  }

  // initial_pose sequence: the left arm joint at 0 (feedback() reports arm_joint).
  void initial_pose_at_zero() {
    Step observe;
    observe.name = "observe_robot";
    observe.type = "joint";
    observe.arm = "left";
    observe.target = {0};
    observe.tolerance = .02;
    robot.motion_sequences["initial_pose"].steps = {observe};
  }

  void shoulder_for_pick() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = robot.planning_frame;
    tf.header.stamp = node->now();
    tf.transform.rotation.w = 1;
    for (const auto & side : {"l", "r"}) {
      tf.child_frame_id = std::string("arm_") + side + "_link1";
      tf.transform.translation.y = std::string(side) == "l" ? .1045 : -.1045;
      broadcaster->sendTransform(tf);
    }
    for (int i = 0; i < 10; ++i) {
      ros.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  void tcp_for_refinement(
      const std::string & side = "left", double x = .5, double y = 0, double z = 1.) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = robot.planning_frame;
    tf.child_frame_id = robot.arms.at(side).tcp;
    tf.header.stamp = node->now();
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf2::Quaternion q;
    q.setRPY(0, -M_PI / 2, 0);
    tf.transform.rotation = tf2::toMsg(q);
    broadcaster->sendTransform(tf);
    tf.child_frame_id = side == "left" ? "arm_l_link6" : "arm_r_link6";
    // Distinct wrist origin verifies that pickup does not use TCP position.
    tf.transform.translation.x -= .1;
    broadcaster->sendTransform(tf);
    for (int i = 0; i < 10; ++i) {
      ros.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  void wrist(bool valid,
      const std::string & detail = "NO_CANDIDATE: empty",
      double x = .54,
      double age = 0,
      const std::string & arm = "left", double confidence = .7) {
    msg::CupObservation o;
    o.header.frame_id = robot.planning_frame;
    o.header.stamp = node->now() - rclcpp::Duration::from_seconds(age);
    o.source = "wrist_" + arm;
    o.cup_id = "cup";
    o.observation_type = 2;
    o.valid = valid;
    o.detail = detail;
    o.body_position.x = x;
    o.body_position.z = .9;
    o.inner_radius = config.inner_radius;
    o.position_error = .008;
    o.confidence = confidence;
    context->observe_cup(o);
    std::this_thread::sleep_for(1ms);
  }

  rclcpp::Node::SharedPtr node;
  Config robot;
  ApproachConfig config;
  std::unique_ptr<RobotContext> context;
  Executor executor;
  Scene scene;
  std::unique_ptr<ApproachCupMotion> motion;
  std::unique_ptr<RefineCupMotion> refinement;
  std::unique_ptr<WithdrawCupMotion> withdrawal;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  rclcpp::executors::SingleThreadedExecutor ros;
};

TEST(RefinementGeometry, TiltKeepsPositionYawAndRollAndIncreasesDownwardPitch) {
  for (double yaw : {-1.2, 0., .8}) {
    for (double roll : {-.3, 0., .3}) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = .5;
      pose.position.y = .2;
      pose.position.z = 1.;
      tf2::Quaternion hand, axes;
      hand.setRPY(roll, 30 * M_PI / 180, yaw);
      axes.setRPY(0, -M_PI / 2, 0);
      pose.orientation = tf2::toMsg(hand * axes);
      auto target = tilt_tcp_target(pose, 5, 75);
      EXPECT_EQ(target.position, pose.position);
      EXPECT_NEAR(*tcp_pitch(target.orientation), 35 * M_PI / 180, 1e-9);
      EXPECT_NEAR(*tcp_yaw(target.orientation), yaw, 1e-9);
      EXPECT_NEAR(*tcp_roll(target.orientation), roll, 1e-9);
      EXPECT_NEAR(*tcp_pitch(tilt_tcp_target(pose, 11, 75).orientation),
          41 * M_PI / 180, 1e-9);
      EXPECT_THROW(tilt_tcp_target(pose, 5, 32), std::invalid_argument);
    }
  }
}

TEST_F(ApproachFixture, RefinementMissingOrOldInputNeverTilts) {
  config.refine_wait = .02;
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  wrist(false, "NO_CANDIDATE: empty", .54, .1);
  refinement->tick();
  std::this_thread::sleep_for(25ms);
  refinement->tick();
  EXPECT_EQ(refinement->result().code, "WRIST_INPUT_MISSING");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementWithoutObservationOpensInPlace) {
  config.refine_observe = false;
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, false));
  refinement->tick();
  EXPECT_TRUE(refinement->result().sequence_completed) << refinement->result().detail;
  EXPECT_TRUE(executor.steps.empty()); // Preview never opens.
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 10 && refinement->running(); ++i) {
    feedback();
    complete();
    refinement->tick();
  }
  ASSERT_TRUE(refinement->result().sequence_completed) << refinement->result().detail;
  ASSERT_EQ(executor.steps.size(), 1u); // No wrist input, tilt, correction or lowering.
  EXPECT_EQ(executor.steps[0].name, "OPEN_GRIPPER");
  EXPECT_EQ(executor.steps[0].target, std::vector<double>{config.refine_gripper_open});
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, RefinementWaitsForGripperBeforeTilting) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    feedback(.8, .1);
    wrist(false);
    refinement->tick();
  }
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(refinement->running());
  EXPECT_NE(refinement->result().detail.find("Waiting for gripper"), std::string::npos);
  for (int i = 0; i < 3; ++i) {
    feedback();
    wrist(false);
    refinement->tick();
  }
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps.front().name, "TILT_TO_OBSERVE");
}

TEST_F(ApproachFixture, CurrentLimitedGripperDoesNotAddRefinementSettleWait) {
  tcp_for_refinement();
  robot.require_passive_gripper_stationary = false;
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    feedback(.8, .1);
    wrist(false);
    refinement->tick();
  }
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps.front().name, "TILT_TO_OBSERVE");
}

TEST_F(ApproachFixture, RefinementGripperWaitIsBounded) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  const auto started = Clock::now();
  while (refinement->running() && elapsed(started) < 2.5) {
    feedback(.8, .1);
    refinement->tick();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(refinement->result().code, "ROBOT_NOT_READY");
  EXPECT_FALSE(refinement->running());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementWaitsForPositionWindowAfterGripperShift) {
  tcp_for_refinement();
  robot.gripper_stop_mode = "position_window";
  robot.gripper_stop_window = .03;
  robot.gripper_completion_window = .03;
  robot.gripper_stop_position_tolerance = .004;
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 6; ++i) {
    feedback(i % 2 ? .804602 : .8);
    wrist(false);
    refinement->tick();
    std::this_thread::sleep_for(8ms);
  }
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(refinement->running());
  EXPECT_NE(refinement->result().detail.find("not stationary"), std::string::npos);
  for (int i = 0; i < 12 && executor.steps.empty(); ++i) {
    feedback(.804602);
    wrist(false);
    refinement->tick();
    std::this_thread::sleep_for(8ms);
  }
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps.front().name, "TILT_TO_OBSERVE");
}

TEST_F(ApproachFixture, RefinementCanCancelWhileWaitingForGripper) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  feedback(.8, .1);
  refinement->tick();
  refinement->cancel();
  refinement->tick();
  EXPECT_EQ(refinement->result().state, "CANCELLED");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementGripperWaitDoesNotHideStaleFeedback) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  feedback(.8, .1);
  refinement->tick();
  robot.state_max_age = .001;
  std::this_thread::sleep_for(5ms);
  refinement->tick();
  EXPECT_EQ(refinement->result().code, "ROBOT_STATE_INVALID");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementRepeatsUntilCeilingAndClampsLastIncrement) {
  tcp_for_refinement();
  config.refine_pitch_ceiling_deg = 12;
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int attempt = 0; attempt < 4; ++attempt) {
    for (int sample = 0; sample < 3; ++sample) {
      feedback();
      wrist(false);
      refinement->tick();
    }
    if (attempt < 3) {
      ASSERT_EQ(executor.steps.size(), size_t(attempt + 1));
      const auto & step = executor.steps.back();
      EXPECT_EQ(step.type, "linear");
      EXPECT_FALSE(step.position_only);
      EXPECT_NEAR(step.target[0], .5, 1e-8);
      EXPECT_NEAR(step.target[1], 0, 1e-8);
      EXPECT_NEAR(step.target[2], 1, 1e-8);
      geometry_msgs::msg::Quaternion q;
      q.x = step.target[3];
      q.y = step.target[4];
      q.z = step.target[5];
      q.w = step.target[6];
      EXPECT_NEAR(*tcp_pitch(q), std::min((attempt + 1) * 5, 12) * M_PI / 180, 1e-8);
      complete();
      refinement->tick();
    }
  }
  EXPECT_EQ(refinement->result().code, "PITCH_LIMIT");
  EXPECT_EQ(executor.steps.size(), 3u);
}

TEST_F(ApproachFixture, RefinementAmbiguousInputTiltsWithoutSelectingCandidate) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(false, "RIM_UNCERTAIN: ambiguous cup candidates");
    refinement->tick();
  }
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(context->snapshot().phase, "TILT_TO_OBSERVE");
  refinement->cancel();
  refinement->tick();
  EXPECT_EQ(refinement->result().state, "CANCELLED");
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, AdvancingStabilityKeepsAngleUntilRimConfirmed) {
  tcp_for_refinement();
  config.refine_view_wait = .01;
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  wrist(false, "RIM_UNCERTAIN: ambiguous cup candidates");
  refinement->tick();
  wrist(false, "RIM_UNSTABLE: 1/3");
  refinement->tick();
  wrist(false, "RIM_UNSTABLE: 2/3");
  refinement->tick();
  EXPECT_TRUE(executor.steps.empty());
  std::this_thread::sleep_for(12ms);
  wrist(true); // Still lacks three stable observations in RobotContext.
  refinement->tick();
  EXPECT_TRUE(executor.steps.empty());
  for (int i = 0; i < 2; ++i) {
    wrist(true);
    refinement->tick();
  }
  ASSERT_EQ(scene.scenes.size(), 1u);
  refinement->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps[0].name, "CORRECT_TO_CUP_CENTER");
}

TEST_F(ApproachFixture, TransformErrorsNeverTriggerTimedViewChange) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 5; ++i) {
    wrist(false, "TF_UNAVAILABLE");
    refinement->tick();
  }
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementPreviewNeverTiltsAndCeilingBlocksMotion) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, false));
  for (int i = 0; i < 3; ++i) {
    wrist(false);
    refinement->tick();
  }
  EXPECT_FALSE(refinement->running());
  EXPECT_TRUE(executor.steps.empty());
  config.refine_pitch_ceiling_deg = 4;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(false);
    refinement->tick();
  }
  ASSERT_EQ(executor.steps.size(), 1u);
  complete();
  refinement->tick();
  for (int i = 0; i < 3; ++i) {
    wrist(false);
    refinement->tick();
  }
  EXPECT_EQ(refinement->result().code, "PITCH_LIMIT");
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, StabilityProgressDoesNotResetOverallSearchDeadline) {
  tcp_for_refinement();
  config.refine_search_timeout = .02;
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  wrist(false, "RIM_UNSTABLE: 1/3");
  refinement->tick();
  std::this_thread::sleep_for(25ms);
  wrist(false, "RIM_UNSTABLE: 2/3");
  refinement->tick();
  EXPECT_EQ(refinement->result().code, "RIM_SEARCH_TIMEOUT");
  EXPECT_FALSE(refinement->running());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementStopsAfterReleaseWithoutWithdrawal) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(true);
    refinement->tick();
  }
  ASSERT_EQ(scene.scenes.size(), 1u);
  EXPECT_NEAR(scene.scenes[0].world.collision_objects[0].primitive_poses[0].position.z,
      .9 - config.cup_height / 2,
      1e-9);
  refinement->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  const auto & step = executor.steps[0];
  EXPECT_EQ(step.type, "pose");
  EXPECT_NEAR(step.orientation_slack, 10 * M_PI / 180, 1e-9);
  EXPECT_DOUBLE_EQ(step.tcp_path_margin, .04);
  EXPECT_DOUBLE_EQ(step.planning_time, 3.0);
  ASSERT_TRUE(step.tcp_path_min_z);
  EXPECT_NEAR(*step.tcp_path_min_z, .915, 1e-9);
  EXPECT_NEAR(step.target[0], .54, 1e-9);
  EXPECT_NEAR(step.target[2], .97, 1e-9);
  EXPECT_NEAR(step.target[4], -std::sin(M_PI / 4), 1e-9);
  wrist(false); // Occlusion after target capture does not change this short correction.
  complete();
  refinement->tick();
  ASSERT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(executor.steps[1].type, "gripper");
  EXPECT_EQ(executor.steps[1].target[0], 0.0);
  complete();
  refinement->tick();
  EXPECT_TRUE(refinement->result().sequence_completed) << refinement->result().detail;
  EXPECT_EQ(executor.steps.size(), 2u);
}

TEST_F(ApproachFixture, WithdrawalNeedsNoCupObservationAndKeepsGripperUnchanged) {
  tcp_for_refinement("left", .54, 0, .97);
  std::string reason;
  ASSERT_TRUE(withdrawal->start("left", "cup", reason, true));
  withdrawal->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps[0].name, "LIFT_FROM_CUP");
  EXPECT_EQ(executor.steps[0].type, "linear");
  EXPECT_NEAR(executor.steps[0].target[0], .54, 1e-9);
  EXPECT_NEAR(executor.steps[0].target[2], 1.07, 1e-9);
  EXPECT_DOUBLE_EQ(executor.steps[0].velocity_scale, config.refine_correction_speed);
  EXPECT_FALSE(withdrawal->result().sequence_completed);
  // The second target follows actual settled feedback, not the previous command.
  complete();
  tcp_for_refinement("left", .539, .001, 1.069);
  withdrawal->tick();
  ASSERT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(executor.steps[1].name, "RETREAT_FROM_CUP");
  EXPECT_EQ(executor.steps[1].type, "linear");
  EXPECT_NEAR(executor.steps[1].target[0], .459, 1e-9);
  EXPECT_NEAR(executor.steps[1].target[1], .001, 1e-9);
  EXPECT_NEAR(executor.steps[1].target[2], 1.069, 1e-9);
  EXPECT_NEAR(executor.steps[1].target[4], -std::sin(M_PI / 4), 1e-9);
  EXPECT_FALSE(withdrawal->result().sequence_completed);
  complete();
  withdrawal->tick();
  EXPECT_TRUE(withdrawal->result().sequence_completed) << withdrawal->result().detail;
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, WithdrawalPreviewNeverMovesOrChangesScene) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(withdrawal->start("left", "cup", reason, false));
  withdrawal->tick();
  EXPECT_TRUE(withdrawal->result().sequence_completed);
  EXPECT_NEAR(withdrawal->approach_target().pose.position.x, .42, 1e-9);
  EXPECT_NEAR(withdrawal->approach_target().pose.position.z, 1.10, 1e-9);
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, BothHandsWithdrawLeftThenRightWithoutGripperCommands) {
  tcp_for_refinement("left", .54, .2, .97);
  tcp_for_refinement("right", .55, -.2, .96);
  std::string reason;
  ASSERT_TRUE(withdrawal->start("both", "cup", reason, true));
  withdrawal->tick();
  complete();
  withdrawal->tick();
  complete();
  withdrawal->tick(); // Left finished; yield before the right hand.
  EXPECT_FALSE(withdrawal->result().sequence_completed);
  ASSERT_EQ(executor.steps.size(), 2u);
  withdrawal->tick();
  complete();
  withdrawal->tick();
  complete();
  withdrawal->tick();
  ASSERT_EQ(executor.steps.size(), 4u);
  const std::vector<std::string> arms{"left", "left", "right", "right"};
  for (size_t i = 0; i < arms.size(); ++i) {
    EXPECT_EQ(executor.steps[i].arm, arms[i]);
    EXPECT_EQ(executor.steps[i].type, "linear");
  }
  EXPECT_NEAR(executor.steps[0].target[2], 1.07, 1e-9);
  EXPECT_NEAR(executor.steps[1].target[0], .46, 1e-9);
  EXPECT_NEAR(executor.steps[2].target[2], 1.06, 1e-9);
  EXPECT_NEAR(executor.steps[3].target[0], .47, 1e-9);
  EXPECT_TRUE(withdrawal->result().sequence_completed);
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, BothHandsWithdrawalRejectsPreviewWithoutMotion) {
  std::string reason;
  EXPECT_FALSE(withdrawal->start("both", "cup", reason, false));
  EXPECT_FALSE(reason.empty());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, FailedOrCancelledWithdrawalNeverContinues) {
  for (size_t failed_step : {1u, 2u}) {
    for (bool cancel : {false, true}) {
      executor.steps.clear();
      tcp_for_refinement();
      tcp_for_refinement("right");
      std::string reason;
      ASSERT_TRUE(withdrawal->start("both", "cup", reason, true));
      withdrawal->tick();
      if (failed_step == 2) {
        tcp_for_refinement("left", .5, 0, 1.1);
        complete();
        withdrawal->tick();
      }
      ASSERT_EQ(executor.steps.size(), failed_step);
      if (cancel) withdrawal->cancel();
      else executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "withdrawal failed"};
      withdrawal->tick();
      EXPECT_EQ(executor.steps.size(), failed_step);
      EXPECT_FALSE(withdrawal->result().sequence_completed);
      EXPECT_EQ(withdrawal->result().state, cancel ? "CANCELLED" : "FAILED");
    }
  }
}

TEST_F(ApproachFixture, WithdrawalWithoutTcpDoesNotMove) {
  std::string reason;
  ASSERT_TRUE(withdrawal->start("left", "cup", reason, true));
  withdrawal->tick();
  EXPECT_EQ(withdrawal->result().code, "TCP_TF_MISSING");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, DownwardRetreatIsRejectedBeforeAnyWithdrawal) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = robot.planning_frame;
  tf.child_frame_id = robot.arms.at("left").tcp;
  tf.header.stamp = node->now();
  tf.transform.translation.z = 1;
  tf.transform.rotation.x = 1; // +TCP Z points down.
  tf.transform.rotation.w = 0;
  broadcaster->sendTransform(tf);
  for (int i = 0; i < 10; ++i) {
    ros.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  std::string reason;
  ASSERT_TRUE(withdrawal->start("left", "cup", reason, true));
  withdrawal->tick();
  EXPECT_EQ(withdrawal->result().code, "RETREAT_DIRECTION");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, SeventyFiveDegreeRimCanProduceCorrectionPreview) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, false));
  for (int i = 0; i < 3; ++i) {
    wrist(true, "VALID_RIM", .54, 0, "left", 75.0 / 360.0);
    refinement->tick();
  }
  EXPECT_TRUE(refinement->result().sequence_completed) << refinement->result().detail;
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, BelowSeventyFiveDegreeRimCannotAuthorizeCorrection) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, false));
  for (int i = 0; i < 3; ++i) {
    wrist(true, "VALID_RIM", .54, 0, "left", 74.0 / 360.0);
    refinement->tick();
  }
  EXPECT_FALSE(refinement->result().sequence_completed);
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, WristStabilityAcceptsFourHundredMillisecondCadence) {
  const auto stamp = node->now();
  const auto received = Clock::now();
  for (const auto & source : {"head", "wrist_left"}) {
    msg::CupObservation input;
    input.header.frame_id = robot.planning_frame;
    input.source = source;
    input.cup_id = "cup";
    input.observation_type = std::string(source) == "head" ? 1 : 2;
    input.valid = true;
    input.body_position.z = .9;
    input.inner_radius = config.inner_radius;
    input.position_error = .009;
    input.confidence = .28;
    for (int i = 0; i < 3; ++i) {
      const double age = .8 - i * .4;
      input.header.stamp = stamp - rclcpp::Duration::from_seconds(age);
      ASSERT_TRUE(context->observe_cup(input, received -
          std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(age))));
    }
    const auto observation = context->cup_observation("cup", source);
    ASSERT_TRUE(observation);
    EXPECT_EQ(observation->stable_frames, std::string(source) == "head" ? 1 : 3);
  }
}

TEST_F(ApproachFixture, CorrectionPlanningFailuresAreBoundedAndKeepTarget) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(true);
    refinement->tick();
  }
  refinement->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  const auto target = executor.steps.front().target;
  for (int i = 0; i < 3; ++i) {
    feedback();
    executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "MoveIt error 99999"};
    refinement->tick();
    ASSERT_EQ(executor.steps.size(), size_t(std::min(i + 2, 3))) << refinement->result().detail;
    EXPECT_EQ(executor.steps.back().target, target);
  }
  EXPECT_EQ(executor.steps[1].planning_time, 5.0);
  EXPECT_EQ(executor.steps[1].tcp_path_margin, .04);
  EXPECT_EQ(executor.steps[2].tcp_path_margin, .06);
  EXPECT_EQ(refinement->result().code, "PLANNING_FAILED");
  EXPECT_FALSE(refinement->result().sequence_completed);
}

TEST_F(ApproachFixture, CorrectionPlanningFailureDoesNotRetryAfterJointMovement) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(true);
    refinement->tick();
  }
  refinement->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  sensor_msgs::msg::JointState moved;
  moved.header.stamp = node->now();
  moved.name = {"left_joint"};
  moved.position = {.1};
  moved.velocity = {0};
  context->observe_joints(moved);
  executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "MoveIt error 99999"};
  refinement->tick();
  EXPECT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(refinement->result().code, "ROBOT_MOVED");
}

TEST_F(ApproachFixture, CombinedCorrectionRejectsExcessiveLoweringBeforeStart) {
  tcp_for_refinement();
  config.refine_lower_distance = .09;
  std::string reason;
  EXPECT_FALSE(refinement->start("left", "cup", reason, true));
  EXPECT_FALSE(reason.empty());
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, FailedOrCancelledCombinedCorrectionNeverOpensGripper) {
  for (bool cancel : {false, true}) {
    executor.steps.clear();
    tcp_for_refinement();
    std::string reason;
    ASSERT_TRUE(refinement->start("left", "cup", reason, true));
    for (int i = 0; i < 3; ++i) {
      wrist(true);
      refinement->tick();
    }
    refinement->tick();
    ASSERT_EQ(executor.steps.size(), 1u);
    if (cancel) {
      refinement->cancel();
    }
    executor.current = {
        cancel ? ExecutionState::Cancelled : ExecutionState::Failed, "TEST", "test"};
    refinement->tick();
    EXPECT_EQ(executor.steps.size(), 1u);
    EXPECT_FALSE(refinement->result().sequence_completed);
  }
}

TEST_F(ApproachFixture, RefinementPreviewAndCancellationNeverDispatchLateCommands) {
  tcp_for_refinement();
  std::string reason;
  ASSERT_TRUE(refinement->start("left", "cup", reason, false));
  for (int i = 0; i < 3; ++i) {
    wrist(true);
    refinement->tick();
  }
  EXPECT_TRUE(refinement->result().sequence_completed);
  EXPECT_TRUE(scene.scenes.empty());
  EXPECT_TRUE(executor.steps.empty());
  scene.finish = false;
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(true);
    refinement->tick();
  }
  refinement->cancel();
  scene.finish = true;
  refinement->tick();
  EXPECT_EQ(refinement->result().state, "CANCELLED");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, RefinementRejectsDistantTargetAndUnsupportedArm) {
  tcp_for_refinement();
  std::string reason;
  EXPECT_FALSE(refinement->start("right", "cup", reason, true));
  ASSERT_TRUE(refinement->start("left", "cup", reason, true));
  for (int i = 0; i < 3; ++i) {
    wrist(true, "", .8);
    refinement->tick();
  }
  EXPECT_EQ(refinement->result().code, "CORRECTION_TOO_FAR");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, PreviewBothArmsNeverSendsSceneOrMotion) {
  config.offset_xy = {.03, -.04};
  executor.stationary = false;
  scene.pending = true;
  for (const std::string arm : {"left", "right"}) {
    std::string reason;
    ASSERT_TRUE(motion->start(arm, "cup", reason, false)) << reason;
    capture_initial();
    EXPECT_TRUE(motion->result().sequence_completed);
    EXPECT_FALSE(motion->at_observation_pose());
    EXPECT_NEAR(motion->approach_target().pose.position.x, .57, 1e-9);
    EXPECT_NEAR(motion->approach_target().pose.position.y, -.04, 1e-9);
    EXPECT_NEAR(motion->approach_target().pose.position.z, .84 + config.cup_height, 1e-9);
  }
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, ReachesTargetOnceWithoutGripperOrAttachmentChanges) {
  ASSERT_TRUE(start("left"));
  advance("OBSERVATION_READY");
  EXPECT_TRUE(motion->at_observation_pose());
  EXPECT_TRUE(motion->result().sequence_completed);
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps[0].type, "pose");
  EXPECT_TRUE(executor.steps[0].position_only);
  EXPECT_EQ(executor.steps[0].arm, "left");
  EXPECT_NEAR(executor.steps[0].target[2], .84 + config.cup_height, 1e-9);
  ASSERT_EQ(scene.scenes.size(), 1u);
  EXPECT_TRUE(scene.scenes[0].robot_state.attached_collision_objects.empty());
  for (int i = 0; i < 10; ++i) {
    motion->tick();
  }
  EXPECT_EQ(executor.steps.size(), 1u);
  std::string reason;
  EXPECT_TRUE(motion->is_ready_to_start("right", "cup", reason));
}

void add_right_return(Config & robot, ApproachConfig & config) {
  MotionSequence sequence;
  Step home;
  home.name = "giver_home";
  home.type = "joint";
  home.arm = "right";
  home.target = {1.0};
  sequence.steps = {home};
  robot.motion_sequences["right_return"] = sequence;
  config.other_arm_return_sequence = "right_return";
}

TEST_F(ApproachFixture, IndependentReturnMovesOnlyTheApproachArm) {
  add_right_return(robot, config);
  config.independent_return = true;
  ASSERT_TRUE(start("left"));
  advance("OBSERVATION_READY");
  EXPECT_EQ(executor.paired_calls, 0);
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps[0].type, "pose");
  EXPECT_EQ(executor.steps[0].arm, "left");
}

TEST_F(ApproachFixture, PairedReturnStaysAvailableWhenIndependentReturnIsOff) {
  add_right_return(robot, config);
  config.independent_return = false;
  ASSERT_TRUE(start("left"));
  advance("OBSERVATION_READY");
  EXPECT_EQ(executor.paired_calls, 1);
  ASSERT_EQ(executor.steps.size(), 2u); // Paired refused: the return, then the approach.
  EXPECT_EQ(executor.steps[0].name, "giver_home");
  EXPECT_EQ(executor.steps[1].arm, "left");
}

TEST_F(ApproachFixture, FixedPoseIsOneFullPosePtpStepWithoutTcpConstraints) {
  config.fixed_pose = true;
  config.forward_offset = .025;
  ASSERT_TRUE(start("left"));
  advance("OBSERVATION_READY");
  ASSERT_EQ(executor.steps.size(), 1u);
  const auto & step = executor.steps[0];
  EXPECT_EQ(step.type, "pose");
  EXPECT_FALSE(step.position_only);
  EXPECT_EQ(step.tcp_roll_limit, 0);
  EXPECT_EQ(step.tcp_pitch_limit, 0);
  EXPECT_EQ(step.tcp_pitch_min, 0);
  EXPECT_EQ(step.tcp_yaw_limit, 0);
  EXPECT_FALSE(step.tcp_pitch_downward_only);
  EXPECT_FALSE(step.tcp_yaw_inward_only);
  EXPECT_EQ(step.orientation_slack, 0);
  EXPECT_DOUBLE_EQ(step.velocity_scale, config.travel_scale);
  EXPECT_EQ(step.timeout, 45);
  const auto q = approach_fixed_orientation(config, "left");
  EXPECT_NEAR(step.target[3], q.x, 1e-12);
  EXPECT_NEAR(step.target[4], q.y, 1e-12);
  EXPECT_NEAR(step.target[5], q.z, 1e-12);
  EXPECT_NEAR(step.target[6], q.w, 1e-12);
  EXPECT_NEAR(step.target[0], .54 + .025 * std::cos(M_PI / 4), 1e-9);
  EXPECT_NEAR(step.target[1], -.025 * std::sin(M_PI / 4), 1e-9);
}

TEST_F(ApproachFixture, FixedPosePreviewReportsTheFixedTarget) {
  config.fixed_pose = true;
  std::string reason;
  ASSERT_TRUE(motion->start("left", "cup", reason, false)) << reason;
  capture_initial();
  ASSERT_TRUE(motion->result().sequence_completed);
  const auto q = approach_fixed_orientation(config, "left");
  const auto & target = motion->approach_target().pose;
  EXPECT_NEAR(target.orientation.x, q.x, 1e-12);
  EXPECT_NEAR(target.orientation.y, q.y, 1e-12);
  EXPECT_NEAR(target.orientation.z, q.z, 1e-12);
  EXPECT_NEAR(target.orientation.w, q.w, 1e-12);
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, FixedPoseFailureBeforeMotionFallsBackToOmplOnce) {
  config.fixed_pose = true;
  ASSERT_TRUE(start("left"));
  advance("MOVE_TO_OBSERVE");
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_FALSE(executor.steps[0].position_only);
  executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "no PTP plan"};
  feedback();
  motion->tick();
  ASSERT_EQ(executor.steps.size(), 2u);
  EXPECT_TRUE(executor.steps[1].position_only);
  EXPECT_EQ(executor.steps[1].name, "MOVE_TO_OBSERVE");
  EXPECT_NEAR(executor.steps[1].target[0], executor.steps[0].target[0], 1e-12); // Same position.
  EXPECT_NEAR(executor.steps[1].target[1], executor.steps[0].target[1], 1e-12);
  // A failure of the fallback itself ends the motion: only one fallback.
  executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "no OMPL plan"};
  motion->tick();
  EXPECT_EQ(motion->result().state, "FAILED");
  EXPECT_NE(motion->result().detail.find("fixed PTP failed first"), std::string::npos);
  EXPECT_EQ(executor.steps.size(), 2u);
}

TEST_F(ApproachFixture, FixedPoseFallbackCanReachTheObservationPose) {
  config.fixed_pose = true;
  ASSERT_TRUE(start("left"));
  advance("MOVE_TO_OBSERVE");
  executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "no PTP plan"};
  feedback();
  motion->tick();
  advance("OBSERVATION_READY");
  EXPECT_TRUE(motion->result().sequence_completed);
  EXPECT_EQ(executor.steps.size(), 2u);
}

TEST_F(ApproachFixture, FixedPoseFailureAfterMotionDoesNotFallBack) {
  config.fixed_pose = true;
  ASSERT_TRUE(start("left"));
  advance("MOVE_TO_OBSERVE");
  arm_joint = .1; // The arm left its start before the failure.
  feedback();
  executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "aborted"};
  motion->tick();
  EXPECT_EQ(motion->result().state, "FAILED");
  EXPECT_EQ(motion->result().code, "EXECUTION_FAILED");
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, FixedPoseUnconfirmedStopDoesNotFallBack) {
  config.fixed_pose = true;
  ASSERT_TRUE(start("left"));
  advance("MOVE_TO_OBSERVE");
  executor.current = {ExecutionState::StopUnconfirmed, "STOP_UNCONFIRMED", "unknown stop"};
  feedback();
  motion->tick();
  EXPECT_EQ(motion->result().code, "STOP_UNCONFIRMED");
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, FixedPoseCancelledDuringPlanningDoesNotFallBack) {
  config.fixed_pose = true;
  ASSERT_TRUE(start("left"));
  advance("MOVE_TO_OBSERVE");
  motion->cancel();
  executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "cancelled while planning"};
  feedback();
  motion->tick();
  EXPECT_FALSE(motion->running());
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, FixedPoseOffKeepsTheOmplRangeGoalWithTheForwardOffset) {
  config.fixed_pose = false;
  config.forward_offset = .025;
  ASSERT_TRUE(start("left"));
  advance("OBSERVATION_READY");
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_TRUE(executor.steps[0].position_only);
  EXPECT_NEAR(executor.steps[0].target[6], 1, 1e-12); // Position placeholder orientation.
  EXPECT_NEAR(executor.steps[0].target[0], .54 + .025 * std::cos(M_PI / 4), 1e-9);
}

TEST_F(ApproachFixture, FixedPoseLeavesThePairedPathUnchanged) {
  add_right_return(robot, config);
  config.independent_return = false;
  config.fixed_pose = true;
  ASSERT_TRUE(start("left"));
  advance("OBSERVATION_READY");
  EXPECT_EQ(executor.paired_calls, 1);
  ASSERT_EQ(executor.steps.size(), 2u);
  EXPECT_TRUE(executor.steps[1].position_only);
}

TEST_F(ApproachFixture, OutsideRollBandApproachesDirectlyWithoutAlignment) {
  config.tcp_pitch_min_deg = 20;
  config.tcp_pitch_limit_deg = 60;
  config.tcp_pitch_downward_only = true;
  config.tcp_roll_limit_deg = 60;
  config.tcp_yaw_inward_only = true;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = robot.planning_frame;
  tf.child_frame_id = robot.arms.at("left").tcp;
  tf.header.stamp = node->now();
  tf.transform.translation.x = .2;
  tf.transform.translation.z = 1.2;
  tf2::Quaternion roll, neutral;
  roll.setRPY(M_PI / 2, 0, 0);
  neutral.setRPY(0, -M_PI / 2, 0);
  tf.transform.rotation = tf2::toMsg(roll * neutral);
  broadcaster->sendTransform(tf);
  for (int i = 0; i < 20; ++i) {
    ros.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  std::string reason;
  ASSERT_TRUE(context->pose_of(tf.child_frame_id));
  ASSERT_TRUE(motion->start("left", "cup", reason, true));
  advance("OBSERVATION_READY");
  ASSERT_TRUE(motion->result().sequence_completed) << motion->result().detail;
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(executor.steps[0].name, "MOVE_TO_OBSERVE");
  EXPECT_NEAR(executor.steps[0].tcp_roll_limit, M_PI / 3, 1e-9);
  EXPECT_TRUE(executor.steps[0].tcp_yaw_inward_only);
  EXPECT_NEAR(executor.steps[0].tcp_pitch_min, 20 * M_PI / 180, 1e-9);
  EXPECT_NEAR(executor.steps[0].tcp_pitch_limit, 60 * M_PI / 180, 1e-9);
  EXPECT_NEAR(executor.steps[0].target[2], .84 + config.cup_height, 1e-9);
}

TEST_F(ApproachFixture, ApproachRejectsStaleOrUncertainHeadWithoutCommands) {
  config.observation_timeout = .01;
  std::string reason;
  ASSERT_TRUE(motion->start("right", "cup", reason, false));
  observe("head", 0, true, .09, 3);
  config.max_age = .000001;
  std::this_thread::sleep_for(12ms);
  motion->tick();
  EXPECT_EQ(motion->result().code, "CUP_NOT_FOUND");
  config.max_age = .5;
  config.max_position_error = .08;
  ASSERT_TRUE(motion->start("right", "cup", reason, true));
  observe("head", 0, true, .09, 3);
  std::this_thread::sleep_for(12ms);
  motion->tick();
  EXPECT_EQ(motion->result().code, "CUP_NOT_FOUND");
  EXPECT_TRUE(scene.scenes.empty());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, HeadCollectorUsesExactlyThreeNewFramesWithoutNestedWarmup) {
  ASSERT_TRUE(start());
  for (int i = 0; i < 3; ++i) {
    observe("head", .003 * i, true, .09, 1);
    motion->tick();
    if (i < 2) {
      EXPECT_TRUE(scene.scenes.empty());
    }
  }
  ASSERT_EQ(scene.scenes.size(), 1u);
  EXPECT_NEAR(motion->estimated_cup_pose().pose.position.x, .543, 1e-9);
  EXPECT_TRUE(executor.steps.empty()); // Scene acknowledgement still precedes motion.
}

TEST_F(ApproachFixture, HeadTrackIsCollectedBeforeRequestsAndSurvivesOcclusion) {
  HeadCupObserver collector(*context, config);
  geometry_msgs::msg::PoseStamped estimate;
  double uncertainty = 0;
  std::string reason;
  for (int i = 0; i < 4; ++i) {
    observe("head", .003 * i, true, .09, 1);
    EXPECT_EQ(collector.poll("cup", estimate, uncertainty, reason), i >= 2);
  }
  EXPECT_NEAR(estimate.pose.position.x, .546, 1e-9);
  const auto stamp = estimate.header.stamp;
  const auto saved = context->tracked_head_cup("cup", config.max_age, reason);
  ASSERT_TRUE(saved);
  observe("head", 0, false, .09, 1, "Cup occluded during handover");
  ASSERT_TRUE(collector.poll("cup", estimate, uncertainty, reason));
  EXPECT_EQ(estimate.header.stamp, stamp);
  EXPECT_NEAR(estimate.pose.position.x, .546, 1e-9);
  EXPECT_EQ(context->tracked_head_cup("cup", config.max_age, reason)->received, saved->received);
  ASSERT_TRUE(motion->start("left", "cup", reason, false));
  motion->tick();
  EXPECT_TRUE(motion->result().sequence_completed);
  EXPECT_NEAR(motion->approach_target().pose.position.x, .546, 1e-9);
  EXPECT_TRUE(scene.scenes.empty());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, HeadTrackRetainsOldPositionUntilNewLocationIsConfirmed) {
  for (int i = 0; i < 3; ++i) observe("head");
  std::string reason;
  auto saved = context->tracked_head_cup("cup", config.max_age, reason);
  ASSERT_TRUE(saved);
  for (int i = 0; i < 3; ++i) {
    observe("head", .1);
    const auto current = context->tracked_head_cup("cup", config.max_age, reason);
    ASSERT_TRUE(current);
    EXPECT_NEAR(current->observation.body_position.x, i < 2 ? .54 : .64, 1e-9);
  }
  // Wrist RIM observations do not update the head BODY track.
  wrist(true, "VALID_RIM", .9);
  EXPECT_NEAR(context->tracked_head_cup("cup", config.max_age, reason)->observation.body_position.x,
      .64, 1e-9);
}

TEST_F(ApproachFixture, RepeatedReadsAndReplayCannotRefreshStoredHeadCup) {
  for (int i = 0; i < 3; ++i) observe("head");
  std::string reason;
  auto saved = context->tracked_head_cup("cup", config.max_age, reason);
  ASSERT_TRUE(saved);
  EXPECT_FALSE(context->observe_cup(saved->observation));
  const auto reread = context->tracked_head_cup("cup", config.max_age, reason);
  ASSERT_TRUE(reread);
  EXPECT_EQ(reread->received, saved->received);
  EXPECT_EQ(reread->observation.header.stamp, saved->observation.header.stamp);
  std::this_thread::sleep_for(2ms);
  EXPECT_FALSE(context->tracked_head_cup("cup", .000001, reason));
  EXPECT_NE(reason.find("expired"), std::string::npos);
}

TEST_F(ApproachFixture, InitialSamplesAreDistinctAndAveragedWithConservativeGeometry) {
  ASSERT_TRUE(start());
  observe("head", -.006);
  motion->tick();
  for (int i = 0; i < 5; ++i) {
    motion->tick(); // Repeated reads of the same image must not fill the sample window.
  }
  EXPECT_TRUE(scene.scenes.empty());
  observe("head", .003);
  motion->tick();
  EXPECT_TRUE(scene.scenes.empty());
  observe("head", .006);
  motion->tick();
  ASSERT_EQ(scene.scenes.size(), 1u);
  EXPECT_NEAR(motion->approach_target().pose.position.x, .541, 1e-9);
  const auto & cup = scene.scenes[0].world.collision_objects[0];
  EXPECT_NEAR(cup.primitive_poses[0].position.x, .541, 1e-9);
  // The 0.097 m sample error is capped in the scene; the target still uses it.
  EXPECT_NEAR(cup.primitives[0].dimensions[1], config.scene_radius + config.scene_margin, 1e-9);
  EXPECT_TRUE(executor.steps.empty()); // Still awaiting scene acknowledgement.
}

TEST_F(ApproachFixture, InvalidSampleBetweenValidFramesDoesNotResetCollection) {
  ASSERT_TRUE(start());
  observe("head");
  motion->tick();
  observe("head", 0, false);
  motion->tick();
  for (int i = 0; i < 2; ++i) {
    observe("head");
    motion->tick();
  }
  EXPECT_EQ(scene.scenes.size(), 1u);
  EXPECT_TRUE(executor.steps.empty()); // Scene acknowledgement still precedes motion.
}

TEST_F(ApproachFixture, ShiftedSampleResetsCollectionAndTimeoutSendsNothing) {
  ASSERT_TRUE(start());
  for (int i = 0; i < 2; ++i) {
    observe("head");
    motion->tick();
  }
  observe("head", .05);
  motion->tick();
  EXPECT_TRUE(scene.scenes.empty());
  config.observation_timeout = .001;
  motion->tick();
  EXPECT_EQ(motion->result().code, "CUP_NOT_FOUND");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, FrozenCupIgnoresShiftUncertaintyAndLossAfterCapture) {
  ASSERT_TRUE(start());
  capture_initial();
  const auto saved = motion->approach_target();
  scene.finish = false;
  observe("head", .04, true, .14);
  motion->tick();
  EXPECT_TRUE(executor.steps.empty());
  observe("head", 0, false, .09, 1, "Cup occluded by hand");
  scene.finish = true;
  motion->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  EXPECT_EQ(scene.scenes.size(), 1u);
  EXPECT_DOUBLE_EQ(executor.steps[0].target[0], saved.pose.position.x);
  EXPECT_DOUBLE_EQ(executor.steps[0].target[2], saved.pose.position.z);
  EXPECT_EQ(context->snapshot().phase, "MOVE_TO_OBSERVE");
  complete();
  motion->tick();
  EXPECT_TRUE(motion->at_observation_pose());
  EXPECT_TRUE(motion->result().sequence_completed);
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, FrozenCupDoesNotRequireFreshCameraButRequiresRobotFeedback) {
  ASSERT_TRUE(start());
  capture_initial();
  config.max_age = .000001;
  motion->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  robot.state_max_age = .000001;
  motion->tick();
  EXPECT_FALSE(motion->result().sequence_completed);
  EXPECT_EQ(executor.current.code, "ROBOT_STATE_INVALID");
  EXPECT_FALSE(motion->at_observation_pose());
}

TEST_F(ApproachFixture, InvalidRobotFeedbackBeforeDispatchStillBlocksMotion) {
  ASSERT_TRUE(start());
  capture_initial();
  robot.state_max_age = .000001;
  motion->tick();
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_EQ(motion->result().code, "ROBOT_NOT_READY");
}

TEST_F(ApproachFixture, NewRequestUsesLatestCupCollectedWhileIdle) {
  std::string reason;
  ASSERT_TRUE(motion->start("left", "cup", reason, false));
  capture_initial();
  const auto previous = motion->approach_target();
  for (int i = 0; i < 3; ++i) observe("head", .04);
  ASSERT_TRUE(motion->start("left", "cup", reason, false));
  motion->tick();
  EXPECT_TRUE(motion->result().sequence_completed);
  EXPECT_NEAR(motion->approach_target().pose.position.x, previous.pose.position.x + .04, 1e-9);
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, ApproachCancelDuringMotionDoesNotReleaseOrClaimArrival) {
  std::string reason;
  ASSERT_TRUE(motion->start("right", "cup", reason, true));
  capture_initial();
  motion->tick();
  ASSERT_EQ(executor.steps.size(), 1u);
  motion->cancel();
  motion->tick();
  EXPECT_EQ(motion->result().state, "CANCELLED");
  EXPECT_FALSE(motion->at_observation_pose());
  EXPECT_FALSE(motion->blocks_other_motion());
  EXPECT_EQ(executor.steps.size(), 1u);
}

TEST_F(ApproachFixture, FailedInitialSceneCannotDispatchMotion) {
  ASSERT_TRUE(start());
  capture_initial();
  ASSERT_EQ(scene.scenes.size(), 1u);
  scene.outcome = {ExecutionState::Failed, "SCENE_FAILED", "scene rejected"};
  motion->tick();
  EXPECT_EQ(motion->result().code, "SCENE_FAILED");
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_FALSE(motion->blocks_other_motion());
}

TEST_F(ApproachFixture, RejectsUnknownObservationTypeAndBodyFromWrist) {
  msg::CupObservation observation;
  observation.header.frame_id = robot.planning_frame;
  observation.header.stamp = node->now();
  observation.source = "right";
  observation.cup_id = "cup";
  observation.valid = true;
  observation.inner_radius = .08;
  observation.confidence = 1;
  EXPECT_FALSE(context->observe_cup(observation));
  observation.observation_type = msg::CupObservation::BODY;
  EXPECT_FALSE(context->observe_cup(observation));
  EXPECT_FALSE(context->cup_observation("cup", "right"));
}

TEST_F(ApproachFixture, CancelPendingSceneCannotDispatchLateMotion) {
  scene.finish = false;
  ASSERT_TRUE(start());
  capture_initial();
  motion->cancel();
  motion->tick();
  EXPECT_TRUE(motion->running());
  scene.finish = true;
  motion->tick();
  EXPECT_FALSE(motion->running());
  EXPECT_FALSE(motion->blocks_other_motion());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, SceneTimeoutBlocksOtherMotions) {
  ASSERT_TRUE(start());
  capture_initial();
  scene.outcome = {ExecutionState::StopUnconfirmed, "SCENE_UNCONFIRMED", "timeout"};
  motion->tick();
  EXPECT_TRUE(motion->blocks_other_motion());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, ReplayCannotRefreshObservation) {
  observe("head");
  auto old = context->cup_observation("cup", "head");
  ASSERT_TRUE(old);
  EXPECT_FALSE(context->observe_cup(old->observation));
  EXPECT_EQ(context->cup_observation("cup", "head")->stable_frames, old->stable_frames);
}

TEST(PickGeometry, SideGraspUsesRimHeightAndHorizontalJawOrientation) {
  PickCupConfig c;
  geometry_msgs::msg::Point rim;
  rim.x = .6;
  rim.y = .2;
  rim.z = 1.0;
  auto p = cup_grasp_pose(rim, .165, c, std::atan2(.2, .6));
  EXPECT_NEAR(p.position.z, .9, 1e-9);
  EXPECT_NEAR(p.position.x, .6 + .03 * std::cos(std::atan2(.2, .6)), 1e-9);
  EXPECT_NEAR(*tcp_pitch(p.orientation), 0, 1e-9);
  EXPECT_NEAR(*tcp_yaw(p.orientation), std::atan2(.2, .6), 1e-9);
  EXPECT_TRUE(c.validate(.165).empty());
  c.stand_off = .9;
  EXPECT_FALSE(c.validate(.165).empty());
}

TEST(PickGeometry, TiltedGraspKeepsPadsOnCupAxisAndJawsLevel) {
  PickCupConfig c;
  c.approach_pitch_deg = 15;
  geometry_msgs::msg::Point rim;
  rim.x = .6;
  rim.y = .2;
  rim.z = 1.0;
  const double yaw = -.3, pitch = 15 * M_PI / 180;
  auto p = cup_grasp_pose(rim, .165, c, yaw);
  EXPECT_NEAR(*tcp_pitch(p.orientation), pitch, 1e-9);
  EXPECT_NEAR(*tcp_yaw(p.orientation), yaw, 1e-9);
  EXPECT_NEAR(*tcp_roll(p.orientation), 0, 1e-9); // Jaw axis stays horizontal.
  // Pad center = TCP - 3 cm along the tilted forward = cup axis at grasp height.
  tf2::Quaternion q;
  tf2::fromMsg(p.orientation, q);
  const auto forward = tf2::quatRotate(q, tf2::Vector3(0, 0, -1));
  EXPECT_NEAR(p.position.x - .03 * forward.x(), .6, 1e-9);
  EXPECT_NEAR(p.position.y - .03 * forward.y(), .2, 1e-9);
  EXPECT_NEAR(p.position.z - .03 * forward.z(), .9, 1e-9);
  EXPECT_TRUE(c.validate(.165).empty());
  c.approach_pitch_deg = 40;
  EXPECT_FALSE(c.validate(.165).empty());
}

TEST_F(ApproachFixture, PickPreviewNeverOpensGripperOrChangesScene) {
  tcp_for_refinement();
  shoulder_for_pick();
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, false));
  for (int i = 0; i < 10 && pick.running(); ++i) {
    observe("head");
    pick.tick();
  }
  EXPECT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(ApproachFixture, PickHeadingIsFixedRegardlessOfStartPose) {
  PickCupConfig pc;
  pc.approach_yaw_deg = -20;
  pc.approach_pitch_deg = 15;
  for (const double x : {.2, .8}) {
    tcp_for_refinement("left", x, x - .5);
    PickCupMotion pick(robot, config, pc, *context, executor, scene);
    std::string reason;
    ASSERT_TRUE(pick.start("left", "cup", reason, false));
    for (int i = 0; i < 10 && pick.running(); ++i) {
      observe("head");
      pick.tick();
    }
    ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
    const auto & q = pick.approach_target().pose.orientation;
    EXPECT_NEAR(*tcp_yaw(q), -20 * M_PI / 180, 1e-9);
    EXPECT_NEAR(*tcp_pitch(q), 15 * M_PI / 180, 1e-9);
  }
}

TEST_F(ApproachFixture, PickApproachStopsAfterOpenAndPregrasp) {
  tcp_for_refinement("left");
  relaxed_pick_arrival = true;
  PickCupConfig pc;
  pc.stop_at_pregrasp = true;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 30 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    complete();
    pick.tick();
  }
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  ASSERT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(executor.steps[0].name, "PICK_OPEN");
  EXPECT_EQ(executor.steps[0].target, std::vector<double>{0.0});
  EXPECT_EQ(executor.steps[1].name, "PICK_PREGRASP");
  EXPECT_EQ(executor.steps[1].type, "pose");
  EXPECT_FALSE(executor.steps[1].position_only); // Full pose selects Pilz PTP.
  EXPECT_EQ(executor.steps[1].orientation_slack, 0);
  EXPECT_NEAR(pick.approach_target().pose.position.x, .45, 1e-8);
  EXPECT_NEAR(pick.approach_target().pose.position.z, .84 - .165 / 2 + .065, 1e-8);
  EXPECT_FALSE(pick.blocks_other_motion());
  for (int i = 0; i < 5; ++i) {
    pick.tick();
  }
  EXPECT_EQ(executor.steps.size(), 2u);
  for (const auto & update : scene.scenes) {
    EXPECT_TRUE(update.robot_state.attached_collision_objects.empty());
    for (const auto & row : update.allowed_collision_matrix.entry_values) {
      EXPECT_TRUE(std::none_of(row.enabled.begin(), row.enabled.end(), [](bool value) {
        return value;
      }));
    }
  }
}

TEST_F(ApproachFixture, PickInsertsClosesAttachesAndLifts) {
  tcp_for_refinement("left");
  relaxed_pick_arrival = true;
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 60 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    complete();
    pick.tick();
  }
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  EXPECT_EQ(context->snapshot().phase, "PICK_COMPLETE");
  ASSERT_EQ(executor.steps.size(), 5u);
  EXPECT_EQ(executor.steps[2].name, "PICK_INSERT");
  EXPECT_EQ(executor.steps[2].type, "linear");
  EXPECT_EQ(executor.steps[3].name, "PICK_CLOSE");
  EXPECT_TRUE(executor.steps[3].gripper_wait_only);
  EXPECT_EQ(executor.steps[3].target, std::vector<double>{pc.close_target});
  EXPECT_EQ(executor.steps[4].name, "PICK_LIFT");
  EXPECT_EQ(executor.steps[4].type, "linear");
  // Pads end insert_depth past the cup axis along the measured heading, at grasp height.
  const auto & insert = executor.steps[2].target;
  geometry_msgs::msg::Quaternion q;
  q.x = insert[3];
  q.y = insert[4];
  q.z = insert[5];
  q.w = insert[6];
  const double yaw = *tcp_yaw(q);
  EXPECT_GT(std::abs(yaw), 5 * M_PI / 180); // Relaxed arrival heading, not the planned 0.
  const double past_axis = pc.insert_depth - pc.tcp_forward_offset;
  EXPECT_NEAR(insert[0], .54 + past_axis * std::cos(yaw), 1e-6);
  EXPECT_NEAR(insert[1], past_axis * std::sin(yaw), 1e-6);
  EXPECT_NEAR(insert[2], .84 - .165 / 2 + pc.grasp_height, 1e-6);
  EXPECT_NEAR(executor.steps[4].target[2], insert[2] + pc.lift_height, 1e-6);
  for (size_t i = 3; i < 7; ++i) {
    EXPECT_NEAR(executor.steps[4].target[i], insert[i], 1e-9); // Lift keeps hand orientation.
  }
  ASSERT_EQ(scene.scenes.size(), 3u);
  const auto & contact = scene.scenes[1].allowed_collision_matrix;
  ASSERT_EQ(contact.entry_names[0], "demo_place_cup");
  for (size_t i = 1; i < contact.entry_names.size(); ++i) {
    EXPECT_EQ(contact.entry_names[i].rfind("gripper_l_rh_p12_rn_", 0), 0u);
    EXPECT_TRUE(contact.entry_values[0].enabled[i]);
    EXPECT_TRUE(contact.entry_values[i].enabled[0]);
  }
  EXPECT_TRUE(scene.scenes[1].robot_state.attached_collision_objects.empty());
  const auto & attached = scene.scenes[2].robot_state.attached_collision_objects;
  ASSERT_EQ(attached.size(), 1u);
  EXPECT_EQ(attached[0].object.id, "demo_place_cup");
  EXPECT_EQ(attached[0].link_name, robot.arms.at("left").tcp);
  EXPECT_TRUE(attached[0].object.primitives.empty());
  EXPECT_NE(std::find(attached[0].touch_links.begin(), attached[0].touch_links.end(),
                "gripper_l_rh_p12_rn_base"),
      attached[0].touch_links.end());
  EXPECT_TRUE(scene.scenes[2].world.collision_objects.empty());
  EXPECT_FALSE(pick.blocks_other_motion());
}

class PlaceFixture : public ApproachFixture {
protected:
  void run_pick(PickCupMotion & pick) {
    std::string reason;
    ASSERT_TRUE(pick.start("left", "cup", reason, true)) << reason;
    for (int i = 0; i < 60 && pick.running(); ++i) {
      feedback(0);
      observe("head");
      complete();
      pick.tick();
    }
    ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  }
};

TEST_F(PlaceFixture, PlaceReversesTheRecordedPick) {
  tcp_for_refinement("left");
  relaxed_pick_arrival = true;
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  run_pick(pick);
  ASSERT_TRUE(pick.last_pick().has_value());
  const auto insert = executor.steps[2].target;
  const size_t steps = executor.steps.size(), scenes = scene.scenes.size();
  PlaceCupMotion place(robot, config, pc, *context, executor, scene, pick);
  std::string reason;
  EXPECT_FALSE(place.start("right", "cup", reason, true)); // The left hand picked the cup.
  ASSERT_TRUE(place.start("left", "cup", reason, true)) << reason;
  size_t steps_at_scene = 0;
  for (int i = 0; i < 60 && place.running(); ++i) {
    feedback(0);
    complete();
    place.tick();
    if (!steps_at_scene && scene.scenes.size() > scenes) {
      steps_at_scene = executor.steps.size() - steps;
    }
  }
  ASSERT_TRUE(place.result().sequence_completed) << place.result().detail;
  EXPECT_EQ(context->snapshot().phase, "PLACE_COMPLETE");
  ASSERT_EQ(executor.steps.size(), steps + 4);
  const auto & carry = executor.steps[steps];
  EXPECT_EQ(carry.name, "PLACE_CARRY");
  EXPECT_EQ(carry.type, "joint");
  EXPECT_EQ(carry.target, pick.last_pick()->lifted_joints);
  const auto & lower = executor.steps[steps + 1];
  EXPECT_EQ(lower.name, "PLACE_LOWER");
  EXPECT_EQ(lower.type, "linear");
  EXPECT_NEAR(lower.target[2], insert[2] + pc.place_clearance, 1e-6);
  for (size_t i : {0u, 1u, 3u, 4u, 5u, 6u}) {
    EXPECT_NEAR(lower.target[i], insert[i], 1e-6); // Same spot and hand orientation as the grasp.
  }
  EXPECT_EQ(executor.steps[steps + 2].name, "PLACE_OPEN");
  EXPECT_EQ(executor.steps[steps + 2].target, std::vector<double>{0.0});
  EXPECT_EQ(executor.steps[steps + 3].name, "PLACE_RETREAT");
  // The cup model leaves the hand only after the gripper opened.
  EXPECT_EQ(steps_at_scene, 3u);
  ASSERT_EQ(scene.scenes.size(), scenes + 1);
  const auto & placed = scene.scenes.back();
  ASSERT_EQ(placed.robot_state.attached_collision_objects.size(), 1u);
  EXPECT_EQ(placed.robot_state.attached_collision_objects[0].object.operation,
      placed.robot_state.attached_collision_objects[0].object.REMOVE);
  EXPECT_EQ(placed.robot_state.attached_collision_objects[0].link_name,
      robot.arms.at("left").tcp);
  const auto & acm = placed.allowed_collision_matrix;
  ASSERT_EQ(acm.entry_names[0], "demo_place_cup");
  EXPECT_NE(std::find(acm.entry_names.begin(), acm.entry_names.end(), "gripper_l_rh_p12_rn_base"),
      acm.entry_names.end());
}

TEST_F(PlaceFixture, PlaceLowerFailureNeverOpensTheGripper) {
  tcp_for_refinement("left");
  relaxed_pick_arrival = true;
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  run_pick(pick);
  const size_t steps = executor.steps.size(), scenes = scene.scenes.size();
  PlaceCupMotion place(robot, config, pc, *context, executor, scene, pick);
  std::string reason;
  ASSERT_TRUE(place.start("left", "cup", reason, true)) << reason;
  for (int i = 0; i < 40 && place.running(); ++i) {
    feedback(0);
    if (executor.steps.size() == steps + 2 && executor.current.state == ExecutionState::Running) {
      executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "table in the way"};
    } else {
      complete();
    }
    place.tick();
  }
  EXPECT_EQ(place.result().state, "FAILED");
  EXPECT_EQ(place.result().code, "PLANNING_FAILED");
  ASSERT_EQ(executor.steps.size(), steps + 2);
  EXPECT_EQ(executor.steps.back().name, "PLACE_LOWER");
  EXPECT_EQ(scene.scenes.size(), scenes); // The cup model stays in the hand.
}

TEST_F(PlaceFixture, PlaceNeedsARecordedPick) {
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  PlaceCupMotion place(robot, config, pc, *context, executor, scene, pick);
  std::string reason;
  EXPECT_FALSE(place.start("left", "cup", reason, true));
  EXPECT_NE(reason.find("No recorded pick"), std::string::npos);
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, PickFallsBackToClearAndLowerWhenDirectPtpFailsBeforeMotion) {
  tcp_for_refinement("left");
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  bool rejected = false;
  for (int i = 0; i < 80 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    if (!rejected && !executor.steps.empty() && executor.steps.back().name == "PICK_PREGRASP" &&
        executor.current.state == ExecutionState::Running) {
      executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "MoveIt error -31"};
      rejected = true;
    } else {
      complete();
    }
    pick.tick();
  }
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  ASSERT_EQ(executor.steps.size(), 7u);
  EXPECT_EQ(executor.steps[2].name, "PICK_CLEAR");
  EXPECT_EQ(executor.steps[2].type, "pose");
  EXPECT_EQ(executor.steps[3].name, "PICK_LOWER");
  EXPECT_EQ(executor.steps[3].type, "linear");
  EXPECT_EQ(executor.steps[4].name, "PICK_INSERT");
  // Clear waypoint sits straight above the pregrasp, clear_height above the rim.
  EXPECT_NEAR(executor.steps[2].target[0], executor.steps[1].target[0], 1e-9);
  EXPECT_NEAR(executor.steps[2].target[1], executor.steps[1].target[1], 1e-9);
  EXPECT_NEAR(executor.steps[2].target[2], .84 + .165 / 2 + pc.clear_height, 1e-9);
  EXPECT_EQ(executor.steps[3].target, executor.steps[1].target);
}

TEST_F(ApproachFixture, PickDoesNotFallBackAfterArmMoved) {
  tcp_for_refinement("left");
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 40 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    if (!executor.steps.empty() && executor.steps.back().name == "PICK_PREGRASP" &&
        executor.current.state == ExecutionState::Running) {
      sensor_msgs::msg::JointState moved;
      moved.header.stamp = node->now();
      moved.name = {"left_joint", "left_grip", "right_joint", "right_grip"};
      moved.position = {.1, 0, 0, 0};
      moved.velocity = {0, 0, 0, 0};
      context->observe_joints(moved);
      executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "controller aborted"};
      pick.tick();
      break;
    }
    complete();
    pick.tick();
  }
  EXPECT_EQ(pick.result().state, "FAILED");
  EXPECT_EQ(executor.steps.back().name, "PICK_PREGRASP");
}

TEST_F(ApproachFixture, PickInsertFailureNeverClosesGripper) {
  tcp_for_refinement("left");
  relaxed_pick_arrival = true;
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 60 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    if (!executor.steps.empty() && executor.steps.back().name == "PICK_INSERT" &&
        executor.current.state == ExecutionState::Running) {
      executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "no LIN path"};
    } else {
      complete();
    }
    pick.tick();
  }
  EXPECT_EQ(pick.result().state, "FAILED");
  EXPECT_EQ(pick.result().code, "PLANNING_FAILED");
  ASSERT_EQ(executor.steps.size(), 3u);
  EXPECT_EQ(executor.steps.back().name, "PICK_INSERT");
  for (const auto & update : scene.scenes) {
    EXPECT_TRUE(update.robot_state.attached_collision_objects.empty());
  }
}

TEST_F(ApproachFixture, PickContinuousRunsPregraspAndInsertAsOnePath) {
  tcp_for_refinement("left");
  initial_pose_at_zero();
  PickCupConfig pc;
  pc.continuous_insert = true;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  size_t scenes_at_start = 0;
  for (int i = 0; i < 60 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    if (!scenes_at_start && !executor.continuous.empty()) scenes_at_start = scene.scenes.size();
    complete();
    pick.tick();
  }
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  ASSERT_EQ(executor.continuous.size(), 1u);
  const auto & bundle = executor.continuous[0];
  ASSERT_EQ(bundle.size(), 2u);
  EXPECT_EQ(bundle[0].name, "PICK_PREGRASP");
  EXPECT_EQ(bundle[0].type, "pose");
  EXPECT_EQ(bundle[0].strict_objects, std::vector<std::string>{"demo_place_cup"});
  EXPECT_EQ(bundle[1].name, "PICK_INSERT");
  EXPECT_EQ(bundle[1].type, "linear");
  EXPECT_TRUE(bundle[1].strict_objects.empty());
  EXPECT_FALSE(bundle[0].endpoints_only);
  EXPECT_TRUE(bundle[1].endpoints_only); // Executed as a checked straight joint line.
  EXPECT_DOUBLE_EQ(bundle[1].tcp_line_tolerance, pc.insert_line_tolerance);
  EXPECT_DOUBLE_EQ(bundle[1].tcp_drop_tolerance, pc.insert_drop_tolerance);
  ASSERT_EQ(executor.steps.size(), 5u); // OPEN, bundle(PREGRASP, INSERT), CLOSE, LIFT
  EXPECT_EQ(executor.steps[0].name, "PICK_OPEN");
  EXPECT_EQ(executor.steps[3].name, "PICK_CLOSE");
  EXPECT_EQ(executor.steps[4].name, "PICK_LIFT");
  // Finger contact is allowed before the bundle is planned.
  ASSERT_EQ(scenes_at_start, 2u);
  const auto & contact = scene.scenes[1].allowed_collision_matrix;
  ASSERT_FALSE(contact.entry_values.empty());
  EXPECT_TRUE(contact.entry_values[0].enabled[1]);
  // Insert comes from the planned pregrasp heading, not a measured arrival.
  geometry_msgs::msg::Pose pregrasp;
  const auto & t = bundle[0].target;
  pregrasp.position.x = t[0];
  pregrasp.position.y = t[1];
  pregrasp.position.z = t[2];
  pregrasp.orientation.x = t[3];
  pregrasp.orientation.y = t[4];
  pregrasp.orientation.z = t[5];
  pregrasp.orientation.w = t[6];
  geometry_msgs::msg::Point rim;
  rim.x = .54;
  rim.z = .84 + config.cup_height / 2;
  const auto expected = cup_insert_pose(pregrasp, rim, config.cup_height, pc);
  ASSERT_TRUE(expected);
  EXPECT_NEAR(bundle[1].target[0], expected->position.x, 1e-9);
  EXPECT_NEAR(bundle[1].target[1], expected->position.y, 1e-9);
  EXPECT_NEAR(bundle[1].target[2], expected->position.z, 1e-9);
}

TEST_F(ApproachFixture, PickContinuousFallsBackToStepsWhenRejectedBeforeMotion) {
  tcp_for_refinement("left");
  initial_pose_at_zero();
  PickCupConfig pc;
  pc.continuous_insert = true;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  bool rejected = false;
  size_t scenes_at_reject = 0;
  for (int i = 0; i < 80 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    if (!rejected && executor.continuous.size() == 1 && executor.steps.size() == 3 &&
        executor.current.state == ExecutionState::Running) {
      executor.current = {ExecutionState::Failed, "PATH_PREPARATION_FAILED",
          "Smoothed path is in collision; gripper_l_rh_p12_rn_r2 / demo_place_cup"};
      rejected = true;
      scenes_at_reject = scene.scenes.size();
    } else {
      complete();
    }
    pick.tick();
  }
  ASSERT_TRUE(rejected);
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  ASSERT_EQ(executor.steps.size(), 7u);
  EXPECT_EQ(executor.steps[3].name, "PICK_PREGRASP");
  EXPECT_EQ(executor.steps[3].type, "pose");
  EXPECT_TRUE(executor.steps[3].strict_objects.empty());
  EXPECT_EQ(executor.steps[4].name, "PICK_INSERT");
  EXPECT_EQ(executor.steps[5].name, "PICK_CLOSE");
  // The separate PTP runs only after cup/finger checks are restored.
  ASSERT_GT(scene.scenes.size(), scenes_at_reject);
  const auto & restored = scene.scenes[scenes_at_reject];
  EXPECT_FALSE(restored.world.collision_objects.empty());
  for (const auto & row : restored.allowed_collision_matrix.entry_values) {
    EXPECT_TRUE(std::none_of(row.enabled.begin(), row.enabled.end(), [](bool value) {
      return value;
    }));
  }
}

TEST_F(ApproachFixture, PickContinuousOnlyFromTheInitialPose) {
  tcp_for_refinement("left");
  initial_pose_at_zero();
  arm_joint = .1; // Outside the initial pose tolerance.
  PickCupConfig pc;
  pc.continuous_insert = true;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 60 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    complete();
    pick.tick();
  }
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  EXPECT_TRUE(executor.continuous.empty());
  ASSERT_EQ(executor.steps.size(), 5u);
  EXPECT_EQ(executor.steps[1].name, "PICK_PREGRASP");
  EXPECT_EQ(executor.steps[2].name, "PICK_INSERT");
  EXPECT_FALSE(executor.steps[2].endpoints_only);
}

TEST_F(ApproachFixture, PickContinuousFallsBackWhenTheExecutorRefusesTheBundle) {
  tcp_for_refinement("left");
  executor.sequence_supported = false;
  initial_pose_at_zero();
  PickCupConfig pc;
  pc.continuous_insert = true;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 60 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    complete();
    pick.tick();
  }
  ASSERT_TRUE(pick.result().sequence_completed) << pick.result().detail;
  EXPECT_TRUE(executor.continuous.empty());
  ASSERT_EQ(executor.steps.size(), 5u);
  EXPECT_EQ(executor.steps[1].name, "PICK_PREGRASP");
  EXPECT_EQ(executor.steps[2].name, "PICK_INSERT");
}

TEST_F(ApproachFixture, PickContinuousFailureAfterMotionNeverClosesGripper) {
  tcp_for_refinement("left");
  initial_pose_at_zero();
  PickCupConfig pc;
  pc.continuous_insert = true;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  for (int i = 0; i < 60 && pick.running(); ++i) {
    feedback(0);
    observe("head");
    if (executor.continuous.size() == 1 && executor.steps.size() == 3 &&
        executor.current.state == ExecutionState::Running) {
      sensor_msgs::msg::JointState moved;
      moved.header.stamp = node->now();
      moved.name = {"left_joint", "left_grip", "right_joint", "right_grip"};
      moved.position = {.1, 0, 0, 0};
      moved.velocity = {0, 0, 0, 0};
      context->observe_joints(moved);
      executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "controller aborted"};
      pick.tick();
      break;
    }
    complete();
    pick.tick();
  }
  EXPECT_EQ(pick.result().state, "FAILED");
  EXPECT_EQ(pick.result().code, "EXECUTION_FAILED");
  EXPECT_EQ(executor.steps.size(), 3u);
  EXPECT_EQ(executor.steps.back().name, "PICK_INSERT");
}

TEST(PickGeometry, InsertKeepsMeasuredOrientationAndGraspHeight) {
  PickCupConfig c;
  geometry_msgs::msg::Pose pregrasp;
  tf2::Quaternion hand, axes;
  hand.setRPY(.05, .08, .3);
  axes.setRPY(0, -M_PI / 2, 0);
  pregrasp.orientation = tf2::toMsg(hand * axes);
  geometry_msgs::msg::Point rim;
  rim.x = .6;
  rim.y = .2;
  rim.z = 1.0;
  const auto p = cup_insert_pose(pregrasp, rim, .165, c);
  ASSERT_TRUE(p);
  const double yaw = *tcp_yaw(pregrasp.orientation);
  EXPECT_NEAR(p->position.x, .6 + (.03 + c.insert_depth) * std::cos(yaw), 1e-9);
  EXPECT_NEAR(p->position.y, .2 + (.03 + c.insert_depth) * std::sin(yaw), 1e-9);
  EXPECT_NEAR(p->position.z, .9, 1e-9);
  EXPECT_EQ(p->orientation, pregrasp.orientation);
  pregrasp.orientation = geometry_msgs::msg::Quaternion{};
  EXPECT_FALSE(cup_insert_pose(pregrasp, rim, .165, c));
  EXPECT_TRUE(c.validate(.165).empty());
  c.lift_height = .5;
  EXPECT_FALSE(c.validate(.165).empty());
  c.lift_height = .08;
  c.close_target = 1.5;
  EXPECT_FALSE(c.validate(.165).empty());
  c.close_target = .8;
  c.insert_depth = .05;
  EXPECT_FALSE(c.validate(.165).empty());
}

namespace {
Step taught(const std::string & name, const std::string & type, const std::string & arm,
    std::vector<double> target) {
  Step s;
  s.name = name;
  s.type = type;
  s.arm = arm;
  s.target = std::move(target);
  s.timeout = 30;
  s.frame = "base_link";
  return s;
}
} // namespace

class CupHandoverFixture : public ApproachFixture {
protected:
  void SetUp() override {
    ApproachFixture::SetUp();
    const std::vector<double> pose = {.38, -.12, .95, 0, 0, 0, 1};
    MotionSequence transfer, grasp;
    transfer.steps = {taught("cup_ready", "joint", "left", {.1})};
    grasp.steps = {taught("cup_insert", "linear", "right", pose),
        taught("receiver_close", "gripper", "right", {.8})};
    grasp.steps[1].duration = 1;
    grasp.steps[1].gripper_wait_only = true;
    MotionSequence release;
    release.steps = {taught("giver_release", "gripper", "left", {0.}),
        taught("cup_retreat", "linear", "left", {.38, .12, .95, 0, 0, 0, 1})};
    release.steps[0].duration = 1;
    robot.motion_sequences["cup_left_to_right"] = transfer;
    robot.motion_sequences["cup_left_to_right_grasp"] = grasp;
    robot.motion_sequences["cup_left_to_right_release"] = release;
  }

  void run(CupHandoverMotion & handover, size_t failing_after = 99) {
    size_t seen = 0;
    for (int i = 0; i < 80 && handover.running(); ++i) {
      feedback();
      if (executor.steps.size() >= failing_after &&
          executor.current.state == ExecutionState::Running) {
        executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "no path"};
      } else {
        complete();
      }
      handover.tick();
      while (seen < scene.scenes.size()) {
        steps_at_scene.push_back(executor.steps.size());
        ++seen;
      }
    }
  }

  std::vector<size_t> steps_at_scene;
};

TEST_F(CupHandoverFixture, TransfersCupModelBetweenCloseAndRelease) {
  CupHandoverMotion handover(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(handover.start("left", "cup", reason, true)) << reason;
  run(handover);
  ASSERT_TRUE(handover.result().sequence_completed) << handover.result().detail;
  EXPECT_EQ(context->snapshot().phase, "CUP_HANDOVER_COMPLETE");
  ASSERT_EQ(executor.steps.size(), 5u);
  const std::vector<std::string> expected = {
      "cup_ready", "cup_insert", "receiver_close", "giver_release", "cup_retreat"};
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(executor.steps[i].name, expected[i]);
  }
  // Contact is allowed only after the transfer move and before the insert; detach and attach
  // follow the receiver's close.
  ASSERT_EQ(scene.scenes.size(), 3u);
  EXPECT_EQ(steps_at_scene, (std::vector<size_t>{1, 3, 3}));
  const auto & contact = scene.scenes[0].allowed_collision_matrix;
  ASSERT_EQ(contact.entry_names[0], "demo_place_cup");
  for (size_t i = 1; i < contact.entry_names.size(); ++i) {
    EXPECT_EQ(contact.entry_names[i].rfind("gripper_r_rh_p12_rn_", 0), 0u);
    EXPECT_TRUE(contact.entry_values[0].enabled[i]);
  }
  EXPECT_NE(std::find(contact.entry_names.begin(), contact.entry_names.end(),
                "gripper_r_rh_p12_rn_base"),
      contact.entry_names.end()); // The wide cup model also touches the palm at a deep grasp.
  const auto & detach = scene.scenes[1].robot_state.attached_collision_objects;
  ASSERT_EQ(detach.size(), 1u);
  EXPECT_EQ(detach[0].object.operation, detach[0].object.REMOVE);
  EXPECT_EQ(detach[0].link_name, robot.arms.at("left").tcp);
  const auto & attach = scene.scenes[2].robot_state.attached_collision_objects;
  ASSERT_EQ(attach.size(), 1u);
  EXPECT_EQ(attach[0].object.id, "demo_place_cup");
  EXPECT_EQ(attach[0].link_name, robot.arms.at("right").tcp);
  EXPECT_TRUE(attach[0].object.primitives.empty());
  // The giver's fingers may still touch the cup while it lets go.
  const auto & acm = scene.scenes[2].allowed_collision_matrix;
  ASSERT_EQ(acm.entry_names[0], "demo_place_cup");
  for (size_t i = 1; i < acm.entry_names.size(); ++i) {
    EXPECT_EQ(acm.entry_names[i].rfind("gripper_l_rh_p12_rn_", 0), 0u);
    EXPECT_TRUE(acm.entry_values[0].enabled[i]);
  }
}

TEST_F(CupHandoverFixture, PreviewNeverMovesAndReportsInsertPose) {
  CupHandoverMotion handover(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(handover.start("left", "cup", reason, false)) << reason;
  handover.tick();
  EXPECT_TRUE(handover.result().sequence_completed) << handover.result().detail;
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
  EXPECT_NEAR(handover.approach_target().pose.position.y, -.12, 1e-9);
}

TEST_F(CupHandoverFixture, InsertFailureNeverClosesOrMovesTheCupModel) {
  CupHandoverMotion handover(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(handover.start("left", "cup", reason, true)) << reason;
  run(handover, 2);
  EXPECT_EQ(handover.result().state, "FAILED");
  EXPECT_EQ(handover.result().code, "PLANNING_FAILED");
  ASSERT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(executor.steps.back().name, "cup_insert");
  ASSERT_EQ(scene.scenes.size(), 1u); // Only the contact allowance; the cup stays on the giver.
  EXPECT_TRUE(scene.scenes[0].robot_state.attached_collision_objects.empty());
}

TEST_F(CupHandoverFixture, RejectsWrongGiverAndUnconfiguredSequences) {
  CupHandoverMotion handover(robot, config, *context, executor, scene, "left");
  std::string reason;
  EXPECT_FALSE(handover.start("right", "cup", reason, true));
  robot.motion_sequences.erase("cup_left_to_right_release");
  EXPECT_FALSE(handover.start("left", "cup", reason, true));
  EXPECT_NE(reason.find("cup_left_to_right_release"), std::string::npos);
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(CupHandoverFixture, CancelDuringSceneSwapStillAttachesBeforeStopping) {
  CupHandoverMotion handover(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(handover.start("left", "cup", reason, true)) << reason;
  for (int i = 0; i < 60 && scene.scenes.size() < 2; ++i) {
    scene.finish = scene.scenes.size() != 2; // Contact completes; the detach stays pending.
    feedback();
    complete();
    handover.tick();
  }
  ASSERT_EQ(scene.scenes.size(), 2u); // Detach is pending.
  handover.cancel();
  scene.finish = true;
  for (int i = 0; i < 10 && handover.running(); ++i) {
    feedback();
    handover.tick();
  }
  EXPECT_EQ(handover.result().state, "CANCELLED");
  ASSERT_EQ(scene.scenes.size(), 3u); // Attach still ran.
  EXPECT_EQ(scene.scenes[2].robot_state.attached_collision_objects[0].link_name,
      robot.arms.at("right").tcp);
  EXPECT_EQ(executor.steps.size(), 3u); // The giver never opened.
}


class B3HandoverFixture : public CupHandoverFixture {
protected:
  void SetUp() override {
    CupHandoverFixture::SetUp();
    robot.smooth_motion = true;
    Step via = taught("cup_ready_via", "dual_joint", "", {});
    via.left_target = {.1};
    via.right_target = {.2};
    via.velocity_scale = via.acceleration_scale = .3;
    via.pipeline = "ompl";
    via.planner = "RRTConnectkConfigDefault";
    via.tolerance = .05;
    via.planning_tolerance = .0005;
    via.blend_with_next = true;
    Step ready = via;
    ready.name = "cup_ready";
    ready.right_target = {.3};
    ready.tolerance = M_PI / 180;
    ready.blend_with_next = false;
    ready.hold_arm = "left";
    ready.hold_tolerance = .001;
    robot.motion_sequences.at("cup_left_to_right").steps = {via, ready};
  }
};

TEST_F(B3HandoverFixture, SendsBothPreparationStepsTogetherBeforeContactAndGrasp) {
  CupHandoverMotion motion(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(motion.start("left", "cup", reason)) << reason;
  run(motion);
  ASSERT_TRUE(motion.result().sequence_completed) << motion.result().detail;
  ASSERT_EQ(executor.continuous.size(), 1u);
  const auto & stages = executor.continuous.front();
  ASSERT_EQ(stages.size(), 2u);
  EXPECT_EQ(stages[0].name, "cup_ready_via");
  EXPECT_EQ(stages[1].name, "cup_ready");
  EXPECT_EQ(stages[0].left_target, stages[1].left_target);
  EXPECT_EQ(stages[0].right_target, (std::vector<double>{.2}));
  EXPECT_EQ(stages[1].right_target, (std::vector<double>{.3}));
  for (const auto & step : stages) {
    EXPECT_EQ(step.type, "dual_joint");
    EXPECT_TRUE(step.gripper_target.empty());
    EXPECT_EQ(step.pipeline, "ompl");
    EXPECT_EQ(step.planner, "RRTConnectkConfigDefault");
    EXPECT_DOUBLE_EQ(step.velocity_scale, .3);
    EXPECT_DOUBLE_EQ(step.acceleration_scale, .3);
  }
  EXPECT_EQ(steps_at_scene, (std::vector<size_t>{2, 4, 4}));
  ASSERT_EQ(executor.steps.size(), 6u);
  EXPECT_EQ(executor.steps[2].name, "cup_insert");
  EXPECT_EQ(executor.steps[3].name, "receiver_close");
  EXPECT_EQ(executor.steps[4].name, "giver_release");
  EXPECT_EQ(executor.steps[5].name, "cup_retreat");
}

TEST_F(B3HandoverFixture, PreparationFailureNeverInsertsClosesOrSwapsTheCup) {
  CupHandoverMotion motion(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(motion.start("left", "cup", reason)) << reason;
  run(motion, 2);
  EXPECT_EQ(motion.result().state, "FAILED");
  EXPECT_EQ(executor.continuous.size(), 1u);
  EXPECT_EQ(executor.steps.size(), 2u);
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(B3HandoverFixture, CancelDuringPreparationNeverInsertsClosesOrSwapsTheCup) {
  CupHandoverMotion motion(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(motion.start("left", "cup", reason)) << reason;
  for (int i = 0; i < 10 && executor.continuous.empty(); ++i) {
    feedback();
    motion.tick();
  }
  ASSERT_EQ(executor.continuous.size(), 1u);
  motion.cancel();
  run(motion);
  EXPECT_EQ(motion.result().state, "CANCELLED");
  EXPECT_EQ(executor.steps.size(), 2u);
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(B3HandoverFixture, PreviewReportsOnlyInsertPoseWithoutPlanningPreparation) {
  CupHandoverMotion motion(robot, config, *context, executor, scene, "left");
  std::string reason;
  ASSERT_TRUE(motion.start("left", "cup", reason, false)) << reason;
  motion.tick();
  EXPECT_TRUE(motion.result().sequence_completed);
  EXPECT_TRUE(executor.continuous.empty());
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

class B2HandoverFixture : public CupHandoverFixture {
protected:
  void SetUp() override {
    CupHandoverFixture::SetUp();
    config.other_arm_return_sequence = "right_return";
    MotionSequence transfer, grasp, release, returning;
    transfer.steps = {taught("receiver_open", "gripper", "left", {0.})};
    grasp.steps = {taught("cup_insert", "linear", "left", {.38, -.05, .95, 0, 0, 0, 1}),
        taught("receiver_close", "gripper", "left", {.8})};
    release.steps = {taught("giver_release", "gripper", "right", {0.}),
        taught("cup_retreat", "linear", "right", {.38, -.16, 1., 0, 0, 0, 1}),
        taught("giver_via", "joint", "right", {.1}),
        taught("giver_home", "joint", "right", {.2})};
    returning.steps = {taught("giver_home_via", "joint", "right", {.3}),
        taught("giver_home", "joint", "right", {.2})};
    robot.motion_sequences["cup_right_to_left"] = transfer;
    robot.motion_sequences["cup_right_to_left_grasp"] = grasp;
    robot.motion_sequences["cup_right_to_left_release"] = release;
    robot.motion_sequences["right_return"] = returning;
  }
};

TEST_F(B2HandoverFixture, IndependentReleaseStopsAtRetreatWithoutChangingTheConfiguredSequence) {
  CupHandoverMotion back(robot, config, *context, executor, scene, "right");
  std::string reason;
  ASSERT_TRUE(back.start("right", "cup", reason)) << reason;
  run(back);
  ASSERT_TRUE(back.result().sequence_completed) << back.result().detail;
  std::vector<std::string> names;
  for (const auto & step : executor.steps) names.push_back(step.name);
  EXPECT_EQ(names, (std::vector<std::string>{"receiver_open", "cup_insert", "receiver_close",
      "giver_release", "cup_retreat"}));
  EXPECT_EQ(back.result().last_completed_step, "cup_retreat");
  EXPECT_EQ(robot.motion_sequences.at("cup_right_to_left_release").steps.size(), 4u);
  ASSERT_EQ(scene.scenes.size(), 3u);
  EXPECT_EQ(steps_at_scene, (std::vector<size_t>{1, 3, 3}));
  EXPECT_EQ(scene.scenes.back().robot_state.attached_collision_objects[0].link_name,
      robot.arms.at("left").tcp);
}

TEST_F(B2HandoverFixture, PairedModeRetainsBothLegacyReturnSteps) {
  config.independent_return = false;
  config.other_arm_return_sequence.clear();
  robot.motion_sequences.erase("right_return");
  CupHandoverMotion back(robot, config, *context, executor, scene, "right");
  std::string reason;
  ASSERT_TRUE(back.start("right", "cup", reason)) << reason;
  run(back);
  ASSERT_TRUE(back.result().sequence_completed) << back.result().detail;
  ASSERT_EQ(executor.steps.size(), 7u);
  EXPECT_EQ(executor.steps[5].name, "giver_via");
  EXPECT_EQ(executor.steps[6].name, "giver_home");
}

TEST_F(B2HandoverFixture, MissingIndependentReturnConfigurationRejectsBeforeAnyMotion) {
  CupHandoverMotion back(robot, config, *context, executor, scene, "right");
  std::string reason;
  config.other_arm_return_sequence.clear();
  EXPECT_FALSE(back.start("right", "cup", reason));
  config.other_arm_return_sequence = "right_return";
  robot.motion_sequences.erase("right_return");
  EXPECT_FALSE(back.start("right", "cup", reason));
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(B2HandoverFixture, PlaceReturnSequenceMustBeALoadedRightJointSequence) {
  CupHandoverMotion back(robot, config, *context, executor, scene, "right");
  std::string reason;
  config.place_return_sequence = "cup_right_return";
  EXPECT_FALSE(back.is_ready_to_start("right", "cup", reason)); // Not loaded.
  EXPECT_NE(reason.find("cup_right_return"), std::string::npos) << reason;
  MotionSequence direct;
  direct.steps = {taught("giver_home", "joint", "right", {.2})};
  robot.motion_sequences["cup_right_return"] = direct;
  EXPECT_TRUE(independent_return_error(robot, config).empty());
  reason.clear();
  EXPECT_TRUE(back.is_ready_to_start("right", "cup", reason)) << reason;
  robot.motion_sequences["cup_right_return"].steps.front().arm = "left";
  EXPECT_FALSE(independent_return_error(robot, config).empty());
  robot.motion_sequences["cup_right_return"].steps = {
      taught("cup_retreat", "linear", "right", {.38, -.16, 1., 0, 0, 0, 1})};
  EXPECT_FALSE(independent_return_error(robot, config).empty());
  EXPECT_FALSE(back.is_ready_to_start("right", "cup", reason));
  config.place_return_sequence.clear(); // Back to right_return only.
  EXPECT_TRUE(independent_return_error(robot, config).empty());
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(B2HandoverFixture, PreviewWithoutReturnConfigurationDoesNotMoveOrSwapTheCup) {
  config.other_arm_return_sequence.clear();
  CupHandoverMotion back(robot, config, *context, executor, scene, "right");
  std::string reason;
  ASSERT_TRUE(back.start("right", "cup", reason, false)) << reason;
  back.tick();
  EXPECT_TRUE(back.result().sequence_completed);
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
}

TEST_F(CupHandoverFixture, RightGiverHandsTheCupBackToTheLeft) {
  config.independent_return = false;
  MotionSequence open, grasp, release;
  open.steps = {taught("receiver_open", "gripper", "left", {0.})};
  open.steps[0].duration = 1;
  grasp.steps = {taught("cup_insert", "linear", "left", {.38, -.05, .95, 0, 0, 0, 1}),
      taught("receiver_close", "gripper", "left", {.8})};
  grasp.steps[1].duration = 1;
  grasp.steps[1].gripper_wait_only = true;
  release.steps = {taught("giver_release", "gripper", "right", {0.}),
      taught("cup_retreat", "linear", "right", {.38, -.16, 1., 0, 0, 0, 1}),
      taught("giver_home", "joint", "right", {.2})};
  release.steps[0].duration = 1;
  robot.motion_sequences["cup_right_to_left"] = open;
  robot.motion_sequences["cup_right_to_left_grasp"] = grasp;
  robot.motion_sequences["cup_right_to_left_release"] = release;
  CupHandoverMotion back(robot, config, *context, executor, scene, "right");
  std::string reason;
  EXPECT_FALSE(back.start("left", "cup", reason, true)); // The right hand holds the cup.
  ASSERT_TRUE(back.start("right", "cup", reason, true)) << reason;
  size_t seen = 0;
  for (int i = 0; i < 80 && back.running(); ++i) {
    feedback();
    complete();
    back.tick();
    while (seen < scene.scenes.size()) {
      steps_at_scene.push_back(executor.steps.size());
      ++seen;
    }
  }
  ASSERT_TRUE(back.result().sequence_completed) << back.result().detail;
  const std::vector<std::string> expected = {
      "receiver_open", "cup_insert", "receiver_close", "giver_release", "cup_retreat", "giver_home"};
  ASSERT_EQ(executor.steps.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(executor.steps[i].name, expected[i]);
  }
  ASSERT_EQ(scene.scenes.size(), 3u);
  EXPECT_EQ(steps_at_scene, (std::vector<size_t>{1, 3, 3}));
  const auto & contact = scene.scenes[0].allowed_collision_matrix;
  for (size_t i = 1; i < contact.entry_names.size(); ++i) {
    EXPECT_EQ(contact.entry_names[i].rfind("gripper_l_rh_p12_rn_", 0), 0u);
  }
  EXPECT_EQ(scene.scenes[1].robot_state.attached_collision_objects[0].link_name,
      robot.arms.at("right").tcp);
  EXPECT_EQ(scene.scenes[2].robot_state.attached_collision_objects[0].link_name,
      robot.arms.at("left").tcp);
}

TEST(PourGeometry, MatchesTheGeneratorAtTheTestedBasket) {
  // scripts/generate_cup_pour_targets.py at basket 0.412 -0.247 (robot-tested 2026-11-11).
  geometry_msgs::msg::Pose tcp;
  tcp.position.x = .38;
  tcp.position.y = .04;
  tcp.position.z = 1.0;
  tcp.orientation.x = .5;
  tcp.orientation.y = -.5;
  tcp.orientation.z = .5;
  tcp.orientation.w = .5;
  geometry_msgs::msg::Pose basket; // Long side along base y, as measured that day.
  basket.position.x = .412;
  basket.position.y = -.247;
  basket.position.z = .725;
  basket.orientation.z = std::sin(M_PI / 4);
  basket.orientation.w = std::cos(M_PI / 4);
  PourConfig pc;
  pc.clearance = .05; // The generator run used 5 cm.
  std::string reason;
  const auto plan = plan_pour(tcp, basket, .165, pc, reason);
  ASSERT_TRUE(plan) << reason;
  EXPECT_NEAR(plan->lowered.position.x, .451994, 2e-3);
  EXPECT_NEAR(plan->lowered.position.y, -.303704, 2e-3);
  EXPECT_NEAR(plan->lowered.position.z, .917349, 2e-3);
  EXPECT_NEAR(plan->raise, .037417, 2e-3);
  EXPECT_NEAR(plan->above.position.z, plan->lowered.position.z + plan->raise, 1e-9);
  const auto & q = plan->tilted.orientation;
  const double sign = q.w < 0 ? -1 : 1;
  EXPECT_NEAR(sign * q.x, -.640855, 1e-3);
  EXPECT_NEAR(sign * q.y, -.298845, 1e-3);
  EXPECT_NEAR(sign * q.z, .640854, 1e-3);
  EXPECT_NEAR(sign * q.w, .298835, 1e-3);
  EXPECT_FALSE(plan->clamped);
  // The pour follows the measured basket one to one.
  basket.position.x += .05;
  basket.position.y -= .02;
  const auto moved = plan_pour(tcp, basket, .165, pc, reason);
  ASSERT_TRUE(moved) << reason;
  EXPECT_NEAR(moved->lowered.position.x - plan->lowered.position.x, .05, 1e-9);
  EXPECT_NEAR(moved->lowered.position.y - plan->lowered.position.y, -.02, 1e-9);
  // The default 9 cm keeps the upright cup above the walls: no lowering into the basket.
  const auto high = plan_pour(tcp, basket, .165, PourConfig{}, reason);
  ASSERT_TRUE(high) << reason;
  EXPECT_EQ(high->raise, 0.0);
  EXPECT_NEAR(high->lowered.position.z, plan->lowered.position.z + .04, 1e-9);
  // A tilted hand means the cup is not held as the handover leaves it.
  tcp.orientation = geometry_msgs::msg::Quaternion{};
  EXPECT_FALSE(plan_pour(tcp, basket, .165, pc, reason));
}

TEST(PourGeometry, KeepsTheLipInsideARotatedBasket) {
  geometry_msgs::msg::Pose tcp;
  tcp.position.x = .38;
  tcp.position.y = .04;
  tcp.position.z = 1.0;
  tcp.orientation.x = .5;
  tcp.orientation.y = -.5;
  tcp.orientation.z = .5;
  tcp.orientation.w = .5;
  geometry_msgs::msg::Pose basket; // Long side along base x: 0.21 inside along y (robot 2026-11-11).
  basket.position.x = .512;
  basket.position.y = -.264;
  basket.position.z = .727;
  basket.orientation.w = 1;
  PourConfig pc;
  std::string reason;
  auto plan = plan_pour(tcp, basket, .165, pc, reason);
  ASSERT_TRUE(plan) << reason;
  EXPECT_FALSE(plan->clamped);
  EXPECT_NEAR(plan->lip.y, -.314, 1e-6);
  EXPECT_NEAR(plan->lowered.position.x, .551, 2e-3); // Robot-checked to tilt 130 deg there.
  EXPECT_NEAR(plan->lowered.position.y, -.321, 2e-3);
  pc.shift = -.12; // Past the rim: pulled back to inner_margin inside it.
  plan = plan_pour(tcp, basket, .165, pc, reason);
  ASSERT_TRUE(plan) << reason;
  EXPECT_TRUE(plan->clamped);
  EXPECT_NEAR(plan->lip.y, -.264 - (pc.inner_width / 2 - pc.inner_margin), 1e-6);
}

class CupPourFixture : public ApproachFixture {
protected:
  void SetUp() override {
    ApproachFixture::SetUp();
    MotionSequence grasp; // The pour checks the right TCP against this transfer hold.
    grasp.steps = {taught("cup_insert", "linear", "right", {.38, .04, 1., .5, -.5, .5, .5})};
    robot.motion_sequences["cup_left_to_right_grasp"] = grasp;
    hold_tcp();
  }

  void hold_tcp(double dx = 0) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = robot.planning_frame;
    tf.header.stamp = node->now();
    tf.child_frame_id = robot.arms.at("right").tcp;
    tf.transform.translation.x = .38 + dx;
    tf.transform.translation.y = .04;
    tf.transform.translation.z = 1.;
    tf.transform.rotation.x = .5;
    tf.transform.rotation.y = -.5;
    tf.transform.rotation.z = .5;
    tf.transform.rotation.w = .5;
    broadcaster->sendTransform(tf);
    for (int i = 0; i < 3; ++i) {
      ros.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  void basket(double x = .412) {
    geometry_msgs::msg::PoseStamped b;
    b.header.frame_id = robot.planning_frame;
    b.pose.position.x = x;
    b.pose.position.y = -.247;
    b.pose.position.z = .725;
    b.pose.orientation.w = 1;
    context->observe_basket(b);
  }

  void run(CupPourMotion & pour, size_t failing_after = 99) {
    for (int i = 0; i < 60 && pour.running(); ++i) {
      feedback();
      if (executor.steps.size() >= failing_after &&
          executor.current.state == ExecutionState::Running) {
        executor.current = {ExecutionState::Failed, "PLANNING_FAILED", "no path"};
      } else {
        complete();
      }
      pour.tick();
    }
  }

  PourConfig pc;
};

TEST_F(CupPourFixture, PoursOverTheMeasuredBasketWithoutSceneChanges) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason, true)) << reason;
  run(pour);
  ASSERT_TRUE(pour.result().sequence_completed) << pour.result().detail;
  ASSERT_EQ(executor.round_trips.size(), 1u);
  EXPECT_EQ(executor.round_trips.front().size(), 3u);
  EXPECT_EQ(context->snapshot().phase, "CUP_POUR_COMPLETE");
  // 9 cm above the walls: the hand goes straight to the pour height, no lower/raise.
  const std::vector<std::string> expected = {
      "pour_above", "pour_tilt", "pour_hold", "pour_untilt", "pour_return"};
  ASSERT_EQ(executor.steps.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(executor.steps[i].name, expected[i]);
  }
  EXPECT_EQ(executor.steps[0].type, "pose"); // Pilz PTP to the turned hand over the basket.
  EXPECT_NEAR(executor.steps[1].target[0], .451994, 2e-3);
  EXPECT_NEAR(executor.steps[1].target[1], -.303704, 2e-3);
  EXPECT_EQ(executor.steps[2].duration, pc.hold_duration);
  EXPECT_EQ(executor.steps[4].type, "joint");
  EXPECT_EQ(executor.steps[4].target, std::vector<double>{0.0}); // Start joints.
  EXPECT_TRUE(scene.scenes.empty()); // The cup stays attached to the right hand.
}

TEST_F(CupPourFixture, PreviewNeverMovesAndFollowsTheBasket) {
  basket(.462);
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason, false)) << reason;
  pour.tick();
  EXPECT_TRUE(pour.result().sequence_completed) << pour.result().detail;
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_NEAR(pour.approach_target().pose.position.x, .451994 + .05, 2e-3);
}

TEST_F(CupPourFixture, TiltPlanFailureRetriesSmallerAngles) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason, true)) << reason;
  int tilts = 0;
  for (int i = 0; i < 80 && pour.running(); ++i) {
    feedback();
    const bool tilting = executor.current.state == ExecutionState::Running &&
                         executor.steps.back().name == "pour_tilt";
    if (tilting && ++tilts == 1) {
      executor.current = {ExecutionState::Failed, "ROUND_TRIP_PLAN_FAILED", "MoveIt error 99999",
          "pour_tilt", {}, true};
    } else {
      complete();
    }
    pour.tick();
  }
  ASSERT_TRUE(pour.result().sequence_completed) << pour.result().detail;
  EXPECT_NE(pour.result().detail.find("poured at 115"), std::string::npos) << pour.result().detail;
  std::vector<std::string> names;
  for (const auto & step : executor.steps) names.push_back(step.name);
  EXPECT_EQ(names, (std::vector<std::string>{"pour_above", "pour_tilt", "pour_tilt", "pour_hold",
                       "pour_untilt", "pour_return"}));
  EXPECT_NE(executor.steps[1].target, executor.steps[2].target); // Less steep second tilt.
}

TEST_F(CupPourFixture, TiltStopsWhenEveryAngleFails) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason, true)) << reason;
  std::vector<std::string> failures;
  for (int i = 0; i < 80 && pour.running(); ++i) {
    feedback();
    if (executor.current.state == ExecutionState::Running &&
        executor.steps.back().name == "pour_tilt") {
      failures.push_back("candidate " + std::to_string(failures.size() + 1) +
                         ": MoveIt error 99999");
      executor.current = {ExecutionState::Failed, "ROUND_TRIP_PLAN_FAILED", failures.back(),
          "pour_tilt", {}, true};
    } else {
      complete();
    }
    pour.tick();
  }
  EXPECT_EQ(pour.result().state, "FAILED");
  ASSERT_EQ(executor.steps.size(), 1u + 1u + pc.fallback_angles_deg.size());
  EXPECT_EQ(executor.steps.back().name, "pour_tilt"); // Never holds or returns.
  ASSERT_EQ(failures.size(), 1u + pc.fallback_angles_deg.size());
  size_t next = 0;
  for (const auto & failure : failures) {
    const auto position = pour.result().detail.find(failure, next);
    ASSERT_NE(position, std::string::npos) << pour.result().detail;
    next = position + failure.size();
  }
}

TEST_F(CupPourFixture, TiltFailureAfterTheArmMovedIsNotRetried) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason, true)) << reason;
  for (int i = 0; i < 80 && pour.running(); ++i) {
    feedback();
    if (executor.current.state == ExecutionState::Running &&
        executor.steps.back().name == "pour_tilt") {
      sensor_msgs::msg::JointState moved; // The right arm turned before the failure.
      moved.header.stamp = node->now();
      moved.name = {robot.arms.at("right").joints[0]};
      moved.position = {.2};
      moved.velocity = {0};
      context->observe_joints(moved);
      executor.current = {ExecutionState::Failed, "ROUND_TRIP_PLAN_FAILED", "candidate failed",
          "pour_tilt", {}, true};
      pour.tick();
      break;
    }
    complete();
    pour.tick();
  }
  EXPECT_EQ(pour.result().state, "FAILED");
  EXPECT_EQ(executor.steps.size(), 2u); // pour_above, one pour_tilt.
}

TEST_F(CupPourFixture, RejectsWithoutBasketAwayFromHoldOrWithTheLeftHand) {
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  EXPECT_FALSE(pour.start("right", "cup", reason, true));
  EXPECT_NE(reason.find("No basket"), std::string::npos) << reason;
  basket();
  EXPECT_FALSE(pour.start("left", "cup", reason, true));
  hold_tcp(.05);
  EXPECT_FALSE(pour.start("right", "cup", reason, true));
  EXPECT_NE(reason.find("transfer hold"), std::string::npos) << reason;
  EXPECT_TRUE(executor.steps.empty());
}

TEST(PourConfig, IndependentTiltScalesHaveDefaultsAndLoadReadOnlyOverrides) {
  PourConfig defaults;
  EXPECT_DOUBLE_EQ(defaults.tilt_speed, .60);
  EXPECT_DOUBLE_EQ(defaults.tilt_acceleration, .30);
  EXPECT_DOUBLE_EQ(defaults.ptp_speed, .40);
  EXPECT_DOUBLE_EQ(defaults.lin_speed, .08);
  EXPECT_DOUBLE_EQ(defaults.hold_duration, 2.0);
  auto default_node = std::make_shared<rclcpp::Node>("pour_tilt_defaults");
  const auto loaded = load_pour_config(*default_node);
  EXPECT_DOUBLE_EQ(loaded.tilt_speed, defaults.tilt_speed);
  EXPECT_DOUBLE_EQ(loaded.tilt_acceleration, defaults.tilt_acceleration);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("pour.tilt_speed", .23),
      rclcpp::Parameter("pour.tilt_acceleration", .11),
      rclcpp::Parameter("pour.ptp_speed", .35)});
  auto node = std::make_shared<rclcpp::Node>("pour_tilt_override", options);
  const auto configured = load_pour_config(*node);
  EXPECT_DOUBLE_EQ(configured.tilt_speed, .23);
  EXPECT_DOUBLE_EQ(configured.tilt_acceleration, .11);
  EXPECT_DOUBLE_EQ(configured.lin_speed, defaults.lin_speed);
  EXPECT_DOUBLE_EQ(configured.ptp_speed, .35);
  EXPECT_TRUE(node->describe_parameter("pour.tilt_speed").read_only);
  EXPECT_TRUE(node->describe_parameter("pour.tilt_acceleration").read_only);
}

TEST(PourConfig, RejectsInvalidTiltScalesDuringValidationAndLoading) {
  for (const std::string field : {"tilt_speed", "tilt_acceleration"}) {
    for (double value : {0., -.1, 1.01, std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::quiet_NaN()}) {
      PourConfig config;
      (field == "tilt_speed" ? config.tilt_speed : config.tilt_acceleration) = value;
      EXPECT_FALSE(config.validate().empty()) << field << "=" << value;
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("pour." + field, value)});
      auto node = std::make_shared<rclcpp::Node>("pour_tilt_invalid", options);
      EXPECT_THROW(load_pour_config(*node), std::invalid_argument);
    }
  }
}

TEST_F(CupPourFixture, TiltScalesAreIndependentAndLowerRaiseAndPtpAreUnchanged) {
  basket();
  pc.clearance = .05;
  pc.tilt_speed = .23;
  pc.tilt_acceleration = .11;
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  run(pour);
  ASSERT_TRUE(pour.result().sequence_completed) << pour.result().detail;
  ASSERT_EQ(executor.round_trips.size(), 1u);
  for (const auto & step : executor.steps) {
    if (step.name == "pour_tilt" || step.name == "pour_untilt") {
      EXPECT_DOUBLE_EQ(step.velocity_scale, pc.tilt_speed);
      EXPECT_DOUBLE_EQ(step.acceleration_scale, pc.tilt_acceleration);
    } else if (step.name == "pour_lower" || step.name == "pour_raise") {
      EXPECT_DOUBLE_EQ(step.velocity_scale, pc.lin_speed);
      EXPECT_DOUBLE_EQ(step.acceleration_scale, pc.lin_speed / 4);
    } else if (step.name == "pour_above" || step.name == "pour_return") {
      EXPECT_DOUBLE_EQ(step.velocity_scale, pc.ptp_speed);
      EXPECT_DOUBLE_EQ(step.acceleration_scale, pc.ptp_speed);
    }
  }
  const std::vector<std::string> expected = {"pour_above", "pour_lower", "pour_tilt",
      "pour_hold", "pour_untilt", "pour_raise", "pour_return"};
  std::vector<std::string> names;
  for (const auto & step : executor.steps) names.push_back(step.name);
  EXPECT_EQ(names, expected);
  EXPECT_DOUBLE_EQ(executor.round_trips.front()[1].duration, pc.hold_duration);
}

TEST_F(CupPourFixture, WholeDurationIncludesAcceptanceApproachAndFinalReturnWithoutA15sTimer) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  pour.request_accepted(Clock::now() - std::chrono::seconds(20));
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  for (int i = 0; i < 60 && executor.steps.empty(); ++i) {
    feedback();
    pour.tick();
  }
  ASSERT_EQ(executor.steps.back().name, "pour_above");
  EXPECT_FALSE(pour.total_duration().has_value());
  for (int i = 0; i < 60 && executor.steps.back().name != "pour_return"; ++i) {
    feedback();
    complete();
    pour.tick();
  }
  ASSERT_EQ(executor.steps.back().name, "pour_return");
  EXPECT_TRUE(pour.running());
  EXPECT_FALSE(pour.total_duration().has_value());
  run(pour);
  ASSERT_TRUE(pour.result().sequence_completed) << pour.result().detail;
  ASSERT_TRUE(pour.total_duration().has_value());
  EXPECT_GE(*pour.total_duration(), 20.0);
  const auto completed = pour.total_duration();
  pour.tick();
  pour.cancel();
  EXPECT_EQ(pour.total_duration(), completed);
}

TEST_F(CupPourFixture, WholeDurationIsRecordedOnceForCancellation) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  pour.request_accepted(Clock::now() - std::chrono::seconds(20));
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  pour.cancel();
  EXPECT_EQ(pour.result().state, "CANCELLED");
  ASSERT_TRUE(pour.total_duration().has_value());
  EXPECT_GE(*pour.total_duration(), 20.0);
  const auto completed = pour.total_duration();
  pour.tick();
  pour.cancel();
  EXPECT_EQ(pour.total_duration(), completed);
}

TEST_F(CupPourFixture, WholeDurationIsRecordedOnceForApproachFailure) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  pour.request_accepted(Clock::now() - std::chrono::seconds(20));
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  for (int i = 0; i < 60 && executor.steps.empty(); ++i) {
    feedback();
    pour.tick();
  }
  executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "approach failed"};
  pour.tick();
  EXPECT_EQ(pour.result().state, "FAILED");
  ASSERT_TRUE(pour.total_duration().has_value());
  EXPECT_GE(*pour.total_duration(), 20.0);
  const auto completed = pour.total_duration();
  pour.tick();
  EXPECT_EQ(pour.total_duration(), completed);
}

TEST_F(CupPourFixture, ExecutionFailureWithoutArmDisplacementDoesNotRetry) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  for (int i = 0; i < 60 && executor.round_trips.empty(); ++i) {
    feedback();
    complete();
    pour.tick();
  }
  ASSERT_EQ(executor.round_trips.size(), 1u);
  executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "tilt execution failed",
      "pour_tilt"};
  pour.tick();
  EXPECT_EQ(pour.result().state, "FAILED");
  EXPECT_FALSE(pour.result().sequence_completed);
  EXPECT_EQ(executor.round_trips.size(), 1u);
  EXPECT_EQ(executor.steps.size(), 2u);
  EXPECT_EQ(pour.result().last_completed_step, "pour_above");
}

TEST_F(CupPourFixture, ReturnFailurePreservesHoldAndDoesNotRaiseOrReturn) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  for (int i = 0; i < 60 && executor.steps.empty(); ++i) pour.tick();
  for (int i = 0; i < 60 && executor.steps.back().name != "pour_untilt"; ++i) {
    feedback();
    complete();
    pour.tick();
  }
  ASSERT_EQ(executor.steps.back().name, "pour_untilt");
  EXPECT_TRUE(pour.running());
  EXPECT_FALSE(pour.result().sequence_completed);
  executor.current = {ExecutionState::Failed, "EXECUTION_FAILED", "return execution failed",
      "pour_untilt", "pour_hold"};
  pour.tick();
  EXPECT_EQ(pour.result().state, "FAILED");
  EXPECT_FALSE(pour.result().sequence_completed);
  EXPECT_EQ(pour.result().last_completed_step, "pour_hold");
  EXPECT_EQ(context->snapshot().phase, "pour_untilt");
  EXPECT_EQ(executor.round_trips.size(), 1u);
  EXPECT_EQ(executor.steps.back().name, "pour_untilt");
}

TEST_F(CupPourFixture, StopUnconfirmedDuringRoundTripStaysLatched) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  for (int i = 0; i < 60 && executor.round_trips.empty(); ++i) {
    feedback();
    complete();
    pour.tick();
  }
  ASSERT_EQ(executor.round_trips.size(), 1u);
  pour.cancel();
  executor.current = {ExecutionState::StopUnconfirmed, "STOP_UNCONFIRMED", "stop not confirmed",
      "pour_tilt"};
  pour.tick();
  EXPECT_EQ(pour.result().state, "FAULT");
  EXPECT_EQ(pour.result().code, "STOP_UNCONFIRMED");
  EXPECT_TRUE(pour.blocks_other_motion());
  EXPECT_FALSE(pour.start("right", "cup", reason));
  EXPECT_EQ(executor.round_trips.size(), 1u);
}

TEST_F(CupPourFixture, CancelDuringRoundTripNeverStartsTheAfterSequence) {
  basket();
  CupPourMotion pour(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pour.start("right", "cup", reason));
  for (int i = 0; i < 60 && executor.round_trips.empty(); ++i) {
    feedback();
    complete();
    pour.tick();
  }
  ASSERT_EQ(executor.round_trips.size(), 1u);
  pour.cancel();
  pour.tick();
  EXPECT_EQ(pour.result().state, "CANCELLED");
  EXPECT_FALSE(pour.result().sequence_completed);
  EXPECT_EQ(executor.steps.size(), 2u);
}

TEST_F(ApproachFixture, PickCancelledDuringObservationDoesNotOpenGripper) {
  tcp_for_refinement();
  shoulder_for_pick();
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  std::string reason;
  ASSERT_TRUE(pick.start("left", "cup", reason, true));
  pick.tick();
  pick.cancel();
  pick.tick();
  EXPECT_EQ(pick.result().state, "CANCELLED");
  EXPECT_TRUE(executor.steps.empty());
}

TEST_F(ApproachFixture, ActionDefaultsToPreviewAndRejectsConcurrentRequests) {
  config.observation_timeout = 5;
  HandoverMotion handover(robot, *context, executor);
  shoulder_for_pick();
  PickCupConfig pc;
  PickCupMotion pick(robot, config, pc, *context, executor, scene);
  TaskManager manager(*node,
      *context,
      handover,
      {{"/approach_cup", *motion}, {"/refine_cup", *refinement}, {"/pick_cup", pick}});
  auto pick_client = rclcpp_action::create_client<action::ApproachCup>(node, "/pick_cup");
  auto client = rclcpp_action::create_client<action::ApproachCup>(node, "/approach_cup");
  auto refine_client = rclcpp_action::create_client<action::ApproachCup>(node, "/refine_cup");
  auto handover_client = rclcpp_action::create_client<action::Handover>(node, "/handover");
  auto until = [&](auto condition) {
    const auto start = Clock::now();
    while (elapsed(start) < 2) {
      feedback();
      ros.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  };
  ASSERT_TRUE(until([&] {
    return client->action_server_is_ready() && handover_client->action_server_is_ready();
  }));
  action::ApproachCup::Goal goal;
  EXPECT_FALSE(goal.execute);
  goal.arm = "left";
  goal.cup_id = "cup";
  auto sent = client->async_send_goal(goal);
  ASSERT_TRUE(until([&] {
    return sent.wait_for(0s) == std::future_status::ready;
  }));
  auto handle = sent.get();
  ASSERT_TRUE(handle);
  EXPECT_FALSE(manager.can_shutdown());
  auto concurrent = client->async_send_goal(goal);
  ASSERT_TRUE(until([&] {
    return concurrent.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_FALSE(concurrent.get());
  auto refine_busy = refine_client->async_send_goal(goal);
  ASSERT_TRUE(until([&] {
    return refine_busy.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_FALSE(refine_busy.get());
  auto pick_busy = pick_client->async_send_goal(goal);
  ASSERT_TRUE(until([&] {
    return pick_busy.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_FALSE(pick_busy.get());
  action::Handover::Goal handover_goal;
  handover_goal.direction = "left_to_right";
  auto other = handover_client->async_send_goal(handover_goal);
  ASSERT_TRUE(until([&] {
    return other.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_FALSE(other.get());
  auto result = client->async_get_result(handle);
  ASSERT_TRUE(until([&] {
    observe("head");
    return result.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(executor.steps.empty());
  EXPECT_TRUE(scene.scenes.empty());
  EXPECT_TRUE(manager.can_shutdown());
  tcp_for_refinement();
  auto refine_sent = refine_client->async_send_goal(goal);
  ASSERT_TRUE(until([&] {
    return refine_sent.wait_for(0s) == std::future_status::ready;
  }));
  auto refine_handle = refine_sent.get();
  ASSERT_TRUE(refine_handle);
  EXPECT_TRUE(refinement->running());
  EXPECT_FALSE(motion->running());
  auto approach_busy = client->async_send_goal(goal);
  ASSERT_TRUE(until([&] {
    return approach_busy.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_FALSE(approach_busy.get());
  auto refine_result = refine_client->async_get_result(refine_handle);
  manager.request_shutdown();
  ASSERT_TRUE(until([&] {
    return refine_result.wait_for(0s) == std::future_status::ready;
  }));
  EXPECT_TRUE(manager.can_shutdown());
  EXPECT_TRUE(executor.steps.empty());
}

} // namespace
} // namespace demo_1f

#include "demo_1f/runtime/execution/paired_path.hpp"
#include "demo_1f/runtime/execution/round_trip_path.hpp"
#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <limits>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace demo_1f {
namespace {
class RosEnvironment : public ::testing::Environment {
  void SetUp() override {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

auto * environment = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

moveit_msgs::msg::RobotTrajectory segment(std::vector<std::vector<double>> positions) {
  moveit_msgs::msg::RobotTrajectory result;
  result.joint_trajectory.joint_names = {"j1", "j2"};
  for (auto & values : positions) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = std::move(values);
    result.joint_trajectory.points.push_back(point);
  }
  return result;
}

class SmootherTest : public ::testing::Test {
protected:
  void SetUp() override {
    auto urdf = urdf::parseURDF(R"(
      <robot name="smooth_test">
        <link name="base"/><link name="one"/>
        <link name="two"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
        <joint name="j1" type="revolute"><parent link="base"/><child link="one"/>
          <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1.2" effort="10"/></joint>
        <joint name="j2" type="revolute"><parent link="one"/><child link="two"/>
          <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/>
          <limit lower="-2" upper="2" velocity="1.2" effort="10"/></joint>
      </robot>)");
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf,
        R"(<robot name="smooth_test"><group name="arms"><joint name="j1"/><joint name="j2"/></group></robot>)"));
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene = std::make_unique<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();
    config.velocity_scale = config.acceleration_scale = 1.0;
    config.coordinated_group = "arms";
    config.motion_limits = {{"j1", {0.6, 0.8, 4.0}}, {"j2", {0.6, 0.8, 4.0}}};
    first.name = "face_each_other";
    first.type = "dual_joint";
    first.velocity_scale = 1.0;
    first.acceleration_scale = 1.0;
    second = first;
    second.name = "open_approach";
    second.velocity_scale = 0.4;
  }

  Config config;
  Step first, second;
  moveit::core::RobotModelPtr model;
  std::unique_ptr<planning_scene::PlanningScene> scene;
};

TEST_F(SmootherTest, ConnectsViaPointWithNonzeroVelocityAndStopsAtTransfer) {
  auto path = smooth_segments(
      {segment({{0, 0}, {0.2, 0.1}, {0.4, 0.2}}), segment({{0.4, 0.2}, {0.8, 0.35}})},
      {first, second},
      config,
      model);
  ASSERT_EQ(path.segment_end_times.size(), 2u);
  bool crossed = false;
  const auto & points = path.trajectory.joint_trajectory.points;
  for (const auto & point : points) {
    if (std::abs(rclcpp::Duration(point.time_from_start).seconds() - path.segment_end_times[0]) <
        1e-8) {
      EXPECT_NEAR(point.positions[0], 0.4, 1e-8);
      EXPECT_GT(std::abs(point.velocities[0]), 0.01);
      EXPECT_NEAR(point.accelerations[0], 0.0, 1e-8);
      crossed = true;
    }
  }
  EXPECT_TRUE(crossed);
  EXPECT_NEAR(points.front().velocities[0], 0.0, 1e-10);
  EXPECT_NEAR(points.back().velocities[0], 0.0, 1e-10);
  EXPECT_NEAR(points.back().accelerations[0], 0.0, 1e-10);
  EXPECT_NEAR(points.back().positions[0], 0.8, 1e-8);
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
}

TEST_F(SmootherTest, CupPreparationKeepsTheRepeatedLeftGoalAndCrossesTheRightViaAtPointThreeScale) {
  first.velocity_scale = first.acceleration_scale = .3;
  second.velocity_scale = second.acceleration_scale = .3;
  auto path = smooth_segments(
      {segment({{0, 0}, {.2, .1}, {.4, .2}}), segment({{.4, .2}, {.4, .5}, {.4, .8}})},
      {first, second}, config, model);
  ASSERT_EQ(path.segment_end_times.size(), 2u);
  bool found_via = false;
  for (const auto & point : path.trajectory.joint_trajectory.points) {
    const double time = rclcpp::Duration(point.time_from_start).seconds();
    if (std::abs(time - path.segment_end_times.front()) < 1e-8) {
      found_via = true;
      EXPECT_NEAR(point.positions[0], .4, 1e-8);
      EXPECT_NEAR(point.positions[1], .2, 1e-8);
      EXPECT_NEAR(point.velocities[0], 0, 1e-8);
      EXPECT_GT(point.velocities[1], 0);
    }
    if (time >= path.segment_end_times.front() - 1e-8) {
      EXPECT_NEAR(point.positions[0], .4, 1e-8);
      EXPECT_NEAR(point.velocities[0], 0, 1e-8);
      EXPECT_NEAR(point.accelerations[0], 0, 1e-8);
    }
  }
  EXPECT_TRUE(found_via);
  const auto & last = path.trajectory.joint_trajectory.points.back();
  EXPECT_NEAR(last.positions[0], .4, 1e-8);
  EXPECT_NEAR(last.positions[1], .8, 1e-8);
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
}

TEST_F(SmootherTest, CupAttachmentIsStillCollisionCheckedAfterPreparationSmoothing) {
  auto path = smooth_segments({segment({{0, 0}, {.3, .2}})}, {first}, config, model);
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "cup_obstacle";
  obstacle.header.frame_id = "base";
  obstacle.operation = obstacle.ADD;
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = sphere.SPHERE;
  sphere.dimensions = {.06};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = .5;
  pose.position.z = .4;
  obstacle.primitives = {sphere};
  obstacle.primitive_poses = {pose};
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  EXPECT_NO_THROW(validate_smooth_path(path, {first}, config, *scene));
  moveit_msgs::msg::AttachedCollisionObject cup;
  cup.link_name = "two";
  cup.object.header.frame_id = "two";
  cup.object.id = "demo_place_cup";
  cup.object.operation = cup.object.ADD;
  pose.position.x = 0;
  cup.object.primitives = {sphere};
  cup.object.primitive_poses = {pose};
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(cup));
  EXPECT_THROW(validate_smooth_path(path, {first}, config, *scene), std::runtime_error);
  config.collision_check_step = .005; // fewer collision samples find it too
  EXPECT_THROW(validate_smooth_path(path, {first}, config, *scene), std::runtime_error);
}

TEST_F(SmootherTest, HeldArmCorridorRejectsAnOmplDetourWithoutCorrectingThePlan) {
  config.arms["left"].joints = {"j1"};
  second.hold_arm = "left";
  second.hold_tolerance = .001;
  second.left_target = {.4};
  const auto detour = segment({{.4, .2}, {.42, .4}, {.4, .8}});
  const auto original = detour;
  EXPECT_THROW(validate_held_arm_path(detour.joint_trajectory, second, config), std::runtime_error);
  EXPECT_THROW(smooth_segments({segment({{0, 0}, {.4, .2}}), detour}, {first, second}, config, model),
      std::runtime_error);
  EXPECT_EQ(detour, original);
}

TEST_F(SmootherTest, HeldArmCorridorAcceptsBoundedPlanningErrorAndChecksTheStart) {
  config.arms["left"].joints = {"j1"};
  second.hold_arm = "left";
  second.hold_tolerance = .001;
  second.left_target = {.4};
  EXPECT_NO_THROW(validate_held_arm_path(segment({{.4005, .2}, {.3995, .8}}).joint_trajectory,
      second, config));
  EXPECT_THROW(validate_held_arm_path(segment({{.402, .2}, {.4, .8}}).joint_trajectory,
      second, config), std::runtime_error);
  EXPECT_THROW(validate_held_arm_path(segment({{.4, .2}, {.402, .8}}).joint_trajectory,
      second, config), std::runtime_error);
}

TEST_F(SmootherTest, HeldArmCorridorRejectsQuinticExcursionsBetweenValidKnots) {
  config.arms["left"].joints = {"j1"};
  first.hold_arm = "left";
  first.hold_tolerance = .001;
  first.left_target = {.4};
  SmoothPath path;
  path.trajectory = segment({{.4, .2}, {.4, .2}});
  path.point_steps = {0, 0};
  auto & points = path.trajectory.joint_trajectory.points;
  for (auto & point : points) {
    point.velocities = {.02, 0};
    point.accelerations = {0, 0};
  }
  points.back().time_from_start = rclcpp::Duration::from_seconds(1);
  const auto original = path.trajectory;
  EXPECT_THROW(validate_smooth_path(path, {first}, config, *scene), std::runtime_error);
  EXPECT_EQ(path.trajectory, original);
}

TEST_F(SmootherTest, HeldArmCorridorAlsoAcceptsValidSmoothedPreparation) {
  config.arms["left"].joints = {"j1"};
  second.hold_arm = "left";
  second.hold_tolerance = .001;
  second.left_target = {.4};
  auto path = smooth_segments({segment({{0, 0}, {.4, .2}}), segment({{.4, .2}, {.4, .8}})},
      {first, second}, config, model);
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
}

TEST_F(SmootherTest, RespectsSlowerApproachAndUrdfVelocityCap) {
  config.motion_limits["j1"].velocity = 10;
  auto path = smooth_segments({segment({{0, 0}, {0.4, 0.2}}), segment({{0.4, 0.2}, {0.8, 0.4}})},
      {first, second},
      config,
      model);
  for (size_t i = 0; i < path.point_steps.size(); ++i) {
    const auto & point = path.trajectory.joint_trajectory.points[i];
    const double cap = path.point_steps[i] == 0 ? 1.2 : 0.48;
    EXPECT_LE(std::abs(point.velocities[0]), cap + 1e-8);
  }
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
}

TEST_F(SmootherTest, KeepsGripperExactlyFixedUntilItsOpeningSegment) {
  config.arms["left"].gripper_joint = "j2";
  second.type = "dual_joint_gripper";
  second.arm = "left";
  auto path = smooth_segments(
      {segment({{0, 0.8}, {0.2, 0.7992}, {0.4, 0.7995}}), segment({{0.4, 0.7995}, {0.8, 0.0}})},
      {first, second},
      config,
      model);
  for (size_t i = 0; i < path.point_steps.size(); ++i) {
    const auto & point = path.trajectory.joint_trajectory.points[i];
    if (path.point_steps[i] == 0) {
      EXPECT_NEAR(point.positions[1], 0.8, 1e-10);
      EXPECT_NEAR(point.velocities[1], 0.0, 1e-10);
    }
  }
  EXPECT_NEAR(path.trajectory.joint_trajectory.points.back().positions[1], 0.0, 1e-8);
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
  EXPECT_THROW(smooth_segments({segment({{0, 0.8}, {0.4, 0.79}})}, {first}, config, model),
      std::runtime_error);
}

TEST_F(SmootherTest, RejectsCollisionIntroducedOnTheExecutedPath) {
  auto path = smooth_segments({segment({{0, 0}, {0.4, 0.2}})}, {first}, config, model);
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = "base";
  obstacle.id = "blocking_box";
  obstacle.operation = obstacle.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {10, 10, 10};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  EXPECT_THROW(validate_smooth_path(path, {first}, config, *scene), std::runtime_error);
}

TEST_F(SmootherTest, StrictObjectsIgnoreSceneContactAllowance) {
  auto path = smooth_segments({segment({{0, 0}, {0.4, 0.2}}), segment({{0.4, 0.2}, {0.8, 0.4}})},
      {first, second}, config, model);
  moveit_msgs::msg::CollisionObject cup;
  cup.header.frame_id = "base";
  cup.id = "demo_place_cup";
  cup.operation = cup.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {10, 10, 10};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  cup.primitives.push_back(box);
  cup.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(cup));
  scene->getAllowedCollisionMatrixNonConst().setEntry("demo_place_cup", "two", true);
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
  first.strict_objects = {"demo_place_cup"};
  EXPECT_THROW(validate_smooth_path(path, {first, second}, config, *scene), PathCollisionError);
  // The scene allowance itself is unchanged for later planning.
  collision_detection::AllowedCollision::Type type;
  ASSERT_TRUE(scene->getAllowedCollisionMatrix().getEntry("demo_place_cup", "two", type));
  EXPECT_EQ(type, collision_detection::AllowedCollision::ALWAYS);
}

TEST_F(SmootherTest, EndpointsOnlySegmentIsEnteredMovingAndStaysOnItsJointLine) {
  second.endpoints_only = true;
  // Dense planned points off the endpoint line are dropped.
  auto path = smooth_segments({segment({{0, 0}, {0.4, 0.1}}),
      segment({{0.4, 0.1}, {0.5, 0.15}, {0.55, 0.3}, {0.6, 0.5}})}, {first, second}, config, model);
  const auto & points = path.trajectory.joint_trajectory.points;
  bool crossed = false;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i].positions;
    if (std::abs(rclcpp::Duration(points[i].time_from_start).seconds() -
            path.segment_end_times[0]) < 1e-8) {
      // Moving at the boundary, along the next segment's direction (0.2, 0.4).
      const auto & v = points[i].velocities;
      EXPECT_GT(std::hypot(v[0], v[1]), 0.01);
      EXPECT_NEAR(v[0] * 0.4 - v[1] * 0.2, 0.0, 1e-6);
      crossed = true;
    }
    if (path.point_steps[i] == 1) {
      EXPECT_NEAR((p[0] - 0.4) * 0.4 - (p[1] - 0.1) * 0.2, 0.0, 1e-6) << "point " << i;
    }
  }
  EXPECT_TRUE(crossed);
  EXPECT_NEAR(points.back().positions[0], 0.6, 1e-8);
  EXPECT_NEAR(points.back().positions[1], 0.5, 1e-8);
  EXPECT_NO_THROW(validate_smooth_path(path, {first, second}, config, *scene));
}

TEST_F(SmootherTest, TcpLineToleranceChecksTheExecutedTcpPath) {
  // Link "two" sits 0.5 m out on j1: a j1-only joint line is a TCP arc, ~10 mm off its chord.
  config.arms["left"].tcp = "two";
  first.arm = "left";
  first.tcp_line_tolerance = .003;
  auto path = smooth_segments({segment({{0, 0}, {0.4, 0}})}, {first}, config, model);
  try {
    validate_smooth_path(path, {first}, config, *scene);
    FAIL() << "A 10 mm arc passed a 3 mm line tolerance";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("straight line"), std::string::npos) << error.what();
  }
  first.tcp_line_tolerance = .015;
  first.tcp_drop_tolerance = .001; // The arc stays level.
  EXPECT_NO_THROW(validate_smooth_path(path, {first}, config, *scene));
  first.arm = "right"; // No TCP for this arm.
  EXPECT_THROW(validate_smooth_path(path, {first}, config, *scene), std::runtime_error);
}

TEST_F(SmootherTest, RejectsBoundsAndDiscontinuousPlanningStarts) {
  auto path = smooth_segments({segment({{0, 0}, {2.1, 0.2}})}, {first}, config, model);
  EXPECT_THROW(validate_smooth_path(path, {first}, config, *scene), std::runtime_error);
  EXPECT_THROW(smooth_segments({segment({{0, 0}, {0.4, 0.2}}), segment({{0.5, 0.2}, {0.8, 0.3}})},
                   {first, second},
                   config,
                   model),
      std::runtime_error);
}

TEST_F(SmootherTest, RejectsMalformedAndNonfinitePaths) {
  auto malformed = segment({{0, 0}, {0.4}});
  EXPECT_THROW(smooth_segments({malformed}, {first}, config, model), std::runtime_error);
  auto invalid = segment({{0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0.2}});
  EXPECT_THROW(smooth_segments({invalid}, {first}, config, model), std::runtime_error);
}

TEST_F(SmootherTest, RepeatedPositionsStillProduceStrictlyIncreasingTimestamps) {
  auto path = smooth_segments({segment({{0, 0}, {0, 0}, {0.4, 0.2}})}, {first}, config, model);
  double previous = -1;
  for (const auto & point : path.trajectory.joint_trajectory.points) {
    const double time = rclcpp::Duration(point.time_from_start).seconds();
    EXPECT_GT(time, previous);
    previous = time;
  }
  EXPECT_NO_THROW(validate_smooth_path(path, {first}, config, *scene));
}

TEST_F(SmootherTest, GripperDurationIsAMinimumAndLimitedProfileIsValidated) {
  first.type = "gripper";
  first.duration = 0.9;
  config.motion_limits["j1"] = {2.0, 2.0, 20.0};
  auto path = smooth_segments({segment({{0, 0}, {0.8, 0}})}, {first}, config, model);
  EXPECT_GE(
      rclcpp::Duration(path.trajectory.joint_trajectory.points.back().time_from_start).seconds(),
      first.duration);
  EXPECT_NO_THROW(validate_smooth_path(path, {first}, config, *scene));
  auto stationary = smooth_segments({segment({{0, 0}, {0, 0}})}, {first}, config, model);
  EXPECT_GE(rclcpp::Duration(stationary.trajectory.joint_trajectory.points.back().time_from_start)
                .seconds(),
      first.duration);
  EXPECT_NO_THROW(validate_smooth_path(stationary, {first}, config, *scene));
}

TEST(PairedPath, EndpointsMatchAndFirstArmComesFirst) {
  const JointPath left{{0.0, 0.0}, {1.0, 0.0}};
  const JointPath right{{5.0}, {6.0}, {8.0}};
  const auto rows = pair_paths(left, right);
  ASSERT_GE(rows.size(), 2u);
  EXPECT_EQ(rows.front(), (std::vector<double>{0.0, 0.0, 5.0}));
  EXPECT_EQ(rows.back(), (std::vector<double>{1.0, 0.0, 8.0}));
  for (const auto & row : rows) EXPECT_EQ(row.size(), 3u); // left's 2 joints, then right's 1
}

TEST(PairedPath, EveryCornerKeptAndProgressMonotonic) {
  // right turns at a third of its length: that row exists, left a third along at the same row
  const JointPath left{{0.0}, {3.0}};
  const JointPath right{{0.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}};
  const auto rows = pair_paths(left, right);
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_NEAR(rows[1][0], 1.0, 1e-12);
  EXPECT_NEAR(rows[1][1], 1.0, 1e-12);
  EXPECT_NEAR(rows[1][2], 0.0, 1e-12);
  for (size_t i = 1; i < rows.size(); ++i) {
    EXPECT_GE(rows[i][0], rows[i - 1][0]);
    EXPECT_GE(rows[i][1] + rows[i][2], rows[i - 1][1] + rows[i - 1][2]);
  }
}

TEST(PairedPath, ZeroLengthPathStaysPut) {
  const JointPath left{{0.0}, {1.0}, {2.0}};
  const JointPath still{{4.0, 4.0}, {4.0, 4.0}};
  const auto rows = pair_paths(left, still);
  ASSERT_EQ(rows.size(), 3u);
  for (const auto & row : rows) {
    EXPECT_EQ(row[1], 4.0);
    EXPECT_EQ(row[2], 4.0);
  }
  EXPECT_EQ(pair_paths(still, left).back(), (std::vector<double>{4.0, 4.0, 2.0}));
  EXPECT_THROW(pair_paths({}, left), std::invalid_argument);
  EXPECT_THROW(pair_paths({{1.0}, {1.0, 2.0}}, left), std::invalid_argument);
}
moveit_msgs::msg::RobotTrajectory round_trip_knots() {
  auto trajectory = segment({{0, .1}, {.2, .3}, {.4, .5}, {.6, .7}});
  auto & joints = trajectory.joint_trajectory;
  joints.header.stamp.sec = 123;
  const std::vector<int64_t> times = {0, 300000001, 1100000007, 2400000011};
  for (size_t i = 0; i < joints.points.size(); ++i) {
    auto & point = joints.points[i];
    point.time_from_start = rclcpp::Duration(std::chrono::nanoseconds(times[i]));
    point.velocities = i == 0 || i == 3 ? std::vector<double>{0, 0} :
        std::vector<double>{.2, -.3};
    point.accelerations = i == 0 || i == 3 ? std::vector<double>{0, 0} :
        std::vector<double>{-.4, .5};
  }
  return trajectory;
}

TEST(RoundTripPath, ReversesNonuniformNanosecondTimesAndDerivatives) {
  const auto forward = round_trip_knots();
  const auto backward = reverse_round_trip_path(forward, {"j2", "j1"});
  const auto & a = forward.joint_trajectory;
  const auto & b = backward.joint_trajectory;
  EXPECT_EQ(b.joint_names, a.joint_names);
  EXPECT_EQ(b.header.stamp.sec, 0);
  EXPECT_EQ(b.header.stamp.nanosec, 0u);
  ASSERT_EQ(b.points.size(), a.points.size());
  const auto total = rclcpp::Duration(a.points.back().time_from_start).nanoseconds();
  for (size_t i = 0; i < a.points.size(); ++i) {
    const auto & original = a.points[a.points.size() - 1 - i];
    EXPECT_EQ(b.points[i].positions, original.positions);
    EXPECT_EQ(b.points[i].accelerations, original.accelerations);
    EXPECT_TRUE(b.points[i].effort.empty());
    EXPECT_EQ(rclcpp::Duration(b.points[i].time_from_start).nanoseconds(),
        total - rclcpp::Duration(original.time_from_start).nanoseconds());
    for (size_t j = 0; j < a.joint_names.size(); ++j) {
      EXPECT_EQ(b.points[i].velocities[j], -original.velocities[j]);
    }
    if (i > 0) {
      EXPECT_GT(rclcpp::Duration(b.points[i].time_from_start).nanoseconds(),
          rclcpp::Duration(b.points[i - 1].time_from_start).nanoseconds());
    }
  }
  EXPECT_EQ(rclcpp::Duration(b.points.front().time_from_start).nanoseconds(), 0);
  EXPECT_EQ(rclcpp::Duration(b.points.back().time_from_start).nanoseconds(), total);
  EXPECT_EQ(b.points.back().positions, a.points.front().positions);
}

TEST(RoundTripPath, ReversingTwiceRestoresEveryKnot) {
  const auto original = round_trip_knots();
  const auto back = reverse_round_trip_path(original, {"j1", "j2"});
  const auto restored = reverse_round_trip_path(back, {"j1", "j2"});
  EXPECT_EQ(restored.joint_trajectory.points, original.joint_trajectory.points);
}

TEST(RoundTripPath, RejectsUnsupportedOrMalformedResponses) {
  for (int defect = 0; defect < 18; ++defect) {
    auto invalid = round_trip_knots();
    auto & joints = invalid.joint_trajectory;
    switch (defect) {
      case 0: joints.joint_names[1] = "j1"; break;
      case 1: joints.joint_names[1] = "other"; break;
      case 2: joints.points.resize(1); break;
      case 3: joints.points[1].positions.pop_back(); break;
      case 4: joints.points[1].velocities.clear(); break;
      case 5: joints.points[1].accelerations.clear(); break;
      case 6: joints.points[1].positions[0] = std::numeric_limits<double>::infinity(); break;
      case 7: joints.points[1].time_from_start = joints.points[0].time_from_start; break;
      case 8: joints.points[0].time_from_start.nanosec = 1; break;
      case 9: joints.points.back().velocities[0] = .01; break;
      case 10: joints.points.front().accelerations[0] = .01; break;
      case 11: joints.points[1].effort = {0, 0}; break;
      case 12: invalid.multi_dof_joint_trajectory.points.resize(1); break;
      case 13: joints.points[1].velocities[0] = std::numeric_limits<double>::infinity(); break;
      case 14: joints.points[1].accelerations[0] = std::numeric_limits<double>::quiet_NaN(); break;
      case 15: joints.points[1].time_from_start.sec = -1; break;
      case 16: joints.points[1].time_from_start.nanosec = 1000000000u; break;
      case 17: invalid.multi_dof_joint_trajectory.joint_names = {"floating"}; break;
    }
    EXPECT_THROW(reverse_round_trip_path(invalid, {"j1", "j2"}), std::runtime_error) << defect;
  }
}
TEST_F(SmootherTest, DenseSingleArmLinUsesTheSameTimingAsItsCollinearEndpoints) {
  first.type = "linear";
  first.arm = "right";
  config.arms["right"].joints = {"j1", "j2"};
  std::vector<std::vector<double>> dense;
  for (int i = 0; i <= 100; ++i) dense.push_back({.004 * i, .002 * i});
  const auto sparse = smooth_segments({segment({{0, 0}, {.4, .2}})}, {first}, config, model);
  const auto sampled = smooth_segments({segment(dense)}, {first}, config, model);
  const auto & a = sparse.trajectory.joint_trajectory.points;
  const auto & b = sampled.trajectory.joint_trajectory.points;
  EXPECT_EQ(a, b);
  EXPECT_GT(rclcpp::Duration(b.back().time_from_start).nanoseconds(), 0);
  for (size_t i = 1; i + 1 < b.size(); ++i) EXPECT_GT(b[i].velocities[0], 0);
  EXPECT_NO_THROW(validate_smooth_path(sampled, {first}, config, *scene));
}

} // namespace
} // namespace demo_1f

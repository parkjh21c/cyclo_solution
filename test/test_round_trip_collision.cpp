#include "demo_1f/runtime/execution/round_trip_collision.hpp"
#include "demo_1f/runtime/execution/round_trip_path.hpp"
#include "demo_1f/runtime/execution/trajectory_smoother.hpp"
#include <gtest/gtest.h>
#include <rclcpp/duration.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace demo_1f {
namespace {
class PourCollisionTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto urdf = urdf::parseURDF(R"(
      <robot name="pour_collision">
        <link name="base"/>
        <link name="right_link"><collision><geometry><sphere radius="0.005"/>
          </geometry></collision></link>
        <link name="left_link"><collision><geometry><sphere radius="0.005"/>
          </geometry></collision></link>
        <joint name="right" type="prismatic"><parent link="base"/><child link="right_link"/>
          <axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="10"/></joint>
        <joint name="left" type="prismatic"><parent link="base"/><child link="left_link"/>
          <origin xyz="0.5 0 0"/><axis xyz="1 0 0"/>
          <limit lower="-1" upper="1" velocity="1" effort="10"/></joint>
      </robot>)");
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, R"(<robot name="pour_collision"><group name="arms">
      <joint name="left"/><joint name="right"/></group></robot>)"));
    const auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene = std::make_unique<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();
    config.coordinated_group = "arms";
    trajectory.joint_trajectory.joint_names = {"right"};
    for (int i = 0; i < 2; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {.05 * i};
      point.velocities = point.accelerations = {0};
      point.time_from_start = rclcpp::Duration::from_seconds(.02 * i);
      trajectory.joint_trajectory.points.push_back(point);
    }
  }

  void obstacle(double y = 0) {
    moveit_msgs::msg::CollisionObject object;
    object.id = "obstacle";
    object.header.frame_id = "base";
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions = {.003};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = .025;
    pose.position.y = y;
    object.primitives = {sphere};
    object.primitive_poses = {pose};
    ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  }

  void attach_cup() {
    moveit_msgs::msg::AttachedCollisionObject cup;
    cup.link_name = "right_link";
    cup.object.header.frame_id = cup.link_name;
    cup.object.id = "test_cup";
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions = {.006};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    pose.position.y = .03;
    cup.object.primitives = {sphere};
    cup.object.primitive_poses = {pose};
    cup.touch_links = {cup.link_name};
    ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(cup));
  }

  std::unique_ptr<planning_scene::PlanningScene> scene;
  Config config;
  moveit_msgs::msg::RobotTrajectory trajectory;
};

TEST_F(PourCollisionTest, OriginalDataIsUnchangedAndNoApplicationDynamicsAreApplied) {
  const auto original = trajectory;
  SmoothPath path;
  path.trajectory = trajectory;
  path.point_steps = {0, 0};
  Step step;
  step.type = "linear";
  step.arm = "right";
  EXPECT_DOUBLE_EQ(MotionLimits{}.jerk, 4.0);
  try {
    validate_smooth_path(path, {step}, config, *scene);
    FAIL() << "Expected the existing dynamic validator to reject this profile";
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("v=0.000000"), std::string::npos);
    EXPECT_NE(message.find("a=0.000000"), std::string::npos);
    EXPECT_NE(message.find("j="), std::string::npos);
  }
  EXPECT_NO_THROW(validate_round_trip_collision(trajectory, {"right"}, config, *scene));
  EXPECT_EQ(trajectory, original);
  const auto reverse = reverse_round_trip_path(trajectory, {"right"});
  EXPECT_NO_THROW(validate_round_trip_collision(reverse, {"right"}, config, *scene));
  EXPECT_EQ(reverse.joint_trajectory.points.back().time_from_start,
      trajectory.joint_trajectory.points.back().time_from_start);
}

TEST_F(PourCollisionTest, DetectsCollisionBetweenCollisionFreeKnotsAndReportsContacts) {
  obstacle();
  for (const auto & point : trajectory.joint_trajectory.points) {
    auto state = scene->getCurrentState();
    state.setVariablePosition("right", point.positions.front());
    state.update();
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state);
    EXPECT_FALSE(result.collision);
  }
  try {
    validate_round_trip_collision(trajectory, {"right"}, config, *scene);
    FAIL() << "Interpolated collision was not detected";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("right_link"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("obstacle"), std::string::npos);
  }
}

TEST_F(PourCollisionTest, AttachedCupAndAllowedContactsAreIncluded) {
  attach_cup();
  obstacle(.03);
  EXPECT_THROW(validate_round_trip_collision(trajectory, {"right"}, config, *scene),
      std::runtime_error);
  scene->getAllowedCollisionMatrixNonConst().setEntry("test_cup", "obstacle", true);
  EXPECT_NO_THROW(validate_round_trip_collision(trajectory, {"right"}, config, *scene));
}

TEST_F(PourCollisionTest, OtherArmStateIsIncluded) {
  scene->getCurrentStateNonConst().setVariablePosition("left", -.475);
  EXPECT_THROW(validate_round_trip_collision(trajectory, {"right"}, config, *scene),
      std::runtime_error);
}

TEST_F(PourCollisionTest, SharedBoundarySamplesAreCheckedOnlyOnce) {
  auto & points = trajectory.joint_trajectory.points;
  points.back().positions = {.02};
  points.back().time_from_start = rclcpp::Duration::from_seconds(.006);
  auto last = points.back();
  last.positions = {.04};
  last.time_from_start = rclcpp::Duration::from_seconds(.016);
  points.push_back(last);
  const auto checks = validate_round_trip_collision(trajectory, {"right"}, config, *scene);
  EXPECT_EQ(checks.samples, 11u);
}

TEST_F(PourCollisionTest, CollisionStepChecksFewerSamplesWithBothEnds) {
  config.collision_check_step = .02; // 0.05 of travel: start, about every 0.02, end
  const auto checks = validate_round_trip_collision(trajectory, {"right"}, config, *scene);
  EXPECT_GE(checks.samples, 3u);
  EXPECT_LE(checks.samples, 5u);
}

TEST_F(PourCollisionTest, CollisionStepStillFindsAnObstacleWiderThanTheStep) {
  config.collision_check_step = .005; // the contact window is 0.016 wide
  obstacle();
  EXPECT_THROW(validate_round_trip_collision(trajectory, {"right"}, config, *scene),
      std::runtime_error);
}

TEST_F(PourCollisionTest, InterpolatedJointBoundsAreStillChecked) {
  auto & points = trajectory.joint_trajectory.points;
  points.back().positions = {.99};
  points.back().velocities = {50};
  points.back().time_from_start = rclcpp::Duration::from_seconds(1);
  auto last = points.front();
  last.time_from_start = rclcpp::Duration::from_seconds(2);
  points.push_back(last);
  scene->getAllowedCollisionMatrixNonConst().setEntry("right_link", "left_link", true);
  EXPECT_THROW(validate_round_trip_collision(trajectory, {"right"}, config, *scene),
      std::runtime_error);
}

TEST_F(PourCollisionTest, JointNamesMapToModelIndicesInEitherOrder) {
  auto & joints = trajectory.joint_trajectory;
  joints.joint_names = {"right", "left"};
  for (auto & point : joints.points) {
    point.positions.push_back(.8);
    point.velocities.push_back(0);
    point.accelerations.push_back(0);
  }
  obstacle();
  EXPECT_THROW(validate_round_trip_collision(trajectory, {"left", "right"}, config, *scene),
      std::runtime_error);
}
} // namespace
} // namespace demo_1f

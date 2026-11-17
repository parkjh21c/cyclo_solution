#include "demo_1f/tasks/task_manager.hpp"
#include "demo_1f/tasks/demo_recovery.hpp"
#include <algorithm>
#include <functional>
#include <gtest/gtest.h>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
namespace demo_1f {
// Replay transport responses without exposing execution controls on TaskManager.
class TaskManagerTestAccess {
public:
  using Request = TaskManager::PickRequest;
  static Request current(const TaskManager & manager) { return manager.pick_; }
  static std::uint64_t run(const TaskManager & manager) { return manager.demo_run_id_; }
  static std::string detail(const TaskManager & manager) { return manager.demo_detail_; }
  static void feedback(TaskManager & manager, const Request & request,
      ffw_vision_msgs::action::PickDynamixel::Feedback::ConstSharedPtr feedback) {
    manager.on_pick_feedback(request, feedback);
  }
  static void result(TaskManager & manager, const Request & request,
      const rclcpp_action::ClientGoalHandle<ffw_vision_msgs::action::PickDynamixel>::WrappedResult & result) {
    manager.on_pick_result(request, result);
  }
  static void release(TaskManager & manager, const Request & request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    manager.on_pick_release_response(request, response);
  }
};

namespace {
using Demo = action::RunDemo;
using Pick = ffw_vision_msgs::action::PickDynamixel;

class Environment : public ::testing::Environment {
  void SetUp() override { int argc = 0; rclcpp::init(argc, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto * environment = ::testing::AddGlobalTestEnvironment(new Environment);

class Executor : public ExecutorPort {
public:
  std::vector<std::string> executed;
  ExecutionResult state;
  std::string rejected_step;
  bool stall = false;
  bool ready(const MotionSequence &, std::string &) const override { return true; }
  bool start(const Step & step) override {
    executed.push_back(step.name);
    if (step.name == rejected_step) {
      state = {ExecutionState::Failed, "EXECUTION_FAILED", "test execution failure"};
      return false;
    }
    state.state = ExecutionState::Running;
    return true;
  }
  void tick() override {
    if (!stall && state.state == ExecutionState::Running)
      state.state = ExecutionState::Succeeded;
  }
  void cancel(const std::string & code, const std::string & detail) override {
    state = {ExecutionState::Cancelled, code, detail};
  }
  ExecutionResult result() const override { return state; }
};
// Records head goals; a sent goal never reports completion, like a stalled controller.
class Head : public HeadCommandPort {
public:
  std::vector<std::string> commands; // Every send attempt, in order.
  int cancels = 0;
  bool refuse = false;    // Missing server or rejection: nothing sent.
  bool in_flight = false;
  bool command(const std::string & name) override {
    commands.push_back(name);
    in_flight = !refuse;
    return !refuse;
  }
  void cancel() override {
    ++cancels;
    in_flight = false;
  }
};
// Side return stand-in: finishes on its first tick unless stalled.
class Side : public SideMotionPort {
public:
  int starts = 0, cancels = 0;
  bool stall = false, refuse = false;
  bool cancel_stalls = false; // Cancel leaves the stop unconfirmed until the test ends it.
  ExecutionResult state;
  bool start(std::string & reason) override {
    ++starts;
    if (refuse) {
      reason = "test side refusal";
      state = {ExecutionState::Failed, "PATH_PREPARATION_FAILED", reason};
      state.retryable_before_motion = true;
      return false;
    }
    state = {ExecutionState::Running, {}, {}};
    return true;
  }
  void tick() override {
    if (state.state == ExecutionState::Running && !stall) state.state = ExecutionState::Succeeded;
  }
  void cancel() override {
    ++cancels;
    if (state.state == ExecutionState::Running) {
      state = {cancel_stalls ? ExecutionState::Cancelling : ExecutionState::Cancelled,
          "CANCELLED", "test cancel"};
    }
  }
  ExecutionResult result() const override { return state; }
  std::string arm() const override { return "right"; }
  void fail(bool before_motion) {
    state = {ExecutionState::Failed, "EXECUTION_FAILED", "test side failure"};
    state.retryable_before_motion = before_motion;
  }
};
class Scene : public ApproachScenePort {
public:
  bool ready() const override { return true; }
  bool apply(const moveit_msgs::msg::PlanningScene & scene) override {
    applied.push_back(scene);
    return true;
  }
  ExecutionResult poll() override { return outcome; }
  std::vector<moveit_msgs::msg::PlanningScene> applied;
  ExecutionResult outcome{ExecutionState::Succeeded, {}, {}};
};
class Cup : public CupMotion {
public:
  using CupMotion::CupMotion;
  int runs = 0;
  int failures = 0;
  int first_failure = 1; // Runs first_failure .. first_failure + failures - 1 fail.
  std::string failure = "CUP_NOT_FOUND";
  std::string requested_arm;
  bool stall = false;
  std::function<bool()> may_finish; // Unset: finishes on its first tick.
  bool start(const std::string & arm, const std::string & cup_id,
      std::string & reason, bool execute = true) override {
    requested_arm = arm;
    return CupMotion::start(arm == "both" ? "left" : arm, cup_id, reason, execute);
  }
  void tick() override {
    if (result_.state == "CANCELLING") { result_.state = "CANCELLED"; return; }
    if (!running() || stall || (may_finish && !may_finish())) return;
    if (runs >= first_failure && runs < first_failure + failures) {
      fail(failure, "test failure");
      return;
    }
    result_.state = "SUCCEEDED";
    result_.sequence_completed = true;
  }
protected:
  const char * initial_phase() const override { return "TEST_CUP"; }
  void reset() override { ++runs; }
};

class DemoTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor loop;
  rclcpp::Node::SharedPtr backend_node;
  rclcpp::executors::SingleThreadedExecutor backend_loop;
  bool separate_backend{false}, run_backend{true};
  Config config;
  ApproachConfig cup_config;
  Executor executor;
  Scene scene;
  std::unique_ptr<RobotContext> context;
  Head head;
  Side side;
  bool independent = true; // demo.independent_return
  std::unique_ptr<HandoverMotion> handover;
  std::unique_ptr<InitialPoseMotion> initial;
  std::unique_ptr<Cup> approach, refine, withdraw, pick_cup, handover_cup, pour, handover_back, place;
  std::unique_ptr<TaskManager> manager;
  rclcpp_action::Server<Pick>::SharedPtr backend;
  rclcpp_action::Client<Demo>::SharedPtr client;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> pending_pick;
  int picks = 0, pick_failures = 0;
  int cells = 1, cell_picks = 0;   // Picks after the cells are gone report BASKET_EMPTY.
  int moved_failure_at = 0, stall_pick_at = 0;
  bool stall_pick = false, moved_failure = false, uncertain_stop = false;
  bool withdraw_enabled = false;
  bool basket_seen = true;  // Wrist observer pose fed on every pump.
  double basket_age = 0;    // s
  int max_rounds = 0;       // 0 keeps the demo.max_cell_rounds default.
  bool prepare = false;     // demo.prepare_pick_during_refine; also serves /dynamixel_pick/release.
  double initial_target = .1; // Right arm initial pose target (pump reports 0).
  bool manual_cancel = false, refuse_release = false;
  int early_picks = 0;      // Requests with hold_before_motion.
  int pumps = 0;
  std::vector<std::string> events; // Pick requests and releases in order.
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> held_pick;
  std::shared_ptr<Pick::Result> held_result;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service;
  bool defer_release_response{false};
  bool defer_release_processing{false};
  int releases_processed{0};
  std::shared_ptr<rmw_request_id_t> delayed_release_header;
  std_srvs::srv::Trigger::Response::SharedPtr delayed_release_response;
  std::vector<TaskManagerTestAccess::Request> observed_requests;
  std::vector<rclcpp_action::GoalUUID> accepted_pick_ids, cancelled_pick_ids;

  void SetUp() override {
    std::vector<rclcpp::Parameter> overrides = {rclcpp::Parameter("demo.retry_delay", .01),
        rclcpp::Parameter("demo.withdraw_cup", withdraw_enabled),
        rclcpp::Parameter("demo.prepare_pick_during_refine", prepare),
        rclcpp::Parameter("demo.independent_return", independent)};
    if (max_rounds > 0) overrides.emplace_back("demo.max_cell_rounds", max_rounds);
    node = std::make_shared<rclcpp::Node>("demo_task_test",
        rclcpp::NodeOptions().parameter_overrides(overrides));
    config.require_tcp_tf = false;
    config.state_max_age = 5;
    for (const std::string name : {"left", "right"}) {
      ArmConfig arm;
      arm.name = name;
      arm.joints = {name + "_joint"};
      arm.gripper_joint = name + "_grip";
      config.arms[name] = arm;
    }
    MotionSequence sequence;
    sequence.direction = "right_to_left";
    sequence.giver = "right";
    sequence.receiver = "left";
    Step step;
    step.name = "handover_test";
    step.type = "joint";
    step.arm = "right";
    step.target = {initial_target};
    step.timeout = 1;
    step.duration = 1;
    sequence.steps = {step};
    config.motion_sequences[sequence.direction] = sequence;
    auto init = sequence;
    init.direction = "initial_pose";
    init.steps.front().name = "initial_test";
    config.motion_sequences[init.direction] = init;
    context = std::make_unique<RobotContext>(*node, config);
    handover = std::make_unique<HandoverMotion>(config, *context, executor);
    initial = std::make_unique<InitialPoseMotion>(config, *context, executor, &scene);
    approach = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    refine = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    withdraw = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    pick_cup = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    handover_cup = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    pour = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    handover_back = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    place = std::make_unique<Cup>(config, cup_config, *context, executor, scene);
    manager = std::make_unique<TaskManager>(*node, *context, *handover,
        std::vector<CupMotionRegistration>{{"/approach_cup", *approach}, {"/refine_cup", *refine},
            {"/withdraw_cup", *withdraw}, {"/pick_cup", *pick_cup},
            {"/handover_cup", *handover_cup}, {"/pour_cup", *pour},
            {"/handover_cup_back", *handover_back}, {"/place_cup", *place}}, initial.get(), &head, &side);
    backend_node = separate_backend ? std::make_shared<rclcpp::Node>("fake_pick_backend") : node;
    if (separate_backend) backend_loop.add_node(backend_node);
    backend = rclcpp_action::create_server<Pick>(backend_node, "/dynamixel_pick/execute",
        [](const auto &, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](auto handle) {
          cancelled_pick_ids.push_back(handle->get_goal_id());
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](auto handle) {
          ++picks;
          observed_requests.push_back(TaskManagerTestAccess::current(*manager));
          accepted_pick_ids.push_back(handle->get_goal_id());
          EXPECT_FALSE(handle->get_goal()->move_to_observe);
          const bool hold = handle->get_goal()->hold_before_motion;
          early_picks += hold;
          if (stall_pick || picks == stall_pick_at) { pending_pick = handle; return; }
          auto result = std::make_shared<Pick::Result>();
          result->holding = false; // No physical grasp verdict; motion success is sufficient.
          result->stop_confirmed = !uncertain_stop;
          if (picks <= pick_failures) {
            result->error_code = "OBJECT_NOT_FOUND";
            result->retryable = true;
            result->motion_started = moved_failure;
          } else if (picks == moved_failure_at) {
            result->error_code = "EXECUTION_FAILED";
            result->motion_started = true;
          } else if (cell_picks < cells) {
            ++cell_picks;
            result->success = true;
          } else {
            result->error_code = "BASKET_EMPTY"; // As ffw_pick: not retryable, nothing moved.
            result->message = "Basket empty in consecutive observations";
          }
          if (hold && result->success) { // As ffw_pick: holds once a path is chosen.
            events.push_back(refine->running() ? "held_pick_during_refine" : "held_pick");
            held_pick = handle;
            held_result = result;
            return;
          }
          if (hold) events.push_back(refine->running() ? "early_result_during_refine" : "early_result");
          else events.push_back("pick");
          if (result->success) handle->succeed(result); else handle->abort(result);
        });
    if (prepare) {
      release_service = node->create_service<std_srvs::srv::Trigger>("/dynamixel_pick/release",
          [this](const std::shared_ptr<rmw_request_id_t> header,
              const std_srvs::srv::Trigger::Request::SharedPtr) {
            events.push_back(refine->running() ? "release_during_refine" : "release");
            if (defer_release_processing) {
              delayed_release_header = header;
            } else {
              process_release(header);
            }
          });
      auto probe = node->create_client<std_srvs::srv::Trigger>("/dynamixel_pick/release");
      ASSERT_TRUE(probe->wait_for_service(2s));
    }
    client = rclcpp_action::create_client<Demo>(node, "/run_demo");
    loop.add_node(node);
    ASSERT_TRUE(client->wait_for_action_server(2s));
    if (separate_backend) {
      auto probe = rclcpp_action::create_client<Pick>(node, "/dynamixel_pick/execute");
      ASSERT_TRUE(probe->wait_for_action_server(2s));
    }
  }
  void process_release(const std::shared_ptr<rmw_request_id_t> & header) {
    ++releases_processed;
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    response->success = held_pick && !refuse_release;
    if (response->success) {
      held_pick->succeed(held_result);
      held_pick.reset();
    }
    if (defer_release_response) {
      delayed_release_header = header;
      delayed_release_response = response;
    } else {
      release_service->send_response(*header, *response);
    }
  }
  void pump() {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = node->now();
    joints.name = {"left_joint", "right_joint", "left_grip", "right_grip"};
    joints.position = {0, 0, 0, 0};
    joints.velocity = {0, 0, 0, 0};
    context->observe_joints(joints);
    if (basket_seen) {
      geometry_msgs::msg::PoseStamped basket;
      basket.header.frame_id = config.planning_frame;
      basket.header.stamp = node->now() - rclcpp::Duration::from_seconds(basket_age);
      basket.pose.orientation.w = 1;
      context->observe_basket(basket);
    }
    if (separate_backend && run_backend) backend_loop.spin_some();
    loop.spin_some();
    if (pending_pick && pending_pick->is_canceling() && !manual_cancel) {
      auto result = std::make_shared<Pick::Result>();
      result->stop_confirmed = true;
      result->error_code = "CANCELLED";
      pending_pick->canceled(result);
      pending_pick.reset();
    }
    if (held_pick && held_pick->is_canceling() && !manual_cancel) finish_held_cancel();
    // Repeated (one sent before the goal response is dropped) but sparse: feedback is taken first and
    // a message every spin would starve the client's goal response (rclcpp_action is_ready order).
    if (held_pick && !held_pick->is_canceling() && ++pumps % 25 == 0) {
      auto feedback = std::make_shared<Pick::Feedback>();
      feedback->phase = "PICK_HOLD";
      held_pick->publish_feedback(feedback);
    }
    std::this_thread::sleep_for(2ms);
  }
  void finish_held_cancel() {
    auto result = std::make_shared<Pick::Result>();
    result->stop_confirmed = true;
    result->error_code = "CANCELLED";
    held_pick->canceled(result);
    held_pick.reset();
  }
  template<class Future> void wait(Future & future) {
    auto end = std::chrono::steady_clock::now() + 5s;
    while (future.wait_for(0s) != std::future_status::ready && std::chrono::steady_clock::now() < end) pump();
    if (future.wait_for(0s) != std::future_status::ready) {
      ADD_FAILURE() << "timed out after 5 s";
      throw std::runtime_error("timed out"); // A later get() would block the test process.
    }
  }
  rclcpp_action::ClientGoalHandle<Demo>::SharedPtr send(const std::string & command = "start") {
    pump();
    Demo::Goal goal;
    goal.command = command;
    goal.cup_id = "cup";
    auto accepted = client->async_send_goal(goal);
    wait(accepted);
    return accepted.get();
  }
  void TearDown() override {
    loop.remove_node(node);
    if (separate_backend) backend_loop.remove_node(backend_node);
    for (auto * goal : {&held_pick, &pending_pick}) { // Unfinished goals end here, not in a destructor.
      if (*goal && (*goal)->is_active()) {
        auto result = std::make_shared<Pick::Result>();
        result->stop_confirmed = true;
        if ((*goal)->is_canceling()) (*goal)->canceled(result); else (*goal)->abort(result);
      }
      goal->reset();
    }
    release_service.reset();
    client.reset(); backend.reset(); backend_node.reset(); manager.reset();
    place.reset(); handover_back.reset(); pour.reset(); handover_cup.reset();
    pick_cup.reset(); withdraw.reset(); refine.reset(); approach.reset(); initial.reset(); handover.reset(); context.reset(); node.reset();
  }
};

// Same fixture with the old WITHDRAW_CUP stage enabled.
class DemoWithdrawTest : public DemoTest {
protected:
  void SetUp() override {
    withdraw_enabled = true;
    DemoTest::SetUp();
  }
};

TEST_F(DemoTest, StartupDoesNotMove) {
  for (int i = 0; i < 30; ++i) pump();
  EXPECT_EQ(picks, 0);
  EXPECT_TRUE(executor.executed.empty());
  EXPECT_EQ(approach->runs, 0);
  EXPECT_EQ(refine->runs, 0);
}

TEST_F(DemoTest, StandaloneCupActionsUseTheirAssignedHeadPose) {
  struct Task {
    std::string action, arm;
    Cup * module;
    std::string head;
  };
  const std::vector<Task> tasks = {
      {"/approach_cup", "left", approach.get(), "look_left"},
      {"/refine_cup", "left", refine.get(), "look_left"},
      {"/pick_cup", "left", pick_cup.get(), "look_left"},
      {"/handover_cup", "left", handover_cup.get(), "look_middle"},
      {"/pour_cup", "right", pour.get(), "look_right"},
      {"/handover_cup_back", "right", handover_back.get(), "look_middle"},
      {"/place_cup", "left", place.get(), "look_left"},
      {"/withdraw_cup", "left", withdraw.get(), "look_middle"}};
  for (const auto & task : tasks) {
    auto action_client = rclcpp_action::create_client<action::ApproachCup>(node, task.action);
    ASSERT_TRUE(action_client->wait_for_action_server(2s));
    action::ApproachCup::Goal request;
    request.arm = task.arm;
    request.cup_id = "cup";
    request.execute = true;
    auto accepted = action_client->async_send_goal(request); wait(accepted);
    ASSERT_TRUE(accepted.get());
    auto result = action_client->async_get_result(accepted.get()); wait(result);
    EXPECT_TRUE(result.get().result->success) << task.action;
    EXPECT_EQ(task.module->runs, 1) << task.action;
    ASSERT_FALSE(head.commands.empty()) << task.action;
    EXPECT_EQ(head.commands.back(), task.head) << task.action;
  }
  // Same pose as the previous goal is not sent again.
  EXPECT_EQ(head.commands, (std::vector<std::string>{"look_left", "look_middle",
      "look_right", "look_middle", "look_left", "look_middle"}));
  auto manual = rclcpp_action::create_client<action::Handover>(node, "/handover");
  ASSERT_TRUE(manual->wait_for_action_server(2s));
  action::Handover::Goal manual_goal;
  manual_goal.direction = "right_to_left";
  auto accepted = manual->async_send_goal(manual_goal); wait(accepted);
  ASSERT_TRUE(accepted.get());
  auto result = manual->async_get_result(accepted.get()); wait(result);
  EXPECT_EQ(head.commands.size(), 6u); // Handover is look_middle, already sent.
}

TEST_F(DemoTest, StandalonePourStartsWithItsHeadCommandWithoutWaiting) {
  auto action_client = rclcpp_action::create_client<action::ApproachCup>(node, "/pour_cup");
  ASSERT_TRUE(action_client->wait_for_action_server(2s));
  action::ApproachCup::Goal request;
  request.arm = "right";
  request.cup_id = "cup";
  request.execute = true;
  auto accepted = action_client->async_send_goal(request); wait(accepted);
  ASSERT_TRUE(accepted.get());
  EXPECT_EQ(pour->runs, 1); // Started in the acceptance callback that sent the head goal.
  EXPECT_EQ(head.commands, (std::vector<std::string>{"look_right"}));
  auto result = action_client->async_get_result(accepted.get()); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_TRUE(head.in_flight); // The head goal never finished.
  EXPECT_TRUE(manager->can_shutdown());
}

TEST_F(DemoTest, StandalonePourRunsWhenHeadCommandIsRefused) {
  head.refuse = true;
  auto action_client = rclcpp_action::create_client<action::ApproachCup>(node, "/pour_cup");
  ASSERT_TRUE(action_client->wait_for_action_server(2s));
  action::ApproachCup::Goal request;
  request.arm = "right";
  request.cup_id = "cup";
  request.execute = true;
  auto accepted = action_client->async_send_goal(request); wait(accepted);
  ASSERT_TRUE(accepted.get());
  auto result = action_client->async_get_result(accepted.get()); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(pour->runs, 1);
  EXPECT_EQ(head.commands, (std::vector<std::string>{"look_right"}));
}

TEST_F(DemoTest, CupPreviewSendsNoHeadCommand) {
  auto action_client = rclcpp_action::create_client<action::ApproachCup>(node, "/pour_cup");
  ASSERT_TRUE(action_client->wait_for_action_server(2s));
  action::ApproachCup::Goal request;
  request.arm = "right";
  request.cup_id = "cup";
  request.execute = false;
  auto accepted = action_client->async_send_goal(request); wait(accepted);
  ASSERT_TRUE(accepted.get());
  auto result = action_client->async_get_result(accepted.get()); wait(result);
  EXPECT_EQ(pour->runs, 1);
  EXPECT_TRUE(head.commands.empty());
}

TEST_F(DemoTest, FullDemoRunsEveryStageWhenHeadCommandsAreRefused) {
  head.refuse = true;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_FALSE(manager->demo_faulted());
  EXPECT_EQ(picks, 2);
  EXPECT_EQ(executor.executed,
      (std::vector<std::string>{"initial_test", "handover_test", "initial_test"}));
  EXPECT_EQ(place->runs, 1);
  // Nothing was sent, so each stage tries its pose again.
  EXPECT_EQ(head.commands,
      (std::vector<std::string>{"look_right", "look_middle", "look_left", "look_left",
          "look_right", "look_left", "look_middle", "look_right", "look_middle", "look_left"}));
  auto again = send(); ASSERT_TRUE(again); // No latch from refused head commands.
  auto finished = client->async_get_result(again); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
}

TEST_F(DemoTest, InitialOnlyNeedsNoPickBackendAndStopsBeforePick) {
  backend.reset();
  auto goal = send("initial"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.get().result->last_completed_stage, "INITIALIZE_ROBOT");
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test"}));
  EXPECT_EQ(picks, 0);
  EXPECT_EQ(approach->runs, 0);
  EXPECT_EQ(refine->runs, 0);
  EXPECT_FALSE(send("resume"));
}

TEST_F(DemoTest, InitialPoseSendsNoHeadCommandAndCancelsTheLiveOne) {
  auto action_client = rclcpp_action::create_client<action::ApproachCup>(node, "/pour_cup");
  ASSERT_TRUE(action_client->wait_for_action_server(2s));
  action::ApproachCup::Goal request;
  request.arm = "right";
  request.cup_id = "cup";
  request.execute = true;
  auto accepted = action_client->async_send_goal(request); wait(accepted);
  ASSERT_TRUE(accepted.get());
  auto poured = action_client->async_get_result(accepted.get()); wait(poured);
  ASSERT_TRUE(head.in_flight);
  auto goal = send("initial"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(head.commands, (std::vector<std::string>{"look_right"}));
  EXPECT_EQ(head.cancels, 1);
  EXPECT_FALSE(head.in_flight);
  // The initial sequence left the head at look_middle; a middle task sends nothing.
  auto back = rclcpp_action::create_client<action::ApproachCup>(node, "/handover_cup_back");
  ASSERT_TRUE(back->wait_for_action_server(2s));
  auto sent = back->async_send_goal(request); wait(sent);
  ASSERT_TRUE(sent.get());
  auto done = back->async_get_result(sent.get()); wait(done);
  EXPECT_EQ(head.commands, (std::vector<std::string>{"look_right"}));
}

TEST_F(DemoTest, InitialPoseReleasesHeldCupModelBeforeOpeningGrippers) {
  auto goal = send("initial"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  ASSERT_EQ(scene.applied.size(), 1u);
  const auto & release = scene.applied[0];
  ASSERT_EQ(release.robot_state.attached_collision_objects.size(), 1u);
  const auto & held = release.robot_state.attached_collision_objects[0].object;
  EXPECT_EQ(held.id, "demo_place_cup");
  EXPECT_EQ(held.operation, held.REMOVE);
  ASSERT_EQ(release.world.collision_objects.size(), 1u);
  EXPECT_EQ(release.world.collision_objects[0].id, "demo_place_cup");
  EXPECT_EQ(release.world.collision_objects[0].operation, held.REMOVE);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test"}));
}

TEST_F(DemoTest, InitialPoseDoesNotMoveWhenCupModelReleaseFails) {
  scene.outcome = {ExecutionState::Failed, "SCENE_REJECTED", "test rejection"};
  auto goal = send("initial"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "SCENE_REJECTED");
  EXPECT_TRUE(executor.executed.empty());
}

TEST_F(DemoTest, InitialOnlyCanBeCancelledWithoutStartingPick) {
  executor.stall = true;
  auto goal = send("initial"); ASSERT_TRUE(goal);
  for (int i = 0; i < 50 && executor.executed.empty(); ++i) pump();
  ASSERT_FALSE(executor.executed.empty());
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 0);
}

TEST_F(DemoTest, SequencesAllMotionsAndRetriesOnlyFailedStage) {
  pick_failures = 1;
  approach->failures = 1;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(picks, 3); // Failed, cell, then the empty-basket check.
  EXPECT_EQ(executor.executed,
      (std::vector<std::string>{"initial_test", "handover_test", "initial_test"}));
  // Stage order; retries and REFINE_CUP repeat no pose; initial stages send none.
  EXPECT_EQ(head.commands,
      (std::vector<std::string>{"look_right", "look_middle", "look_left", "look_right",
          "look_left", "look_middle", "look_right", "look_middle", "look_left"}));
  EXPECT_EQ(head.cancels, 2); // INITIALIZE_ROBOT and RETURN_INITIAL.
  EXPECT_FALSE(head.in_flight);
  EXPECT_EQ(approach->runs, 2);
  EXPECT_EQ(refine->runs, 1);
  EXPECT_EQ(withdraw->runs, 0); // Disabled by default.
  EXPECT_EQ(pick_cup->runs, 1);
  EXPECT_EQ(result.get().result->holding_state, "UNVERIFIED");
  EXPECT_EQ(pick_cup->requested_arm, "left");
  // Cup routine after the pick: each hand that holds or takes the cup, once.
  EXPECT_EQ(handover_cup->runs, 1);
  EXPECT_EQ(handover_cup->requested_arm, "left");
  EXPECT_EQ(pour->runs, 1);
  EXPECT_EQ(pour->requested_arm, "right");
  EXPECT_EQ(handover_back->runs, 1);
  EXPECT_EQ(handover_back->requested_arm, "right");
  EXPECT_EQ(place->runs, 1);
  EXPECT_EQ(place->requested_arm, "left");
  EXPECT_EQ(result.get().result->last_completed_stage, "RETURN_INITIAL");
}
TEST_F(DemoWithdrawTest, EnabledWithdrawalRunsBetweenReleaseAndReturn) {
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(withdraw->runs, 1);
  EXPECT_EQ(withdraw->requested_arm, "both");
  EXPECT_EQ(pick_cup->runs, 1);
  EXPECT_EQ(executor.executed,
      (std::vector<std::string>{"initial_test", "handover_test", "initial_test"}));
  EXPECT_EQ(result.get().result->last_completed_stage, "RETURN_INITIAL");
}
TEST_F(DemoTest, CupPickRetriesOnlyMissingCup) {
  pick_cup->failures = 1;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(pick_cup->runs, 2);
  EXPECT_EQ(executor.executed.size(), 3u);
}
TEST_F(DemoTest, CupPickExecutionFailureStopsWithoutResume) {
  pick_cup->failures = 1;
  pick_cup->failure = "EXECUTION_FAILED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(result.get().result->last_completed_stage, "REFINE_CUP");
  EXPECT_EQ(pick_cup->runs, 1);
  EXPECT_EQ(picks, 2); // Cell and empty-basket check.
  EXPECT_EQ(refine->runs, 1);
}
TEST_F(DemoTest, ExhaustedObservationRetriesResumeWithoutRepicking) {
  approach->failures = 3;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  ASSERT_TRUE(result.get().result->can_resume);
  EXPECT_EQ(picks, 1);
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  auto finished = client->async_get_result(resumed); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
  EXPECT_EQ(picks, 2); // No repick; only the empty-basket check after the round.
  EXPECT_EQ(executor.executed.size(), 3u);
}
TEST_F(DemoTest, PickOnlyCanContinueWithHandover) {
  const auto initial_sequence = config.motion_sequences.at("initial_pose");
  config.motion_sequences.erase("initial_pose"); // Pick must not require an initialization sequence.
  auto goal = send("pick"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_TRUE(executor.executed.empty());
  EXPECT_EQ(head.commands, (std::vector<std::string>{"look_right"}));
  config.motion_sequences["initial_pose"] = initial_sequence; // Full continuation returns here.
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  auto finished = client->async_get_result(resumed); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
  EXPECT_EQ(picks, 2); // The resumed round ends with the empty-basket check.
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"handover_test", "initial_test"}));
  EXPECT_EQ(head.commands,
      (std::vector<std::string>{"look_right", "look_middle", "look_left", "look_right",
          "look_left", "look_middle", "look_right", "look_middle", "look_left"}));
}

TEST_F(DemoTest, PourFailureStopsWithoutResumeHandoverBackOrReturn) {
  pour->failures = 1;
  pour->failure = "EXECUTION_FAILED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(result.get().result->last_completed_stage, "HANDOVER_CUP");
  EXPECT_EQ(pour->runs, 1);
  EXPECT_EQ(handover_back->runs, 0);
  EXPECT_EQ(place->runs, 0);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test", "handover_test"}));
}

TEST_F(DemoTest, PlaceFailureNeverReturnsToInitialWithTheCup) {
  place->failures = 1;
  place->failure = "PLANNING_FAILED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->last_completed_stage, "HANDOVER_CUP_BACK");
  EXPECT_EQ(place->runs, 1); // Not retried: the left hand holds the cup.
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test", "handover_test"}));
}

TEST_F(DemoTest, FailedRefinementNeverStartsReturnToInitial) {
  refine->failures = 1;
  refine->failure = "EXECUTION_FAILED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->last_completed_stage, "APPROACH_CUP");
  EXPECT_EQ(withdraw->runs, 0);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test", "handover_test"}));
}

TEST_F(DemoWithdrawTest, WithdrawalFailureNeverStartsReturnOrRepeatsRelease) {
  withdraw->failures = 1;
  withdraw->failure = "EXECUTION_FAILED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->last_completed_stage, "REFINE_CUP");
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(refine->runs, 1);
  EXPECT_EQ(withdraw->runs, 1);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test", "handover_test"}));
}

TEST_F(DemoTest, StandaloneWithdrawalDoesNotStartReleaseOrInitialPose) {
  backend.reset();
  pump();
  auto manual = rclcpp_action::create_client<action::ApproachCup>(node, "/withdraw_cup");
  ASSERT_TRUE(manual->wait_for_action_server(2s));
  action::ApproachCup::Goal request;
  request.arm = "left";
  request.cup_id = "cup";
  request.execute = true;
  auto accepted = manual->async_send_goal(request); wait(accepted);
  ASSERT_TRUE(accepted.get());
  auto result = manual->async_get_result(accepted.get()); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(withdraw->runs, 1);
  EXPECT_EQ(refine->runs, 0);
  EXPECT_EQ(picks, 0);
  EXPECT_TRUE(executor.executed.empty());
}

TEST_F(DemoTest, CupPickCanBeCancelledWithoutRepeatingRelease) {
  pick_cup->stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && pick_cup->runs == 0; ++i) pump();
  ASSERT_EQ(pick_cup->runs, 1);
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 2); // Cell and empty-basket check.
  EXPECT_EQ(refine->runs, 1);
  EXPECT_EQ(executor.executed.size(), 2u);
  EXPECT_TRUE(manager->can_shutdown());
}
TEST_F(DemoTest, ExecutionFailureDoesNotReplayPickOrRelease) {
  pick_failures = 5; moved_failure = true;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(picks, 1);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test"}));
  EXPECT_EQ(refine->runs, 0);
}
TEST_F(DemoTest, CancellationWaitsForBackendAndBlocksOtherGoals) {
  stall_pick = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 500 && !pending_pick; ++i) pump();
  ASSERT_TRUE(pending_pick);
  EXPECT_FALSE(send());
  auto manual = rclcpp_action::create_client<action::Handover>(node, "/handover");
  action::Handover::Goal manual_goal;
  manual_goal.direction = "right_to_left";
  auto rejected = manual->async_send_goal(manual_goal); wait(rejected);
  EXPECT_FALSE(rejected.get());
  auto cup_client = rclcpp_action::create_client<action::ApproachCup>(node, "/approach_cup");
  action::ApproachCup::Goal cup_goal;
  cup_goal.arm = "left"; cup_goal.cup_id = "cup"; cup_goal.execute = true;
  auto cup_rejected = cup_client->async_send_goal(cup_goal); wait(cup_rejected);
  EXPECT_FALSE(cup_rejected.get());
  EXPECT_TRUE(head.in_flight); // look_right never finishes.
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test"}));
  EXPECT_FALSE(head.in_flight); // Cancel requested; its result is not awaited.
  EXPECT_TRUE(manager->can_shutdown());
}
TEST_F(DemoTest, UnknownStopLatchesFault) {
  uncertain_stop = true;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().result->error_code, "STOP_UNCONFIRMED");
  EXPECT_FALSE(send());
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test"}));
}
TEST_F(DemoTest, InitializationFailureNeverStartsPick) {
  executor.rejected_step = "initial_test";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(picks, 0);
  EXPECT_EQ(approach->runs, 0);
}
TEST_F(DemoTest, InitializationCanBeCancelledBeforeAnyPick) {
  executor.stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 50 && executor.executed.empty(); ++i) pump();
  ASSERT_FALSE(executor.executed.empty());
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 0);
  EXPECT_TRUE(manager->can_shutdown());
}
TEST(DemoRecovery, DoesNotRetryReleaseOrArbitraryExecutionFailures) {
  EXPECT_FALSE(demo_can_retry("HANDOVER", "EXECUTION_FAILED"));
  EXPECT_FALSE(demo_can_retry("REFINE_CUP", "EXECUTION_FAILED"));
  EXPECT_TRUE(demo_can_retry("REFINE_CUP", "WRIST_NOT_READY"));
  EXPECT_TRUE(demo_can_retry("PICK_CUP", "CUP_NOT_FOUND"));
  EXPECT_FALSE(demo_can_retry("PICK_CUP", "SCENE_UNAVAILABLE"));
  EXPECT_FALSE(demo_can_retry("PICK_CUP", "EXECUTION_FAILED"));
  for (const std::string stage : {"HANDOVER_CUP", "POUR_CUP", "HANDOVER_CUP_BACK", "PLACE_CUP"}) {
    EXPECT_FALSE(demo_can_retry(stage, "PLANNING_FAILED")) << stage;
    EXPECT_FALSE(demo_can_retry(stage, "EXECUTION_FAILED")) << stage;
  }
}
} // namespace
} // namespace demo_1f

namespace demo_1f {
namespace {
TEST_F(DemoTest, NewStartAndInitialAllowedAfterCompletedPickFailure) {
  pick_failures = 1;
  moved_failure = true;
  auto goal = send("pick"); ASSERT_TRUE(goal);
  auto failed = client->async_get_result(goal); wait(failed);
  ASSERT_FALSE(failed.get().result->success);
  auto initial_goal = send("initial"); ASSERT_TRUE(initial_goal);
  auto initialized = client->async_get_result(initial_goal); wait(initialized);
  ASSERT_TRUE(initialized.get().result->success);
  auto start_goal = send(); ASSERT_TRUE(start_goal);
  auto finished = client->async_get_result(start_goal); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
}

// Same fixture with demo.max_cell_rounds = 2.
class DemoRoundLimitTest : public DemoTest {
protected:
  void SetUp() override {
    max_rounds = 2;
    DemoTest::SetUp();
  }
};

long handovers(const Executor & executor) {
  return std::count(executor.executed.begin(), executor.executed.end(), "handover_test");
}

TEST_F(DemoTest, TwoCellRoundsThenCupSectionOnce) {
  cells = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(picks, 3);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{
      "initial_test", "handover_test", "handover_test", "initial_test"}));
  EXPECT_EQ(approach->runs, 2);
  EXPECT_EQ(refine->runs, 2);
  for (const auto * cup : {pick_cup.get(), handover_cup.get(), pour.get(), handover_back.get(),
           place.get()}) {
    EXPECT_EQ(cup->runs, 1);
  }
  EXPECT_EQ(result.get().result->last_completed_stage, "RETURN_INITIAL");
}

TEST_F(DemoTest, EmptyBasketAtFirstPickGoesStraightToCupSection) {
  cells = 0;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(picks, 1);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test", "initial_test"}));
  EXPECT_EQ(approach->runs, 0);
  EXPECT_EQ(refine->runs, 0);
  EXPECT_EQ(pick_cup->runs, 1);
  EXPECT_EQ(place->runs, 1);
  EXPECT_EQ(result.get().result->last_completed_stage, "RETURN_INITIAL");
}

TEST_F(DemoTest, EmptyPickWithoutBasketViewRetriesThenStopsBeforeCupSection) {
  cells = 0;
  basket_seen = false;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "BASKET_UNKNOWN");
  EXPECT_TRUE(result.get().result->can_resume);
  EXPECT_EQ(picks, 3); // demo.max_retries = 2.
  EXPECT_EQ(pick_cup->runs, 0);
  EXPECT_EQ(executor.executed, (std::vector<std::string>{"initial_test"}));
}

TEST_F(DemoTest, EmptyPickWithStaleBasketViewStopsBeforeCupSection) {
  cells = 0;
  basket_age = 10; // Over the 5 s demo.basket_empty_max_age default.
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().result->error_code, "BASKET_UNKNOWN");
  EXPECT_TRUE(result.get().result->can_resume);
  EXPECT_EQ(picks, 3);
  EXPECT_EQ(pick_cup->runs, 0);
}

TEST_F(DemoTest, PickOnlyEmptyBasketFailsWithItsCode) {
  cells = 0;
  auto goal = send("pick"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "BASKET_EMPTY");
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(picks, 1);
  EXPECT_EQ(pick_cup->runs, 0);
}

TEST_F(DemoTest, MovedPickFailureInSecondRoundStopsWithoutRepeating) {
  cells = 3;
  moved_failure_at = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(result.get().result->last_completed_stage, "REFINE_CUP");
  EXPECT_EQ(picks, 2);
  EXPECT_EQ(handovers(executor), 1);
  EXPECT_EQ(refine->runs, 1);
  EXPECT_EQ(pick_cup->runs, 0);
}

TEST_F(DemoRoundLimitTest, RoundLimitStopsResumableAndResumeCountsAgain) {
  cells = 3;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "CELL_ROUND_LIMIT");
  EXPECT_TRUE(result.get().result->can_resume);
  EXPECT_EQ(result.get().result->last_completed_stage, "REFINE_CUP");
  EXPECT_EQ(picks, 2);
  EXPECT_EQ(refine->runs, 2);
  EXPECT_EQ(pick_cup->runs, 0);
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  auto finished = client->async_get_result(resumed); wait(finished);
  EXPECT_TRUE(finished.get().result->success); // Round 3 is the first of the new count.
  EXPECT_EQ(picks, 4);
  EXPECT_EQ(refine->runs, 3);
  EXPECT_EQ(pick_cup->runs, 1);
}

TEST_F(DemoTest, CancelInSecondRoundDoesNotRepeatTheFinishedRound) {
  cells = 2;
  stall_pick_at = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 500 && !pending_pick; ++i) pump();
  ASSERT_TRUE(pending_pick);
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 2);
  EXPECT_EQ(handovers(executor), 1);
  EXPECT_EQ(approach->runs, 1);
  EXPECT_EQ(refine->runs, 1);
  EXPECT_EQ(pick_cup->runs, 0);
  EXPECT_TRUE(manager->can_shutdown());
}

TEST_F(DemoTest, ResumeAfterSecondRoundFailureContinuesWithoutRepicking) {
  cells = 2;
  approach->first_failure = 2; // Second round's approach fails past demo.max_retries.
  approach->failures = 3;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  ASSERT_TRUE(result.get().result->can_resume);
  EXPECT_EQ(result.get().result->last_completed_stage, "HANDOVER");
  EXPECT_EQ(picks, 2);
  EXPECT_EQ(handovers(executor), 2);
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  auto finished = client->async_get_result(resumed); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
  EXPECT_EQ(picks, 3); // Only the empty-basket check.
  EXPECT_EQ(handovers(executor), 2);
  EXPECT_EQ(approach->runs, 5);
  EXPECT_EQ(refine->runs, 2);
  EXPECT_EQ(pick_cup->runs, 1);
}

// demo.prepare_pick_during_refine on; REFINE_CUP ends only after its early pick request arrived.
class DemoPrepareTest : public DemoTest {
protected:
  void SetUp() override {
    prepare = true;
    initial_target = 0; // The right arm reports the initial pose.
    DemoTest::SetUp();
    refine->may_finish = [this] { return early_picks >= refine->runs; };
  }
};

TEST_F(DemoPrepareTest, PickRequestedDuringRefineMovesOnlyAfterReleaseAndEarlyEmptyIsUsed) {
  cells = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  // Round 2's pick holds until PICK_DYNAMIXEL releases it; round 3's finds the basket empty early.
  EXPECT_EQ(events, (std::vector<std::string>{
      "pick", "held_pick_during_refine", "release", "early_result_during_refine"}));
  EXPECT_EQ(picks, 3);
  EXPECT_EQ(handovers(executor), 2);
  EXPECT_EQ(refine->runs, 2);
  EXPECT_EQ(pick_cup->runs, 1);
  EXPECT_EQ(result.get().result->last_completed_stage, "RETURN_INITIAL");
}

TEST_F(DemoPrepareTest, RefineFailureCancelsHeldPickAndWaitsForItsResult) {
  cells = 2;
  manual_cancel = true;
  refine->failures = 1;
  refine->failure = "EXECUTION_FAILED"; // Not retryable.
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal);
  for (int i = 0; i < 1000 && !(held_pick && held_pick->is_canceling()); ++i) pump();
  ASSERT_TRUE(held_pick && held_pick->is_canceling());
  for (int i = 0; i < 50; ++i) pump();
  EXPECT_NE(result.wait_for(0s), std::future_status::ready); // No demo result before the pick's.
  EXPECT_FALSE(manager->can_shutdown());
  finish_held_cancel();
  wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "EXECUTION_FAILED");
  EXPECT_EQ(events, (std::vector<std::string>{"pick", "held_pick_during_refine"}));
  EXPECT_EQ(handovers(executor), 1);
  EXPECT_TRUE(manager->can_shutdown());
}

TEST_F(DemoPrepareTest, RefusedReleaseCancelsThePickBeforeAnyMotion) {
  cells = 3;
  refuse_release = true;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "PICK_RELEASE_FAILED");
  EXPECT_EQ(events, (std::vector<std::string>{"pick", "held_pick_during_refine", "release"}));
  EXPECT_EQ(handovers(executor), 1);
  EXPECT_TRUE(manager->can_shutdown());
}

TEST_F(DemoTest, WithoutPrepareEveryPickStartsAtItsOwnStage) {
  cells = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(events, (std::vector<std::string>{"pick", "pick", "pick"}));
  EXPECT_EQ(early_picks, 0);
}

TEST_F(DemoTest, RetiredFeedbackAndResultCannotChangeCurrentPick) {
  auto first = send("pick"); ASSERT_TRUE(first);
  auto first_result = client->async_get_result(first); wait(first_result);
  ASSERT_TRUE(first_result.get().result->success);
  const auto old = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(old && old->result);
  const auto old_result = old->result;
  stall_pick = true;
  auto second = send("pick"); ASSERT_TRUE(second);
  for (int i = 0; i < 500; ++i) {
    const auto request = TaskManagerTestAccess::current(*manager);
    if (pending_pick && request && request->handle) break;
    pump();
  }
  ASSERT_TRUE(pending_pick);
  const auto current = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(current && current->handle);
  EXPECT_NE(old->id.run, current->id.run);
  EXPECT_NE(old->id.request, current->id.request);
  const auto current_handle = current->handle;
  const auto current_phase = current->phase;
  const auto current_detail = TaskManagerTestAccess::detail(*manager);
  auto feedback = std::make_shared<Pick::Feedback>();
  feedback->phase = "PICK_HOLD";
  feedback->detail = "Retired request feedback";
  TaskManagerTestAccess::feedback(*manager, old, feedback);
  rclcpp_action::ClientGoalHandle<Pick>::WrappedResult duplicate;
  duplicate.code = rclcpp_action::ResultCode::ABORTED;
  duplicate.result = std::make_shared<Pick::Result>();
  duplicate.result->error_code = "STALE_RESULT";
  TaskManagerTestAccess::result(*manager, old, duplicate);
  EXPECT_EQ(old->result, old_result);
  EXPECT_EQ(TaskManagerTestAccess::current(*manager), current);
  EXPECT_EQ(current->handle, current_handle);
  EXPECT_EQ(current->phase, current_phase);
  EXPECT_EQ(TaskManagerTestAccess::detail(*manager), current_detail);
  EXPECT_FALSE(current->result);
  EXPECT_FALSE(current->holding);
  EXPECT_FALSE(current->cancel_wanted);
  EXPECT_FALSE(manager->demo_faulted());
  auto cancel = client->async_cancel_goal(second); wait(cancel);
  auto finished = client->async_get_result(second); wait(finished);
  EXPECT_EQ(finished.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 2);
}

class DemoLateAcceptanceTest : public DemoTest {
protected:
  void SetUp() override {
    separate_backend = true;
    run_backend = false;
    stall_pick = true;
    DemoTest::SetUp();
  }
};

TEST_F(DemoLateAcceptanceTest, CancellationBeforeAcceptanceCancelsThatGoalAndWaitsForItsResult) {
  auto goal = send("pick"); ASSERT_TRUE(goal);
  for (int i = 0; i < 500 && !TaskManagerTestAccess::current(*manager); ++i) pump();
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request);
  ASSERT_TRUE(request->pending);
  EXPECT_FALSE(request->handle);
  EXPECT_EQ(picks, 0);
  auto result = client->async_get_result(goal);
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  EXPECT_TRUE(request->cancel_wanted);
  EXPECT_FALSE(request->cancel_sent);
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_NE(result.wait_for(0s), std::future_status::ready);
  EXPECT_FALSE(manager->can_shutdown());
  run_backend = true;
  wait(result);
  ASSERT_EQ(accepted_pick_ids.size(), 1u);
  ASSERT_EQ(cancelled_pick_ids.size(), 1u);
  EXPECT_EQ(cancelled_pick_ids.front(), accepted_pick_ids.front());
  EXPECT_TRUE(request->cancel_sent);
  ASSERT_TRUE(request->result);
  EXPECT_TRUE(request->result->stop_confirmed);
  EXPECT_FALSE(request->handle);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 1);
  EXPECT_FALSE(manager->demo_faulted());
  EXPECT_TRUE(manager->can_shutdown());
}

TEST_F(DemoTest, UnconfirmedCancelledRequestBlocksNewDemoAndPick) {
  stall_pick = manual_cancel = true;
  auto goal = send("pick"); ASSERT_TRUE(goal);
  for (int i = 0; i < 500 && !pending_pick; ++i) pump();
  ASSERT_TRUE(pending_pick);
  const auto request = TaskManagerTestAccess::current(*manager);
  auto result = client->async_get_result(goal);
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  for (int i = 0; i < 500 && !pending_pick->is_canceling(); ++i) pump();
  ASSERT_TRUE(pending_pick->is_canceling());
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_EQ(TaskManagerTestAccess::current(*manager), request);
  EXPECT_NE(result.wait_for(0s), std::future_status::ready);
  auto stopped = std::make_shared<Pick::Result>();
  stopped->error_code = "STOP_UNCONFIRMED";
  stopped->stop_confirmed = false;
  pending_pick->canceled(stopped);
  pending_pick.reset();
  wait(result);
  EXPECT_TRUE(manager->demo_faulted());
  EXPECT_EQ(result.get().result->error_code, "STOP_UNCONFIRMED");
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_EQ(picks, 1);
  ASSERT_TRUE(request->result);
  EXPECT_FALSE(request->result->stop_confirmed);
}

TEST_F(DemoTest, RetryAndResumeNeverReusePickOrRunIdentities) {
  pick_failures = 1;
  auto goal = send("pick"); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  ASSERT_TRUE(result.get().result->success);
  ASSERT_EQ(observed_requests.size(), 2u);
  const auto first = observed_requests[0];
  const auto retry = observed_requests[1];
  EXPECT_EQ(first->id.run, retry->id.run);
  EXPECT_NE(first->id.request, retry->id.request);
  EXPECT_EQ(retry->id.run, TaskManagerTestAccess::run(*manager));
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  const auto resumed_run = TaskManagerTestAccess::run(*manager);
  EXPECT_NE(resumed_run, retry->id.run);
  auto finished = client->async_get_result(resumed); wait(finished);
  ASSERT_TRUE(finished.get().result->success);
  ASSERT_EQ(observed_requests.size(), 3u);
  EXPECT_EQ(observed_requests[2]->id.run, resumed_run);
  EXPECT_NE(observed_requests[2]->id.request, first->id.request);
  EXPECT_NE(observed_requests[2]->id.request, retry->id.request);
}

TEST_F(DemoPrepareTest, PreparedPickTakeoverRetainsContextAndIdentityWithoutResending) {
  cells = 2;
  refine->stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && !held_pick; ++i) pump();
  ASSERT_TRUE(held_pick);
  const auto prepared = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(prepared);
  const auto id = prepared->id;
  EXPECT_TRUE(prepared->early);
  EXPECT_TRUE(prepared->hold_requested);
  EXPECT_FALSE(prepared->owned);
  EXPECT_FALSE(prepared->release_sent);
  EXPECT_EQ(picks, 2);
  refine->stall = false;
  for (int i = 0; i < 500 && !prepared->owned; ++i) pump();
  ASSERT_TRUE(prepared->owned);
  EXPECT_EQ(TaskManagerTestAccess::current(*manager), prepared);
  EXPECT_TRUE(prepared->id == id);
  EXPECT_FALSE(prepared->early);
  EXPECT_EQ(picks, 2);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_TRUE(prepared->release_sent);
  EXPECT_EQ(early_picks, 2);
  EXPECT_EQ(picks, 3);
}

TEST_F(DemoPrepareTest, LateStopAfterPreparedCancelTimeoutCleansOnlyItsRetainedRequest) {
  cells = 2;
  manual_cancel = true;
  refine->failures = 1;
  refine->failure = "EXECUTION_FAILED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal);
  for (int i = 0; i < 1000 && !(held_pick && held_pick->is_canceling()); ++i) pump();
  ASSERT_TRUE(held_pick && held_pick->is_canceling());
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request && request->handle);
  const auto deadline = std::chrono::steady_clock::now() + 12s;
  while (result.wait_for(0s) != std::future_status::ready &&
      std::chrono::steady_clock::now() < deadline) pump();
  ASSERT_EQ(result.wait_for(0s), std::future_status::ready);
  EXPECT_EQ(result.get().result->error_code, "STOP_UNCONFIRMED");
  EXPECT_TRUE(manager->demo_faulted());
  EXPECT_FALSE(request->result);
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  finish_held_cancel();
  for (int i = 0; i < 500 && !request->result; ++i) pump();
  ASSERT_TRUE(request->result);
  EXPECT_TRUE(request->result->stop_confirmed);
  EXPECT_FALSE(request->handle);
  EXPECT_FALSE(request->pending);
  EXPECT_TRUE(manager->demo_faulted());
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_EQ(picks, 2);
}

class DemoDeferredReleaseTest : public DemoPrepareTest {
protected:
  void SetUp() override {
    defer_release_response = true;
    DemoPrepareTest::SetUp();
  }

  void check_retired_release(bool success) {
    cells = 2;
    auto goal = send(); ASSERT_TRUE(goal);
    for (int i = 0; i < 1000 && !delayed_release_header; ++i) pump();
    ASSERT_TRUE(delayed_release_header && delayed_release_response);
    ASSERT_EQ(observed_requests.size(), 2u);
    const auto old = observed_requests[1];
    for (int i = 0; i < 500 && !old->result; ++i) pump();
    ASSERT_TRUE(old->result && old->result->stop_confirmed);
    ASSERT_TRUE(old->release_sent);
    EXPECT_FALSE(old->release_response_received);
    auto cancel_old = client->async_cancel_goal(goal); wait(cancel_old);
    auto result = client->async_get_result(goal); wait(result);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_FALSE(send("start"));
    EXPECT_FALSE(send("pick"));
    EXPECT_FALSE(send("resume"));
    EXPECT_EQ(picks, 2);
    EXPECT_FALSE(manager->can_shutdown());
    delayed_release_response->success = success;
    delayed_release_response->message = "Retired release response";
    release_service->send_response(*delayed_release_header, *delayed_release_response);
    for (int i = 0; i < 500 && !old->release_response_received; ++i) pump();
    ASSERT_TRUE(old->release_response_received);
    ASSERT_TRUE(old->release_response);
    EXPECT_EQ(old->release_response->success, success);
    EXPECT_EQ(old->release_error.empty(), success);
    EXPECT_FALSE(manager->demo_faulted());
    EXPECT_TRUE(manager->can_shutdown());
    stall_pick = true;
    auto next = send("pick"); ASSERT_TRUE(next);
    for (int i = 0; i < 500; ++i) {
      const auto request = TaskManagerTestAccess::current(*manager);
      if (pending_pick && request && request->handle) break;
      pump();
    }
    ASSERT_TRUE(pending_pick);
    const auto current = TaskManagerTestAccess::current(*manager);
    ASSERT_TRUE(current && current->handle);
    const auto handle = current->handle;
    const auto phase = current->phase;
    const auto id = current->id;
    auto next_result = client->async_get_result(next);
    auto duplicate_release = std::make_shared<std_srvs::srv::Trigger::Response>();
    duplicate_release->success = !success;
    TaskManagerTestAccess::release(*manager, old, duplicate_release);
    auto feedback = std::make_shared<Pick::Feedback>();
    feedback->phase = "PICK_HOLD";
    TaskManagerTestAccess::feedback(*manager, old, feedback);
    rclcpp_action::ClientGoalHandle<Pick>::WrappedResult duplicate_result;
    duplicate_result.code = rclcpp_action::ResultCode::ABORTED;
    duplicate_result.result = std::make_shared<Pick::Result>();
    TaskManagerTestAccess::result(*manager, old, duplicate_result);
    EXPECT_EQ(old->release_response->success, success);
    EXPECT_EQ(TaskManagerTestAccess::current(*manager), current);
    EXPECT_TRUE(current->id == id);
    EXPECT_EQ(current->handle, handle);
    EXPECT_EQ(current->phase, phase);
    EXPECT_FALSE(current->result);
    EXPECT_FALSE(current->holding);
    EXPECT_TRUE(current->release_error.empty());
    EXPECT_FALSE(current->cancel_wanted);
    EXPECT_FALSE(current->cancel_sent);
    EXPECT_FALSE(manager->demo_faulted());
    EXPECT_NE(next_result.wait_for(0s), std::future_status::ready);
    auto cancel = client->async_cancel_goal(next); wait(cancel);
    wait(next_result);
    EXPECT_EQ(next_result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(picks, 3);
  }
};

TEST_F(DemoDeferredReleaseTest, RetiredSuccessfulReleaseDoesNotCompleteCurrentPick) {
  check_retired_release(true);
}

TEST_F(DemoDeferredReleaseTest, RetiredRefusedReleaseDoesNotCancelCurrentPick) {
  check_retired_release(false);
}

TEST_F(DemoDeferredReleaseTest, CurrentReleaseRemainsMonitoredAfterActionResult) {
  cells = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && !delayed_release_header; ++i) pump();
  ASSERT_TRUE(delayed_release_header && delayed_release_response);
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request && request->release_sent);
  for (int i = 0; i < 500 && !request->result; ++i) pump();
  ASSERT_TRUE(request->result && request->result->stop_confirmed);
  for (int i = 0; i < 50; ++i) pump();
  EXPECT_EQ(TaskManagerTestAccess::current(*manager), request);
  EXPECT_FALSE(request->release_response_received);
  EXPECT_TRUE(request->release_future.valid());
  EXPECT_EQ(picks, 2);
  EXPECT_FALSE(send("pick"));
  release_service->send_response(*delayed_release_header, *delayed_release_response);
  for (int i = 0; i < 500 && !request->release_response_received; ++i) pump();
  ASSERT_TRUE(request->release_response_received);
  EXPECT_FALSE(request->release_timed_out);
  EXPECT_FALSE(manager->demo_faulted());
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(picks, 3);
}

class DemoDeferredProcessingTest : public DemoPrepareTest {
protected:
  void SetUp() override {
    defer_release_processing = true;
    DemoPrepareTest::SetUp();
  }
};

TEST_F(DemoDeferredProcessingTest, ActionCancellationCannotClearUnprocessedRelease) {
  cells = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && !delayed_release_header; ++i) pump();
  ASSERT_TRUE(delayed_release_header);
  ASSERT_TRUE(held_pick);
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request && request->release_sent);
  EXPECT_EQ(releases_processed, 0);
  EXPECT_FALSE(request->result);
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(request->result && request->result->stop_confirmed);
  EXPECT_FALSE(request->handle);
  EXPECT_FALSE(request->release_response_received);
  EXPECT_TRUE(request->release_future.valid());
  EXPECT_EQ(releases_processed, 0);
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_FALSE(send("resume"));
  EXPECT_FALSE(manager->demo_faulted());
  EXPECT_FALSE(manager->can_shutdown());
  EXPECT_EQ(picks, 2);
  process_release(delayed_release_header);
  for (int i = 0; i < 500 && !request->release_response_received; ++i) pump();
  ASSERT_TRUE(request->release_response_received);
  ASSERT_TRUE(request->release_response);
  EXPECT_FALSE(request->release_response->success);
  EXPECT_EQ(releases_processed, 1);
  EXPECT_FALSE(manager->demo_faulted());
  stall_pick = true;
  auto next = send("pick"); ASSERT_TRUE(next);
  for (int i = 0; i < 500 && !pending_pick; ++i) pump();
  ASSERT_TRUE(pending_pick);
  EXPECT_EQ(picks, 3);
  auto cancel_next = client->async_cancel_goal(next); wait(cancel_next);
  auto finished = client->async_get_result(next); wait(finished);
  EXPECT_EQ(finished.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(picks, 3);
}

TEST_F(DemoDeferredReleaseTest, ResultBeforeReleaseTimeoutStillLatchesFaultAndRetainsLateResponse) {
  cells = 2;
  refine->may_finish = [this] {
    return refine->runs > 1 || early_picks >= 1;
  };
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && !delayed_release_header; ++i) pump();
  ASSERT_TRUE(delayed_release_header && delayed_release_response);
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request);
  for (int i = 0; i < 500 && !request->result; ++i) pump();
  ASSERT_TRUE(request->result && request->result->stop_confirmed);
  const auto deadline = std::chrono::steady_clock::now() + 6s;
  while (!manager->demo_faulted() && std::chrono::steady_clock::now() < deadline) pump();
  ASSERT_TRUE(manager->demo_faulted());
  EXPECT_TRUE(request->release_timed_out);
  EXPECT_FALSE(request->release_response_received);
  EXPECT_TRUE(request->release_future.valid());
  EXPECT_EQ(request->release_error, "Pick release unanswered for 5 s");
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().result->error_code, "PICK_RELEASE_FAILED");
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  release_service->send_response(*delayed_release_header, *delayed_release_response);
  for (int i = 0; i < 500 && !request->release_response_received; ++i) pump();
  ASSERT_TRUE(request->release_response_received);
  EXPECT_TRUE(request->release_response->success);
  EXPECT_FALSE(request->release_future.valid());
  EXPECT_TRUE(request->release_timed_out);
  EXPECT_EQ(request->release_error, "Pick release unanswered for 5 s");
  EXPECT_TRUE(manager->demo_faulted());
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_FALSE(send("resume"));
  EXPECT_EQ(picks, 2);
}

TEST_F(DemoDeferredProcessingTest, TimeoutCancelsPickButDoesNotCancelRemoteReleaseProcessing) {
  cells = 2;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && !delayed_release_header; ++i) pump();
  ASSERT_TRUE(delayed_release_header && held_pick);
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request && request->handle);
  EXPECT_EQ(releases_processed, 0);
  const auto deadline = std::chrono::steady_clock::now() + 6s;
  while (!manager->demo_faulted() && std::chrono::steady_clock::now() < deadline) pump();
  ASSERT_TRUE(manager->demo_faulted());
  EXPECT_TRUE(request->cancel_wanted);
  EXPECT_TRUE(request->cancel_sent);
  EXPECT_TRUE(request->release_timed_out);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().result->error_code, "PICK_RELEASE_FAILED");
  ASSERT_TRUE(request->result && request->result->stop_confirmed);
  EXPECT_EQ(releases_processed, 0);
  EXPECT_FALSE(request->release_response_received);
  EXPECT_TRUE(request->release_future.valid());
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  process_release(delayed_release_header);
  for (int i = 0; i < 500 && !request->release_response_received; ++i) pump();
  ASSERT_TRUE(request->release_response_received);
  EXPECT_EQ(releases_processed, 1);
  EXPECT_TRUE(request->release_timed_out);
  EXPECT_EQ(request->release_error, "Pick release unanswered for 5 s");
  EXPECT_TRUE(manager->demo_faulted());
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_EQ(picks, 2);
}

class DemoDeferredResumeTest : public DemoDeferredReleaseTest {
protected:
  void SetUp() override {
    max_rounds = 2;
    DemoDeferredReleaseTest::SetUp();
  }
};

TEST_F(DemoDeferredResumeTest, ResumableCheckpointWaitsForReleaseConfirmation) {
  cells = 2;
  refine->may_finish = [this] {
    return refine->runs > 1 || early_picks >= 1;
  };
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  ASSERT_EQ(result.get().result->error_code, "CELL_ROUND_LIMIT");
  ASSERT_TRUE(result.get().result->can_resume);
  ASSERT_TRUE(delayed_release_header && delayed_release_response);
  const auto request = TaskManagerTestAccess::current(*manager);
  ASSERT_TRUE(request && request->result && request->result->stop_confirmed);
  EXPECT_FALSE(request->release_response_received);
  EXPECT_FALSE(send("resume"));
  EXPECT_FALSE(send("start"));
  EXPECT_FALSE(send("pick"));
  EXPECT_EQ(picks, 2);
  release_service->send_response(*delayed_release_header, *delayed_release_response);
  for (int i = 0; i < 500 && !request->release_response_received; ++i) pump();
  ASSERT_TRUE(request->release_response_received);
  EXPECT_FALSE(manager->demo_faulted());
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  auto finished = client->async_get_result(resumed); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
  EXPECT_EQ(picks, 3);
}

TEST_F(DemoTest, ApproachStartsTheSideReturnAndRefineDoesNotWaitForIt) {
  side.stall = true;
  refine->stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs == 0; ++i) pump();
  ASSERT_EQ(refine->runs, 1);
  EXPECT_EQ(side.starts, 1);
  EXPECT_EQ(approach->runs, 1);
  EXPECT_EQ(side.state.state, ExecutionState::Running); // Not finished, yet REFINE_CUP runs.
  side.stall = false;
  refine->stall = false;
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
}

TEST_F(DemoTest, PickWaitsForTheSideReturnAndStartsWhenItArrives) {
  cells = 2;
  side.stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs == 0; ++i) pump();
  ASSERT_EQ(refine->runs, 1);
  for (int i = 0; i < 200; ++i) pump(); // REFINE_CUP done; the next PICK_DYNAMIXEL must wait.
  EXPECT_EQ(picks, 1);
  EXPECT_EQ(handovers(executor), 1);
  side.stall = false;
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(picks, 3);
  EXPECT_EQ(handovers(executor), 2);
}

TEST_F(DemoTest, SideFailureBeforeMotionFailsApproachWithoutStartingIt) {
  side.refuse = true;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "SIDE_BEFORE_MOTION");
  EXPECT_TRUE(result.get().result->can_resume); // Before motion: retried, then resumable.
  EXPECT_EQ(side.starts, 3);                     // demo.max_retries = 2.
  EXPECT_EQ(approach->runs, 0);
  EXPECT_EQ(result.get().result->last_completed_stage, "HANDOVER");
}

TEST_F(DemoTest, SideFailureAfterMotionEndsTheDemoWithoutResume) {
  side.stall = true;
  refine->stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs == 0; ++i) pump();
  ASSERT_EQ(refine->runs, 1);
  side.fail(false);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, "SIDE_ARM_FAILED");
  EXPECT_FALSE(result.get().result->can_resume);
  EXPECT_EQ(picks, 1);
  EXPECT_FALSE(send("resume"));
}

TEST_F(DemoTest, CancelStopsTheSideAndShutdownWaitsForItsStop) {
  side.stall = true;
  side.cancel_stalls = true;
  refine->stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs == 0; ++i) pump();
  ASSERT_EQ(refine->runs, 1);
  auto cancel = client->async_cancel_goal(goal); wait(cancel);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_GE(side.cancels, 1);
  EXPECT_FALSE(manager->can_shutdown()); // The side stop is not confirmed yet.
  EXPECT_FALSE(send());
  side.state = {ExecutionState::Cancelled, "CANCELLED", "stopped"};
  pump();
  EXPECT_TRUE(manager->can_shutdown());
}

TEST_F(DemoTest, RetryableRefineFailureKeepsTheSideReturnAndContinues) {
  side.stall = true;
  refine->failures = 1;
  refine->failure = "RIM_NOT_CONFIRMED"; // Retryable before motion.
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs < 2; ++i) pump();
  ASSERT_EQ(refine->runs, 2);
  EXPECT_EQ(side.cancels, 0);
  EXPECT_EQ(side.starts, 1);
  EXPECT_EQ(side.state.state, ExecutionState::Running);
  side.stall = false;
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
}

TEST_F(DemoTest, StageFailureEndsDemoWhileTheSideFinishesThenResumeWorks) {
  side.stall = true;
  refine->failures = 3; // Retries used up: resumable as before.
  refine->failure = "RIM_NOT_CONFIRMED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().result->error_code, "RIM_NOT_CONFIRMED");
  EXPECT_TRUE(result.get().result->can_resume);
  EXPECT_EQ(side.cancels, 0);
  EXPECT_EQ(side.state.state, ExecutionState::Running);
  EXPECT_FALSE(manager->can_shutdown());
  EXPECT_FALSE(send("resume")); // Rejected until the side return ends.
  EXPECT_FALSE(send());
  side.stall = false;
  for (int i = 0; i < 50 && !manager->can_shutdown(); ++i) pump();
  EXPECT_TRUE(manager->can_shutdown());
  auto resumed = send("resume"); ASSERT_TRUE(resumed);
  auto finished = client->async_get_result(resumed); wait(finished);
  EXPECT_TRUE(finished.get().result->success);
  EXPECT_EQ(refine->runs, 4);
}

TEST_F(DemoTest, SideFailureAfterTheDemoEndedBlocksResume) {
  side.stall = true;
  refine->failures = 3;
  refine->failure = "RIM_NOT_CONFIRMED";
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  ASSERT_TRUE(result.get().result->can_resume);
  side.fail(false);
  for (int i = 0; i < 10; ++i) pump();
  EXPECT_FALSE(send("resume"));
}

// demo.independent_return = false: the approach pairs the return itself as before.
class DemoPairedReturnTest : public DemoTest {
protected:
  void SetUp() override {
    independent = false;
    DemoTest::SetUp();
  }
};

TEST_F(DemoPairedReturnTest, NoSideReturnIsStarted) {
  pick_failures = 1;
  approach->failures = 1;
  auto goal = send(); ASSERT_TRUE(goal);
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(side.starts, 0);
  EXPECT_EQ(picks, 3);
  EXPECT_EQ(executor.executed,
      (std::vector<std::string>{"initial_test", "handover_test", "initial_test"}));
  EXPECT_EQ(approach->runs, 2);
}

TEST_F(DemoPrepareTest, EarlyPickWaitsForTheSideReturnDuringRefine) {
  cells = 2;
  side.stall = true;
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs == 0; ++i) pump();
  ASSERT_EQ(refine->runs, 1);
  for (int i = 0; i < 100; ++i) pump();
  EXPECT_EQ(early_picks, 0); // Deferred while the side arm returns.
  side.stall = false;
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(events, (std::vector<std::string>{
      "pick", "held_pick_during_refine", "release", "early_result_during_refine"}));
}

TEST_F(DemoPrepareTest, RefineEndingBeforeTheSideReturnSkipsTheEarlyPick) {
  cells = 2;
  side.stall = true;
  refine->may_finish = nullptr; // REFINE_CUP ends before the side arrives.
  auto goal = send(); ASSERT_TRUE(goal);
  for (int i = 0; i < 1000 && refine->runs == 0; ++i) pump();
  for (int i = 0; i < 200; ++i) pump();
  EXPECT_EQ(early_picks, 0);
  EXPECT_EQ(picks, 1);
  side.stall = false;
  for (int i = 0; i < 1000 && picks < 2; ++i) pump();
  ASSERT_EQ(picks, 2);
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events[1], "pick"); // An ordinary PICK_DYNAMIXEL request, not a held one.
  auto result = client->async_get_result(goal); wait(result);
  EXPECT_TRUE(result.get().result->success);
}
} // namespace
} // namespace demo_1f

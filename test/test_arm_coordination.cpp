#include "demo_1f/core/arm_coordination.hpp"
#include <gtest/gtest.h>

namespace demo_1f::coordination {
namespace {

class ArmCoordinationTest : public ::testing::Test {
protected:
  ArmCoordination policy{42, 3};

  void SetUp() override {
    ASSERT_TRUE(policy.record_initial_stop(42, Arm::Left));
    ASSERT_TRUE(policy.record_initial_stop(42, Arm::Right));
  }

  WorkId id(Operation operation) const { return {policy.current_round(), operation}; }

  void prepare(const WorkId & task, Resources resources) {
    ASSERT_TRUE(policy.prepare(task, resources));
    ASSERT_TRUE(policy.preparation_complete(task));
  }

  void finish(const WorkId & task, Outcome outcome = Outcome::Success) {
    ASSERT_TRUE(policy.execution_complete(task, outcome));
    EXPECT_FALSE(policy.release(task));
    ASSERT_TRUE(policy.confirm_stop(task, true));
    ASSERT_TRUE(policy.release(task));
  }

  void execute(const WorkId & task, Resources resources, Outcome outcome = Outcome::Success) {
    prepare(task, resources);
    ASSERT_TRUE(policy.acquire(task));
    ASSERT_TRUE(policy.command_accepted(task));
    finish(task, outcome);
  }

  WorkId first_handover() {
    const auto left = id(Operation::ReceiveReady);
    const auto pick = id(Operation::PickCell);
    const auto handover = id(Operation::HandoverCell);
    execute(left, Resources::Left);
    execute(pick, Resources::Right);
    prepare(handover, Resources::Both);
    EXPECT_TRUE(policy.start_handover(handover, left, pick));
    finish(handover);
    return handover;
  }
};

TEST(ArmCoordination, UnownedArmsStartUnknown) {
  ArmCoordination policy(42, 1);
  const WorkId task{{42, 1, 1}, Operation::PickCell};
  ASSERT_TRUE(policy.prepare(task, Resources::Right));
  ASSERT_TRUE(policy.preparation_complete(task));
  EXPECT_FALSE(policy.arm(Arm::Right).owner);
  EXPECT_FALSE(policy.arm(Arm::Right).stop_confirmed);
  EXPECT_FALSE(policy.acquire(task));
  EXPECT_FALSE(policy.record_initial_stop(41, Arm::Right));
  ASSERT_TRUE(policy.record_initial_stop(42, Arm::Right));
  EXPECT_TRUE(policy.acquire(task));
  EXPECT_FALSE(policy.record_initial_stop(42, Arm::Right));
}

TEST_F(ArmCoordinationTest, RejectsDuplicateArmAndForeignRelease) {
  const auto first = id(Operation::PickCell);
  const auto second = id(Operation::ArmTask);
  prepare(first, Resources::Right);
  prepare(second, Resources::Right);
  ASSERT_TRUE(policy.acquire(first));
  EXPECT_EQ(policy.work(first)->phase, Phase::Sending);
  EXPECT_FALSE(policy.acquire(first));
  EXPECT_FALSE(policy.acquire(second));
  EXPECT_FALSE(policy.release(second));
  ASSERT_TRUE(policy.arm(Arm::Right).owner);
  EXPECT_EQ(*policy.arm(Arm::Right).owner, first);
  finish(first);
  ASSERT_TRUE(policy.acquire(second));
  EXPECT_FALSE(policy.release(first));
  EXPECT_FALSE(policy.execution_complete(first, Outcome::Failed));
  EXPECT_FALSE(policy.confirm_stop(first, false));
  EXPECT_EQ(*policy.arm(Arm::Right).owner, second);
}

TEST_F(ArmCoordinationTest, BothArmAcquisitionIsAtomicWithEitherArmBusy) {
  for (const auto busy : {Resources::Left, Resources::Right}) {
    ArmCoordination candidate(42, 3);
    ASSERT_TRUE(candidate.record_initial_stop(42, Arm::Left));
    ASSERT_TRUE(candidate.record_initial_stop(42, Arm::Right));
    const WorkId single{{42, 1, 1}, busy == Resources::Left ?
        Operation::ReceiveReady : Operation::PickCell};
    const WorkId both{{42, 1, 1}, Operation::ArmTask};
    ASSERT_TRUE(candidate.prepare(single, busy));
    ASSERT_TRUE(candidate.preparation_complete(single));
    ASSERT_TRUE(candidate.acquire(single));
    ASSERT_TRUE(candidate.prepare(both, Resources::Both));
    ASSERT_TRUE(candidate.preparation_complete(both));
    EXPECT_FALSE(candidate.acquire(both));
    const auto free_arm = busy == Resources::Left ? Arm::Right : Arm::Left;
    EXPECT_FALSE(candidate.arm(free_arm).owner);
    EXPECT_TRUE(candidate.arm(free_arm).stop_confirmed);
    ASSERT_TRUE(candidate.execution_complete(single, Outcome::Success));
    ASSERT_TRUE(candidate.confirm_stop(single, true));
    ASSERT_TRUE(candidate.release(single));
    ASSERT_TRUE(candidate.acquire(both));
    EXPECT_EQ(*candidate.arm(Arm::Left).owner, both);
    EXPECT_EQ(*candidate.arm(Arm::Right).owner, both);
  }
}

TEST_F(ArmCoordinationTest, PreparedPickNeedsExplicitPermission) {
  const auto pick = id(Operation::PickCell);
  ASSERT_TRUE(policy.prepare(pick, Resources::Right));
  EXPECT_EQ(policy.work(pick)->phase, Phase::Preparing);
  EXPECT_FALSE(policy.acquire(pick));
  EXPECT_FALSE(policy.arm(Arm::Right).owner);
  ASSERT_TRUE(policy.preparation_complete(pick));
  EXPECT_FALSE(policy.arm(Arm::Right).owner);
  EXPECT_FALSE(policy.command_accepted(pick));
  EXPECT_FALSE(policy.execution_complete(pick, Outcome::Success));
  EXPECT_TRUE(policy.acquire(pick));
}

TEST_F(ArmCoordinationTest, FirstJoinNeedsBothSidesInEitherCompletionOrder) {
  for (const bool left_first : {false, true}) {
    ArmCoordination candidate(42, 3);
    ASSERT_TRUE(candidate.record_initial_stop(42, Arm::Left));
    ASSERT_TRUE(candidate.record_initial_stop(42, Arm::Right));
    const WorkId left{{42, 1, 1}, Operation::ReceiveReady};
    const WorkId pick{{42, 1, 1}, Operation::PickCell};
    const WorkId handover{{42, 1, 1}, Operation::HandoverCell};
    ASSERT_TRUE(candidate.prepare(left, Resources::Left));
    ASSERT_TRUE(candidate.prepare(pick, Resources::Right));
    ASSERT_TRUE(candidate.prepare(handover, Resources::Both));
    for (const auto & task : {left, pick, handover})
      ASSERT_TRUE(candidate.preparation_complete(task));
    ASSERT_TRUE(candidate.acquire(left));
    ASSERT_TRUE(candidate.acquire(pick));
    const auto first = left_first ? left : pick;
    const auto second = left_first ? pick : left;
    ASSERT_TRUE(candidate.execution_complete(first, Outcome::Success));
    ASSERT_TRUE(candidate.confirm_stop(first, true));
    ASSERT_TRUE(candidate.release(first));
    EXPECT_FALSE(candidate.handover_ready(handover, left, pick));
    ASSERT_TRUE(candidate.execution_complete(second, Outcome::Success));
    EXPECT_FALSE(candidate.handover_ready(handover, left, pick));
    ASSERT_TRUE(candidate.confirm_stop(second, true));
    EXPECT_FALSE(candidate.handover_ready(handover, left, pick));
    ASSERT_TRUE(candidate.release(second));
    EXPECT_TRUE(candidate.handover_ready(handover, left, pick));
    EXPECT_FALSE(candidate.acquire(handover));
    EXPECT_TRUE(candidate.start_handover(handover, left, pick));
    EXPECT_FALSE(candidate.start_handover(handover, left, pick));
  }
}

TEST_F(ArmCoordinationTest, NextJoinRequiresPreviousPlacementAndMatchingRound) {
  const auto previous = first_handover();
  const WorkId place{previous.round, Operation::PlaceCell};
  const auto pick = id(Operation::PickCell);
  const auto handover = id(Operation::HandoverCell);
  prepare(place, Resources::Left);
  prepare(handover, Resources::Both);
  execute(pick, Resources::Right);
  EXPECT_FALSE(policy.start_handover(handover, place, pick));
  ASSERT_TRUE(policy.acquire(place));
  finish(place);
  auto wrong = pick;
  --wrong.round.cell;
  EXPECT_FALSE(policy.start_handover(handover, place, wrong));
  wrong = pick;
  ++wrong.round.attempt;
  EXPECT_FALSE(policy.start_handover(handover, place, wrong));
  wrong = place;
  ++wrong.round.attempt;
  EXPECT_FALSE(policy.start_handover(handover, wrong, pick));
  wrong = pick;
  ++wrong.round.run;
  EXPECT_FALSE(policy.start_handover(handover, place, wrong));
  EXPECT_TRUE(policy.start_handover(handover, place, pick));
}

TEST_F(ArmCoordinationTest, NextJoinAlsoWaitsWhenPlacementFinishesFirst) {
  const auto previous = first_handover();
  const WorkId place{previous.round, Operation::PlaceCell};
  const auto pick = id(Operation::PickCell);
  const auto handover = id(Operation::HandoverCell);
  execute(place, Resources::Left);
  prepare(pick, Resources::Right);
  prepare(handover, Resources::Both);
  EXPECT_FALSE(policy.start_handover(handover, place, pick));
  ASSERT_TRUE(policy.acquire(pick));
  ASSERT_TRUE(policy.execution_complete(pick, Outcome::Success));
  EXPECT_FALSE(policy.start_handover(handover, place, pick));
  ASSERT_TRUE(policy.confirm_stop(pick, true));
  ASSERT_TRUE(policy.release(pick));
  EXPECT_TRUE(policy.start_handover(handover, place, pick));
}

TEST_F(ArmCoordinationTest, RetryCannotCombineSuccessfulOldAttemptWithNewAttempt) {
  const auto old_left = id(Operation::ReceiveReady);
  const auto old_pick = id(Operation::PickCell);
  execute(old_left, Resources::Left);
  execute(old_pick, Resources::Right);
  ASSERT_TRUE(policy.retry_round(2));
  EXPECT_FALSE(policy.prepare(old_pick, Resources::Right));
  EXPECT_FALSE(policy.command_accepted(old_pick));
  EXPECT_FALSE(policy.execution_complete(old_pick, Outcome::Success));
  const auto pick = id(Operation::PickCell);
  const auto left = id(Operation::ReceiveReady);
  const auto handover = id(Operation::HandoverCell);
  execute(pick, Resources::Right);
  prepare(handover, Resources::Both);
  EXPECT_FALSE(policy.start_handover(handover, old_left, pick));
  execute(left, Resources::Left);
  EXPECT_FALSE(policy.start_handover(handover, left, old_pick));
  EXPECT_TRUE(policy.start_handover(handover, left, pick));
}

TEST_F(ArmCoordinationTest, CancelRejectsLateAcceptanceAndCompletion) {
  const auto pick = id(Operation::PickCell);
  prepare(pick, Resources::Right);
  ASSERT_TRUE(policy.acquire(pick));
  policy.request_cancel();
  EXPECT_TRUE(policy.cancel_requested());
  EXPECT_EQ(policy.work(pick)->phase, Phase::ConfirmingStop);
  EXPECT_EQ(*policy.arm(Arm::Right).owner, pick);
  EXPECT_FALSE(policy.command_accepted(pick));
  EXPECT_FALSE(policy.execution_complete(pick, Outcome::Success));
  EXPECT_FALSE(policy.release(pick));
  ASSERT_TRUE(policy.confirm_stop(pick, true));
  ASSERT_TRUE(policy.release(pick));
  EXPECT_EQ(policy.work(pick)->outcome, Outcome::Cancelled);
  EXPECT_FALSE(policy.acquire(pick));
  EXPECT_FALSE(policy.retry_round(2));
}

TEST_F(ArmCoordinationTest, CancelInvalidatesPreparedPermissionAndReadyJoin) {
  const auto left = id(Operation::ReceiveReady);
  const auto pick = id(Operation::PickCell);
  const auto handover = id(Operation::HandoverCell);
  execute(left, Resources::Left);
  execute(pick, Resources::Right);
  prepare(handover, Resources::Both);
  ASSERT_TRUE(policy.handover_ready(handover, left, pick));
  policy.request_cancel();
  EXPECT_FALSE(policy.preparation_complete(handover));
  EXPECT_FALSE(policy.start_handover(handover, left, pick));
  EXPECT_FALSE(policy.arm(Arm::Left).owner);
  EXPECT_FALSE(policy.arm(Arm::Right).owner);
}

TEST_F(ArmCoordinationTest, UnconfirmedStopRetainsOwnershipAndLatchesGlobalFault) {
  const auto pick = id(Operation::PickCell);
  const auto left = id(Operation::ReceiveReady);
  prepare(pick, Resources::Right);
  prepare(left, Resources::Left);
  ASSERT_TRUE(policy.acquire(pick));
  ASSERT_TRUE(policy.execution_complete(pick, Outcome::Success));
  ASSERT_TRUE(policy.confirm_stop(pick, false));
  EXPECT_TRUE(policy.fault_locked());
  EXPECT_EQ(policy.work(pick)->outcome, Outcome::Success);
  EXPECT_FALSE(policy.release(pick));
  EXPECT_FALSE(policy.arm(Arm::Right).stop_confirmed);
  EXPECT_EQ(*policy.arm(Arm::Right).owner, pick);
  EXPECT_FALSE(policy.acquire(left));
  EXPECT_FALSE(policy.execution_complete(pick, Outcome::Success));
  ASSERT_TRUE(policy.confirm_stop(pick, true));
  ASSERT_TRUE(policy.release(pick));
  EXPECT_TRUE(policy.fault_locked());
  EXPECT_FALSE(policy.retry_round(2));
}

TEST_F(ArmCoordinationTest, FaultInvalidatesOtherwiseReadyJoin) {
  const auto left = id(Operation::ReceiveReady);
  const auto pick = id(Operation::PickCell);
  const auto handover = id(Operation::HandoverCell);
  execute(left, Resources::Left);
  execute(pick, Resources::Right);
  prepare(handover, Resources::Both);
  ASSERT_TRUE(policy.handover_ready(handover, left, pick));
  policy.latch_fault();
  EXPECT_FALSE(policy.start_handover(handover, left, pick));
  EXPECT_FALSE(policy.cup_section_ready(left, pick));
}

TEST_F(ArmCoordinationTest, BasketEmptyWaitsForLeftWorkAndNeverEnablesHandover) {
  const auto previous = first_handover();
  const WorkId place{previous.round, Operation::PlaceCell};
  const auto empty = id(Operation::PickCell);
  const auto handover = id(Operation::HandoverCell);
  execute(empty, Resources::Right, Outcome::BasketEmpty);
  prepare(handover, Resources::Both);
  prepare(place, Resources::Left);
  ASSERT_TRUE(policy.acquire(place));
  EXPECT_FALSE(policy.cup_section_ready(place, empty));
  ASSERT_TRUE(policy.execution_complete(place, Outcome::Success));
  EXPECT_FALSE(policy.cup_section_ready(place, empty));
  ASSERT_TRUE(policy.confirm_stop(place, true));
  ASSERT_TRUE(policy.release(place));
  EXPECT_FALSE(policy.start_handover(handover, place, empty));
  EXPECT_TRUE(policy.cup_section_ready(place, empty));
  policy.request_cancel();
  EXPECT_FALSE(policy.cup_section_ready(place, empty));
}

TEST_F(ArmCoordinationTest, SuccessfulPickDoesNotEnableCupSection) {
  const auto left = id(Operation::ReceiveReady);
  const auto pick = id(Operation::PickCell);
  execute(left, Resources::Left);
  execute(pick, Resources::Right);
  EXPECT_FALSE(policy.cup_section_ready(left, pick));
}

TEST_F(ArmCoordinationTest, FirstEmptyPickRequiresMatchingLeftReadiness) {
  const auto left = id(Operation::ReceiveReady);
  const auto empty = id(Operation::PickCell);
  execute(empty, Resources::Right, Outcome::BasketEmpty);
  EXPECT_FALSE(policy.cup_section_ready(left, empty));
  execute(left, Resources::Left);
  EXPECT_TRUE(policy.cup_section_ready(left, empty));
  auto stale = empty;
  ++stale.round.attempt;
  EXPECT_FALSE(policy.cup_section_ready(left, stale));
  policy.latch_fault();
  EXPECT_FALSE(policy.cup_section_ready(left, empty));
}

TEST(ArmCoordination, RoundLimitBlocksNextPickAndHandoverWithoutImplyingEmptyBasket) {
  ArmCoordination policy(42, 1);
  ASSERT_TRUE(policy.record_initial_stop(42, Arm::Left));
  ASSERT_TRUE(policy.record_initial_stop(42, Arm::Right));
  const RoundId round{42, 1, 1};
  const WorkId left{round, Operation::ReceiveReady};
  const WorkId pick{round, Operation::PickCell};
  const WorkId handover{round, Operation::HandoverCell};
  ASSERT_TRUE(policy.prepare(left, Resources::Left));
  ASSERT_TRUE(policy.prepare(pick, Resources::Right));
  ASSERT_TRUE(policy.prepare(handover, Resources::Both));
  for (const auto & task : {left, pick}) {
    ASSERT_TRUE(policy.preparation_complete(task));
    ASSERT_TRUE(policy.acquire(task));
    ASSERT_TRUE(policy.execution_complete(task, Outcome::Success));
    ASSERT_TRUE(policy.confirm_stop(task, true));
    ASSERT_TRUE(policy.release(task));
  }
  ASSERT_TRUE(policy.preparation_complete(handover));
  ASSERT_TRUE(policy.start_handover(handover, left, pick));
  EXPECT_TRUE(policy.round_limit_reached());
  const WorkId next_pick{policy.current_round(), Operation::PickCell};
  const WorkId next_handover{policy.current_round(), Operation::HandoverCell};
  EXPECT_FALSE(policy.prepare(next_pick, Resources::Right));
  EXPECT_FALSE(policy.prepare(next_handover, Resources::Both));
  EXPECT_FALSE(policy.acquire(next_pick));
  EXPECT_FALSE(policy.start_handover(next_handover, left, pick));
  EXPECT_FALSE(policy.cup_section_ready(left, next_pick));
  EXPECT_FALSE(policy.retry_round(2));
  ASSERT_TRUE(policy.execution_complete(handover, Outcome::Success));
  ASSERT_TRUE(policy.confirm_stop(handover, true));
  ASSERT_TRUE(policy.release(handover));
  const WorkId place{round, Operation::PlaceCell};
  ASSERT_TRUE(policy.prepare(place, Resources::Left));
  ASSERT_TRUE(policy.preparation_complete(place));
  ASSERT_TRUE(policy.acquire(place));
  ASSERT_TRUE(policy.execution_complete(place, Outcome::Success));
  ASSERT_TRUE(policy.confirm_stop(place, true));
  ASSERT_TRUE(policy.release(place));
  EXPECT_FALSE(policy.cup_section_ready(place, next_pick));
}

TEST_F(ArmCoordinationTest, PreExecutionResultsPreserveAnotherOwnersStopEvidence) {
  for (const bool waiting : {false, true}) {
    for (const auto outcome : {Outcome::Failed, Outcome::BasketEmpty}) {
      ArmCoordination candidate(42, 3);
      ASSERT_TRUE(candidate.record_initial_stop(42, Arm::Right));
      const WorkId owner{{42, 1, 1}, Operation::ArmTask};
      const WorkId pick{{42, 1, 1}, Operation::PickCell};
      ASSERT_TRUE(candidate.prepare(owner, Resources::Right));
      ASSERT_TRUE(candidate.preparation_complete(owner));
      ASSERT_TRUE(candidate.acquire(owner));
      ASSERT_TRUE(candidate.prepare(pick, Resources::Right));
      if (waiting) {
        ASSERT_TRUE(candidate.preparation_complete(pick));
      }
      EXPECT_FALSE(candidate.finish_before_execution(pick, Outcome::Success));
      ASSERT_TRUE(candidate.finish_before_execution(pick, outcome));
      EXPECT_EQ(candidate.work(pick)->phase, Phase::Finished);
      EXPECT_EQ(candidate.work(pick)->outcome, outcome);
      EXPECT_EQ(candidate.work(pick)->failure_stage, outcome == Outcome::Failed ?
          FailureStage::Preparation : FailureStage::None);
      EXPECT_FALSE(candidate.work(pick)->execution_authorized);
      EXPECT_FALSE(candidate.work(pick)->stop_confirmed);
      EXPECT_FALSE(candidate.arm(Arm::Right).stop_confirmed);
      EXPECT_FALSE(candidate.arm(Arm::Left).stop_confirmed);
      EXPECT_EQ(*candidate.arm(Arm::Right).owner, owner);
      EXPECT_FALSE(candidate.release(pick));
      EXPECT_FALSE(candidate.confirm_stop(pick, true));
      EXPECT_FALSE(candidate.acquire(pick));
      EXPECT_FALSE(candidate.fault_locked());
    }
  }
}

TEST_F(ArmCoordinationTest, PreExecutionEmptyPickWaitsForLeftWorkWithoutAcquiringRightArm) {
  const auto previous = first_handover();
  const WorkId place{previous.round, Operation::PlaceCell};
  const auto pick = id(Operation::PickCell);
  const auto handover = id(Operation::HandoverCell);
  prepare(place, Resources::Left);
  ASSERT_TRUE(policy.acquire(place));
  ASSERT_TRUE(policy.prepare(pick, Resources::Right));
  ASSERT_TRUE(policy.finish_before_execution(pick, Outcome::BasketEmpty));
  EXPECT_FALSE(policy.arm(Arm::Right).owner);
  EXPECT_TRUE(policy.arm(Arm::Right).stop_confirmed);
  EXPECT_FALSE(policy.cup_section_ready(place, pick));
  prepare(handover, Resources::Both);
  finish(place);
  EXPECT_TRUE(policy.cup_section_ready(place, pick));
  EXPECT_FALSE(policy.start_handover(handover, place, pick));
}

TEST(ArmCoordination, PreExecutionEmptyResultCannotCreateInitialStopEvidence) {
  ArmCoordination policy(42, 3);
  const WorkId pick{{42, 1, 1}, Operation::PickCell};
  ASSERT_TRUE(policy.prepare(pick, Resources::Right));
  ASSERT_TRUE(policy.finish_before_execution(pick, Outcome::BasketEmpty));
  EXPECT_FALSE(policy.arm(Arm::Right).owner);
  EXPECT_FALSE(policy.arm(Arm::Right).stop_confirmed);
  EXPECT_FALSE(policy.work(pick)->stop_confirmed);
}

TEST_F(ArmCoordinationTest, ExecutionFailureStopsPeerAndKeepsFaultAfterCleanup) {
  for (const bool accepted : {false, true}) {
    policy = ArmCoordination(42, 3);
    SetUp();
    const auto left = id(Operation::ReceiveReady);
    const auto pick = id(Operation::PickCell);
    const auto followup = id(Operation::ArmTask);
    prepare(left, Resources::Left);
    prepare(pick, Resources::Right);
    prepare(followup, Resources::Both);
    ASSERT_TRUE(policy.acquire(left));
    ASSERT_TRUE(policy.acquire(pick));
    if (accepted) {
      ASSERT_TRUE(policy.command_accepted(pick));
    }
    EXPECT_FALSE(policy.finish_before_execution(pick, Outcome::Failed));
    ASSERT_TRUE(policy.execution_complete(pick, Outcome::Failed));
    EXPECT_TRUE(policy.fault_locked());
    EXPECT_EQ(policy.work(pick)->outcome, Outcome::Failed);
    EXPECT_EQ(policy.work(pick)->failure_stage, FailureStage::Execution);
    EXPECT_EQ(policy.work(left)->phase, Phase::ConfirmingStop);
    EXPECT_TRUE(policy.work(left)->stop_requested);
    EXPECT_TRUE(policy.work(pick)->stop_requested);
    EXPECT_EQ(*policy.arm(Arm::Left).owner, left);
    EXPECT_EQ(*policy.arm(Arm::Right).owner, pick);
    EXPECT_FALSE(policy.acquire(followup));
    EXPECT_FALSE(policy.release(left));
    EXPECT_FALSE(policy.release(pick));
    EXPECT_FALSE(policy.command_accepted(left));
    EXPECT_FALSE(policy.execution_complete(left, Outcome::Success));
    ASSERT_TRUE(policy.confirm_stop(pick, true));
    ASSERT_TRUE(policy.release(pick));
    EXPECT_FALSE(policy.retry_round(2));
    ASSERT_TRUE(policy.confirm_stop(left, true));
    ASSERT_TRUE(policy.release(left));
    EXPECT_TRUE(policy.fault_locked());
    EXPECT_FALSE(policy.acquire(followup));
    EXPECT_FALSE(policy.retry_round(2));
    EXPECT_FALSE(policy.prepare(followup, Resources::Both));
  }
}

TEST_F(ArmCoordinationTest, ArmTaskRejectsLateRegistrationAcrossRoundRetry) {
  const auto old = id(Operation::ArmTask);
  ASSERT_TRUE(policy.retry_round(2));
  EXPECT_FALSE(policy.prepare(old, Resources::Right));
  EXPECT_FALSE(policy.acquire(old));
  const auto current = id(Operation::ArmTask);
  prepare(current, Resources::Right);
  EXPECT_TRUE(policy.acquire(current));
}

TEST_F(ArmCoordinationTest, ArmTaskCannotAcquireAfterItsRoundBecomesStale) {
  const auto stale = id(Operation::ArmTask);
  prepare(stale, Resources::Right);
  const auto previous = first_handover();
  EXPECT_FALSE(policy.acquire(stale));
  auto late = stale;
  late.round.attempt = 2;
  EXPECT_FALSE(policy.prepare(late, Resources::Right));
  const WorkId place{previous.round, Operation::PlaceCell};
  prepare(place, Resources::Left);
  EXPECT_TRUE(policy.acquire(place));
}

TEST_F(ArmCoordinationTest, PlacementPreparationRetryPreservesHandoverAndRunningNextPick) {
  for (const bool waiting : {false, true}) {
    policy = ArmCoordination(42, 3);
    SetUp();
    const auto previous = first_handover();
    const WorkId failed{previous.round, Operation::PlaceCell};
    const auto pick = id(Operation::PickCell);
    const auto handover = id(Operation::HandoverCell);
    const auto next_round = policy.current_round();
    prepare(pick, Resources::Right);
    ASSERT_TRUE(policy.acquire(pick));
    ASSERT_TRUE(policy.prepare(failed, Resources::Left));
    if (waiting) {
      ASSERT_TRUE(policy.preparation_complete(failed));
    }
    ASSERT_TRUE(policy.finish_before_execution(failed, Outcome::Failed));
    EXPECT_FALSE(policy.retry_placement(failed, 1));
    const WorkId retry{previous.round, Operation::PlaceCell, 2};
    EXPECT_FALSE(policy.prepare(retry, Resources::Left));
    ASSERT_TRUE(policy.retry_placement(failed, 2));
    ASSERT_TRUE(policy.current_placement());
    EXPECT_EQ(*policy.current_placement(), retry);
    EXPECT_EQ(policy.current_round(), next_round);
    EXPECT_EQ(*policy.arm(Arm::Right).owner, pick);
    EXPECT_EQ(policy.work(previous)->outcome, Outcome::Success);
    EXPECT_FALSE(policy.acquire(previous));
    prepare(handover, Resources::Both);
    prepare(retry, Resources::Left);
    ASSERT_TRUE(policy.acquire(retry));
    EXPECT_FALSE(policy.preparation_complete(failed));
    EXPECT_FALSE(policy.finish_before_execution(failed, Outcome::BasketEmpty));
    EXPECT_FALSE(policy.execution_complete(failed, Outcome::Success));
    EXPECT_FALSE(policy.confirm_stop(failed, true));
    EXPECT_FALSE(policy.release(failed));
    EXPECT_FALSE(policy.retry_placement(failed, 3));
    EXPECT_EQ(*policy.arm(Arm::Left).owner, retry);
    EXPECT_FALSE(policy.work(retry)->stop_confirmed);
    finish(pick);
    EXPECT_FALSE(policy.start_handover(handover, failed, pick));
    EXPECT_FALSE(policy.start_handover(handover, retry, pick));
    finish(retry);
    EXPECT_FALSE(policy.start_handover(handover, failed, pick));
    EXPECT_TRUE(policy.start_handover(handover, retry, pick));
    EXPECT_EQ(policy.work(failed)->outcome, Outcome::Failed);
  }
}

TEST_F(ArmCoordinationTest, PlacementRetryRequiresStoppedFreeLeftArm) {
  const auto previous = first_handover();
  const WorkId place{previous.round, Operation::PlaceCell};
  const auto owner = id(Operation::ArmTask);
  prepare(owner, Resources::Left);
  ASSERT_TRUE(policy.acquire(owner));
  ASSERT_TRUE(policy.prepare(place, Resources::Left));
  ASSERT_TRUE(policy.finish_before_execution(place, Outcome::Failed));
  EXPECT_FALSE(policy.retry_placement(place, 2));
  ASSERT_TRUE(policy.execution_complete(owner, Outcome::Success));
  EXPECT_FALSE(policy.retry_placement(place, 2));
  ASSERT_TRUE(policy.confirm_stop(owner, true));
  EXPECT_FALSE(policy.retry_placement(place, 2));
  ASSERT_TRUE(policy.release(owner));
  EXPECT_TRUE(policy.retry_placement(place, 2));
}

TEST_F(ArmCoordinationTest, PlacementExecutionFailureCannotUsePreparationRetry) {
  const auto previous = first_handover();
  const WorkId place{previous.round, Operation::PlaceCell};
  prepare(place, Resources::Left);
  ASSERT_TRUE(policy.acquire(place));
  ASSERT_TRUE(policy.execution_complete(place, Outcome::Failed));
  ASSERT_TRUE(policy.confirm_stop(place, true));
  ASSERT_TRUE(policy.release(place));
  EXPECT_FALSE(policy.retry_placement(place, 2));
  EXPECT_TRUE(policy.fault_locked());
}

} // namespace
} // namespace demo_1f::coordination

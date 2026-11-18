# Dual arm execution: phases 2-A and 2-B1

`core/arm_coordination.hpp` remains a standalone C++17 policy with no runtime ownership or execution-permission connection. Phase 2-B1 changes pick callback attribution in TaskManager. Sequential execution and hold/release dispatch conditions remain; the release cleanup correction also requires release processing confirmation before another pick or run is admitted.

## State and ownership

- Each request has a non-reusable ID: unique run, one-based cell round, round attempt, operation, and operation attempt. ArmTask also requires the current round and round attempt. Only PlaceCell can increase its operation attempt; other operations use 1. Registration and acquisition both validate identity.
- Phases are Preparing, WaitingPermission, Sending, Executing, ConfirmingStop, and Finished. Preparation grants no ownership; explicit acquisition enters Sending before dispatch.
- Each arm's joints and gripper share one owner. Both-arm acquisition is atomic. Sending and ConfirmingStop retain ownership; only the matching ID releases after confirmed stop. No owner does not imply stopped: initial stop evidence is required.
- Outcome, stop evidence, cancellation, and fault lock are separate. Cancellation invalidates permission; late responses cannot revive it. Unconfirmed stop retains ownership and latches the fault. Later stop evidence permits cleanup without unlocking.
- Callers serialize access and aggregate both-arm stop evidence. No spatial reservation or runtime readiness relaxation is provided.

## Results before and after execution permission

- `finish_before_execution` accepts Failed or pick-only BasketEmpty during Preparing or WaitingPermission. It ends the work without creating ownership or changing any arm's stop evidence, including evidence belonging to another owner. The work's `stop_confirmed` remains false because no command was authorized.
- Failed before permission records `FailureStage::Preparation`. It does not interrupt unrelated work or latch a motion fault. The failed ID stays terminal; retry requires an explicit retry transition and the applicable stopped/free arm checks.
- Failure in Sending or Executing records `FailureStage::Execution`, retains the Failed outcome, and latches the global fault. Sending is conservative because dispatch may already have occurred. Every active owner retains its resources, enters ConfirmingStop, and exposes `stop_requested`; pending work is retired. New grants and joins are blocked immediately.
- Confirmed stop clears that work's stop request, and release permits cleanup only. Neither release nor retry clears a fault. Recovery requires independently verified safe state, explicit operator recovery, and a new policy with a fresh run ID; this model provides no in-place unlock. Backend Cancelled after permission also blocks the run through cancellation.

## Cell handover join

- First round: successful left ReceiveReady plus successful right PickCell for the same current run, round, and attempt.
- Later rounds: successful previous HandoverCell and its current successful PlaceCell attempt, plus successful current PickCell. Placement retains the admitted handover's round and round attempt. Completion order is unrestricted.
- Prerequisites require confirmed stop and released ownership. Cancellation, fault, unknown stop, or missing evidence blocks `start_handover`, which atomically acquires both arms. Ordinary acquisition cannot bypass this join.
- Admission selects the next round at attempt 1; placement retains the previous round's ID. `retry_round` requires stopped, free arms and retires unfinished preparation under the old attempt.
- `retry_placement(failed_id, attempt)` handles only terminal preparation failure of the current placement. It requires successful handover and a stopped, free left arm, and increases only the placement operation attempt. The next pick may remain active under its unchanged ID. Old placement callbacks cannot mutate or release the new attempt; failed placement never satisfies the join. Execution failure requires fault recovery instead.
- BasketEmpty never enables handover. Cup entry requires the current empty pick, matching completed left prerequisite, and both arms stopped and free. A pre-execution empty result requires no fictitious work stop evidence; previously established arm evidence still must hold. Adapters retain basket freshness and eligibility checks. No result claims sensor-confirmed grasp or transfer.
- The limit blocks the next pick and handover, while allowing current placement to finish. It neither implies empty basket nor permits cup entry.

## Connection in phases 2-B and 2-C

Current `TaskManager` stages in `src/tasks/demo_actions.cpp` execute sequentially, with an optional prepared pick using hold/release. Future independent execution would overlap the next right pick with left placement and join their completion.

2-B1 implements request identity and callback context only, as detailed below. Remaining 2-B work must map those IDs to policy round/operation attempts, resolve preparation versus execution permission, and translate stop evidence without changing the Python path. Route results before permission to `finish_before_execution` and translate `stop_requested` into the existing stop procedure. Map INITIALIZE_ROBOT, HANDOVER, APPROACH_CUP, and REFINE_CUP completion in `start_demo_stage`/`tick_demo` to policy evidence; define the complete left placement boundary and carry operation attempts through placement callbacks.

2-C connects ownership and join admission to standalone pick, handover, approach, and refine handlers and fault reporting. Resolve shared run identity, retry/resume identity, aggregated stop evidence, and operator recovery. Fault recovery requires independent safe-state verification and a new policy with a fresh run ID; no unlock API exists.

Path separation validation and actual concurrent execution are phase 3 work. TrajectoryExecutor, pair_paths, interfaces, speeds, timing, and motion order remain outside 2-A.

## Head motion alongside arm motion

Requested 2026-11-17: the head turns while the arms move; no arm operation waits for it. The user states that the head camera is not used (cup source is `wrist_left_init`, the cell pick uses the wrist) and that the head pose does not affect any arm path.

Decision: the head is not an arm resource and is outside the ownership policy. It gets its own command channel, a FollowJointTrajectory goal sent straight to `/head_controller`, which bypasses MoveGroup, ExecuteTrajectory and the single TrajectoryExecutor operation. Consequences:

- Each task commands its head pose when it starts and never waits for or checks the result. The LOOK stages and the standalone head wait are removed.
- A head command failure is logged and does not stop, fault or delay the arm operation.
- Head moves are not planned or collision-checked; the targets are the fixed look_* poses.
- On release, left arm changes are not checked (user decision 2026-11-18); only right arm movement and a hold over 10 s cause re-observation.
- The initial pose sequence commands the head itself through MoveIt, so no separate head command is sent for it.

This step is implemented before 2-B2 so the stage machine that 2-B2 connects to has no head stages.

## Scope agreed on 2026-11-17

The target flow is the user's task table: the arms act together only in the three handovers. Elsewhere the right arm either picks while the left stays, or returns to the initial pose while the left works.

- Preparing the next pick during cell placement stays computation only; the right arm does not move for the pick until the left arm is done. Overlapping the pick motion with placement is out of scope, so the 2-B2/2-C wiring of the ownership policy is on hold. 2-A remains unused and 2-B1 stays in place.
- Remaining work: the right arm's return to the initial pose runs as its own execution, at its own pace, beside the left arm's cup approach (menu 4) and beside the left arm's cup placement (menu 0). The two arms must not be time-matched to arrive together, which replaces the current paired approach.
- Once the right arm is back at the initial pose it may start its next computation (next pick, basket pose) without waiting for the left arm.

## Phase 2-B1 request context

- Every accepted demo start, pick-only start, initial start, and resume allocates a fresh run ID. Every sent pick, including retries, allocates a fresh request ID. Both use a process-local monotonic sequence. Each context holds a const `(run, request)` identity; no persistence across process restart is needed.
- The context owns pending acceptance, handle, feedback, terminal result/code, cancellation flags, hold/takeover/release flags, release future/response/error/timeout latch, and dispatch/stage/cancellation/release timestamps. Stage ownership means PICK_DYNAMIXEL takeover only, not ArmCoordination command ownership.
- Goal response, feedback, and result lambdas capture their created context. Feedback changes the demo detail only for the current run's current request. A terminal result is recorded once; duplicates cannot overwrite it. A result from a retired request still provides that request's cleanup and stop evidence. Unknown stop latches the existing demo fault.
- Cancellation marks the originating context even before acceptance. Late acceptance cancels that exact handle. All release futures, current and retired, are polled every manager tick even after the action result or demo termination; a late refusal cancels only its context, and a late success cannot complete a new pick.
- Action settlement and release settlement are independent. New starts, resumes, pick-only starts, and pick dispatches remain blocked while any retained request has unknown acceptance, an active handle, unknown dispatch outcome, no confirmed final stop, or a sent release without a response confirming processing. Both positive and negative non-null responses confirm that the service processed the request; they retain their distinct success/failure semantics. Cancellation and cleanup remain available while admission is blocked.
- The existing stage timeout, 10 s action cancellation/end waits, 5 s release threshold, STOP_UNCONFIRMED handling, and faulted shutdown rule remain. Action finalization checks action stop evidence separately, so a pending release alone does not misreport an unconfirmed action stop. The next PICK_DYNAMIXEL dispatch waits for release confirmation instead of creating a new request or resetting the release clock.
- At 5 s without a release response, the context latches `release_timed_out`, requests cancellation of its original pick, and latches the demo fault. The service may still process release remotely: its future, unresolved state, and context are preserved. A late response supplies cleanup evidence but neither clears the timeout/error nor unlocks admission. Late action stop evidence likewise never clears a fault.
- REFINE_CUP sends the existing held request. PICK_DYNAMIXEL takes over the same context and ID, preserving an early terminal result or waiting for PICK_HOLD before release under the existing conditions. The pick timeout still starts at stage entry/takeover. No duplicate dispatch or concurrent movement is enabled.
- Current context is retained through processing; replacement occurs only after both action and release settlement checks. Retired contexts leave the manager's list only after a confirmed final stop and, if release was sent, a confirmed processing response and consumed future. Timeout alone never permits retirement. Outstanding action callbacks retain their own shared context. Unknown requests remain retained and block admission.

## Validation status

Head routing applies to both the full demo and standalone executed actions. PickDynamixel and PourCup use look_right; ApproachCup, RefineCup, PickCup and PlaceCup use look_left; initialization, handovers, withdrawal and return use look_middle. Preview requests do not move the head. Fresh stationary feedback at the configured target skips a redundant head move. Otherwise the head must finish successfully before the requested arm operation starts, and cancellation during head preparation never starts the arm operation. Head stages retain the last completed task checkpoint, including pick-only resume, and their cell-round durations are logged separately.

On release, prepared pick paths do not check left arm changes (user decision 2026-11-18); only right arm movement and a hold over 10 s cause re-observation. The prepare_pick_during_refine setting remains unchanged. Current source contains 56 demo tests and 6 hold tests. On 2026-11-17 the user built this change in the container and reported `test_demo_task` 56 passed (67.65 s), `test_cup` 110, `test_handover` 118 and `test_hold.py` 14 passed, after correcting one test's withdraw arm, widening two wait loops and raising the `test_demo_task` ctest timeout from 60 to 120 s. The user also confirmed the head directions on the robot; timing with the head stages has not been measured.

The user reports a successful container build and all 25 `test_arm_coordination` cases passing for the revised 2-A policy before 2-B1. This is supplied validation evidence, not a run performed during this change. The three warned single-line `if` statements now have braces; their expectations are unchanged.

The initial eight `test_demo_task` regressions cover retired feedback/results, cancellation before acceptance with exact goal UUID checking, unknown terminal stop, retry/resume IDs, prepared-context takeover, late stop after the original 10 s cancellation wait, and delayed release success/refusal. The fake backend can delay acceptance through a separately pumped executor and defer service responses using [rclcpp deferred service callbacks](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/any_service_callback.hpp). A private friend in the test file replays duplicate transport responses; no public API grants permission or bypasses stage order.

Release cleanup adds five regressions, for 50 `test_demo_task` cases total: continued monitoring of the current context after action completion, cancellation with release processing itself still deferred, timeout after action completion with a late positive response, timeout before remote processing with a late negative response, and a valid resumable checkpoint blocked until release confirmation. The fake service can now defer the release operation itself, not merely delivery of its response. The timeout cases use the real 5 s threshold without changing timestamps or execution order.

The two existing delayed-response tests now reject new requests while release processing is unconfirmed, including after confirmed action cancellation. After a response arrives within the deadline, a fresh pick must start normally. Duplicated release, feedback, and result callbacks must still leave the new context unchanged. This replaces the former expectation that a confirmed action stop alone allowed another pick, because the unscoped remote release could still be pending and affect a later goal.

Build, tests, container copy, and physical execution were not performed for 2-B1. Only source, diff, and whitespace checks were performed. Runtime permission/ownership wiring and real concurrency remain unvalidated and unimplemented.

The 2-A policy test expectations remain unchanged. Its atomic acquisition test uses distinct operations in the same current round, replacing its formerly accepted future-attempt ArmTask identity so that it exercises contention under the stricter identity rules.

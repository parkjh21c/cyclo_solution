# DYNAMIXEL pick → handover → cup demo

TaskManager owns the full state machine. There is no separate DemoSequence runner.
The existing per-motion actions remain available for debugging.

## Software layout

- `src/tasks/demo_actions.cpp`: `/run_demo`, stage progression, bounded retries, cancellation and checkpoints.
- `include/demo_1f/tasks/demo_recovery.hpp`: retry policy.
- `modules/dynamixel_pick/ffw_vision`: upstream YOLO11 segmentation + aligned depth measurement.
- `modules/dynamixel_pick/ffw_grasp`: upstream multi-frame fusion, basket geometry and grasp candidates.
- `modules/dynamixel_pick/ffw_pick`: upstream analytic IK/path search and execution; `action_server.py` exposes a cancellable backend.
- `modules/dynamixel_pick/ffw_vision_msgs`: upstream perception interfaces plus internal `PickDynamixel` action.

Python is retained for the existing pick engine. TaskManager talks to its internal
`/dynamixel_pick/execute` action and waits for its terminal result. Pick uses its existing
MoveIt execution client; handover/cup motions use the C++ TrajectoryExecutor. TaskManager
serializes these two execution paths. Do not invoke the internal backend or old `pick_moveit`
CLI concurrently with demo motions. Both use the SAME move_group and robot controllers.

The original checkout under `ai_worker/` is not changed or used to build the imported modules;
`modules/dynamixel_pick/UPSTREAM.md` records their source commit. `ai_worker/COLCON_IGNORE`
prevents a duplicate bringup build. Camera drivers and robot bringup remain external.

## 중앙 상태 제어와 복구 구조의 발전 방향

2026-11-13 멘토 논의에서 나온 중앙 관리·모듈화·트리·리커버리 BT를 다음 방향으로 정리한다.
BT(Behavior Tree)는 도입 후보이며, 아래 내용은 현재 구현과 구분한 설계 방향이다.

### 현재 기반

지금도 `TaskManager`가 전체 데모의 순서, 실행 소유권, 제한된 재시도·취소·재개를 관리한다.
개별 메뉴는 모션을 따로 검증하는 수단이며, 전체 데모는 `src/tasks/demo_actions.cpp`의
상태 전이로 진행한다. 모션은 `start()` → `tick()` → `result()`와 취소 인터페이스를 제공한다.
따라서 중앙 관리를 처음부터 새로 만드는 것이 아니라, 기존 중앙 실행기에 상태 판단과
복구 정책을 보강하는 방향이다. 현재 중앙 실행기는 BT로 구현되어 있지 않다.

### 역할과 상태 보고

| 계층 | 책임 |
|---|---|
| 중앙 작업 제어 | 시작 조건 확인, 모듈 선택, 완료 판단, 재시도·대체·복구·중단 결정 |
| 동작 모듈 | 셀 집기·핸드오버·컵 접근·붓기 수행, 진행·결과·실패 후 상태 보고 |
| 궤적 실행기와 컨트롤러 | 경로 계획·충돌 검사·실행·취소·정지 확인 |
| 상태 공유 | 관절·TF·인식 등 관측값과 시각 제공; 명령 실행과 분리 |

양팔 짝 이동처럼 한 작업 안에서 동기화해야 하는 궤적은 모션과 실행기가 담당한다.
중앙은 관절 궤적을 직접 계산하기보다 작업의 시작·완료 조건과 다음 행동을 관리한다.
개별 모듈을 각각 별도 프로세스로 나누는 것은 이 구조의 필수 조건이 아니다.

각 모듈의 계약에는 다음을 명시하고, 중앙에서 필요한 항목을 공통 결과·상태로 제공한다.

- 시작 조건: 팔 자세, 관측값의 유효성, 물체·장면 상태, 다른 실행의 종료 여부.
- 완료 조건: 요청 수락이나 컨트롤러 종료와 구분한 작업 목표의 달성 기준.
- 진행과 실패: 마지막 완료 단계, 실패 단계, 실행 전 실패인지 움직인 뒤 실패인지.
- 실패 후 상태: 정지 확인 여부, 물체를 든 손과 컵 기울기 등 복구에 필요한 상태.
- 복구 범위: 재시도·대체가 가능한 조건과 사용자 개입이 필요한 조건.

물리적 상태는 센서로 확인한 사실, 명령 결과로 추정한 상태, 알 수 없는 상태를 구분한다.
그리퍼 닫힘이나 MoveIt 컵 모델 부착만으로 실제 파지 성공을 확정하지 않는다.
현재 `RunDemo.action`의 `holding_state`는 항상 `UNVERIFIED`인 호환 필드이며 실행 판단에
사용하지 않는다. 향후 물체 소유 상태를 도입할 때 확인 근거와 관측 시각도 함께 정의한다.

### BT와 리커버리의 의미

BT는 작업 순서와 조건·대체 분기를 트리로 표현하는 방법이다. 기존 모션을 감싸 실행하고
진행 중·성공·실패를 중앙에 반환하는 노드로 연결할 수 있다. 노드 중단 시에는 모션 취소와
정지 확인까지 연결하고, 실행 소유권을 중복해서 만들지 않아야 한다.
리커버리는 같은 동작의 무조건 반복이 아니라 실패 종류와 현재 상태에 따른 행동 선택이다.

- 컵 접근의 짝 이동이 실행 전에 계획·검증에서 실패했다면, 조건을 확인하고 순차 이동으로 대체한다.
- 실행 중 실패하거나 정지가 확인되지 않았다면 다음 이동을 시작하지 않는다.
- 붓기 후 `pour_untilt`가 실패했다면 컵이 이미 기울어져 있을 수 있다. 붓기 전체 재시도와
  현재 자세에서의 복구를 구분한다. 왕복 경로 사전 검증은 동작 모듈의 개선 과제다.
- 관측 실패는 유효한 입력을 다시 기다릴 수 있지만, 완료한 파지·인계를 자동 반복하지 않는다.

현재 제한된 재시도 정책은 `include/demo_1f/tasks/demo_recovery.hpp`와 픽 백엔드 결과를
따른다. 위 방향을 기록했다고 새 복구 동작이나 자동 재시도가 활성화되는 것은 아니다.

### 진행 순서

1. 지금처럼 개별 동작을 실물에서 검증하면서 시작·완료 조건과 실패 후 상태를 정리한다.
2. 중앙이 사용하는 결과·상태 계약과 모듈 간 조건을 명확히 한다.
3. 실행 전 실패, 실행 중 실패, 취소, 정지 미확인에 대한 전이·복구 정책을 검증한다.
4. 분기가 늘어 BT가 유리해지면 기존 모션을 재사용해 중앙 상태 전이를 점진적으로 옮긴다.

BT 도입 자체보다 상태와 복구 조건의 명확화가 먼저다. 모션 검증과 모듈화를 계속 진행하는
현재 방향은 이 구조의 기반이 되며, 중앙 제어 방식이 바뀌어도 재사용한다.

## Build in an existing container

Copy/mount this folder at `/root/ros2_ws/src/demo_1f`, then:

```bash
cd /root/ros2_ws/src/demo_1f
./scripts/build_workspace.sh
source /root/ros2_ws/install/setup.bash
```

The explicit package paths in this script are required: colcon normally stops descending
when it finds the parent `demo_1f/package.xml`. This builds the four imported packages too.
No Docker image rebuild is necessary. The Dockerfile has also been updated for future builds.

The DYNAMIXEL detector uses PyTorch/Ultralytics, independently of the existing cup detector's
TensorRT/OpenCV backend. Check dependencies in the Python interpreter used for pick nodes:

```bash
/opt/demo-pick-venv/bin/python3 scripts/check_pick_dependencies.py
```

If torch/Ultralytics are absent on an x86 PC, create an isolated CPU inference environment:

```bash
./scripts/setup_pick_inference.sh
# Created /opt/demo-pick-venv; existing ROS/OpenCV installation is preserved.
```

On Jetson, use a Python environment with torch/torchvision compatible with that JetPack.
The setup script reuses a working installation and does not replace it with desktop wheels.
It stops if compatible torch/torchvision are missing on ARM. Use `pick_python:=` below to
select that interpreter. It must also have access to ROS Python packages and the built overlay.

The Dockerfile prepares `/opt/demo-pick-venv` during image build. The first build
needs internet access for Python packages, using host networking; ROS compilation
runs offline. The setup script reuses the base image's GPU torch/torchvision.
Container startup performs no package installation and does not mount over the
image's Python environment. Compose uses NVIDIA runtime, host networking/IPC and
`restart: unless-stopped`. The idle container returns after a host reboot unless
it was explicitly stopped; robot motion still requires an action request.

## Start

Bringup and head/right/left wrist camera topics must already be available. These launches
never open camera devices or start a second hardware bringup.

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /root/ros2_ws/install/setup.bash
ros2 launch demo_1f full_demo.launch.py use_sim_time:=false
```

The image's inference environment is selected by default. To use another one, append:

```text
pick_python:=/path/to/python3
```

If the cup viewer/perception is already running, append `start_cup_perception:=false`.
If move_group is already running, append `start_moveit:=false`. Do not keep a previous
handover_demo_node running at the same time. Cup viewer uses port 8765; the new DYNAMIXEL
viewer uses 8767. Original camera topics are in ffw_vision/config/cameras.yaml; the right
wrist needs RGB, camera_info and `aligned_depth_to_color/image_raw` containing metric depth.

Nothing moves on launch. In another terminal with the same environment:

```bash
ros2 run demo_1f action_runner.py
```

- `d`: full demo.
- `1`: initial pose; `2`: right-hand cell pick from the current pose.
- `3`: right-to-left handover; `4`: left cup approach.
- `5`: left wrist refinement and release; `6`: both-hand withdrawal.
- `r`: resume a recoverable checkpoint; after `2`, continue with handover.
- `c`: cancel the goal sent by this runner and wait for the final result.
- `p`: status; `q`: exit, waiting for cancellation if necessary.

There are no extra menus, direction selectors, previews, cup-pick entries or server launch controls.
Start `full_demo.launch.py` separately. Exiting the runner does not stop that server.

Equivalent direct action:

```bash
ros2 action send_goal /run_demo demo_1f/action/RunDemo \
  '{command: start, cup_id: cup}' --feedback
# {command: pick, cup_id: cup} or {command: resume, cup_id: cup}
```

## Sequence and recovery

1. INITIALIZE_ROBOT: open the grippers, then move both arms, lift (-0.1 m) and head
   ([0.69, 0] rad) together in one MoveIt plan to the basket observation pose.
   There is no folded-arm waypoint or intermediate lift height.
   Targets in `config/demo.yaml` follow the ai_worker custom initial positions;
   the original head target 0.76 rad is replaced with 0.69 rad within the SG2 URDF limit.
   All movements use MoveIt and wait for completion. Launching the server performs no motion.
2. PICK_DYNAMIXEL: acquire fresh
   images, choose a collision-free grasp, open, approach, close, lift and test gripper position.
3. HANDOVER: right_to_left sequence, once. After release, move each TCP outward
   5 cm from its own transfer position, preserving height and orientation.
   On completion, continue directly to cup approach using the saved head track.
4. APPROACH_CUP: snapshot the latest saved head BODY location → left TCP observation pose.
   RobotContext tracks the head cup continuously, including before handover. Three valid,
   transformed samples confirm each update (under 1 s apart and 2 cm position spread).
   Detection loss keeps the last confirmed location and its real timestamps for at most
   `approach.max_age` (30 s). No post-request head observation is required. Wrist RIM data
   remains separate. Input freshness/TF limits are unchanged. Each approach freezes its
   own goal and collision model while background tracking continues for future requests.
   This memory is reset with the node and assumes a stationary robot base.
5. REFINE_CUP: wrist rim observation, pitch search, correction/lowering and release.
   Pitch speed is 0.06. After at least three fresh unconfirmed frames and one second
   without stability progress, increase pitch by 5 degrees up to 75 degrees.
   Advancing stability keeps the current angle. The overall 15-second search budget
   includes observation and pitch planning/execution; an active pitch is cancelled
   and its stop confirmed on expiry. Missing camera/TF input never triggers a tilt.
6. WITHDRAW_CUP: run left then right sequentially (`arm: both`). For each hand,
   lift its measured TCP 10 cm in world Z and retreat 8 cm along +TCP Z
   (opposite hand-forward), preserving orientation. Each withdrawal uses collision-checked LIN;
   failure/cancellation stops the sequence. A downward retreat is rejected.
7. RETURN_INITIAL: reuse the initial-pose module after successful withdrawal; do not repeat pick.

Cell rounds (`start`): after REFINE_CUP the demo picks again, so steps 2–5 repeat without
re-initialization (APPROACH_CUP returns the right arm to the observation pose). When a pick
ends with `BASKET_EMPTY` before any motion and `/demo/basket_pose` is in the planning frame and
at most `demo.basket_empty_max_age` (5 s) old, the cup section runs once:
(WITHDRAW_CUP if enabled and at least one round finished) → PICK_CUP → HANDOVER_CUP → POUR_CUP
→ HANDOVER_CUP_BACK → PLACE_CUP → RETURN_INITIAL. Without that basket view the pick fails as
`BASKET_UNKNOWN`, retries, then stops resumable; the cup section does not start.
`demo.max_cell_rounds` (10) finished rounds stop with `CELL_ROUND_LIMIT` (resumable; resume
counts again from zero). Feedback detail starts with `Round N: ` during a round.
`pick` alone ends with `BASKET_EMPTY` on an empty basket.

Menu `5` / `/refine_cup` stops after release. Menu `6` / `/withdraw_cup` only withdraws
both hands sequentially, with no perception or gripper commands.
The action also accepts `arm: left` or `arm: right`; preview each hand separately. Manual sequence: `5` → `6` → `1`.
The full demo calls withdrawal and then returns to initial pose automatically.
The full cup-side tabletop collision model is not added by this withdrawal sequence.

`initial` runs initialization only; `start` initializes before the full demo.
`pick` starts directly from the current pose. Retrying pick or resuming a later
stage does not repeat initialization; the pick backend skips its old right-arm-only HOME move.

`config/demo.yaml` defaults to two additional attempts, one second apart. Pick perception /
pre-grasp planning failures retry only if no grasp execution started. Cup observation failures
retry their own stage, never restart pick or handover. The existing wrist module performs
its pitch-search recovery. Arbitrary controller/execution faults do not automatically replay
motions, especially gripper operations. Unknown stop/scene results latch a fault and block
further goals.

After retry exhaustion, a result with `can_resume: true` retains the failed stage. `r` repeats
that stage after the observation issue has been resolved. Resume checkpoints are in memory;
restarting the server clears them. A manual motion invalidates the saved checkpoint.
After an unrecoverable failure, inspect/recover the robot using the individual motions.
Once the object is removed and both grippers are ready for a new pick, explicitly clear
only the software record:

```bash
ros2 service call /handover_demo/reset_demo std_srvs/srv/Trigger '{}'
```

This service commands no motion, does not clear an unknown-stop fault and does not assert
that the grippers are empty. Restart after resolving a latched stop fault.

## Geometry and verification limits

The demo builds `urdf/sg2_extended_grippers.urdf` from the installed SG2 description.
Both hands use `gripper_l_tcp` / `gripper_r_tcp`, fixed 45 mm along the original
end-effector frame's local -Z. The handover launch publishes only these two added
static transforms; existing bringup/camera transforms are preserved. All demo
motions and both MoveIt/pick IK use this shared model. Distal finger collision
geometry includes an approximate layered taper, following the actual finger joints.
The original handover Cartesian target positions/orientations are preserved by
re-solving its joint targets for the extended TCPs. Timing/gripper commands stay the same.

Pick's `tip_below_end_effector` is now zero: the new TCP already includes the
45 mm extension. Its legacy attached finger boxes are removed, because the URDF
owns finger collisions for both hands throughout the demo. Pick reads the same
model file as MoveIt rather than the unextended bringup description topic.
Original camera/arm bias values remain robot-specific; override
`dynamixel_calibration:=/path/to/robot_calibration.yaml` if they differ.

Basket/table objects remain in MoveIt's scene; cup scene updates preserve these
obstacles rather than erase them. Printed-finger shape is approximate, not a measured CAD mesh.

A pick grip is inferred from fresh gripper position after closing and lifting. The existing
handover has no independent transfer sensor (`LEFT_ASSUMED`). Release success means the
motion and opening completed (`RELEASED_UNVERIFIED`); object landing is not verified.

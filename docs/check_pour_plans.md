# Plan-only pour feasibility check

`check_pour_plans` checks the current transfer-hold state, plans the approach, then independently plans 130°, 115°, and 100° from the same hypothetical pre-tilt joint state. It never invokes `/pour_cup`, constructs a `TrajectoryExecutor`, sends controller/gripper commands, or requests trajectory execution. Its only action client is the configured MoveGroup endpoint; every goal has `plan_only=true`, `replan=false`, and `look_around=false`. A timeout may cancel that planning goal.

## Inputs and reuse

- Read all parameters from the running `/handover_demo` (override with `config_node`). Use the existing `load_config`, `load_approach_config`, and `load_pour_config` functions. Missing live parameters and angles other than 130/115/100 are errors; the tool does not change remote parameters or YAML.
- Call the existing `plan_pour` for the approach and all three tilt poses. The request helper matches the ordinary pose/LIN branch in `src/runtime/execution/moveit_goal.cpp`: group, TCP, pipeline, planning time/attempts, position/orientation tolerances, and speed/acceleration. The helper is separate from the production executor, whose success callbacks can dispatch execution. Future changes to that production branch must be reflected in this helper.
- Read the live URDF/SRDF from the same parameter snapshot. Require fresh, finite feedback for every independent model joint; non-gripper joints must have a known stationary velocity. Preserve scene attachments, world objects, transforms, octomap, and allowed collision matrix. Require `demo_place_cup` geometry attached to the right TCP (override ID with `attached_cup_id`). No object is attached or moved in the live scene.
- Require the current TCP to be within the production transfer-hold tolerance. Always plan `pour_above` with PTP, followed by `pour_lower` with LIN when `plan_pour` requires it. Apply each returned final joint position to the complete local robot state, preserving other joints and attached bodies. Check trajectory structure using the existing round-trip validator, and reject a changed planning start.
- Freeze the resulting complete pre-tilt state. Each angle gets its own copy, so a previous tilt result cannot become the next angle's start. Each MoveGroup request carries the complete private scene and explicit non-diff robot state.

## Basket evidence

The tool plans from the retained `/demo/basket_pose` observation, as `/pour_cup` does; it must be a valid pose in the live planning frame. It prints the basket, `pour.shift`, the lip target, and whether the lip target was clamped inside the rim. The scene must already hold the `basket_*` collision objects for that observation; the tool does not move them. Confirm the observer's scene update succeeded before checking.

Read-only inspection in the container:

```bash
ros2 topic echo /demo/basket_pose --once --qos-durability transient_local
ros2 param get /handover_demo planning_frame
```

## Build and run

These commands are for the user; they were not executed during implementation. The container must already contain the current host baseline, including the earlier round-trip changes. The C++ tool requires a build. No node restart is required; use the existing idle demo and MoveIt services, with the cup already attached at the transfer hold.

Host:

```bash
cd /home/robotis/ai_worker_demo/cyclo_solution
docker exec docker-demo_1f-1 mkdir -p /root/ros2_ws/src/demo_1f/tools
docker cp CMakeLists.txt docker-demo_1f-1:/root/ros2_ws/src/demo_1f/CMakeLists.txt
docker cp tools/check_pour_plans.cpp docker-demo_1f-1:/root/ros2_ws/src/demo_1f/tools/check_pour_plans.cpp
bash docker/container.sh enter
```

Container:

```bash
cd /root/ros2_ws
source install/setup.bash
colcon build --base-paths src/demo_1f src/demo_1f/modules/dynamixel_pick/ffw_* \
  --packages-select demo_1f --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
set -o pipefail
ros2 run demo_1f check_pour_plans 2>&1 | tee /tmp/check_pour_plans.log
```

Use the same ROS domain, middleware, and service remappings as the existing demo. The tool reads its ROS clock mode from the live configuration. Do not start another demo or MoveIt node for this check.

## Interpretation and limits

Output includes the frozen initial and pre-tilt joint values, scene object IDs/positions, target poses, planning settings, and each angle's MoveIt code, symbolic name, action status, planning duration, source, and message. If MoveIt returns generic 99999 without a message, the output explicitly says the reason is unavailable; consult the corresponding move_group logs rather than interpreting 99999 as necessarily NO_IK_SOLUTION.

Exit 0 means all three tilt plans passed the structural checks; exit 1 means at least one planner failed; exit 2 means the check was incomplete (missing input, attachment, feedback, approach failure, rejected goal, timeout, or malformed result). Every uncompleted angle gets an INCOMPLETE line. Ordinary planning failures still allow the other angles to be checked. Missing prerequisites never cause planning directly from the transfer hold to the tilted target.

This is a current scene snapshot simulation, not an exact replay of the old failure. It does not reproduce tracking error or alternative IK branches produced by a physically executed approach. It does not prove return-path collision validation, controller execution, settling, cup retention, or successful pouring. The source's round-trip shape checks are stricter than a bare MoveIt success, including at-rest endpoints. The tool requires complete fresh single-variable joint feedback and will reject unsupported/incomplete states instead of filling them with defaults.

The planning request helper was reviewed against the current production source. No build, test execution, live planning request, container copy, node restart, or commit was performed during implementation. Existing passing tests reported before this tool was added do not validate the new tool.

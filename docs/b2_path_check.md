# B2 read-only return path check

This tool sends only GetPlanningScene, GetPositionFK, GetPositionIK and GetStateValidity service requests. It never applies a scene, changes parameters, sends an action goal, or moves the robot. No C++ build or full_demo restart is needed for this script change. The commands below are for the user and have not been executed during implementation.

## Copy and prerequisites

Host, from the repository root:

```bash
cd /home/robotis/ai_worker_demo/cyclo_solution
docker cp scripts/ptp_path_check.py docker-demo_1f-1:/root/ros2_ws/src/demo_1f/scripts/ptp_path_check.py
```

Only that script needs copying. Its existing `cup_layout_check.py`, `generate_cup_handover_targets.py`, ROS Python packages, NumPy, PyYAML and ffw_pick clearance/config/gripper modules must already be available in the container. The documentation file `docs/b2_path_check.md` is the other changed file and is not required at runtime.

Run the following inside the container **with full_demo already running**, using the same ROS domain and middleware. Keep the robot and world stationary throughout the check. The script checks the actual scene attachment: a unique `demo_place_cup` with geometry, attached to `gripper_l_tcp`, geometry expressed in that link, explicit touch_links, and no duplicate world cup. It requires basket and demo_table boxes in base_link. Missing or unsupported conditions cause failure; it never infers left-hand ownership just because a scene was returned. Other attachment frames are deliberately unsupported and cause failure rather than an invented transform.

Use the current post-menu-9 scene or an already retained equivalent receiving state; this document does not command a physical menu action. The left arm seed must match `cup_right_to_left_grasp/cup_insert` within 1 mm and 0.5 degrees. The script does not move the left arm to achieve that condition. If no such seed/attachment is available, stop and report the missing condition.

## Explicit seed capture

Inside the container, capture an explicit seed from an existing stationary state. This only writes `/tmp/b2_seed.yaml`; it sends no motion. The file records all scene joint positions and their source. Use a retained, correctly sourced joint mapping instead if the current left arm is not at the menu 9 receiver pose. Never substitute zero/default joint values to make a missing state pass.

```bash
cd /root/ros2_ws
source install/setup.bash
cd src/demo_1f/scripts
python3 - <<'PY'
from datetime import datetime, timezone
import rclpy
import yaml
from ptp_path_check import PathChecker

rclpy.init()
node = PathChecker()
try:
    scene = node.read_scene()
    js = scene.robot_state.joint_state
    seed = {
        'source': 'GetPlanningScene stationary seed captured at ' + datetime.now(timezone.utc).isoformat(),
        'joints': dict(zip(js.name, map(float, js.position))),
    }
    with open('/tmp/b2_seed.yaml', 'x') as handle:
        yaml.safe_dump(seed, handle, sort_keys=False)
    print('Saved /tmp/b2_seed.yaml; receiver pose and attachment still require tool validation')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

The capture uses exclusive creation to preserve an existing seed. Choose another filename in both capture and check commands when another capture is needed. The minimum seed schema is `source: <provenance>` and `joints: {joint_name: radians}`. All seven joints of each arm and `gripper_l_joint1`, `gripper_r_joint1` are mandatory; additional known joints are accepted. Other model joints retain the scene values and are listed. No timestamp freshness claim is made for the underlying scene joint feedback.

The seed is not claimed to be a measured release endpoint. The tool verifies the left receiver TCP by FK, reconstructs the right handover TCP using `cup_left_to_right_grasp/cup_insert` and IK seeded from this file, and sets the two gripper joints locally to the configured receiver_close/giver_release targets. It prints these overrides and the complete reconstructed state. The right-arm reconstruction assumes the preceding pour returned to that taught handover TCP; it does not replay the pour or prove the same IK branch as the runtime.

## Check both return choices

Inside the container, with full_demo still running and no robot/world motion:

```bash
python3 ptp_path_check.py --help
set -o pipefail
python3 ptp_path_check.py right_return giver_home_via \
  --yaml /root/ros2_ws/src/demo_1f/config/handover.yaml \
  --from-sequence cup_right_to_left_release --from-step cup_retreat \
  --ik-seed /tmp/b2_seed.yaml \
  --then right_return/giver_home --compare-release --samples 60 \
  2>&1 | tee /tmp/b2_return_paths.log
```

`--compare-release` reuses the exact same computed cup_retreat endpoint and other-arm state for the existing `cup_right_to_left_release/giver_via -> giver_home` chain. A failed folded segment still allows the legacy chain to be reported; an exception or invalid initial/LIN state stops the check. No candidate is written to operational YAML.

To check only the legacy chain:

```bash
python3 ptp_path_check.py cup_right_to_left_release giver_via \
  --yaml /root/ros2_ws/src/demo_1f/config/handover.yaml \
  --from-step cup_retreat --ik-seed /tmp/b2_seed.yaml \
  --then cup_right_to_left_release/giver_home --samples 60
```

`--then SEQUENCE/STEP` is repeatable and accepts same-arm joint targets. Existing current-state, joint/dual `--from-step`, `--from-sequence`, and `--via-search` forms remain available. The destination positional STEP remains a joint step, as before. Linear `--from-step` currently supports only `cup_right_to_left_release/cup_retreat`, requires a right-arm destination and `--ik-seed`, and cannot be combined with `--via-search`. `--compare-release` requires that linear start. `--then` and `--via-search` cannot be combined. An existing joint/dual example is:

```bash
python3 ptp_path_check.py right_to_left giver_home \
  --yaml /root/ros2_ws/src/demo_1f/config/handover.yaml \
  --from-step retreat_together --via-search
```

The YAML path is explicit to use the current B1 folded target rather than an older installed configuration. Confirm the container's file matches the intended host baseline; this change does not copy or modify YAML. Tool FK/IK assumes the runtime's base_link and standard arm/TCP names. YAML poses are x,y,z,qx,qy,qz,qw; conversion to w,x,y,z is explicit.

## Read the result

The tool prints seed provenance, both arms, reconstructed joints, each LIN solution, attachments including touch_links and geometry, complete world conditions, fixed transforms and ACM. It walks the LIN with the preceding solution as seed, using at least `--samples` samples and at most 5 mm translation / 2 degrees orientation between requested poses. A solution jump over 15 degrees in any joint per LIN sample is rejected. This is a conservative diagnostic threshold, not a controller limit. Each return chain uses its preceding waypoint as the next segment's start and prints segment start, target, maximum absolute joint displacement, all returned disallowed contact pairs/invalid sample fractions, and gripper-only minimum basket/table clearance in mm with object and sample location.

All-link validity uses an empty planning group (whole robot), full non-diff robot state with copied attachments, and GetStateValidity's live world/ACM. It preserves legitimate attachment touch_links and ACM allowances, including any giver-cup allowances present in the scene; it never removes cup contacts from the returned results. The printed ACM is part of the evidence, and an allowed pair is not proof of geometric separation. `valid=false` with no reported contacts is still a failure. A finite contact list can be limited by the server, so it is not an exhaustive list of every simultaneous contact pair.

The tool does not install a private scene into MoveIt. Because GetStateValidity cannot accept a private world/ACM, it compares world, ACM, transforms, padding/scaling and attachments to its initial snapshot before and after each validity call, and fails on changes. This is not an atomic snapshot guarantee: changes and reversions between reads may escape detection. Use an idle, stationary scene. No world, attachment or ACM mutation is sent, and all joint/gripper changes are request-local copies.

Gripper clearance uses ffw_pick's sampled fingers/body/wrist model and box distances; it does not measure forearm/other-arm/cup clearance. `>100 mm` means outside that model's cutoff, not an exact minimum distance. Clearance availability is mandatory for the chain report. Collision validity and gripper clearance are separate results; a positive gripper gap cannot override an invalid robot state.

Exit 0 means the requested sampled chain (and legacy comparison when selected) completed with no reported disallowed contacts or invalid states. A collision, invalid start, missing prerequisite, IK/FK error, scene change or exception exits nonzero. Via-search output remains diagnostic; finding a clear alternative does not make the original colliding direct path pass.

Sampled joint lines are approximations, not a reproduction of Pilz timing or continuous collision detection. IK reconstruction and LIN sampling can differ from the actual measured cup_retreat endpoint and runtime IK branch. The left arm is stationary in these checks. A pass does not prove concurrent PLACE_CUP/main/side motion, and does not replace runtime measured-state validation, all-pairs checks or gate_main. B1's earlier 57 mm measurement has different starting/left-arm/cup conditions and must not be reused as B2 evidence. If the folded path fails, retain the B1 shared waypoint and review a separate B2 path/sequence port after inspecting these results.

No build, tests, script/help invocation, live service requests, container copies, restarts, physical execution or commits were performed while implementing this change. Review the diff before running the commands:

```bash
git diff -- scripts/ptp_path_check.py
git diff --no-index /dev/null docs/b2_path_check.md
```

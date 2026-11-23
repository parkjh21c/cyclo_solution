# Right-arm return path comparison (plan-only)

`compare_return_paths` builds the right_return motion (right arm back to `giver_home` after a right-to-left handover) three ways. Every result goes through the runtime `smooth_segments` and `validate_smooth_path`, and the tool compares the final trajectories.

| Method | Path source | Runs |
|---|---|---|
| `straight_via` | Runtime side path: S → `giver_home_via` → `giver_home`, two straight joint segments (same as `prepare_side`) | 1 |
| `ompl` | MoveGroup plan_only, `ompl` / `RRTConnectkConfigDefault`, goal `giver_home` (no via) | `--runs` (20) |
| `stomp` | MoveGroup plan_only, `stomp` pipeline, goal `giver_home` (no via) | `--runs` (20) |
| `stomp_seed` | As `stomp`, seeded with the straight S → `giver_home_via` → `giver_home` path (`trajectory_constraints`, `stomp.num_timesteps` points, group joint order) | `--runs` (20) |
| `via_N` | As `straight_via` with a via from `--via q1,...,q7` or `--via-alpha a1,...,a7` (via = S + a (H - S) per joint) | 1 each |

- `direct` (S → home, no via) is evaluated for every start as well.
- `--planners ompl,stomp,stomp_seed` picks planner methods (default all for right_return, none for left_return); `--skip-planners` runs none.

### `--sequence left_return`

The left return starts where REFINE_CUP leaves the arm: the fixed approach pose over the cup (`refine_observe: false` opens the gripper there). That pose has a fixed height and orientation; only the cup x/y changes it. The tool therefore builds one start per cup shift instead of reading a configured pose:

- Cup BODY (cup middle): the scene's `demo_place_cup` (registered by the last approach), or `--cup x,y,z`.
- Grid: `--cup-dx` (default -0.05,-0.025,0) × `--cup-dy` (default -0.025,0,0.025) metres, applied to a private scene copy that moves the cup.
- Per shift: `approach_tcp_target` + `approach_fixed_orientation`, then move_group `/compute_ik` seeded with the runtime PTP start (left at `retreat_together`, right at `giver_home`, grippers open). Runtime Pilz PTP may still choose another IK branch.
- Extra method `lift_N` (`--lift-mm`, default 50,100): via = the start TCP raised N mm with the same orientation (IK seeded with the start), then home. It differs per start, so the runtime would need to compute it.
- `--tcp-via dx,dy,dz` (mm, repeatable): via = the start TCP moved by that offset in the planning frame, same orientation (`lift_N` is `0,0,N`).
- `--via-alpha` is relative to each start (a token `=v` sets that joint to the absolute value v); `--via` is absolute.
- Summary rows carry `start`, `cup_dx`, `cup_dy`; `run` is the start index.
- `--padding-mm X` grows world primitives by X on every side in the planning requests only (box sizes +2X, cylinder radius +X and height +2X, sphere radius +X; meshes unchanged). Validation and clearance use the real scene.
- The tool prints the `/move_group` STOMP settings it planned with. It never changes them; set `stomp.num_iterations_after_valid` with `ros2 param set` between runs. Whether STOMP picks up a changed value without a restart is unverified; compare the printed value with the results.

## Safety

- The tool is read-only. All MoveGroup goals use `plan_only=true`, `replan=false`, `look_around=false`.
- The tool never sends a controller/gripper goal or `/execute_trajectory`, never publishes, and never applies or changes the planning scene. It does not construct a `TrajectoryExecutor`.
- A timeout can cancel the tool's own planning goal.
- If the server reports an executed trajectory, the tool aborts.

## Inputs

- **Configuration:** all parameters are read from the running `/handover_demo` (`--config-node`) and passed to `load_config`. The robot model comes from the same snapshot.
- **Planning scene:** `/get_planning_scene` is read once. The tool prints the read time, the world objects and the attached objects. It stops if `demo_table`, `basket_wall_*` or `basket_slope_*` is missing.
- **Start state** (shared by all methods):
  - right arm: `right_to_left/retreat_together` right_target
  - left arm: left_target
  - right gripper: `giver_release` target
  - left gripper: `receiver_close` target
  - lift, head and attachments: scene values
- **`--from-current`:** start from one snapshot of measured `joint_states` instead. Every joint must be fresh, and every non-gripper joint must be stopped.
- **Start in collision:** if the start state is in collision, nothing is evaluated. The tool prints the contact pairs and exits with status 2.
- **ompl/stomp requests** copy the single-arm `joint` branch of `moveit_goal.cpp`:
  - group: right arm
  - goal: joint constraints with the step's `planning_tolerance`, or `tolerance` when that is unset
  - speed and acceleration: the step's scales
  - planning time: the step value, falling back to the global value
  - attempts: `ompl_planning_attempts` for OMPL, 1 otherwise
  - Each request carries the complete private scene and an explicit non-diff start state.

## Metrics (validated runs, smoothed trajectory)

- **Timing:** `plan_time`, `duration`.
- **Point counts:** `raw_points` is the planner output (4 for `straight_via`: two segments of two points). `simplified_points` is `SmoothPath::knots`.
- **`min_mid_speed`:** the minimum right-arm joint speed norm between 10 % and 90 % of the duration. This is the main metric.
- **`stop_count`:** within the same window, the number of times the speed norm drops below `--stop-threshold` (default 0.05 rad/s).
- **`reversing_joints`:** joints whose velocity changes sign (|v| > 1e-3 rad/s).
- **Path length:** `joint_path_len` is the sum of |Δq| over joints. `tcp_path_len` is in the planning frame.
- **Clearance:** `min_clearance_mm`, with the link, the object and the position as % of the duration.
  - Computed with FCL `distanceRobot`: the global minimum signed distance between the updated links of the right-arm group (gripper included) and world objects.
  - The scene ACM is applied.
  - Sampled at intervals of at most 0.05 s.
  - Negative values are penetration estimates.

Failures are recorded by stage:
- `plan`: the MoveIt error name
- `smooth`: a smoother exception
- `validate`: collision pairs or the limit that was exceeded

## Output (`--out-dir`, default `/tmp/return_compare`)

- `summary.csv`: one row per run.
- `samples.csv`: every smoothed sample of each validated run. `clearance_mm` is filled only at the clearance samples.
- `scene_objects.csv`: shapes and poses in the planning frame, used to draw outlines.
- The console shows each method's success count, failure counts by reason, and median [min, max] of each metric.

`tools/plot_return_compare.py` reads only these CSVs (matplotlib, no ROS) and writes five plots:
- `1_joints.png`
- `2_speed.png`
- `3_tcp_path.png`
- `4_clearance.png`
- `5_metrics.png`

In each plot, the median-duration run of each method is drawn bold.

## STOMP pipeline

`launch/sg2_moveit.launch.py` now also loads `stomp`. `ffw_moveit_config` has no `stomp_planning.yaml`, so MoveIt uses the `moveit_configs_utils` default:
- 60 timesteps, 40 iterations
- request adapters include `CheckStartStateBounds`

`SnapStartStateBounds` is inserted before that adapter, or first if the adapter is missing. The default planning pipeline is unchanged (`ompl`). The default yaml defines no planner configuration ID, so the STOMP `planner_id` is empty unless you set `--stomp-planner`. move_group must be restarted (restart full_demo) to load the pipeline.

## Limits

- By default the start is the configured `retreat_together` target. On the real robot the start is the measured retreat pose, which differs slightly.
- The left arm is a static obstacle. Simultaneous left-arm motion and the main/side pair check are not reproduced.
- The runtime side return never calls a planner. Using ompl or stomp at runtime needs separate work in the side preparation stage. This tool only previews that result.
- Clearance covers world objects only: no self-distance or attached-object distance.
- Outlines in `3_tcp_path.png` are box hulls. Cylinders are drawn as their bounding box, and meshes are not drawn.

#!/usr/bin/env python3
"""Read-only: why a taught joint (PTP) step fails to plan from the current robot state.

Never sends motion goals. Needs full_demo running. Reports start-state contacts, joint limits
and contacts along the joint-space straight line that Pilz PTP follows (held cup included).
--via-search tries via states that move a subset of joints to the target first, then the
rest, and prints a joint step for the first via whose two PTP lines are both clear.
--from-step starts the arm at another step's target instead of the current state.
B2 linear start supports cup_right_to_left_release/cup_retreat only, with --ik-seed FILE.
FILE has source: provenance and joints: {joint_name: radians}; both arms and grippers required.
The left seed must match menu 9 receive pose and the live cup must be attached to its TCP.
Right handover IK then sampled LIN use continuous seeds; actual Pilz joints may differ.
--then SEQUENCE/STEP appends a same-arm joint waypoint (repeatable).
--compare-release checks the legacy return from the identical computed LIN endpoint. Example:
  python3 ptp_path_check.py right_return giver_home_via --yaml handover.yaml \
    --from-sequence cup_right_to_left_release --from-step cup_retreat \
    --ik-seed /tmp/b2_seed.yaml --then right_return/giver_home --compare-release --samples 60
See docs/b2_path_check.md for the legacy comparison, seed capture and limits.
With --via-search a table follows: gripper clearance to the basket and table boxes per path,
also with the basket boxes moved (computed only; the planning scene is never changed).
Usage: python3 ptp_path_check.py SEQUENCE STEP [--yaml handover.yaml] [--samples 30] [--via-search]
       [--from-step STEP [--from-sequence NAME]]
Dual targets are not supported as destinations; --from-step supports dual endpoints only.
This tool does not reproduce B3 planning, blending or smoothing from the menu 6 state.
  e.g. python3 ptp_path_check.py cup_right_to_left_release giver_home
       python3 ptp_path_check.py right_to_left giver_home --from-step retreat_together --via-search
"""
import argparse
import copy
import itertools
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from cup_layout_check import Checker, slerp
from generate_cup_handover_targets import LEFT_JOINTS, RIGHT_JOINTS, joints_of, with_joints
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive

DEFAULT_YAML = "/root/ros2_ws/install/demo_1f/share/demo_1f/config/handover.yaml"
HERE = os.path.dirname(os.path.abspath(__file__))
PICK_SOURCE = os.path.join(HERE, "..", "modules", "dynamixel_pick")  # source tree fallback
TOWARD_CM = (1, 2, 3, 4, 5, 6)  # basket moved toward the robot
SIDE_CM = (2, 4)  # ... to either side, and away from the robot
KEEP_MM = 15  # least clearance a path should keep


def ffw_pick_modules():
    """(clearance, config, gripper) of ffw_pick: installed, else the source tree next to this."""
    try:
        from ffw_pick import clearance, config, gripper
    except ImportError:
        sys.path.insert(0, os.path.join(PICK_SOURCE, "ffw_pick"))
        from ffw_pick import clearance, config, gripper
    return clearance, config, gripper


def share_file(package, *parts):
    """A package's installed share file, else the same file in the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(package), *parts)
    except (ImportError, LookupError):
        return os.path.join(PICK_SOURCE, package, *parts)


def quat_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_matrix(pose):
    m = np.eye(4)
    o = pose.orientation
    m[:3, :3] = quat_matrix(o.w, o.x, o.y, o.z)
    m[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return m


def scene_boxes(scene):
    """(boxes for clearance.path_clearance, skipped ids) of the basket_* and demo_table objects."""
    boxes, skipped = [], []
    for obj in scene.world.collision_objects:
        if not (obj.id.startswith("basket_") or obj.id == "demo_table"):
            continue
        if obj.header.frame_id.lstrip("/") not in ("", "base_link"):
            skipped.append(f"{obj.id} (frame {obj.header.frame_id})")
            continue
        base = pose_matrix(obj.pose)
        for i, shape in enumerate(obj.primitives):
            if shape.type != SolidPrimitive.BOX or len(shape.dimensions) < 3:
                skipped.append(f"{obj.id} (not a box)")
                continue
            m = base @ pose_matrix(obj.primitive_poses[i]) if i < len(obj.primitive_poses) else base
            boxes.append({"id": obj.id if len(obj.primitives) == 1 else f"{obj.id}[{i}]",
                          "target": False, "center": m[:3, 3], "rotation": m[:3, :3],
                          "size": np.array(shape.dimensions[:3], float)})
    return boxes, skipped


def moved(boxes, offset):
    """Copies with every basket_ box shifted by offset (base_link); others kept."""
    return [dict(b, center=b["center"] + offset) if b["id"].startswith("basket_") else b
            for b in boxes]


def path_poses(node, start, names, points, link, samples):
    """Per joint line between points: end effector poses (4x4, base_link) by FK at samples."""
    segments = []
    for a, b in zip(points, points[1:]):
        poses = []
        for i in range(samples + 1):
            t = i / samples
            position, quat = node.link_pose(
                link, with_joints(start, names, [x + t * (y - x) for x, y in zip(a, b)]))
            m = np.eye(4)
            m[:3, :3], m[:3, 3] = quat_matrix(*quat), position
            poses.append(m)
        segments.append(poses)
    return segments


def nearest(segments, parts, boxes, path_clearance):
    """[(m, object, %) or None per segment] of the gripper to the boxes."""
    return [path_clearance(poses, parts, boxes)["near"] for poses in segments]


def least(near):
    """Smallest of nearest()'s per segment values, inf when all are past the cut."""
    return min((n[0] for n in near if n), default=math.inf)


def mm(value):
    return ">100" if math.isinf(value) else f"{value * 1000:.0f}"


def first_contact(node, start, names, a, b, samples):
    """(t, contacts) of the first colliding sample on the joint line a -> b, else None."""
    for i in range(samples + 1):
        t = i / samples
        state = with_joints(start, names, [x + t * (y - x) for x, y in zip(a, b)])
        contacts = node.state_collisions(state, ignore_cup=False)
        if contacts:
            return t, contacts
    return None


def via_search(node, start, names, current, target, samples):
    """Clear vias with the fewest joints moved first (all of that size); both lines checked."""
    clear = []
    for size in range(1, len(names)):
        for moved in itertools.combinations(range(len(names)), size):
            via = [target[i] if i in moved else current[i] for i in range(len(names))]
            hit = first_contact(node, start, names, current, via, samples)
            if hit is None:
                hit = first_contact(node, start, names, via, target, samples)
            label = "+".join(f"j{i + 1}" for i in moved)
            if hit is None:
                clear.append((label, via))
            print(f"  first {label:24s} " + ("clear" if hit is None else
                                             f"t {hit[0]:.2f} {', '.join(hit[1])}"), flush=True)
        if clear:
            return clear
    return clear


def arm_of(sequence_name, step):
    """Resolve giver/receiver exactly as runtime/config.cpp does."""
    giver = "left" if "left_to_right" in sequence_name else "right"
    arm = step.get("arm", "giver")
    result = {"giver": giver, "receiver": "right" if giver == "left" else "left"}.get(arm, arm)
    if result not in ("left", "right"):
        raise ValueError(f"Unsupported arm: {result}")
    return result


def step_start(sequences, sequence_name, step_name, arm):
    """(arm values, other arm values or None) at the end of a joint or dual step."""
    if sequence_name not in sequences or step_name not in sequences[sequence_name]:
        raise SystemExit(f"--from-step: no step {step_name} in {sequence_name}")
    step = sequences[sequence_name][step_name]
    kind = step.get("type", "joint")
    other = "left" if arm == "right" else "right"
    if kind == "joint":
        if arm_of(sequence_name, step) != arm:
            raise SystemExit(f"--from-step: {step_name} moves the {arm_of(sequence_name, step)}"
                             f" arm, not the {arm} arm")
        return step["target"], None
    if kind in ("dual_joint", "dual_joint_gripper"):
        return step[f"{arm}_target"], step.get(f"{other}_target")
    raise SystemExit(f"--from-step: {step_name} is {kind}; joint or dual steps only")


def gripper_model(scene, arm):
    """(clearance module, sampled gripper parts, description) or (None, None, why)."""
    try:
        clearance, config, gripper = ffw_pick_modules()
        task_file = share_file("ffw_grasp", "config", "dynamixel_top_grasp.yaml")
        calibration = share_file("ffw_grasp", "config", "robot_calibration.yaml")
        task = {**config.ros_parameters(task_file), **config.ros_parameters(calibration)}
        table = gripper.load_gripper(share_file("ffw_pick", "config", "gripper_table.yaml")).table
    except (ImportError, OSError, KeyError, ValueError) as error:
        return None, None, f"unavailable ({type(error).__name__}: {error})"
    joint = f"gripper_{arm[0]}_joint1"
    state = dict(zip(scene.robot_state.joint_state.name, scene.robot_state.joint_state.position))
    joints, openings = zip(*table)  # joints ascending, openings shrinking
    opening = float(np.interp(state.get(joint, 0.0), joints, openings))
    parts = clearance.sampled_parts(clearance.gripper_parts(task, opening))
    return clearance, parts, (f"{joint} {state.get(joint, float('nan')):.3f} rad -> opening"
                              f" {opening * 1000:.0f} mm (ffw_pick gripper_table)")


def clearance_table(node, scene, start, names, arm, current, target, clear, direct_clear,
                    samples):
    """Per path: joint move, highest TCP z and gripper clearance, also with the basket moved."""
    clearance, parts, about = gripper_model(scene, arm)
    print(f"\ngripper model: {about}")
    boxes, skipped = scene_boxes(scene)
    print(f"boxes: {', '.join(b['id'] for b in boxes) or 'none'}"
          + (f"; skipped {', '.join(skipped)}" if skipped else ""))
    basket = [b for b in boxes if b["id"] == "basket_floor"] or [
        b for b in boxes if b["id"].startswith("basket_")]
    if basket:
        cx, cy = basket[0]["center"][:2]
        toward = np.array([-math.copysign(1.0, cx), 0.0, 0.0])
        print(f"basket at x {cx:.3f} y {cy:.3f} (base_link): toward robot ="
              f" {'-' if toward[0] < 0 else '+'}x")
    else:
        toward = np.array([-1.0, 0.0, 0.0])
        print("no basket boxes: moved-basket columns are the same as now")
    reference = list(target)
    reference[0] = current[0]  # cup_ready_via / giver_via style: the target with j1 not turned yet
    paths = [("direct", [current, target], direct_clear),
             ("j1 last (giver_via style)", [current, reference, target], None)]
    paths += [(label, [current, via, target], True) for label, via in clear]
    tcp = f"gripper_{arm[0]}_tcp"
    shifts = [(f"{c}", toward * c / 100) for c in TOWARD_CM]
    others = [(f"y{s:+d}", np.array([0.0, s / 100, 0.0])) for c in SIDE_CM for s in (c, -c)]
    others += [(f"away {c}", -toward * c / 100) for c in SIDE_CM]
    rows = []
    for label, points, valid in paths:
        if valid is None:  # reference via: both lines checked like via_search
            valid = all(first_contact(node, start, names, a, b, samples) is None
                        for a, b in zip(points, points[1:]))
        move = math.degrees(sum(max(abs(y - x) for x, y in zip(a, b))
                                for a, b in zip(points, points[1:])))
        segments = path_poses(node, start, names, points, tcp, samples)
        top = max(p[2, 3] for poses in segments for p in poses)
        row = {"label": label, "valid": valid, "move": move, "top": top}
        if clearance is not None:
            row["now"] = nearest(segments, parts, boxes, clearance.path_clearance)
            row["toward"] = [
                least(nearest(segments, parts, moved(boxes, v), clearance.path_clearance))
                for _, v in shifts]
            row["others"] = [(name, least(nearest(segments, parts, moved(boxes, v),
                                                  clearance.path_clearance))) for name, v in others]
        rows.append(row)
    print(f"\n{'via':26s} valid move(deg) zmax(m)  now (object)             toward robot "
          + " ".join(f"{c}cm".rjust(5) for c in TOWARD_CM) + "   worst other")
    for row in rows:
        text = (f"{row['label']:26s} {'yes' if row['valid'] else 'no ':5s}"
                f" {row['move']:8.1f} {row['top']:8.3f}")
        if "now" in row:
            hits = [n for n in row["now"] if n]
            worst = min(hits, default=None)
            now = (f"{mm(worst[0])} mm ({worst[1]})" if worst else ">100 mm")
            name, other = min(row["others"], key=lambda o: o[1])
            text += (f"  {now:24s} {'':13s}" + " ".join(mm(v).rjust(5) for v in row["toward"])
                     + f"   {mm(other)} ({name})")
        print(text)
    if clearance is None:
        print("clearance columns unavailable: see gripper model above")
        return
    print("\nper line (now): nearest object and where along the line")
    for row in rows:
        lines = [f"line {n}: "
                 + (f"{mm(near[0])} mm {near[1]} at {near[2]}%" if near else ">100 mm")
                 for n, near in enumerate(row["now"], 1)]
        print(f"  {row['label']:26s} " + "; ".join(lines))
    print(f"\nkeeps {KEEP_MM} mm with the basket moved toward the robot by:")
    for row in rows:
        kept = 0
        for cm, value in zip((0,) + TOWARD_CM, [least(row["now"])] + row["toward"]):
            if value * 1000 < KEEP_MM:
                break
            kept = cm
        ok = least(row["now"]) * 1000 >= KEEP_MM
        most = " (every one tried)" if kept == TOWARD_CM[-1] else ""
        print(f"  {row['label']:26s} "
              + (f"up to {kept} cm{most}" if ok else f"below {KEEP_MM} mm already now"))


class PathChecker(Checker):
    """Read-only services; never discard cup contacts or invalid-without-contact results."""

    frozen = None

    def read_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.TRANSFORMS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.LINK_PADDING_AND_SCALING)
        return self.call(self.scene, request).scene

    def guard_scene(self):
        if self.frozen is None:
            return
        live = self.read_scene()
        for field in ("world", "allowed_collision_matrix", "fixed_frame_transforms", "link_padding",
                      "link_scale", "robot_model_name"):
            if getattr(live, field) != getattr(self.frozen, field):
                raise RuntimeError(f"Scene changed during check: {field}")
        if live.robot_state.attached_collision_objects != self.frozen.robot_state.attached_collision_objects:
            raise RuntimeError("Attachments changed during check")

    def state_collisions(self, state, ignore_cup=False):
        self.guard_scene()
        request = GetStateValidity.Request()
        request.robot_state = copy.deepcopy(state)
        request.robot_state.is_diff = False
        answer = self.call(self.valid, request)
        self.guard_scene()
        pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in answer.contacts})
        if not answer.valid and not pairs:
            pairs = ["INVALID (no contacts returned; bounds/constraints or contact report limit)"]
        return pairs

    def link_pose(self, link, state):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = [link]
        request.robot_state = state
        answer = self.call(self.fk_client, request)
        if answer.error_code.val != 1 or len(answer.pose_stamped) != 1:
            raise RuntimeError(f"FK failed for {link}: {answer.error_code.val}")
        pose = answer.pose_stamped[0].pose
        q = pose.orientation
        values = finite_values([pose.position.x, pose.position.y, pose.position.z, q.w, q.x, q.y, q.z], 7)
        norm = math.sqrt(sum(v * v for v in values[3:]))
        if abs(norm - 1) > 0.001:
            raise RuntimeError(f"FK returned an invalid quaternion for {link}")
        return tuple(values[:3]), tuple(v / norm for v in values[3:])

    def solve(self, arm, pose, seed):
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose[0]
        w, x, y, z = pose[1]
        target.pose.orientation = Quaternion(w=w, x=x, y=y, z=z)
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_l" if arm == "left" else "arm_r"
        request.ik_request.ik_link_name = f"gripper_{arm[0]}_tcp"
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state = copy.deepcopy(seed)
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.nanosec = 500_000_000
        answer = self.call(self.ik, request)
        if answer.error_code.val != 1:
            raise RuntimeError(f"IK failed: {answer.error_code.val}")
        names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
        result = with_joints(seed, names, finite_values(joints_of(answer.solution, names), len(names)))
        require_pose(self, result, arm, pose, "IK output")
        return result


def yaml_pose(step):
    if step.get("frame", "base_link") != "base_link":
        raise ValueError("Only base_link linear targets are supported")
    x, y, z, qx, qy, qz, qw = finite_values(step["target"], 7)
    q = (qw, qx, qy, qz)
    norm = math.sqrt(sum(v * v for v in q))
    if abs(norm - 1) > 0.001:
        raise ValueError("YAML quaternion is not normalized")
    return (x, y, z), tuple(v / norm for v in q)


def finite_values(values, count):
    values = [float(v) for v in values]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError(f"Expected {count} finite values")
    return values


def require_pose(node, state, arm, expected, label):
    position, q = node.link_pose(f"gripper_{arm[0]}_tcp", state)
    distance = math.dist(position, expected[0])
    angle = 2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(q, expected[1])))))
    print(f"{label}: {arm} pose error {distance * 1000:.3f} mm, {math.degrees(angle):.3f} deg")
    if distance > 0.001 or angle > math.radians(0.5):
        raise ValueError(f"{label}: missing required handover pose")


def b2_start(node, scene, sequences, origin, seed_file, samples):
    if origin != ("cup_right_to_left_release", "cup_retreat"):
        raise ValueError("Linear --from-step currently supports only cup_right_to_left_release/cup_retreat")
    if not seed_file:
        raise ValueError("Linear --from-step requires --ik-seed FILE")
    with open(seed_file) as handle:
        seed = yaml.safe_load(handle)
    if not isinstance(seed, dict) or not isinstance(seed.get("joints"), dict) or not seed.get("source"):
        raise ValueError("Seed requires source and joints mapping")
    required = LEFT_JOINTS + RIGHT_JOINTS + ["gripper_l_joint1", "gripper_r_joint1"]
    missing = set(required) - seed["joints"].keys()
    if missing:
        raise ValueError(f"Seed missing joints: {sorted(missing)}")
    available = set(scene.robot_state.joint_state.name)
    if set(seed["joints"]) - available or set(required) - available:
        raise ValueError("Seed/model joint names are incomplete or unknown")
    names = list(seed["joints"])
    values = finite_values([seed["joints"][n] for n in names], len(names))
    start = with_joints(scene.robot_state, names, values)
    start.is_diff = False
    print(f"IK seed file: {seed_file}; source: {seed['source']}")
    print("Seed joints:", dict(zip(names, values)))
    print("Other joints from scene:", sorted(available - set(names)))
    cup = [a for a in start.attached_collision_objects if a.object.id == "demo_place_cup"]
    if len(cup) != 1 or cup[0].link_name != "gripper_l_tcp":
        raise ValueError("Missing unique demo_place_cup attachment on gripper_l_tcp")
    obj = cup[0].object
    if obj.header.frame_id.lstrip("/") != "gripper_l_tcp":
        raise ValueError("Cup geometry must be expressed in gripper_l_tcp; cannot infer attachment transform")
    if not (obj.primitives or obj.meshes) or "demo_place_cup" in [o.id for o in scene.world.collision_objects]:
        raise ValueError("Cup geometry missing or duplicated in world")
    if not cup[0].touch_links:
        raise ValueError("Cup attachment has no explicit touch_links")
    require_pose(node, start, "left", yaml_pose(sequences["cup_right_to_left_grasp"]["cup_insert"]),
                 "Menu 9 receiver seed")
    print("Reconstructing right handover pose by IK from the explicit seed; this is not a measured state.")
    start = node.solve("right", yaml_pose(sequences["cup_left_to_right_grasp"]["cup_insert"]), start)
    print("Reconstructed handover right joints:", joints_of(start, RIGHT_JOINTS))
    for arm, seq, step in (("left", "cup_right_to_left_grasp", "receiver_close"),
                           ("right", "cup_right_to_left_release", "giver_release")):
        expected = float(sequences[seq][step]["target"][0])
        joint = f"gripper_{arm[0]}_joint1"
        print(f"Local release-state gripper override: {joint} {seed['joints'][joint]} -> {expected} ({seq}/{step})")
        start = with_joints(start, [joint], finite_values([expected], 1))
    print("Attachment geometry and ACM from verified live scene, copied only into requests:")
    print(cup[0])
    print(scene.allowed_collision_matrix)
    boxes, skipped = scene_boxes(scene)
    if skipped or not any(b["id"].startswith("basket_") for b in boxes) or not any(
            b["id"] == "demo_table" for b in boxes):
        raise ValueError(f"Missing/unsupported basket or table boxes: {skipped}")
    print("World (full snapshot):", scene.world)
    print("Fixed transforms:", scene.fixed_frame_transforms)
    print("Full seed state:", start)
    contacts = node.state_collisions(start)
    if contacts:
        raise RuntimeError(f"Handover start invalid: {contacts}")
    begin = node.link_pose("gripper_r_tcp", start)
    end = yaml_pose(sequences[origin[0]][origin[1]])
    count = max(samples, math.ceil(math.dist(begin[0], end[0]) / 0.005),
                math.ceil(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(begin[1], end[1])))))
                          / math.radians(2)))
    for i in range(1, count + 1):
        t = i / count
        pose = (tuple(a + t * (b - a) for a, b in zip(begin[0], end[0])), slerp(begin[1], end[1], t))
        solved = node.solve("right", pose, start)
        jump = max(abs(a - b) for a, b in zip(joints_of(start, RIGHT_JOINTS), joints_of(solved, RIGHT_JOINTS)))
        print(f"LIN {i}/{count}: right joints {joints_of(solved, RIGHT_JOINTS)}; max jump {math.degrees(jump):.3f} deg")
        if jump > math.radians(15):
            raise RuntimeError("LIN IK branch jump exceeds 15 deg per sample")
        contacts = node.state_collisions(solved)
        if contacts:
            raise RuntimeError(f"LIN invalid at {t:.6f}: {contacts}")
        start = solved
    print("Estimated LIN endpoint, not measured Pilz endpoint:", joints_of(start, RIGHT_JOINTS))
    return start


def chain_report(node, scene, start, sequences, refs, arm, samples):
    names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
    model_scene = copy.deepcopy(scene)
    model_scene.robot_state = copy.deepcopy(start)
    clearance, parts, about = gripper_model(model_scene, arm)
    if clearance is None:
        raise RuntimeError(f"Clearance unavailable: {about}")
    boxes, skipped = scene_boxes(scene)
    if skipped or not boxes:
        raise ValueError(f"Clearance boxes incomplete: {skipped}")
    print("Gripper-only basket/table clearance model:", about)
    print("All-link validity uses the live ACM and attachment touch_links printed above; allowed pairs are not contacts.")
    contacts = node.state_collisions(start)
    if contacts:
        raise RuntimeError(f"Chain start invalid: {contacts}")
    valid = True
    for ref in refs:
        seq, step_name = ref.split("/", 1)
        step = sequences[seq][step_name]
        if step.get("type", "joint") != "joint" or arm_of(seq, step) != arm:
            raise ValueError(f"Chain target must be a joint step on {arm}: {ref}")
        a, b = joints_of(start, names), finite_values(step["target"], len(names))
        print(f"Segment {ref}: start {a}; target {b}; max joint move {math.degrees(max(abs(y-x) for x,y in zip(a,b))):.3f} deg")
        hits = []
        for i in range(samples + 1):
            t = i / samples
            contacts = node.state_collisions(with_joints(start, names, [x+t*(y-x) for x,y in zip(a,b)]))
            if contacts:
                hits.append((t, contacts))
        near = nearest(path_poses(node, start, names, [a, b], f"gripper_{arm[0]}_tcp", samples),
                       parts, boxes, clearance.path_clearance)[0]
        if near is not None and not math.isfinite(near[0]):
            raise RuntimeError("Clearance returned a non-finite distance")
        print("  all-link sampled validity:", "FAIL " + str(hits) if hits else "no disallowed contacts / invalid states")
        print("  gripper-only minimum basket/table clearance:",
              f"{near[0] * 1000:.3f} mm to {near[1]} at {near[2]}%" if near else ">100 mm (model cutoff)")
        valid = valid and not hits
        start = with_joints(start, names, b)
    return valid


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ik-seed", help="YAML source + joints mapping; left must be at menu 9 receive pose; right seeds handover IK")
    parser.add_argument("--compare-release", action="store_true",
                        help="also check release/giver_via -> giver_home from the identical B2 LIN endpoint")
    parser.add_argument("--then", action="append", default=[], metavar="SEQUENCE/STEP",
                        help="append a same-arm joint target; repeat for a continuous chain")
    parser.add_argument("sequence")
    parser.add_argument("step")
    parser.add_argument("--yaml", default=DEFAULT_YAML)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--via-search", action="store_true")
    parser.add_argument("--from-step", help="start at a joint/dual target or B2 cup_retreat linear estimate")
    parser.add_argument("--from-sequence", help="sequence of --from-step (default: SEQUENCE)")
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    if args.then and args.via_search:
        parser.error("--then and --via-search cannot be combined")
    with open(args.yaml) as handle:
        sequences = yaml.safe_load(handle)["handover_demo"]["ros__parameters"]["motion_sequences"]
    sequence = sequences[args.sequence]
    step = sequence[args.step]
    if step.get("type", "joint") != "joint":
        raise SystemExit(f"{args.step} is {step['type']}, only joint steps are checked")
    arm = arm_of(args.sequence, step)
    names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
    target = finite_values(step["target"], len(names))
    origin = (args.from_sequence or args.sequence, args.from_step) if args.from_step else None
    linear = origin and sequences[origin[0]][origin[1]].get("type") == "linear"
    if linear and (arm != "right" or args.via_search):
        parser.error("B2 linear start requires right-arm targets without --via-search")
    if args.compare_release and not linear:
        parser.error("--compare-release requires a B2 linear --from-step")
    if args.ik_seed and not linear:
        parser.error("--ik-seed requires a linear --from-step")
    from_values = step_start(sequences, origin[0], origin[1], arm) if origin and not linear else None

    rclpy.init()
    node = PathChecker()
    try:
        scene = node.read_scene()
        finite_values(scene.robot_state.joint_state.position, len(scene.robot_state.joint_state.name))
        if len(set(scene.robot_state.joint_state.name)) != len(scene.robot_state.joint_state.name):
            raise ValueError("Scene contains duplicate joint names")
        node.frozen = copy.deepcopy(scene)
        start = copy.deepcopy(scene.robot_state)
        if linear:
            start = b2_start(node, scene, sequences, origin, args.ik_seed, args.samples)
        elif origin:
            values, other_values = from_values
            other = "left" if arm == "right" else "right"
            joints_of(start, names)
            start = with_joints(start, names, finite_values(values, len(names)))
            print(f"{arm} start from {origin[0]}/{origin[1]}")
            if other_values is not None:
                other_names = LEFT_JOINTS if other == "left" else RIGHT_JOINTS
                joints_of(start, other_names)
                start = with_joints(start, other_names, finite_values(other_values, len(other_names)))
                print(f"{other} arm also set from {origin[0]}/{origin[1]} (as right after it)")
            else:
                print(f"{other} arm from the current robot state")
            print("lift, grippers, attached objects and world from the current scene")
        else:
            print(f"{arm} start from the current robot state")
        if args.then or linear:
            print("Left joints:", joints_of(start, LEFT_JOINTS))
            print("Right joints:", joints_of(start, RIGHT_JOINTS))
            print("LIMITS: sampled joint lines and continuous-seed IK estimates, not actual Pilz trajectories.")
            print("Stationary other arm only; simultaneous main/side pair checks and gate_main remain required.")
            print("World and ACM services use the live scene, guarded before/after calls; no atomic snapshot guarantee.")
            valid = chain_report(node, scene, copy.deepcopy(start), sequences,
                                 [f"{args.sequence}/{args.step}"] + args.then, arm, args.samples)
            if args.compare_release:
                print("Comparison: legacy release path from the identical frozen LIN endpoint")
                legacy_valid = chain_report(node, scene, copy.deepcopy(start), sequences,
                                            ["cup_right_to_left_release/giver_via",
                                             "cup_right_to_left_release/giver_home"], arm, args.samples)
                print(f"Path results: requested={valid}, legacy={legacy_valid}")
                valid = valid and legacy_valid
            node.guard_scene()
            if not valid:
                raise RuntimeError("One or more chain segments failed")
            print("RESULT: sampled chain passed under the reported conditions only")
            return
        if args.via_search:
            print(f"clearance table (end): {arm} gripper boxes only (fingers, body, wrist link);"
                  " forearm and other arm links are not measured")
        print("attached:", [(a.object.id, a.link_name) for a in start.attached_collision_objects])
        print("world:", [o.id for o in scene.world.collision_objects])
        current = joints_of(start, names)
        print(f"{arm} {'start  ' if origin else 'current'} {[round(v, 3) for v in current]}")
        print(f"{arm} target  {[round(v, 3) for v in target]}")
        print("largest joint move %.1f deg" % max(math.degrees(abs(a - b))
                                                  for a, b in zip(current, target)))
        contacts = node.state_collisions(start, ignore_cup=False)
        print("start contacts:", contacts or "none")
        if contacts:
            raise RuntimeError("Start state invalid")
        first = None
        for i in range(1, args.samples + 1):
            t = i / args.samples
            state = with_joints(start, names, [a + t * (b - a) for a, b in zip(current, target)])
            contacts = node.state_collisions(state, ignore_cup=False)
            if contacts:
                first = first if first is not None else t
                print(f"  t {t:.2f}: {', '.join(contacts)}")
        print("path: " + ("clear" if first is None else f"first contact at t {first:.2f}"))
        clear = []
        if args.via_search and first is not None:
            print("via search (joints moved to the target first):")
            clear = via_search(node, start, names, current, target, args.samples)
            if not clear:
                print("no single-subset via found")
            else:
                label, via = clear[0]
                print(f"\nclear options: {[c[0] for c in clear]}")
                print(f"use '{label}' first; add before {args.step} (same arm/speed fields):")
                print(yaml.safe_dump({f"{args.step}_via": {
                    "type": "joint", "arm": step.get("arm", "giver"),
                    "velocity_scale": step.get("velocity_scale", 0.3),
                    "acceleration_scale": step.get("acceleration_scale", 0.3),
                    "tolerance": 0.05, "timeout": step.get("timeout", 60.0),
                    "target": [round(v, 6) for v in via]}}, sort_keys=False))
        if args.via_search:  # Report clearance separately from all-link validity.
            clearance_table(node, scene, start, names, arm, current, target, clear,
                            first is None, args.samples)
        node.guard_scene()
        if first is not None:
            raise RuntimeError("Direct path invalid; via candidates do not change that result")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"RESULT: FAIL ({type(error).__name__}: {error})", file=sys.stderr)
        sys.exit(1)

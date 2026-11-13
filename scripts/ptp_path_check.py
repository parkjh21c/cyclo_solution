#!/usr/bin/env python3
"""Read-only: why a taught joint (PTP) step fails to plan from the current robot state.

Never sends motion goals. Needs full_demo running. Reports start-state contacts, joint limits
and contacts along the joint-space straight line that Pilz PTP follows (held cup included).
--via-search tries via states that move a subset of joints to the target first, then the
rest, and prints a joint step for the first via whose two PTP lines are both clear.
--from-step starts the arm at another step's target instead of the current state.
With --via-search a table follows: gripper clearance to the basket and table boxes per path,
also with the basket boxes moved (computed only; the planning scene is never changed).
Usage: python3 ptp_path_check.py SEQUENCE STEP [--yaml handover.yaml] [--samples 30] [--via-search]
       [--from-step STEP [--from-sequence NAME]]
  e.g. python3 ptp_path_check.py cup_left_to_right cup_carry
       python3 ptp_path_check.py right_to_left giver_home --from-step retreat_together --via-search
"""
import argparse
import itertools
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from cup_layout_check import Checker
from generate_cup_handover_targets import LEFT_JOINTS, RIGHT_JOINTS, joints_of, with_joints
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
    for i in range(1, samples + 1):
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
    """Arm a joint step moves (config.cpp rule: giver is left unless right_to_left; pour right)."""
    right = "right_to_left" in sequence_name or sequence_name.startswith("cup_pour_right")
    giver = "right" if right else "left"
    return {"giver": giver, "receiver": "right" if giver == "left" else "left"}.get(
        step.get("arm", "giver"), step.get("arm"))


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence")
    parser.add_argument("step")
    parser.add_argument("--yaml", default=DEFAULT_YAML)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--via-search", action="store_true")
    parser.add_argument("--from-step", help="start the arm at this step's target (joint or dual)")
    parser.add_argument("--from-sequence", help="sequence of --from-step (default: SEQUENCE)")
    args = parser.parse_args()
    with open(args.yaml) as handle:
        sequences = yaml.safe_load(handle)["handover_demo"]["ros__parameters"]["motion_sequences"]
    sequence = sequences[args.sequence]
    step = sequence[args.step]
    if step.get("type", "joint") != "joint":
        raise SystemExit(f"{args.step} is {step['type']}, only joint steps are checked")
    arm = arm_of(args.sequence, step)
    names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
    target = step["target"]
    origin = (args.from_sequence or args.sequence, args.from_step) if args.from_step else None
    from_values = step_start(sequences, origin[0], origin[1], arm) if origin else None

    rclpy.init()
    node = Checker()
    try:
        scene = node.read_scene()
        start = scene.robot_state
        if origin:
            values, other_values = from_values
            other = "left" if arm == "right" else "right"
            start = with_joints(start, names, values)
            print(f"{arm} start from {origin[0]}/{origin[1]}")
            if other_values is not None:
                start = with_joints(start, LEFT_JOINTS if other == "left" else RIGHT_JOINTS,
                                    other_values)
                print(f"{other} arm also set from {origin[0]}/{origin[1]} (as right after it)")
            else:
                print(f"{other} arm from the current robot state")
            print("lift, grippers, attached objects and world from the current scene")
        else:
            print(f"{arm} start from the current robot state")
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
        print("start contacts:", node.state_collisions(start, ignore_cup=False) or "none")
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
        if args.via_search:  # extra information; the clear / contact verdicts above stand
            clearance_table(node, scene, start, names, arm, current, target, clear,
                            first is None, args.samples)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

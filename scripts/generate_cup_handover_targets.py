#!/usr/bin/env python3
"""Read-only: solve taught cup handover (left -> right) targets with MoveIt IK and write YAML.

Never sends motion goals. Run with the robot at the initial pose and the cup measured
(VALID_INIT). Merge the output under handover_demo.ros__parameters.motion_sequences.
Usage: python3 generate_cup_handover_targets.py CUP_X CUP_Y RIM_Z --output /tmp/cup_handover.yaml
"""
import argparse
import copy
import math
import random
import sys

import rclpy
import yaml
from cup_layout_check import CUP_HEIGHT, Checker, grasp, hand, shifted

LEFT_JOINTS = [f"arm_l_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"arm_r_joint{i}" for i in range(1, 8)]


def joints_of(state, names):
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    return [round(float(values[name]), 8) for name in names]


def perturbed(state, names, spread, rng):
    """Copy of a robot state with the named joints shifted: different arm postures for IK seeds."""
    shifted_state = copy.deepcopy(state)
    positions = list(shifted_state.joint_state.position)
    for i, name in enumerate(shifted_state.joint_state.name):
        if name in names:
            positions[i] += rng.uniform(-spread, spread)
    shifted_state.joint_state.position = positions
    return shifted_state


def with_joints(state, names, values):
    result = copy.deepcopy(state)
    positions = list(result.joint_state.position)
    for i, name in enumerate(result.joint_state.name):
        if name in names:
            positions[i] = values[names.index(name)]
    result.joint_state.position = positions
    return result


def search_candidate(node, rng, carried, left_hold, right_pre, right_hold, left_out):
    """(left state, right-ready state) if another left posture passes every segment, else None."""
    status, candidate = node.check("left", left_hold, perturbed(carried, LEFT_JOINTS, 0.8, rng))
    if candidate is None:
        return None
    status, other_ready = node.check("right", right_pre, candidate)
    if other_ready is None:
        return None
    status, other_inserted = node.line("right", right_pre, right_hold, other_ready)
    if other_inserted is None:
        return None
    status, other_out = node.line("left", left_hold, left_out, other_inserted)
    return (candidate, other_ready) if other_out is not None else None


def rotate(q, v):
    w, x, y, z = q
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
         (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
         (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def inverse(q):
    return (q[0], -q[1], -q[2], -q[3])


def joint_path_report(node, begin, end, names, steps=10):
    """What Pilz PTP does: straight line in joint space. Report elbow height, cup tilt, contacts."""
    _, hold_q = node.link_pose("gripper_l_tcp", end)
    cup_axis_in_hand = rotate(inverse(hold_q), (0.0, 0.0, 1.0))
    start_values = joints_of(begin, names)
    end_values = joints_of(end, names)
    worst_tilt, worst_elbow, contacts = 0.0, 0.0, set()
    for i in range(steps + 1):
        t = i / steps
        state = with_joints(end, names, [a + t * (b - a) for a, b in zip(start_values, end_values)])
        _, q = node.link_pose("gripper_l_tcp", state)
        axis = rotate(q, cup_axis_in_hand)
        worst_tilt = max(worst_tilt, math.degrees(math.acos(max(-1.0, min(1.0, axis[2])))))
        worst_elbow = max(worst_elbow, node.link_z("arm_l_link4", state))
        contacts.update(node.state_collisions(state))
    return worst_tilt, worst_elbow, sorted(contacts)


def pose_of(pose):
    (x, y, z), (w, qx, qy, qz) = pose
    return [round(v, 6) for v in (x, y, z, qx, qy, qz, w)]


class RosDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cup_x", type=float)
    parser.add_argument("cup_y", type=float)
    parser.add_argument("rim_z", type=float)
    parser.add_argument("--output", required=True)
    parser.add_argument("--transfer-x", type=float, default=0.38)
    parser.add_argument("--transfer-y", type=float, default=0.0)
    parser.add_argument("--transfer-bottom-z", type=float, default=0.88)
    parser.add_argument("--left-yaw", type=float, default=-90.0)
    parser.add_argument("--right-height", type=float, default=0.12)
    parser.add_argument("--right-depth", type=float, default=0.01,
                        help="pad center past the cup axis; about 0.027 touches the palm")
    parser.add_argument("--travel", type=float, default=0.12, help="right insert / left retreat")
    parser.add_argument("--speed", type=float, default=0.3, help="dual_joint velocity/accel scale")
    parser.add_argument("--standoff", type=float, default=0.15,
                        help="right hand waits this far behind the grasp (simultaneous mode)")
    parser.add_argument("--simultaneous", action="store_true",
                        help="both arms at once with Pilz PTP (dual_joint); needs a test, off by default")
    parser.add_argument("--elbow-tries", type=int, default=0, help="random IK seeds for a low elbow")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = Checker()
    try:
        scene = node.read_scene()
        start = scene.robot_state
        bottom = args.rim_z - CUP_HEIGHT
        cup = (args.cup_x, args.cup_y)
        left = dict(pitch=math.radians(15), height=0.075, depth=0.025)
        yaw_l = math.radians(args.left_yaw)
        yaw_r = yaw_l + math.pi
        back_l = (-math.cos(yaw_l), -math.sin(yaw_l))
        back_r = (-math.cos(yaw_r), -math.sin(yaw_r))
        transfer = (args.transfer_x, args.transfer_y)
        tz = args.transfer_bottom_z

        # Same left pick chain as /pick_cup so the carry starts from the real lifted pose.
        table = grasp(cup, bottom, 0.0, **left)
        pre = shifted(grasp(cup, bottom, 0.0, left["pitch"], left["height"], 0.0), -0.12, 0.0)
        status, state = node.check("left", pre, start)
        if state is not None:
            status, state = node.line("left", pre, table, state)
        lifted = shifted(table, 0.0, 0.0, 0.08)
        if state is not None:
            status, state = node.line("left", table, lifted, state)
        if state is None:
            sys.exit(f"pick chain failed: {status}")

        left_hold = grasp(transfer, tz, yaw_l, **left)
        right_hold = grasp(transfer, tz, yaw_r, 0.0, args.right_height, args.right_depth)
        # Simultaneous: wait --standoff behind the grasp, then one straight approach. Sequential
        # keeps the shorter --travel pre-pose.
        sequential = not args.simultaneous
        back = args.travel if sequential else args.standoff
        right_pre = shifted(right_hold, back * back_r[0], back * back_r[1])
        left_out = shifted(left_hold, args.travel * back_l[0], args.travel * back_l[1])

        failures = []
        status, carried = node.line("left", lifted, left_hold, state)
        if carried is None:
            failures.append(f"left carry: {status}")
        status, ready = node.check("right", right_pre, carried or state)
        if ready is None:
            failures.append(f"right wait: {status}")
        inserted = None
        if ready is not None:
            status, inserted = node.line("right", right_pre, right_hold, ready)
            if inserted is None:
                failures.append(f"right approach: {status}")
        if inserted is not None:
            status, out = node.line("left", left_hold, left_out, inserted)
            if out is None:
                failures.append(f"left retreat: {status}")
        if failures:
            sys.exit("not writing targets; " + "; ".join(failures))

        # The first IK solution can hold the elbow high. Try other postures that still pass every
        # segment and keep the lowest left elbow (link4 height), like the DYNAMIXEL handover pose.
        elbow = lambda st: node.link_z("arm_l_link4", st)
        reference = with_joints(start, LEFT_JOINTS,
                                [-0.18441049, 0.83519439, -0.21086751, -1.4514526,
                                 1.63022887, 1.49778519, -0.79315651])
        print(f"left elbow z: DYNAMIXEL handover pose {elbow(reference):.3f}, first IK {elbow(carried):.3f}")
        best = (elbow(carried), carried, ready)
        rng = random.Random(args.seed)
        for _ in range(args.elbow_tries if sequential else 0):
            try:
                found = search_candidate(node, rng, carried, left_hold, right_pre, right_hold, left_out)
            except RuntimeError:
                continue  # A slow validity service must not end the search.
            if found is None:
                continue
            height = elbow(found[0])
            if height < best[0] - 0.005:
                best = (height, found[0], found[1])
        print(f"left elbow z chosen {best[0]:.3f}")
        carried, ready = best[1], best[2]
        try:
            tilt, high, contacts = joint_path_report(node, state, carried, LEFT_JOINTS)
            print(f"carry PTP path: max cup tilt {tilt:.1f} deg, max elbow z {high:.3f}, "
                  f"contacts {contacts or 'none'}")
        except Exception as error:  # The report is optional; never block target generation.
            print("carry path report skipped:", error)
        left_joints = joints_of(carried, LEFT_JOINTS)
        right_joints = joints_of(ready, RIGHT_JOINTS)
        slow = {"velocity_scale": args.speed, "acceleration_scale": args.speed}
        precise = {"tolerance": 0.017453292519943295, "planning_tolerance": 0.0005,
                   "timeout": 60.0}
        close = {"type": "gripper", "arm": "receiver", "gripper_wait_only": True,
                 "target": [0.8], "duration": 1.0, "velocity_scale": 1.0,
                 "acceleration_scale": 1.0, "timeout": 12.0}
        release = {"type": "gripper", "arm": "giver", "target": [0.0], "duration": 0.6,
                   "velocity_scale": 1.0, "acceleration_scale": 1.0, "timeout": 12.0}
        lin = {"type": "linear", "frame": "base_link", "velocity_scale": 0.06,
               "acceleration_scale": 0.015, "timeout": 30.0}
        insert = {**lin, "arm": "receiver", "target": pose_of(right_hold)}
        # Receiver contact is allowed only between the transfer and the grasp sequences.
        sequential_steps = {
            "steps": ["cup_carry", "cup_ready"],
            "cup_carry": {"type": "joint", "arm": "giver", **slow, **precise,
                          "target": left_joints},
            "cup_ready": {"type": "joint", "arm": "receiver", **slow, **precise,
                          "target": right_joints},
        }
        simultaneous_steps = {
            "steps": ["cup_ready"],
            "cup_ready": {"type": "dual_joint", "pipeline": "pilz_industrial_motion_planner",
                          "planner": "PTP", **slow, **precise,
                          "left_target": left_joints, "right_target": right_joints},
        }
        sequences = {
            "cup_left_to_right": (sequential_steps if sequential else simultaneous_steps),
            "cup_left_to_right_grasp": {
                "steps": ["cup_insert", "receiver_close"],
                "cup_insert": {**insert, "velocity_scale": 0.08, "acceleration_scale": 0.02},
                "receiver_close": close,
            },
            "cup_left_to_right_release": {
                "steps": ["giver_release", "cup_retreat"],
                "giver_release": release,
                "cup_retreat": {**lin, "arm": "giver", "target": pose_of(left_out)},
            },
        }
        document = {"handover_demo": {"ros__parameters": {"motion_sequences": sequences}}}
        with open(args.output, "w") as handle:
            yaml.dump(document, handle, Dumper=RosDumper, sort_keys=False)
        print("left hold joints ", left_joints)
        print("right pre joints ", right_joints)
        print("right insert pose", pose_of(right_hold))
        print("left retreat pose", pose_of(left_out))
        print("wrote", args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

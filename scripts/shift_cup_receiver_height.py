#!/usr/bin/env python3
"""Read-only: move menu 7 (cup_left_to_right) arm targets up or down by DZ.

--arm right (default): the right cup_ready / insert targets. --arm left: the shared left hold,
checked against both right targets. Never sends motion goals; uses /compute_fk, /compute_ik and
/check_state_validity only. Seeds IK with the current targets (same posture); prints YAML to paste.
Usage: python3 shift_cup_receiver_height.py -0.012 [--arm left] [--config .../handover.yaml]
"""
import argparse
import math
import sys

import rclpy
import yaml
from cup_layout_check import Checker

RIGHT = [f"arm_r_joint{i}" for i in range(1, 8)]
LEFT = [f"arm_l_joint{i}" for i in range(1, 8)]


def with_joints(state, names, values):
    joint_state = state.joint_state
    positions = list(joint_state.position)
    for i, name in enumerate(joint_state.name):
        if name in names:
            positions[i] = values[names.index(name)]
    joint_state.position = positions
    return state


def joints_of(state, names):
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    return [round(float(values[name]), 8) for name in names]


def shift(node, arm, joints, other, dz):
    """(new joints, new TCP pose, solved state) for one arm target moved by dz; exits on failure."""
    names, others = (LEFT, RIGHT) if arm == "left" else (RIGHT, LEFT)
    state = with_joints(with_joints(node.read_scene().robot_state, others, other), names, joints)
    (x, y, z), q = node.link_pose(f"gripper_{arm[0]}_tcp", state)
    status, solved = node.check(arm, ((x, y, z + dz), q), state)
    if solved is None:
        sys.exit(f"{arm} at z {z + dz:.4f}: {status}")
    values = joints_of(solved, names)
    jump = max(abs(a - b) for a, b in zip(values, joints))
    (nx, ny, nz), _ = node.link_pose(f"gripper_{arm[0]}_tcp", solved)
    print(f"{arm}: TCP z {z:.4f} -> {nz:.4f} at [{nx:.4f}, {ny:.4f}], max joint change {math.degrees(jump):.1f} deg")
    if jump > math.radians(20):
        sys.exit("posture flipped (joint change above 20 deg); not writing")
    return values, ((nx, ny, nz), q), solved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dz", type=float, help="TCP height change in m (negative = lower)")
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--config", default="/root/ros2_ws/src/demo_1f/config/handover.yaml")
    args = parser.parse_args()
    if abs(args.dz) > 0.05:
        sys.exit("dz above 5 cm; regenerate with generate_cup_handover_targets.py instead")

    with open(args.config) as f:
        sequences = yaml.safe_load(f)["handover_demo"]["ros__parameters"]["motion_sequences"]
    ready = sequences["cup_left_to_right"]["cup_ready"]
    insert = sequences["cup_left_to_right_continuous"]["cup_insert_joint"]
    if ready["left_target"] != insert["left_target"]:
        sys.exit("cup_ready and cup_insert_joint left_target differ")
    left = ready["left_target"]

    rclpy.init()
    node = Checker()
    try:
        new = {}
        if args.arm == "right":
            new["cup_ready right_target"], ready_pose, ready_state = shift(
                node, "right", ready["right_target"], left, args.dz)
            new["cup_insert_joint right_target"], insert_pose, _ = shift(
                node, "right", insert["right_target"], left, args.dz)
            status, _ = node.line("right", ready_pose, insert_pose, ready_state)
            print(f"ready -> insert straight-line IK/collision walk: {status}")
            if status != "OK":
                sys.exit("not writing targets")
            lin = sequences["cup_left_to_right_grasp"]["cup_insert"]["target"]
            print(f"cup_left_to_right_grasp cup_insert z: {lin[2]} -> {round(lin[2] + args.dz, 6)}")
        else:
            values, pose, _ = shift(node, "left", left, insert["right_target"], args.dz)
            state = with_joints(node.read_scene().robot_state, RIGHT, ready["right_target"])
            status, _ = node.check("left", pose, with_joints(state, LEFT, values))
            print(f"left hold with right at cup_ready: {status}")
            if status != "OK":
                sys.exit("not writing targets")
            new["left_target (cup_ready and cup_insert_joint)"] = values
            for seq, step in (("cup_left_to_right_release", "cup_retreat"),
                              ("cup_right_to_left_grasp", "cup_insert")):
                z = sequences[seq][step]["target"][2]
                print(f"{seq} {step} z: {z} -> {round(z + args.dz, 6)}")
        print("\n# paste")
        for name, values in new.items():
            print(f"{name}:")
            for v in values:
                print(f"  - {v}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

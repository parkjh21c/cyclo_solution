#!/usr/bin/env python3
"""Read-only: why a taught joint (PTP) step fails to plan from the current robot state.

Never sends motion goals. Needs full_demo running. Reports start-state contacts, joint limits
and contacts along the joint-space straight line that Pilz PTP follows (held cup included).
--via-search tries via states that move a subset of joints to the target first, then the
rest, and prints a joint step for the first via whose two PTP lines are both clear.
Usage: python3 ptp_path_check.py SEQUENCE STEP [--yaml handover.yaml] [--samples 30] [--via-search]
  e.g. python3 ptp_path_check.py cup_left_to_right cup_carry
"""
import argparse
import itertools
import math

import rclpy
import yaml
from cup_layout_check import Checker
from generate_cup_handover_targets import LEFT_JOINTS, RIGHT_JOINTS, joints_of, with_joints

DEFAULT_YAML = "/root/ros2_ws/install/demo_1f/share/demo_1f/config/handover.yaml"


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence")
    parser.add_argument("step")
    parser.add_argument("--yaml", default=DEFAULT_YAML)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--via-search", action="store_true")
    args = parser.parse_args()
    with open(args.yaml) as handle:
        sequences = yaml.safe_load(handle)["handover_demo"]["ros__parameters"]["motion_sequences"]
    sequence = sequences[args.sequence]
    step = sequence[args.step]
    if step.get("type", "joint") != "joint":
        raise SystemExit(f"{args.step} is {step['type']}, only joint steps are checked")
    # Same rule as config.cpp: giver is left unless the name says right_to_left; pour is right.
    right = "right_to_left" in args.sequence or args.sequence.startswith("cup_pour_right")
    giver = "right" if right else "left"
    arm = {"giver": giver, "receiver": "right" if giver == "left" else "left"}.get(
        step.get("arm", "giver"), step.get("arm"))
    names = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS
    target = step["target"]

    rclpy.init()
    node = Checker()
    try:
        scene = node.read_scene()
        start = scene.robot_state
        print("attached:", [(a.object.id, a.link_name) for a in start.attached_collision_objects])
        print("world:", [o.id for o in scene.world.collision_objects])
        current = joints_of(start, names)
        print(f"{arm} current {[round(v, 3) for v in current]}")
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

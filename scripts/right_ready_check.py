#!/usr/bin/env python3
"""Read-only: pick the right hand's wait distance whose Pilz PTP path avoids the held cup.

Needs full_demo running and the robot where cup_carry ended (left holding the cup at the
transfer pose, right arm at the initial pose). For each wait distance behind the grasp it solves
IK, walks the joint-space interpolation from the current right arm joints (cup contacts
included) and checks the straight approach to the grasp. Prints YAML-ready joints for passes.
Usage: python3 right_ready_check.py [--waits 0.12,0.2,0.25,0.3]
"""
import argparse
import math

import rclpy
from cup_layout_check import Checker, grasp, shifted
from generate_cup_handover_targets import RIGHT_JOINTS, joints_of, with_joints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--waits", default="0.12,0.2,0.25,0.3,0.35")
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--transfer-x", type=float, default=0.38)
    parser.add_argument("--transfer-y", type=float, default=0.0)
    parser.add_argument("--transfer-bottom-z", type=float, default=0.88)
    parser.add_argument("--left-yaw", type=float, default=-90.0)
    parser.add_argument("--right-height", type=float, default=0.12)
    parser.add_argument("--right-depth", type=float, default=0.01)
    args = parser.parse_args()

    rclpy.init()
    node = Checker()
    try:
        scene = node.read_scene()
        start = scene.robot_state
        print("attached:", [o.object.id for o in scene.robot_state.attached_collision_objects])
        begin = joints_of(start, RIGHT_JOINTS)
        yaw_r = math.radians(args.left_yaw) + math.pi
        back = (-math.cos(yaw_r), -math.sin(yaw_r))
        hold = grasp((args.transfer_x, args.transfer_y), args.transfer_bottom_z, yaw_r,
                     0.0, args.right_height, args.right_depth)
        print("wait   IK   path contacts (first t / pairs)             approach LIN")
        for wait in [float(v) for v in args.waits.split(",")]:
            pose = shifted(hold, wait * back[0], wait * back[1])
            status, solved = node.check("right", pose, start)
            if solved is None:
                print(f"{wait:4.2f}  {status}")
                continue
            target = joints_of(solved, RIGHT_JOINTS)
            first, pairs = None, set()
            for i in range(args.steps + 1):
                t = i / args.steps
                state = with_joints(
                    start, RIGHT_JOINTS, [a + t * (b - a) for a, b in zip(begin, target)])
                found = node.state_collisions(state, ignore_cup=False)
                if found:
                    first = t if first is None else first
                    pairs.update(found)
            lin_status, _ = node.line("right", pose, hold, solved)
            path = "clear" if first is None else f"t={first:.2f} " + ",".join(sorted(pairs))
            print(f"{wait:4.2f}  OK   {path:45s} {lin_status}")
            if first is None and lin_status == "OK":
                print("      right joints", target)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

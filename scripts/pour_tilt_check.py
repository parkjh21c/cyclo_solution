#!/usr/bin/env python3
"""Read-only: why pour_tilt fails from the current pose (right hand lowered over the basket).

Never sends motion goals. Needs full_demo running and the cup held by the right hand. Walks the
LIN from the current TCP to the pour_tilt target like Pilz (slerp, 4 deg steps) and reports the
first IK failure, joint limit or contact (held cup included), plus the basket objects.
Usage: python3 pour_tilt_check.py [--yaml handover.yaml] [--step pour_tilt]
"""
import argparse
import math

import rclpy
import yaml
from cup_layout_check import slerp
from generate_cup_pour_targets import TCP, PourChecker, basket
from ptp_path_check import DEFAULT_YAML


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", default=DEFAULT_YAML)
    parser.add_argument("--step", default="pour_tilt")
    parser.add_argument("--step-deg", type=float, default=4.0)
    args = parser.parse_args()
    with open(args.yaml) as handle:
        sequences = yaml.safe_load(handle)["handover_demo"]["ros__parameters"]["motion_sequences"]
    x, y, z, qx, qy, qz, qw = sequences["cup_pour_right"][args.step]["target"]
    goal = ((x, y, z), (qw, qx, qy, qz))

    rclpy.init()
    node = PourChecker()
    try:
        scene = node.read_scene()
        state = scene.robot_state
        (bx, by), top, inner_x, inner_y = basket(scene)
        print(f"basket center {bx:.3f} {by:.3f} (targets solved at 0.412 -0.247), wall top {top:.3f}, "
              f"inner x {inner_x[0]:.3f}..{inner_x[1]:.3f} y {inner_y[0]:.3f}..{inner_y[1]:.3f}")
        start = node.link_pose(TCP, state)
        print(f"TCP now  {[round(v, 3) for v in start[0]]}, goal {[round(v, 3) for v in goal[0]]}")
        print("start contacts:", node.state_collisions(state, ignore_cup=False) or "none")
        dot = min(1.0, abs(sum(a * b for a, b in zip(start[1], goal[1]))))
        angle = math.degrees(2 * math.acos(dot))
        count = max(1, math.ceil(angle / args.step_deg))
        print(f"rotation {angle:.1f} deg in {count} steps")
        for i in range(1, count + 1):
            t = i / count
            point = tuple(a + t * (b - a) for a, b in zip(start[0], goal[0]))
            status, next_state = node.check("right", (point, slerp(start[1], goal[1], t)), state)
            if next_state is None:
                print(f"first failure at {angle * t:.1f} deg: {status}")
                return
            state = next_state
        print("tilt path clear")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

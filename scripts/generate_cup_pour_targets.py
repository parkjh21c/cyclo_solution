#!/usr/bin/env python3
"""Read-only: solve the right-hand cup pour over the basket with MoveIt IK and write YAML.

Never sends motion goals. Needs full_demo running, the basket registered (menu 2) and the cup
held by the right hand (menu 7). Cup contacts ARE checked here (cup vs basket, table, robot).
Carry: Pilz PTP to a joint target with the hand turned about the upright cup axis (--yaws),
checked along the joint-space path. Pour: LIN down, LIN rotation about the fixed TCP so the lip
ends --clearances above the wall top at --design-angle, wait, LIN back, PTP to the start joints.
Merge the output under handover_demo.ros__parameters.motion_sequences.
Usage: python3 generate_cup_pour_targets.py --output /tmp/cup_pour.yaml [--angle 60]
"""
import argparse
import math
import random
import sys

import rclpy
import yaml
from basket_info import as_tuple, compose, rotate
from cup_layout_check import Checker, axis_angle, quat_mul
from generate_cup_handover_targets import RIGHT_JOINTS, joints_of, perturbed, with_joints
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK, GetStateValidity

CUP_ID = "demo_place_cup"
TCP = "gripper_r_tcp"
REAL_RADIUS = 0.043  # Measured cup; the collision model is wider.
DIRECTIONS = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}
# SG2 right arm URDF limits (modules/dynamixel_pick/ffw_pick/test/arm_r.urdf).
RIGHT_LIMITS = [(-3.14, 3.14), (-3.14, 0.0), (-3.14, 3.14), (-2.9361, 1.0786),
                (-3.14, 3.14), (-1.57, 1.57), (-1.8201, 1.5804)]


class PourChecker(Checker):
    """Checker whose IK check reports every contact, including the held cup."""

    limit_margin = 0.05  # rad; solutions closer to a joint limit are rejected.

    def check(self, arm, pose, seed):
        (x, y, z), (w, qx, qy, qz) = pose
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=w)
        ik = GetPositionIK.Request()
        ik.ik_request.group_name = "arm_r"
        ik.ik_request.ik_link_name = TCP
        ik.ik_request.pose_stamped = target
        ik.ik_request.robot_state = seed
        ik.ik_request.avoid_collisions = False
        ik.ik_request.timeout.nanosec = 500_000_000
        answer = self.call(self.ik, ik)
        if answer.error_code.val != 1:
            return "NOIK", None
        for i, (value, (low, high)) in enumerate(
                zip(joints_of(answer.solution, RIGHT_JOINTS), RIGHT_LIMITS)):
            if min(value - low, high - value) < self.limit_margin:
                return f"LIMIT j{i + 1} {value:+.3f}", None
        request = GetStateValidity.Request()
        request.robot_state = answer.solution
        request.robot_state.is_diff = True
        pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}"
                        for c in self.call(self.valid, request).contacts})
        if pairs:
            return "COLL " + ",".join(pairs), None
        return "OK", answer.solution

    def closest_ik(self, pose, start, tries, rng):
        """Collision-free IK nearest to start joints over random seeds; (status, state)."""
        best, status, origin = None, "NOIK", joints_of(start, RIGHT_JOINTS)
        for i in range(tries):
            seed = start if i == 0 else perturbed(start, RIGHT_JOINTS, 0.8, rng)
            result, state = self.check("right", pose, seed)
            if state is None:
                status = result if status == "NOIK" else status
                continue
            jump = max(abs(a - b) for a, b in zip(joints_of(state, RIGHT_JOINTS), origin))
            if best is None or jump < best[0]:
                best = (jump, state)
        return ("OK", best[1]) if best else (status, None)


def held_cup(node, scene):
    """(center, up axis, height, model radius) of the cup attached to the right TCP, base_link."""
    for attached in scene.robot_state.attached_collision_objects:
        obj = attached.object
        if obj.id != CUP_ID or not obj.primitives:
            continue
        if attached.link_name != TCP:
            sys.exit(f"cup is attached to {attached.link_name}, not {TCP}: run menu 7 first")
        frame = obj.header.frame_id or attached.link_name
        parent = (((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)) if frame == "base_link"
                  else node.link_pose(frame, scene.robot_state))
        center, q = compose(compose(parent, as_tuple(obj.pose)), as_tuple(obj.primitive_poses[0]))
        height, radius = obj.primitives[0].dimensions[:2]
        return center, rotate(q, (0.0, 0.0, 1.0)), height, radius
    sys.exit("no cup attached to the right hand: run menu 6 then 7 first")


def basket(scene):
    """(center xy, wall top z, inner x range, inner y range) from the basket objects."""
    objects = {o.id: o for o in scene.world.collision_objects if o.id.startswith("basket_")}
    if "basket_floor" not in objects:
        sys.exit("no basket in the scene: run a DYNAMIXEL pick (menu 2) first")
    floor = compose(as_tuple(objects["basket_floor"].pose),
                    as_tuple(objects["basket_floor"].primitive_poses[0]))[0]
    walls = []
    for name, obj in objects.items():
        if name.startswith("basket_wall"):
            p = compose(as_tuple(obj.pose), as_tuple(obj.primitive_poses[0]))[0]
            walls.append((p, obj.primitives[0].dimensions[2]))
    top = max(p[2] + h / 2 for p, h in walls)
    xs, ys = [p[0] for p, _ in walls], [p[1] for p, _ in walls]
    return floor[:2], top, (min(xs), max(xs)), (min(ys), max(ys))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def ptp_report(node, start, goal, up_local, samples=20):
    """Pilz PTP is linear in joint space: (max cup tilt deg, contacts) along that path."""
    a, b = joints_of(start, RIGHT_JOINTS), joints_of(goal, RIGHT_JOINTS)
    tilt, contacts = 0.0, set()
    for i in range(1, samples + 1):
        t = i / samples
        state = with_joints(start, RIGHT_JOINTS, [x + t * (y - x) for x, y in zip(a, b)])
        _, q = node.link_pose(TCP, state)
        tilt = max(tilt, math.degrees(math.acos(min(1.0, rotate(q, up_local)[2]))))
        contacts.update(node.state_collisions(state, ignore_cup=False))
    return tilt, sorted(contacts)


def tilt_sweep(node, position, q0, tilt, angle, seed, step_deg):
    """Rotate about the fixed TCP up to angle; (largest angle reached, status)."""
    reached, state = 0.0, seed
    count = max(1, math.ceil(angle / step_deg))
    for i in range(1, count + 1):
        deg = angle * i / count
        q = quat_mul(axis_angle(tilt, math.radians(deg)), q0)
        status, state = node.check("right", (position, q), state)
        if state is None:
            return reached, status
        reached = deg
    return reached, "OK"


def pose_list(pose):
    (x, y, z), (w, qx, qy, qz) = pose
    return [round(v, 6) for v in (x, y, z, qx, qy, qz, w)]


class RosDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--angle", type=float, default=130.0, help="tilt written to the YAML")
    parser.add_argument("--design-angle", type=float, default=130.0,
                        help="tilt at which the lip sits over the target; also the sweep limit")
    parser.add_argument("--yaws", default="-90,-75,-60,-45",
                        help="hand turn about the cup axis, deg; -90 points the right hand forward")
    parser.add_argument("--directions", default="+y",
                        help="lip directions, preference order; +y pours right to left")
    parser.add_argument("--shifts", default="0.0", help="lip offset along the direction")
    parser.add_argument("--clearances", default="0.05", help="lip height above the wall top")
    parser.add_argument("--approach-margin", type=float, default=0.03,
                        help="cup bottom above the wall top while moving over the basket")
    parser.add_argument("--max-carry-tilt", type=float, default=10.0, help="cup tilt allowed in the PTP")
    parser.add_argument("--ik-tries", type=int, default=8)
    parser.add_argument("--limit-margin", type=float, default=0.05, help="rad from any joint limit")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds at full tilt")
    parser.add_argument("--speed", type=float, default=0.08, help="LIN velocity scale")
    parser.add_argument("--joint-speed", type=float, default=0.2, help="PTP velocity scale")
    parser.add_argument("--step-deg", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    numbers = lambda text: [float(v) for v in text.split(",")]
    if args.angle > args.design_angle:
        sys.exit("--angle must not exceed --design-angle (the sweep stops there)")
    rng = random.Random(args.seed)

    rclpy.init()
    node = PourChecker()
    node.limit_margin = args.limit_margin
    try:
        scene = node.read_scene()
        start = scene.robot_state
        tcp0, q0 = node.link_pose(TCP, start)
        center, up, height, model_radius = held_cup(node, scene)
        (bx, by), top, inner_x, inner_y = basket(scene)
        rim = add(center, tuple(height / 2 * u for u in up))
        bottom0 = center[2] - height / 2 * up[2]
        up_local = rotate((q0[0], -q0[1], -q0[2], -q0[3]), up)  # Cup axis in the TCP frame.
        print(f"held cup: center {[round(v, 3) for v in center]}, up {[round(v, 3) for v in up]}, "
              f"height {height:.3f}, model radius {model_radius:.3f}")
        print(f"right TCP {[round(v, 3) for v in tcp0]}")
        print(f"basket center {bx:.3f} {by:.3f}, wall top {top:.3f}, inner x {inner_x[0]:.3f}.."
              f"{inner_x[1]:.3f} y {inner_y[0]:.3f}..{inner_y[1]:.3f}")
        if up[2] < 0.95:
            print(f"WARNING: cup is not upright now (up z {up[2]:.3f})")

        chosen = None
        print("\nyaw dir  shift | TCP over basket (x y z)  | carry PTP (tilt, contacts) | lower | tilt reached")
        for yaw in numbers(args.yaws):
            # Turn hand + cup about the vertical cup axis; the cup stays upright.
            q_yaw = axis_angle((0.0, 0.0, 1.0), math.radians(yaw))
            axis_pt = (center[0], center[1], tcp0[2])
            tcp_b = add(axis_pt, rotate(q_yaw, sub(tcp0, axis_pt)))
            q_b = quat_mul(q_yaw, q0)
            for name in args.directions.split(","):
                d = DIRECTIONS[name]
                tilt_axis = (-d[1], d[0], 0.0)  # z x d: tilting about it brings the top toward d.
                q_pour = axis_angle(tilt_axis, math.radians(args.design_angle))
                lip0 = (rim[0] + REAL_RADIUS * d[0], rim[1] + REAL_RADIUS * d[1], rim[2])
                for clearance in numbers(args.clearances):
                    for shift in numbers(args.shifts):
                        lip_goal = (bx + shift * d[0], by + shift * d[1], top + clearance)
                        tcp = sub(lip_goal, rotate(q_pour, sub(lip0, tcp_b)))
                        bottom = bottom0 + tcp[2] - tcp0[2]
                        raise_z = max(0.0, top + args.approach_margin - bottom)
                        high = (tcp[0], tcp[1], tcp[2] + raise_z)
                        row = (f"{yaw:4.0f} {name:3s} {shift:+.2f} | {tcp[0]:.3f} {tcp[1]:.3f} "
                               f"{tcp[2]:.3f} |")
                        status, above = node.closest_ik((high, q_b), start, args.ik_tries, rng)
                        if above is None:
                            print(row, status[:60], flush=True)
                            continue
                        carry_tilt, contacts = ptp_report(node, start, above, up_local)
                        row += f" {carry_tilt:4.1f} deg {','.join(contacts)[:40] or 'none':10s} |"
                        if contacts or carry_tilt > args.max_carry_tilt:
                            print(row, flush=True)
                            continue
                        state = above
                        if raise_z > 0:
                            status, state = node.line("right", (high, q_b), (tcp, q_b), above)
                            row += f" {status[:30]:5s} |"
                        else:
                            row += "   -   |"
                        if state is None:
                            print(row, flush=True)
                            continue
                        reached, status = tilt_sweep(node, tcp, q_b, tilt_axis, args.design_angle,
                                                     state, args.step_deg)
                        print(row, f"{reached:5.1f} deg {'' if status == 'OK' else status}", flush=True)
                        if reached >= args.design_angle and chosen is None:
                            chosen = dict(yaw=yaw, name=name, shift=shift, clearance=clearance,
                                          tcp=tcp, high=high, raise_z=raise_z, q=q_b,
                                          axis=tilt_axis, lip0=lip0, tcp_b=tcp_b,
                                          above=joints_of(above, RIGHT_JOINTS), tilt=carry_tilt)
        if chosen is None:
            sys.exit("\nno candidate reaches the design angle; not writing targets")

        c = chosen
        q_end = axis_angle(c["axis"], math.radians(args.angle))
        lin = {"type": "linear", "arm": "right", "frame": "base_link",
               "velocity_scale": args.speed, "acceleration_scale": args.speed / 4, "timeout": 60.0}
        joint = {"type": "joint", "arm": "right", "velocity_scale": args.joint_speed,
                 "acceleration_scale": args.joint_speed, "tolerance": 0.017453292519943295,
                 "planning_tolerance": 0.0005, "timeout": 60.0}
        steps = {"pour_above": {**joint, "target": c["above"]}}
        if c["raise_z"] > 0:
            steps["pour_lower"] = {**lin, "target": pose_list((c["tcp"], c["q"]))}
        steps["pour_tilt"] = {**lin, "target": pose_list((c["tcp"], quat_mul(q_end, c["q"])))}
        steps["pour_hold"] = {"type": "wait", "arm": "right", "duration": args.hold,
                              "timeout": args.hold + 5.0}
        steps["pour_untilt"] = {**lin, "target": pose_list((c["tcp"], c["q"]))}
        if c["raise_z"] > 0:
            steps["pour_raise"] = {**lin, "target": pose_list((c["high"], c["q"]))}
        steps["pour_return"] = {**joint, "target": joints_of(start, RIGHT_JOINTS)}
        sequences = {"cup_pour_right": {"steps": list(steps), **steps}}
        document = {"handover_demo": {"ros__parameters": {"motion_sequences": sequences}}}
        with open(args.output, "w") as handle:
            yaml.dump(document, handle, Dumper=RosDumper, sort_keys=False)

        lip_end = add(c["tcp"], rotate(q_end, sub(c["lip0"], c["tcp_b"])))
        print(f"\nchosen: hand yaw {c['yaw']:.0f}, lip toward {c['name']}, shift {c['shift']:+.2f}, "
              f"clearance {c['clearance']:.2f}, approach raise {c['raise_z']:.3f}, "
              f"carry tilt {c['tilt']:.1f} deg")
        print(f"lip at {args.angle:.0f} deg: {[round(v, 3) for v in lip_end]}")
        print("steps:", ", ".join(steps))
        print("wrote", args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

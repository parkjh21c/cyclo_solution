#!/usr/bin/env python3
"""Read-only layout check for cup handover and pour: IK + collision along every segment.

Never sends motion goals. Run with the robot at the initial pose and the cup measured.
Contacts with demo_place_cup are ignored: the cup is not attached to the hand here, so
cup-vs-robot clearance is NOT checked. Usage:
  python3 cup_layout_check.py CUP_X CUP_Y RIM_Z [--pour-x X --pour-y Y]
"""
import argparse
import itertools
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.node import Node

CUP_HEIGHT = 0.165
PAD_OFFSET = 0.03  # Pad center behind the TCP along hand forward.


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def axis_angle(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def slerp(a, b, t):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-x for x in b), -dot
    if dot > 0.9995:
        q = tuple(x + t * (y - x) for x, y in zip(a, b))
    else:
        theta = math.acos(dot)
        q = tuple((math.sin((1 - t) * theta) * x + math.sin(t * theta) * y) / math.sin(theta)
                  for x, y in zip(a, b))
    n = math.sqrt(sum(x * x for x in q))
    return tuple(x / n for x in q)


def hand(yaw, pitch, roll=0.0):
    """SG2 TCP quaternion (w,x,y,z): heading yaw, fingers tilted down by pitch, rolled left by roll."""
    level = quat_mul(axis_angle((0, 0, 1), yaw), axis_angle((0, 1, 0), -math.pi / 2))
    q = quat_mul(axis_angle((-math.sin(yaw), math.cos(yaw), 0), pitch), level)
    forward = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch))
    return quat_mul(axis_angle(forward, -roll), q)  # Negative about forward tips the top to the left.


def grasp(axis_xy, bottom_z, yaw, pitch, height, depth):
    """(TCP xyz, quaternion) holding an upright cup: pads at axis + depth along the heading."""
    h = (math.cos(yaw), math.sin(yaw))
    forward = (math.cos(pitch) * h[0], math.cos(pitch) * h[1], -math.sin(pitch))
    pad = (axis_xy[0] + depth * h[0], axis_xy[1] + depth * h[1], bottom_z + height)
    return tuple(p + PAD_OFFSET * f for p, f in zip(pad, forward)), hand(yaw, pitch)


def shifted(pose, dx, dy, dz=0.0):
    (x, y, z), q = pose
    return (x + dx, y + dy, z + dz), q


class Checker(Node):
    def __init__(self):
        super().__init__("cup_layout_check")
        self.scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")

    def link_z(self, link, state):
        """Height of a link origin in base_link for a robot state (read-only FK)."""
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = [link]
        request.robot_state = state
        answer = self.call(self.fk_client, request)
        return answer.pose_stamped[0].pose.position.z if answer.pose_stamped else float("nan")

    def link_pose(self, link, state):
        """(position xyz, quaternion w,x,y,z) of a link in base_link; read-only FK."""
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = [link]
        request.robot_state = state
        pose = self.call(self.fk_client, request).pose_stamped[0].pose
        q = pose.orientation
        return (pose.position.x, pose.position.y, pose.position.z), (q.w, q.x, q.y, q.z)

    def state_collisions(self, state, ignore_cup=True):
        """Contact pairs of a robot state (cup contacts ignored by default, as in check)."""
        request = GetStateValidity.Request()
        request.robot_state = state
        request.robot_state.is_diff = True
        validity = self.call(self.valid, request)
        return sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in validity.contacts
                       if not ignore_cup or "demo_place_cup" not in (c.contact_body_1, c.contact_body_2)})

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError(f"No response: {client.srv_name}")
        return future.result()

    def read_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        return self.call(self.scene, request).scene

    def check(self, arm, pose, seed):
        """Return (status, state). The other arm keeps its joints from seed."""
        (x, y, z), (w, qx, qy, qz) = pose
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=w)
        ik = GetPositionIK.Request()
        ik.ik_request.group_name = "arm_l" if arm == "left" else "arm_r"
        ik.ik_request.ik_link_name = "gripper_l_tcp" if arm == "left" else "gripper_r_tcp"
        ik.ik_request.pose_stamped = target
        ik.ik_request.robot_state = seed
        ik.ik_request.avoid_collisions = False
        ik.ik_request.timeout.nanosec = 500_000_000
        answer = self.call(self.ik, ik)
        if answer.error_code.val != 1:
            return "NOIK", None
        request = GetStateValidity.Request()
        request.robot_state = answer.solution
        request.robot_state.is_diff = True
        validity = self.call(self.valid, request)
        pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in validity.contacts
                        if "demo_place_cup" not in (c.contact_body_1, c.contact_body_2)})
        if pairs:
            return "COLL " + ",".join(pairs), None
        return "OK", answer.solution

    def line(self, arm, start, goal, seed, step=0.02, angle_step=math.radians(8)):
        """Walk a LIN move (lerp + slerp) like Pilz; return (status, state at goal)."""
        (a, qa), (b, qb) = start, goal
        dot = min(1.0, abs(sum(x * y for x, y in zip(qa, qb))))
        count = max(1, math.ceil(math.dist(a, b) / step), math.ceil(2 * math.acos(dot) / angle_step))
        state = seed
        for i in range(1, count + 1):
            t = i / count
            point = tuple(p + t * (q - p) for p, q in zip(a, b))
            status, state = self.check(arm, (point, slerp(qa, qb, t)), state)
            if state is None:
                return f"{status} at {t:.2f}", None
        return "OK", state


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cup_x", type=float)
    parser.add_argument("cup_y", type=float)
    parser.add_argument("rim_z", type=float)
    parser.add_argument("--transfer-x", default="0.33,0.38,0.43")
    parser.add_argument("--transfer-y", default="0.0,0.05")
    parser.add_argument("--transfer-bottom-z", default="0.85,0.92")
    parser.add_argument("--left-yaw", type=float, default=-90.0)
    parser.add_argument("--right-height", type=float, default=0.12)
    parser.add_argument("--right-depth", type=float, default=0.0)
    parser.add_argument("--right-pitch", type=float, default=0.0)
    parser.add_argument("--pour-x", help="comma list of cup axis x over the basket")
    parser.add_argument("--pour-y", help="comma list of cup axis y over the basket")
    parser.add_argument("--pour-bottom-z", type=float)
    parser.add_argument("--pour-max-deg", type=float, default=150.0)
    args = parser.parse_args()
    numbers = lambda text: [float(v) for v in text.split(",")]

    rclpy.init()
    node = Checker()
    try:
        scene = node.read_scene()
        start = scene.robot_state
        bottom = args.rim_z - CUP_HEIGHT
        cup = (args.cup_x, args.cup_y)
        left = dict(pitch=math.radians(15), height=0.075, depth=0.025)
        right = dict(pitch=math.radians(args.right_pitch), height=args.right_height,
                     depth=args.right_depth)
        print("world objects:", [o.id for o in scene.world.collision_objects])

        # Pour point: explicit, else above basket_floor (cup axis 6 cm right of its center).
        pours = []
        floor = next((o for o in scene.world.collision_objects if o.id == "basket_floor"), None)
        if args.pour_x and args.pour_y:
            pours = list(itertools.product(numbers(args.pour_x), numbers(args.pour_y)))
        elif floor is not None:
            p = floor.pose.position
            print(f"basket_floor at {p.x:.3f} {p.y:.3f} {p.z:.3f}")
            pours = [(p.x, p.y - 0.06)]
        pour_bottom = args.pour_bottom_z
        if pours and pour_bottom is None:
            pour_bottom = (floor.pose.position.z if floor is not None else bottom) + 0.20
        if not pours:
            print("no basket_floor in the scene and no --pour-x/--pour-y: pour checks skipped")
        else:
            print(f"pour points {pours}, cup bottom z {pour_bottom:.3f}")

        # Pick chain from the current pose, to seed the left arm like the real motion.
        grasp_table = grasp(cup, bottom, 0.0, **left)
        insert0 = grasp(cup, bottom, 0.0, left["pitch"], left["height"], 0.0)
        pregrasp = shifted(insert0, -0.12, 0.0)
        status, state = node.check("left", pregrasp, start)
        if state is not None:
            status, state = node.line("left", pregrasp, grasp_table, state)
        lifted = shifted(grasp_table, 0.0, 0.0, 0.08)
        if state is not None:
            status, state = node.line("left", grasp_table, lifted, state)
        print("pick chain (pregrasp, insert, lift):", status)
        if state is None:
            return
        held = state

        yaw_l = math.radians(args.left_yaw)
        yaw_r = yaw_l + math.pi  # Hands face each other.
        back_l = (-math.cos(yaw_l), -math.sin(yaw_l))
        back_r = (-math.cos(yaw_r), -math.sin(yaw_r))
        print("\ntransfer x     y  bottom | L carry | R pre | R insert | L retreat | R to pour | pour roll")
        for tx, ty, tz in itertools.product(numbers(args.transfer_x), numbers(args.transfer_y),
                                            numbers(args.transfer_bottom_z)):
            row = f"      {tx:5.2f} {ty:5.2f}  {tz:5.2f} |"
            results, state = [], held
            left_hold = grasp((tx, ty), tz, yaw_l, **left)
            right_hold = grasp((tx, ty), tz, yaw_r, **right)
            right_pre = shifted(grasp((tx, ty), tz, yaw_r, right["pitch"], right["height"], 0.0),
                                0.12 * back_r[0], 0.12 * back_r[1])
            left_out = shifted(left_hold, 0.12 * back_l[0], 0.12 * back_l[1])

            def run(label, fn):
                nonlocal state
                if state is None:
                    results.append("-")
                    return
                status, state = fn(state)
                results.append(status)

            run("L carry", lambda s: node.line("left", lifted, left_hold, s))
            run("R pre", lambda s: node.check("right", right_pre, s))
            run("R insert", lambda s: node.line("right", right_pre, right_hold, s))
            run("L retreat", lambda s: node.line("left", left_hold, left_out, s))
            print(row, " | ".join(results))
            after_retreat = state
            for pour in pours:
                over = grasp(pour, pour_bottom, 0.0, **right)
                line = f"   pour x {pour[0]:.2f} y {pour[1]:.2f} :"
                status, local = (("-", None) if after_retreat is None else
                                 node.line("right", right_hold, over, after_retreat))
                line += f" R to pour {status} |"
                if local is not None:
                    # Roll in place about hand forward; report the largest reachable angle.
                    reached = 0
                    for deg in range(10, int(args.pour_max_deg) + 1, 10):
                        pose = (over[0], hand(0.0, right["pitch"], math.radians(deg)))
                        status, local = node.check("right", pose, local)
                        if local is None:
                            line += f" roll {reached}deg then {status}"
                            break
                        reached = deg
                    else:
                        line += f" roll {reached}deg OK"
                print(line)
            continue
            print(row, " | ".join(results))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

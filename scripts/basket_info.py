#!/usr/bin/env python3
"""Read-only: print the pose and size of every basket and table object in the planning scene.

Needs full_demo running and the basket registered (run a DYNAMIXEL pick, menu 2, or d, first).
Primitive poses are relative to the object pose; both are composed into base_link here.
Usage: python3 basket_info.py
"""
import math

import rclpy
from cup_layout_check import Checker, quat_mul
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene


def rotate(q, v):
    w, x, y, z = q
    t = quat_mul(quat_mul(q, (0.0, *v)), (w, -x, -y, -z))
    return t[1:]


def compose(parent, child):
    """World pose of child given parent pose; poses are ((x,y,z), (w,x,y,z))."""
    (pp, pq), (cp, cq) = parent, child
    offset = rotate(pq, cp)
    return tuple(a + b for a, b in zip(pp, offset)), quat_mul(pq, cq)


def as_tuple(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)


def main():
    rclpy.init()
    node = Checker()
    try:
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        objects = node.call(node.scene, request).scene.world.collision_objects
        found = False
        for obj in objects:
            if not obj.id.startswith(("basket", "demo_table", "demo_pick_table")):
                continue
            found = found or obj.id.startswith("basket")
            for shape, local in zip(obj.primitives, obj.primitive_poses):
                (x, y, z), (w, qx, qy, qz) = compose(as_tuple(obj.pose), as_tuple(local))
                yaw = math.degrees(math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
                print(f"{obj.id:18s} frame {obj.header.frame_id} xyz {x:.3f} {y:.3f} {z:.3f} "
                      f"yaw {yaw:6.1f} deg size {[round(v, 3) for v in shape.dimensions]}")
        if not found:
            print("no basket objects in the scene: run a DYNAMIXEL pick first")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

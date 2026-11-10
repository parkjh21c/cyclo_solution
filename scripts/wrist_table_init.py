#!/usr/bin/env python3
"""Table front edge from both wrist depths at the initial pose -> one fixed MoveIt box.

Height is configured (base_link height does not change); only the edge x and yaw are measured.
Measured once and kept; ~/remeasure measures again. Never commands motion.
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from table_geometry import depth_points, front_edge, strip_level, table_box, table_level
from wrist_cup_geometry import rotation_matrix

SIDES = ("left", "right")


class WristTableInit(Node):
    def __init__(self):
        super().__init__("wrist_table_init")
        p = self.declare_parameter
        self.frame = p("planning_frame", "base_link").value
        self.object_id = p("object_id", "demo_table").value
        self.table_z = p("table_z", 0.720).value  # tape-measured top in base_link
        self.top_margin = p("top_margin", 0.008).value  # box top this much below table_z
        self.edge_margin = p("edge_margin", 0.005).value  # box front this much toward the robot
        self.size = p("table_size", [0.8, 1.2]).value  # depth, width
        self.thickness = p("thickness", 0.04).value
        self.search = p("level_search", [0.69, 0.78]).value  # z band where the level is looked for
        self.frames = p("frames", 5).value  # depth frames per camera
        self.near = p("depth_min_m", 0.07).value
        self.far = p("depth_max_m", 1.0).value
        self.dump = p("dump_path", "").value  # npz of the points of each attempt (debug)
        initial = p("initial_joints", [1.05, 0.0, 0.0, -2.27, 0.0, 0.35, 0.0]).value
        self.joint_tolerance = p("joint_tolerance", 0.03).value
        if not (0 <= self.top_margin <= 0.03 and 0 <= self.edge_margin <= 0.03
                and len(self.size) == 2 and min(self.size) > 0 and self.thickness > 0
                and len(initial) == 7 and self.frames >= 1 and self.search[0] < self.search[1]):
            raise ValueError("Invalid wrist table parameters")
        self.initial = {f"arm_{s[0]}_joint{i + 1}": v for s in SIDES for i, v in enumerate(initial)}
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.info = {}
        self.clouds = {s: [] for s in SIDES}
        self.joints = None
        self.fixed = False
        self.last_status = ""
        self.pending_text = ""
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, "~/status", latched)
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.create_service(Trigger, "~/remeasure", self.remeasure)
        self.create_subscription(JointState, "/joint_states", self.joint_state, 10)
        depth_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in SIDES:
            base = f"/camera_{side}/camera_{side}/depth"
            self.create_subscription(Image, p(f"{side}_depth_topic", base + "/image_rect_raw").value,
                                     lambda m, s=side: self.depth(s, m), depth_qos)
            self.create_subscription(CameraInfo, p(f"{side}_info_topic", base + "/camera_info").value,
                                     lambda m, s=side: self.info.__setitem__(s, m),
                                     qos_profile_sensor_data)
        self.create_timer(0.5, self.process)
        self.report("WAITING: initial pose and both wrist depths")

    def report(self, text, force=False):
        if force or text != self.last_status:
            self.last_status = text
            self.status.publish(String(data=text))
            self.get_logger().info(text)

    def joint_state(self, message):
        self.joints = (self.get_clock().now(), dict(zip(message.name, message.position)))

    def pose_problem(self):
        if self.joints is None:
            return "NO_JOINT_STATES"
        stamp, positions = self.joints
        if (self.get_clock().now() - stamp).nanoseconds > 5e8:
            return "JOINT_STATES_STALE"
        if any(n not in positions for n in self.initial):
            return "ARM_JOINTS_MISSING"
        offset = max(abs(positions[n] - v) for n, v in self.initial.items())
        return None if offset <= self.joint_tolerance else f"NOT_INITIAL_POSE: {offset:.3f}rad"

    def depth(self, side, message):
        if self.fixed or len(self.clouds[side]) >= self.frames or self.pose_problem():
            return
        info = self.info.get(side)
        if (info is None or info.header.frame_id != message.header.frame_id
                or (info.height, info.width) != (message.height, message.width)
                or info.p[0] <= 0 or message.encoding.lower() not in ("16uc1", "32fc1")):
            return
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        try:
            t = self.buffer.lookup_transform(self.frame, message.header.frame_id, stamp)
        except TransformException:
            return  # TF trails the image; a later frame will do
        meters = self.bridge.imgmsg_to_cv2(message).astype(np.float32)
        if message.encoding.lower() == "16uc1":
            meters *= 0.001
        keep = np.isfinite(meters) & (meters >= self.near) & (meters <= self.far)
        q, tr = t.transform.rotation, t.transform.translation
        self.clouds[side].append(depth_points(
            meters, keep, (info.p[0], info.p[5], info.p[2], info.p[6]),
            rotation_matrix([q.x, q.y, q.z, q.w]), [tr.x, tr.y, tr.z]))

    def process(self):
        if self.fixed:
            return
        problem = self.pose_problem()
        if problem:
            for side in SIDES:
                self.clouds[side].clear()  # frames must all come from the initial pose
            self.report(f"WAITING: {problem}")
            return
        if any(len(self.clouds[s]) < self.frames for s in SIDES):
            self.report("MEASURING: collecting wrist depth frames")
            return
        clouds = {s: np.concatenate(self.clouds[s]) for s in SIDES}
        for side in SIDES:
            self.clouds[side].clear()
        if self.dump:
            np.savez_compressed(self.dump, **clouds)
        result, why = self.measure(clouds)
        if result is None:
            self.report(f"TABLE_FAILED: {why}; retrying", force=True)
            return
        if not self.apply(result["box"]):
            self.report("TABLE_FAILED: /apply_planning_scene unavailable; retrying", force=True)
            return
        self.fixed = True  # stop measuring; the reply confirms or undoes it
        self.pending_text = result["text"]

    def measure(self, clouds):
        levels = {}
        for side in SIDES:
            level, why = table_level(clouds[side][:, 2], self.search)
            if level is None:
                return None, f"{side} wrist: {why}"
            levels[side] = level
        # each camera's own level selects its table points (their height offsets differ)
        tagged = np.concatenate([clouds[s] - [0.0, 0.0, levels[s] - levels["left"]] for s in SIDES])
        edge, why = front_edge(tagged, levels["left"])
        if edge is None:
            return None, why
        offsets = []
        for side in SIDES:
            strip = strip_level(clouds[side], edge, levels[side])
            level = strip if strip is not None else levels[side]
            offsets.append(f"{side} {level:.3f} ({(level - self.table_z) * 1000:+.0f}mm"
                           f"{'' if strip is not None else ', no edge strip'})")
        top = self.table_z - self.top_margin
        box = table_box(edge, top, self.size[0], self.size[1], self.thickness, self.edge_margin)
        text = (f"TABLE_FIXED: edge x={edge['a']:.3f} yaw={math.degrees(edge['yaw']):+.1f}deg"
                f" bins={edge['bins']}/{edge['total']} span={edge['span']:.2f}m resid={edge['resid'] * 1000:.0f}mm;"
                f" box top {top:.3f}; wrist table level vs {self.table_z:.3f}: "
                + ", ".join(offsets))
        return {"box": box, "text": text}, None

    def apply(self, box):
        if not self.scene.service_is_ready():
            return False
        obj = CollisionObject(id=self.object_id, operation=CollisionObject.ADD)
        obj.header.frame_id = self.frame
        obj.pose.orientation.w = 1.0
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(box["size"]))]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = box["center"]
        pose.orientation.z = math.sin(box["yaw"] / 2)
        pose.orientation.w = math.cos(box["yaw"] / 2)
        obj.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        future = self.scene.call_async(ApplyPlanningScene.Request(scene=scene))
        future.add_done_callback(self.applied)
        return True

    def applied(self, future):
        result = future.result()
        if result is None or not result.success:
            self.fixed = False
            self.report("TABLE_FAILED: move_group refused the table box; retrying", force=True)
        else:
            self.report(self.pending_text, force=True)

    def remeasure(self, request, response):
        self.fixed = False
        for side in SIDES:
            self.clouds[side].clear()
        response.success = True
        response.message = "Table will be measured again at the initial pose"
        self.report("WAITING: remeasure requested", force=True)
        return response


def main():
    rclpy.init()
    node = WristTableInit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

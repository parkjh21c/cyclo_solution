#!/usr/bin/env python3
"""Right wrist basket location at the initial pose; publishes it and moves the MoveIt basket.

Same rim fit (ffw_grasp fit_basket), parameter files, camera bias and scene boxes as the
DYNAMIXEL pick, so an empty basket is measured too. Updates only while the right arm is still
at the menu-1 pose (initial_pose_check). Never commands motion.
"""
import math
from dataclasses import fields, replace

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from ffw_grasp.grasp_planner import GraspParams, fit_basket, matrix_to_quat, merge_planes
from ffw_pick.config import ros_parameters
from ffw_pick.pick_moveit import basket_boxes
from ffw_vision.depth_stats import color_from_msg, depth_from_msg
from ffw_vision.input_check import stamp_to_ns
from ffw_vision.seg_detector import pixel_rays
from ffw_vision_msgs.msg import ObjectObservationArray
from geometry_msgs.msg import Pose, PoseStamped
from initial_pose_check import InitialPoseCheck
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

BUFFER_SIZE = 30
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def config_file(package, name):
    return f"{get_package_share_directory(package)}/config/{name}"


def grasp_params(values):
    """GraspParams from merged ros__parameters (lists become tuples where the default is one)."""
    kept = {}
    for field in fields(GraspParams):
        if field.name in values:
            value = values[field.name]
            kept[field.name] = tuple(value) if isinstance(field.default, tuple) else value
    return replace(GraspParams(), **kept)


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def plane_rotation(z, x):
    """Columns x, y, z with z up the plane normal and x along x (as ffw_grasp's planner)."""
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def pose(position, rotation):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, position)
    o = p.orientation
    o.x, o.y, o.z, o.w = matrix_to_quat(rotation)
    return p


class WristBasketInit(Node):
    def __init__(self):
        super().__init__("wrist_basket_init")
        self.frame = self.declare_parameter("planning_frame", "base_link").value
        prefix = "/camera_right/camera_right/"
        color_topic = self.declare_parameter("color_topic", prefix + "color/image_rect_raw").value
        depth_topic = self.declare_parameter(
            "depth_topic", prefix + "aligned_depth_to_color/image_raw").value
        info_topic = self.declare_parameter("info_topic", prefix + "color/camera_info").value
        observation_topic = self.declare_parameter(
            "observation_topic", "/ffw_vision/right_wrist/object_observations").value
        self.depth_scale = self.declare_parameter("depth_scale", 0.001).value
        self.frames = self.declare_parameter("frames", 5).value
        self.min_move = self.declare_parameter("min_scene_move", 0.005).value  # m
        self.min_turn = math.radians(self.declare_parameter("min_scene_turn_deg", 1.0).value)
        files = [config_file("ffw_vision", "blue_basket.yaml"),
                 config_file("ffw_grasp", "dynamixel_top_grasp.yaml"),
                 config_file("ffw_grasp", "robot_calibration.yaml")]  # Same order as the planner.
        self.task = {}
        for path in files:
            self.task.update(ros_parameters(path))
        self.params = grasp_params(self.task)
        self.bias = np.array([self.params.camera_x_bias, self.params.camera_y_bias,
                              self.params.camera_z_bias])
        self.check = InitialPoseCheck(
            "right", joint_tolerance=self.declare_parameter("joint_tolerance", 0.02).value,
            lift_tolerance=self.declare_parameter("lift_tolerance", 0.01).value)
        self.colors, self.depths = {}, {}
        self.info, self.rays = None, None
        self.window = []  # (observation, color, depth)
        self.applied = None  # (position, rotation) last sent to MoveIt
        self.scene_future = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, "/demo/basket_pose", LATCHED)
        self.status = self.create_publisher(String, "/wrist_basket_init/status", 1)
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.create_subscription(Image, color_topic, lambda m: self.store(self.colors, m),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, lambda m: self.store(self.depths, m),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.camera_info, qos_profile_sensor_data)
        self.create_subscription(ObjectObservationArray, observation_topic, self.observation, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state, 10)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def store(self, buffer, message):
        buffer[stamp_to_ns(message.header.stamp)] = message
        if len(buffer) > BUFFER_SIZE:
            del buffer[min(buffer)]

    def camera_info(self, message):
        self.info = message

    def joint_state(self, message):
        self.check.observe(self.now(), message.name, message.position)

    def report(self, text):
        self.status.publish(String(data=text))

    def observation(self, message):
        reason = self.check.reason(self.now())
        if reason is not None:
            self.window.clear()
            self.report(reason)
            return
        stamp = stamp_to_ns(message.header.stamp)
        color, depth = self.colors.pop(stamp, None), self.depths.pop(stamp, None)
        if not message.has_support_plane or color is None or depth is None or self.info is None:
            return  # No table plane or no matching images for this frame.
        self.window.append((message, color, depth))
        if len(self.window) >= self.frames:
            frames, self.window = self.window, []
            self.measure(frames)

    def measure(self, frames):
        info = self.info
        key = (info.width, info.height, tuple(info.k), tuple(info.d))
        if self.rays is None or self.rays[0] != key:
            self.rays = (key, pixel_rays(info.width, info.height, info.k, info.d))
        try:
            depth = np.nanmedian(np.stack(
                [depth_from_msg(d, self.depth_scale) for _, _, d in frames]), axis=0)
            color = color_from_msg(frames[-1][1])
        except ValueError as error:
            self.report(f"IMAGE_INVALID: {error}")
            return
        plane = merge_planes([list(o.support_plane) for o, _, _ in frames])
        basket, reason = fit_basket(color, depth, info.k, info.d, self.rays[1], plane, self.params)
        if basket is None:
            self.report(f"BASKET_NOT_FOUND: {reason}")
            return
        header = frames[-1][0].header
        try:
            tf = self.buffer.lookup_transform(self.frame, header.frame_id, Time.from_msg(header.stamp))
        except Exception as error:  # TF may trail the image; the next window tries again.
            self.report(f"TF_UNAVAILABLE: {error}")
            return
        rotation = quaternion_matrix(tf.transform.rotation)
        t = tf.transform.translation
        translation = np.array([t.x, t.y, t.z])
        # Same pose as ffw_grasp publishes: outer bottom center on the table, x along the long side.
        position = rotation @ basket.center + translation - self.bias
        frame = plane_rotation(rotation @ plane[0], rotation @ basket.axis)
        # The rim fit picks either end of the long axis; keep yaw in [-45, 135) deg (a basket
        # along the robot's y stays at +90) so the scene and the pour see one direction.
        if not -45.0 <= math.degrees(math.atan2(frame[1, 0], frame[0, 0])) < 135.0:
            frame[:, :2] *= -1.0  # 180 deg about the plane normal.
        message = PoseStamped()
        message.header.frame_id = self.frame
        message.header.stamp = header.stamp
        message.pose = pose(position, frame)
        self.pose_pub.publish(message)
        yaw = math.degrees(math.atan2(frame[1, 0], frame[0, 0]))
        cut = f" cut {basket.cut}" if basket.cut else ""
        self.report(f"BASKET {position[0]:.3f} {position[1]:.3f} {position[2]:.3f} yaw {yaw:.1f}{cut}")
        self.update_scene(position, frame)

    def update_scene(self, position, frame):
        if self.applied is not None:
            moved = np.linalg.norm(position - self.applied[0])
            turned = math.acos(np.clip((np.trace(self.applied[1].T @ frame) - 1) / 2, -1, 1))
            if moved < self.min_move and turned < self.min_turn:
                return
        if (self.scene_future is not None and not self.scene_future.done()) or \
                not self.scene.service_is_ready():
            return  # The next measurement retries.
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        for name, dims, center, rotation in basket_boxes(self.task):
            obj = CollisionObject(id=f"basket_{name}", operation=CollisionObject.ADD)
            obj.header.frame_id = self.frame
            obj.pose = pose(position + frame @ center, frame @ rotation)
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                             dimensions=[float(d) for d in dims])]
            obj.primitive_poses = [pose(np.zeros(3), np.eye(3))]
            scene.world.collision_objects.append(obj)
        self.scene_future = self.scene.call_async(ApplyPlanningScene.Request(scene=scene))
        self.scene_future.add_done_callback(lambda f: self.scene_done(f, position, frame))

    def scene_done(self, future, position, frame):
        result = future.result()
        if result is not None and result.success:
            self.applied = (position, frame)  # Only a confirmed update stops the retries.
        else:
            self.report("SCENE_UPDATE_FAILED")


def main():
    rclpy.init()
    node = WristBasketInit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

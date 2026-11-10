#!/usr/bin/env python3
"""Left wrist cup location at the initial pose. Publishes observations, never commands."""

from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from demo_1f.msg import CupObservation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from wrist_cup_geometry import rotation_matrix, table_cup_rim

ARM_JOINTS = [f"arm_l_joint{i}" for i in range(1, 8)]


class WristCupInit(Node):
    def __init__(self):
        super().__init__("wrist_cup_init")
        self.frame = self.declare_parameter("planning_frame", "base_link").value
        self.cup_id = self.declare_parameter("cup_id", "cup").value
        self.inner = self.declare_parameter("inner_radius", 0.045).value
        self.wall = self.declare_parameter("wall_thickness", 0.005).value
        self.cup_height = self.declare_parameter("cup_height", 0.165).value
        self.table_z = self.declare_parameter("table_z", 0.725).value
        self.workspace = self.declare_parameter("workspace", [0.25, 0.9, -0.1, 0.6]).value
        self.near = self.declare_parameter("depth_min_m", 0.15).value
        self.far = self.declare_parameter("depth_max_m", 0.8).value
        # BODY uncertainty must cover the cup radius for the head-compatible checks.
        self.error = self.declare_parameter("position_error", 0.06).value
        # config/demo.yaml initial_pose, left arm.
        self.initial = self.declare_parameter(
            "initial_joints", [1.05, 0.0, 0.0, -2.27, 0.0, 0.35, 0.0]
        ).value
        self.joint_tolerance = self.declare_parameter("joint_tolerance", 0.03).value
        self.period = self.declare_parameter("period", 0.2).value
        if not (
            0 < self.near < self.far
            and len(self.workspace) == 4
            and self.workspace[0] < self.workspace[1]
            and self.workspace[2] < self.workspace[3]
            and len(self.initial) == len(ARM_JOINTS)
            and 0 < self.joint_tolerance <= 0.1
            and 0.05 <= self.cup_height <= 0.4
            and self.inner + self.wall <= self.error <= 0.15
        ):
            raise ValueError("Invalid wrist cup initial-pose parameters")
        self.bridge = CvBridge()
        self.info = None
        self.joints = None
        self.pending = deque(maxlen=8)
        self.tf_wait = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.history = deque(maxlen=3)
        self.observations = self.create_publisher(CupObservation, "/perception/wrist_cup", 5)
        self.images = self.create_publisher(Image, "/wrist_cup_init/image", qos_profile_sensor_data)
        self.status = self.create_publisher(String, "/wrist_cup_init/status", 1)
        topic = self.declare_parameter(
            "depth_topic", "/camera_left/camera_left/depth/image_rect_raw"
        ).value
        info_topic = self.declare_parameter(
            "info_topic", "/camera_left/camera_left/depth/camera_info"
        ).value
        self.create_subscription(
            Image, topic, self.image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(CameraInfo, info_topic, self.camera_info, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.joint_state, 10)
        self.create_timer(self.period, self.process)

    def camera_info(self, message):
        self.info = message

    def image(self, message):
        self.pending.append(message)

    def joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in ARM_JOINTS):
            self.joints = (self.get_clock().now(), [positions[name] for name in ARM_JOINTS])

    def at_initial_pose(self):
        if self.joints is None:
            return "NO_JOINT_STATES"
        stamp, positions = self.joints
        if (self.get_clock().now() - stamp).nanoseconds / 1e9 > 0.5:
            return "JOINT_STATES_STALE"
        offset = np.max(np.abs(np.subtract(positions, self.initial)))
        return None if offset <= self.joint_tolerance else f"NOT_INITIAL_POSE: {offset:.3f}rad"

    def publish(self, message, detail, frame=None, fit=None):
        observation = CupObservation()
        observation.header = message.header
        observation.header.frame_id = self.frame
        observation.source = "wrist_left_init"
        observation.cup_id = self.cup_id
        observation.observation_type = CupObservation.BODY
        observation.inner_radius = self.inner
        observation.detail = detail
        if fit is not None:
            observation.valid = True
            # BODY Z is the cup middle, as the approach expects.
            x, y, rim_z = [float(v) for v in fit["center"]]
            observation.body_position.x = x
            observation.body_position.y = y
            observation.body_position.z = rim_z - self.cup_height / 2
            observation.position_error = float(self.error)
            observation.confidence = min(1.0, fit["coverage_deg"] / 360.0)
        self.observations.publish(observation)
        self.status.publish(String(data=detail))
        if frame is not None:
            color = (0, 220, 0) if fit is not None else (0, 80, 220)
            cv2.putText(frame, detail[:48], (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
            image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            image.header = message.header
            self.images.publish(image)

    def process(self):
        # Newest frame whose capture-time TF has arrived; TF often trails the image.
        message, transform = None, None
        now = self.get_clock().now()
        for candidate in reversed(self.pending):
            stamp = rclpy.time.Time.from_msg(candidate.header.stamp)
            age = (now - stamp).nanoseconds / 1e9
            if age < -0.1 or age > 0.5:
                continue
            if self.buffer.can_transform(self.frame, candidate.header.frame_id, stamp):
                message = candidate
                transform = self.buffer.lookup_transform(
                    self.frame, candidate.header.frame_id, stamp
                )
                break
        if message is None:
            # Status only after 1 s; the gap check resets stability on a real outage.
            if self.pending:
                self.tf_wait = self.tf_wait or now
                if (now - self.tf_wait).nanoseconds > 1e9:
                    self.status.publish(String(data="TF_UNAVAILABLE"))
            return
        self.tf_wait = None
        self.pending.clear()
        pose = self.at_initial_pose()
        if pose is not None:
            self.history.clear()
            self.publish(message, pose)
            return
        info = self.info
        if (
            info is None
            or info.header.frame_id != message.header.frame_id
            or (info.height, info.width) != (message.height, message.width)
            or info.p[0] <= 0
            or info.p[5] <= 0
        ):
            self.history.clear()
            self.publish(message, "CAMERA_INFO_INVALID")
            return
        try:
            self.detect(message, info, transform)
        except (ValueError, cv2.error) as error:
            self.history.clear()
            self.publish(message, f"DEPTH_INVALID: {error}")

    def detect(self, message, info, transform):
        if message.encoding.lower() not in ("16uc1", "32fc1"):
            raise ValueError("Numeric 16UC1/32FC1 depth required")
        meters = self.bridge.imgmsg_to_cv2(message).astype(np.float32)
        if message.encoding.lower() == "16uc1":
            meters *= 0.001
        keep = np.isfinite(meters) & (meters >= self.near) & (meters <= self.far)
        q, t = transform.transform.rotation, transform.transform.translation
        rotation = rotation_matrix([q.x, q.y, q.z, q.w])
        translation = np.array([t.x, t.y, t.z])
        fit, detail, regions = table_cup_rim(
            meters,
            keep,
            (info.p[0], info.p[5], info.p[2], info.p[6]),
            rotation,
            translation,
            self.table_z,
            self.cup_height,
            self.workspace,
            self.inner,
            self.wall,
        )
        display = np.nan_to_num(np.clip((self.far - meters) / (self.far - self.near), 0, 1)) * 255
        display[~keep] = 0
        frame = cv2.cvtColor(display.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        color = (0, 220, 0) if fit is not None else (0, 80, 220)
        for region in regions:
            contours, _ = cv2.findContours(
                region["mask"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(frame, contours, -1, color, 1)
        if fit is None:
            self.history.clear()
            self.publish(message, detail, frame)
            return

        def pixels(world_points):
            camera = (world_points - translation) @ rotation
            camera = camera[camera[:, 2] > 0.03]
            return np.column_stack(
                (
                    camera[:, 0] / camera[:, 2] * info.p[0] + info.p[2],
                    camera[:, 1] / camera[:, 2] * info.p[5] + info.p[6],
                )
            ).astype(int)

        # Blue: rim band points; magenta: fitted rim circle and center.
        for point in pixels(fit["points"])[::3]:
            cv2.circle(frame, tuple(point), 1, (255, 120, 0), -1)
        angles = np.linspace(0, 2 * np.pi, 100)
        ring = np.tile(fit["center"], (100, 1))
        ring[:, 0] += fit["radius"] * np.cos(angles)
        ring[:, 1] += fit["radius"] * np.sin(angles)
        projected = pixels(ring)
        if len(projected) == 100:
            cv2.polylines(frame, [projected], True, (255, 0, 255), 1)
        center_pixel = pixels(fit["center"][None, :])
        if len(center_pixel):
            cv2.drawMarker(frame, tuple(center_pixel[0]), (255, 0, 255), cv2.MARKER_CROSS, 15, 2)
        stamp = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds
        if self.history and (
            stamp - self.history[-1][0] > 1_000_000_000
            or max(np.linalg.norm(fit["center"] - p) for _, p in self.history) > 0.008
        ):
            self.history.clear()
        self.history.append((stamp, fit["center"].copy()))
        if len(self.history) < 3:
            self.publish(message, f"RIM_UNSTABLE: {len(self.history)}/3", frame)
            return
        fit["center"] = np.mean([p for _, p in self.history], axis=0)
        detail = (
            f"VALID_INIT: rim XYZ={np.round(fit['center'], 4).tolist()} "
            f"radius={fit['radius']:.3f}m residual={fit['residual'] * 1000:.1f}mm "
            f"arc={fit['coverage_deg']:.0f}deg stable=3/3; no motion command"
        )
        self.publish(message, detail, frame, fit)


def main():
    rclpy.init()
    node = WristCupInit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

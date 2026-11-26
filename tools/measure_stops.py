#!/usr/bin/env python3
"""Read-only: log every interval in which both arms (and the lift) stand still, with the demo
stage/phase around it. Sends nothing to the robot.

In the demo container with full_demo running: start this, run d, press Ctrl+C after it ends.
  python3 measure_stops.py [--min 0.15] [--speed 0.02] [--csv /tmp/stops.csv]
"""
import argparse
import csv
import math
import time

ARMS = [f'arm_{s}_joint{i}' for s in 'lr' for i in range(1, 8)]
GRIPPERS = ['gripper_l_joint1', 'gripper_r_joint1']


class StopTracker:
    """Stop intervals from (time, {joint: speed}) samples; no ROS here."""

    def __init__(self, moving_joints, gripper_joints, speed, min_stop, gripper_travel=0.01):
        self.moving_joints, self.gripper_joints = moving_joints, gripper_joints
        self.speed, self.min_stop = speed, min_stop
        self.gripper_travel = gripper_travel  # rad: a gripper moved this far = it moved
        self.stopped_since = None  # None = moving, or not moved yet
        self.started = None  # first motion
        self.last_motion = None
        self.gripper_moved = False
        self.gripper_start = {}
        self.context_at_stop = ''
        self.stops = []  # (start s, duration s, before, after, gripper moved)

    def sample(self, t, speeds, context, positions=None):
        """Returns the stop that just ended (if long enough) or None."""
        positions = positions or {}
        moving = any(abs(speeds.get(n, 0.0)) > self.speed for n in self.moving_joints)
        if moving:
            ended = None
            if self.started is None:
                self.started = t
            elif self.stopped_since is not None:
                duration = t - self.stopped_since
                if duration >= self.min_stop:
                    ended = (self.stopped_since - self.started, duration, self.context_at_stop,
                             context, self.gripper_moved)
                    self.stops.append(ended)
            self.stopped_since = None
            self.last_motion = t
            return ended
        if self.started is not None and self.stopped_since is None:
            self.stopped_since, self.gripper_moved, self.context_at_stop = t, False, context
            self.gripper_start = {n: positions[n] for n in self.gripper_joints if n in positions}
        # position travel, not velocity: gripper velocities are noisy at rest
        self.gripper_moved |= any(abs(positions[n] - p) > self.gripper_travel
                                  for n, p in self.gripper_start.items() if n in positions)
        return None


def summary(tracker):
    if tracker.started is None:
        return ['no arm motion seen']
    span = tracker.last_motion - tracker.started
    stopped = sum(s[1] for s in tracker.stops)
    lines = [f'first motion -> last motion {span:.1f} s: stopped {stopped:.1f} s in'
             f' {len(tracker.stops)} stops (>= {tracker.min_stop:.2f} s), moving {span - stopped:.1f} s']
    by_stage = {}
    for _, duration, before, _, _ in tracker.stops:
        stage = before.split(' / ')[0] or '?'
        total, count = by_stage.get(stage, (0.0, 0))
        by_stage[stage] = (total + duration, count + 1)
    lines.append('stopped time by stage (stage when the stop began):')
    for stage, (total, count) in sorted(by_stage.items(), key=lambda kv: -kv[1][0]):
        lines.append(f'  {total:6.2f} s  {count:3d}x  {stage}')
    lines.append('longest stops:')
    for start, duration, before, after, gripper in sorted(tracker.stops, key=lambda s: -s[1])[:15]:
        lines.append(f'  {duration:5.2f} s at {start:6.1f} s  {before} -> {after}'
                     + ('  [gripper moved]' if gripper else ''))
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--min', type=float, default=0.15, help='shortest stop reported, s')
    parser.add_argument('--speed', type=float, default=0.02, help='rad/s (m/s) = still below this')
    parser.add_argument('--lift', default='lift_joint', help="lift joint name ('' = ignore)")
    parser.add_argument('--csv', default='', help='also write every stop here')
    args = parser.parse_args()

    import rclpy
    from demo_1f.action import RunDemo
    from demo_1f.msg import DemoStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    moving_joints = ARMS + ([args.lift] if args.lift else [])
    tracker = StopTracker(moving_joints, GRIPPERS, args.speed, args.min)
    context = {'stage': '', 'phase': '', 'step': ''}
    previous = {}  # joint: (t, position) for messages without velocities

    def label():
        return ' / '.join(v for v in (context['stage'], context['step'] or context['phase']) if v)

    def on_joints(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or time.monotonic()
        speeds = {}
        has_velocity = len(msg.velocity) == len(msg.name)
        for i, name in enumerate(msg.name):
            if has_velocity and math.isfinite(msg.velocity[i]):
                speeds[name] = msg.velocity[i]
            else:  # differentiate positions
                last = previous.get(name)
                if last and t > last[0]:
                    speeds[name] = (msg.position[i] - last[1]) / (t - last[0])
                previous[name] = (t, msg.position[i])
        ended = tracker.sample(t, speeds, label(), dict(zip(msg.name, msg.position)))
        if ended:
            start, duration, before, after, gripper = ended
            print(f'{start:7.2f} s  stop {duration:5.2f} s  {before} -> {after}'
                  + ('  [gripper moved]' if gripper else ''), flush=True)

    def on_feedback(msg):
        context['stage'], context['phase'] = msg.feedback.stage, msg.feedback.phase

    def on_status(msg):
        context['step'] = msg.phase

    import signal
    from rclpy.signals import SignalHandlerOptions

    # own Ctrl+C: rclpy's handler can interrupt a message take and lose the summary
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('measure_stops')
    node.create_subscription(JointState, '/joint_states', on_joints, qos_profile_sensor_data)
    node.create_subscription(RunDemo.Impl.FeedbackMessage, '/run_demo/_action/feedback',
                             on_feedback, 10)
    node.create_subscription(DemoStatus, '/handover_demo/status', on_status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    print(f'listening (still = all of {len(moving_joints)} arm/lift joints below'
          f' {args.speed} rad/s); run d, then Ctrl+C', flush=True)
    while not stop['now'] and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print('\n'.join(summary(tracker)))
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['start_s', 'duration_s', 'before', 'after', 'gripper_moved'])
            writer.writerows(tracker.stops)
        print(f'wrote {args.csv}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

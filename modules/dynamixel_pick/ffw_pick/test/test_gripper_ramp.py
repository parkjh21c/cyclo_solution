import math
import unittest

from builtin_interfaces.msg import Duration
from ffw_pick.pick_moveit import add_joint_ramp, ramp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def point(t, positions, velocities=(), accelerations=()):
    p = JointTrajectoryPoint(positions=list(positions), velocities=list(velocities),
                             accelerations=list(accelerations))
    p.time_from_start = Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9)))
    return p


class TestRamp(unittest.TestCase):

    def test_starts_and_ends_at_rest(self):
        q, v, _ = ramp(0.0, 0.0, 0.4, 1.0)
        self.assertAlmostEqual(q, 0.0)
        self.assertAlmostEqual(v, 0.0)
        q, v, a = ramp(1.0, 0.0, 0.4, 1.0)
        self.assertEqual((q, v, a), (0.4, 0.0, 0.0))
        self.assertEqual(ramp(3.0, 0.0, 0.4, 1.0), (0.4, 0.0, 0.0))  # held afterwards

    def test_monotonic_and_velocity_matches_position(self):
        last, dt = -math.inf, 1e-4
        for i in range(100):
            t = i / 100
            q, v, _ = ramp(t, 0.1, 0.5, 1.0)
            self.assertGreaterEqual(q, last)
            last = q
            ahead = ramp(t + dt, 0.1, 0.5, 1.0)[0]
            self.assertAlmostEqual((ahead - q) / dt, v, places=2)

    def test_closing_direction(self):
        self.assertAlmostEqual(ramp(0.5, 0.6, 0.2, 1.0)[0], 0.4)


class TestAddJointRamp(unittest.TestCase):

    def test_adds_the_joint_to_every_point_with_its_own_columns(self):
        jt = JointTrajectory(joint_names=['a', 'b'], points=[
            point(0.0, [0, 0], [0, 0]), point(0.5, [1, 1], [1, 1]), point(2.0, [2, 2], [0, 0])])
        add_joint_ramp(jt, 'g', 0.0, 0.4, 1.0)
        self.assertEqual(list(jt.joint_names), ['a', 'b', 'g'])
        self.assertEqual([len(p.positions) for p in jt.points], [3, 3, 3])
        self.assertEqual([len(p.velocities) for p in jt.points], [3, 3, 3])
        self.assertEqual([len(p.accelerations) for p in jt.points], [0, 0, 0])  # none before
        self.assertAlmostEqual(jt.points[0].positions[2], 0.0)
        self.assertAlmostEqual(jt.points[1].positions[2], 0.2)
        self.assertAlmostEqual(jt.points[2].positions[2], 0.4)
        self.assertEqual(list(jt.points[1].positions[:2]), [1, 1])  # arm columns unchanged


if __name__ == '__main__':
    unittest.main()

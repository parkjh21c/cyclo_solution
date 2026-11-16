import math
import unittest

from ffw_grasp.frame_select import pose_change, select_buffered
import numpy as np

S = 1_000_000_000  # ns per s
MOVE, TURN = 0.002, 0.5


def turn_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


HOME = (np.eye(3), np.array([0.3, -0.2, 1.0]))


def select(stamps, poses=None, request=10 * S, last_end=0, max_age=5.0, now=HOME, frames=10):
    poses = poses or [HOME] * len(stamps)
    return select_buffered(stamps, request, last_end, max_age, lambda i: poses[i], now, frames,
                           MOVE, TURN)


class SelectBufferedTest(unittest.TestCase):

    def test_all_usable(self):
        stamps = [int((6 + 0.4 * i) * S) for i in range(10)]
        self.assertEqual(select(stamps), (list(range(10)), 0))

    def test_newest_frames_only(self):
        stamps = [int((5.5 + 0.3 * i) * S) for i in range(12)]
        self.assertEqual(select(stamps), (list(range(2, 12)), 0))

    def test_only_after_last_plan(self):
        stamps = [7 * S, 8 * S, 9 * S]
        self.assertEqual(select(stamps, last_end=8 * S), ([2], 9))  # equal stamp left out

    def test_age_limit(self):
        stamps = [4 * S, 5 * S, 6 * S, 9 * S]
        self.assertEqual(select(stamps, max_age=5.0), ([1, 2, 3], 7))
        self.assertEqual(select(stamps, max_age=2.0), ([3], 9))

    def test_zero_age_is_off(self):
        self.assertEqual(select([9 * S], max_age=0.0), ([], 10))

    def test_camera_moved(self):
        stamps = [6 * S, 7 * S, 8 * S, 9 * S]
        r, t = HOME
        poses = [(r, t + [0.003, 0, 0]),  # 3 mm away
                 (turn_z(0.6), t),  # 0.6 deg turned
                 (r, t + [0.0015, 0, 0]),  # within tolerance
                 None]  # no TF
        self.assertEqual(select(stamps, poses), ([2], 9))

    def test_no_pose_now(self):
        self.assertEqual(select([9 * S], now=None), ([], 10))

    def test_nothing_kept(self):
        self.assertEqual(select([]), ([], 10))

    def test_pose_change(self):
        moved, turned = pose_change(HOME, (turn_z(2.0), HOME[1] + [0, 0.01, 0]))
        self.assertAlmostEqual(moved, 0.01)
        self.assertAlmostEqual(turned, 2.0, places=6)


if __name__ == '__main__':
    unittest.main()

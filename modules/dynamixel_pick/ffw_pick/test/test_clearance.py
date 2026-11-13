import unittest

from ffw_pick.clearance import (
    box_distance, gripper_parts, line_poses, part_text, path_clearance, sampled_parts, shifted,
    thin, tip_z)
import numpy as np

# dynamixel_top_grasp.yaml + robot_calibration.yaml values the gripper model reads
TASK = {'tip_below_end_effector': 0.0, 'finger_tip': 0.005, 'finger_thick': 0.025,
        'taper_height': 0.030, 'finger_max': 0.030, 'finger_width': 0.015,
        'finger_width_root': 0.025, 'finger_length': 0.100, 'finger_overlap': 0.01,
        'finger_layers': [0.0, 0.01, 0.02, 0.03, 0.045], 'body_half_y': 0.055,
        'body_x_min': -0.042, 'body_x_max': 0.042, 'link_start': 0.155, 'link_half_y': 0.039,
        'link_x_min': -0.060, 'link_x_max': 0.148}
OPENING = 0.05


def pose_at(position):
    m = np.eye(4)
    m[:3, 3] = position
    return m


def box(box_id, center, size, target=False):
    return {'id': box_id, 'target': target, 'center': np.asarray(center, float),
            'rotation': np.eye(3), 'size': np.asarray(size, float)}


class TestGripperParts(unittest.TestCase):

    def test_fingers_body_and_link(self):
        parts = gripper_parts(TASK, OPENING)
        fingers = [p for p in parts if p[0] == 'finger']
        self.assertEqual(len(fingers), 2 * 5)  # layers up to 45 mm, then to finger_length
        self.assertEqual(min(p[1][2] for p in fingers), 0.0)  # the fingertip
        self.assertAlmostEqual(max(p[2][2] for p in fingers), 0.100)
        for _, low, high in fingers:  # inner faces at the opening, thicker going up
            self.assertAlmostEqual(min(abs(low[1]), abs(high[1])), OPENING / 2)
        top = max(fingers, key=lambda p: p[2][2])
        self.assertAlmostEqual(top[2][1] - top[1][1], 0.030)  # finger_max
        body = {p[0]: p for p in parts if p[0] != 'finger'}
        np.testing.assert_allclose(body['body'][1], [-0.042, -0.055, 0.100])
        np.testing.assert_allclose(body['link'][2], [0.148, 0.039, 0.255])


class TestBoxDistance(unittest.TestCase):

    def test_outside_inside_and_corner(self):
        points = np.array([[0.0, 0.0, 0.03], [0.0, 0.0, 0.0], [0.2, 0.2, 0.01]])
        d = box_distance(points, np.zeros(3), np.eye(3), (0.2, 0.2, 0.02))
        np.testing.assert_allclose(d, [0.02, -0.01, np.hypot(0.1, 0.1)], atol=1e-12)


class TestPathClearance(unittest.TestCase):

    def setUp(self):
        self.parts = sampled_parts(gripper_parts(TASK, OPENING))
        # top-down: end effector z (tip -> body) is base z; fingertip 0.3 -> 0.2 m straight down
        self.poses = line_poses(pose_at((0.0, 0.0, 0.3)), np.array([0.0, 0.0, 0.2]))

    def test_gap_below_the_fingertips(self):
        # a 20 mm slab whose top is 20 mm under the lowest fingertip
        result = path_clearance(self.poses, self.parts, [box(2, (0.0, 0.0, 0.17),
                                                             (0.2, 0.2, 0.02))])
        distance, box_id, percent = result['near']
        self.assertAlmostEqual(distance, 0.02, delta=1e-9)
        self.assertEqual((box_id, percent, result['overlaps']), (2, 100, 0))
        self.assertIsNone(result['target'])
        self.assertAlmostEqual(result['lowest'], 0.2, delta=1e-9)

    def test_overlap_and_target_kept_apart(self):
        slab = (0.0, 0.0, 0.205)  # the last poses push the fingertips 5 mm into it
        result = path_clearance(self.poses, self.parts, [box(1, slab, (0.2, 0.2, 0.02))])
        self.assertLessEqual(result['near'][0], 0.0)
        self.assertGreater(result['overlaps'], 0)
        result = path_clearance(self.poses, self.parts,
                                [box(1, slab, (0.2, 0.2, 0.02), target=True)])
        self.assertIsNone(result['near'])
        self.assertEqual(result['overlaps'], 0)
        self.assertLessEqual(result['target'], 0.0)

    def test_small_box_inside_the_link_overlaps(self):
        # no link surface sample is in a 2 mm box at the link's middle: its corners tell
        pose = pose_at((0.0, 0.0, 0.0))
        inside = box(4, (0.044, 0.0, 0.205), (0.002, 0.002, 0.002))
        result = path_clearance([pose], self.parts, [inside])
        self.assertEqual(result['overlaps'], 1)
        self.assertEqual(result['near'][0], 0.0)

    def test_far_box_is_not_measured(self):
        result = path_clearance(self.poses, self.parts, [box(1, (1.0, 0.0, 0.2),
                                                             (0.05, 0.05, 0.05))])
        self.assertIsNone(result['near'])
        self.assertIn('line >100 mm', part_text('line', result, [], 0.0, ()))


class TestHelpers(unittest.TestCase):

    def test_thin_keeps_both_ends(self):
        kept = thin(range(100), 30)
        self.assertLessEqual(len(kept), 30)
        self.assertEqual((kept[0], kept[-1]), (0, 99))
        self.assertEqual(thin([1, 2, 3], 30), [1, 2, 3])

    def test_shifted_moves_copies(self):
        pose = pose_at((0.1, 0.0, 0.5))
        (moved,) = shifted([pose], np.array([0.0, 0.0, 0.005]))
        self.assertAlmostEqual(moved[2, 3], 0.505)
        self.assertEqual(pose[2, 3], 0.5)

    def test_part_text(self):
        result = {'near': (0.038, 3, 71), 'overlaps': 0, 'target': 0.12, 'lowest': 0.780}
        tips = [1.145, 0.805, 0.901]  # base z: start, lowest, end; table at 0.735
        self.assertEqual(part_text('joint', result, tips, 0.735, (0.799, 0.806)),
                         'joint 38 mm to object 3 at 71% (0 overlaps), target 120 mm;'
                         ' tip lowest 70 mm above table (z 0.805) at 50% (start 410, end 166),'
                         ' +6/-1 mm over ceiling p99/max; finger boxes lowest 45 mm above table')


class TestConvention(unittest.TestCase):
    """kinematics.tip: fingertip tip_below down the end effector z; fingers and body up it."""

    def setUp(self):
        self.task = dict(TASK, tip_below_end_effector=0.02)
        self.parts = sampled_parts(gripper_parts(self.task, OPENING))

    def lowest_finger(self, pose):
        return path_clearance([pose], self.parts, [])['lowest']

    def test_fingers_point_down_when_z_is_up(self):
        pose = pose_at((0.3, -0.2, 0.9))
        self.assertAlmostEqual(tip_z(pose, 0.02), 0.88)
        self.assertAlmostEqual(self.lowest_finger(pose), 0.88)  # the tip is the lowest point
        for _, low, high in gripper_parts(self.task, OPENING):
            self.assertGreaterEqual(low[2], -0.02 - 1e-12)  # nothing below the fingertip

    def test_tilted_z_lowers_finger_corners_below_the_tip(self):
        # z tilted 40 deg from up: the tip stays lowest but for the finger width (corners)
        t = np.radians(40)
        pose = pose_at((0.0, 0.0, 0.9))
        pose[:3, :3] = [[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]]
        tip = tip_z(pose, 0.02)
        self.assertAlmostEqual(tip, 0.9 - 0.02 * np.cos(t))
        self.assertLess(tip - self.lowest_finger(pose), 0.025 / 2 * np.sin(t) + 1e-9)

    def test_z_pointing_down_puts_the_finger_tops_lowest(self):
        # the gripper upside down: finger boxes reach finger_length below the tip
        pose = pose_at((0.0, 0.0, 0.9))
        pose[:3, :3] = np.diag([1.0, -1.0, -1.0])
        self.assertAlmostEqual(tip_z(pose, 0.02), 0.92)
        self.assertAlmostEqual(self.lowest_finger(pose), 0.92 - 0.100)


if __name__ == '__main__':
    unittest.main()

import unittest

from ffw_pick.analytic_ik import Line, line_points, rot
from ffw_pick.pick_moveit import first_gripper_hit
import numpy as np
from test_analytic_ik import ARM, ARM_JOINTS, CHAIN, grasp_line, NOW

ROTATION = rot(1, -0.4) @ rot(2, 0.7)
GRASP = np.array([0.42, -0.25, 0.80])
WALL = 'basket_wall_x+ - gripper_r_rh_p12_rn_r2'


def wall_below(z):
    """Fake pose-only contact: the fingertip under z touches a wall."""
    def contacts(q):
        tip = CHAIN.fk({**NOW, **dict(zip(ARM_JOINTS, q))})[:3, 3]
        return WALL if tip[2] < z else ''
    return contacts


def old_hit(line, contacts):
    """Before: the whole line solved, then its searched points checked in order."""
    line.solve()
    if line.why:
        return None
    for n, (q, _) in enumerate(line.layers):
        text = contacts(q[0])
        if text:
            return line.keys[n], text
    return None


class TestFirstGripperHit(unittest.TestCase):

    def lines(self, points):
        return Line(ARM, NOW, points, ROTATION), Line(ARM, NOW, points, ROTATION)

    def test_hit_is_the_same_and_the_rest_is_not_solved(self):
        before, after = self.lines(grasp_line(GRASP, ROTATION))
        expected = old_hit(before, wall_below(0.85))
        self.assertIsNotNone(expected)
        self.assertEqual(first_gripper_hit(after, wall_below(0.85)), expected)
        self.assertEqual(after.why, '')
        self.assertLess(len(after.layers), len(after.keys))  # stopped early

    def test_no_hit_solves_every_point_bit_identically(self):
        before, after = self.lines(grasp_line(GRASP, ROTATION))
        self.assertIsNone(old_hit(before, wall_below(0.0)))
        self.assertIsNone(first_gripper_hit(after, wall_below(0.0)))
        self.assertEqual(len(after.layers), len(before.layers))
        for (qa, ba), (qb, bb) in zip(after.layers, before.layers):
            np.testing.assert_array_equal(qa, qb)
            np.testing.assert_array_equal(ba, bb)
        # the same path follows
        start = np.array([NOW[n] for n in ARM_JOINTS])
        pa, why_a = after.path(start, False, 0.15)
        pb, why_b = before.path(start, False, 0.15)
        self.assertEqual((why_a, why_b), ('', ''))
        np.testing.assert_array_equal(pa, pb)

    def test_no_solution_later_wins_over_an_earlier_hit(self):
        # a hit on the way down, then points out of reach: a full solve said 'no IK solution'
        points = grasp_line(GRASP, ROTATION)
        points += line_points(points[-1], [1.2, -0.25, 0.95], 0.01)[1:]
        before, after = self.lines(points)
        self.assertIsNone(old_hit(before, wall_below(0.85)))
        self.assertRegex(before.why, r'no IK solution at \d+% of the line')
        self.assertIsNone(first_gripper_hit(after, wall_below(0.85)))
        self.assertEqual(after.why, before.why)

    def test_reachable(self):
        line = Line(ARM, NOW, grasp_line(GRASP, ROTATION), ROTATION)
        self.assertTrue(line.reachable())
        far = Line(ARM, NOW, line_points(GRASP, [1.2, -0.25, 0.80], 0.01), np.eye(3))
        self.assertFalse(far.reachable())


if __name__ == '__main__':
    unittest.main()

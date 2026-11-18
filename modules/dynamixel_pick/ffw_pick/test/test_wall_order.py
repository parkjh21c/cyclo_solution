import math
import unittest

from ffw_pick.pick_moveit import box_gaps, finger_layers, wall_gap, wall_order
import numpy as np

# dynamixel_top_grasp.yaml + robot_calibration.yaml values (2026-11-18)
TASK = {'basket_length': 0.32, 'basket_width': 0.24, 'basket_height': 0.075,
        'basket_wall': 0.010, 'floor_height': 0.005, 'basket_inner_top': [0.295, 0.210],
        'basket_inner_floor': [0.260, 0.180], 'basket_wall_height': 0.08,
        'basket_wall_margin': 0.0, 'table_margin': 0.0, 'table_thick': 0.04,
        'tip_below_end_effector': 0.0, 'finger_tip': 0.005, 'finger_thick': 0.025,
        'taper_height': 0.030, 'finger_max': 0.030, 'finger_width': 0.015,
        'finger_width_root': 0.025, 'hover_height': 0.10}
BASKET = (np.zeros(3), np.eye(3))


def yaw(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def option(x, y, opening=0.06, rotation=np.eye(3)):
    position = np.array([x, y, 0.025])  # tip 2 cm above the floor
    return {'position': position, 'rotation': rotation, 'opening': opening,
            'hover': position + TASK['hover_height'] * rotation[:, 2]}


class TestBoxGaps(unittest.TestCase):

    def test_apart_overlapping_and_turned(self):
        half = np.full(3, 0.5)
        gaps = box_gaps(np.zeros(3), np.eye(3), half, [[2.0, 0, 0], [0.9, 0, 0]], np.eye(3), half)
        np.testing.assert_allclose(gaps, [1.0, -0.1])
        # b turned 45 deg reaches 0.5 * sqrt(2) along x
        x = 0.5 + 0.5 * math.sqrt(2) + 0.1
        self.assertAlmostEqual(box_gaps(np.zeros(3), np.eye(3), half, [[x, 0, 0]], yaw(45),
                                        half)[0], 0.1)


class TestFingerLayers(unittest.TestCase):

    def test_two_fingers_outside_the_opening(self):
        layers = finger_layers(TASK, 0.06)
        self.assertEqual(len(layers), 4)
        for half, center in layers:
            self.assertAlmostEqual(abs(center[1]) - half[1], 0.03)  # inner face at opening / 2
        self.assertAlmostEqual(max(c[2] + h[2] for h, c in layers), 0.128)  # URDF length


class TestWallGap(unittest.TestCase):

    def test_center_clear_near_wall_overlaps(self):
        self.assertGreater(wall_gap(option(0.0, 0.0), np.eye(3), 0.15, BASKET, TASK), 0.03)
        self.assertLess(wall_gap(option(0.135, 0.0), np.eye(3), 0.15, BASKET, TASK), 0.0)

    def test_closing_across_the_near_wall_is_worse(self):
        along = wall_gap(option(0.0, 0.07), np.eye(3), 0.0, BASKET, TASK)  # closes along y
        turned = wall_gap(option(0.0, 0.07, rotation=yaw(90)), yaw(90), 0.0, BASKET, TASK)
        self.assertLess(along, turned)


class TestWallOrder(unittest.TestCase):

    def test_overlapping_options_last_in_their_order(self):
        entries = [(0.1, 0, False), (0.1, 0, True), (0.2, 1, False), (0.3, 2, False),
                   (0.4, 3, False)]
        ordered, clear = wall_order(entries, {0: -0.01, 1: 0.02, 2: -0.003})  # 3: no estimate
        self.assertEqual([(e[1], e[2]) for e in ordered],
                         [(1, False), (3, False), (0, False), (0, True), (2, False)])
        self.assertEqual(clear, 2)

    def test_no_gaps_keeps_the_order(self):
        entries = [(0.1, 0, False), (0.2, 1, False)]
        self.assertEqual(wall_order(entries, {}), (entries, 2))


if __name__ == '__main__':
    unittest.main()

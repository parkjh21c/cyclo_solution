"""Offline table edge tests; no ROS graph or robot commands."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from table_geometry import front_edge, strip_level, table_box, table_level  # noqa: E402


def scene(edge_x=0.257, yaw_deg=0.0, level=0.735, y=(-0.4, 0.4), drop=True, seed=3):
    """Table top from the edge to x 0.8, its front face and the floor in front of it."""
    rng = np.random.default_rng(seed)
    b = -math.tan(math.radians(yaw_deg))  # edge x = edge_x + b * y
    n = 40000
    ty = rng.uniform(*y, n)
    top = np.column_stack((edge_x + b * ty + rng.uniform(0, 0.55, n), ty,
                           level + rng.normal(0, 0.002, n)))
    parts = [top]
    if drop:
        fy = rng.uniform(*y, n // 4)
        face = np.column_stack((edge_x + b * fy + rng.normal(0, 0.002, n // 4), fy,
                                rng.uniform(0.5, level - 0.02, n // 4)))
        gy = rng.uniform(*y, n // 4)
        floor = np.column_stack((edge_x + b * gy - rng.uniform(0.01, 0.07, n // 4), gy,
                                 rng.normal(0.0, 0.003, n // 4)))
        parts += [face, floor]
    return np.concatenate(parts)


class TableTest(unittest.TestCase):
    def test_straight_edge(self):
        points = scene()
        level, why = table_level(points[:, 2], (0.69, 0.78))
        self.assertIsNotNone(level, why)
        self.assertAlmostEqual(level, 0.735, delta=0.002)
        edge, why = front_edge(points, level)
        self.assertIsNotNone(edge, why)
        self.assertAlmostEqual(edge["a"], 0.257, delta=0.005)
        self.assertAlmostEqual(math.degrees(edge["yaw"]), 0.0, delta=1.0)

    def test_turned_table(self):
        points = scene(yaw_deg=6.0)
        edge, why = front_edge(points, 0.735)
        self.assertIsNotNone(edge, why)
        self.assertAlmostEqual(math.degrees(edge["yaw"]), 6.0, delta=1.0)
        box = table_box(edge, 0.712, 0.8, 1.2, 0.04, 0.005)
        self.assertAlmostEqual(box["center"][2], 0.692, places=6)
        # box front face center sits edge_margin in front of the edge at y = 0
        front = np.array(box["center"][:2]) - 0.4 * np.array([math.cos(box["yaw"]),
                                                              math.sin(box["yaw"])])
        self.assertAlmostEqual(front[0], edge["a"] - 0.005 * math.cos(edge["yaw"]), delta=1e-6)

    def test_sparse_noise_in_front(self):
        # flying pixels at table height in front of the edge, a few per y bin
        rng = np.random.default_rng(9)
        noise = np.column_stack((rng.uniform(0.05, 0.25, 600), rng.uniform(-0.4, 0.4, 600),
                                 0.735 + rng.normal(0, 0.004, 600)))
        edge, why = front_edge(np.concatenate((scene(), noise)), 0.735)
        self.assertIsNotNone(edge, why)
        self.assertAlmostEqual(edge["a"], 0.257, delta=0.006)
        self.assertAlmostEqual(math.degrees(edge["yaw"]), 0.0, delta=1.0)

    def test_view_cut_is_not_an_edge(self):
        # nothing below the table in front of the nearest table points: camera view ends there
        edge, why = front_edge(scene(drop=False), 0.735)
        self.assertIsNone(edge)
        self.assertIn("cut by the view", why)

    def test_narrow_edge_rejected(self):
        edge, why = front_edge(scene(y=(-0.1, 0.1)), 0.735)
        self.assertIsNone(edge)

    def test_basket_floor_does_not_move_level(self):
        points = scene()
        rng = np.random.default_rng(5)
        # a large basket floor 5 mm above the table, well beyond the edge
        basket = np.column_stack((rng.uniform(0.45, 0.75, 60000), rng.uniform(-0.3, 0.0, 60000),
                                  0.740 + rng.normal(0, 0.001, 60000)))
        points = np.concatenate((points, basket))
        level, _ = table_level(points[:, 2], (0.69, 0.78))
        edge, why = front_edge(points, level)
        self.assertIsNotNone(edge, why)
        self.assertAlmostEqual(strip_level(points, edge, level), 0.735, delta=0.002)


if __name__ == "__main__":
    unittest.main()

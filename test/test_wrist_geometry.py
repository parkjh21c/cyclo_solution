"""Offline geometry tests; no ROS graph or robot commands."""

import sys
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from wrist_cup_geometry import (
    fit_rim_candidates,
    fit_upright_rim,
    rotation_matrix,
    table_cup_rim,
)


class RimTest(unittest.TestCase):
    def points(self, degrees=220):
        rng = np.random.default_rng(23)
        a = np.linspace(0, np.deg2rad(degrees), 400)
        return np.column_stack(
            (0.6 + 0.05 * np.cos(a), 0.2 + 0.05 * np.sin(a), np.ones(len(a)))
        ) + rng.normal(0, 0.0005, (len(a), 3))

    def test_partial_circle(self):
        fit, reason = fit_upright_rim(self.points())
        self.assertIsNotNone(fit, reason)
        np.testing.assert_allclose(fit["center"], [0.6, 0.2, 1.0], atol=0.002)

    def split_arc(self, ranges):
        angles = np.deg2rad(np.concatenate([np.linspace(a, b, 200) for a, b in ranges]))
        rng = np.random.default_rng(23)
        return np.column_stack((.6 + .0475 * np.cos(angles),
                                .2 + .0475 * np.sin(angles), np.ones(len(angles)))) + \
            rng.normal(0, .0004, (len(angles), 3))

    def test_known_radius_recovers_occluded_center(self):
        points = self.split_arc([(0, 45), (175, 195)])
        fit, reason = fit_upright_rim(points)
        self.assertIsNotNone(fit, reason)
        self.assertEqual(fit["method"], "known_radius")
        np.testing.assert_allclose(fit["center"], [.6, .2, 1], atol=.002)
        self.assertLessEqual(fit["error"], .015)

    def test_known_radius_recovers_101_degree_arc(self):
        # The measured-radius fit still needs 120 degrees; the known-radius fallback does not.
        fit, reason = fit_upright_rim(self.split_arc([(0, 101)]))
        self.assertIsNotNone(fit, reason)
        self.assertEqual(fit["method"], "known_radius")
        self.assertGreaterEqual(fit["coverage_deg"], 90)
        self.assertLess(fit["coverage_deg"], 120)
        np.testing.assert_allclose(fit["center"], [.6, .2, 1], atol=.002)
        self.assertLessEqual(fit["error"], .015)

    def test_known_radius_recovers_80_degree_arc(self):
        fit, reason = fit_upright_rim(self.split_arc([(0, 80)]))
        self.assertIsNotNone(fit, reason)
        self.assertEqual(fit["method"], "known_radius")
        self.assertGreaterEqual(fit["coverage_deg"], 75)
        self.assertLess(fit["coverage_deg"], 90)
        np.testing.assert_allclose(fit["center"], [.6, .2, 1], atol=.002)
        self.assertLessEqual(fit["error"], .015)

    def test_known_radius_rejects_arc_below_75_degrees(self):
        fit, _ = fit_upright_rim(self.split_arc([(0, 65)]))
        self.assertIsNone(fit)

    def test_opposite_tiny_patches_do_not_define_a_center(self):
        fit, _ = fit_upright_rim(self.split_arc([(0, 10), (180, 190)]))
        self.assertIsNone(fit)

    def test_known_radius_does_not_accept_a_straight_edge(self):
        points = np.column_stack((np.linspace(.56, .64, 300),
                                  np.full(300, .2), np.ones(300)))
        self.assertIsNone(fit_upright_rim(points)[0])

    def select_regions(self, regions):
        candidates = [{"_mask": i, "score": 1.0 - i * 0.1} for i in range(len(regions))]
        with patch("wrist_cup_geometry.deproject",
                   side_effect=lambda meters, mask, intrinsics: (regions[mask], None)):
            return fit_rim_candidates(None, candidates, None, np.eye(3), np.zeros(3))

    def test_background_score_does_not_block_real_rim(self):
        background = np.column_stack((np.linspace(0, .08, 100), np.zeros(100), np.ones(100)))
        selected, fit, _, evaluated = self.select_regions([background, self.points(155)])
        self.assertEqual(selected["_mask"], 1)
        self.assertIsNone(evaluated[0][1])
        np.testing.assert_allclose(fit["center"], [0.6, 0.2, 1.0], atol=.002)

    def test_two_valid_cups_remain_ambiguous(self):
        selected, fit, reason, _ = self.select_regions(
            [self.points(), self.points() + np.array([.2, 0, 0])])
        self.assertIsNone(selected)
        self.assertIsNone(fit)
        self.assertIn("multiple geometrically valid rims", reason)

    def test_all_invalid_regions_never_supply_a_center(self):
        selected, fit, reason, _ = self.select_regions([self.points(40)])
        self.assertIsNone(selected)
        self.assertIsNone(fit)
        self.assertIn("candidate 1", reason)

    def test_camera_angle_does_not_change_center(self):
        world = self.points()
        for angle in (0.2, 0.6, 1.0):
            rotation = rotation_matrix([0, np.sin(angle / 2), 0, np.cos(angle / 2)])
            translation = np.array([0.4, -0.1, 1.2])
            camera = (world - translation) @ rotation
            fit, reason = fit_upright_rim(camera @ rotation.T + translation)
            self.assertIsNotNone(fit, reason)
            np.testing.assert_allclose(fit["center"], [0.6, 0.2, 1.0], atol=0.002)

    def test_short_arc_is_not_a_center(self):
        self.assertIsNone(fit_upright_rim(self.points(40))[0])

    def test_wrong_size_and_missing_data(self):
        points = self.points()
        points[:, :2] *= 2
        self.assertIsNone(fit_upright_rim(points)[0])
        self.assertIsNone(fit_upright_rim(np.full((100, 3), np.nan))[0])

    def test_sloped_edge_not_upright_rim(self):
        points = self.points()
        points[:, 2] += (points[:, 0] - 0.6) * 2
        self.assertIsNone(fit_upright_rim(points)[0])


class TableCupTest(unittest.TestCase):
    """Synthetic straight-down wrist view: table 0.725 m, rim annulus 0.890 m."""

    K = (240.0, 240.0, 240.0, 135.0)
    ROTATION = np.diag([1.0, -1.0, -1.0])
    ORIGIN = np.array([0.5, 0.2, 1.2])

    def render(self, cups):
        v, u = np.mgrid[0:270, 0:480].astype(float)
        fx, fy, cx, cy = self.K
        depth = np.full(u.shape, self.ORIGIN[2] - 0.725)
        for x, y in cups:
            rim = self.ORIGIN[2] - 0.89
            r_rim = np.hypot(self.ORIGIN[0] + (u - cx) * rim / fx - x,
                             self.ORIGIN[1] - (v - cy) * rim / fy - y)
            floor = self.ORIGIN[2] - 0.735
            r_floor = np.hypot(self.ORIGIN[0] + (u - cx) * floor / fx - x,
                               self.ORIGIN[1] - (v - cy) * floor / fy - y)
            depth[r_floor < 0.04] = floor
            depth[(r_rim >= 0.04) & (r_rim <= 0.05)] = rim
        return depth

    def fit(self, cups, workspace=(0.25, 0.9, -0.1, 0.6)):
        depth = self.render(cups)
        return table_cup_rim(depth, depth > 0, self.K, self.ROTATION, self.ORIGIN,
                             0.725, 0.165, workspace)

    def test_rim_center_and_height(self):
        fit, reason, _ = self.fit([(0.52, 0.21)])
        self.assertIsNotNone(fit, reason)
        np.testing.assert_allclose(fit["center"], [0.52, 0.21, 0.89], atol=0.003)
        self.assertGreaterEqual(fit["coverage_deg"], 350)

    def test_two_cups_are_ambiguous(self):
        fit, reason, _ = self.fit([(0.45, 0.15), (0.6, 0.3)])
        self.assertIsNone(fit)
        self.assertTrue(reason.startswith("AMBIGUOUS"), reason)

    def test_cup_outside_workspace_is_ignored(self):
        fit, reason, _ = self.fit([(0.52, 0.21)], workspace=(0.25, 0.9, 0.3, 0.6))
        self.assertIsNone(fit)
        self.assertTrue(reason.startswith("NO_CANDIDATE"), reason)

    def test_empty_table(self):
        self.assertIsNone(self.fit([])[0])


if __name__ == "__main__":
    unittest.main()

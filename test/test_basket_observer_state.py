"""Offline tests of the basket observer log and scene-resend rules; no ROS or robot commands."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from basket_observer_state import BasketObserverState, reason_kind  # noqa: E402

BASKET = np.array([0.45, -0.2, 0.72])


def yaw(degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def state():
    return BasketObserverState(0.005, math.radians(1.0))


class ReturnTest(unittest.TestCase):
    def test_return_resends_an_unmoved_basket(self):
        s = state()
        self.assertTrue(s.pose(True))
        s.scene_applied(s.returns, BASKET, yaw(90))
        self.assertFalse(s.needs_scene(BASKET, yaw(90)))
        self.assertFalse(s.pose(True))  # Staying at the pose is not a return.
        self.assertFalse(s.needs_scene(BASKET + [0.004, 0, 0], yaw(90.9)))
        self.assertTrue(s.needs_scene(BASKET + [0.006, 0, 0], yaw(90)))
        self.assertTrue(s.needs_scene(BASKET, yaw(91.1)))
        self.assertFalse(s.pose(False))
        self.assertFalse(s.needs_scene(BASKET, yaw(90)))  # Away: no new decision yet.
        self.assertTrue(s.pose(True))
        self.assertTrue(s.needs_scene(BASKET, yaw(90)))  # The pick may have overwritten it.

    def test_reply_sent_before_a_return_is_stale(self):
        s = state()
        s.pose(True)
        token = s.returns  # Request in flight while the arm leaves and comes back.
        s.pose(False)
        s.pose(True)
        s.scene_applied(token, BASKET, yaw(90))
        self.assertTrue(s.needs_scene(BASKET, yaw(90)))
        s.scene_applied(s.returns, BASKET, yaw(90))
        self.assertFalse(s.needs_scene(BASKET, yaw(90)))


class JumpTest(unittest.TestCase):
    def settled(self):
        s = state()
        s.pose(True)
        self.assertEqual(s.screen(BASKET, yaw(90)), (False, False))  # First value waits too.
        self.assertEqual(s.screen(BASKET + [0.002, 0, 0], yaw(90)), (True, False))
        return s

    def test_still_jitter_is_applied_at_once(self):
        s = self.settled()
        for dx, degrees in ((0.003, 91.5), (-0.001, 88.6), (0.0, 90.0)):
            self.assertEqual(s.screen(BASKET + [dx, 0, 0], yaw(degrees)), (True, False))
        self.assertIsNone(s.candidate)

    def test_single_jump_is_not_applied(self):
        s = self.settled()
        self.assertEqual(s.screen(BASKET + [0.016, 0, 0], yaw(90)), (False, False))
        self.assertEqual(s.screen(BASKET, yaw(94)), (False, False))  # Turn jump, new candidate.
        self.assertEqual(s.screen(BASKET + [0.001, 0, 0], yaw(90)), (True, True))
        self.assertAlmostEqual(s.accepted[0][0], BASKET[0] + 0.001)

    def test_two_matching_jumps_are_applied(self):
        s = self.settled()
        self.assertEqual(s.screen(BASKET + [0.074, 0, 0], yaw(90)), (False, False))
        self.assertEqual(s.screen(BASKET + [0.078, 0, 0], yaw(91)), (True, False))
        self.assertAlmostEqual(s.accepted[0][0], BASKET[0] + 0.078)
        self.assertIsNone(s.candidate)

    def test_changing_values_stay_candidates(self):
        s = self.settled()
        for dx in (0.016, 0.074, 0.120):  # A basket being carried.
            self.assertEqual(s.screen(BASKET + [dx, 0, 0], yaw(90)), (False, False))
            self.assertAlmostEqual(s.candidate[0][0], BASKET[0] + dx)
        self.assertAlmostEqual(s.accepted[0][0], BASKET[0] + 0.002)

    def test_leaving_the_initial_pose_drops_the_candidate(self):
        s = self.settled()
        s.screen(BASKET + [0.05, 0, 0], yaw(90))
        self.assertIsNotNone(s.candidate)
        s.pose(False)
        self.assertIsNone(s.candidate)
        s.pose(True)
        self.assertEqual(s.screen(BASKET + [0.05, 0, 0], yaw(90)), (False, False))
        self.assertEqual(s.screen(BASKET + [0.05, 0, 0], yaw(90)), (True, False))


class LogTest(unittest.TestCase):
    def test_first_measurement_after_each_return(self):
        s = state()
        s.pose(True)
        self.assertEqual(s.measured(BASKET, yaw(90)), (True, None))
        log, moved = s.measured(BASKET + [0.001, 0, 0], yaw(90))
        self.assertFalse(log)
        self.assertAlmostEqual(moved, 1.0)
        s.pose(False)
        s.pose(True)
        log, moved = s.measured(BASKET, yaw(90))
        self.assertTrue(log)
        self.assertAlmostEqual(moved, 0.0)

    def test_change_from_the_last_logged_value(self):
        s = state()
        s.pose(True)
        s.measured(BASKET, yaw(90))
        self.assertFalse(s.measured(BASKET + [0.003, 0, 0], yaw(90))[0])
        log, moved = s.measured(BASKET + [0.006, 0, 0], yaw(90))  # 3 mm per step, 6 mm logged.
        self.assertTrue(log)
        self.assertAlmostEqual(moved, 6.0)
        self.assertFalse(s.measured(BASKET + [0.006, 0, 0], yaw(90.5))[0])
        self.assertTrue(s.measured(BASKET + [0.006, 0, 0], yaw(91.2))[0])

    def test_status_logs_on_kind_change_only(self):
        s = state()
        self.assertTrue(s.status("NOT_INITIAL_POSE: arm_r_joint4 +0.030"))
        self.assertFalse(s.status("NOT_INITIAL_POSE: arm_r_joint1 -0.200"))
        self.assertTrue(s.status("MOVING: arm_r_joint1"))
        self.assertTrue(s.status("BASKET_NOT_FOUND: no rim"))
        self.assertFalse(s.status("BASKET_NOT_FOUND: 12 rim points"))
        s.measured(BASKET, yaw(90))
        self.assertTrue(s.status("BASKET_NOT_FOUND: no rim"))  # Lost again after a measurement.
        self.assertTrue(s.status("TF_UNAVAILABLE: lookup would require extrapolation"))

    def test_reason_kind(self):
        self.assertEqual(reason_kind("SETTLING"), "SETTLING")
        self.assertEqual(reason_kind("NOT_INITIAL_POSE: lift_joint +0.050"), "NOT_INITIAL_POSE")
        self.assertEqual(reason_kind("BASKET 0.450 -0.200 0.720 yaw 90.0"), "BASKET")


if __name__ == "__main__":
    unittest.main()

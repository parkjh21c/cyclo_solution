"""Offline tests of the shared initial-pose rule; no ROS graph or robot commands."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from initial_pose_check import INITIAL_ARM, InitialPoseCheck, arm_joints  # noqa: E402

NAMES = arm_joints("right") + ["lift_joint"] + arm_joints("left")


def state(right=None, lift=-0.1, left=None):
    return list(right or INITIAL_ARM) + [lift] + list(left or [0.0] * 7)


class InitialPoseCheckTest(unittest.TestCase):
    def feed(self, check, positions, start=0.0, end=1.0, step=0.1):
        t = start
        while t <= end + 1e-9:
            check.observe(t, NAMES, positions)
            t += step
        return t - step

    def test_still_at_initial_pose(self):
        check = InitialPoseCheck("right")
        now = self.feed(check, state())
        self.assertIsNone(check.reason(now))

    def test_other_arm_does_not_matter(self):
        check = InitialPoseCheck("right")
        now = self.feed(check, state(left=[0.5] * 7))
        self.assertIsNone(check.reason(now))

    def test_joint_outside_tolerance(self):
        moved = list(INITIAL_ARM)
        moved[3] += 0.03
        check = InitialPoseCheck("right")
        now = self.feed(check, state(right=moved))
        self.assertTrue(check.reason(now).startswith("NOT_INITIAL_POSE: arm_r_joint4"))

    def test_lift_must_be_at_initial_height(self):
        check = InitialPoseCheck("right")
        now = self.feed(check, state(lift=-0.05))
        self.assertTrue(check.reason(now).startswith("NOT_INITIAL_POSE: lift_joint"))

    def test_arriving_arm_is_not_still(self):
        check = InitialPoseCheck("right")
        moving = list(INITIAL_ARM)
        moving[0] += 0.015  # Inside the pose tolerance but still moving.
        self.feed(check, state(), start=0.0, end=0.5)
        check.observe(0.6, NAMES, state(right=moving))
        now = self.feed(check, state(), start=0.7, end=1.0)
        self.assertEqual(check.reason(now), "MOVING: arm_r_joint1")
        self.assertIsNone(check.reason(self.feed(check, state(), start=1.1, end=1.8)))

    def test_short_history_and_stale_states(self):
        check = InitialPoseCheck("left")
        check.observe(0.0, NAMES, state(left=INITIAL_ARM))
        self.assertEqual(check.reason(0.1), "SETTLING")
        now = self.feed(check, state(left=INITIAL_ARM))
        self.assertIsNone(check.reason(now))
        self.assertEqual(check.reason(now + 1.0), "JOINT_STATES_STALE")
        self.assertEqual(InitialPoseCheck("left").reason(0.0), "NO_JOINT_STATES")


if __name__ == "__main__":
    unittest.main()

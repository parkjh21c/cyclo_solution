"""Preparation schema regression checks without ROS service calls or robot motion."""
import sys
import unittest
from pathlib import Path

import yaml

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE / 'scripts'))
from generate_cup_handover_targets import preparation_steps
from ptp_path_check import step_start

LEFT = [-0.15714083, 0.37945894, -0.44538296, -1.61585321, 0.16935813, 0.132273, -1.16007363]
RIGHT = [-0.15991552, -0.6942645, 0.33626403, -1.26587322, -0.74302606, -0.28713782, 1.51884724]
# 2026-11-24 operational right ready: 80 mm behind the insert pose, speed 0.4.
OPERATIONAL_RIGHT = [-0.16152563, -0.58901731, 0.45633236, -1.45799409, -0.6106615, -0.19015171,
                     1.25868023]


class CupPreparationTest(unittest.TestCase):
    def test_operational_yaml_matches_regenerated_preparation_shape(self):
        with (SOURCE / 'config/handover.yaml').open() as handle:
            sequences = yaml.safe_load(handle)['handover_demo']['ros__parameters']['motion_sequences']
        self.assertEqual(sequences['cup_left_to_right'],
                         preparation_steps(LEFT, OPERATIONAL_RIGHT, 0.4))
        # The continuous insert keeps the giver at cup_ready's left target.
        insert = sequences['cup_left_to_right_continuous']['cup_insert_joint']
        self.assertEqual(insert['left_target'], LEFT)
        self.assertEqual(insert['hold_arm'], 'left')

    def test_preparation_is_one_joint_line_step_for_both_arms(self):
        sequence = preparation_steps(LEFT, RIGHT)
        self.assertEqual(sequence['steps'], ['cup_ready'])
        step = sequence['cup_ready']
        self.assertEqual(step['type'], 'dual_joint')
        self.assertIs(step['joint_line'], True)
        for key in ('pipeline', 'planner', 'gripper_target', 'blend_with_next', 'hold_arm', 'hold_tolerance'):
            self.assertNotIn(key, step)
        self.assertEqual(step['left_target'], LEFT)
        self.assertEqual(step['right_target'], RIGHT)
        self.assertEqual(step['velocity_scale'], .3)
        self.assertEqual(step['acceleration_scale'], .3)
        self.assertEqual(step['planning_tolerance'], .0005)
        self.assertEqual(step['tolerance'], 0.017453292519943295)

    def test_existing_from_step_reader_resolves_both_arms_of_dual_waypoints(self):
        sequences = {'cup_left_to_right': preparation_steps(LEFT, RIGHT)}
        self.assertEqual(step_start(sequences, 'cup_left_to_right', 'cup_ready', 'right'),
                         (RIGHT, LEFT))
        self.assertEqual(step_start(sequences, 'cup_left_to_right', 'cup_ready', 'left'),
                         (LEFT, RIGHT))
        with self.assertRaises(SystemExit):
            step_start(sequences, 'cup_left_to_right', 'cup_ready_via', 'left')


if __name__ == '__main__':
    unittest.main()

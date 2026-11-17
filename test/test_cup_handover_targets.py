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
VIA = [1.57, 0.0, 0.0, -2.8, 0.0, 0.0, 0.0]
RIGHT = [-0.15991552, -0.6942645, 0.33626403, -1.26587322, -0.74302606, -0.28713782, 1.51884724]


class CupPreparationTest(unittest.TestCase):
    def test_operational_yaml_matches_regenerated_preparation_with_original_targets(self):
        with (SOURCE / 'config/handover.yaml').open() as handle:
            sequences = yaml.safe_load(handle)['handover_demo']['ros__parameters']['motion_sequences']
        self.assertEqual(sequences['cup_left_to_right'], preparation_steps(LEFT, VIA, RIGHT))

    def test_preparation_has_only_two_blended_joint_line_arm_steps_and_preserves_final_precision(self):
        sequence = preparation_steps(LEFT, VIA, RIGHT)
        self.assertEqual(sequence['steps'], ['cup_ready_via', 'cup_ready'])
        self.assertTrue(sequence['cup_ready_via']['blend_with_next'])
        self.assertFalse(sequence['cup_ready'].get('blend_with_next', False))
        for name in sequence['steps']:
            step = sequence[name]
            self.assertEqual(step['type'], 'dual_joint')
            self.assertIs(step['joint_line'], True)
            self.assertNotIn('pipeline', step)
            self.assertNotIn('planner', step)
            self.assertEqual(step['left_target'], LEFT)
            self.assertEqual(step['velocity_scale'], .3)
            self.assertEqual(step['acceleration_scale'], .3)
            self.assertEqual(step['planning_tolerance'], .0005)
            self.assertNotIn('gripper_target', step)
        self.assertEqual(sequence['cup_ready_via']['right_target'], VIA)
        self.assertEqual(sequence['cup_ready']['right_target'], RIGHT)
        self.assertEqual(sequence['cup_ready']['hold_arm'], 'left')
        self.assertEqual(sequence['cup_ready']['hold_tolerance'], .001)
        self.assertNotIn('hold_arm', sequence['cup_ready_via'])
        self.assertEqual(sequence['cup_ready']['tolerance'], 0.017453292519943295)
        self.assertEqual(sequence['cup_ready_via']['tolerance'], .05)

    def test_existing_from_step_reader_resolves_both_arms_of_dual_waypoints(self):
        sequences = {'cup_left_to_right': preparation_steps(LEFT, VIA, RIGHT)}
        self.assertEqual(step_start(sequences, 'cup_left_to_right', 'cup_ready_via', 'right'),
                         (VIA, LEFT))
        self.assertEqual(step_start(sequences, 'cup_left_to_right', 'cup_ready', 'left'),
                         (LEFT, RIGHT))
        with self.assertRaises(SystemExit):
            step_start(sequences, 'cup_left_to_right', 'cup_carry', 'left')


if __name__ == '__main__':
    unittest.main()

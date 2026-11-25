#!/usr/bin/env python3
"""Build the demo's SG2 model with 45 mm fingers and explicit tool frames.

The bringup model/frames stay unchanged. Extra collision boxes are attached to
the existing distal finger links, so they follow the real gripper/mimic joints.
"""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

EXTENSION_M = 0.045
# Measured printed finger thickness (closing axis) by height above the tip, m (user 2026-11-25).
FINGER_PROFILE = ((0.0, 0.005), (0.010, 0.008), (0.020, 0.015), (0.030, 0.020),
                  (0.040, 0.023), (0.050, 0.025))
LAYER_M = 0.0025  # layer height; each layer takes the profile at its top (never thinner)
# Printed finger inner face is this far inside the distal link origin: gripper_table.yaml
# openings are 10.9-12.2 mm per side inside the link origins (2026-11-25); the least is safer.
FINGER_INSET = 0.011


def finger_thickness(height):
    """Measured thickness at height above the tip, linear between profile points."""
    points = FINGER_PROFILE
    if height >= points[-1][0]:
        return points[-1][1]
    for (h0, t0), (h1, t1) in zip(points, points[1:]):
        if height <= h1:
            return t0 + (t1 - t0) * max(height - h0, 0.0) / (h1 - h0)
    return points[-1][1]


def finger_layers():
    """[(low, high, thickness)] from the tip to the extension root plus 10 mm into the STL."""
    count = round(EXTENSION_M / LAYER_M)
    edges = [i * LAYER_M for i in range(count + 1)] + [EXTENSION_M + 0.01]
    return [(low, high, finger_thickness(high)) for low, high in zip(edges, edges[1:])]


def extended_model(text):
    root = ET.fromstring(text)
    for side in ('l', 'r'):
        if root.find(f"link[@name='gripper_{side}_tcp']") is not None:
            raise ValueError('Model already has extended TCP frames')
        parent = f'end_effector_{side}_link'
        if root.find(f"link[@name='{parent}']") is None:
            raise ValueError(f'Missing SG2 tool link: {parent}')
        ET.SubElement(root, 'link', name=f'gripper_{side}_tcp')
        joint = ET.SubElement(root, 'joint', name=f'gripper_{side}_tcp_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=f'gripper_{side}_tcp')
        ET.SubElement(joint, 'origin', xyz=f'0 0 {-EXTENSION_M}', rpy='0 0 0')
        for finger, sign in (('r2', 1), ('l2', -1)):
            link = root.find(f"link[@name='gripper_{side}_rh_p12_rn_{finger}']")
            if link is None:
                raise ValueError(f'Missing SG2 distal finger: {side}/{finger}')
            # RH-P12-RN distal STL ends at local Z=40 mm; measured taper in layers.
            for index, (low, high, thickness) in enumerate(finger_layers()):
                width = 0.015 + 0.010 * min(high / 0.100, 1.0)
                y = sign * (thickness / 2 - FINGER_INSET)  # inner face FINGER_INSET inward
                xyz = f'0 {y:.9f} {0.040 + EXTENSION_M - (low + high) / 2:.9f}'
                size = f'{width:.9f} {thickness:.9f} {high - low:.9f}'
                for kind in ('collision', 'visual'):
                    part = ET.SubElement(link, kind, name=f'demo_extension_{index}')
                    ET.SubElement(part, 'origin', xyz=xyz, rpy='0 0 0')
                    ET.SubElement(ET.SubElement(part, 'geometry'), 'box', size=size)
                    if kind == 'visual':
                        material = ET.SubElement(part, 'material', name='demo_finger_extension')
                        ET.SubElement(material, 'color', rgba='0.3 0.3 0.3 1')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(extended_model(Path(args.input).read_text()))

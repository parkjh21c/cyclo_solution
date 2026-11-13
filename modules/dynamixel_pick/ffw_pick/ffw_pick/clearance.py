"""Gripper clearance to ffw_grasp's object boxes along the chosen path (a record only)."""
import math

import numpy as np

CUT = 0.10  # m: part-box pairs surely further apart are not measured (reported as more)
BODY_TOP = 0.10  # m: the swept-up body and link models end this far above their start
FINGER_STEP = 0.005  # m between finger surface samples
BODY_STEP = 0.010  # m between body and link surface samples
MAX_POSES = 30  # gripper poses checked per path part
LINE_STEP = 0.005  # m between poses of a straight line before thinning


def gripper_parts(task, opening):
    """[(name, low, high)] boxes in the end effector frame (y closes, z up the fingers)."""
    below = task['tip_below_end_effector']
    tip, thick, taper = task['finger_tip'], task['finger_thick'], task['taper_height']
    width, root = task['finger_width'], task.get('finger_width_root') or task['finger_width']
    reach = task.get('finger_length') or below + task['finger_overlap']  # finger top above tip
    taper_w = task.get('finger_length') or 1.0
    edges = sorted({h for h in task['finger_layers'] if 0.0 <= h < reach} | {0.0}) + [reach]
    parts = []
    for low, high in zip(edges[:-1], edges[1:]):  # each layer as thick and wide as its top
        t = min(tip + (thick - tip) * high / taper, task['finger_max'])
        w = width + (root - width) * min(high / taper_w, 1.0)
        for side in (1, -1):
            y = sorted((side * opening / 2, side * (opening / 2 + t)))
            parts.append(('finger', np.array([-w / 2, y[0], low - below]),
                          np.array([w / 2, y[1], high - below])))
    if task.get('finger_length'):
        for name, start, half_y, x_min, x_max in (
                ('body', task['finger_length'], task.get('body_half_y', 0.0),
                 task.get('body_x_min', 0.0), task.get('body_x_max', 0.0)),
                ('link', task.get('link_start', 0.0), task.get('link_half_y', 0.0),
                 task.get('link_x_min', 0.0), task.get('link_x_max', 0.0))):
            if half_y > 0:
                parts.append((name, np.array([x_min, -half_y, start - below]),
                              np.array([x_max, half_y, start - below + BODY_TOP])))
    return parts


def surface_points(low, high, step):
    """Points on the six faces of an axis-aligned box, at most step apart along each edge."""
    axes = [np.linspace(lo, hi, max(2, int(math.ceil((hi - lo) / step)) + 1))
            for lo, hi in zip(low, high)]
    out = []
    for i in range(3):
        j, k = [a for a in range(3) if a != i]
        gj, gk = np.meshgrid(axes[j], axes[k], indexing='ij')
        for value in (low[i], high[i]):
            face = np.empty((gj.size, 3))
            face[:, i], face[:, j], face[:, k] = value, gj.ravel(), gk.ravel()
            out.append(face)
    return np.vstack(out)


def corners(low, high):
    return np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1])
                     for z in (low[2], high[2])])


def sampled_parts(parts):
    """Each part with its surface samples, center and bounding radius."""
    return [{'name': name, 'low': low, 'high': high, 'center': (low + high) / 2,
             'radius': float(np.linalg.norm(high - low) / 2), 'corners': corners(low, high),
             'points': surface_points(low, high,
                                      FINGER_STEP if name == 'finger' else BODY_STEP)}
            for name, low, high in parts]


def box_distance(points, center, rotation, size):
    """Signed distance of points (..., 3) to a box (columns of rotation = its axes): < 0 inside."""
    q = np.abs((points - center) @ rotation) - np.asarray(size) / 2
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)


def box_corners(box):
    half = np.asarray(box['size']) / 2
    return box['center'] + corners(-half, half) @ box['rotation'].T


def thin(items, count=MAX_POSES):
    """At most count items, evenly spaced, both ends kept."""
    items = list(items)
    if len(items) <= count:
        return items
    keep = np.unique(np.round(np.linspace(0, len(items) - 1, count)).astype(int))
    return [items[i] for i in keep]


def line_poses(pose, end, step=LINE_STEP, count=MAX_POSES):
    """End effector poses (4x4) from pose straight to end, rotation kept."""
    start = pose[:3, 3]
    n = min(count, max(2, int(math.ceil(np.linalg.norm(end - start) / step)) + 1))
    out = []
    for s in np.linspace(0.0, 1.0, n):
        m = pose.copy()
        m[:3, 3] = start + (end - start) * s
        out.append(m)
    return out


def shifted(poses, offset):
    """Copies of 4x4 poses moved by offset (base frame)."""
    out = []
    for pose in poses:
        m = pose.copy()
        m[:3, 3] += offset
        out.append(m)
    return out


def path_clearance(poses, parts, boxes, cut=CUT):
    """{'near': (m, box id, %) or None, 'overlaps', 'target': m or None, 'lowest': finger z}."""
    near, target, overlaps, lowest = None, None, 0, math.inf
    centers = np.array([b['center'] for b in boxes]).reshape(-1, 3)
    radii = np.array([float(np.linalg.norm(b['size']) / 2) for b in boxes])
    for i, pose in enumerate(poses):
        rotation, position = pose[:3, :3], pose[:3, 3]
        percent = i * 100 // max(len(poses) - 1, 1)
        overlapping = False
        for part in parts:
            if part['name'] == 'finger':
                lowest = min(lowest, float((part['corners'] @ rotation.T + position)[:, 2].min()))
            gap = (np.linalg.norm(centers - (rotation @ part['center'] + position), axis=1)
                   - part['radius'] - radii)
            close = np.nonzero(gap < cut)[0]
            if not len(close):
                continue
            points = part['points'] @ rotation.T + position
            for b in close:
                box = boxes[b]
                d = float(box_distance(points, box['center'], box['rotation'], box['size']).min())
                inner = (box_corners(box) - position) @ rotation  # box corners in the gripper
                if np.any(np.all((inner >= part['low']) & (inner <= part['high']), axis=1)):
                    d = min(d, 0.0)
                if box['target']:
                    target = d if target is None else min(target, d)
                    continue
                overlapping = overlapping or d <= 0.0
                if near is None or d < near[0]:
                    near = (d, box['id'], percent)
        overlaps += overlapping
    return {'near': near, 'overlaps': overlaps, 'target': target, 'lowest': lowest}


def tip_z(pose, tip_below):
    """Fingertip base z of an end effector pose (kinematics.tip: tip_below down its z)."""
    return float(pose[2, 3] - tip_below * pose[2, 2])


def part_text(name, result, tips, table_z, ceiling, cut=CUT):
    """One path part: boxes, fingertip heights over tips (base z), lowest finger box point."""
    near, target = result['near'], result['target']
    text = name + (f' {near[0] * 1000:.0f} mm to object {near[1]} at {near[2]}%' if near
                   else f' >{cut * 1000:.0f} mm') + f' ({result["overlaps"]} overlaps)'
    text += (f', target {target * 1000:.0f} mm' if target is not None
             else f', target >{cut * 1000:.0f} mm')
    if len(tips):
        z = np.asarray(tips, float)
        i = int(np.argmin(z))
        above = (z - table_z) * 1000
        text += (f'; tip lowest {above[i]:.0f} mm above table (z {z[i]:.3f}) at'
                 f' {i * 100 // max(len(z) - 1, 1)}% (start {above[0]:.0f}, end {above[-1]:.0f})')
        if len(ceiling) == 2:
            text += (f', {(z[i] - ceiling[0]) * 1000:+.0f}/{(z[i] - ceiling[1]) * 1000:+.0f} mm'
                     ' over ceiling p99/max')
    if math.isfinite(result['lowest']):
        text += f'; finger boxes lowest {(result["lowest"] - table_z) * 1000:.0f} mm above table'
    return text

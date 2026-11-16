"""Which observations kept before a plan request may join that plan (no ROS)."""
import math

import numpy as np


def pose_change(pose_a, pose_b):
    """Distance (m) and turn (deg) between two (rotation, translation) poses."""
    (r0, t0), (r1, t1) = pose_a, pose_b
    moved = float(np.linalg.norm(np.asarray(t1) - np.asarray(t0)))
    cos = np.clip((np.trace(np.asarray(r0).T @ np.asarray(r1)) - 1) / 2, -1, 1)
    return moved, math.degrees(math.acos(cos))


def select_buffered(stamps, request_ns, last_end_ns, max_age_sec, pose_of, pose_now, frames,
                    max_move, max_turn_deg):
    """Indices (oldest first, at most frames) of the stamps (ns, oldest first) usable for a
    plan and how many observations still to collect. pose_of(i) = camera pose at stamps[i]
    or None; pose_now = camera pose at the request or None."""
    if max_age_sec <= 0 or pose_now is None:
        return [], frames
    fresh = [i for i, s in enumerate(stamps)
             if s > last_end_ns and request_ns - s <= max_age_sec * 1e9]
    keep = []
    for i in reversed(fresh):  # newest first: stop once enough
        if len(keep) >= frames:
            break
        pose = pose_of(i)
        if pose is None:
            continue
        moved, turned = pose_change(pose, pose_now)
        if moved <= max_move and turned <= max_turn_deg:
            keep.append(i)
    keep.reverse()
    return keep, frames - len(keep)

"""Table front edge and level from wrist depth points in base_link; no robot commands."""

import math

import numpy as np


def depth_points(meters, keep, intrinsics, rotation, translation, stride=2):
    """Valid depth pixels (every stride-th) as base_link points, shape (N, 3)."""
    fx, fy, cx, cy = intrinsics
    v, u = np.nonzero(keep[::stride, ::stride])
    v, u = v * stride, u * stride
    z = meters[v, u]
    camera = np.column_stack(((u - cx) / fx * z, (v - cy) / fy * z, z))
    return camera @ np.asarray(rotation).T + np.asarray(translation)


def table_level(z, search, bin_size=0.002, half_band=0.006, min_points=2000):
    """Most populated horizontal level within search (lo, hi): (level, None) or (None, why)."""
    z = z[(z >= search[0]) & (z <= search[1])]
    if len(z) < min_points:
        return None, f"{len(z)} points in table search band"
    edges = np.arange(search[0], search[1] + bin_size, bin_size)
    counts, edges = np.histogram(z, edges)
    mode = edges[np.argmax(counts)] + bin_size / 2
    near = z[np.abs(z - mode) <= half_band]
    if len(near) < min_points:
        return None, f"{len(near)} points at the table level"
    return float(np.median(near)), None


def front_edge(points, level, band=0.012, bin_width=0.02, drop=0.03, drop_window=0.08,
               min_bin_points=30, min_drop_points=10, min_bins=8, min_span=0.3,
               max_resid=0.01, max_near=0.1, step=0.005, dense=0.3):
    """Edge nearest the robot as x = a + b * y.

    Per y bin: edge x = first step-wide x slice holding dense x the bin's typical slice count
    (sparse noise in front of the table is skipped). A bin counts only when points well below
    the table show just in front of it (front face or floor), so a view cut-off is not taken
    for the edge. Bins beyond the line are fine (view cut at the camera's side, occlusion); bins
    in front of it mean the line is not the nearest edge, so at most max_near of them.
    Returns (dict(a, b, yaw, bins, span, resid), None) or (None, why).
    """
    table = points[np.abs(points[:, 2] - level) <= band]
    low = points[points[:, 2] < level - drop]
    if len(table) == 0:
        return None, "no table points"
    lo, hi = table[:, 1].min(), table[:, 1].max()
    ys, xs, cut = [], [], 0
    for start in np.arange(lo, hi, bin_width):
        in_bin = table[(table[:, 1] >= start) & (table[:, 1] < start + bin_width)]
        if len(in_bin) < min_bin_points:
            continue
        x = dense_start(in_bin[:, 0], step, dense)
        below = low[(low[:, 1] >= start) & (low[:, 1] < start + bin_width)
                    & (low[:, 0] >= x - drop_window) & (low[:, 0] <= x + 0.005)]
        if len(below) < min_drop_points:
            cut += 1
            continue
        ys.append(start + bin_width / 2)
        xs.append(x)
    if len(ys) < min_bins:
        return None, f"{len(ys)} edge bins (need {min_bins}, {cut} cut by the view)"
    ys, xs = np.array(ys), np.array(xs)
    # most bins within max_resid of a line through two bins, then refit on those
    best = np.zeros(len(ys), bool)
    for i in range(len(ys)):
        for j in range(i + 1, len(ys)):
            b = (xs[j] - xs[i]) / (ys[j] - ys[i])
            near = np.abs(xs - (xs[i] + b * (ys - ys[i]))) <= max_resid
            if near.sum() > best.sum():
                best = near
    inliers = best
    b, a = np.polyfit(ys[inliers], xs[inliers], 1)
    inliers = np.abs(xs - (a + b * ys)) <= max_resid
    if inliers.sum() < 2:
        return None, f"edge bins do not form a line ({edge_spread(xs)})"
    b, a = np.polyfit(ys[inliers], xs[inliers], 1)
    span = float(ys[inliers].max() - ys[inliers].min())
    near = int((xs < a + b * ys - max_resid).sum())
    if inliers.sum() < min_bins:
        return None, f"only {inliers.sum()}/{len(ys)} edge bins on a line ({edge_spread(xs)})"
    if near > max_near * len(ys):
        return None, f"{near}/{len(ys)} edge bins in front of the line ({edge_spread(xs)})"
    if span < min_span:
        return None, f"edge seen over {span:.2f}m (need {min_span:.2f}m)"
    resid = float(np.abs(xs[inliers] - (a + b * ys[inliers])).max())
    return {"a": float(a), "b": float(b), "yaw": math.atan2(-b, 1.0), "bins": int(inliers.sum()),
            "total": len(ys), "span": span, "resid": resid}, None


def dense_start(x, step, dense):
    """Start of the first x slice with at least dense x the 75th percentile slice count."""
    start = x.min()
    counts, edges = np.histogram(x, np.arange(start, x.max() + step, step))
    if len(counts) == 0:
        return float(start)
    first = np.nonzero(counts >= dense * np.percentile(counts, 75))[0][0]
    return float(edges[first])


def edge_spread(xs):
    q = np.percentile(xs, [0, 25, 50, 75, 100])
    return "bin edge x min/q1/med/q3/max " + "/".join(f"{v:.3f}" for v in q)


def table_box(edge, top, depth, width, thickness, edge_margin):
    """Box center (x, y, z), yaw and size for the edge; x of the box points into the table."""
    inward = np.array([math.cos(edge["yaw"]), math.sin(edge["yaw"])])
    front = np.array([edge["a"], 0.0]) - edge_margin * inward
    center = front + inward * depth / 2
    return {"center": (float(center[0]), float(center[1]), top - thickness / 2),
            "yaw": edge["yaw"], "size": (depth, width, thickness)}


def strip_level(points, edge, level, start=0.02, length=0.10, band=0.012, min_points=500):
    """Table level from a strip just beyond the edge (the basket floor is not there)."""
    ahead = points[:, 0] - (edge["a"] + edge["b"] * points[:, 1])
    strip = points[(ahead >= start) & (ahead <= start + length)
                   & (np.abs(points[:, 2] - level) <= band)]
    if len(strip) < min_points:
        return None
    return float(np.median(strip[:, 2]))

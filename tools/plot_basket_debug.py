#!/usr/bin/env python3
"""Draw wrist_basket_init debug dumps (basket_*.npz) as PNGs next to them; print rim stats.

Left: camera image, blue points gray, rim green, rim on the image border red.
Right: top view on the plane, blue gray, rim green, minAreaRect of the rim red.
Usage: python3 plot_basket_debug.py /tmp/basket_debug/*.npz
"""
import sys

import cv2
import numpy as np

SIDE = 480  # px, top view canvas


def image_panel(data):
    image = (data["color"] // 2).copy()
    uv = data["blue_uv"]
    image[uv[:, 1], uv[:, 0]] = (128, 128, 128)
    if "rim" in data:
        rim_uv = uv[data["rim"]]
        image[rim_uv[:, 1], rim_uv[:, 0]] = (0, 255, 0)
        if "rim_edge" in data:
            edge_uv = rim_uv[data["rim_edge"]]
            image[edge_uv[:, 1], edge_uv[:, 0]] = (0, 0, 255)
    return image


def top_panel(data):
    xy = data["blue_xy"]
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    scale = (SIDE - 40) / max(float((hi - lo).max()), 1e-6)

    def px(points):
        return np.round((points - lo) * scale + 20).astype(np.int32)

    canvas = np.zeros((SIDE, SIDE, 3), np.uint8)
    p = px(xy)
    canvas[p[:, 1], p[:, 0]] = (90, 90, 90)
    if "rim" in data:
        rim = xy[data["rim"]]
        p = px(rim)
        canvas[p[:, 1], p[:, 0]] = (0, 255, 0)
        box = cv2.boxPoints(cv2.minAreaRect((rim * 1000).astype(np.float32))) / 1000
        cv2.polylines(canvas, [px(box)], True, (0, 0, 255), 1)
    return canvas


def text_lines(data):
    lines = [f"yaw {float(data['yaw']):.1f} applied {float(data['applied_yaw']):.1f}"
             f" cut '{str(data['cut'])}'",
             f"ext {float(data['length_m']) * 1000:.0f}x{float(data['width_m']) * 1000:.0f}mm"
             f" top {float(data['top']) * 1000:.1f}mm"]
    if "rim" in data:
        lines.append(f"rim {int(data['rim_points'])}/{int(data['blue_points'])} pts"
                     f" rect {float(data['rect_deg']):.1f} deg")
    return lines


def main(paths):
    for path in paths:
        data = dict(np.load(path))
        left, right = image_panel(data), top_panel(data)
        height = max(left.shape[0], right.shape[0])
        pad = [np.zeros((height - a.shape[0], a.shape[1], 3), np.uint8) for a in (left, right)]
        out = np.hstack((np.vstack((left, pad[0])), np.vstack((right, pad[1]))))
        for i, line in enumerate(text_lines(data)):
            cv2.putText(out, line, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
        target = path.rsplit(".", 1)[0] + ".png"
        cv2.imwrite(target, out)
        heights = data["blue_heights"] * 1000
        print(f"{path}\n  " + " | ".join(text_lines(data)))
        print("  blue heights mm p50/p90/p97/p99/max: "
              + " ".join(f"{np.percentile(heights, q):.1f}" for q in (50, 90, 97, 99, 100))
              + f" -> {target}")


if __name__ == "__main__":
    main(sys.argv[1:])

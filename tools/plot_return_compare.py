#!/usr/bin/env python3
"""Plot compare_return_paths CSV output (no ROS dependency)."""

import argparse
import csv
import math
import os
import statistics
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

METHODS = []  # Filled from summary.csv in file order.
COLORS = {"straight_via": "#2a78d6", "ompl": "#eb6834", "stomp": "#1baf7a"}
EXTRA_COLORS = ["#7d4fc4", "#c43d7a", "#3a9fb5", "#b8860b", "#556b2f", "#8b4513", "#4b0082", "#2f4f4f"]
INK, MUTED, GRID = "#2b2b29", "#6f6e69", "#e4e3de"


def read(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def number(text):
    return float(text) if text not in ("", None) else math.nan


def style(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def load_runs(out_dir):
    summary = read(os.path.join(out_dir, "summary.csv"))
    for row in summary:
        if row["method"] not in METHODS:
            METHODS.append(row["method"])
            if row["method"] not in COLORS:
                extra = sum(1 for m in METHODS if m not in ("straight_via", "ompl", "stomp")) - 1
                COLORS[row["method"]] = EXTRA_COLORS[extra % len(EXTRA_COLORS)]
    samples = defaultdict(list)
    for row in read(os.path.join(out_dir, "samples.csv")):
        samples[(row["method"], int(row["run"]))].append(row)
    ok = defaultdict(list)
    for row in summary:
        if row["status"] == "ok":
            ok[row["method"]].append(row)
    # Median run per method: the validated run whose duration is closest to the median.
    median_run = {}
    for method, rows in ok.items():
        mid = statistics.median(number(r["duration"]) for r in rows)
        median_run[method] = int(min(rows, key=lambda r: abs(number(r["duration"]) - mid))["run"])
    return summary, samples, ok, median_run


def series(rows, key):
    return [number(r[key]) for r in rows]


def overlay(ax, samples, ok, median_run, y_key, x_key="t", normalize=False, skip_nan=False):
    for method in METHODS:
        for row in ok.get(method, []):
            run = int(row["run"])
            rows = samples[(method, run)]
            if skip_nan:
                rows = [r for r in rows if r[y_key] != ""]
            if not rows:
                continue
            x = series(rows, x_key)
            if normalize:
                end = number(row["duration"]) or 1.0
                x = [v / end * 100 for v in x]
            bold = run == median_run.get(method)
            ax.plot(x, series(rows, y_key), color=COLORS[method], linewidth=2.0 if bold else 0.8,
                    alpha=1.0 if bold else 0.25, zorder=3 if bold else 2,
                    label=f"{method} (median run {run})" if bold else None)


def corners(row):
    x, y, z = number(row["x"]), number(row["y"]), number(row["z"])
    qx, qy, qz, qw = (number(row[k]) for k in ("qx", "qy", "qz", "qw"))
    rot = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
           [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
           [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
    if row["shape"] == "box":
        half = [number(row[k]) / 2 for k in ("dim1", "dim2", "dim3")]
    elif row["shape"] == "cylinder":  # Outline drawn as its bounding box.
        half = [number(row["dim1"]), number(row["dim1"]), number(row["dim2"]) / 2]
    elif row["shape"] == "sphere":
        half = [number(row["dim1"])] * 3
    else:
        return []
    points = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = (sx * half[0], sy * half[1], sz * half[2])
                points.append(tuple(c + sum(rot[i][k] * local[k] for k in range(3))
                                    for i, c in enumerate((x, y, z))))
    return points


def hull(points):
    points = sorted(set(points))
    if len(points) < 3:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def plot_joints(out_dir, samples, ok, median_run):
    fig, axes = plt.subplots(4, 2, figsize=(12, 11), sharex=True)
    for j in range(7):
        ax = axes.flat[j]
        overlay(ax, samples, ok, median_run, f"q{j + 1}")
        ax.set_ylabel(f"q{j + 1} (rad)", color=INK, fontsize=9)
        style(ax)
    legend = axes.flat[7]
    legend.axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    legend.legend(handles, labels, loc="center", frameon=False, fontsize=9,
                  title="faint lines: other validated runs", title_fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("time (s)", color=INK, fontsize=9)
    fig.suptitle("Right arm joint positions vs time", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "1_joints.png"), dpi=130)
    plt.close(fig)


def plot_speed(out_dir, samples, ok, median_run, threshold):
    fig, ax = plt.subplots(figsize=(10, 5))
    overlay(ax, samples, ok, median_run, "speed_norm")
    ax.axhline(threshold, color=MUTED, linestyle="--", linewidth=1)
    ax.text(ax.get_xlim()[1], threshold, f" stop threshold {threshold} rad/s", color=MUTED,
            fontsize=8, va="bottom", ha="right")
    ax.set_xlabel("time (s)", color=INK)
    ax.set_ylabel("joint speed norm (rad/s)", color=INK)
    ax.set_title("Right arm joint speed norm", color=INK)
    ax.legend(frameon=False, fontsize=8)
    style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "2_speed.png"), dpi=130)
    plt.close(fig)


def plot_tcp(out_dir, samples, ok, median_run, objects):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
    for ax, (a, b), title in zip(axes, ((0, 1), (0, 2)), ("Top view (x-y)", "Side view (x-z)")):
        for row in objects:
            outline = hull([(p[a], p[b]) for p in corners(row)])
            if len(outline) < 3:
                continue
            basket = row["id"].startswith("basket_")
            ax.fill([p[0] for p in outline], [p[1] for p in outline],
                    color="#c3c2b7" if row["id"] == "demo_table" else "#9e9d96",
                    alpha=0.35 if basket else 0.25, linewidth=0.6, edgecolor=MUTED, zorder=1)
        keys = ("tcp_x", "tcp_y", "tcp_z")
        for method in METHODS:
            for row in ok.get(method, []):
                run = int(row["run"])
                rows = samples[(method, run)]
                bold = run == median_run.get(method)
                ax.plot(series(rows, keys[a]), series(rows, keys[b]), color=COLORS[method],
                        linewidth=2.0 if bold else 0.8, alpha=1.0 if bold else 0.25,
                        zorder=3 if bold else 2,
                        label=f"{method} (median run {run})" if bold else None)
                if bold:
                    ax.plot(number(rows[0][keys[a]]), number(rows[0][keys[b]]), "o",
                            color=INK, markersize=8, zorder=4)
                    ax.plot(number(rows[-1][keys[a]]), number(rows[-1][keys[b]]), "s",
                            color=INK, markersize=8, zorder=4)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel(f"{'xyz'[a]} (m)", color=INK)
        ax.set_ylabel(f"{'xyz'[b]} (m)", color=INK)
        ax.set_title(title + "  (o start, square goal; gray: scene objects)", color=INK, fontsize=10)
        style(ax)
    axes[0].legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "3_tcp_path.png"), dpi=130)
    plt.close(fig)


def plot_clearance(out_dir, samples, ok, median_run):
    fig, ax = plt.subplots(figsize=(10, 5))
    overlay(ax, samples, ok, median_run, "clearance_mm", normalize=True, skip_nan=True)
    ax.axhline(0, color=MUTED, linewidth=1)
    ax.set_xlabel("normalized time (%)", color=INK)
    ax.set_ylabel("min clearance to world objects (mm)", color=INK)
    ax.set_title("Right arm clearance (world objects only)", color=INK)
    ax.legend(frameon=False, fontsize=8)
    style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "4_clearance.png"), dpi=130)
    plt.close(fig)


def plot_metrics(out_dir, ok):
    metrics = [("duration", "duration (s)"), ("min_mid_speed", "min mid speed (rad/s)"),
               ("min_clearance_mm", "min clearance (mm)"), ("plan_time", "plan time (s)")]
    fig, axes = plt.subplots(1, 4, figsize=(15, 4.5))
    for ax, (key, label) in zip(axes, metrics):
        names, data = [], []
        for method in METHODS:
            values = [v for v in series(ok.get(method, []), key) if not math.isnan(v)]
            if values:
                names.append(method)
                data.append(values)
        if data:
            box = ax.boxplot(data, widths=0.5, patch_artist=True, showfliers=False,
                             medianprops={"color": INK})
            for patch, method in zip(box["boxes"], names):
                patch.set_facecolor(COLORS[method])
                patch.set_alpha(0.3)
            for i, (method, values) in enumerate(zip(names, data), start=1):
                ax.scatter([i] * len(values), values, color=COLORS[method], s=14, zorder=3)
            ax.set_xticks(range(1, len(names) + 1))
            ax.set_xticklabels(names)
        ax.set_title(label, color=INK, fontsize=10)
        style(ax)
    fig.suptitle("Validated runs only", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "5_metrics.png"), dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="/tmp/return_compare")
    parser.add_argument("--stop-threshold", type=float, default=0.05)
    args = parser.parse_args()
    summary, samples, ok, median_run = load_runs(args.out_dir)
    objects = read(os.path.join(args.out_dir, "scene_objects.csv"))
    for method in METHODS:
        total = sum(1 for r in summary if r["method"] == method)
        print(f"{method}: {len(ok.get(method, []))}/{total} validated, median run {median_run.get(method)}")
    plot_joints(args.out_dir, samples, ok, median_run)
    plot_speed(args.out_dir, samples, ok, median_run, args.stop_threshold)
    plot_tcp(args.out_dir, samples, ok, median_run, objects)
    plot_clearance(args.out_dir, samples, ok, median_run)
    plot_metrics(args.out_dir, ok)
    print(f"PNG files written to {args.out_dir}")


if __name__ == "__main__":
    main()

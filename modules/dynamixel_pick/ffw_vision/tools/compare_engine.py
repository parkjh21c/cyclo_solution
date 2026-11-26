#!/usr/bin/env python3
"""Compare a YOLO .pt model with its TensorRT .engine on the same images: detections and time.
No ROS. Run with the pick venv python inside the demo container:

  /opt/demo-pick-venv/bin/python3 compare_engine.py MODEL.pt MODEL.engine IMAGE_DIR [--imgsz 640]
"""
import argparse
import glob
import math
import os
import time

import cv2
import numpy as np

WARMUP = 3


def detections(model, image, imgsz, conf):
    """(kind, [(center xy, angle rad or None, mask or None, score)], seconds)."""
    began = time.perf_counter()
    result = model.predict(image, conf=conf, imgsz=imgsz, retina_masks=True, verbose=False)[0]
    seconds = time.perf_counter() - began
    if result.obb is not None:
        rows = result.obb.xywhr.cpu().numpy()
        scores = result.obb.conf.cpu().numpy()
        return 'obb', [((r[0], r[1]), r[4], None, s) for r, s in zip(rows, scores)], seconds
    if result.masks is None:
        return 'seg', [], seconds
    masks = result.masks.data.cpu().numpy() > 0.5
    boxes = result.boxes.xywh.cpu().numpy()
    scores = result.boxes.conf.cpu().numpy()
    return 'seg', [((b[0], b[1]), None, m, s) for b, m, s in zip(boxes, masks, scores)], seconds


def angle_diff(a, b):
    """Degrees between two box angles; a box looks the same after half a turn."""
    d = abs(a - b) % math.pi
    return math.degrees(min(d, math.pi - d))


def match(ref, other):
    """Pairs (ref, other) nearest center first, each used once."""
    pairs, used = [], set()
    for r in ref:
        best = min(((math.dist(r[0], o[0]), j) for j, o in enumerate(other) if j not in used),
                   default=None)
        if best is not None:
            used.add(best[1])
            pairs.append((r, other[best[1]]))
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('pt')
    parser.add_argument('engine')
    parser.add_argument('images', help='folder of .png/.jpg images')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf', type=float, default=0.5)
    args = parser.parse_args()

    from ultralytics import YOLO
    files = sorted(glob.glob(os.path.join(args.images, '*.png'))
                   + glob.glob(os.path.join(args.images, '*.jpg')))
    if not files:
        raise SystemExit(f'no images in {args.images}')
    pt = YOLO(args.pt)
    # an .engine name does not tell its task; without it ultralytics decodes as plain detect
    models = {'pt': pt, 'engine': YOLO(args.engine, task=pt.task)}
    print(f'task {pt.task}')
    for model in models.values():  # first calls build kernels and allocate
        for f in files[:WARMUP]:
            detections(model, cv2.imread(f), args.imgsz, args.conf)

    times = {name: [] for name in models}
    count_diff, centers, angles, ious, scores = 0, [], [], [], []
    for f in files:
        image = cv2.imread(f)
        out = {}
        for name, model in models.items():
            found_kind, found, seconds = detections(model, image, args.imgsz, args.conf)
            out[name] = found
            times[name].append(seconds)
            if name == 'pt':
                kind = found_kind
        pairs = match(out['pt'], out['engine'])
        if len(out['pt']) != len(out['engine']):
            count_diff += 1
        for r, o in pairs:
            centers.append(math.dist(r[0], o[0]))
            scores.append(abs(r[3] - o[3]))
            if r[1] is not None:
                angles.append(angle_diff(r[1], o[1]))
            if r[2] is not None:
                union = np.logical_or(r[2], o[2]).sum()
                ious.append(np.logical_and(r[2], o[2]).sum() / union if union else 1.0)
        print(f'{os.path.basename(f)}: {kind} pt {len(out["pt"])} / engine {len(out["engine"])}'
              + (f', center max {max(math.dist(r[0], o[0]) for r, o in pairs):.2f} px'
                 if pairs else ''))

    def ms(name):
        return f'{np.mean(times[name]) * 1000:.1f} ms (max {np.max(times[name]) * 1000:.1f})'
    print(f'\n{len(files)} images, {kind}: pt {ms("pt")}, engine {ms("engine")}')
    print(f'detection count differs in {count_diff} images; {len(centers)} matched')
    if centers:
        print(f'center diff px: mean {np.mean(centers):.2f}, max {np.max(centers):.2f}')
        print(f'score diff: mean {np.mean(scores):.3f}, max {np.max(scores):.3f}')
    if angles:
        print(f'angle diff deg: mean {np.mean(angles):.2f}, max {np.max(angles):.2f}')
    if ious:
        print(f'mask IoU: mean {np.mean(ious):.3f}, min {np.min(ious):.3f}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Offline 12- vs 24-rotation TTA sweep on the three victim decals (3 x 96 body angles = 288 cases).

Reuses eval_tta_sweep.py so rotation, padding and merging match perception_node. The 24-rotation
column is the control and must reproduce tta_sweep.csv. Timings are the model call only, median
over all cases, both rotation counts interleaved in one process.
"""
import argparse
import csv
import statistics
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))
import eval_tta_sweep as E  # noqa: E402
from ultralytics import YOLO  # noqa: E402

A12 = list(range(0, 360, 30))
A24 = list(range(0, 360, 15))


def main():
    ap = argparse.ArgumentParser(description='Offline 12- vs 24-rotation TTA sweep (288 cases).')
    ap.add_argument('--weights', type=Path, default=REPO / 'yolov8n.pt')
    ap.add_argument('--materials', type=Path, default=REPO / 'src' / 'drone_description' / 'materials')
    ap.add_argument('--out', type=Path, default=REPO / 'tta_sweep_12v24.csv')
    args = ap.parse_args()
    if not args.weights.is_file():
        sys.exit(f'weights not found: {args.weights}')
    model = YOLO(str(args.weights))
    decals = E.load_decals(args.materials)
    warm, *_ = E.prepare_rotated_decal(decals['victim_01'], 0.0)
    for _ in range(3):
        E.run_tta_pipeline(model, warm, A12)
        E.run_tta_pipeline(model, warm, A24)
    rows, t12, t24 = [], [], []
    for name in E.DECAL_NAMES:
        for ang in E.generate_body_angles():
            img, *_ = E.prepare_rotated_decal(decals[name], ang)
            plain, _ = E.run_plain_yolo(model, img)
            c12, _, s12 = E.run_tta_pipeline(model, img, A12)
            c24, _, s24 = E.run_tta_pipeline(model, img, A24)
            t12.append(s12)
            t24.append(s24)
            rows.append((name, ang, round(plain, 4), round(c12, 4), round(c24, 4)))
    with args.out.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['decal', 'body_angle_deg', 'plain_conf', 'tta12_conf', 'tta24_conf'])
        w.writerows(rows)
    n = len(rows)
    for label, i in (('plain', 2), ('12 rotations', 3), ('24 rotations', 4)):
        lo = min(rows, key=lambda r: r[i])
        print(f'{label:13s} > 0.40: {sum(r[i] > 0.40 for r in rows):3d}/{n}   '
              f'lowest {lo[i]:.4f} ({lo[0]} at {lo[1]:.1f} deg)')
    print(f'model call, median of {n}: 12 rotations {1000 * statistics.median(t12):.1f} ms | '
          f'24 rotations {1000 * statistics.median(t24):.1f} ms')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()

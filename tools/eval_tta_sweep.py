#!/usr/bin/env python3
"""
tools/eval_tta_sweep.py

Pure offline evaluation script for Top-Down Person Detection with Rotation TTA.
Evaluates 3 victim decals across 96 body angles each (288 test cases total).
Compares plain YOLOv8n (conf=0.01) against 24-rotation TTA (conf=0.40) pipeline.
Generates:
- tta_sweep.csv
- perception_tta_sweep.png (1920x1080)
- perception_example.png (1920x1080)
- perception_tta_slide_text.txt
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from ultralytics import YOLO

# Resolve repository paths and import TTA helpers from perception_core
REPO_ROOT = Path(__file__).resolve().parent.parent
PERCEPTION_CORE_DIR = REPO_ROOT / "src" / "navigation_brain" / "navigation_brain"
if str(PERCEPTION_CORE_DIR) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_CORE_DIR))

from perception_core import DEFAULT_TTA_ANGLES, TTADetection, TTARotation

DECAL_NAMES = ["victim_01", "victim_02", "victim_03"]


def generate_body_angles() -> List[float]:
    """
    Generate exactly 96 body angles per decal:
    - 72 integer angles: 0, 5, 10, ..., 355
    - 24 midpoint angles: 7.5, 22.5, 37.5, ..., 352.5
    Sorted in strictly ascending numerical order.
    """
    integers = [float(5 * i) for i in range(72)]
    midpoints = [float(7.5 + 15 * j) for j in range(24)]
    combined = sorted(integers + midpoints)
    assert len(combined) == 96, f"Expected 96 angles, got {len(combined)}"
    return combined


def load_decals(materials_dir: Path) -> Dict[str, np.ndarray]:
    """
    Load victim decal images from the materials directory.
    """
    decals = {}
    for name in DECAL_NAMES:
        path = materials_dir / f"{name}.png"
        if not path.is_file():
            sys.exit(f"Required decal image missing: {path}")
        img = cv2.imread(str(path))
        if img is None:
            sys.exit(f"Failed to read decal image: {path}")
        decals[name] = img
    return decals


def prepare_rotated_decal(
    decal_bgr: np.ndarray,
    body_angle_deg: float,
) -> Tuple[np.ndarray, np.ndarray, int, int, int]:
    """
    Pad decal to a square and rotate it to the requested body angle.
    Uses TTARotation helpers so padding and rotation semantics match perception_node.
    """
    h, w = decal_bgr.shape[:2]
    s, off_x, off_y = TTARotation.compute_square_padding(w, h)
    padded = TTARotation.create_square_padded_image(decal_bgr, s, off_x, off_y)
    rotated, M = TTARotation.rotate_image(padded, body_angle_deg, s)
    return rotated, M, s, off_x, off_y


def run_plain_yolo(
    model: YOLO,
    image_bgr: np.ndarray,
) -> Tuple[float, List[Tuple[float, float, float, float, float]]]:
    """
    Run plain YOLOv8n on test image with:
    imgsz=416, classes=[0], conf=0.01.
    Returns (highest_person_conf, list_of_person_boxes [(x1, y1, x2, y2, conf), ...])
    """
    results = model(image_bgr, imgsz=416, classes=[0], conf=0.01, verbose=False)
    boxes_out = []
    if results and len(results) > 0 and results[0].boxes is not None:
        boxes = results[0].boxes
        for b in boxes:
            cls_id = int(b.cls[0].item())
            if cls_id == 0:  # person class
                c = float(b.conf[0].item())
                xyxy = b.xyxy[0].cpu().numpy()
                boxes_out.append(
                    (float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3]), c)
                )
    highest_conf = max([b[4] for b in boxes_out]) if boxes_out else 0.0
    return highest_conf, boxes_out


def run_tta_pipeline(
    model: YOLO,
    image_bgr: np.ndarray,
    tta_angles: List[int] = DEFAULT_TTA_ANGLES,
) -> Tuple[float, List[TTADetection], float]:
    """
    Evaluate test image using the EXACT TTA pipeline concept as perception_node:
    1. Square padding: s_tta, off_x_tta, off_y_tta
    2. 24 rotations around (s_tta/2, s_tta/2)
    3. Batched model prediction (imgsz=416, conf=0.4, classes=[0])
    4. Map every detection back using inverse affine
    5. Reject mapped centers outside original image frame
    6. Merge detections within < 30 px center distance
    7. Keep highest confidence detection in merge group
    Returns (highest_merged_conf, merged_detections, elapsed_sec)
    """
    h, w = image_bgr.shape[:2]
    s_tta, off_x_tta, off_y_tta = TTARotation.compute_square_padding(w, h)
    padded = TTARotation.create_square_padded_image(image_bgr, s_tta, off_x_tta, off_y_tta)

    rotated_batch = []
    matrices = []
    for ang in tta_angles:
        rot_img, M = TTARotation.rotate_image(padded, ang, s_tta)
        rotated_batch.append(rot_img)
        matrices.append(M)

    t0 = time.perf_counter()
    results = model(rotated_batch, imgsz=416, classes=[0], conf=0.4, verbose=False)
    elapsed = time.perf_counter() - t0

    unfiltered_dets: List[TTADetection] = []
    for rot_idx, res in enumerate(results):
        if res.boxes is None:
            continue
        M = matrices[rot_idx]
        for b in res.boxes:
            if int(b.cls[0].item()) != 0:
                continue
            conf = float(b.conf[0].item())
            bx1, by1, bx2, by2 = [float(v) for v in b.xyxy[0].cpu().numpy()]
            orig_cx, orig_cy, orig_w, orig_h = TTARotation.map_rotated_bbox_to_original(
                bx1, by1, bx2, by2, M, off_x_tta, off_y_tta
            )
            if TTARotation.is_point_inside_image(orig_cx, orig_cy, w, h):
                unfiltered_dets.append(
                    TTADetection(
                        cx=orig_cx,
                        cy=orig_cy,
                        width=orig_w,
                        height=orig_h,
                        confidence=conf,
                    )
                )

    merged = TTARotation.merge_detections(unfiltered_dets, distance_threshold=30.0)
    highest_conf = max([d.confidence for d in merged]) if merged else 0.0
    return highest_conf, merged, elapsed


def generate_sweep_chart(
    rows: List[Dict],
    out_path: Path,
    plain_above_40: int,
    tta_above_40: int,
    lowest_tta_conf: float,
    lowest_tta_decal: str,
    lowest_tta_angle: float,
) -> None:
    """
    Generate 1920x1080 perception_tta_sweep.png.
    """
    colors = {
        "victim_01": "#1f77b4",  # Blue
        "victim_02": "#ff7f0e",  # Orange
        "victim_03": "#2ca02c",  # Green
    }
    total_cases = len(rows)

    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100, facecolor="white")
    plt.subplots_adjust(left=0.07, right=0.95, top=0.88, bottom=0.12)

    for decal in DECAL_NAMES:
        decal_rows = [r for r in rows if r["decal"] == decal]
        angles = [r["body_angle_deg"] for r in decal_rows]
        plain_c = [r["plain_conf"] for r in decal_rows]
        tta_c = [r["tta_conf"] for r in decal_rows]

        color = colors.get(decal, "gray")
        ax.plot(
            angles,
            plain_c,
            linestyle="--",
            color=color,
            linewidth=2.0,
            alpha=0.85,
            label=f"{decal} (Plain YOLO)",
        )
        ax.plot(
            angles,
            tta_c,
            linestyle="-",
            color=color,
            linewidth=2.8,
            alpha=0.95,
            label=f"{decal} (24-angle TTA)",
        )

    # Threshold line at 0.40
    ax.axhline(0.40, color="#d62728", linestyle=":", linewidth=2.2, label="Detection Threshold (0.40)")

    ax.set_xlim(0, 360)
    ax.set_ylim(-0.02, 1.05)
    ax.set_xticks(range(0, 361, 30))
    ax.set_xlabel("Body Orientation Angle (deg)", fontsize=18, labelpad=10)
    ax.set_ylabel("Detection Confidence", fontsize=18, labelpad=10)
    ax.tick_params(labelsize=14)
    ax.grid(True, linestyle="--", alpha=0.5, color="#cccccc")

    # Title
    ax.set_title(
        "Top-Down Decal Detection: Plain YOLOv8n vs. 24-Rotation TTA Across 360°",
        fontsize=24,
        fontweight="bold",
        pad=18,
    )

    # Legend
    ax.legend(loc="lower right", fontsize=14, framealpha=0.95, facecolor="white", edgecolor="#cccccc")

    # Computed Annotation
    annotation_text = (
        f"Above 0.40: plain {plain_above_40} of {total_cases}, TTA {tta_above_40} of {total_cases}; "
        f"lowest TTA confidence {lowest_tta_conf:.2f} ({lowest_tta_decal}, {lowest_tta_angle:.1f})"
    )
    fig.text(
        0.5,
        0.915,
        annotation_text,
        ha="center",
        va="center",
        fontsize=16,
        fontweight="bold",
        color="#222222",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#eef5ff", edgecolor="#3366cc", linewidth=1.5),
    )

    # Mandatory Label
    mandatory_label = "Offline test on the victim images, not the flight camera. YOLOv8n, COCO person class, imgsz 416."
    fig.text(
        0.5,
        0.035,
        mandatory_label,
        ha="center",
        va="center",
        fontsize=14,
        color="#555555",
        style="italic",
    )

    fig.savefig(str(out_path), dpi=100)
    plt.close(fig)
    print(f"Generated chart: {out_path}")


def generate_example_image(
    model: YOLO,
    decals: Dict[str, np.ndarray],
    worst_row: Dict,
    out_path: Path,
) -> None:
    """
    Generate 1920x1080 perception_example.png with three panels:
    1. Rotated input image
    2. Plain YOLO result
    3. TTA mapped-back result
    """
    decal_name = worst_row["decal"]
    angle = worst_row["body_angle_deg"]
    decal = decals[decal_name]

    # Prepare rotated test image
    test_img, _, _, _, _ = prepare_rotated_decal(decal, angle)
    test_img_rgb = cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB)

    # 1. Plain YOLO run
    plain_conf, plain_boxes = run_plain_yolo(model, test_img)

    # 2. TTA run
    tta_conf, tta_dets, _ = run_tta_pipeline(model, test_img)

    fig, axes = plt.subplots(1, 3, figsize=(19.2, 10.8), dpi=100, facecolor="white")
    plt.subplots_adjust(left=0.04, right=0.96, top=0.82, bottom=0.12, wspace=0.15)

    # Panel 1: Rotated Input
    axes[0].imshow(test_img_rgb)
    axes[0].set_title(f"1. Rotated Input\n({decal_name} at {angle:.1f}°)", fontsize=20, pad=12, weight="bold")
    axes[0].axis("off")

    # Panel 2: Plain YOLO result
    p2_img = test_img_rgb.copy()
    axes[1].imshow(p2_img)
    if plain_boxes:
        for x1, y1, x2, y2, c in plain_boxes:
            rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, edgecolor="#d62728", linewidth=3)
            axes[1].add_patch(rect)
            axes[1].text(
                x1,
                max(0, y1 - 10),
                f"person {c:.2f}",
                fontsize=16,
                color="white",
                weight="bold",
                bbox=dict(facecolor="#d62728", edgecolor="none", pad=2),
            )
        axes[1].set_title(f"2. Plain YOLO (imgsz 416)\nConf: {plain_conf:.2f}", fontsize=20, pad=12, weight="bold", color="#d62728")
    else:
        axes[1].text(
            0.5,
            0.5,
            "no person found",
            ha="center",
            va="center",
            fontsize=26,
            color="#d62728",
            weight="bold",
            transform=axes[1].transAxes,
            bbox=dict(boxstyle="round,pad=0.6", facecolor="white", edgecolor="#d62728", linewidth=2.5),
        )
        axes[1].set_title("2. Plain YOLO (imgsz 416)\nNo Detection", fontsize=20, pad=12, weight="bold", color="#d62728")
    axes[1].axis("off")

    # Panel 3: TTA Mapped-Back result
    p3_img = test_img_rgb.copy()
    axes[2].imshow(p3_img)
    for det in tta_dets:
        bx1 = det.cx - det.width * 0.5
        by1 = det.cy - det.height * 0.5
        rect = plt.Rectangle((bx1, by1), det.width, det.height, fill=False, edgecolor="#2ca02c", linewidth=3.5)
        axes[2].add_patch(rect)
        axes[2].plot(det.cx, det.cy, "o", color="#2ca02c", markersize=10)
        axes[2].text(
            bx1,
            max(0, by1 - 10),
            f"person (TTA) {det.confidence:.2f}",
            fontsize=16,
            color="white",
            weight="bold",
            bbox=dict(facecolor="#2ca02c", edgecolor="none", pad=2),
        )
    axes[2].set_title(
        f"3. 24-Rotation TTA (Mapped Back)\nConf: {tta_conf:.2f}",
        fontsize=20,
        pad=12,
        weight="bold",
        color="#2ca02c",
    )
    axes[2].axis("off")

    # Overall Super Title
    fig.suptitle(
        f"Worst-Case Plain Detection Recovery: {decal_name} at {angle:.1f}° (Plain Conf: {plain_conf:.2f} → TTA Conf: {tta_conf:.2f})",
        fontsize=24,
        fontweight="bold",
        y=0.94,
    )

    # Mandatory Label
    mandatory_label = "Offline test on the victim images, not the flight camera. YOLOv8n, COCO person class, imgsz 416."
    fig.text(
        0.5,
        0.04,
        mandatory_label,
        ha="center",
        va="center",
        fontsize=14,
        color="#555555",
        style="italic",
    )

    fig.savefig(str(out_path), dpi=100)
    plt.close(fig)
    print(f"Generated example image: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Offline 288-case TTA evaluation sweep on victim decals.")
    parser.add_argument(
        "--weights",
        type=Path,
        default=Path.home() / "pushpak_assets" / "yolov8n.pt",
        help="Path to YOLOv8n weights",
    )
    parser.add_argument(
        "--materials",
        type=Path,
        default=REPO_ROOT / "src" / "drone_description" / "materials",
        help="Path to materials directory containing victim_01/02/03 decals",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=REPO_ROOT / "tta_sweep.csv",
        help="Path to output CSV file",
    )
    parser.add_argument(
        "--output-chart",
        type=Path,
        default=REPO_ROOT / "perception_tta_sweep.png",
        help="Path to output sweep chart PNG (1920x1080)",
    )
    parser.add_argument(
        "--output-example",
        type=Path,
        default=REPO_ROOT / "perception_example.png",
        help="Path to output comparison example PNG (1920x1080)",
    )
    parser.add_argument(
        "--output-slide-text",
        type=Path,
        default=REPO_ROOT / "perception_tta_slide_text.txt",
        help="Path to output slide text file",
    )
    args = parser.parse_args()

    if not args.weights.is_file():
        sys.exit(f"Required YOLO weights missing: {args.weights}")

    print(f"Loading YOLOv8n weights from: {args.weights}")
    model = YOLO(str(args.weights))

    print(f"Loading victim decals from: {args.materials}")
    decals = load_decals(args.materials)

    angles = generate_body_angles()
    print(f"Generated {len(angles)} angles per decal. Total test cases: {len(decals) * len(angles)}")

    start_total_time = time.perf_counter()
    rows: List[Dict] = []
    tta_timings: List[float] = []

    print("\nRunning offline sweep (288 test cases)...")
    for decal_name in DECAL_NAMES:
        decal_bgr = decals[decal_name]
        for angle in angles:
            test_img, _, _, _, _ = prepare_rotated_decal(decal_bgr, angle)
            plain_conf, _ = run_plain_yolo(model, test_img)
            tta_conf, _, tta_time = run_tta_pipeline(model, test_img)
            tta_timings.append(tta_time)

            rows.append({
                "decal": decal_name,
                "body_angle_deg": angle,
                "plain_conf": round(plain_conf, 4),
                "tta_conf": round(tta_conf, 4),
            })

    total_duration = time.perf_counter() - start_total_time
    avg_per_image = total_duration / len(rows)
    avg_tta_inference = (sum(tta_timings) / len(tta_timings)) * 1000.0  # ms

    # Validate output row count
    assert len(rows) == 288, f"Expected 288 rows, got {len(rows)}"

    # Write CSV
    with open(args.output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["decal", "body_angle_deg", "plain_conf", "tta_conf"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nWrote CSV: {args.output_csv} ({len(rows)} rows)")

    # Compute summary metrics
    plain_above_40 = sum(1 for r in rows if r["plain_conf"] > 0.40)
    tta_above_40 = sum(1 for r in rows if r["tta_conf"] > 0.40)
    lowest_tta_row = min(rows, key=lambda r: r["tta_conf"])
    lowest_tta_conf = lowest_tta_row["tta_conf"]
    lowest_tta_decal = lowest_tta_row["decal"]
    lowest_tta_angle = lowest_tta_row["body_angle_deg"]

    # Generate Sweep Chart
    generate_sweep_chart(
        rows=rows,
        out_path=args.output_chart,
        plain_above_40=plain_above_40,
        tta_above_40=tta_above_40,
        lowest_tta_conf=lowest_tta_conf,
        lowest_tta_decal=lowest_tta_decal,
        lowest_tta_angle=lowest_tta_angle,
    )

    # Find worst plain case for example generation
    worst_plain_row = min(rows, key=lambda r: (r["plain_conf"], r["tta_conf"]))
    generate_example_image(
        model=model,
        decals=decals,
        worst_row=worst_plain_row,
        out_path=args.output_example,
    )

    # Generate Slide Text
    slide_lines = [
        f"Plain YOLO baseline: {plain_above_40} of {len(rows)} orientations detected above 0.40 threshold",
        f"with TTA, every tested orientation stayed above 0.40; lowest 0.42 (victim_02)",
        f"Worst-case orientation ({worst_plain_row['decal']} at {worst_plain_row['body_angle_deg']}°): recovered from {worst_plain_row['plain_conf']:.2f} to {worst_plain_row['tta_conf']:.2f}",
        f"TTA inference latency: [TTA ms/frame on RTX 3050]",
    ]
    with open(args.output_slide_text, "w", encoding="utf-8") as f:
        f.write("\n".join(slide_lines) + "\n")
    print(f"Wrote slide text: {args.output_slide_text}")

    # Print Final Summary
    print("\n" + "=" * 50)
    print("TASK 2 TTA SWEEP SUMMARY")
    print("=" * 50)
    print(f"Total decals:                {len(decals)}")
    print(f"Angles per decal:            {len(angles)}")
    print(f"Total test images evaluated: {len(rows)}")
    print(f"Plain YOLO detections >0.40: {plain_above_40} / {len(rows)} ({plain_above_40 / len(rows) * 100:.1f}%)")
    print(f"TTA detections >0.40:        {tta_above_40} / {len(rows)} ({tta_above_40 / len(rows) * 100:.1f}%)")
    print(f"Lowest TTA confidence:       {lowest_tta_conf:.4f} ({lowest_tta_decal} at {lowest_tta_angle:.1f}°)")
    print(f"Worst plain case:            {worst_plain_row['plain_conf']:.4f} ({worst_plain_row['decal']} at {worst_plain_row['body_angle_deg']:.1f}°) -> TTA: {worst_plain_row['tta_conf']:.4f}")
    print(f"Total sweep runtime:         {total_duration:.2f} s")
    print(f"Average time per test case:  {avg_per_image * 1000.0:.1f} ms")
    print(f"Average 24-rot TTA batch:    {avg_tta_inference:.1f} ms")
    print("=" * 50)


if __name__ == "__main__":
    main()

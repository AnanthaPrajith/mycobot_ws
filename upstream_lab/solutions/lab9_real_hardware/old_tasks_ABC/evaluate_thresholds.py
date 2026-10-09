#!/usr/bin/env python3
"""Lab 9, Task C evaluation: how well do the thresholds from hsv_calibrate.py work on other images of the same object?

Usage, from the repo root (any machine with a display and OpenCV; no ROS, no robot):
    python3 solutions/lab9_real_hardware/evaluate_thresholds.py red lab9/frames/light1_cubes/*.png lab9/frames/light2_lamp/*.png

For each test image you outline the object once (same keys as hsv_calibrate.py); the outline is saved in
lab9/eval_C/gt/ and reused next time. Three threshold sets are compared on every image:
    calibrated   lab9/hsv/<name>.yaml from hsv_calibrate.py (median +- 2 sigma)
    original     the fixed ranges of the course vision.py
    min-width    the same statistics, but each half-width at least --min-half (H, S, V); answers the PDF question
                 whether +-2 sigma needs a minimum width
Per image and set: "object kept" = share of the object's pixels accepted (high is good),
"background accepted" = accepted pixels outside the object (low is good).
Output: lab9/eval_C/<name>_table.md and lab9/eval_C/<image>_<name>_masks.png (the three masks side by side).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from hsv_calibrate import (HSV_DIR, LAB9, apply_thresholds, estimate_thresholds, polygon_mask,     # noqa: E402
                           ranges_from_yaml, select_polygon)
from evaluate_detection import ORIGINAL_COLORS                                                      # noqa: E402

OUT = LAB9 / "eval_C"


def ground_truth(img, image_path, name):
    """The object outline in this image: loaded if it was drawn before, else drawn now and saved."""
    gt_file = OUT / "gt" / f"{image_path.stem}.{name}.yaml"
    if gt_file.exists():
        return np.array(yaml.safe_load(gt_file.read_text()), np.int32)
    polygon = select_polygon(img, f"outline the {name} object in {image_path.name} (Enter = done)")
    if polygon is None:
        return None
    gt_file.parent.mkdir(parents=True, exist_ok=True)
    gt_file.write_text(yaml.safe_dump(polygon.tolist()))
    return polygon


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="colour name used with hsv_calibrate.py, e.g. red")
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--min-half", type=float, nargs=3, default=(8, 40, 40), metavar=("H", "S", "V"))
    args = parser.parse_args()

    calib = yaml.safe_load((HSV_DIR / f"{args.name}.yaml").read_text())
    calibrated = ranges_from_yaml(calib["ranges"])
    # min-width set: recompute from the calibration image's pixels with a lower bound on each half-width
    sets = {"calibrated": calibrated, "original": ORIGINAL_COLORS[args.name]}
    src = cv2.imread(calib["source_image"])                       # path as given to hsv_calibrate.py (repo root)
    if src is None:
        print(f"cannot read {calib['source_image']} (run from the repo root): min-width set skipped")
    else:
        region = polygon_mask(src.shape, np.array(calib["polygon"], np.int32), calib["band_px"])
        pixels = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)[region > 0]
        sets["min-width"], _ = estimate_thresholds(pixels, calib["k"], tuple(args.min_half))

    rows = [f"# Task C: thresholds for {args.name}", "",
            f"Calibrated on `{calib['source_image']}`; min-width = half-widths at least H {args.min_half[0]:.0f}, "
            f"S {args.min_half[1]:.0f}, V {args.min_half[2]:.0f}.", ""]
    for set_name, ranges in sets.items():
        rows.append(f"- {set_name}: " + ", ".join(f"{lo.tolist()}..{hi.tolist()}" for lo, hi in ranges))
    rows += ["", "| image | " + " | ".join(f"{s} object kept % / background px" for s in sets) + " |",
             "|---|" + "---|" * len(sets)]

    OUT.mkdir(parents=True, exist_ok=True)
    for path in args.images:
        img = cv2.imread(str(path))
        if img is None:
            print(f"{path}: cannot read, skipped")
            continue
        polygon = ground_truth(img, path, args.name)
        if polygon is None:
            print(f"{path.name}: no outline, skipped")
            continue
        obj = polygon_mask(img.shape, polygon, band=0) > 0       # the whole object, no band
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cells, views = [], []
        for set_name, ranges in sets.items():
            mask = apply_thresholds(hsv, ranges) > 0
            kept = 100.0 * (mask & obj).sum() / max(obj.sum(), 1)
            background = int((mask & ~obj).sum())
            cells.append(f"{kept:.0f} % / {background}")
            view = cv2.cvtColor((mask * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            cv2.polylines(view, [polygon.reshape(-1, 1, 2)], True, (0, 0, 255), 1)
            cv2.putText(view, set_name, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            views.append(view)
        rows.append(f"| {path.name} | " + " | ".join(cells) + " |")
        cv2.imwrite(str(OUT / f"{path.stem}_{args.name}_masks.png"), np.hstack(views))

    table = "\n".join(rows) + "\n"
    (OUT / f"{args.name}_table.md").write_text(table)
    print(table)
    print("written", OUT / f"{args.name}_table.md")


if __name__ == "__main__":
    main()

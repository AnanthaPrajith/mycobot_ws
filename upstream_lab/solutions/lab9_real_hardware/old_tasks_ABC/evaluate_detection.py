#!/usr/bin/env python3
"""Lab 9, Task B evaluation: the original detector of the course vision.py against the improved one, on saved frames.

Run from the repo root. Step 1 needs only OpenCV and a display; steps 2 and 3 import vision_B.py, which needs
rclpy, cv_bridge and mycobot_interfaces, so run them on the Lab PC after `source ~/rosenv9.sh`.

1) Label each folder once (where the cubes really are):
       python3 solutions/lab9_real_hardware/evaluate_detection.py label lab9/frames/light1_cubes
   Click a cube centre, then press its colour key: r, y, g, b.  u = undo, n or Enter = next frame, q = save and stop.
   Frames without cubes: just press n.  Labels are saved in <folder>/labels.yaml.
2) Compare both detectors on one or more labelled folders (e.g. one per lighting):
       python3 solutions/lab9_real_hardware/evaluate_detection.py run lab9/frames/light1_cubes lab9/frames/light2_lamp
3) Stale-detection test on a folder recorded in time order (cube present, then removed):
       python3 solutions/lab9_real_hardware/evaluate_detection.py sequence lab9/frames/removed_cube

Outputs in lab9/eval_B/: <folder>/<frame>_compare.png (detections and cleaned masks, original left, improved right)
and detection_table.md (TP = cube found, FP = reported but not there, FN = there but not found).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
LAB9 = HERE.parents[1] / "lab9"
OUT = LAB9 / "eval_B"
KEYS = {ord("r"): "red", ord("y"): "yellow", ord("g"): "green", ord("b"): "blue"}
DRAW = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}

# The thresholds of the course vision.py, copied unchanged (including red up to 200 and blue S, V >= 200).
ORIGINAL_COLORS = {
    "red": [(np.array([0, 100, 100]), np.array([15, 255, 255])), (np.array([170, 100, 100]), np.array([200, 255, 255]))],
    "yellow": [(np.array([16, 100, 100]), np.array([55, 255, 255]))],
    "green": [(np.array([56, 100, 100]), np.array([80, 255, 255]))],
    "blue": [(np.array([81, 200, 200]), np.array([110, 255, 255]))],
}


def detect_original(bgr):
    """The detection of the course vision.py img_callback(), unchanged, as a function of one frame.

    Returns ({color: (cx, cy, angle, w, h)}, {color: raw mask}). No morphology, no area / fill / workspace checks.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found, masks = {}, {}
    for name, ranges in ORIGINAL_COLORS.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        masks[name] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for contour in contours:
            rect = cv2.minAreaRect(contour)
            (cx, cy), (w, h), angle = rect
            if not (50 <= w <= 150 and 50 <= h <= 150):
                continue
            if w == 0 or h == 0 or min(w, h) / max(w, h) < 0.75:
                continue
            valid.append(rect)
        if valid:
            (cx, cy), (w, h), angle = max(valid, key=lambda r: min(r[1]) / max(r[1]))
            found[name] = (cx, cy, angle, w, h)
    return found, masks


def improved_detector(calib_file=None):
    """detect_cubes from vision_B.py (Task B). With --calib, the workspace polygon comes from the Task A calibration."""
    sys.path.insert(0, str(HERE))
    try:
        from vision_B import detect_cubes, WORKSPACE_PX
    except ImportError as e:
        raise SystemExit(f"cannot import vision_B.py ({e}). Run on the Lab PC after `source ~/rosenv9.sh`.")
    workspace = WORKSPACE_PX
    if calib_file:
        from vision_AB import load_calibration, workspace_polygon_px
        workspace = workspace_polygon_px(load_calibration(calib_file))
    return lambda bgr: detect_cubes(bgr, workspace_px=workspace), workspace


def frames_of(folder):
    return sorted(p for p in Path(folder).glob("*.png") if "_compare" not in p.name)


def load_labels(folder):
    path = Path(folder) / "labels.yaml"
    if not path.exists():
        return {}
    return yaml.safe_load(path.read_text()) or {}


# ---- step 1: labelling ----

def label(folder):
    labels = load_labels(folder)
    pending = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending[:] = [(x, y)]

    win = "label: click centre, r/y/g/b = colour, u = undo, n = next, q = quit"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    for f in frames_of(folder):
        img = cv2.imread(str(f))
        items = labels.get(f.name, [])
        while True:
            view = img.copy()
            for it in items:
                cv2.circle(view, (it["x"], it["y"]), 8, DRAW[it["color"]], 3)
            if pending:
                cv2.drawMarker(view, pending[0], (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(view, f.name, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(win, view)
            key = cv2.waitKey(20) & 0xFF
            if key in KEYS and pending:
                items.append({"color": KEYS[key], "x": int(pending[0][0]), "y": int(pending[0][1])})
                pending.clear()
            elif key == ord("u") and items:
                items.pop()
            elif key in (ord("n"), 13, 10):
                break
            elif key == ord("q"):
                labels[f.name] = items
                (Path(folder) / "labels.yaml").write_text(yaml.safe_dump(labels))
                cv2.destroyAllWindows()
                return
        labels[f.name] = items
    (Path(folder) / "labels.yaml").write_text(yaml.safe_dump(labels))
    cv2.destroyAllWindows()
    print("saved", Path(folder) / "labels.yaml")


# ---- step 2: comparison ----

def score(found, items):
    """TP / FP / FN of one frame. A detection is correct if a label of the same colour lies inside half its long side."""
    tp = fp = 0
    matched = set()
    for color, (cx, cy, angle, w, h) in found.items():
        hits = [i for i, it in enumerate(items) if it["color"] == color and i not in matched
                and np.hypot(it["x"] - cx, it["y"] - cy) <= max(w, h) / 2]
        if hits:
            tp += 1
            matched.add(hits[0])
        else:
            fp += 1
    return tp, fp, len(items) - len(matched)


def draw(img, found, workspace=None):
    vis = img.copy()
    for color, (cx, cy, angle, w, h) in found.items():
        box = cv2.boxPoints(((cx, cy), (w, h), angle)).astype(np.intp)
        cv2.drawContours(vis, [box], 0, DRAW[color], 2)
        cv2.circle(vis, (int(cx), int(cy)), 4, DRAW[color], -1)
    if workspace is not None:
        cv2.polylines(vis, [workspace], True, (255, 255, 255), 1)
    return vis


def combined(masks):
    out = np.zeros(next(iter(masks.values())).shape, dtype=np.uint8)
    for m in masks.values():
        out = cv2.bitwise_or(out, m)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)


def run(folders, calib_file):
    detect_new, workspace = improved_detector(calib_file)
    rows = ["| folder | frame | cubes | original TP / FP / FN | improved TP / FP / FN |", "|---|---|---|---|---|"]
    totals = {}
    for folder in folders:
        labels = load_labels(folder)
        if not labels:
            print(f"{folder}: no labels.yaml, run the 'label' step first")
            continue
        out_dir = OUT / Path(folder).name
        out_dir.mkdir(parents=True, exist_ok=True)
        t = totals.setdefault(Path(folder).name, np.zeros(6, int))
        for f in frames_of(folder):
            if f.name not in labels:
                continue
            img = cv2.imread(str(f))
            old, old_masks = detect_original(img)
            new, new_masks = detect_new(img)
            so, sn = score(old, labels[f.name]), score(new, labels[f.name])
            t += np.array(so + sn)
            rows.append(f"| {Path(folder).name} | {f.name} | {len(labels[f.name])} | {so[0]} / {so[1]} / {so[2]} | "
                        f"{sn[0]} / {sn[1]} / {sn[2]} |")
            grid = np.vstack([np.hstack([draw(img, old), draw(img, new, workspace)]),
                              np.hstack([combined(old_masks), combined(new_masks)])])
            cv2.putText(grid, "original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(grid, "improved", (img.shape[1] + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imwrite(str(out_dir / f"{f.stem}_compare.png"), grid)
    rows += ["", "| folder | original TP / FP / FN | improved TP / FP / FN |", "|---|---|---|"]
    for name, t in totals.items():
        rows.append(f"| {name} | {t[0]} / {t[1]} / {t[2]} | {t[3]} / {t[4]} / {t[5]} |")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "detection_table.md").write_text("# Task B: detection, original vs improved\n\n" + "\n".join(rows) + "\n")
    print("\n".join(rows))
    print("written", OUT / "detection_table.md")


# ---- step 3: stale detections ----

def sequence(folder, calib_file):
    """Frames in time order. The original node UPDATES its dictionary, so a removed cube stays reported;
    the improved node REPLACES it every frame. Count colours reported although no such cube is labelled."""
    detect_new, _ = improved_detector(calib_file)
    labels = load_labels(folder)
    stale_old, rows = {}, ["| frame | labelled | original node reports | improved node reports |", "|---|---|---|---|"]
    fp_old = fp_new = 0
    for f in frames_of(folder):
        if f.name not in labels:
            continue
        img = cv2.imread(str(f))
        stale_old.update(detect_original(img)[0])                 # original behaviour: old entries survive
        new = detect_new(img)[0]                                  # improved behaviour: this frame only
        present = {it["color"] for it in labels[f.name]}
        fp_old += len(set(stale_old) - present)
        fp_new += len(set(new) - present)
        rows.append(f"| {f.name} | {', '.join(sorted(present)) or '-'} | {', '.join(sorted(stale_old)) or '-'} | "
                    f"{', '.join(sorted(new)) or '-'} |")
    rows.append(f"\nReported but not present: original {fp_old}, improved {fp_new}.")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"sequence_{Path(folder).name}.md").write_text("# Task B: stale detections\n\n" + "\n".join(rows) + "\n")
    print("\n".join(rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("step", choices=["label", "run", "sequence"])
    parser.add_argument("folders", nargs="+")
    parser.add_argument("--calib", help="camera_calibration.yaml: workspace polygon from Task A (approach A + B)")
    args = parser.parse_args()
    if args.step == "label":
        for folder in args.folders:
            label(folder)
    elif args.step == "run":
        run(args.folders, args.calib)
    else:
        for folder in args.folders:
            sequence(folder, args.calib)


if __name__ == "__main__":
    main()

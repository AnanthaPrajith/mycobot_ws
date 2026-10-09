#!/usr/bin/env python3
"""Lab 9, Task C: HSV thresholds from an object outlined in a real camera image (TC1-TC5).

Usage, from the repo root (any machine with a display and OpenCV; no ROS, no robot):
    python3 solutions/lab9_real_hardware/hsv_calibrate.py lab9/frames/light1_cubes/light1_cubes_000.png --name red

Window keys (TC1): left click = add a vertex, right click = remove the last vertex, Enter = finish, Esc = abort.
Then the four panels (original, polygon, mask, masked image) are shown; any key closes them.
Prints median and standard deviation, the thresholds, and an entry to paste into the COLORS dictionary of vision.py.
Saves lab9/hsv/<name>.yaml (used by evaluate_thresholds.py) and lab9/hsv/<name>_panels.png (for the report).

OpenCV conventions: images are BGR; hue 0..179 (degrees / 2, circular), saturation and value 0..255.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

LAB9 = Path(__file__).resolve().parents[2] / "lab9"
HSV_DIR = LAB9 / "hsv"
BGR_DRAW = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}


# ---- TC1: click the polygon ----

def select_polygon(img, title="outline the object: left=add, right=undo, Enter=done, Esc=abort"):
    """Let the user click the vertices of a polygon. Returns an (N, 2) int32 array, or None on Esc."""
    points = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            points.pop()

    cv2.namedWindow(title)
    cv2.setMouseCallback(title, on_mouse)
    while True:
        view = img.copy()                                   # redraw on a fresh copy every time
        if points:
            pts = np.array(points, np.int32).reshape(-1, 1, 2)
            cv2.polylines(view, [pts], False, (255, 255, 255), 1)
            for p in points:
                cv2.circle(view, p, 3, (255, 255, 255), -1)
        cv2.imshow(title, view)
        key = cv2.waitKey(20) & 0xFF
        if key in (13, 10) and len(points) >= 3:            # Enter
            break
        if key == 27:                                       # Esc
            points = []
            break
    cv2.destroyWindow(title)
    return np.array(points, np.int32) if points else None


# ---- TC2: mask of the polygon, minus a band along its edge ----

def polygon_mask(shape, polygon, band=7):
    """255 inside the polygon, 0 outside; then eroded by `band` px so edge pixels (part background) are excluded."""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [polygon.reshape(-1, 1, 2)], 255)
    if band > 0:
        mask = cv2.erode(mask, np.ones((band, band), np.uint8))
    return mask


# ---- TC3 + TC4: statistics and thresholds ----

def circular_hue(h):
    """Centre and spread of OpenCV hues, treating 0..179 as a circle (179 and 0 are neighbours).

    Each hue becomes an angle (h * 2 deg); the mean direction of the unit vectors is the centre. The spread is the
    standard deviation of the signed distances to that centre, each wrapped into -90..90.
    """
    ang = np.deg2rad(h.astype(np.float64) * 2.0)
    centre = round(float(np.rad2deg(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2.0), 6) % 180.0
    diff = (h.astype(np.float64) - centre + 90.0) % 180.0 - 90.0
    return centre, diff.std()


def estimate_thresholds(pixels, k=2.0, min_half=(0, 0, 0)):
    """HSV ranges from the selected pixels (N x 3).

    S and V: median +- k * std, clipped to 0..255.  H: circular centre +- k * spread; an interval that crosses 0/179
    becomes two ranges. min_half = smallest allowed half-width (H, S, V), to test whether +-2 sigma is too narrow.
    Returns (ranges, stats): ranges = [(lower, upper), ...] as uint8 arrays, like the COLORS dictionary of vision.py.
    """
    h, s, v = pixels[:, 0], pixels[:, 1], pixels[:, 2]
    hc, hs = circular_hue(h)
    stats = {"n_pixels": int(len(pixels)),
             "hue_centre": round(float(hc), 1), "hue_spread": round(float(hs), 2),
             "s_median": float(np.median(s)), "s_std": round(float(s.std()), 2),
             "v_median": float(np.median(v)), "v_std": round(float(v.std()), 2)}
    h_half = max(k * hs, min_half[0])
    s_half = max(k * s.std(), min_half[1])
    v_half = max(k * v.std(), min_half[2])
    s_lo, s_hi = np.clip([np.median(s) - s_half, np.median(s) + s_half], 0, 255)
    v_lo, v_hi = np.clip([np.median(v) - v_half, np.median(v) + v_half], 0, 255)
    h_lo, h_hi = int(np.floor(hc - h_half)), int(np.ceil(hc + h_half))
    if h_hi - h_lo >= 179:                                  # spread covers the whole circle: any hue
        hue_parts = [(0, 179)]
    elif h_lo < 0:                                          # e.g. -6..8 -> 174..179 and 0..8
        hue_parts = [(0, h_hi), (180 + h_lo, 179)]
    elif h_hi > 179:                                        # e.g. 172..186 -> 172..179 and 0..6
        hue_parts = [(h_lo, 179), (0, h_hi - 180)]
    else:
        hue_parts = [(h_lo, h_hi)]
    ranges = [(np.array([a, int(s_lo), int(v_lo)], np.uint8), np.array([b, int(np.ceil(s_hi)), int(np.ceil(v_hi))], np.uint8))
              for a, b in hue_parts]
    return ranges, stats


# ---- TC5: apply and print ----

def apply_thresholds(hsv, ranges):
    """Binary mask of the whole image: OR of cv2.inRange over all ranges."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def format_for_vision(name, ranges):
    """Text that can be pasted into the COLORS dictionary of vision.py."""
    lines = [f'    "{name}": {{', '        "ranges": [']
    parts = [f"            (np.array([{lo[0]}, {lo[1]}, {lo[2]}]), np.array([{hi[0]}, {hi[1]}, {hi[2]}]))" for lo, hi in ranges]
    lines.append(",\n".join(parts))
    lines += ["        ],", f'        "bgr": {BGR_DRAW.get(name, (255, 255, 255))}', "    },"]
    return "\n".join(lines)


def ranges_to_yaml(ranges):
    return [[lo.tolist(), hi.tolist()] for lo, hi in ranges]


def ranges_from_yaml(data):
    return [(np.array(lo, np.uint8), np.array(hi, np.uint8)) for lo, hi in data]


def panels(img, polygon, mask):
    """2 x 2 overview: original | polygon, mask | masked image."""
    with_poly = img.copy()
    cv2.polylines(with_poly, [polygon.reshape(-1, 1, 2)], True, (255, 255, 255), 2)
    masked = cv2.bitwise_and(img, img, mask=mask)           # only the accepted pixels keep their colour
    top = np.hstack([img, with_poly])
    bottom = np.hstack([cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), masked])
    return np.vstack([top, bottom])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", type=Path)
    parser.add_argument("--name", required=True, help="colour name, e.g. red (used for the file names and the entry)")
    parser.add_argument("--k", type=float, default=2.0, help="width in standard deviations (PDF: 2)")
    parser.add_argument("--band", type=int, default=7, help="px removed along the polygon edge (TC2)")
    args = parser.parse_args()

    img = cv2.imread(str(args.image))
    if img is None:                                         # TC1: check that loading worked
        raise SystemExit(f"cannot read {args.image}")
    polygon = select_polygon(img)
    if polygon is None or len(polygon) < 3:
        raise SystemExit("aborted: no polygon")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)              # TC2
    region = polygon_mask(img.shape, polygon, args.band)
    pixels = hsv[region > 0]                                # (N, 3): only the pixels inside the eroded polygon
    if len(pixels) == 0:
        raise SystemExit("no pixels left inside the polygon: draw a larger polygon or use a smaller --band")

    ranges, stats = estimate_thresholds(pixels, args.k)     # TC3, TC4
    mask = apply_thresholds(hsv, ranges)                    # TC5

    print(f"{stats['n_pixels']} pixels   hue centre {stats['hue_centre']} spread {stats['hue_spread']}   "
          f"S median {stats['s_median']:.0f} std {stats['s_std']}   V median {stats['v_median']:.0f} std {stats['v_std']}")
    for lo, hi in ranges:
        print(f"range: lower {lo.tolist()}  upper {hi.tolist()}")
    print("\nPaste into COLORS in vision.py:\n" + format_for_vision(args.name, ranges))

    HSV_DIR.mkdir(parents=True, exist_ok=True)
    (HSV_DIR / f"{args.name}.yaml").write_text(yaml.safe_dump(
        {"name": args.name, "source_image": str(args.image), "k": args.k, "band_px": args.band,
         "polygon": polygon.tolist(), "stats": stats, "ranges": ranges_to_yaml(ranges)}, sort_keys=False))
    overview = panels(img, polygon, mask)
    cv2.imwrite(str(HSV_DIR / f"{args.name}_panels.png"), overview)
    print(f"\nsaved {HSV_DIR / (args.name + '.yaml')} and {args.name}_panels.png")
    cv2.imshow("original | polygon / mask | masked   (any key closes)", overview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

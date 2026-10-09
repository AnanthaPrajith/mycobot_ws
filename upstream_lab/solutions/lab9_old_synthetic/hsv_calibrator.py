#!/usr/bin/env python3
"""Interactive calibration of HSV colour thresholds from a selected object (Lab 9, task "Interactive Calibration of HSV Thresholds").

Usage:   python hsv_calibrator.py path/to/image.png [--k 2.0]

Mouse: left click = add a polygon vertex, right click = remove the last vertex, Enter = finish, Esc = abort.
The program prints a `colors` entry that can be copied into vision.py.

The functions `circular_hue`, `estimate_thresholds`, `apply_thresholds` and `format_for_vision` contain no GUI code and can be
imported and tested without a display (see Lab9_PickAndPlace_MoveIt_solution.ipynb). Only `select_polygon` / `main` need a window.

OpenCV conventions: hue 0..179 (degrees / 2, wraps around), saturation and value 0..255, images are BGR.
"""

import argparse
import sys

import cv2
import numpy as np


def circular_hue(hues):
    """Centre and spread of OpenCV hue values (0..179, circular).

    The naive mean of the hues 178, 179, 1, 2 would be 90 (a completely different colour). Mapping hue h to the angle
    2*pi*h/180 and averaging the unit vectors gives the correct centre (about 0.0); the signed differences to that
    centre, wrapped into [-90, 90), give the spread.
    """
    hues = np.asarray(hues, dtype=float)
    angle = hues * 2.0 * np.pi / 180.0
    centre = np.arctan2(np.sin(angle).mean(), np.cos(angle).mean()) * 180.0 / (2.0 * np.pi) % 180.0
    diff = (hues - centre + 90.0) % 180.0 - 90.0
    return centre, diff.std()


def estimate_thresholds(bgr, polygon, erode_px=5, k=2.0, min_width_sv=30, min_half_hue=4):
    """Estimate HSV thresholds from the pixels inside `polygon` (Nx2 int array, image coordinates).

    S and V: median +/- k*sigma (sigma = standard deviation, not variance), clipped to [0, 255] and never narrower than
    `min_width_sv`. Hue: circular centre +/- k*sigma, at least +/- `min_half_hue`; an interval that crosses 0 / 179 is split
    into two ranges. Returns a dictionary with 'hue_ranges' (list of (lo, hi)), 'S', 'V' and the statistics used.
    """
    mask = np.zeros(bgr.shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.asarray(polygon, np.int32)], 255)
    if erode_px > 1:                                            # drop a band along the boundary (background pixels)
        mask = cv2.erode(mask, np.ones((erode_px, erode_px), np.uint8))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    pixels = hsv[mask > 0].astype(float)
    if len(pixels) == 0:
        raise ValueError('the selected region contains no pixels (polygon too small?)')

    result = {'n_pixels': int(len(pixels))}
    for name, channel in (('S', 1), ('V', 2)):
        median, sigma = float(np.median(pixels[:, channel])), float(pixels[:, channel].std())
        half = max(k * sigma, min_width_sv / 2.0)
        result[name] = (int(max(0, median - half)), int(min(255, median + half)))
        result[name + '_stats'] = (median, sigma)

    centre, sigma_h = circular_hue(pixels[:, 0])
    half = max(k * sigma_h, float(min_half_hue))
    low, high = centre - half, centre + half
    if low < 0:                                                 # wraps below 0: [0, high] and [180 + low, 179]
        ranges = [(0, int(np.ceil(high))), (int(np.floor(180 + low)), 179)]
    elif high > 179:                                            # wraps above 179
        ranges = [(int(np.floor(low)), 179), (0, int(np.ceil(high - 180)))]
    else:
        ranges = [(int(np.floor(low)), int(np.ceil(high)))]
    result['hue_ranges'] = ranges
    result['H_stats'] = (centre, sigma_h)
    return result


def apply_thresholds(bgr, thresholds):
    """Binary mask (uint8, 0/255) of all pixels inside the estimated thresholds (hue ranges OR-ed together)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    for hue_lo, hue_hi in thresholds['hue_ranges']:
        lower = np.array([hue_lo, thresholds['S'][0], thresholds['V'][0]])
        upper = np.array([hue_hi, thresholds['S'][1], thresholds['V'][1]])
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def format_for_vision(name, thresholds):
    """Text that can be pasted into the `colors` dictionary of vision.py."""
    lines = [f'"{name}": {{', '    "ranges": [']
    for lo, hi in thresholds['hue_ranges']:
        lines.append(f'        (np.array([{lo}, {thresholds["S"][0]}, {thresholds["V"][0]}]), '
                     f'np.array([{hi}, {thresholds["S"][1]}, {thresholds["V"][1]}])),')
    lines += ['    ],', '},']
    return '\n'.join(lines)


def select_polygon(image, window='select object'):
    """Let the user click polygon vertices. Returns an Nx2 int array or None (Esc / fewer than 3 vertices)."""
    points = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            points.pop()

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    while True:
        view = image.copy()                                     # draw on a copy: the original pixels stay untouched
        if len(points) > 1:
            cv2.polylines(view, [np.array(points, np.int32)], False, (0, 255, 0), 2)
        for p in points:
            cv2.circle(view, p, 3, (0, 0, 255), -1)
        cv2.putText(view, 'left: add  right: undo  Enter: done  Esc: abort', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow(window, view)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10) and len(points) >= 3:
            break
        if key == 27:
            points = []
            break
    cv2.destroyWindow(window)
    return np.array(points, np.int32) if len(points) >= 3 else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('image')
    parser.add_argument('--k', type=float, default=2.0, help='half width of the interval in standard deviations')
    parser.add_argument('--name', default='colour')
    args = parser.parse_args(argv)

    image = cv2.imread(args.image)
    if image is None:
        print(f'could not read {args.image}')
        return 1
    polygon = select_polygon(image)
    if polygon is None:
        print('no polygon selected')
        return 1

    thresholds = estimate_thresholds(image, polygon, k=args.k)
    mask = apply_thresholds(image, thresholds)
    outline = image.copy()
    cv2.polylines(outline, [polygon], True, (0, 255, 0), 2)
    masked = cv2.bitwise_and(image, image, mask=mask)
    panel = np.hstack([outline, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), masked])
    print(f"{thresholds['n_pixels']} pixels used")
    print('H centre / sigma: %.1f / %.1f' % thresholds['H_stats'])
    print('S median / sigma: %.1f / %.1f' % thresholds['S_stats'])
    print('V median / sigma: %.1f / %.1f' % thresholds['V_stats'])
    print(format_for_vision(args.name, thresholds))
    cv2.imshow('polygon | mask | masked image', panel)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())

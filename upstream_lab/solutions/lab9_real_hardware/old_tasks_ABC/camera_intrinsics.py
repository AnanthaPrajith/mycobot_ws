#!/usr/bin/env python3
"""Lab 9, Task A, step TA1: camera intrinsics K and distortion coefficients d from checkerboard images.

Usage, from the repo root (any machine with OpenCV; no ROS, no robot):
    python3 solutions/lab9_real_hardware/camera_intrinsics.py lab9/frames/checkerboard/*.png --cols 9 --rows 6 --square 0.025
    python3 solutions/lab9_real_hardware/camera_intrinsics.py --from-fov 90 --size 640 480     # rough fallback, no checkerboard

--cols / --rows count the INNER corners (where four squares meet), not the squares: a board of 10 x 7 squares has 9 x 6.
--square is the side of one square in metres, measured on the print (or on the screen showing the board).
The images must have exactly the resolution that vision.py receives (640x480, run guide D2).

Output: lab9/calib/intrinsics.yaml, lab9/calib/intrinsics_check/*.png (found corners drawn), lab9/calib/undistorted_example.png
"""
import argparse
import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

LAB9 = Path(__file__).resolve().parents[2] / "lab9"
CALIB = LAB9 / "calib"


def find_corners(img, pattern):
    """Inner checkerboard corners with sub-pixel accuracy, shape (N, 1, 2); None if the board is not found."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)    # stop after 30 steps or 0.001 px change
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def board_points(cols, rows, square):
    """3D corner positions on the flat board (z = 0), metres, in the order findChessboardCorners returns them."""
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square              # (0,0), (s,0), (2s,0), ... row by row
    return pts


def calibrate(files, cols, rows, square):
    check_dir = CALIB / "intrinsics_check"
    check_dir.mkdir(parents=True, exist_ok=True)
    obj_pts, img_pts, used, size = [], [], [], None
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"  {f}: cannot read, skipped")
            continue
        h, w = img.shape[:2]
        if size is None:
            size = (w, h)
        elif (w, h) != size:
            raise SystemExit(f"{f}: {w}x{h}, but the first image is {size[0]}x{size[1]}. Use one resolution only.")
        corners = find_corners(img, (cols, rows))
        if corners is None:
            print(f"  {f.name}: board not found, skipped")
            continue
        obj_pts.append(board_points(cols, rows, square))
        img_pts.append(corners)
        used.append(f)
        vis = img.copy()
        cv2.drawChessboardCorners(vis, (cols, rows), corners, True)
        cv2.imwrite(str(check_dir / f.name), vis)
    if len(used) < 10:
        raise SystemExit(f"only {len(used)} usable images: take at least 10, better 15-20, with the board in different "
                         "places and tilts (corners of the image too)")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)

    per_image = []                                       # mean reprojection error of each image, px
    for o, i, r, t in zip(obj_pts, img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, dist)
        per_image.append(float(np.linalg.norm(proj - i, axis=2).mean()))
    return rms, K, dist.ravel(), size, used, per_image


def from_fov(fov_deg, w, h):
    """Rough K from the diagonal field of view (pinhole, principal point in the centre, no distortion)."""
    f = 0.5 * np.hypot(w, h) / np.tan(np.deg2rad(fov_deg) / 2)
    K = np.array([[f, 0.0, w / 2], [0.0, f, h / 2], [0.0, 0.0, 1.0]])
    return K, np.zeros(5)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="*", type=Path)
    parser.add_argument("--cols", type=int, default=9, help="inner corners per row")
    parser.add_argument("--rows", type=int, default=6, help="inner corners per column")
    parser.add_argument("--square", type=float, default=0.025, help="square side in metres")
    parser.add_argument("--from-fov", type=float, help="fallback: diagonal field of view in degrees (C930e: 90)")
    parser.add_argument("--size", type=int, nargs=2, metavar=("W", "H"), default=(640, 480))
    args = parser.parse_args()

    data = {"created": datetime.datetime.now().isoformat(timespec="seconds"),
            "units": "K in pixels; dist = OpenCV model [k1, k2, p1, p2, k3], dimensionless"}
    if args.from_fov:
        K, dist = from_fov(args.from_fov, *args.size)
        data.update(image_width=args.size[0], image_height=args.size[1], approximate=True,
                    method=f"from field of view {args.from_fov} deg, no distortion (rough; say so in the report)")
        print("WARNING: approximate intrinsics. Positions will be less accurate than with a checkerboard calibration.")
    else:
        if not args.images:
            parser.error("give checkerboard images, or --from-fov")
        rms, K, dist, size, used, per_image = calibrate(args.images, args.cols, args.rows, args.square)
        data.update(image_width=size[0], image_height=size[1], approximate=False,
                    method=f"checkerboard {args.cols}x{args.rows} inner corners, square {args.square} m",
                    rms_px=float(rms), n_images=len(used),
                    per_image_px={f.name: round(e, 3) for f, e in zip(used, per_image)})
        print(f"used {len(used)} images, RMS reprojection error {rms:.3f} px   (< 0.5 good, < 1.0 acceptable)")
        for f, e in zip(used, per_image):
            print(f"  {f.name}: {e:.3f} px" + ("   <- high: blurred or moved? remove it and run again" if e > 1.0 else ""))
        undist = cv2.undistort(cv2.imread(str(used[0])), K, dist)
        cv2.imwrite(str(CALIB / "undistorted_example.png"), undist)     # straight edges should look straight

    data["K"] = np.round(K, 4).tolist()
    data["dist"] = np.round(np.ravel(dist), 6).tolist()
    CALIB.mkdir(parents=True, exist_ok=True)
    out = CALIB / "intrinsics.yaml"
    out.write_text("# Lab 9 Task A, TA1: camera intrinsics, written by camera_intrinsics.py\n"
                   + yaml.safe_dump(data, sort_keys=False))
    np.set_printoptions(precision=2, suppress=True)
    print("K =\n", K, "\nd =", np.ravel(dist))
    print("written", out)


if __name__ == "__main__":
    main()

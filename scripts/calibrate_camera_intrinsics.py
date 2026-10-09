#!/usr/bin/env python3
"""Fit intrinsics from saved chessboard images; dimensions count INNER corners."""
import argparse
import json
import glob
from pathlib import Path
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('images', type=Path, nargs='+')
    p.add_argument('--columns', type=int, required=True)
    p.add_argument('--rows', type=int, required=True)
    p.add_argument('--square-mm', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if min(a.columns, a.rows) < 3 or a.square_mm <= 0:
        p.error('Use at least 3 inner corners per axis and a positive square size.')
    images = []
    for pattern in a.images:
        matches = sorted(glob.glob(str(pattern)))
        if not matches:
            p.error(f'No images match {pattern}. Capture chessboard photos first with scripts/capture_calibration_images.py.')
        images.extend(Path(name) for name in matches)
    images = list(dict.fromkeys(images))
    obj = np.zeros((a.rows*a.columns, 3), np.float32)
    obj[:, :2] = np.mgrid[:a.columns, :a.rows].T.reshape(-1, 2)*a.square_mm
    objects, pixels, accepted, size = [], [], [], None
    for path in images:
        im = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if im is None:
            raise ValueError(f'Cannot read {path}')
        wh = im.shape[::-1]
        if size is not None and size != wh:
            raise ValueError('All images must have the same resolution.')
        size = wh
        ok, corners = cv2.findChessboardCorners(im, (a.columns, a.rows))
        if ok:
            corners = cv2.cornerSubPix(im, corners, (11,11), (-1,-1),
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
            objects.append(obj.copy()); pixels.append(corners); accepted.append(str(path))
    if len(objects) < 12:
        raise ValueError(f'Only {len(objects)} boards detected; collect at least 12 varied views.')
    rms, k, d, rs, ts = cv2.calibrateCamera(objects, pixels, size, None, None)
    errors = []
    for o, xy, r, t in zip(objects, pixels, rs, ts):
        projected = cv2.projectPoints(o, r, t, k, d)[0]
        errors.append(float(np.sqrt(np.mean(np.sum((projected-xy)**2, axis=2)))))
    result = dict(camera_matrix=k.tolist(), dist_coeffs=d.ravel().tolist(),
        intrinsics_resolution=list(size), rms_px=float(rms), per_view_rms_px=errors,
        images=accepted, square_mm=a.square_mm)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(f'{len(objects)} views; RMS {rms:.3f} px; saved {a.output}')

if __name__ == '__main__':
    main()

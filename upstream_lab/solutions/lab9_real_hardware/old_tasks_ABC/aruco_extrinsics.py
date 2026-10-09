#!/usr/bin/env python3
"""Lab 9, Task A, steps TA2, TA3 and TA6 (reprojection + consistency): camera pose from two ArUco markers.

Usage, from the repo root (any machine with OpenCV 4.6 or newer; no ROS, no robot):
    python3 solutions/lab9_real_hardware/aruco_extrinsics.py lab9/frames/markers/*.png

Inputs:
    solutions/lab9_real_hardware/markers.yaml    TA2: dictionary, marker size, marker poses in B, camera height, workspace
    lab9/calib/intrinsics.yaml                    TA1: K and d (camera_intrinsics.py)
Outputs:
    lab9/calib/camera_calibration.yaml            everything vision_AB.py needs (K, d, R_BC, t_BC, heights, workspace)
    lab9/calib/annotated_<image>.png              detected corners (green), reprojected corners (red), B axes, workspace
    lab9/calib/extrinsics_report.md               per-image and final errors (the TA6 table)

Convention: p_C = R_CB p_B + t_CB (what solvePnP returns) and its inverse p_B = R_BC p_C + t_BC, with R_BC = R_CB^T and
t_BC = -R_CB^T t_CB. B = robot base frame g_base (x forward, y left, z up); C = OpenCV camera frame (x right, y down,
z along the optical axis). Metres, radians, pixels.
An image in which a configured marker is missing is skipped (two markers = 8 corners are needed for a reliable pose).
If no image is usable, nothing is written and the previous calibration file stays as it was.
"""
import argparse
import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
LAB9 = HERE.parents[1] / "lab9"
CALIB = LAB9 / "calib"


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def marker_corners_B(marker, size, plane_z):
    """The 4 corners of one marker in frame B, in ArUco order: top-left, top-right, bottom-right, bottom-left."""
    s = size / 2
    local = np.array([[-s, s], [s, s], [s, -s], [-s, -s]])          # marker frame: x right, y up (pattern upright)
    a = np.deg2rad(marker["yaw_deg"])
    Rz = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])  # rotation by yaw about z_B
    xy = local @ Rz.T + [marker["x"], marker["y"]]                    # rotate each corner, then move to the centre
    return np.column_stack([xy, np.full(4, plane_z)])                 # all corners lie in the marker plane


def detect_markers(img, dictionary_name):
    """{id: (4, 2) corner array} for every marker found. Works with OpenCV 4.6 (old API) and 4.7+ (new API)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, "ArucoDetector"):                           # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:                                                             # OpenCV 4.6 (Ubuntu 24.04 python3-opencv)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}


def correspondences(found, setup):
    """Stack the 3D corners (B) and the detected 2D corners of all configured markers. Returns obj, img, missing ids."""
    obj, img, missing = [], [], []
    for m in setup["markers"]:
        if m["id"] not in found:
            missing.append(m["id"])
            continue
        obj.append(marker_corners_B(m, setup["marker_size"], setup["plane_z"]))
        img.append(found[m["id"]])
    if not obj:
        return None, None, missing
    return np.vstack(obj).astype(np.float64), np.vstack(img).astype(np.float64), missing


def invert(rvec, tvec):
    """solvePnP gives the base->camera transform (R_CB, t_CB). Return camera->base: R_BC, t_BC (camera centre in B)."""
    R_CB, _ = cv2.Rodrigues(rvec)
    R_BC = R_CB.T
    t_BC = -R_BC @ np.ravel(tvec)
    return R_BC, t_BC


def solve_pose(obj, img, K, dist, plane_z, h_measured):
    """Camera pose from coplanar points. IPPE returns the two poses a flat target allows; keep the physical one."""
    _, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    best = None
    for rvec, tvec, err in zip(rvecs, tvecs, np.ravel(errs)):
        R_BC, t_BC = invert(rvec, tvec)
        height = t_BC[2] - plane_z                                    # camera above the marker plane?
        looks_down = (R_BC @ [0.0, 0.0, 1.0])[2] < 0                   # optical axis z_C must point downwards in B
        if height <= 0 or not looks_down:
            continue
        score = abs(height - h_measured) if h_measured > 0 else err    # closest to the measured height, else best fit
        if best is None or score < best[0]:
            best = (score, rvec, tvec)
    if best is None:
        return None, None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, best[1], best[2])   # polish with Levenberg-Marquardt
    return rvec, tvec


def reprojection(obj, img, rvec, tvec, K, dist):
    """Project the known 3D corners with the estimated pose; return the projected points and the error per corner (px)."""
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    return proj, np.linalg.norm(proj - img, axis=1)


def rotation_angle_deg(Ra, Rb):
    """Angle of the rotation that turns Ra into Rb (0 = identical orientation)."""
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def annotate(img, found, setup, proj, rvec, tvec, K, dist):
    vis = img.copy()
    for mid, c in found.items():
        for p in c:
            cv2.circle(vis, tuple(int(v) for v in p), 5, (0, 255, 0), 2)                   # detected: green circle
        cv2.putText(vis, f"id {mid}", tuple(int(v) for v in c[0] + [0, -10]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    for p in proj:
        cv2.drawMarker(vis, tuple(int(round(v)) for v in p), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)   # reprojected: red +
    ws, z = setup["workspace"], setup["plane_z"]
    xm, ym = (ws["x_min"] + ws["x_max"]) / 2, (ws["y_min"] + ws["y_max"]) / 2
    axes = np.array([[xm, ym, z], [xm + 0.05, ym, z], [xm, ym + 0.05, z]])            # 5 cm arrows along x_B and y_B
    a, _ = cv2.projectPoints(axes, rvec, tvec, K, dist)
    a = [tuple(int(v) for v in p) for p in a.reshape(-1, 2)]
    cv2.arrowedLine(vis, a[0], a[1], (0, 0, 255), 2)
    cv2.putText(vis, "x_B", a[1], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.arrowedLine(vis, a[0], a[2], (0, 200, 0), 2)
    cv2.putText(vis, "y_B", a[2], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
    zt = setup["plane_z"] + setup["cube_height"]
    rect = np.array([[ws["x_min"], ws["y_min"], zt], [ws["x_max"], ws["y_min"], zt],
                     [ws["x_max"], ws["y_max"], zt], [ws["x_min"], ws["y_max"], zt]])
    r, _ = cv2.projectPoints(rect, rvec, tvec, K, dist)
    cv2.polylines(vis, [r.reshape(-1, 1, 2).astype(np.int32)], True, (255, 0, 0), 1)   # workspace at cube-top height
    return vis


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="+", type=Path, help="frames of the markers (camera at its final position)")
    parser.add_argument("--setup", type=Path, default=HERE / "markers.yaml")
    parser.add_argument("--intrinsics", type=Path, default=CALIB / "intrinsics.yaml")
    args = parser.parse_args()

    setup = load_yaml(args.setup)
    intr = load_yaml(args.intrinsics)
    K = np.array(intr["K"], dtype=np.float64)
    dist = np.array(intr["dist"], dtype=np.float64)
    h_meas = float(setup.get("camera_height") or 0.0)
    pz = float(setup["plane_z"])

    rows, obj_all, img_all, used = [], [], [], []
    for f in args.images:
        img = cv2.imread(str(f))
        if img is None:
            print(f"{f}: cannot read, skipped")
            continue
        if (img.shape[1], img.shape[0]) != (intr["image_width"], intr["image_height"]):
            raise SystemExit(f"{f.name}: {img.shape[1]}x{img.shape[0]}, but K is for "
                             f"{intr['image_width']}x{intr['image_height']}. Calibrate at the vision resolution.")
        found = detect_markers(img, setup["dictionary"])
        obj, pts, missing = correspondences(found, setup)
        if missing:
            print(f"{f.name}: marker(s) {missing} not found (found {sorted(found)}), image skipped")
            continue
        rvec, tvec = solve_pose(obj, pts, K, dist, pz, h_meas)
        if rvec is None:
            print(f"{f.name}: no pose with the camera above the plane, image skipped (check markers.yaml)")
            continue
        R_BC, t_BC = invert(rvec, tvec)
        _, err = reprojection(obj, pts, rvec, tvec, K, dist)
        rows.append(dict(image=f.name, R_BC=R_BC, t_BC=t_BC, mean=err.mean(), max=err.max()))
        obj_all.append(obj)
        img_all.append(pts)
        used.append((f, img, found))

    if not rows:
        raise SystemExit("no usable image: nothing written (the previous calibration file is unchanged)")

    # Final pose: the camera and the markers did not move between the frames, so all corners of all frames
    # describe the same scene. One solvePnP over all of them averages out the per-frame detection noise.
    obj, pts = np.vstack(obj_all), np.vstack(img_all)
    rvec, tvec = solve_pose(obj, pts, K, dist, pz, h_meas)
    R_BC, t_BC = invert(rvec, tvec)
    _, err = reprojection(obj, pts, rvec, tvec, K, dist)
    h_est = float(t_BC[2] - pz)

    CALIB.mkdir(parents=True, exist_ok=True)
    for f, img, found in used:
        o, p, _ = correspondences(found, setup)
        proj, _ = reprojection(o, p, rvec, tvec, K, dist)
        cv2.imwrite(str(CALIB / f"annotated_{f.stem}.png"), annotate(img, found, setup, proj, rvec, tvec, K, dist))

    lines = ["# Task A: extrinsic calibration report", "",
             f"Created {datetime.datetime.now().isoformat(timespec='seconds')} from {len(rows)} image(s); "
             f"intrinsics: {intr.get('method', '?')}.", "",
             "| image | reproj. mean px | reproj. max px | camera x mm | y mm | z mm | angle to final ° |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        x, y, z = r["t_BC"] * 1000
        lines.append(f"| {r['image']} | {r['mean']:.2f} | {r['max']:.2f} | {x:.1f} | {y:.1f} | {z:.1f} | "
                     f"{rotation_angle_deg(r['R_BC'], R_BC):.2f} |")
    T = np.array([r["t_BC"] for r in rows]) * 1000
    lines += ["", f"**Final (all images together):** reprojection mean {err.mean():.2f} px, max {err.max():.2f} px; "
              f"camera at ({t_BC[0]*1000:.1f}, {t_BC[1]*1000:.1f}, {t_BC[2]*1000:.1f}) mm in B.",
              f"**Consistency:** spread (std) of the camera position over the images: "
              f"x {T[:, 0].std():.1f} mm, y {T[:, 1].std():.1f} mm, z {T[:, 2].std():.1f} mm.",
              f"**Height check:** estimated {h_est*1000:.1f} mm above the marker plane, measured "
              + (f"{h_meas*1000:.1f} mm, difference {abs(h_est - h_meas)*1000:.1f} mm." if h_meas > 0 else "not given.")]
    report = "\n".join(lines) + "\n"
    (CALIB / "extrinsics_report.md").write_text(report)
    print(report)
    if h_meas > 0 and abs(h_est - h_meas) > 0.01:
        print("WARNING: estimated and measured camera height differ by more than 1 cm. "
              "Check marker_size, the marker poses and camera_height in markers.yaml before using this calibration.")
    if err.max() > 2.0:
        print("WARNING: a corner reprojects more than 2 px away: check the marker poses (id swapped? yaw wrong?).")

    out = {"created": datetime.datetime.now().isoformat(timespec="seconds"),
           "convention": "p_B = R_BC p_C + t_BC; p_C = R_CB p_B + t_CB (rvec_CB, tvec_CB as from solvePnP). "
                         "B = g_base (x forward, y left, z up), C = OpenCV camera (x right, y down, z forward). "
                         "Metres, radians, pixels.",
           "image_width": intr["image_width"], "image_height": intr["image_height"],
           "approximate_intrinsics": bool(intr.get("approximate", False)),
           "K": intr["K"], "dist": intr["dist"],
           "R_BC": np.round(R_BC, 6).tolist(), "t_BC": np.round(t_BC, 5).tolist(),
           "rvec_CB": np.round(np.ravel(rvec), 6).tolist(), "tvec_CB": np.round(np.ravel(tvec), 5).tolist(),
           "plane_z": pz, "cube_height": float(setup["cube_height"]), "z_top": pz + float(setup["cube_height"]),
           "camera_height_measured": h_meas, "camera_height_estimated": round(h_est, 4),
           "reprojection_px": {"mean": round(float(err.mean()), 3), "max": round(float(err.max()), 3)},
           "workspace": setup["workspace"],
           "images": [r["image"] for r in rows]}
    path = CALIB / "camera_calibration.yaml"
    path.write_text("# Lab 9 Task A: camera calibration, written by aruco_extrinsics.py\n"
                    + yaml.safe_dump(out, sort_keys=False))
    print("written", path)


if __name__ == "__main__":
    main()

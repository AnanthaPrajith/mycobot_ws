#!/usr/bin/env python3
"""Lab 9, Task A step TA6 and the Part 0 baseline: position error of /cube_coordinates against ruler measurements.

Run on the Lab PC, from the repo root, with camera + vision running (run guide B4, B5); the robot is not needed:
    source ~/rosenv9.sh
    python3 solutions/lab9_real_hardware/position_errors.py baseline --color red      # original vision.py (Part 0)
    python3 solutions/lab9_real_hardware/position_errors.py calibrated --color red    # after Task A
    python3 solutions/lab9_real_hardware/position_errors.py calibrated --summary       # only print the table again

For each position: place the cube, measure its centre from the robot base origin with a ruler (x forward, y left),
then type "x y" in millimetres, optionally followed by a note (e.g. "150 0 centre", "250 -100 edge").
The script asks vision once and appends a row to lab9/tables/position_errors_<label>.csv.
Empty line = finish and print mean / max. Use the SAME positions for every label, so the tables can be compared.
This script only calls a service; it never moves the robot.
"""
import argparse
import csv
from pathlib import Path

import numpy as np

LAB9 = Path(__file__).resolve().parents[2] / "lab9"
NOT_DETECTED = [1.0] * 6                                    # vision's answer when the colour is not seen
FIELDS = ["label", "color", "x_meas_mm", "y_meas_mm", "x_ret_mm", "y_ret_mm", "dx_mm", "dy_mm", "err_mm", "note"]


def ask_vision(node, client, color):
    """One call of /cube_coordinates. Returns the 6 numbers, or None if the service did not answer."""
    import rclpy
    from mycobot_interfaces.srv import GetCubeCoords
    request = GetCubeCoords.Request()
    request.color = color
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    return list(future.result().coords) if future.done() and future.result() else None


def summary(path):
    """Print mean / max error of one CSV as a markdown table."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    ok = [r for r in rows if r["err_mm"] != ""]
    print(f"\n{path.name}: {len(rows)} positions, {len(ok)} detected, {len(rows) - len(ok)} not detected")
    if not ok:
        return
    e = np.array([float(r["err_mm"]) for r in ok])
    dx = np.array([float(r["dx_mm"]) for r in ok])
    dy = np.array([float(r["dy_mm"]) for r in ok])
    print("| n | mean error mm | max error mm | mean dx mm | mean dy mm |")
    print("|---|---|---|---|---|")
    print(f"| {len(ok)} | {e.mean():.1f} | {e.max():.1f} | {dx.mean():+.1f} | {dy.mean():+.1f} |")
    print("(mean dx / dy far from 0 = a constant offset: the mapping is shifted; scattered signs = noise or scale)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("label", help="name of this measurement series, e.g. baseline or calibrated")
    parser.add_argument("--color", default="red")
    parser.add_argument("--summary", action="store_true", help="only print the table of an existing CSV")
    args = parser.parse_args()

    path = LAB9 / "tables" / f"position_errors_{args.label}.csv"
    if args.summary:
        summary(path)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists()

    import rclpy
    from rclpy.node import Node
    from mycobot_interfaces.srv import GetCubeCoords
    rclpy.init()
    node = Node("position_errors")
    client = node.create_client(GetCubeCoords, "/cube_coordinates")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("/cube_coordinates not available: is vision running, and is ~/rosenv9.sh sourced?")

    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if new_file:
            writer.writeheader()
        while True:
            line = input(f"[{args.label}] place the {args.color} cube, type measured 'x y [note]' in mm (Enter = done): ")
            if not line.strip():
                break
            parts = line.replace(",", " ").split()
            try:
                xm, ym = float(parts[0]), float(parts[1])
            except (ValueError, IndexError):
                print("  type two numbers, e.g. 150 0")
                continue
            note = " ".join(parts[2:])
            coords = ask_vision(node, client, args.color)
            row = dict(label=args.label, color=args.color, x_meas_mm=xm, y_meas_mm=ym, note=note,
                       x_ret_mm="", y_ret_mm="", dx_mm="", dy_mm="", err_mm="")
            if coords is None or coords == NOT_DETECTED:
                print("  not detected (no rectangle on the cube in the vision window?)")
                row["note"] = (note + " not detected").strip()
            else:
                xr, yr = coords[0] * 1000, coords[1] * 1000                 # service answers in metres
                dx, dy = xr - xm, yr - ym
                row.update(x_ret_mm=round(xr, 1), y_ret_mm=round(yr, 1), dx_mm=round(dx, 1), dy_mm=round(dy, 1),
                           err_mm=round(float(np.hypot(dx, dy)), 1))
                print(f"  vision: ({xr:.1f}, {yr:.1f}) mm   error {row['err_mm']} mm   (dx {dx:+.1f}, dy {dy:+.1f})")
            writer.writerow(row)
            f.flush()
    node.destroy_node()
    rclpy.shutdown()
    summary(path)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Lab 9 evaluation logger: turns live queries and vision logs into CSV files and report tables.

Writes to <repo>/lab9/eval/ (queries.csv, features.csv). It never moves the robot: it only calls /get_object
and reads log files.

1) Queries (Task "colour and shape" item 5 + evaluation: existing / absent / several matches). Lab PC, vision_lab9 running:
       source ~/rosenv9.sh
       python3 solutions/lab9_real_hardware/lab9_eval_logger.py query scene1
   asks /get_object for every colour x shape (and "any colour") and labels each answer:
   found with 1 match = existing, found with >= 2 matches = several, not found = absent.
       python3 solutions/lab9_real_hardware/lab9_eval_logger.py query scene1 --color red --shape cylinder --case "red cylinder, 2 on the plate"
   asks one request (add --id red_cylinder_2 to select a specific match).

2) Shape features (Task "colour and shape" item 3: thresholds from images of the actual camera). vision_lab9 prints one
   line per scene change: "objects: red_cube_1 (180, 41) C 0.79 fill 0.62 v 4, ...". Save that output, e.g.
       ros2 run lab9_pick_place vision_lab9 2>&1 | tee lab9/eval/vision_scene1.log
   (or use the node's ROS log file in ~/.ros/log/), then on any machine (no ROS needed):
       python3 solutions/lab9_real_hardware/lab9_eval_logger.py features lab9/eval/vision_scene1.log --scene scene1
   prints C / fill / vertices per detected shape next to the thresholds in lab9.yaml.

3) Report tables from everything logged so far:
       python3 solutions/lab9_real_hardware/lab9_eval_logger.py summary
"""
import argparse
import csv
import datetime
import re
import statistics
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
EVAL = REPO / "lab9" / "eval"
CONFIG = REPO / "pp_moveit_ws" / "src" / "lab9_pick_place" / "config" / "lab9.yaml"
COLORS = ["red", "yellow", "green", "blue"]
SHAPES = ["cube", "cylinder"]
Q_FIELDS = ["time", "scene", "case", "req_color", "req_shape", "req_id", "found", "id", "shape",
            "x_mm", "y_mm", "z_top_mm", "yaw_deg", "height_mm", "n_matches", "matches"]
F_FIELDS = ["scene", "log_time", "id", "color", "shape", "x_mm", "y_mm", "C", "fill", "v"]
OBJ = re.compile(r"([a-z]+)_(cube|cylinder|unknown)_(\d+) \((-?\d+), (-?\d+)\) C ([\d.]+) fill ([\d.]+) v (\d+)")
STAMP = re.compile(r"(\d{10}\.\d+)")


def append(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if new:
            w.writeheader()
        w.writerows(rows)


def read(path):
    if not path.exists():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


# ---------------- 1) queries ----------------

def query(args):
    import rclpy
    from rclpy.node import Node
    from lab9_interfaces.srv import GetObject

    rclpy.init()
    node = Node("lab9_eval_logger")
    client = node.create_client(GetObject, "/get_object")
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit("/get_object not available: is vision_lab9 running, and ~/rosenv9.sh sourced?")

    if args.color is not None or args.shape is not None or args.id:
        requests = [(args.color or "", args.shape or "", args.id or "")]
    else:                                   # the standard set: every colour x shape, then any colour per shape
        requests = [(c, s, "") for c in COLORS for s in SHAPES] + [("", s, "") for s in SHAPES] + [("", "", "")]

    rows = []
    for color, shape, obj_id in requests:
        req = GetObject.Request()
        req.color, req.shape, req.id = color, shape, obj_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        res = future.result() if future.done() else None
        if res is None:
            print(f"  {color or 'any'} {shape or 'any'}: no answer")
            continue
        n = len(res.matches)
        case = args.case or ("absent" if not res.found else "several" if n >= 2 else "existing")
        x, y, z, _, _, yaw = list(res.coords) if res.found else [None] * 6
        rows.append(dict(time=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), scene=args.scene, case=case,
                         req_color=color, req_shape=shape, req_id=obj_id, found=res.found,
                         id=res.id if res.found else "", shape=res.shape if res.found else "",
                         x_mm=round(x * 1000) if res.found else "", y_mm=round(y * 1000) if res.found else "",
                         z_top_mm=round(z * 1000) if res.found else "",
                         yaw_deg=round(float(yaw) * 57.29578) if res.found else "",
                         height_mm=round(res.height * 1000) if res.found else "",
                         n_matches=n, matches=" ".join(res.matches)))
    node.destroy_node()
    rclpy.shutdown()
    append(EVAL / "queries.csv", Q_FIELDS, rows)
    print_queries(rows)
    print(f"appended {len(rows)} rows to {EVAL / 'queries.csv'}")


def print_queries(rows):
    print("| scene | request (colour, shape, id) | case | answer id | position (mm) | matches |")
    print("|---|---|---|---|---|---|")
    for r in rows:
        req = f"{r['req_color'] or 'any'}, {r['req_shape'] or 'any'}" + (f", {r['req_id']}" if r["req_id"] else "")
        pos = f"({r['x_mm']}, {r['y_mm']})" if str(r["found"]) == "True" else "–"
        print(f"| {r['scene']} | {req} | {r['case']} | {r['id'] or 'not found'} | {pos} | {r['matches'] or '–'} |")


# ---------------- 2) features ----------------

def features(args):
    rows = []
    for line in Path(args.logfile).read_text(errors="replace").splitlines():
        if "objects:" not in line:
            continue
        m = STAMP.search(line)
        stamp = datetime.datetime.fromtimestamp(float(m.group(1))).strftime("%H:%M:%S") if m else ""
        for color, shape, num, x, y, c, fill, v in OBJ.findall(line.split("objects:", 1)[1]):
            rows.append(dict(scene=args.scene, log_time=stamp, id=f"{color}_{shape}_{num}", color=color, shape=shape,
                             x_mm=int(x), y_mm=int(y), C=float(c), fill=float(fill), v=int(v)))
    if not rows:
        raise SystemExit(f"no 'objects:' lines with features found in {args.logfile}")
    append(EVAL / "features.csv", F_FIELDS, rows)
    print(f"appended {len(rows)} object samples to {EVAL / 'features.csv'}")
    print_features(rows)


def print_features(rows):
    cfg = yaml.safe_load(CONFIG.read_text())["shape"] if CONFIG.exists() else None
    print("\n| detected as | n | C min / mean / max | fill min / mean / max | vertices |")
    print("|---|---|---|---|---|")
    for shape in ("cube", "cylinder", "unknown"):
        s = [r for r in rows if r["shape"] == shape]
        if not s:
            continue
        C = [float(r["C"]) for r in s]
        F = [float(r["fill"]) for r in s]
        V = sorted({int(r["v"]) for r in s})
        print(f"| {shape} | {len(s)} | {min(C):.2f} / {statistics.mean(C):.2f} / {max(C):.2f} | "
              f"{min(F):.2f} / {statistics.mean(F):.2f} / {max(F):.2f} | {V} |")
    print("| theory: square | | 0.785 (π/4) | 0.64 (2/π) | 4 |")
    print("| theory: circle | | 1.00 | 1.00 | many |")
    if cfg:
        cyl, cube = cfg["cylinder"], cfg["cube"]
        print(f"\nthresholds in lab9.yaml: cylinder if C >= {cyl['circularity_min']} and fill >= {cyl['fill_min']}; "
              f"cube if C <= {cube['circularity_max']}, fill <= {cube['fill_max']}, vertices {cube['vertices']}; "
              "else unknown")
    unknown = [r for r in rows if r["shape"] == "unknown"]
    if unknown:
        print("\nuncertain cases (classified unknown):")
        for r in unknown:
            print(f"  {r['scene']} {r['log_time']} {r['id']} at ({r['x_mm']}, {r['y_mm']}) mm: "
                  f"C {r['C']}, fill {r['fill']}, v {r['v']}")
    print("Note: the vision log prints each object again whenever the scene changes, so n counts samples, not objects.")


# ---------------- 3) summary ----------------

def summary(args):
    q, f = read(EVAL / "queries.csv"), read(EVAL / "features.csv")
    print(f"# Lab 9 evaluation tables ({EVAL})\n")
    if f:
        scenes = sorted({r["scene"] for r in f})
        print(f"## Shape features ({len(f)} samples, scenes: {', '.join(scenes)})")
        print_features(f)
    else:
        print("no features.csv yet")
    print()
    if q:
        print(f"## Queries ({len(q)})")
        print_queries(q)
        cases = {c: sum(1 for r in q if r["case"] == c) for c in ("existing", "absent", "several")}
        print(f"\ncases covered: {cases}  (the PDF asks for all three)")
    else:
        print("no queries.csv yet")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("query", help="ask /get_object and log the answers")
    p.add_argument("scene", help="label of the object arrangement, e.g. scene1")
    p.add_argument("--color")
    p.add_argument("--shape")
    p.add_argument("--id")
    p.add_argument("--case", help="own label instead of the automatic existing / absent / several")
    p.set_defaults(func=query)
    p = sub.add_parser("features", help="parse a vision_lab9 log into features.csv")
    p.add_argument("logfile")
    p.add_argument("--scene", default="")
    p.set_defaults(func=features)
    p = sub.add_parser("summary", help="print all report tables")
    p.set_defaults(func=summary)
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

"""Lab 9 vision node for the real camera (tasks: object shape detection + object interface extension).

camera/image -> calibration from 2 ArUco marker centres (PlaneMap, frame g_base) -> per-colour HSV masks -> contours
-> shape cube / cylinder / unknown (circularity, approxPolyDP, fill) -> new object list every frame.

Services:
  /get_object        GetObject     colour + shape (+ id) -> found, id, pose of the object top, matches
  /cube_coordinates  GetCubeCoords original interface (colour -> nearest cube), so the original brain still runs
Window keys: c = re-read the markers, s = save raw + annotated frame to lab9_pick_place/frames/ (next to LAB9_REPORT.md)
Config: parameter "config" (default lab9_pick_place/config/lab9.yaml); "show" (default true).
Run: ros2 run lab9_pick_place vision_lab9
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from mycobot_interfaces.srv import GetCubeCoords        # course service (unchanged), served for the original brain
from lab9_interfaces.srv import GetObject

from ament_index_python.packages import get_package_share_directory

DEFAULT_CONFIG = os.path.join(get_package_share_directory("lab9_pick_place"), "config", "lab9.yaml")   # symlink to src
FRAMES_DIR = os.path.normpath(os.path.join(os.path.realpath(DEFAULT_CONFIG), "..", "..", "frames"))   # next to LAB9_REPORT.md
DRAW = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}
STALE_S = 1.0          # objects older than this are not returned (camera stopped)
CALIB_FRAMES = 5       # frames with both markers used for the calibration (median)


def detect_markers(img, dictionary):
    """{id: 4x2 corners} (OpenCV >= 4.7 ArucoDetector, or the older detectMarkers API of the system OpenCV 4.6)."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers(img)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=cv2.aruco.DetectorParameters_create())
    return {} if ids is None else {int(i): c[0] for i, c in zip(ids.ravel(), corners)}


class PlaneMap:
    """Pixel (u, v) <-> g_base (x, y) in m on the plate, from the 2 marker centres (camera looks straight down).
    Similarity as complex numbers: w = a*z + b, z = u - i*v (image y points down), w = x + i*y."""

    def __init__(self, px, m, frame_wh, cam_h):
        i1, i2 = sorted(m)
        z1, z2 = (complex(px[i][0], -px[i][1]) for i in (i1, i2))
        w1, w2 = (complex(*m[i]) for i in (i1, i2))
        self.a = (w2 - w1) / (z2 - z1)
        self.b = w1 - self.a * z1
        self.px, self.cam_h = px, cam_h
        self.cam_xy = self.to_robot(frame_wh[0] / 2, frame_wh[1] / 2)   # plate point under the camera

    def to_robot(self, u, v):
        w = self.a * complex(u, -v) + self.b
        return w.real, w.imag

    def to_image(self, x, y):
        z = (complex(x, y) - self.b) / self.a
        return z.real, -z.imag

    def top_to_xy(self, u, v, h):
        """g_base xy of a point h above the plate seen at pixel (u, v): undo the outward shift of raised points."""
        x, y = self.to_robot(u, v)
        k = (self.cam_h - h) / self.cam_h
        return self.cam_xy[0] + (x - self.cam_xy[0]) * k, self.cam_xy[1] + (y - self.cam_xy[1]) * k

    @property
    def m_per_px(self):
        return abs(self.a)


def P(pt):
    return tuple(int(round(c)) for c in pt)


class VisionLab9(Node):
    def __init__(self):
        super().__init__("vision_lab9")
        path = os.path.expanduser(self.declare_parameter("config", DEFAULT_CONFIG).value)
        with open(path) as f:
            self.cfg = yaml.safe_load(f)
        self.get_logger().info(f"config: {path}")
        self.show = self.declare_parameter("show", True).value      # False: no window (headless test)
        self.bridge = CvBridge()
        self.pm = None
        self.calib_frames = []
        self.objects, self.stamp = [], 0.0
        self.last_summary = None
        self.create_subscription(Image, "camera/image", self.on_image, 1)
        self.create_service(GetObject, "/get_object", self.srv_get_object)
        self.create_service(GetCubeCoords, "/cube_coordinates", self.srv_cube_coords)
        if self.show:
            cv2.namedWindow("vision_lab9", cv2.WINDOW_NORMAL)
        self.get_logger().info("waiting for camera/image; markers must be visible for the calibration")

    # ---------------- calibration ----------------
    def calibrate(self):
        mk = self.cfg["markers"]
        want = {int(k): tuple(v) for k, v in mk["centers"].items()}
        seen = {i: [] for i in want}
        for f in self.calib_frames:
            found = detect_markers(f, mk["dictionary"])
            for i in want:
                if i in found:
                    seen[i].append(found[i].mean(0))
        if any(len(v) == 0 for v in seen.values()):
            missing = [i for i, v in seen.items() if not v]
            self.get_logger().warn(f"marker(s) {missing} not visible ({mk['dictionary']}); retrying",
                                   throttle_duration_sec=5.0)
            self.calib_frames = []
            return
        px = {i: tuple(np.median(v, 0)) for i, v in seen.items()}
        h, w = self.calib_frames[0].shape[:2]
        self.pm = PlaneMap(px, want, (w, h), self.cfg["camera"]["height_m"])
        self.calib_frames = []
        rot = math.degrees(np.angle(self.pm.a))
        self.get_logger().info(f"calibrated: {w}x{h}, {self.pm.m_per_px * 1000:.3f} mm/px, rotation {rot:.1f} deg, "
                               f"camera over ({self.pm.cam_xy[0]:.3f}, {self.pm.cam_xy[1]:.3f}) m")

    # ---------------- detection ----------------
    def search_mask(self, shape):
        ws, m = self.cfg["workspace"], self.cfg["workspace"]["search_margin_m"]
        corners = [(ws["x"][0] - m, ws["y"][0] - m), (ws["x"][1] + m, ws["y"][0] - m),
                   (ws["x"][1] + m, ws["y"][1] + m), (ws["x"][0] - m, ws["y"][1] + m)]
        mask = np.zeros(shape, np.uint8)
        cv2.fillPoly(mask, [np.array([P(self.pm.to_image(*c)) for c in corners], np.int32)], 255)
        return mask

    def classify(self, cnt):
        """Shape features of one contour -> (shape, circularity, fill, vertices)."""
        area, peri = cv2.contourArea(cnt), cv2.arcLength(cnt, True)
        circ = 4 * math.pi * area / peri ** 2 if peri > 0 else 0.0
        _, r = cv2.minEnclosingCircle(cnt)
        fill = area / (math.pi * r * r) if r > 0 else 0.0
        nv = len(cv2.approxPolyDP(cnt, 0.04 * peri, True))
        cyl, cube = self.cfg["shape"]["cylinder"], self.cfg["shape"]["cube"]
        if circ >= cyl["circularity_min"] and fill >= cyl["fill_min"]:
            return "cylinder", circ, fill, nv
        if circ <= cube["circularity_max"] and fill <= cube["fill_max"] and cube["vertices"][0] <= nv <= cube["vertices"][1]:
            return "cube", circ, fill, nv
        return "unknown", circ, fill, nv

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hc, ob, ws = self.cfg["hsv"], self.cfg["objects"], self.cfg["workspace"]
        area_mask = self.search_mask(frame.shape[:2])
        k3, k5 = np.ones((3, 3), np.uint8), np.ones((5, 5), np.uint8)
        objs = []
        for color, ranges in hc["hue"].items():
            mask = np.zeros(frame.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, (lo, hc["sat_min"], hc["val_min"]), (hi, 255, 255))
            mask = cv2.morphologyEx(cv2.morphologyEx(mask & area_mask, cv2.MORPH_OPEN, k3), cv2.MORPH_CLOSE, k5)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in cnts:
                _, r = cv2.minEnclosingCircle(cnt)
                size_mm = 2 * r * self.pm.m_per_px * 1000
                if not hc["size_mm"][0] <= size_mm <= hc["size_mm"][1]:
                    continue                               # too small (noise) or too large (plate, merged blobs)
                shape, circ, fill, nv = self.classify(cnt)
                height = ob["cylinder_h_m"] if shape == "cylinder" else ob["cube_m"]
                mo = cv2.moments(cnt)
                u, v = mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]
                x, y = self.pm.top_to_xy(u, v, height / 2)   # outline = top + visible sides -> about half height
                yaw, rect = 0.0, None
                if shape == "cube":
                    rect = cv2.minAreaRect(cnt)
                    p0, p1 = cv2.boxPoints(rect)[:2]
                    (x0, y0), (x1, y1) = self.pm.to_robot(*p0), self.pm.to_robot(*p1)
                    yaw = (math.atan2(y1 - y0, x1 - x0) + math.pi / 4) % (math.pi / 2) - math.pi / 4
                inside = ws["x"][0] <= x <= ws["x"][1] and ws["y"][0] <= y <= ws["y"][1]
                objs.append(dict(color=color, shape=shape, u=u, v=v, x=x, y=y, z_top=ob["surface_z_m"] + height,
                                 height=height, yaw=yaw, inside=inside, cnt=cnt, rect=rect,
                                 circ=circ, fill=fill, nv=nv))
        # unique ids per colour + shape, nearest to the robot base first
        objs.sort(key=lambda o: math.hypot(o["x"], o["y"]))
        count = {}
        for o in objs:
            key = (o["color"], o["shape"])
            count[key] = count.get(key, 0) + 1
            o["id"] = f"{o['color']}_{o['shape']}_{count[key]}"
        return objs

    def draw(self, img, objs):
        ws = self.cfg["workspace"]
        box = [(ws["x"][0], ws["y"][0]), (ws["x"][1], ws["y"][0]), (ws["x"][1], ws["y"][1]), (ws["x"][0], ws["y"][1])]
        cv2.polylines(img, [np.array([P(self.pm.to_image(*p)) for p in box])], True, (0, 255, 255), 1)
        o = self.pm.to_image(0, 0)
        cv2.circle(img, P(o), 6, (255, 0, 255), 2)
        cv2.arrowedLine(img, P(o), P(self.pm.to_image(0.05, 0)), (0, 0, 255), 2)     # +x 50 mm
        cv2.arrowedLine(img, P(o), P(self.pm.to_image(0, 0.05)), (0, 255, 0), 2)     # +y 50 mm
        for i, (u, v) in self.pm.px.items():
            cv2.drawMarker(img, P((u, v)), (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(img, f"id{i}", P((u + 8, v - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
        for ob in objs:
            col = DRAW[ob["color"]] if ob["inside"] else (128, 128, 128)
            if ob["shape"] == "cube":
                cv2.drawContours(img, [np.int32(np.round(cv2.boxPoints(ob["rect"])))], 0, col, 2)
            elif ob["shape"] == "cylinder":
                (cx, cy), r = cv2.minEnclosingCircle(ob["cnt"])
                cv2.circle(img, P((cx, cy)), int(round(r)), col, 2)
            else:
                cv2.drawContours(img, [ob["cnt"]], 0, col, 1)
            cv2.circle(img, P((ob["u"], ob["v"])), 3, (255, 255, 255), -1)
            yaw = f" yaw {math.degrees(ob['yaw']):.0f}" if ob["shape"] == "cube" else ""
            x1, y1 = int(ob["u"]) - 40, int(ob["v"]) - 30
            cv2.putText(img, ob["id"], (x1, y1 - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            cv2.putText(img, f"({ob['x'] * 1000:.0f}, {ob['y'] * 1000:.0f}) mm{yaw}", (x1, y1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.pm is None:
            self.calib_frames.append(frame)
            if len(self.calib_frames) >= CALIB_FRAMES:
                self.calibrate()
        vis = frame.copy()
        if self.pm is not None:
            objs = self.detect(frame)
            self.objects, self.stamp = objs, time.time()
            self.draw(vis, objs)
            summary = tuple((o["id"], round(o["x"] * 200), round(o["y"] * 200)) for o in objs if o["inside"])
            if summary != self.last_summary:                      # print only when the scene changes (5 mm)
                self.last_summary = summary
                self.get_logger().info("objects: " + (", ".join(
                    f"{o['id']} ({o['x'] * 1000:.0f}, {o['y'] * 1000:.0f}) C {o['circ']:.2f} fill {o['fill']:.2f} v {o['nv']}"
                    for o in objs if o["inside"]) or "none"))
        else:
            cv2.putText(vis, "NO CALIBRATION: markers not visible", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if not self.show:
            return
        cv2.imshow("vision_lab9", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("c"):
            self.pm, self.calib_frames = None, []
            self.get_logger().info("re-reading the markers (keep them uncovered)")
        elif key == ord("s"):
            os.makedirs(FRAMES_DIR, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            for tag, img in (("raw", frame), ("annotated", vis)):
                path = os.path.join(FRAMES_DIR, f"lab9_{tag}_{stamp}.png")
                cv2.imwrite(path, img)
                self.get_logger().info(f"saved {path}")

    # ---------------- services ----------------
    def current(self):
        return self.objects if time.time() - self.stamp <= STALE_S else []

    def srv_get_object(self, req, res):
        objs = [o for o in self.current() if o["inside"]
                and (not req.color or o["color"] == req.color)
                and (not req.shape or o["shape"] == req.shape)]
        res.matches = [o["id"] for o in objs]                     # already nearest first
        if req.id:
            objs = [o for o in objs if o["id"] == req.id]
        res.found = bool(objs)
        if objs:
            o = objs[0]
            res.id, res.shape, res.height = o["id"], o["shape"], float(o["height"])
            res.coords = [float(o["x"]), float(o["y"]), float(o["z_top"]), 0.0, 0.0, float(o["yaw"])]
        self.get_logger().info(f"/get_object color='{req.color}' shape='{req.shape}' id='{req.id}' -> "
                               f"{res.id if res.found else 'not found'} (matches {list(res.matches)})")
        return res

    def srv_cube_coords(self, req, res):
        cubes = [o for o in self.current() if o["inside"] and o["color"] == req.color and o["shape"] == "cube"]
        if cubes:
            o = cubes[0]
            res.coords = [float(o["x"]), float(o["y"]), float(o["z_top"]), 0.0, 0.0, float(o["yaw"])]
        else:
            res.coords = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]          # original "not detected" code
        return res


def main(args=None):
    rclpy.init(args=args)
    node = VisionLab9()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

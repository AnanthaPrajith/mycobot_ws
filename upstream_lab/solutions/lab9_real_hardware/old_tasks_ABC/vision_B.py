import cv2
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from mycobot_interfaces.srv import GetCubeCoords

# Lab 9 reference vision.py for approach 2 (Task B + Task C): robust HSV cube detection.
# Same node name, topic (camera/image) and service (/cube_coordinates) as the course vision.py.
# The pixel -> robot mapping in send_cube_coords() is the ORIGINAL one (Task A changes it, see vision_AB.py).
# Changes against the course file are marked TB1..TB4 (GUIDE.md, Task B).

# ---- Task B: detection (identical in vision_B.py and vision_AB.py) ----

# TB1: HSV thresholds, one entry per colour; red needs two hue intervals because it lies at both ends of the hue scale.
# OpenCV hue runs 0..179 (degrees / 2): the original red upper limit 200 was outside that range.
# REPLACE these numbers with your measured values (mouse overlay or hsv_calibrate.py output, GUIDE TB1).
COLORS = {
    "red": {
        "ranges": [
            (np.array([0, 100, 100]), np.array([15, 255, 255])),
            (np.array([170, 100, 100]), np.array([179, 255, 255]))     # original: 200 (invalid hue)
        ],
        "bgr": (0, 0, 255)
    },
    "yellow": {
        "ranges": [
            (np.array([16, 100, 100]), np.array([55, 255, 255]))
        ],
        "bgr": (0, 255, 255)
    },
    "green": {
        "ranges": [
            (np.array([56, 100, 100]), np.array([80, 255, 255]))
        ],
        "bgr": (0, 255, 0)
    },
    "blue": {
        "ranges": [
            (np.array([81, 100, 100]), np.array([110, 255, 255]))      # original: S and V >= 200 (misses blue in normal light)
        ],
        "bgr": (255, 0, 0)
    }
}

# TB2: mask cleaning. A 5x5 ellipse removes specks smaller than about 5 px (opening) and closes holes and gaps
# up to about 5 px (closing). A cube top is 50..150 px wide, so its outline and centre barely change.
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# TB3: candidate checks. Size and aspect ratio are the original checks; fill, solidity and workspace are new.
SIDE_MIN, SIDE_MAX = 50, 150        # px, both sides of the rotated rectangle (original)
ASPECT_MIN = 0.75                   # short side / long side (original)
FILL_MIN = 0.80                     # outline area / rectangle area: a square top gives 0.9..1.0; an L-shape, a notch or
                                    # two touching blobs give less
SOLID_MIN = 0.85                    # mask pixels inside the outline / outline area: catches rings and frames, which
                                    # pass FILL_MIN because cv2.contourArea counts the hole as area
# Workspace polygon in pixels: 4 corners of the tray area, read with the mouse overlay (GUIDE TB3). None = no check.
WORKSPACE_PX = None                 # e.g. np.array([[150, 40], [500, 40], [500, 350], [150, 350]], np.int32)


def color_mask(hsv, ranges):
    """Binary mask (0 / 255) of the pixels inside any of the HSV ranges of one colour."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def clean_mask(mask):
    """TB2: opening (erode, then dilate) removes isolated specks; closing (dilate, then erode) fills small gaps."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)


def is_cube(contour, rect, mask, workspace_px):
    """TB3: True if the contour looks like the top of one cube inside the workspace."""
    (cx, cy), (w, h), angle = rect
    if not (SIDE_MIN <= w <= SIDE_MAX and SIDE_MIN <= h <= SIDE_MAX):
        return False                                            # too small (noise) or too big (merged blobs, table)
    if min(w, h) / max(w, h) < ASPECT_MIN:
        return False                                            # not square enough
    area = cv2.contourArea(contour)                             # area enclosed by the outline (holes included)
    if area / (w * h) < FILL_MIN:
        return False                                            # outline is not a full rectangle
    inside = np.zeros_like(mask)
    cv2.drawContours(inside, [contour], -1, 255, -1)            # the outline, filled
    if cv2.countNonZero(cv2.bitwise_and(mask, inside)) / max(area, 1.0) < SOLID_MIN:
        return False                                            # region has a big hole: ring, frame, not a cube top
    if workspace_px is not None and cv2.pointPolygonTest(workspace_px, (float(cx), float(cy)), False) < 0:
        return False                                            # centre outside the tray (bins, robot, background)
    return True


def detect_cubes(bgr, colors=COLORS, workspace_px=WORKSPACE_PX):
    """Detect at most one cube per colour in ONE frame.

    Returns ({color: (cx, cy, angle, w, h)}, {color: cleaned mask}). Pure function: no ROS, no state,
    so the same code runs in the node and in evaluate_detection.py on saved frames.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found, masks = {}, {}
    for name, data in colors.items():
        mask = clean_mask(color_mask(hsv, data["ranges"]))
        masks[name] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        squares = []
        for contour in contours:
            rect = cv2.minAreaRect(contour)                     # ((cx, cy), (w, h), angle)
            if is_cube(contour, rect, mask, workspace_px):
                squares.append(rect)
        if squares:
            # as in the original: the most square-like candidate (aspect ratio closest to 1)
            (cx, cy), (w, h), angle = max(squares, key=lambda r: min(r[1]) / max(r[1]))
            found[name] = (cx, cy, angle, w, h)
    return found, masks

# ---- end Task B ----


class Vision(Node):
    def __init__(self):
        super().__init__("vision")
        self.bridge = CvBridge()
        self.detected_cubes = {}  # color -> (cx, cy, angle, w, h); only cubes seen in the LATEST frame (TB4)
        self.declare_parameter("show_masks", False)     # True: second window with the cleaned masks (GUIDE TB2)

        # HSV debug: store mouse position
        self.mouse_x = 0
        self.mouse_y = 0
        cv2.namedWindow("Color Detection")
        cv2.setMouseCallback("Color Detection", self._mouse_callback)

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            msg_type=Image,
            topic="camera/image",
            callback=self.img_callback,
            qos_profile=1
        )
        self.get_logger().info("Colored cube detector ready! (HSV-debug active)")

        # initialize cube coordinate service
        self.cube_coords_service = self.create_service(GetCubeCoords, "/cube_coordinates", self.send_cube_coords)
        self.get_logger().info("Cube coordinate service ready!")


    # send cube coordinates to brain node (original linear mapping)
    def send_cube_coords(self, request, response):
        color = request.color
        if color in self.detected_cubes:
            cx, cy, rot = self.detected_cubes[color][:3]
            # Map pixel coordinates to real coordinates
            # Pixel square: (185,335) BL, (465,335) BR, (465,55) TR, (185,55) TL
            # Real square: (0.23,-0.075) BL, (0.23,0.075) BR, (0.08,0.075) TR, (0.08,-0.075) TL
            px_norm = (cy - 55) / 280.0
            rx = 0.08 + px_norm * 0.15
            py_norm = (cx - 185) / 280.0
            ry = -0.075 + py_norm * 0.15
            rotz = rot/180 * 3.14159
            response.coords = [rx, ry, 0.01, 0.0, 0.0, rotz]
        else:
            response.coords = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        return response

    # saves mouse coordinates for the HSV Debug
    def _mouse_callback(self, event, x, y, flags, param):
        self.mouse_x = x
        self.mouse_y = y

    def img_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # HSV of the unmodified frame, for the mouse overlay below
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # TB1-TB3: detection on this frame only
            found, masks = detect_cubes(cv_image)

            # TB4: replace the dictionary instead of updating it, so a removed cube is gone after one frame
            self.detected_cubes = found

            for color_name, (cx, cy, angle, w, h) in found.items():
                draw_color = COLORS[color_name]["bgr"]
                box = cv2.boxPoints(((cx, cy), (w, h), angle)).astype(np.intp)   # np.int0 is deprecated in NumPy
                cv2.drawContours(cv_image, [box], 0, draw_color, 2)
                cv2.circle(cv_image, (int(cx), int(cy)), 5, draw_color, -1)
                text = f"{color_name}: ({int(cx)},{int(cy)}) {int(w)}x{int(h)} {int(angle)}deg"
                cv2.putText(cv_image, text, (int(cx) - 60, int(cy) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)

            if WORKSPACE_PX is not None:
                cv2.polylines(cv_image, [WORKSPACE_PX], True, (255, 255, 255), 1)

            if self.get_parameter("show_masks").value:
                combined = np.zeros(cv_image.shape[:2], dtype=np.uint8)
                for mask in masks.values():
                    combined = cv2.bitwise_or(combined, mask)
                cv2.imshow("Masks", combined)

            # --- HSV debug overlay ---
            h_img, w_img = cv_image.shape[:2]
            mx = max(0, min(self.mouse_x, w_img - 1))
            my = max(0, min(self.mouse_y, h_img - 1))
            hsv_val = hsv[my, mx]
            debug_text = f"HSV: H={hsv_val[0]}  S={hsv_val[1]}  V={hsv_val[2]}  px=({mx},{my})"
            # Black background bar at top
            cv2.rectangle(cv_image, (0, 0), (450, 30), (0, 0, 0), -1)
            cv2.putText(cv_image, debug_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            # Crosshair at mouse position
            cv2.drawMarker(cv_image, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

            # Display the result
            cv2.imshow("Color Detection", cv_image)
            cv2.waitKey(10)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    detector = Vision()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

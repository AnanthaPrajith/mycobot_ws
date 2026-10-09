"""Synthetic test bench for the Lab 9 notebook: images and masks with known ground truth.

Nothing in here is part of the solution. These functions only create inputs (marker images seen by a virtual camera, coloured cubes,
cylinders, shaded cubes) so that the solution functions in the notebook can be tested and scored without the real camera.
In the lab the images come from the camera (`camera/image`) or from saved frames instead.
"""
import cv2
import numpy as np

# ---------------------------------------------------------------- Task A: ArUco markers seen by a virtual camera
def render_markers(markers, aruco_dict, project_B, size, seed=0, noise=2.0, padpx=60):
    """Grey table with ArUco markers, as the virtual camera sees them.

    markers: {id: ((cx, cy) in B, yaw, side)}; project_B(points_B) -> pixels (the camera model); size = (W, H).
    Each marker is drawn with a white margin of `padpx` pixels (ArUco needs a quiet zone), warped to the pixel positions given
    by the camera model, blurred and corrupted with Gaussian noise.
    """
    W, H = size
    aruco = cv2.aruco
    rng = np.random.default_rng(seed)
    canvas = np.full((H, W), 150, np.uint8)
    for mid, (center, yaw, side) in markers.items():
        img = cv2.copyMakeBorder(aruco.drawMarker(aruco_dict, mid, 200), padpx, padpx, padpx, padpx, cv2.BORDER_CONSTANT, value=255)
        n = img.shape[0] - 1
        src = np.float32([[0, 0], [n, 0], [n, n], [0, n]])
        h = side / 2.0 * (1 + 2 * padpx / 200.0)                      # half side of the padded square in metres
        padded_M = np.array([[-h, h], [h, h], [h, -h], [-h, -h]])
        c, s = np.cos(yaw), np.sin(yaw)
        xy = padded_M @ np.array([[c, -s], [s, c]]).T + np.asarray(center)
        dst = np.float32(project_B(np.column_stack([xy, np.zeros(4)])))
        cv2.warpPerspective(img, cv2.getPerspectiveTransform(src, dst), (W, H), dst=canvas, borderMode=cv2.BORDER_TRANSPARENT)
    canvas = cv2.GaussianBlur(canvas, (3, 3), 0.8)
    return np.clip(canvas + rng.normal(0, noise, canvas.shape), 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- Task B: coloured cubes on a grey table
HUE = {'red': 2, 'yellow': 28, 'green': 65, 'blue': 100}          # OpenCV hue (0..179) of the synthetic cubes
CUBES = [('red', 180, 150, 60, 15), ('yellow', 330, 160, 58, 40), ('green', 450, 300, 62, 70), ('blue', 200, 330, 60, 25)]   # colour, cx, cy, side [px], angle


def hsv_to_bgr(h, s, v):
    return tuple(int(x) for x in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0])


def make_scene(cubes=CUBES, brightness=1.0, seed=0, noise=6, shadow=False, glare=False, stripe=False, cable=False):
    """Four coloured cubes on a grey table, with controls for brightness, a shadow band, glare, a thin reflection stripe across the
    yellow cube and a thin cable attached to the green one. Returns (bgr image, ground truth [(colour, cx, cy), ...])."""
    rng = np.random.default_rng(seed)
    hsv = np.zeros((480, 640, 3), np.uint8); hsv[..., 0] = 20; hsv[..., 1] = 25; hsv[..., 2] = int(150 * brightness)       # grey table
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    truth = []
    for color, cx, cy, side, ang in cubes:
        box = cv2.boxPoints(((cx, cy), (side, side), ang)).astype(np.int32)
        cv2.fillConvexPoly(img, box, hsv_to_bgr(HUE[color], 230, int(210 * brightness)))
        truth.append((color, cx, cy))
    if shadow:
        m = np.zeros((480, 640), np.uint8); cv2.rectangle(m, (0, 300), (640, 360), 255, -1)
        img = np.where(m[..., None] > 0, (img * 0.45).astype(np.uint8), img)
    if glare:
        cv2.ellipse(img, (cubes[0][1] + 8, cubes[0][2] - 8), (10, 6), 20, 0, 360, (250, 250, 250), -1)
    if stripe:            # thin reflection/scratch across the yellow cube: splits its mask
        cv2.line(img, (330 - 40, 158), (330 + 40, 158), (235, 235, 235), 4)
        cv2.line(img, (328, 120), (328, 200), (235, 235, 235), 3)
    if cable:             # thin cable of similar hue attached to the green cube
        cv2.line(img, (480, 300), (560, 300), hsv_to_bgr(HUE['green'], 230, 210), 3)
    for _ in range(60):   # isolated red pixels (sensor noise, dust) and a small red distractor
        img[rng.integers(0, 480), rng.integers(0, 640)] = hsv_to_bgr(HUE['red'], 240, 200)
    cv2.rectangle(img, (500, 60), (512, 72), hsv_to_bgr(HUE['red'], 230, 200), -1)
    img = np.clip(img.astype(np.float32) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img, truth


def score(det, truth, tol=6):
    """(TP, FN, FP): TP = cube found within `tol` px of the truth, FN = missed, FP = wrong or extra detection."""
    ref = {t[0]: (t[1], t[2]) for t in truth}
    tp = sum(1 for k, (x, y) in ref.items() if k in det and np.hypot(det[k][0] - x, det[k][1] - y) <= tol)
    fp = sum(1 for k in det if k not in ref or np.hypot(det[k][0] - ref[k][0], det[k][1] - ref[k][1]) > tol)
    return tp, len(ref) - tp, fp


# ---------------------------------------------------------------- Task C: one shaded cube
def shaded_cube_scene(hue, brightness=1.0, seed=0, hue_jitter=5):
    """One cube (120 px) on a grey table; V has a left-to-right gradient, H/S noisy. Returns bgr, true mask, and a polygon a user would click (a bit inside the cube)."""
    rng = np.random.default_rng(seed)
    hsv = np.zeros((480, 640, 3), np.float32); hsv[..., 0] = 20; hsv[..., 1] = 30; hsv[..., 2] = 150
    truth = np.zeros((480, 640), np.uint8); cx, cy, side, ang = 300, 240, 120, 20
    box = cv2.boxPoints(((cx, cy), (side, side), ang)).astype(np.int32); cv2.fillConvexPoly(truth, box, 255)
    xs = np.arange(640)[None, :].repeat(480, 0)
    vcube = (150 + 90 * (xs - (cx - 70)) / 140.0).clip(120, 240)
    m = truth > 0
    hsv[..., 0][m] = hue + rng.normal(0, hue_jitter, m.sum())
    hsv[..., 1][m] = 200 + rng.normal(0, 18, m.sum())
    hsv[..., 2][m] = vcube[m]
    hsv[..., 2] *= brightness
    hsv[..., 0] = np.mod(hsv[..., 0], 180)
    img = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    img = np.clip(img.astype(np.float32) + rng.normal(0, 4, img.shape), 0, 255).astype(np.uint8)
    polygon = cv2.boxPoints(((cx, cy), (side - 14, side - 14), ang)).astype(np.int32)
    return img, truth, polygon


def iou(a, b):
    """Intersection over union of two masks: 1.0 = identical."""
    a, b = a > 0, b > 0
    return (a & b).sum() / max((a | b).sum(), 1)


# ---------------------------------------------------------------- Task D: cubes and cylinders
def draw_shape(kind, size=60, angle=0, seed=0, occlude=0.0, noise=0.0):
    """Binary mask of one 'cube' (rotated square) or 'cylinder' (circle, top view), optionally partly hidden and with a jagged boundary."""
    rng = np.random.default_rng(seed); m = np.zeros((200, 200), np.uint8)
    if kind == 'cube':
        cv2.fillConvexPoly(m, cv2.boxPoints(((100, 100), (size, size), angle)).astype(np.int32), 255)
    else:
        cv2.circle(m, (100, 100), size // 2, 255, -1)
    if occlude > 0:                                                       # part of the object hidden (arm, neighbour)
        m[:, int(100 + size / 2 * (1 - 2 * occlude)):] = 0
    if noise > 0:                                                         # jagged boundary
        edge = cv2.morphologyEx(m, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
        flip = edge & (rng.random(m.shape) < noise)
        m[flip] = 255 - m[flip]
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    return m


def scene_shapes():
    """A scene with cubes AND cylinders, two objects of the same colour (two red cylinders)."""
    img, _ = make_scene(cubes=[], seed=2)
    items = [('red', 'cylinder', 150, 130), ('red', 'cylinder', 300, 130), ('red', 'cube', 470, 120),
             ('green', 'cube', 180, 320), ('blue', 'cylinder', 350, 330), ('yellow', 'cube', 500, 330)]
    for color, kind, cx, cy in items:
        col = hsv_to_bgr(HUE[color], 230, 210)
        if kind == 'cube':
            cv2.fillConvexPoly(img, cv2.boxPoints(((cx, cy), (62, 62), 25)).astype(np.int32), col)
        else:
            cv2.circle(img, (cx, cy), 32, col, -1)
    return img

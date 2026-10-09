"""Lab 9 simple pick and place: all values in one place (read by vision_pc.py on the Lab PC and robot_pi.py on the Pi).

Units: mm, robot base frame (x forward, y left, z up) = the frame of pymycobot send_coords.

>>> Fill the 5 MEASURE blocks (M1..M5) before the first real run. README step 3 says how. <<<
>>> Edit this file on the Lab PC, then copy it to the Pi (README step 3, last line).      <<<
Values marked LAB8 come from the Lab 8 robot run (commit fb4d71c) and worked there.
"""

# ======================= MEASURE (README step 3) =======================

# M1  Marker CENTRES in the robot frame: jog the nozzle tip over the centre of each marker, press p, copy x y.
#     WARNING: these values define the whole camera -> robot mapping. A 5 mm error here = 5 mm miss at the cube.
#     The numbers below are a GUESS from the 10-02 photo, not measured.
MARKERS_MM = {
    1: (135.0, -60.0),    # marker id 1 (nearer the robot, -y side); 10-05 ruler (90, -60) + plate moved 45 mm in +x
    2: (255.0, 60.0),     # marker id 2 (far corner, +y side);       10-05 ruler (210, 60) + plate moved 45 mm in +x
    # plate now x 120..270 in the robot frame (moved +45 on 10-05 to clear the near-base collision zone)
}
MARKERS_MEASURED = True   # set True only after M1 is measured; vision_pc.py refuses to send targets while False

# M2  Plate surface z: jog the tip down in 1 mm steps until it just touches the plate, press p, copy z.
#     WARNING: too low = the nozzle presses into the cube; too high = no suction. Lab 8 picked a cube with 0.
SURFACE_Z_MM = 12           # 10-02: mean marker tip z (15, 24). The 5-8 mm too high on the first pick = CUBE_MM 40 vs real 35.

# M3  Bin centres (x, y): jog the tip over each bin opening (start z 120), press p, copy x y.
#     WARNING: LAB8 values. In the 10-02 camera view yellow (200, 170) is over the TABLE, not a bin -> must be re-measured.
#     Colours without a real bin: point them to a bin that exists, or remove them from COLOR_ORDER below.
BIN_COORDS = {
    "red":    (65.0, 170.0),
    "yellow": (200.0, 170.0),
    "blue":  (185.0, 170.0),
}

# M4  Camera height: tape measure from the camera lens straight down to the plate surface.
#     Used for the cube-top correction (~8 mm at the plate edge). 470 = estimate from the 90° FOV.
CAMERA_HEIGHT_MM = 470.0

# M5  (optional) Edge of the black marker square, ruler. Adds a scale check at start; None = no check.
MARKER_SIDE_MM = None

# ======================= fixed setup =======================

# ROS link Lab PC -> Pi: geometry_msgs/PointStamped, point = cube top centre (m), header.frame_id = colour
TOPIC = "/simple_pp/target"

# Camera (Lab PC)
CAMERA_DEVICE = "/dev/video0"  # Logitech C930e
FRAME_W, FRAME_H = 848, 480    # camera default; markers ~23 px. If markers are not found: 1280, 720
ARUCO_DICT = "DICT_6X6_50"     # ids 1 and 2, checked on the 10-02 reference frame

# Objects / YOLO
CUBE_MM = 35.0                 # 10-02: lab cubes are 35 x 35 mm (course model/Lab 8 assumed 40)
YOLO_WEIGHTS = "~/mycobot-project/pp_yolo_ws/weights/best.pt"   # from Lab 09 PP.zip (gitignored, never commit)
YOLO_CONF = 0.5                # course used 0.75; lower (0.3) if cubes are not boxed, raise if false boxes
BOX_MM = (30.0, 75.0)          # longer box side in mm: 35 mm cube incl. visible sides ~40-60 (58 on 10-02); plate ~150 (was boxed as "red 0.86")
CLASS_TO_COLOR = {"RED CUBE": "red", "GREEN CUBE": "green", "YELLOW CUBE": "yellow", "CYAN CUBE": "cyan"}
COLOR_ORDER = ["red", "yellow", "blue"]   # pick order; only these colours are sent (10-05: = the colours in BIN_COORDS)
# WARNING: the model knows 4 cube classes only (red, cyan, green, yellow); blue gets no box even at conf 0.05 (10-05).
CYL_HEIGHT_MM = 35.0           # MEASURE: cylinder height (diameter 35 = cube edge); assumed equal to the cube
# Colour outlines + blue (HSV, OpenCV hue 0..179). Blue is detected by colour only, inside the workspace + margin.
SAT_MIN, VAL_MIN = 90, 60      # white plate, grey table, shadows, robot have low saturation
HUE_RANGES = {"red": [(0, 10), (170, 179)], "yellow": [(18, 35)], "green": [(40, 85)], "blue": [(95, 130)]}
HSV_COLORS = ["blue"]          # colours found by HSV alone (not YOLO classes)
HSV_MARGIN_MM = 20.0           # HSV search area = workspace grown by this much (keeps the bins out)
CYL_FILL_MIN = 0.75            # blob area / enclosing-circle area: cylinder ~0.8-1.0, cube ~0.6-0.7

# Send logic (Lab PC)
STABLE_S = 1.0                 # cube must stay within STABLE_MM for STABLE_S before it is sent
STABLE_MM = 5.0
RESEND_S = 1.0                 # resend period while it stays (the Pi ignores targets while it moves)

# Workspace = the white plate (LAB8 motion_node limits). Both sides reject targets outside it.
WS_X = (120.0, 265.0)          # 10-05: reachable band (near base: awkward IK, crash at 120/26; > 240: hover z 185 out of reach)
WS_Y = (-75.0, 75.0)

# Robot (Pi)
SERIAL_PORT, BAUD = "/dev/serial0", 1000000   # udev link -> /dev/ttyAMA0
PUMP_PIN, VALVE_PIN = 20, 21   # active low (LAB8)
NOZZLE_LENGTH_MM = 68.0        # LAB8
HOVER_MARGIN = 70.0            # LAB8
DROP_Z_MM = 100.0               # LAB8 tuned: tip height for the release above the bin
MOVE_SPEED = 50                # LAB8; first run: 30
MOTION_SLEEP = 1.5             # LAB8; raise to 2.5 together with a lower speed
# HOME_COORDS = [50, -65, 410, -90, 0.0, -90]   # LAB8 (old: lies inside MoveIt's env_camera box -> brain_lab9 error -10)
HOME_COORDS = [0, 180, 200, -180, 0.0, 0.0]    # 10-05: same home as brain_lab9 (lab9.yaml robot.home), tool down
INTER_COORDS = [155, -20, 245, 180, 0.0, 0.0]             # LAB8
HOME_SPEED, HOME_DELAY = 40, 4.0
SETTLE_AFTER_S = 2.0           # Pi ignores targets for this long after a pick (vision sees the new scene first)

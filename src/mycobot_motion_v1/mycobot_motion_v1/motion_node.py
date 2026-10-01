import time
import threading
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from mycobot_msgs.msg import DetectedObject
from std_msgs.msg import String
from pymycobot.mycobot280 import MyCobot280
import RPi.GPIO as GPIO

# ============================================================
# ----------------------- CONSTANTS --------------------------
# ============================================================

MOVE_SPEED = 10

# -------- TOOL GEOMETRY (TCP) --------
NOZZLE_LENGTH_MM = 68.0
HOVER_MARGIN = 70.0

# -------- MOTION TIMING --------
MOTION_SLEEP = 4.0
VACUUM_START_DELAY = 1.0
VACUUM_DWELL = 2.0
DETECTION_REARM_TIME = 3.0

# Empirical corrections measured with the Lab 8 camera/robot setup.
HOVER_X_OFFSET_MM = -2.0
HOVER_Y_OFFSET_MM = -46.0
PRE_PICK_X_OFFSET_MM = -5.9
PRE_PICK_Y_OFFSET_MM = -46.7
PICK_X_OFFSET_MM = -9.3
PICK_Y_OFFSET_MM = -47.7
PRE_PICK_CLEARANCE_MM = 20.0

PICKUP_OFFSETS_MM = {
    "yellow": {
        "hover": (0.0, 0.0),
        "pre_pick": (-3.9, -0.7),
        "pick": (-7.3, -1.7),
    },
    "red": {
        "hover": (0.0, 0.0),
        "pre_pick": (-3.9, -0.7),
        "pick": (-7.3, -1.7),
    },
}

# -------- IDLE BEHAVIOR --------
NO_DETECTION_TIMEOUT = 0.2
IDLE_CHECK_RATE = 1.0

# -------- DEBUG SAFETY --------
DEBUG_STOP_ENABLE = False

# ----------------------- HOME -------------------------------
# Manually taught collision-free upright home (2026-10-01).
HOME_ANGLES = [2.98, 2.54, -1.31, -0.70, -2.37, 75.14]
HOME_SPEED = 5
HOME_DELAY = 12.0

# ----------------------- WORKSPACE (mm) ---------------------
WORKSPACE_X_MIN = 75.0
WORKSPACE_X_MAX = 225.0
WORKSPACE_Y_MIN = -75.0
WORKSPACE_Y_MAX = 75.0

# A single cube on the calibrated table reports approximately Z=30 mm.
DETECTED_Z_MIN_MM = 15.0
DETECTED_Z_MAX_MM = 50.0

# ----------------------- DROP -------------------------------
DROP_CLEARANCE_MM = 15.0
DROP_Z_MM = 60.0
BIN_TRANSIT_Z_MM = 220.0
BIN_XY_TOLERANCE_MM = 15.0
BIN_RELEASE_Z_MM = {
    "yellow": 186.1,
    "red": 155.8,
}

# ----------------------- DETECTION WAIT ---------------------
DETECTION_WAIT_TIME = 2.0  # seconds

# ----------------------- BINS -------------------------------
BIN_COORDS = {
    "red":    [140.6, -124.2],
    "yellow": [218.3, -137.6],
    "green":  [115.8, 177.3],
    "blue":   [-6.9, 173.2],
    "cyan":   [-6.9, 173.2],
}

#----------------COLOR PRIORITY------------------------------
COLOR_PRIORITY = {
    "red": 0,
    "yellow": 1,
    "green": 2,
    "cyan": 3,
}


# ============================================================
# ------------------ KEYBOARD LISTENER -----------------------
# ============================================================

def keyboard_listener(stop_flag):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            ch = sys.stdin.read(1)
            if ch.lower() == 's':
                stop_flag["stop"] = True
                print("\n[SAFETY] STOP requested by user")
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ============================================================
# ----------------------- MOTION NODE ------------------------
# ============================================================

class MotionNode(Node):

    def __init__(self):
        super().__init__("motion_node")

        # ---------------- GPIO SETUP ----------------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)   # Vacuum
        GPIO.setup(21, GPIO.OUT)   # Release
        GPIO.output(20, 1)
        GPIO.output(21, 1)

        self.vacuum_on = False

        # ---------------- Subscriber ----------------
        self.sub = self.create_subscription(
            DetectedObject,
            "/detected_objects",
            self.object_callback,
            10
        )
        self.color_sub = self.create_subscription(
            String,
            "/mycobot/target_color",
            self.target_color_callback,
            10
        )

        # ---------------- Robot ----------------
        self.mc = MyCobot280("/dev/serial0", 1000000)
        time.sleep(0.2)

        # ---------------- State ----------------
        self.busy = False
        self.at_home = True
        self.target = None
        self.detection_buffer = []


        # Detection wait logic
        self.pending_target = None
        self.first_detection_time = None

        self.stop_flag = {"stop": False}
        self.last_detection_time = time.time()
        self.suppressed_color = None
        self.last_suppressed_detection_time = 0.0
        self.selected_color = "none"

        # ---------------- Idle timer ----------------
        self.create_timer(IDLE_CHECK_RATE, self.idle_check_callback)

        # ---------------- Safety Keyboard ----------------
        if DEBUG_STOP_ENABLE:
            threading.Thread(
                target=keyboard_listener,
                args=(self.stop_flag,),
                daemon=True
            ).start()
            self.get_logger().warn("DEBUG STOP ENABLED — press 's'")

        # Initial HOME
        if self.go_home():
            self.get_logger().info(
                "Motion node ready; target_color=none. "
                "Publish red, yellow, auto, or none on /mycobot/target_color"
            )
        else:
            self.get_logger().error(
                "Home failed; automatic motion remains disabled"
            )

    # ========================================================
    # ---------------- WORKSPACE CHECK ------------------------
    # ========================================================

    def is_inside_workspace(self, msg):
        x_mm = msg.x * 1000.0
        y_mm = msg.y * 1000.0
        z_mm = msg.z * 1000.0
        return (
            WORKSPACE_X_MIN <= x_mm <= WORKSPACE_X_MAX and
            WORKSPACE_Y_MIN <= y_mm <= WORKSPACE_Y_MAX and
            DETECTED_Z_MIN_MM <= z_mm <= DETECTED_Z_MAX_MM
        )

    # ========================================================
    # ---------------- PUMP CONTROL ---------------------------
    # ========================================================

    def pump_on(self):
        GPIO.output(20, 0)   # Vacuum ON
        GPIO.output(21, 0)   # Release OFF
        self.vacuum_on = True

    def pump_off(self):
        GPIO.output(20, 1)
        time.sleep(0.3)
        GPIO.output(21, 1)
        time.sleep(2.0)
        self.vacuum_on = False

    # ========================================================
    # ---------------- SAFETY CHECK ---------------------------
    # ========================================================

    def check_stop(self):
        if self.stop_flag["stop"]:
            self.get_logger().error("Motion stopped by user!")
            self.pump_off()
            self.busy = False
            return True
        return False

    #-------------------PRIORITY-----------------------------

    def choose_highest_priority_target(self, detections):
        """
        detections: list of DetectedObject
        returns: DetectedObject or None
        """
        calibrated_colors = set(PICKUP_OFFSETS_MM) & set(BIN_RELEASE_Z_MM)
        valid = [
            d for d in detections
            if d.color in calibrated_colors
            and self.is_inside_workspace(d)
            and (
                self.selected_color == "auto"
                or d.color == self.selected_color
            )
        ]

        if not valid:
            return None

        # Prefer color priority, then the detection nearest the calibrated
        # 30 mm tabletop cube height. High Z detections are filtered above.
        valid.sort(
            key=lambda d: (
                COLOR_PRIORITY[d.color],
                abs(d.z * 1000.0 - 30.0),
            )
        )
        return valid[0]


    # ========================================================
    # ---------------- CALLBACK -------------------------------
    # ========================================================

    def target_color_callback(self, msg):
        requested = msg.data.strip().lower()
        allowed = {"none", "auto", "red", "yellow"}

        if requested not in allowed:
            self.get_logger().warn(
                f"[SELECT] Unsupported color {requested!r}; "
                "choose none, red, yellow, or auto"
            )
            return

        self.selected_color = requested
        self.detection_buffer.clear()
        self.first_detection_time = None
        self.suppressed_color = None
        self.get_logger().info(f"[SELECT] Target color set to {requested}")

    def object_callback(self, msg):
        self.last_detection_time = time.time()

        if self.selected_color == "none":
            return

        if self.selected_color != "auto" and msg.color != self.selected_color:
            return

        # Repeated frames of a cube already handled must not start another cycle.
        if msg.color == self.suppressed_color:
            self.last_suppressed_detection_time = self.last_detection_time
            return

        if self.busy or self.stop_flag["stop"]:
            return

        if not self.at_home:
            return

        if not self.is_inside_workspace(msg):
            return

        now = time.time()

        # First detection → start wait timer
        if self.first_detection_time is None:
            self.first_detection_time = now
            #self.pending_target = msg
            self.detection_buffer = [msg]
            self.get_logger().info("[WAIT] Object detected — collecting for priority")
            return

        # Still waiting
        if (now - self.first_detection_time) < DETECTION_WAIT_TIME:
            #self.pending_target = msg
            self.detection_buffer.append(msg)
            return

        # Time elapsed → choose highest priority
        target = self.choose_highest_priority_target(self.detection_buffer)

        # Accept target after wait
        #self.target = self.pending_target
        self.detection_buffer = []
        self.first_detection_time = None

        if target is None:
            return

        self.target = target
        self.suppressed_color = target.color
        self.last_suppressed_detection_time = now
        self.at_home = False
        self.execute_pick_and_place()

    # ========================================================
    # ---------------- IDLE CHECK -----------------------------
    # ========================================================

    def idle_check_callback(self):
        idle_time = time.time() - self.last_detection_time
        suppressed_idle_time = time.time() - self.last_suppressed_detection_time

        if self.suppressed_color is not None and suppressed_idle_time > DETECTION_REARM_TIME:
            self.get_logger().info(
                f"[READY] {self.suppressed_color} target cleared; detection re-armed"
            )
            self.suppressed_color = None

        if idle_time > NO_DETECTION_TIMEOUT:
            if not self.busy and not self.at_home:
                self.first_detection_time = None
                self.pending_target = None
                self.target = None
                self.go_home()
                self.at_home = True

    # ========================================================
    # ---------------- PICK & PLACE ---------------------------
    # ========================================================

    def execute_pick_and_place(self):
        self.busy = True

        obj_x = self.target.x * 1000.0
        obj_y = self.target.y * 1000.0
        obj_z = self.target.z * 1000.0
        color = self.target.color
        offsets = PICKUP_OFFSETS_MM.get(
            color, PICKUP_OFFSETS_MM["yellow"]
        )
        hover_dx, hover_dy = offsets["hover"]
        pre_pick_dx, pre_pick_dy = offsets["pre_pick"]
        pick_dx, pick_dy = offsets["pick"]

        self.get_logger().info(
            f"[Target mm] X={obj_x:.1f}, Y={obj_y:.1f}, Z={obj_z:.1f}"
        )

        pick_ee_z = obj_z + NOZZLE_LENGTH_MM
        pre_pick_ee_z = pick_ee_z + PRE_PICK_CLEARANCE_MM
        hover_ee_z = pick_ee_z + HOVER_MARGIN

        hover_pose = [
            obj_x + hover_dx, obj_y + hover_dy,
            hover_ee_z, 180, 0, 0
        ]
        pre_pick_pose = [
            obj_x + pre_pick_dx, obj_y + pre_pick_dy,
            pre_pick_ee_z, 180, 0, 0
        ]
        pick_pose = [
            obj_x + pick_dx, obj_y + pick_dy,
            pick_ee_z, 180, 0, 0
        ]

        self.get_logger().info(
            f"[Corrected mm] color={color}, hover={hover_pose[:3]}, "
            f"pre-pick={pre_pick_pose[:3]}, pick={pick_pose[:3]}"
        )

        # Approach
        if self.check_stop(): return
        self.mc.send_coords(hover_pose, MOVE_SPEED)
        time.sleep(MOTION_SLEEP)
        self.mc.send_coords(pre_pick_pose, MOVE_SPEED)
        time.sleep(MOTION_SLEEP)

        # Vacuum ON
        self.pump_on()
        time.sleep(VACUUM_START_DELAY)

        # Descend
        if self.check_stop(): return
        self.mc.send_coords(pick_pose, MOVE_SPEED)
        time.sleep(MOTION_SLEEP)
        time.sleep(VACUUM_DWELL)

        # Lift
        if self.check_stop(): return
        self.mc.send_coords(pre_pick_pose, MOVE_SPEED)
        time.sleep(MOTION_SLEEP)
        self.mc.send_coords(hover_pose, MOVE_SPEED)
        time.sleep(MOTION_SLEEP)

        # Bin move
        if color not in BIN_COORDS:
            self.get_logger().warn(f"Unknown color '{color}'")
            self.pump_off()
            self.busy = False
            return

        bx, by = BIN_COORDS[color]
        bin_transit_z = max(hover_ee_z, BIN_TRANSIT_Z_MM)
        drop_ee_z = BIN_RELEASE_Z_MM.get(
            color, DROP_Z_MM + NOZZLE_LENGTH_MM
        )

        # Raise before crossing the workspace, then approach above the bin.
        if self.check_stop(): return
        self.mc.send_coords(
            [hover_pose[0], hover_pose[1], bin_transit_z, 180, 0, 0],
            MOVE_SPEED
        )
        time.sleep(MOTION_SLEEP)
        self.mc.send_coords([bx, by, bin_transit_z, 180, 0, 0], MOVE_SPEED)
        time.sleep(MOTION_SLEEP)

        # Descend to the calibrated release pose.
        if self.check_stop(): return
        self.mc.send_coords([bx, by, drop_ee_z, 180, 0, 0], MOVE_SPEED)
        time.sleep(MOTION_SLEEP)

        actual = self.mc.get_coords()
        bin_reached = (
            isinstance(actual, list)
            and len(actual) == 6
            and abs(actual[0] - bx) <= BIN_XY_TOLERANCE_MM
            and abs(actual[1] - by) <= BIN_XY_TOLERANCE_MM
        )
        if not bin_reached:
            self.get_logger().error(
                f"[BIN ERROR] Expected XY=({bx:.1f}, {by:.1f}), got {actual}. "
                "Keeping suction on and returning home."
            )
            self.go_home()
            self.last_suppressed_detection_time = time.time()
            self.detection_buffer.clear()
            self.first_detection_time = None
            self.busy = False
            self.at_home = True
            self.target = None
            return

        self.get_logger().info(
            f"[BIN READY] Reached {color} bin at {actual}; releasing cube"
        )
        self.pump_off()
        time.sleep(1.0)

        # Retreat
        if self.check_stop(): return
        self.mc.send_coords([bx, by, bin_transit_z, 180, 0, 0], MOVE_SPEED)
        time.sleep(MOTION_SLEEP)

        # HOME
        self.go_home()

        # Start re-arm timing after motion so queued timer events cannot
        # immediately accept stale camera frames from this completed cycle.
        self.last_suppressed_detection_time = time.time()
        self.detection_buffer.clear()
        self.first_detection_time = None
        self.busy = False
        self.at_home = True
        self.target = None

    # ========================================================
    # ---------------- HOME -----------------------------------
    # ========================================================

    def go_home(self):
        self.get_logger().info(f"[HOME] Moving slowly to {HOME_ANGLES}")
        self.mc.send_angles(HOME_ANGLES, HOME_SPEED)
        time.sleep(HOME_DELAY)

        error = self.mc.get_error_information()
        status = self.mc.get_servo_status()
        if error not in (0, None) or any(status):
            self.get_logger().error(
                f"[HOME ERROR] error={error}, servo_status={status}"
            )
            self.stop_flag["stop"] = True
            return False

        self.get_logger().info("[HOME] Safe home reached")
        return True

# ============================================================
# ----------------------- MAIN -------------------------------
# ============================================================

def main():
    rclpy.init()
    node = MotionNode()
    rclpy.spin(node)

    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

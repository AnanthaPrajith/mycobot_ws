import rclpy
from rclpy.node import Node
from mycobot_msgs.msg import DetectedObject
from pymycobot.mycobot280 import MyCobot280
import time
import RPi.GPIO as GPIO

# ----------------------- GPIO CONFIG -----------------------
GPIO.setwarnings(False)

# ----------------------- PICK & DROP HEIGHTS -----------------------
BLOCK_HEIGHT_MM = 40.0
NOZZLE_LENGTH_MM = 68.0
PICK_Z = BLOCK_HEIGHT_MM + NOZZLE_LENGTH_MM   # 108 mm
HOVER_Z = PICK_Z + 60                         # 168 mm
MOVE_SPEED = 50

# ----------------------- SORTING BIN COORDS -----------------------
BIN_COORDS = {
    "red":    [132.2, -146, HOVER_Z],
    "yellow": [238.8, -146, HOVER_Z],
    "green":  [115.8, 177.3, HOVER_Z],
    "blue":   [-6.9, 173.2, HOVER_Z],
    "cyan":   [-6.9, 173.2, HOVER_Z],
}

MOVE_DELAY = 4.0

# ----------------------- SAFE CARTESIAN HOME (MEASURED) -----------------------
HOME_COORDS = [51.3, -63.3, 412.67, -91.75, -0.63, -89.98]
INTER_COORDS = [156.4, -17.2, 247.8, 179.27, 0.21, -1.4]
HOME_SPEED = 40
HOME_DELAY = 4.0


class PiMotionNode(Node):
    def __init__(self):
        super().__init__("mycobot_pi_motion_node")

        # Connect to MyCobot
        self.mycobot = MyCobot280("/dev/serial0", 1000000)

        # ----------------------- GPIO SETUP (SUCTION PUMP) -----------------------
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)
        GPIO.output(20, 1)
        GPIO.output(21, 1)

        # ROS subscriber
        self.sub = self.create_subscription(
            DetectedObject,
            "/detected_objects",
            self.callback_detected_object,
            10
        )

        self.busy = False

    # ----------------------- SUCTION CONTROL -----------------------
    def pump_on(self):
        GPIO.output(20, 0)

    def pump_off(self):
        GPIO.output(20, 1)
        time.sleep(0.05)
        GPIO.output(21, 0)
        time.sleep(1)
        GPIO.output(21, 1)
        time.sleep(0.05)

    # ----------------------- CALLBACK -----------------------
    def callback_detected_object(self, msg: DetectedObject):
        if self.busy:
            return

        self.busy = True
        self.get_logger().info(
            f"[VISION] Pick {msg.color} at x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}"
        )

        self.pick_and_place(msg.x, msg.y, msg.z, msg.color)
        self.busy = False

    # ----------------------- PICK & PLACE -----------------------
    def pick_and_place(self, x, y, z, color):
        mc = self.mycobot
        wrist = [180, 0, 0]

        mc.send_coords([x, y, HOVER_Z] + wrist, MOVE_SPEED)
        time.sleep(MOVE_DELAY)

        mc.send_coords([x, y, PICK_Z] + wrist, MOVE_SPEED)
        time.sleep(MOVE_DELAY)

        self.pump_on()
        time.sleep(1)

        mc.send_coords([x, y, HOVER_Z] + wrist, MOVE_SPEED)
        time.sleep(MOVE_DELAY)

        drop_x, drop_y, _ = BIN_COORDS.get(color, BIN_COORDS["blue"])
        mc.send_coords([drop_x, drop_y, HOVER_Z] + wrist, MOVE_SPEED)
        time.sleep(MOVE_DELAY)

        mc.send_coords([drop_x, drop_y, PICK_Z] + wrist, MOVE_SPEED)
        time.sleep(MOVE_DELAY)

        self.pump_off()
        time.sleep(1)

        mc.send_coords([drop_x, drop_y, HOVER_Z] + wrist, MOVE_SPEED)
        time.sleep(MOVE_DELAY)

        self.go_home()

        self.get_logger().info(f"[PI] Pick & place {color} done.")

    # ----------------------- SAFE IK HOME -----------------------
    def go_home(self):
        self.mycobot.send_coords(INTER_COORDS, HOME_SPEED)
        time.sleep(HOME_DELAY)
        self.mycobot.send_coords(HOME_COORDS, HOME_SPEED)
        time.sleep(HOME_DELAY)


def main(args=None):
    rclpy.init(args=args)
    node = PiMotionNode()
    rclpy.spin(node)

    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

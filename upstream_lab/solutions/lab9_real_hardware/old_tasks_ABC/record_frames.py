#!/usr/bin/env python3
"""Save real camera frames from the ROS topic camera/image (Lab 9, all approaches).

Run on the Lab PC, from the repo root, while the camera node runs (run guide B4); the robot is not needed:
    source ~/rosenv9.sh
    python3 solutions/lab9_real_hardware/record_frames.py light1_cubes      # frames go to lab9/frames/light1_cubes/
Keys in the window: s = save the current frame, q or Esc = quit.
The saved PNG is exactly what vision.py receives (same topic, same size); the text in the window is not saved.
This script only listens to a topic; it never moves the robot.
"""
import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

LAB9 = Path(__file__).resolve().parents[2] / "lab9"     # <repo>/lab9, on the Lab PC ~/mycobot-project/lab9


class Recorder(Node):
    def __init__(self):
        super().__init__("record_frames")
        self.bridge = CvBridge()
        self.frame = None                                    # latest image, BGR
        self.create_subscription(Image, "camera/image", self.on_image, 1)

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="subfolder of lab9/frames, e.g. light1_cubes, markers, checkerboard")
    parser.add_argument("--out", default=str(LAB9 / "frames"), help="parent folder (default: lab9/frames)")
    args = parser.parse_args()

    out = Path(args.out).expanduser() / args.name
    out.mkdir(parents=True, exist_ok=True)
    count = len(list(out.glob("*.png")))                     # continue numbering if the folder has frames already
    print(f"saving to {out}  (s = save, q = quit)")

    rclpy.init()
    node = Recorder()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)          # handle at most one incoming image
            if node.frame is None:
                continue                                     # no image yet: is the camera node running?
            view = node.frame.copy()                         # draw on a copy, save the original
            h, w = view.shape[:2]
            cv2.putText(view, f"{w}x{h}  saved: {count}  s=save q=quit", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("record_frames", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                path = out / f"{args.name}_{count:03d}.png"
                cv2.imwrite(str(path), node.frame)
                count += 1
                print("saved", path)
            elif key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

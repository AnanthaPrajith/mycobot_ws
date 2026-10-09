#!/usr/bin/env python3
"""Publish a still image as `camera/image` (sensor_msgs/Image, bgr8) at 10 Hz, so that the Lab 9 Vision node can run without a camera.

Usage (ROS sourced, e.g. in ~/venvs/mycobot with system site packages):
    python publish_test_image.py                    # synthetic scene: four coloured cubes on a grey table
    python publish_test_image.py --scene shapes     # cubes and cylinders, two objects of one colour
    python publish_test_image.py saved_frame.png    # any image file (640x480 is what vision.py assumes)
    python publish_test_image.py --rate 5

It replaces `ros2 run mycobot_280pi opencv_camera` (which needs a real USB camera). The synthetic scenes come from
`lab9_synthetic.py`, so they only approximate real images: use them to test the pipeline, not to tune thresholds for your lighting.
Nothing here talks to the robot.
"""
import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('image', nargs='?', help='image file; without it a synthetic scene is used')
    parser.add_argument('--scene', choices=['cubes', 'shapes'], default='cubes', help='synthetic scene (when no file is given)')
    parser.add_argument('--rate', type=float, default=10.0, help='frames per second (the real camera node publishes 10 Hz)')
    args = parser.parse_args(argv)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f'could not read {args.image}')
            return 1
    else:
        import lab9_synthetic as syn                        # next to this file
        frame = syn.make_scene(stripe=False)[0] if args.scene == 'cubes' else syn.scene_shapes()
    print('image', frame.shape[1], 'x', frame.shape[0], '-> topic camera/image at', args.rate, 'Hz  (Ctrl+C to stop)')

    rclpy.init()
    node = Node('test_image_publisher')
    publisher = node.create_publisher(Image, 'camera/image', 1)
    bridge = CvBridge()
    node.create_timer(1.0 / args.rate, lambda: publisher.publish(bridge.cv2_to_imgmsg(frame, 'bgr8')))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

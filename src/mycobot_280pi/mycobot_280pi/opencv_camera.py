"""Publish images from one persistent Linux USB camera capture."""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Image_Publisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.cap_num = int(self.declare_parameter('num', 0).value)
        self.show_preview = bool(self.declare_parameter('show_preview', False).value)
        width = int(self.declare_parameter('width', 640).value)
        height = int(self.declare_parameter('height', 480).value)
        fps = float(self.declare_parameter('fps', 10.0).value)
        if min(width, height) <= 0 or fps <= 0:
            raise ValueError('Camera width, height and fps must be positive.')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/image', 1)
        self.cap = cv2.VideoCapture(self.cap_num, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f'Cannot open /dev/video{self.cap_num}; check the device and other camera processes.')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.requested_size = (width, height)
        self.actual_size = None
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning('Camera read failed; no image published.', throttle_duration_sec=5.0)
            return
        height, width = frame.shape[:2]
        if self.actual_size != (width, height):
            self.actual_size = (width, height)
            self.get_logger().info(f'Camera /dev/video{self.cap_num}: actual image {width}x{height}.')
            if self.actual_size != self.requested_size:
                self.get_logger().warning(
                    f'Requested {self.requested_size}, got {self.actual_size}; intrinsics must match actual resolution.')
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(msg)
        if self.show_preview:
            cv2.imshow('myCobot camera preview', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    img_pub = None
    try:
        img_pub = Image_Publisher()
        rclpy.spin(img_pub)
    except KeyboardInterrupt:
        pass
    finally:
        if img_pub is not None:
            img_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

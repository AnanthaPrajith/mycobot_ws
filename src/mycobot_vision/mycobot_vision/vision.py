import cv2
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from mycobot_interfaces.srv import GetCubeCoords

class Vision(Node):
    def __init__(self):
        super().__init__("vision")
        self.bridge = CvBridge()
        self.detected_cubes = {}  # Store detected cube centers: color -> (cx, cy)

        # HSV debug: store mouse position
        self.mouse_x = 0
        self.mouse_y = 0
        self.hsv_frame = None
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


    # send cube coordinates to brain node
    def send_cube_coords(self, request, response):
        color = request.color
        if color in self.detected_cubes:
            cx, cy, rot = self.detected_cubes[color]
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

            # Convert BGR image to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define color ranges in HSV
            colors = {
                "red": {
                    "ranges": [
                        (np.array([0, 100, 100]), np.array([15, 255, 255])),
                        (np.array([170, 100, 100]), np.array([200, 255, 255]))
                    ],
                    "bgr": (0, 0, 255) # Red for drawing
                },
                "yellow": {
                    "ranges": [
                        (np.array([16, 100, 100]), np.array([55, 255, 255]))
                    ],
                    "bgr": (0, 255, 255) # Yellow for drawing
                },
                "green": {
                    "ranges": [
                        (np.array([56, 100, 100]), np.array([80, 255, 255]))
                    ],
                    "bgr": (0, 255, 0) # Green for drawing
                },
                "blue": {
                    "ranges": [
                        (np.array([81, 200, 200]), np.array([110, 255, 255]))
                    ],
                    "bgr": (255, 0, 0) # Blue for drawing
                }
            }

            for color_name, color_data in colors.items():
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                for (lower, upper) in color_data["ranges"]:
                    mask_part = cv2.inRange(hsv, lower, upper)
                    mask = cv2.bitwise_or(mask, mask_part)

                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Filter: only square-like contours (50-150px side length, including rotated)
                valid_squares = []
                for contour in contours:
                    # minAreaRect also detects rotated rectangles
                    rect = cv2.minAreaRect(contour)  # ((cx,cy), (w,h), angle)
                    (cx, cy), (w, h), angle = rect
                    #print(f"[{color_name}] w={w:.0f} h={h:.0f} ratio={min(w,h)/max(w,h) if max(w,h)>0 else 0:.2f}")

                    # Check size (50-150px for 4cm cube)
                    if not (50 <= w <= 150 and 50 <= h <= 150):
                        continue

                    # Check aspect ratio (must be ~1:1 for square)
                    if w == 0 or h == 0:
                        continue
                    aspect_ratio = float(min(w, h)) / float(max(w, h))
                    if aspect_ratio < 0.75:
                        continue

                    valid_squares.append((contour, rect))

                # Draw the most square-like cube per color (aspect ratio closest to 1)
                if valid_squares:
                    largest = max(valid_squares, key=lambda s: min(s[1][1]) / max(s[1][1]) if max(s[1][1]) > 0 else 0)
                    contour, rect = largest
                    (cx, cy), (w, h), angle = rect
                    cx, cy = int(cx), int(cy)

                    self.detected_cubes[color_name] = (cx, cy, angle)

                    # Draw rotated rectangle
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    draw_color = color_data["bgr"]
                    cv2.drawContours(cv_image, [box], 0, draw_color, 2)
                    cv2.circle(cv_image, (cx, cy), 5, draw_color, -1)

                    text = f"{color_name}: ({cx},{cy}) {int(w)}x{int(h)} {int(angle)}deg"
                    cv2.putText(cv_image, text, (cx - 60, cy - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)

            # --- HSV debug overlay ---
            h_img, w_img = cv_image.shape[:2]
            mx = max(0, min(self.mouse_x, w_img - 1))
            my = max(0, min(self.mouse_y, h_img - 1))
            hsv_val = hsv[my, mx]
            debug_text = f"HSV: H={hsv_val[0]}  S={hsv_val[1]}  V={hsv_val[2]}"
            # Black background bar at top
            cv2.rectangle(cv_image, (0, 0), (350, 30), (0, 0, 0), -1)
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

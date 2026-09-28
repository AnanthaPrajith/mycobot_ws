# mycobot_vision

A ROS 2 Python package for real-time colored cube detection. It processes a live usb camera feed using OpenCV and HSV color thresholding to detect 4 cm cubes, maps their pixel coordinates to robot-frame coordinates, and exposes those coordinates via a ROS 2 service for use by [**Brain**](../mycobot_brain/README.md).  
Note that the **Vision** package as well as this README was mainly created by AI (Gemini & Claude).  
See the code here: [**vision.py**](../mycobot_vision/mycobot_vision/vision.py)

---

## Interfaces

### Topics
| Topic Name | Message Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `camera/image` | `sensor_msgs/Image` | Subscriber | Receives raw RGB images from the USB camera. |

### Services
| Service Name | Service Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/cube_coordinates` | `mycobot_interfaces/GetCubeCoords` | Server| Returns the pose in the robot-frame `[x, y, z, roll, pitch, yaw]` for a requested cube color. Returns `[1,1,1,1,1,1]` if the color is not detected. |

---

## Key Functions

| Function | Description |
|----------|-------------|
| `img_callback(msg)` | Converts the ROS image to BGR, applies HSV masking per color, finds and filters square-like contours (50–150 px), selects the best candidate per color, draws results on the frame, and renders the HSV debug overlay. |
| `send_cube_coords(request, response)` | Service handler. Looks up the stored centroid for the requested color and maps pixel coordinates `(cx, cy)` linearly to robot-frame coordinates `(x, y)`. Rotation angle is converted to radians and placed in `yaw`. |

---

## Coordinate Mapping

The vision node uses a fixed linear mapping between the camera's pixel space and the robot's XY plane:

| Corner | Pixel `(cx, cy)` | Robot `(x, y)` [m] |
|--------|-----------------|---------------------|
| Bottom-left  | (185, 335) | (0.23, −0.075) |
| Bottom-right | (465, 335) | (0.23,  0.075) |
| Top-right    | (465,  55) | (0.08,  0.075) |
| Top-left     | (185,  55) | (0.08, −0.075) |

Z is fixed at `0.01 m` (cube surface height). Roll and pitch are `0`; yaw is taken from `cv2.minAreaRect`'s angle output.

---

## Detected Colors & HSV Ranges

| Color  | HSV Range(s) |
|--------|-------------|
| Red    | [0, 100, 100]–[15, 255, 255] **and** [170, 100, 100]–[200, 255, 255] |
| Yellow | [16, 100, 100]–[55, 255, 255] |
| Green  | [56, 100, 100]–[80, 255, 255] |
| Blue   | [81, 100, 100]–[110, 255, 255] |

> **Tip:** Use the live HSV overlay in the *Color Detection* window to read HSV values directly from the camera feed and fine-tune these ranges for your lighting conditions.

---

## Dependencies

| Dependency | Role |
|-----------|------|
| `rclpy` | ROS 2 Python client library |
| `sensor_msgs` | `Image` message type |
| `cv_bridge` | Converts ROS `Image` ↔ OpenCV `Mat` |
| `opencv-python` (`cv2`) | Image processing, contour detection, display |
| `numpy` | Array operations for HSV masking |
| `mycobot_interfaces` | Custom `GetCubeCoords` service definition |

---

## Building & Running

```bash
# Build
cd ~/mycobot_280pi/ros2_ws
colcon build --packages-select mycobot_vision
source install/setup.bash

# Run
ros2 run mycobot_vision vision
```

Make sure the camera node is running and publishing on `camera/image` before starting the vision node.

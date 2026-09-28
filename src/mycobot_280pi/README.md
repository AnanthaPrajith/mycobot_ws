# mycobot_280pi

This package from elephant robotics contains different useful functions specific to the myCobot 280pi, such as a tele-operation via keyboard or a program for the cobot to follow its simulated rviz twin. For our purpose we only need [**opencv_camera.py**](../mycobot_280pi/mycobot_280pi/opencv_camera.py)

### `opencv_camera.py`

This takes input from the usb camera and publishes one image every 0.1 sec in the format of a sensor_msg Image to the topic **camera/image**. This topic is then subscribed by [**Vision**](../mycobot_vision/README.md).

>Note: To get this node to work, it is crucial to set the camera ID according to your computer:

```python
    [...]
    # Declaring launch parameters
    self.declare_parameter('num', 0)
    [...]
```

>To check the ID of the usb camera use:

```bash
v4l2-ctl --list-devices
```
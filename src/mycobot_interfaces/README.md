# mycobot_interfaces

A ROS 2 Python package that contains custom ROS 2 message and service definitions for the myCobot 280pi. I extended the base interfaces provided by elephant robotics by adding [**GetCubeCoords.srv**](../mycobot_interfaces/srv/GetCubeCoords.srv). In fact, for our purpose we only need this custom service.

## Custom Services

### `GetCubeCoords.srv`
This service is the bridge between [**Vision**](../mycobot_vision/README.md) and [**Brain**](../mycobot_brain/README.md). It allows [**Brain**](../mycobot_brain/README.md) to request the world coordinates of a cube with a specific color from [**Vision**](../mycobot_vision/README.md).

```python
# request
string color                # color of the cube to detect ("red", "yellow", "green", "blue")
---
# response
float64[] coords            # x, y, z, rx, ry, rz coordinates of the cube in the robot-frame
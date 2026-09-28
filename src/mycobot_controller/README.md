# mycobot_controller

> **Note:** This node must run on the Raspberry Pi that is physically connected to the robot arm. It will not work on a remote machine without the serial connection and GPIO hardware.

A ROS 2 Python package that acts as the hardware interface node for the **myCobot 280 Pi**. It bridges [**Brain**](../mycobot_brain/README.md) to the physical robot by executing joint trajectories planned by [**MoveIt**](../mycobot_280_moveit2/README.md), publishing live joint states for simulation feedback, and controlling the pump via GPIO.  
This package is designed to run **on the Raspberry Pi** inside the robot arm.  
See the code here: [**controller.py**](../mycobot_controller/mycobot_controller/controller.py)

---

## Interfaces

### Topics
| Topic Name | Message Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/JointState` | Publisher | Publishes the current joint angles read from the robot at 20 Hz. Used by `robot_state_publisher` and [**MoveIt**](../mycobot_280_moveit2/README.md) for simulation feedback. |
| `/pump_controller` | `std_msgs/String` | Subscriber | Receives `"on"` or `"off"` from [**Brain**](../mycobot_brain/README.md) to control the pump via GPIO pin 20. |

### Actions
| Action Name | Action Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/arm_group_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Server | Receives a joint trajectory from [**Brain**](../mycobot_brain/README.md) or [**MoveIt**](../mycobot_280_moveit2/README.md) and executes it waypoint-by-waypoint on the physical robot, timing each step according to the trajectory timestamps. |

---

## Key Functions

| Function | Description |
|----------|-------------|
| `publish_joint_states()` | Timer callback (20 Hz): reads current joint angles from the robot via `mc.get_radians()` and publishes them as a `JointState` message. |
| `control_pump(state_handle)` | Subscription callback: sets GPIO pin 20 to LOW (`"on"`, pump active) or HIGH (`"off"`, pump inactive). |
| `execute_trajectory(goal_handle)` | Action server callback: iterates over the trajectory waypoints, sends each joint position to the robot via `mc.send_radians()`, and sleeps for the inter-waypoint duration to follow the MoveIt timing. |

---

## Hardware Details

| Component | Details |
|-----------|---------|
| Robot connection | Serial port `/dev/serial0` at 1 000 000 baud via `pymycobot.MyCobot280` |
| Pump GPIO pin | BCM pin **20** — LOW = pump ON, HIGH = pump OFF |

---

## Dependencies

| Dependency | Role |
|-----------|------|
| `rclpy` | ROS 2 Python client library |
| `std_msgs`, `sensor_msgs`, `control_msgs` | String, JointState, and FollowJointTrajectory types |
| `pymycobot` (`MyCobot280`) | Official Python SDK for serial communication with the robot |
| `RPi.GPIO` | Raspberry Pi GPIO control for the pump |
| `numpy` | Used to compute max joint velocity for speed scaling |

---

## Building & Running

```bash
# Build (run on the Raspberry Pi)
cd ~/mycobot_280pi/ros2_ws
colcon build --packages-select mycobot_controller
source install/setup.bash

# Run
ros2 run mycobot_controller controller
```
>Note: If the rviz simulation is running, the controller node should be **killed**, so that there is no interference between the two.
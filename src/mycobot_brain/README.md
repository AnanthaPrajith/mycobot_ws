# mycobot_brain

A ROS 2 Python package that acts as the central node for the **myCobot 280 Pi** pick-and-place system. It requests cube coordinates from [**Vision**](../mycobot_vision/README.md), builds collision objects in the [**MoveIt**](../../mycobot_280_moveit2/README.md) planning scene, plans and executes trajectories via [**MoveIt**](../../mycobot_280_moveit2/README.md), and controls the pump via [**Controller**](../mycobot_controller/README.md).  
See the code here: [**brain.py**](../mycobot_brain/mycobot_brain/brain.py)

---

## Interfaces

### Topics
| Topic Name | Message Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/pump_controller` | `std_msgs/String` | Publisher | Sends `"on"` or `"off"` to the controller node to control the pump. |
| `/collision_object` | `moveit_msgs/CollisionObject` | Publisher | Adds or removes cube collision objects in the [**MoveIt**](../../mycobot_280_moveit2/README.md) planning scene. |
| `/attached_collision_object` | `moveit_msgs/AttachedCollisionObject` | Publisher | Attaches or detaches cube collision objects to or from a robot link. |

### Services
| Service Name | Service Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/cube_coordinates` | `mycobot_interfaces/GetCubeCoords` | Client | Requests the robot-frame pose `[x, y, z, roll, pitch, yaw]` of a cube by color from [**Vision**](../mycobot_vision/README.md). |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | Client | Requests a straight-line cartesian path from [**MoveIt**](../../mycobot_280_moveit2/README.md) for precise up and down motions during pick and place. |

### Actions
| Action Name | Action Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `move_action` | `moveit_msgs/MoveGroup` | Client | Sends a full motion planning + execution goal to [**MoveIt**](../../mycobot_280_moveit2/README.md) used for free-space moves. |
| `/arm_group_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Client | Sends a cartesian path directly to [**Controller**](../mycobot_controller/README.md) for execution. |

---

## Key Functions

| Function | Description |
|----------|-------------|
| `get_cube_coords(color)` | Calls the `/cube_coordinates` service and returns the pose of the cube. Returns `False` if the cube is not detected. |
| `spawn_cube(color)` | Queries cube coordinates and publishes a `CollisionObject` to add a 4 cm³ cube to the MoveIt planning scene. Also attaches it to `env_table` to fix inter cube collision errors. |
| `destroy_cube(color)` | Removes a cube collision object from the MoveIt planning scene. |
| `attach_cube(color, link)` | Attaches a cube collision object to a given robot link (e.g. `pump_head` or `env_table`) via `AttachedCollisionObject`. |
| `detach_cube(color, link)` | Detaches a cube collision object from a robot link. |
| `send_pump_state(goal_state)` | Publishes `"on"` or `"off"` to `/pump_controller`. |
| `send_goal_pose(goal_coords)` | Plans and executes a free-space trajectory to a robot-frame goal pose using the MoveIt `move_action` server (1 mm position tolerance, 0.01 rad orientation tolerance). |
| `send_cartesian_path(goal_coords)` | Computes a Cartesian straight-line path via `/compute_cartesian_path` and executes it directly on the controller. Used for vertical pick and place motions. |
| `go_home()` | Moves the arm to the fixed home pose `[0.16, -0.06, 0.32, 0, π/2, 0]`. |
| `pick(color)` | Full pick sequence: hover → lower (cartesian) → pump on → retract (cartesian) → home. Manages collision object attachment. |
| `place(color_top, color_bottom, number)` | Full place sequence: hover over target cube (with stacking height offset) → lower (cartesian) → pump off → retract (cartesian) → home. |
| `drop(color, coords)` | Moves to an absolute drop position and releases the cube (pump off). |
| `stack_cubes()` | Automated sequence: stacks green on red, then blue on green. |
| `choose_pick()` | Interactive CLI: prompts the user to select a cube color and calls `pick()` and `choose_drop()`. |
| `choose_drop(color)` | Interactive CLI: prompts the user to select one of four drop bins (A–D) and calls `drop()`. |
| `control_menu()` | Top-level interactive menu: choose between *Stack Cubes* mode and *Sort Cubes into Bins* mode. |

---

## Pick-and-Place Motion Flow

```
control_menu()
    ├── stack_cubes()
    │       ├── spawn_cube() × 4          # add cubes to planning scene
    │       ├── pick(color)
    │       │     ├── send_goal_pose()    # free-space hover
    │       │     ├── send_cartesian_path()  # straight down
    │       │     ├── send_pump_state("on")
    │       │     └── send_cartesian_path()  # straight up
    │       └── place(color_top, color_bottom)
    │             ├── send_goal_pose()    # free-space hover
    │             ├── send_cartesian_path()  # straight down
    │             ├── send_pump_state("off")
    │             └── send_cartesian_path()  # straight up
    └── choose_pick(color) → pick(color) → choose_drop(color) → drop(color) → back to choose_pick()
```

---

## Dependencies

| Dependency | Role |
|-----------|------|
| `rclpy` | ROS 2 Python client library |
| `moveit_msgs` | MoveIt action/service/message types |
| `control_msgs` | `FollowJointTrajectory` action |
| `geometry_msgs`, `shape_msgs`, `std_msgs` | Pose, primitive shape, and string messages |
| `mycobot_interfaces` | Custom `GetCubeCoords` service |
| `scipy` | Euler→quaternion conversion via `Rotation.from_euler` |

---

## Building & Running

```bash
# Build
cd ~/mycobot_280pi/ros2_ws
colcon build --packages-select mycobot_brain
source install/setup.bash

# Run (requires Vision, Controller, and MoveIt to be running)
ros2 run mycobot_brain brain
```

# mycobot_280_moveit2

This is the moveit2 package of the myCobot 280pi. It features trajectory planning and uses the [**mycobot_irf_scene.urdf**](../mycobot_description/urdf/mycobot_280_pi/mycobot_irf_scene.urdf) to avoid collisions. It also provides a rviz simulation to see and plan trajectories or sequences before executing them in reality.

---

## Key launch files

### `move_group.launch.py`

[**move_group.launch.py**](../mycobot_280_moveit2/launch/move_group.launch.py) launches the trajectory planning interface that is used by [**Brain**](../mycobot_brain/README.md).

```bash
ros2 launch mycobot_280_moveit2 move_group.launch.py
```

### `demo.launch.py`

[**demo.launch.py**](../mycobot_280_moveit2/launch/demo.launch.py) launches the rviz simulation of the myCobot 280pi. The simulation allows us to observe trajectories and sequences before executing them in reality. It also features the simulation of the environement and the colored cubes (excluding gravity).

```bash
ros2 launch mycobot_280_moveit2 demo.launch.py
```
>Note: If the rviz simulation is running, the controller node should be **killed**, so that there is no interference between the two.

---

## Interfaces

### Topics
| Topic Name | Message Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/collision_object` | `moveit_msgs/msg/CollisionObject` | Subscriber | Collision objects such as the colored cubes are added to the planning scene. |
| `/attached_collision_object` | `moveit_msgs/msg/AttachedCollisionObject` | Subscriber | Collision objects are attached to a specific robot link. |

### Services
| Service Name | Service Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | Server | Returns a cartesian path between two points. |

### Actions
| Action Name | Action Type | Role | Description |
| :--- | :--- | :--- | :--- |
| `/move_action` | `moveit_msgs/action/MoveGroup` | Server | A trajectory to a goal pose is computed and forwarded to /arm_group_controller/follow_joint_trajectory. |
| `/arm_group_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Client | Sends a trajectory to execute. |
"""Lab 9 brain for the real robot (task: extension of the object interface), based on brain.py.

Changes against brain.py:
  - objects are requested by colour + shape via /get_object (GetObject) and identified by their unique id
    (planning scene, attach, detach, remove), get_cube_coords(color) -> get_object(color, shape, id)
  - spawn_object(): BOX for cubes, CYLINDER for cylinders, real sizes (35 mm), placed by the object's top surface
  - heights from lab9.yaml: contact = object top + nozzle offset below pump_head, hover above it, drop height per bin
  - bins per colour from lab9.yaml; menu as a loop (no recursive calls), auto sort mode
  - pick goes straight to the bin (no home detour); yaw retries because MoveIt's IK fails at random for some yaws
  - MoveIt velocity scaling from lab9.yaml (robot.velocity_scaling, 1.0 = course value), planning/execution failures stop the sequence (pump off, home)
Config: parameter "config" (default lab9_pick_place/config/lab9.yaml).
Run: ros2 run lab9_pick_place brain_lab9 (Lab PC, last)
"""
import math
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from lab9_interfaces.srv import GetObject
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume,
                             CollisionObject, AttachedCollisionObject, JointConstraint)
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

from ament_index_python.packages import get_package_share_directory

DEFAULT_CONFIG = os.path.join(get_package_share_directory("lab9_pick_place"), "config", "lab9.yaml")   # symlink to src
MAX_PER_COLOR = 6                   # auto sort: at most this many picks per colour
DOWN = [0.0, math.pi, 0.0]          # pump_head pointing down (roll, pitch, yaw); yaw 0: suction needs no yaw
TOUCH = ["pump_head", "pump_box", "env_table"]
JOINTS = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5",
          "joint6output_to_joint6"]     # same order and sign as pymycobot get_angles (controller: get_radians)
PLAN_FAIL = (-1, -2, -31)           # PLANNING_FAILED, INVALID_MOTION_PLAN, NO_IK_SOLUTION: nothing moved, retry is safe
                                    # NOT -6 TIMED_OUT: execution had started and the controller cannot be cancelled
                                    # (10-05: retrying it stacked 5 home trajectories on the real arm)
SCENE_SETTLE_S = 0.5                # move_group applies scene topics asynchronously; planning earlier gave error -2


def down(x, y, z, yaw):
    return [x, y, z, 0.0, math.pi, yaw]


def yaw_candidates(x, y):
    """Suction needs no yaw. MoveIt's IK fails at random for some yaws (simulation test 10-05), so try several."""
    a = math.atan2(y, x)
    return [0.0, a, a + math.pi / 2, math.pi / 2, -math.pi / 2, a - math.pi / 2] * 2     # 2 rounds


def to_pose(coords):
    """[x, y, z, roll, pitch, yaw] -> geometry_msgs/Pose."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(c) for c in coords[:3])
    q = Rotation.from_euler("xyz", coords[3:6]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(c) for c in q)
    return p


class BrainLab9(Node):
    def __init__(self):
        super().__init__("mycobot_brain_lab9")
        path = os.path.expanduser(self.declare_parameter("config", DEFAULT_CONFIG).value)
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.rb, self.ob = cfg["robot"], cfg["objects"]
        self.get_logger().info(f"config: {path}")
        self.scene_ids = set()                       # collision objects currently in the planning scene

        self.pump_publisher = self.create_publisher(String, "/pump_controller", 10)
        self.object_client = self.create_client(GetObject, "/get_object")
        self.cube_publisher = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.attached_cube_publisher = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.trajectory_client = ActionClient(self, MoveGroup, "move_action")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.cartesian_action_client = ActionClient(self, FollowJointTrajectory,
                                                    "/arm_group_controller/follow_joint_trajectory")
        for name, ready in (("/get_object (vision_lab9)", self.object_client.wait_for_service),
                            ("move_action (MoveIt)", self.trajectory_client.wait_for_server),
                            ("/compute_cartesian_path (MoveIt)", self.cartesian_client.wait_for_service),
                            ("follow_joint_trajectory (controller on the Pi)", self.cartesian_action_client.wait_for_server)):
            self.get_logger().info(f"waiting for {name} ...")
            ready()
        self.get_logger().info("all servers ready")

    # ---------------- vision ----------------
    def get_object(self, color="", shape="", obj_id=""):
        """GetObject response (found, id, shape, coords, height, matches)."""
        req = GetObject.Request()
        req.color, req.shape, req.id = color, shape, obj_id
        future = self.object_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res.found:
            self.get_logger().warn(f"no object for color='{color}' shape='{shape}' id='{obj_id}'")
        return res

    # ---------------- planning scene ----------------
    def spawn_object(self, res):
        obj = CollisionObject()
        obj.id = res.id
        obj.header.frame_id = "g_base"
        obj.operation = CollisionObject.ADD
        centre = list(res.coords)
        centre[2] -= res.height / 2                         # coords give the top surface, the primitive is centred
        obj.pose = to_pose(centre)
        prim = SolidPrimitive()
        if res.shape == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [res.height, self.ob["cylinder_d_m"] / 2]      # [height, radius]
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [self.ob["cube_m"]] * 2 + [res.height]
        obj.primitives.append(prim)
        obj.primitive_poses.append(to_pose([0, 0, 0, 0, 0, 0]))
        self.cube_publisher.publish(obj)
        self.scene_ids.add(res.id)
        time.sleep(0.1)                                      # the object must exist before it is attached (2 topics)
        self.attach_object(res.id, "env_table")             # as brain.py: no collision checks between objects

    def destroy_object(self, obj_id):
        obj = CollisionObject()
        obj.id = obj_id
        obj.operation = CollisionObject.REMOVE
        self.cube_publisher.publish(obj)
        self.scene_ids.discard(obj_id)

    def attach_object(self, obj_id, link="pump_head"):
        att = AttachedCollisionObject()
        att.object.id = obj_id
        att.object.operation = CollisionObject.ADD
        att.link_name = link
        att.touch_links = TOUCH + sorted(self.scene_ids)
        self.attached_cube_publisher.publish(att)

    def detach_object(self, obj_id, link="pump_head"):
        att = AttachedCollisionObject()
        att.object.id = obj_id
        att.object.operation = CollisionObject.REMOVE
        att.link_name = link
        att.touch_links = TOUCH + sorted(self.scene_ids)
        self.attached_cube_publisher.publish(att)

    def refresh_scene(self):
        """Remove old objects, add every object vision sees now. Returns the list of (id, response)."""
        for obj_id in list(self.scene_ids):
            self.detach_object(obj_id, "env_table")
            self.destroy_object(obj_id)
        time.sleep(SCENE_SETTLE_S)                           # removals must be applied before the same ids are re-added
        found = []
        for obj_id in self.get_object().matches:
            res = self.get_object(obj_id=obj_id)
            if res.found:
                self.spawn_object(res)
                found.append(res)
        time.sleep(SCENE_SETTLE_S)
        self.get_logger().info(f"planning scene: {[r.id for r in found] or 'no objects'}")
        return found

    # ---------------- motion ----------------
    def send_pump_state(self, state):
        self.pump_publisher.publish(String(data=state))
        time.sleep(0.5)
        self.get_logger().info(f"pump {state}")

    def send_joint_goal(self, angles_deg):
        """MoveIt plan + execute to a joint posture (pymycobot angles in deg). Returns the MoveIt error code."""
        constraints = Constraints()
        for name, a in zip(JOINTS, angles_deg):
            constraints.joint_constraints.append(JointConstraint(joint_name=name, position=math.radians(a),
                                                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        return self.send_constraints(constraints, f"joints {angles_deg}")

    def send_goal_pose(self, coords):
        """MoveIt plan + execute to a pump_head pose. Returns the MoveIt error code (1 = success)."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "g_base"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = to_pose(coords)
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        region = BoundingVolume()
        region.primitive_poses.append(goal_pose.pose)
        region.primitives.append(sphere)
        pc = PositionConstraint(link_name="pump_head", constraint_region=region, weight=1.0)
        pc.header.frame_id = "g_base"
        oc = OrientationConstraint(link_name="pump_head", orientation=goal_pose.pose.orientation, weight=1.0,
                                   absolute_x_axis_tolerance=0.01, absolute_y_axis_tolerance=0.01,
                                   absolute_z_axis_tolerance=0.01)
        oc.header.frame_id = "g_base"
        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        return self.send_constraints(constraints, f"{[round(c, 3) for c in coords]}")

    def send_constraints(self, constraints, label):
        req = MotionPlanRequest(group_name="arm_group", num_planning_attempts=10, allowed_planning_time=5.0,
                                max_velocity_scaling_factor=float(self.rb["velocity_scaling"]),
                                max_acceleration_scaling_factor=float(self.rb["velocity_scaling"]))
        req.goal_constraints.append(constraints)
        goal = MoveGroup.Goal()
        goal.request = req
        self.get_logger().info(f"move to {label}")
        goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        handle = goal_future.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected by MoveIt")
            return 0
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        if code != 1:
            self.get_logger().warn(f"MoveIt error code {code}")
        return code

    def move_down(self, x, y, z):
        """Move pump_head (pointing down) to x, y, z, trying yaw candidates while planning fails.
        Returns the yaw used, or None (no plan, or execution failed)."""
        for yaw in yaw_candidates(x, y):
            code = self.send_goal_pose(down(x, y, z, yaw))
            if code == 1:
                return yaw
            if code not in PLAN_FAIL:
                return None                                  # execution problem: do not retry
        return None

    def compute_cartesian(self, coords, avoid_collisions=True):
        """Straight-line trajectory of pump_head from the current state to coords, or None if not fully feasible.
        avoid_collisions=False for the short vertical descent/lift onto the target (touching it is the goal)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()
        req.avoid_collisions = avoid_collisions
        req.start_state.is_diff = True
        req.group_name = "arm_group"
        req.link_name = "pump_head"
        req.waypoints = [to_pose(coords)]
        req.max_step = 0.01
        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.fraction < 0.99:
            self.get_logger().warn(f"cartesian path only {res.fraction * 100:.0f} % feasible")
            return None
        return res.solution.joint_trajectory

    def execute(self, traj):
        traj.header.stamp = self.get_clock().now().to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal_future = self.cartesian_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code == 0

    def send_cartesian_path(self, coords, avoid_collisions=True):
        traj = self.compute_cartesian(coords, avoid_collisions)
        return traj is not None and self.execute(traj)

    def go_home(self):
        if self.rb.get("home_joints_deg"):                      # fixed posture (joint-space goal)
            for _ in range(3):
                code = self.send_joint_goal(self.rb["home_joints_deg"])
                if code == 1:
                    return True
                if code not in PLAN_FAIL:
                    break
            self.get_logger().error("could not reach home posture")
            return False
        h = self.rb["home"]
        if abs(h[3]) < 1e-3 and abs(h[4] - math.pi) < 1e-3:   # home with the tool down: yaw is free, use the retries
            if self.move_down(h[0], h[1], h[2]) is not None:
                return True
            self.get_logger().error("could not reach home")
            return False
        for _ in range(3):                                   # planner fails at random sometimes
            code = self.send_goal_pose(self.rb["home"])
            if code == 1:
                return True
            if code not in PLAN_FAIL:
                break
        self.get_logger().error("could not reach home")
        return False

    def abort(self, why):
        self.get_logger().error(f"sequence stopped: {why}")
        self.send_pump_state("off")
        self.go_home()
        return False

    def pick(self, res):
        x, y, z_top = res.coords[:3]
        contact = z_top + self.rb["tip_below_pump_head_m"]
        hover = contact + self.rb["hover_m"]
        for yaw in yaw_candidates(x, y):                     # hover, then only go down if the straight line is feasible
            code = self.send_goal_pose(down(x, y, hover, yaw))
            if code not in (1,) + PLAN_FAIL:
                return self.abort(f"hover above {res.id}")
            traj = self.compute_cartesian(down(x, y, contact, yaw), False) if code == 1 else None
            if traj is not None:
                break
        else:
            return self.abort(f"no plan to pick {res.id} at ({x:.3f}, {y:.3f}) (reach limit?)")
        if not self.execute(traj):
            return self.abort(f"descend to {res.id}")
        self.send_pump_state("on")
        self.detach_object(res.id, "env_table")
        self.attach_object(res.id, "pump_head")
        time.sleep(SCENE_SETTLE_S)
        if not self.send_cartesian_path(down(x, y, hover, yaw), False):
            return self.abort(f"lift {res.id}")
        self.get_logger().info(f"{res.id} picked")
        return True

    def drop(self, obj_id, bin_name):
        bx, by = self.rb["bins"][bin_name]
        z = self.rb["drop_tip_z_m"] + self.rb["tip_below_pump_head_m"]
        if self.move_down(bx, by, z) is None:
            return self.abort(f"no plan to bin {bin_name}")
        self.send_pump_state("off")
        self.detach_object(obj_id, "pump_head")
        self.destroy_object(obj_id)
        time.sleep(SCENE_SETTLE_S)
        self.get_logger().info(f"{obj_id} dropped into bin {bin_name}")
        return self.go_home()

    def place_on(self, top_id, bottom):
        """Stack: put the held object on top of 'bottom' (a GetObject response)."""
        x, y, z_top = bottom.coords[:3]
        contact = z_top + self.ob["cube_m"] + self.rb["tip_below_pump_head_m"]   # held object's top after placing
        yaw = self.move_down(x, y, contact + self.rb["hover_m"])
        if yaw is None:
            return self.abort(f"no plan above {bottom.id}")
        if not self.send_cartesian_path(down(x, y, contact + 0.005, yaw), False):
            return self.abort(f"lower onto {bottom.id}")
        self.send_pump_state("off")
        self.detach_object(top_id, "pump_head")
        self.destroy_object(top_id)
        time.sleep(SCENE_SETTLE_S)
        self.send_cartesian_path(down(x, y, contact + self.rb["hover_m"], yaw), False)
        return self.go_home()

    # ---------------- modes ----------------
    def choose_pick(self):
        objs = self.refresh_scene()
        if not objs:
            return
        colors = {"r": "red", "y": "yellow", "g": "green", "b": "blue"}
        shapes = {"c": "cube", "z": "cylinder"}
        color = input("colour: red (r), yellow (y), green (g), blue (b), Enter = any, q = back\n").strip().lower()
        if color == "q":
            return
        shape = input("shape: cube (c), cylinder (z), Enter = any\n").strip().lower()
        res = self.get_object(colors.get(color, color), shapes.get(shape, shape))
        if not res.found:
            return
        if len(res.matches) > 1:
            for i, m in enumerate(res.matches):
                r = self.get_object(obj_id=m)
                print(f"  {i}: {m} at ({r.coords[0] * 1000:.0f}, {r.coords[1] * 1000:.0f}) mm")
            sel = input("select number (Enter = 0)\n").strip()
            res = self.get_object(obj_id=res.matches[int(sel)] if sel.isdigit() and int(sel) < len(res.matches)
                                  else res.matches[0])
            if not res.found:
                return
        if self.pick(res):                                  # straight from the hover pose to the bin (no home detour)
            self.choose_drop(res)

    def choose_drop(self, res):
        own = res.id.split("_")[0]
        names = list(self.rb["bins"])
        b = input(f"bin: {', '.join(names)} (Enter = {own if own in names else names[0]})\n").strip().lower()
        b = b if b in names else (own if own in names else names[0])
        self.drop(res.id, b)

    def auto_sort(self):
        """Pick every object inside the workspace into its colour's bin, in sort_order, until none is left."""
        for color in self.rb["sort_order"]:
            for _ in range(MAX_PER_COLOR):                   # bound: a failed grasp must not loop forever
                self.refresh_scene()
                res = self.get_object(color)
                if not res.found:
                    break
                if not self.pick(res) or not self.drop(res.id, color):
                    return

    def stack(self):
        order = self.rb["stack_order"]
        self.refresh_scene()
        bottom = self.get_object(order[0], "cube")
        if not bottom.found:
            return
        for color in order[1:]:
            top = self.get_object(color, "cube")
            if not top.found:
                continue
            if not self.pick(top) or not self.place_on(top.id, bottom):
                return
            bottom.coords[2] += self.ob["cube_m"]             # the stack grew by one cube
            bottom.id = top.id

    def control_menu(self):
        self.send_pump_state("off")
        self.go_home()
        while True:
            print("\n--- Lab 9 menu ---\n1 - pick one object (colour + shape)\n2 - auto sort: all objects into their bins\n"
                  f"3 - stack cubes {' -> '.join(self.rb['stack_order'])}\nq - quit")
            mode = input("select:\n").strip().lower()
            if mode == "1":
                self.choose_pick()
            elif mode == "2":
                self.auto_sort()
            elif mode == "3":
                self.stack()
            elif mode == "q":
                return
            else:
                self.get_logger().warn("invalid choice")


def main():
    rclpy.init()
    node = BrainLab9()
    try:
        node.control_menu()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_pump_state("off")
        for obj_id in list(node.scene_ids):
            node.destroy_object(obj_id)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
